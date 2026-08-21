// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// ort_backend.cpp — ONNX Runtime C++ inference backend implementation.

#include "ort_backend.hpp"

#include "preprocessing.hpp"
#include "dynalgo_log.hpp"

#include <cassert>
#include <memory>
#include <vector>
#include <algorithm>

#ifdef ENABLE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace dynalgo {

#ifdef ENABLE_ONNXRUNTIME

struct OrtBackend::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "DynalgoOrt"};
    std::unique_ptr<Ort::Session> session;
    std::unique_ptr<Ort::SessionOptions> sessionOptions;
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;
    std::vector<int64_t> inputShape;  // NCHW
    std::vector<int64_t> outputShape;

    Impl() {
        sessionOptions = std::make_unique<Ort::SessionOptions>();
        sessionOptions->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    }

    bool setupIO() {
        if (!session) return false;

        // Get input info
        size_t numInputs = session->GetInputCount();
        inputNames.reserve(numInputs);
        for (size_t i = 0; i < numInputs; ++i) {
            inputNames.push_back(session->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions()).get());
            Ort::TypeInfo typeInfo = session->GetInputTypeInfo(i);
            auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
            inputShape = tensorInfo.GetShape();
        }

        // Get output info
        size_t numOutputs = session->GetOutputCount();
        outputNames.reserve(numOutputs);
        for (size_t i = 0; i < numOutputs; ++i) {
            outputNames.push_back(session->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions()).get());
            Ort::TypeInfo typeInfo = session->GetOutputTypeInfo(i);
            auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
            outputShape = tensorInfo.GetShape();
        }

        return !inputNames.empty() && !outputNames.empty();
    }
};

OrtBackend::OrtBackend() : pimpl_(std::make_unique<Impl>()) {}
OrtBackend::~OrtBackend() = default;
OrtBackend::OrtBackend(OrtBackend&&) noexcept = default;
OrtBackend& OrtBackend::operator=(OrtBackend&&) noexcept = default;

void OrtBackend::setProviders(const std::vector<std::string>& providers) {
    providers_ = providers;
}

void OrtBackend::setIntraOpThreads(int n) {
    intraOpThreads_ = n;
    if (pimpl_->sessionOptions) {
        pimpl_->sessionOptions->SetIntraOpNumThreads(n);
    }
}

bool OrtBackend::load(const DynalgoModelConfig& cfg) {
    if (cfg.modelPath.empty()) {
        DYNALGO_LOG_ERROR_S("OrtBackend::load: empty modelPath");
        return false;
    }

    // Configure providers
    if (pimpl_->sessionOptions) {
        pimpl_->sessionOptions->SetIntraOpNumThreads(intraOpThreads_ > 0 ? intraOpThreads_ : 0);
    }

    // Create session with providers
    for (const auto& provider : providers_) {
        if (provider == "CUDAExecutionProvider") {
            OrtCUDAProviderOptions cudaOptions;
            cudaOptions.device_id = 0;
            pimpl_->sessionOptions->AppendExecutionProvider_CUDA(cudaOptions);
        } else if (provider == "TensorrtExecutionProvider") {
            OrtTensorRTProviderOptions trtOptions;
            trtOptions.device_id = 0;
            trtOptions.trt_fp16_enable = 1;
            pimpl_->sessionOptions->AppendExecutionProvider_TensorRT(trtOptions);
        }
    }

    try {
        pimpl_->session = std::make_unique<Ort::Session>(
            pimpl_->env, cfg.modelPath.c_str(), *pimpl_->sessionOptions);
    } catch (const std::exception& e) {
        DYNALGO_LOG_ERROR_S("OrtBackend::load: failed to create session: " << e.what());
        return false;
    }

    if (!pimpl_->setupIO()) {
        DYNALGO_LOG_ERROR_S("OrtBackend::load: failed to setup I/O");
        return false;
    }

    // Parse input shape (assume NCHW)
    if (pimpl_->inputShape.size() == 4) {
        inputChannels_ = static_cast<int>(pimpl_->inputShape[1]);
        inputHeight_ = static_cast<int>(pimpl_->inputShape[2]);
        inputWidth_ = static_cast<int>(pimpl_->inputShape[3]);
    } else if (pimpl_->inputShape.size() == 3) {
        inputChannels_ = static_cast<int>(pimpl_->inputShape[0]);
        inputHeight_ = static_cast<int>(pimpl_->inputShape[1]);
        inputWidth_ = static_cast<int>(pimpl_->inputShape[2]);
    }

    // Parse output shape for numClasses
    // Typical YOLO output: [batch, numDets, 5+numClasses] or [batch, 5+numClasses, numDets]
    if (pimpl_->outputShape.size() >= 2) {
        numClasses_ = static_cast<int>(pimpl_->outputShape.back()) - 5;
        if (numClasses_ < 1) numClasses_ = 80;
    }

    // Preprocessing config
    preprocessCfg_.inputWidth = inputWidth_;
    preprocessCfg_.inputHeight = inputHeight_;
    preprocessCfg_.letterbox = true;
    preprocessCfg_.bgrToRgb = true;
    preprocessCfg_.normalize = true;
    preprocessCfg_.mean[0] = 0.0f; preprocessCfg_.mean[1] = 0.0f; preprocessCfg_.mean[2] = 0.0f;
    preprocessCfg_.std[0] = 1.0f; preprocessCfg_.std[1] = 1.0f; preprocessCfg_.std[2] = 1.0f;
    preprocessCfg_.hwcToChw = true;

    isLoaded_ = true;
    DYNALGO_LOG_INFO_S("OrtBackend loaded: " << cfg.modelPath
                       << " input=" << inputWidth_ << "x" << inputHeight_
                       << " classes=" << numClasses_
                       << " providers=" << providers_.size());
    return true;
}

bool OrtBackend::infer(const DynalgoFrame& frame,
                       std::vector<DynalgoDetectionResult>& out) {
    if (!isLoaded_) {
        DYNALGO_LOG_WARN_S("OrtBackend::infer: not loaded");
        return false;
    }

    // Preprocess
    std::optional<LetterboxResult> lb;
    std::vector<float> inputTensor = preprocessFrame(frame, preprocessCfg_, &lb);
    if (inputTensor.empty() || !lb) {
        DYNALGO_LOG_WARN_S("OrtBackend::infer: preprocessing failed");
        return false;
    }

    // Create input tensor
    std::vector<int64_t> inputDims = {1, inputChannels_, inputHeight_, inputWidth_};
    Ort::Value inputTensorOrt = Ort::Value::CreateTensor<float>(
        pimpl_->memoryInfo, inputTensor.data(), inputTensor.size(),
        inputDims.data(), inputDims.size());

    // Run inference
    std::vector<Ort::Value> outputTensors;
    try {
        outputTensors = pimpl_->session->Run(
            Ort::RunOptions{nullptr},
            pimpl_->inputNames.data(), &inputTensorOrt, 1,
            pimpl_->outputNames.data(), pimpl_->outputNames.size());
    } catch (const std::exception& e) {
        DYNALGO_LOG_ERROR_S("OrtBackend::infer: inference failed: " << e.what());
        return false;
    }

    // Get output data
    float* outputData = outputTensors[0].GetTensorMutableData<float>();
    size_t outputSize = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

    // Estimate numDetections
    int numDetections = 0;
    if (pimpl_->outputShape.size() == 3) {
        // [batch, numDets, 5+numClasses]
        numDetections = static_cast<int>(pimpl_->outputShape[1]);
    } else if (pimpl_->outputShape.size() >= 2) {
        numDetections = static_cast<int>(outputSize / (5 + numClasses_));
    }

    // Postprocess
    NMSConfig nmsCfg;
    nmsCfg.iouThreshold = 0.45f;
    nmsCfg.confThreshold = 0.25f;
    nmsCfg.maxDetections = 1000;
    nmsCfg.classAgnostic = false;

    out = postprocessDetections(outputData, numDetections, numClasses_,
                                *lb, nmsCfg, frame.width, frame.height, true);

    return true;
}

std::unique_ptr<DynalgoModelBackend> createOrtBackend() {
    return std::make_unique<OrtBackend>();
}

#else // ENABLE_ONNXRUNTIME not defined

OrtBackend::OrtBackend() {}
OrtBackend::~OrtBackend() = default;
OrtBackend::OrtBackend(OrtBackend&&) noexcept = default;
OrtBackend& OrtBackend::operator=(OrtBackend&&) noexcept = default;

void OrtBackend::setProviders(const std::vector<std::string>&) {}
void OrtBackend::setIntraOpThreads(int) {}

bool OrtBackend::load(const DynalgoModelConfig&) {
    DYNALGO_LOG_ERROR_S("OrtBackend: ONNX Runtime not enabled (ENABLE_ONNXRUNTIME=OFF)");
    return false;
}
bool OrtBackend::infer(const DynalgoFrame&, std::vector<DynalgoDetectionResult>&) {
    return false;
}
std::unique_ptr<DynalgoModelBackend> createOrtBackend() {
    return nullptr;
}

#endif // ENABLE_ONNXRUNTIME

} // namespace dynalgo