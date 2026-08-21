// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// trt_backend.cpp — TensorRT C++ inference backend implementation.

#include "trt_backend.hpp"

#include "preprocessing.hpp"
#include "dynalgo_log.hpp"

#include <cassert>
#include <cstring>
#include <fstream>
#include <memory>
#include <numeric>
#include <vector>

// TensorRT includes (guarded for optional build)
#ifdef ENABLE_TENSORRT
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#endif

namespace dynalgo {

#ifdef ENABLE_TENSORRT

// Logger for TensorRT
class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            switch (severity) {
                case Severity::kINTERNAL_ERROR:
                case Severity::kERROR:
                    DYNALGO_LOG_ERROR_S("TensorRT: " << msg);
                    break;
                case Severity::kWARNING:
                    DYNALGO_LOG_WARN_S("TensorRT: " << msg);
                    break;
                case Severity::kINFO:
                    DYNALGO_LOG_INFO_S("TensorRT: " << msg);
                    break;
                case Severity::kVERBOSE:
                    DYNALGO_LOG_DEBUG_S("TensorRT: " << msg);
                    break;
            }
        }
    }
} gTrtLogger;

struct TrtBackend::Impl {
    TrtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime, void(*)(nvinfer1::IRuntime*)> runtime{nullptr, [](auto* p){ if(p) p->destroy(); }};
    std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)> engine{nullptr, [](auto* p){ if(p) p->destroy(); }};
    std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)> context{nullptr, [](auto* p){ if(p) p->destroy(); }};

    // Device buffers
    void* dInput = nullptr;
    void* dOutput = nullptr;
    size_t inputSize = 0;
    size_t outputSize = 0;
    int inputBindingIdx = -1;
    int outputBindingIdx = -1;

    // CUDA stream
    cudaStream_t stream = 0;

    Impl() {
        runtime.reset(nvinfer1::createInferRuntime(logger));
        cudaStreamCreate(&stream);
    }

    ~Impl() {
        if (dInput) cudaFree(dInput);
        if (dOutput) cudaFree(dOutput);
        if (stream) cudaStreamDestroy(stream);
    }

    bool setupBindings() {
        if (!engine) return false;

        int nbBindings = engine->getNbBindings();
        for (int i = 0; i < nbBindings; ++i) {
            nvinfer1::Dims dims = engine->getBindingDimensions(i);
            nvinfer1::DataType dtype = engine->getBindingDataType(i);
            bool isInput = engine->bindingIsInput(i);

            size_t vol = 1;
            for (int j = 0; j < dims.nbDims; ++j) vol *= dims.d[j];
            size_t elemSize = (dtype == nvinfer1::DataType::kFLOAT) ? 4 :
                              (dtype == nvinfer1::DataType::kHALF) ? 2 : 4;

            if (isInput) {
                inputBindingIdx = i;
                inputSize = vol * elemSize;
                cudaMalloc(&dInput, inputSize);
            } else {
                outputBindingIdx = i;
                outputSize = vol * elemSize;
                cudaMalloc(&dOutput, outputSize);
            }
        }
        return inputBindingIdx >= 0 && outputBindingIdx >= 0;
    }

    bool infer(const float* hInput, float* hOutput) {
        if (!context || !dInput || !dOutput) return false;

        // Copy input to device
        cudaMemcpyAsync(dInput, hInput, inputSize, cudaMemcpyHostToDevice, stream);

        // Execute
        void* bindings[] = {dInput, dOutput};
        bool status = context->enqueueV2(bindings, stream, nullptr);

        // Copy output to host
        cudaMemcpyAsync(hOutput, dOutput, outputSize, cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        return status;
    }
};

TrtBackend::TrtBackend() : pimpl_(std::make_unique<Impl>()) {}
TrtBackend::~TrtBackend() = default;
TrtBackend::TrtBackend(TrtBackend&&) noexcept = default;
TrtBackend& TrtBackend::operator=(TrtBackend&&) noexcept = default;

bool TrtBackend::load(const DynalgoModelConfig& cfg) {
    if (cfg.modelPath.empty()) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: empty modelPath");
        return false;
    }

    // Load engine file
    std::ifstream file(cfg.modelPath, std::ios::binary);
    if (!file) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: cannot open " << cfg.modelPath);
        return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engineData(size);
    file.read(engineData.data(), size);
    file.close();

    pimpl_->engine.reset(pimpl_->runtime->deserializeCudaEngine(engineData.data(), size, nullptr));
    if (!pimpl_->engine) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: failed to deserialize engine");
        return false;
    }

    pimpl_->context.reset(pimpl_->engine->createExecutionContext());
    if (!pimpl_->context) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: failed to create execution context");
        return false;
    }

    if (!pimpl_->setupBindings()) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: failed to setup bindings");
        return false;
    }

    // Parse input dimensions
    nvinfer1::Dims inputDims = pimpl_->engine->getBindingDimensions(pimpl_->inputBindingIdx);
    if (inputDims.nbDims == 4) { // NCHW
        inputChannels_ = inputDims.d[1];
        inputHeight_ = inputDims.d[2];
        inputWidth_ = inputDims.d[3];
    } else if (inputDims.nbDims == 3) { // CHW
        inputChannels_ = inputDims.d[0];
        inputHeight_ = inputDims.d[1];
        inputWidth_ = inputDims.d[2];
    }

    // Parse output dimensions for numClasses
    nvinfer1::Dims outputDims = pimpl_->engine->getBindingDimensions(pimpl_->outputBindingIdx);
    // Assuming output format: [batch, numDets, 5+numClasses] or [batch, 5+numClasses, numDets]
    if (outputDims.nbDims >= 2) {
        // Heuristic: last dim is 5+numClasses
        numClasses_ = outputDims.d[outputDims.nbDims - 1] - 5;
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
    DYNALGO_LOG_INFO_S("TrtBackend loaded: " << cfg.modelPath
                       << " input=" << inputWidth_ << "x" << inputHeight_
                       << " classes=" << numClasses_);
    return true;
}

bool TrtBackend::infer(const DynalgoFrame& frame,
                       std::vector<DynalgoDetectionResult>& out) {
    if (!isLoaded_) {
        DYNALGO_LOG_WARN_S("TrtBackend::infer: not loaded");
        return false;
    }

    // Preprocess
    std::optional<LetterboxResult> lb;
    std::vector<float> inputTensor = preprocessFrame(frame, preprocessCfg_, &lb);
    if (inputTensor.empty() || !lb) {
        DYNALGO_LOG_WARN_S("TrtBackend::infer: preprocessing failed");
        return false;
    }

    // Allocate output buffer
    std::vector<float> outputTensor(pimpl_->outputSize / sizeof(float));

    // Run inference
    if (!pimpl_->infer(inputTensor.data(), outputTensor.data())) {
        DYNALGO_LOG_ERROR_S("TrtBackend::infer: inference failed");
        return false;
    }

    // Postprocess
    NMSConfig nmsCfg;
    nmsCfg.iouThreshold = preprocessCfg_.normalize ? 0.45f : 0.45f;
    nmsCfg.confThreshold = 0.25f;
    nmsCfg.maxDetections = 1000;
    nmsCfg.classAgnostic = false;

    // Estimate numDetections from output size
    int numDetections = static_cast<int>(outputTensor.size() / (5 + numClasses_));

    out = postprocessDetections(outputTensor.data(), numDetections, numClasses_,
                                *lb, nmsCfg, frame.width, frame.height, true);

    return true;
}

bool TrtBackend::buildEngineFromOnnx(const std::string& onnxPath,
                                     const std::string& enginePath,
                                     bool fp16, int maxBatchSize, size_t workspaceSize) {
    DYNALGO_LOG_INFO_S("TrtBackend::buildEngineFromOnnx: " << onnxPath << " -> " << enginePath);

    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(gTrtLogger);
    if (!builder) return false;

    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
    if (!network) { builder->destroy(); return false; }

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gTrtLogger);
    if (!parser) { network->destroy(); builder->destroy(); return false; }

    if (!parser->parseFromFile(onnxPath.c_str(),
                                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        DYNALGO_LOG_ERROR_S("Failed to parse ONNX: " << onnxPath);
        parser->destroy(); network->destroy(); builder->destroy();
        return false;
    }

    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    config->setMaxWorkspaceSize(workspaceSize);
    if (fp16 && builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    // Optimization profile for dynamic batch
    nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
    // Assuming first input is the image tensor
    nvinfer1::Dims inputDims = network->getInput(0)->getDimensions();
    nvinfer1::Dims minDims = inputDims, optDims = inputDims, maxDims = inputDims;
    minDims.d[0] = 1;
    optDims.d[0] = maxBatchSize;
    maxDims.d[0] = maxBatchSize;
    profile->setDimensions(network->getInput(0)->getName(),
                           nvinfer1::OptProfileSelector::kMIN, minDims);
    profile->setDimensions(network->getInput(0)->getName(),
                           nvinfer1::OptProfileSelector::kOPT, optDims);
    profile->setDimensions(network->getInput(0)->getName(),
                           nvinfer1::OptProfileSelector::kMAX, maxDims);
    config->addOptimizationProfile(profile);

    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    if (!engine) {
        DYNALGO_LOG_ERROR_S("Failed to build engine");
        profile->destroy(); config->destroy(); parser->destroy(); network->destroy(); builder->destroy();
        return false;
    }

    // Serialize and save
    nvinfer1::IHostMemory* serialized = engine->serialize();
    std::ofstream out(enginePath, std::ios::binary);
    out.write(static_cast<const char*>(serialized->data()), serialized->size());
    out.close();

    serialized->destroy();
    engine->destroy();
    profile->destroy();
    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();

    DYNALGO_LOG_INFO_S("Engine built and saved to: " << enginePath);
    return true;
}

std::unique_ptr<DynalgoModelBackend> createTrtBackend() {
    return std::make_unique<TrtBackend>();
}

#else // ENABLE_TENSORRT not defined

TrtBackend::TrtBackend() {}
TrtBackend::~TrtBackend() = default;
TrtBackend::TrtBackend(TrtBackend&&) noexcept = default;
TrtBackend& TrtBackend::operator=(TrtBackend&&) noexcept = default;

bool TrtBackend::load(const DynalgoModelConfig&) {
    DYNALGO_LOG_ERROR_S("TrtBackend: TensorRT not enabled (ENABLE_TENSORRT=OFF)");
    return false;
}
bool TrtBackend::infer(const DynalgoFrame&, std::vector<DynalgoDetectionResult>&) {
    return false;
}
bool TrtBackend::buildEngineFromOnnx(const std::string&, const std::string&, bool, int, size_t) {
    return false;
}
std::unique_ptr<DynalgoModelBackend> createTrtBackend() {
    return nullptr;
}

#endif // ENABLE_TENSORRT

} // namespace dynalgo