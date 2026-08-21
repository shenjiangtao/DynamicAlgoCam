// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// rknn_backend.cpp — RKNN Runtime C++ inference backend implementation.

#include "rknn_backend.hpp"

#include "preprocessing.hpp"
#include "dynalgo_log.hpp"

#include <cstdint>
#include <memory>
#include <vector>
#include <algorithm>

#ifdef ENABLE_RKNN
#include <rknn_api.h>
#include <dlfcn.h>
#endif

namespace dynalgo {

#ifdef ENABLE_RKNN

// RKNN API function pointers (loaded dynamically)
struct RknnApi {
    using rknn_init_func = int(*)(rknn_context*, const void*, size_t, int, const char*);
    using rknn_destroy_func = int(*)(rknn_context);
    using rknn_query_func = int(*)(rknn_context, rknn_query_cmd, void*, uint32_t);
    using rknn_inputs_set_func = int(*)(rknn_context, uint32_t, rknn_input*);
    using rknn_run_func = int(*)(rknn_context, void*);
    using rknn_outputs_get_func = int(*)(rknn_context, uint32_t, rknn_output*, void*);
    using rknn_outputs_release_func = int(*)(rknn_context, uint32_t, rknn_output*);

    rknn_init_func init = nullptr;
    rknn_destroy_func destroy = nullptr;
    rknn_query_func query = nullptr;
    rknn_inputs_set_func inputs_set = nullptr;
    rknn_run_func run = nullptr;
    rknn_outputs_get_func outputs_get = nullptr;
    rknn_outputs_release_func outputs_release = nullptr;

    void* handle = nullptr;

    bool load() {
        handle = dlopen("librknnrt.so", RTLD_LAZY);
        if (!handle) return false;

        init = reinterpret_cast<rknn_init_func>(dlsym(handle, "rknn_init"));
        destroy = reinterpret_cast<rknn_destroy_func>(dlsym(handle, "rknn_destroy"));
        query = reinterpret_cast<rknn_query_func>(dlsym(handle, "rknn_query"));
        inputs_set = reinterpret_cast<rknn_inputs_set_func>(dlsym(handle, "rknn_inputs_set"));
        run = reinterpret_cast<rknn_run_func>(dlsym(handle, "rknn_run"));
        outputs_get = reinterpret_cast<rknn_outputs_get_func>(dlsym(handle, "rknn_outputs_get"));
        outputs_release = reinterpret_cast<rknn_outputs_release_func>(dlsym(handle, "rknn_outputs_release"));

        return init && destroy && query && inputs_set && run && outputs_get && outputs_release;
    }

    ~RknnApi() { if (handle) dlclose(handle); }
};

static RknnApi gRknnApi;

struct RknnBackend::Impl {
    rknn_context ctx = 0;
    std::vector<rknn_tensor_attr> inputAttrs;
    std::vector<rknn_tensor_attr> outputAttrs;
    std::vector<rknn_input> inputs;
    std::vector<rknn_output> outputs;
    std::vector<uint8_t> inputBuffer;
    std::vector<uint8_t> outputBuffers[8]; // max 8 outputs

    Impl() {
        if (!gRknnApi.handle) gRknnApi.load();
    }

    ~Impl() {
        if (ctx && gRknnApi.destroy) gRknnApi.destroy(ctx);
    }

    bool setupIO() {
        if (!ctx || !gRknnApi.query) return false;

        // Query input/output tensors
        rknn_input_output_num ioNum;
        if (gRknnApi.query(ctx, RKNN_QUERY_IN_OUT_NUM, &ioNum, sizeof(ioNum)) < 0) return false;

        inputAttrs.resize(ioNum.n_input);
        for (uint32_t i = 0; i < ioNum.n_input; ++i) {
            inputAttrs[i].index = i;
            if (gRknnApi.query(ctx, RKNN_QUERY_INPUT_ATTR, &inputAttrs[i], sizeof(rknn_tensor_attr)) < 0) return false;
        }

        outputAttrs.resize(ioNum.n_output);
        for (uint32_t i = 0; i < ioNum.n_output; ++i) {
            outputAttrs[i].index = i;
            if (gRknnApi.query(ctx, RKNN_QUERY_OUTPUT_ATTR, &outputAttrs[i], sizeof(rknn_tensor_attr)) < 0) return false;
        }

        // Prepare input structures
        inputs.resize(ioNum.n_input);
        for (uint32_t i = 0; i < ioNum.n_input; ++i) {
            inputs[i].index = i;
            inputs[i].type = RKNN_TENSOR_FLOAT32; // we'll convert
            inputs[i].fmt = RKNN_TENSOR_NCHW;
            inputs[i].pass_through = false;
        }

        outputs.resize(ioNum.n_output);
        for (uint32_t i = 0; i < ioNum.n_output; ++i) {
            outputs[i].index = i;
            outputs[i].want_float = 1; // get float output
        }

        return true;
    }
};

RknnBackend::RknnBackend() : pimpl_(std::make_unique<Impl>()) {}
RknnBackend::~RknnBackend() = default;
RknnBackend::RknnBackend(RknnBackend&&) noexcept = default;
RknnBackend& RknnBackend::operator=(RknnBackend&&) noexcept = default;

void RknnBackend::setCoreMask(int coreMask) {
    coreMask_ = coreMask;
}

bool RknnBackend::load(const DynalgoModelConfig& cfg) {
    if (cfg.modelPath.empty()) {
        DYNALGO_LOG_ERROR_S("RknnBackend::load: empty modelPath");
        return false;
    }

    if (!gRknnApi.handle) {
        DYNALGO_LOG_ERROR_S("RknnBackend: RKNN API not available");
        return false;
    }

    // Read model file
    FILE* fp = fopen(cfg.modelPath.c_str(), "rb");
    if (!fp) {
        DYNALGO_LOG_ERROR_S("RknnBackend::load: cannot open " << cfg.modelPath);
        return false;
    }
    fseek(fp, 0, SEEK_END);
    long modelSize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    std::vector<char> modelData(modelSize);
    fread(modelData.data(), 1, modelSize, fp);
    fclose(fp);

    // Initialize RKNN context
    int ret = gRknnApi.init(&pimpl_->ctx, modelData.data(), modelSize,
                           RKNN_FLAG_PRIOR_MEDIUM, nullptr);
    if (ret < 0 || !pimpl_->ctx) {
        DYNALGO_LOG_ERROR_S("RknnBackend::load: rknn_init failed: " << ret);
        return false;
    }

    if (!pimpl_->setupIO()) {
        DYNALGO_LOG_ERROR_S("RknnBackend::load: setupIO failed");
        return false;
    }

    // Parse input shape
    if (!pimpl_->inputAttrs.empty()) {
        const auto& attr = pimpl_->inputAttrs[0];
        if (attr.n_dims == 4) { // NCHW
            inputChannels_ = attr.dims[1];
            inputHeight_ = attr.dims[2];
            inputWidth_ = attr.dims[3];
        } else if (attr.n_dims == 3) { // CHW
            inputChannels_ = attr.dims[0];
            inputHeight_ = attr.dims[1];
            inputWidth_ = attr.dims[2];
        }
    }

    // Parse output for numClasses
    if (!pimpl_->outputAttrs.empty()) {
        const auto& attr = pimpl_->outputAttrs[0];
        if (attr.n_dims >= 2) {
            numClasses_ = attr.dims[attr.n_dims - 1] - 5;
            if (numClasses_ < 1) numClasses_ = 80;
        }
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
    DYNALGO_LOG_INFO_S("RknnBackend loaded: " << cfg.modelPath
                       << " input=" << inputWidth_ << "x" << inputHeight_
                       << " classes=" << numClasses_);
    return true;
}

bool RknnBackend::infer(const DynalgoFrame& frame,
                        std::vector<DynalgoDetectionResult>& out) {
    if (!isLoaded_ || !pimpl_->ctx || !gRknnApi.run) {
        DYNALGO_LOG_WARN_S("RknnBackend::infer: not loaded");
        return false;
    }

    // Preprocess
    std::optional<LetterboxResult> lb;
    std::vector<float> inputTensor = preprocessFrame(frame, preprocessCfg_, &lb);
    if (inputTensor.empty() || !lb) {
        DYNALGO_LOG_WARN_S("RknnBackend::infer: preprocessing failed");
        return false;
    }

    // Set input
    pimpl_->inputs[0].buf = inputTensor.data();
    pimpl_->inputs[0].size = inputTensor.size() * sizeof(float);

    if (gRknnApi.inputs_set(pimpl_->ctx, pimpl_->inputs.size(), pimpl_->inputs.data()) < 0) {
        DYNALGO_LOG_ERROR_S("RknnBackend::infer: inputs_set failed");
        return false;
    }

    // Run inference
    if (gRknnApi.run(pimpl_->ctx, nullptr) < 0) {
        DYNALGO_LOG_ERROR_S("RknnBackend::infer: run failed");
        return false;
    }

    // Get outputs
    if (gRknnApi.outputs_get(pimpl_->ctx, pimpl_->outputs.size(),
                             pimpl_->outputs.data(), nullptr) < 0) {
        DYNALGO_LOG_ERROR_S("RknnBackend::infer: outputs_get failed");
        return false;
    }

    // Process output (assuming first output is detections)
    float* outputData = static_cast<float*>(pimpl_->outputs[0].buf);
    size_t outputSize = pimpl_->outputs[0].size / sizeof(float);

    int numDetections = static_cast<int>(outputSize / (5 + numClasses_));

    NMSConfig nmsCfg;
    nmsCfg.iouThreshold = 0.45f;
    nmsCfg.confThreshold = 0.25f;
    nmsCfg.maxDetections = 1000;
    nmsCfg.classAgnostic = false;

    out = postprocessDetections(outputData, numDetections, numClasses_,
                                *lb, nmsCfg, frame.width, frame.height, true);

    // Release outputs
    gRknnApi.outputs_release(pimpl_->ctx, pimpl_->outputs.size(), pimpl_->outputs.data());

    return true;
}

std::unique_ptr<DynalgoModelBackend> createRknnBackend() {
    return std::make_unique<RknnBackend>();
}

#else // ENABLE_RKNN not defined

RknnBackend::RknnBackend() {}
RknnBackend::~RknnBackend() = default;
RknnBackend::RknnBackend(RknnBackend&&) noexcept = default;
RknnBackend& RknnBackend::operator=(RknnBackend&&) noexcept = default;

void RknnBackend::setCoreMask(int) {}

bool RknnBackend::load(const DynalgoModelConfig&) {
    DYNALGO_LOG_ERROR_S("RknnBackend: RKNN not enabled (ENABLE_RKNN=OFF)");
    return false;
}
bool RknnBackend::infer(const DynalgoFrame&, std::vector<DynalgoDetectionResult>&) {
    return false;
}
std::unique_ptr<DynalgoModelBackend> createRknnBackend() {
    return nullptr;
}

#endif // ENABLE_RKNN

} // namespace dynalgo