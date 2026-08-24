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

// INT8 Calibrator implementation
class TrtInt8Calibrator : public nvinfer1::IInt8Calibrator {
public:
    TrtInt8Calibrator(int batchSize, const std::string& cacheFile,
                      std::function<bool(void* bindings[], const char* names[], int nbBindings)> getBatchFunc,
                      std::function<const void*(size_t&)> readCacheFunc,
                      std::function<void(const void*, size_t)> writeCacheFunc)
        : batchSize_(batchSize), cacheFile_(cacheFile),
          getBatchFunc_(std::move(getBatchFunc)),
          readCacheFunc_(std::move(readCacheFunc)),
          writeCacheFunc_(std::move(writeCacheFunc)) {}

    int getBatchSize() const noexcept override { return batchSize_; }

    bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override {
        if (getBatchFunc_) {
            return getBatchFunc_(bindings, names, nbBindings);
        }
        return false;
    }

    const void* readCalibrationCache(size_t& length) noexcept override {
        if (readCacheFunc_) {
            return readCacheFunc_(length);
        }
        length = 0;
        return nullptr;
    }

    void writeCalibrationCache(const void* cache, size_t length) noexcept override {
        if (writeCacheFunc_) {
            writeCacheFunc_(cache, length);
        }
    }

private:
    int batchSize_;
    std::string cacheFile_;
    std::function<bool(void* bindings[], const char* names[], int nbBindings)> getBatchFunc_;
    std::function<const void*(size_t&)> readCacheFunc_;
    std::function<void(const void*, size_t)> writeCacheFunc_;
};

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
    bool ownsStream = true;

    Impl() {
        runtime.reset(nvinfer1::createInferRuntime(gTrtLogger));
    }

    ~Impl() {
        if (dInput) cudaFree(dInput);
        if (dOutput) cudaFree(dOutput);
        if (ownsStream && stream) cudaStreamDestroy(stream);
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

    bool infer(const float* hInput, float* hOutput, cudaStream_t stream) {
        if (!context || !dInput || !dOutput) return false;

        cudaStream_t useStream = stream ? stream : this->stream;

        // Copy input to device
        cudaMemcpyAsync(dInput, hInput, inputSize, cudaMemcpyHostToDevice, useStream);

        // Execute
        void* bindings[] = {dInput, dOutput};
        bool status = context->enqueueV2(bindings, useStream, nullptr);

        // Copy output to host
        cudaMemcpyAsync(hOutput, dOutput, outputSize, cudaMemcpyDeviceToHost, useStream);
        cudaStreamSynchronize(useStream);

        return status;
    }

    // Set external CUDA stream
    void setExternalStream(cudaStream_t stream) {
        if (ownsStream && this->stream) {
            cudaStreamDestroy(this->stream);
            ownsStream = false;
        }
        this->stream = stream;
    }
};

TrtBackend::TrtBackend() : pimpl_(std::make_unique<Impl>()), config_{} {
    // Default preprocessing config
    preprocessCfg_.inputWidth = 640;
    preprocessCfg_.inputHeight = 640;
    preprocessCfg_.letterbox = true;
    preprocessCfg_.bgrToRgb = true;
    preprocessCfg_.normalize = true;
    preprocessCfg_.mean[0] = 0.0f; preprocessCfg_.mean[1] = 0.0f; preprocessCfg_.mean[2] = 0.0f;
    preprocessCfg_.std[0] = 1.0f; preprocessCfg_.std[1] = 1.0f; preprocessCfg_.std[2] = 1.0f;
    preprocessCfg_.hwcToChw = true;
}
TrtBackend::~TrtBackend() = default;
TrtBackend::TrtBackend(TrtBackend&&) noexcept = default;
TrtBackend& TrtBackend::operator=(TrtBackend&&) noexcept = default;

void TrtBackend::setConfig(const TrtBackendConfig& config) {
    config_ = config;
}

const TrtBackendConfig& TrtBackend::getConfig() const {
    return config_;
}

void TrtBackend::setCUDAStream(cudaStream_t stream) {
    config_.externalStream = stream;
    if (pimpl_) {
        pimpl_->setExternalStream(stream);
    }
}

bool TrtBackend::load(const DynalgoModelConfig& cfg) {
    // Use config_ if modelPath is set there, otherwise fall back to cfg.modelPath
    std::string modelPath = !config_.modelPath.empty() ? config_.modelPath : cfg.modelPath;
    
    if (modelPath.empty()) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: empty modelPath");
        return false;
    }

    // Set CUDA device
    if (config_.deviceId >= 0) {
        cudaSetDevice(config_.deviceId);
    }

    // Load engine file
    std::ifstream file(modelPath, std::ios::binary);
    if (!file) {
        DYNALGO_LOG_ERROR_S("TrtBackend::load: cannot open " << modelPath);
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

    // Set external stream if provided
    if (config_.externalStream) {
        pimpl_->setExternalStream(config_.externalStream);
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
    if (outputDims.nbDims >= 2) {
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
    DYNALGO_LOG_INFO_S("TrtBackend loaded: " << modelPath
                       << " input=" << inputWidth_ << "x" << inputHeight_
                       << " classes=" << numClasses_
                       << " fp16=" << config_.fp16
                       << " int8=" << config_.int8);
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
    cudaStream_t stream = config_.externalStream ? config_.externalStream : pimpl_->stream;
    if (!pimpl_->infer(inputTensor.data(), outputTensor.data(), stream)) {
        DYNALGO_LOG_ERROR_S("TrtBackend::infer: inference failed");
        return false;
    }

    // Postprocess
    NMSConfig nmsCfg;
    nmsCfg.iouThreshold = 0.45f;
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
    TrtBackendConfig config;
    config.modelPath = onnxPath;
    config.fp16 = fp16;
    config.int8 = false;
    config.maxBatchSize = maxBatchSize;
    config.workspaceSize = workspaceSize;
    return buildEngineFromOnnx(config);
}

bool TrtBackend::buildEngineFromOnnx(const TrtBackendConfig& config) {
    DYNALGO_LOG_INFO_S("TrtBackend::buildEngineFromOnnx: " << config.modelPath << " -> " << config.modelPath.replace(config.modelPath.find_last_of('.'), std::string::npos, ".engine"));

    std::string enginePath = config.modelPath;
    size_t dotPos = enginePath.rfind('.');
    if (dotPos != std::string::npos) {
        enginePath = enginePath.substr(0, dotPos) + ".engine";
    }

    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(gTrtLogger);
    if (!builder) return false;

    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
    if (!network) { builder->destroy(); return false; }

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gTrtLogger);
    if (!parser) { network->destroy(); builder->destroy(); return false; }

    if (!parser->parseFromFile(config.modelPath.c_str(),
                                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        DYNALGO_LOG_ERROR_S("Failed to parse ONNX: " << config.modelPath);
        parser->destroy(); network->destroy(); builder->destroy();
        return false;
    }

    nvinfer1::IBuilderConfig* builderConfig = builder->createBuilderConfig();
    builderConfig->setMaxWorkspaceSize(config.workspaceSize);
    
    if (config.fp16 && builder->platformHasFastFp16()) {
        builderConfig->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    if (config.int8 && builder->platformHasFastInt8()) {
        builderConfig->setFlag(nvinfer1::BuilderFlag::kINT8);
        // Note: INT8 requires calibrator - use buildEngineWithCalibration instead
    }

    // Timing cache
    if (!config.timingCache.empty()) {
        std::ifstream cacheFile(config.timingCache, std::ios::binary);
        if (cacheFile) {
            cacheFile.seekg(0, std::ios::end);
            size_t cacheSize = cacheFile.tellg();
            cacheFile.seekg(0, std::ios::beg);
            std::vector<char> cacheData(cacheSize);
            cacheFile.read(cacheData.data(), cacheSize);
            nvinfer1::ITimingCache* timingCache = builderConfig->createTimingCache(cacheData.data(), cacheSize);
            if (timingCache) {
                builderConfig->setTimingCache(*timingCache, false);
                timingCache->destroy();
            }
        }
    }

    // Optimization profile for dynamic batch
    if (config.enableDynamicBatch) {
        nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
        // Assuming first input is the image tensor
        nvinfer1::Dims inputDims = network->getInput(0)->getDimensions();
        nvinfer1::Dims minDims = inputDims, optDims = inputDims, maxDims = inputDims;
        minDims.d[0] = 1;
        optDims.d[0] = config.maxBatchSize;
        maxDims.d[0] = config.maxBatchSize;
        profile->setDimensions(network->getInput(0)->getName(),
                               nvinfer1::OptProfileSelector::kMIN, minDims);
        profile->setDimensions(network->getInput(0)->getName(),
                               nvinfer1::OptProfileSelector::kOPT, optDims);
        profile->setDimensions(network->getInput(0)->getName(),
                               nvinfer1::OptProfileSelector::kMAX, maxDims);
        builderConfig->addOptimizationProfile(profile);
    }

    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network, *builderConfig);
    if (!engine) {
        DYNALGO_LOG_ERROR_S("Failed to build engine");
        // Cleanup
        if (config.enableDynamicBatch) {
            // profile destroyed with builderConfig
        }
        builderConfig->destroy(); parser->destroy(); network->destroy(); builder->destroy();
        return false;
    }

    // Save timing cache
    if (!config.timingCache.empty()) {
        nvinfer1::ITimingCache* timingCache = builderConfig->getTimingCache();
        if (timingCache) {
            nvinfer1::IHostMemory* cacheData = timingCache->serialize();
            std::ofstream out(config.timingCache, std::ios::binary);
            out.write(static_cast<const char*>(cacheData->data()), cacheData->size());
            cacheData->destroy();
        }
    }

    // Serialize and save
    nvinfer1::IHostMemory* serialized = engine->serialize();
    std::string outputPath = config.modelPath;
    size_t dotPos = outputPath.rfind('.');
    if (dotPos != std::string::npos) {
        outputPath = outputPath.substr(0, dotPos) + ".engine";
    }
    std::ofstream out(outputPath, std::ios::binary);
    out.write(static_cast<const char*>(serialized->data()), serialized->size());
    out.close();

    // Cleanup
    serialized->destroy();
    engine->destroy();
    builderConfig->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();

    DYNALGO_LOG_INFO_S("Engine built and saved to: " << outputPath);
    return true;
}

bool TrtBackend::buildEngineWithCalibration(const TrtBackendConfig& config,
                                            IInt8CalibratorProvider* calibrator) {
    if (!calibrator) {
        DYNALGO_LOG_ERROR_S("TrtBackend::buildEngineWithCalibration: calibrator is null");
        return false;
    }

    DYNALGO_LOG_INFO_S("TrtBackend::buildEngineWithCalibration: " << config.modelPath);

    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(gTrtLogger);
    if (!builder) return false;

    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
    if (!network) { builder->destroy(); return false; }

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gTrtLogger);
    if (!parser) { network->destroy(); builder->destroy(); return false; }

    if (!parser->parseFromFile(config.modelPath.c_str(),
                                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        DYNALGO_LOG_ERROR_S("Failed to parse ONNX: " << config.modelPath);
        parser->destroy(); network->destroy(); builder->destroy();
        return false;
    }

    nvinfer1::IBuilderConfig* builderConfig = builder->createBuilderConfig();
    builderConfig->setMaxWorkspaceSize(config.workspaceSize);
    builderConfig->setFlag(nvinfer1::BuilderFlag::kINT8);

    // Create calibrator
    TrtInt8Calibrator calibratorImpl(
        calibrator->getBatchSize(),
        config.calibrationCache,
        [calibrator](void* bindings[], const char* names[], int nbBindings) {
            return calibrator->getBatch(bindings, names, nbBindings);
        },
        [calibrator](size_t& length) {
            return calibrator->readCalibrationCache(length);
        },
        [calibrator](const void* cache, size_t length) {
            calibrator->writeCalibrationCache(cache, length);
        }
    );

    builderConfig->setInt8Calibrator(&calibratorImpl);

    // Optimization profile
    if (config.enableDynamicBatch) {
        nvinfer1::IOptimizationProfile* profile = builder->createOptimizationProfile();
        nvinfer1::Dims inputDims = network->getInput(0)->getDimensions();
        nvinfer1::Dims minDims = inputDims, optDims = inputDims, maxDims = inputDims;
        minDims.d[0] = 1;
        optDims.d[0] = config.maxBatchSize;
        maxDims.d[0] = config.maxBatchSize;
        profile->setDimensions(network->getInput(0)->getName(),
                               nvinfer1::OptProfileSelector::kMIN, minDims);
        profile->setDimensions(network->getInput(0)->getName(),
                               nvinfer1::OptProfileSelector::kOPT, optDims);
        profile->setDimensions(network->getInput(0)->getName(),
                               nvinfer1::OptProfileSelector::kMAX, maxDims);
        builderConfig->addOptimizationProfile(profile);
    }

    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network, *builderConfig);
    if (!engine) {
        DYNALGO_LOG_ERROR_S("Failed to build INT8 engine");
        builderConfig->destroy(); parser->destroy(); network->destroy(); builder->destroy();
        return false;
    }

    // Serialize and save
    nvinfer1::IHostMemory* serialized = engine->serialize();
    std::string outputPath = config.modelPath;
    size_t dotPos = outputPath.rfind('.');
    if (dotPos != std::string::npos) {
        outputPath = outputPath.substr(0, dotPos) + ".engine";
    }
    std::ofstream out(outputPath, std::ios::binary);
    out.write(static_cast<const char*>(serialized->data()), serialized->size());
    out.close();

    // Cleanup
    serialized->destroy();
    engine->destroy();
    builderConfig->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();

    DYNALGO_LOG_INFO_S("INT8 Engine built and saved to: " << outputPath);
    return true;
}

bool TrtBackend::serializeEngine(const std::string& enginePath) const {
    if (!pimpl_ || !pimpl_->engine) {
        DYNALGO_LOG_ERROR_S("TrtBackend::serializeEngine: no engine loaded");
        return false;
    }

    nvinfer1::IHostMemory* serialized = pimpl_->engine->serialize();
    if (!serialized) {
        DYNALGO_LOG_ERROR_S("TrtBackend::serializeEngine: serialization failed");
        return false;
    }

    std::ofstream out(enginePath, std::ios::binary);
    out.write(static_cast<const char*>(serialized->data()), serialized->size());
    out.close();

    serialized->destroy();
    DYNALGO_LOG_INFO_S("Engine serialized to: " << enginePath);
    return true;
}

std::unique_ptr<DynalgoModelBackend> createTrtBackend() {
    return std::make_unique<TrtBackend>();
}

#else // ENABLE_TENSORRT not defined

TrtBackend::TrtBackend() : config_{} {}
TrtBackend::~TrtBackend() = default;
TrtBackend::TrtBackend(TrtBackend&&) noexcept = default;
TrtBackend& TrtBackend::operator=(TrtBackend&&) noexcept = default;

void TrtBackend::setConfig(const TrtBackendConfig&) {}
const TrtBackendConfig& TrtBackend::getConfig() const { return config_; }
void TrtBackend::setCUDAStream(cudaStream_t) {}

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
bool TrtBackend::buildEngineFromOnnx(const TrtBackendConfig&) {
    return false;
}
bool TrtBackend::buildEngineWithCalibration(const TrtBackendConfig&, IInt8CalibratorProvider*) {
    return false;
}
bool TrtBackend::serializeEngine(const std::string&) const {
    return false;
}
void TrtBackend::setConfig(const TrtBackendConfig&) {}
const TrtBackendConfig& TrtBackend::getConfig() const { return config_; }
void TrtBackend::setCUDAStream(cudaStream_t) {}

std::unique_ptr<DynalgoModelBackend> createTrtBackend() {
    return nullptr;
}

#endif // ENABLE_TENSORRT

} // namespace dynalgo