// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// trt_backend.hpp — TensorRT C++ inference backend for DynalgoModelBackend.

#pragma once

#include "dynalgo_model.hpp"
#include "preprocessing.hpp"

#include <memory>
#include <string>
#include <vector>
#include <functional>

#ifdef ENABLE_TENSORRT
#include <cuda_runtime_api.h>
#endif

namespace dynalgo {

// Forward declaration for TensorRT types
struct IExecutionContext;
struct ICudaEngine;
struct IRuntime;
struct IInt8Calibrator;

// Calibration data provider interface for INT8 quantization
class IInt8CalibratorProvider {
public:
    virtual ~IInt8CalibratorProvider() = default;
    virtual bool getBatch(void* bindings[], const char* names[], int nbBindings) = 0;
    virtual const void* readCalibrationCache(size_t& length) = 0;
    virtual void writeCalibrationCache(const void* cache, size_t length) = 0;
    virtual int getBatchSize() const = 0;
};

// TensorRT backend configuration
struct TrtBackendConfig {
    std::string modelPath;           // Path to .engine or .onnx file
    bool fp16 = true;                // Enable FP16 mode
    bool int8 = false;               // Enable INT8 mode (requires calibration)
    std::string calibrationCache;    // Path to calibration cache file
    std::string timingCache;         // Path to timing cache file
    int maxBatchSize = 1;            // Maximum batch size
    size_t workspaceSize = 1 << 30;  // Workspace size (1GB default)
    int deviceId = 0;                // GPU device ID
    bool enableDynamicBatch = true;  // Enable dynamic batch optimization profile
    std::string inputName;           // Input tensor name (optional)
    std::string outputName;          // Output tensor name (optional)
#ifdef ENABLE_TENSORRT
    cudaStream_t externalStream = 0; // External CUDA stream for async pipeline
#else
    void* externalStream = nullptr;  // Placeholder when TensorRT not enabled
#endif
};

class TrtBackend : public DynalgoModelBackend
{
public:
    TrtBackend();
    ~TrtBackend() override;

    // Non-copyable, movable
    TrtBackend(const TrtBackend&) = delete;
    TrtBackend& operator=(const TrtBackend&) = delete;
    TrtBackend(TrtBackend&&) noexcept;
    TrtBackend& operator=(TrtBackend&&) noexcept;

    // DynalgoModelBackend interface
    bool load(const DynalgoModelConfig& cfg) override;
    bool infer(const DynalgoFrame& frame,
               std::vector<DynalgoDetectionResult>& out) override;
    const char* name() const noexcept override { return "TensorRT"; }

    // TensorRT-specific configuration
    void setConfig(const TrtBackendConfig& config) { config_ = config; }
    const TrtBackendConfig& getConfig() const { return config_; }

    // Set external CUDA stream for async pipeline integration
#ifdef ENABLE_TENSORRT
    void setCUDAStream(cudaStream_t stream) { config_.externalStream = stream; }
#else
    void setCUDAStream(void* stream) { (void)stream; }
#endif

    // Build engine with full configuration
    bool buildEngineFromOnnx(const TrtBackendConfig& config);

    // Build engine with INT8 calibration
    bool buildEngineWithCalibration(const TrtBackendConfig& config,
                                    IInt8CalibratorProvider* calibrator);

    // Get engine info
    int getInputWidth() const { return inputWidth_; }
    int getInputHeight() const { return inputHeight_; }
    int getNumClasses() const { return numClasses_; }
    bool isLoaded() const { return isLoaded_; }

    // Get engine serialization for deployment
    bool serializeEngine(const std::string& enginePath) const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    TrtBackendConfig config_;

    // Model metadata
    int inputWidth_ = 640;
    int inputHeight_ = 640;
    int numClasses_ = 80;
    int inputChannels_ = 3;
    bool isLoaded_ = false;

    // Preprocessing config
    PreprocessConfig preprocessCfg_;
};

// Factory function for self-registration
std::unique_ptr<DynalgoModelBackend> createTrtBackend();

} // namespace dynalgo