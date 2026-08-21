// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// trt_backend.hpp — TensorRT C++ inference backend for DynalgoModelBackend.

#pragma once

#include "dynalgo_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dynalgo {

// Forward declaration for TensorRT types
struct IExecutionContext;
struct ICudaEngine;
struct IRuntime;

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

    // TensorRT-specific
    bool buildEngineFromOnnx(const std::string& onnxPath,
                             const std::string& enginePath,
                             bool fp16 = true,
                             int maxBatchSize = 1,
                             size_t workspaceSize = 1 << 30); // 1GB

    // Get engine info
    int getInputWidth() const { return inputWidth_; }
    int getInputHeight() const { return inputHeight_; }
    int getNumClasses() const { return numClasses_; }

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    // Model metadata
    int inputWidth_ = 640;
    int inputHeight_ = 640;
    int numClasses_ = 80;
    int inputChannels_ = 3;
    bool isLoaded_ = false;

    // Preprocessing config
    struct PreprocessConfig preprocessCfg_;
};

// Factory function for self-registration
std::unique_ptr<DynalgoModelBackend> createTrtBackend();

} // namespace dynalgo