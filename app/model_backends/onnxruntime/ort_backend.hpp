// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// ort_backend.hpp — ONNX Runtime C++ inference backend for DynalgoModelBackend.

#pragma once

#include "dynalgo_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dynalgo {

class OrtBackend : public DynalgoModelBackend
{
public:
    OrtBackend();
    ~OrtBackend() override;

    // Non-copyable, movable
    OrtBackend(const OrtBackend&) = delete;
    OrtBackend& operator=(const OrtBackend&) = delete;
    OrtBackend(OrtBackend&&) noexcept;
    OrtBackend& operator=(OrtBackend&&) noexcept;

    // DynalgoModelBackend interface
    bool load(const DynalgoModelConfig& cfg) override;
    bool infer(const DynalgoFrame& frame,
               std::vector<DynalgoDetectionResult>& out) override;
    const char* name() const noexcept override { return "ONNXRuntime"; }

    // ONNX Runtime specific
    void setProviders(const std::vector<std::string>& providers); // "CUDAExecutionProvider", "CPUExecutionProvider"
    void setIntraOpThreads(int n);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    int inputWidth_ = 640;
    int inputHeight_ = 640;
    int numClasses_ = 80;
    int inputChannels_ = 3;
    bool isLoaded_ = false;

    struct PreprocessConfig preprocessCfg_;
    std::vector<std::string> providers_ = {"CPUExecutionProvider"};
    int intraOpThreads_ = 0;
};

// Factory function for self-registration
std::unique_ptr<DynalgoModelBackend> createOrtBackend();

} // namespace dynalgo