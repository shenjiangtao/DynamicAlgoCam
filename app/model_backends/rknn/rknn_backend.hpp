// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// rknn_backend.hpp — RKNN Runtime C++ inference backend for DynalgoModelBackend.

#pragma once

#include "dynalgo_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dynalgo {

class RknnBackend : public DynalgoModelBackend
{
public:
    RknnBackend();
    ~RknnBackend() override;

    // Non-copyable, movable
    RknnBackend(const RknnBackend&) = delete;
    RknnBackend& operator=(const RknnBackend&) = delete;
    RknnBackend(RknnBackend&&) noexcept;
    RknnBackend& operator=(RknnBackend&&) noexcept;

    // DynalgoModelBackend interface
    bool load(const DynalgoModelConfig& cfg) override;
    bool infer(const DynalgoFrame& frame,
               std::vector<DynalgoDetectionResult>& out) override;
    const char* name() const noexcept override { return "RKNN"; }

    // RKNN specific
    void setCoreMask(int coreMask); // NPU core affinity

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    int inputWidth_ = 640;
    int inputHeight_ = 640;
    int numClasses_ = 80;
    int inputChannels_ = 3;
    bool isLoaded_ = false;

    struct PreprocessConfig preprocessCfg_;
    int coreMask_ = 0;
};

// Factory function for self-registration
std::unique_ptr<DynalgoModelBackend> createRknnBackend();

} // namespace dynalgo