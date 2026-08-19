// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dummy_model_backend.hpp — In-process stub backend for dry-run / unit tests.
// Produces a fixed synthetic detection at frame centre when enabled.

#pragma once

#include "dynalgo_model.hpp"

namespace dynalgo {

class DummyModelBackend : public DynalgoModelBackend
{
public:
    DummyModelBackend() = default;

    bool load(const DynalgoModelConfig& cfg) override;
    bool infer(const DynalgoFrame& frame, std::vector<DynalgoDetectionResult>& out) override;
    const char* name() const override { return "DUMMY"; }

    // Test knobs
    void setEnabled(bool v) { enabled_ = v; }
    void setFixedDetection(const DynalgoDetectionResult& det) { fixedDet_ = det; hasFixedDet_ = true; }
    void clearFixedDetection() { hasFixedDet_ = false; }

private:
    bool enabled_ = true;
    bool hasFixedDet_ = false;
    DynalgoDetectionResult fixedDet_{};
};

} // namespace dynalgo