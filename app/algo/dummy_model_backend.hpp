// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dummy_model_backend.hpp — In-process stub backend for dry-run / unit tests.
// Produces a fixed synthetic detection at frame centre when enabled.

#pragma once

#include "dynalgo_model.hpp"

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 虚拟模型后端，用于干跑运行和单元测试
// English: Dummy model backend for dry-run and unit tests
class DummyModelBackend : public DynalgoModelBackend
{
public:
    DummyModelBackend() = default;

    // [方法说明 / Method Description]
    // 中文: 加载模型配置
    // English: Load model configuration
    bool load(const DynalgoModelConfig& cfg) override;
    // [方法说明 / Method Description]
    // 中文: 执行模型推理
    // English: Run model inference
    bool infer(const DynalgoFrame& frame, std::vector<DynalgoDetectionResult>& out) override;
    // [方法说明 / Method Description]
    // 中文: 获取模型后端名称
    // English: Get model backend name
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