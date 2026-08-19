// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_model.hpp — SDK-neutral model inference abstraction layer.
//
// [文件说明 / File Description]
// 中文：SDK中立的模型推理抽象层，定义可插拔算法/推理后端的接口契约
// English: SDK-neutral model inference abstraction layer, defines the contract for pluggable algorithm/inference backends
//
// Defines the contract for pluggable algorithm / inference backends
// (YOLOv8, ONNX Runtime, TensorRT, dummy, ...). Backends live outside
// dynalgo_core (e.g. under app/driver or app/models); this header only states
// the interface. dynalgo_core stays SDK-neutral — no torch / onnx / python
// dependency is added by including this file.
//
// Usage:
//   auto backend = createModelBackend(DynalgoModelType::DUMMY, cfg);
//   if (backend && backend->load("path/to/weights")) {
//       std::vector<DynalgoDetectionResult> out;
//       backend->infer(frame, out);
//   }
//
// See docs/dynamic_algo_cam/models_overview.md for the engineering rationale
// (vendors/ vs app/models/, license disclosure, future IPC bridge patterns).

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dynalgo {

// [枚举说明 / Enum Description]
// 中文：算法/推理后端类型，与DynalgoDriverVendor模式保持一致
// English: Algorithm / inference backend type. Mirrors DynalgoDriverVendor's pattern.
enum class DynalgoModelType {
    NONE = 0,      // no backend — infer() is a no-op
    DUMMY,         // in-process stub for unit tests / dry runs
    YOLOV8_PY,     // ultralytics YOLOv8 via Python (subprocess or IPC bridge)
    ONNXRUNTIME,   // future: ONNX Runtime C++ backend
    TENSORRT       // future: TensorRT backend
};

// [结构体说明 / Struct Description]
// 中文：检测/分类结果，SDK中立，保持POD结构，后端可映射自己的输出格式
// English: One detection / classification result, SDK-neutral. Keep POD-like; backends map their own output schema onto this.
struct DynalgoDetectionResult {
    int classId = -1;            // category index (model-specific)
    float score = 0.0f;          // confidence in [0,1]
    float x = 0.0f;              // bounding-box top-left X (pixels, src frame)
    float y = 0.0f;              // bounding-box top-left Y
    float w = 0.0f;              // box width
    float h = 0.0f;              // box height
    std::string label;           // human-readable class label (optional)
};

// [结构体说明 / Struct Description]
// 中文：后端配置，保留用于后端特定参数（权重路径、设备提示、置信度阈值等）
// English: Backend configuration. Reserved for backend-specific params (weights path, device hint, conf threshold, ...).
struct DynalgoModelConfig {
    std::string modelPath;       // weights / model file or model name
    std::string deviceHint;      // e.g. "cpu", "gpu", "cuda:0", empty = default
    float confThreshold = 0.25f; // minimum score to keep a detection
    float iouThreshold = 0.45f;  // NMS IoU threshold (when applicable)
};

// [类说明 / Class Description]
// 中文：抽象推理后端基类，具体实现由DynalgoModelFactory创建
// English: Abstract inference backend. Concrete implementations are created by DynalgoModelFactory.
class DynalgoModelBackend
{
public:
    virtual ~DynalgoModelBackend() = default;

    // [方法说明 / Method Description]
    // 中文：加载权重/初始化后端，成功返回true，必须在infer()之前调用
    // English: Load weights / initialise backend. Returns true on success. Must be called once before infer().
    virtual bool load(const DynalgoModelConfig& cfg) = 0;

    // [方法说明 / Method Description]
    // 中文：对单帧运行推理，结果追加到out向量，线程安全实现需支持多线程调用
    // English: Run inference on one frame. Results appended to `out`. Thread-safe implementations should support multi-threaded calls.
    virtual bool infer(const DynalgoFrame& frame, std::vector<DynalgoDetectionResult>& out) = 0;

    // [方法说明 / Method Description]
    // 中文：返回人类可读的后端名称
    // English: Return human-readable backend name
    virtual const char* name() const = 0;
};

} // namespace dynalgo
