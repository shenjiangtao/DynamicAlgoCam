// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_model.hpp — SDK-neutral model inference abstraction layer.
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

// Algorithm / inference backend type. Mirrors DynalgoDriverVendor's pattern.
enum class DynalgoModelType {
    NONE = 0,      // no backend — infer() is a no-op
    DUMMY,         // in-process stub for unit tests / dry runs
    YOLOV8_PY,     // ultralytics YOLOv8 via Python (subprocess or IPC bridge)
    ONNXRUNTIME,   // future: ONNX Runtime C++ backend
    TENSORRT       // future: TensorRT backend
};

// One detection / classification result, SDK-neutral.
// Keep POD-like; backends map their own output schema onto this.
struct DynalgoDetectionResult {
    int classId = -1;            // category index (model-specific)
    float score = 0.0f;          // confidence in [0,1]
    float x = 0.0f;              // bounding-box top-left X (pixels, src frame)
    float y = 0.0f;              // bounding-box top-left Y
    float w = 0.0f;              // box width
    float h = 0.0f;              // box height
    std::string label;           // human-readable class label (optional)
};

// Backend configuration. Reserved for backend-specific params (weights path,
// device hint, conf threshold, ...). Backends cast/handle keys they understand.
struct DynalgoModelConfig {
    std::string modelPath;       // weights / model file or model name
    std::string deviceHint;      // e.g. "cpu", "gpu", "cuda:0", empty = default
    float confThreshold = 0.25f; // minimum score to keep a detection
    float iouThreshold = 0.45f;  // NMS IoU threshold (when applicable)
};

// Abstract inference backend. Concrete implementations are created by
// DynalgoModelFactory (see dynalgo_model_factory.hpp) and live in driver/model code.
class DynalgoModelBackend
{
public:
    virtual ~DynalgoModelBackend() = default;

    // Load weights / initialise backend. Returns true on success.
    // Must be called once before infer().
    virtual bool load(const DynalgoModelConfig& cfg) = 0;

    // Run inference on one frame. Implementations should be thread-safe
    // if they intend to be called from multiple capture threads.
    // Returns true on success; results appended to `out` (NOT cleared by
    // the interface — caller controls output buffer lifecycle).
    virtual bool infer(const DynalgoFrame& frame, std::vector<DynalgoDetectionResult>& out) = 0;

    // Human-readable backend name, e.g. "YOLOV8_PY", "DUMMY".
    virtual const char* name() const = 0;
};

} // namespace dynalgo
