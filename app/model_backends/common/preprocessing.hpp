// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// preprocessing.hpp — Shared preprocessing utilities for C++ inference backends.

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"
#include "dynalgo_model.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <optional>

namespace dynalgo {

// Preprocessing configuration
struct PreprocessConfig {
    int inputWidth = 640;
    int inputHeight = 640;
    bool letterbox = true;         // preserve aspect ratio with padding
    bool bgrToRgb = true;          // convert BGR to RGB
    bool normalize = true;         // normalize to [0,1] or [-1,1]
    float mean[3] = {0.0f, 0.0f, 0.0f};
    float std[3] = {1.0f, 1.0f, 1.0f};
    bool hwcToChw = true;          // HWC -> CHW layout
};

// Letterbox result
struct LetterboxResult {
    int newW, newH;
    int padLeft, padTop;
    float scale;
};

// Compute letterbox transform (preserves aspect ratio)
LetterboxResult computeLetterbox(int srcW, int srcH, int dstW, int dstH);

// Preprocess a DynalgoFrame to contiguous float tensor (CHW or HWC)
// Returns vector<float> of size C*H*W
std::vector<float> preprocessFrame(const DynalgoFrame& frame,
                                   const PreprocessConfig& cfg,
                                   std::optional<LetterboxResult>* lbOut = nullptr);

// CUDA-accelerated preprocessing (if available)
// Preprocess directly to device memory
#ifdef ENABLE_CUDA
bool preprocessFrameCUDA(const DynalgoFrame& frame,
                         const PreprocessConfig& cfg,
                         void* deviceOutput,
                         LetterboxResult* lbOut);
#endif

// NMS implementation (CPU)
struct NMSConfig {
    float iouThreshold = 0.45f;
    float confThreshold = 0.25f;
    int maxDetections = 1000;
    bool classAgnostic = false;
};

// Detection box for NMS
struct DetectionBox {
    float x1, y1, x2, y2;
    float score;
    int classId;
};

// CPU NMS
std::vector<int> nmsCPU(const std::vector<DetectionBox>& boxes, const NMSConfig& cfg);

// CUDA NMS (if available)
#ifdef ENABLE_CUDA
std::vector<int> nmsCUDA(const std::vector<DetectionBox>& boxes, const NMSConfig& cfg);
#endif

// Convert raw model output to DynalgoDetectionResult vector
// Handles: YOLO format [x, y, w, h, conf, class...] or [x1, y1, x2, y2, conf, class...]
std::vector<DynalgoDetectionResult> postprocessDetections(
    const float* output,
    int numDetections,
    int numClasses,
    const LetterboxResult& lb,
    const NMSConfig& cfg,
    int srcW, int srcH,
    bool xywhFormat = true); // true: xywh, false: x1y1x2y2

} // namespace dynalgo