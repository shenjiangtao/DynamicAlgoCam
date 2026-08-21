// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// sahi.hpp — SAHI (Slicing Aided Hyper Inference) C++ implementation for small object detection.

#pragma once

#include "dynalgo_model.hpp"
#include "dynalgo_frame.hpp"

#include <vector>
#include <memory>
#include <functional>

namespace dynalgo {

// SAHI configuration
struct SahiConfig {
    int sliceHeight = 640;
    int sliceWidth = 640;
    float overlapHeightRatio = 0.2f;  // overlap between slices
    float overlapWidthRatio = 0.2f;
    float confThreshold = 0.25f;
    float iouThreshold = 0.45f;
    bool autoSliceResolution = false; // if true, adapt slice size to image
    int maxSlices = 100;              // safety limit
};

// Slice information
struct Slice {
    int x, y;           // top-left in original image
    int w, h;           // slice size
    int srcW, srcH;     // original image size
};

// SAHI result (detection in original image coordinates)
struct SahiDetection {
    DynalgoDetectionResult detection;
    int sliceIdx;       // which slice this came from
};

// SAHI inference interface
class SahiInference {
public:
    using InferFunc = std::function<bool(const DynalgoFrame&, std::vector<DynalgoDetectionResult>&)>;

    SahiInference(const SahiConfig& cfg, InferFunc inferFunc);
    ~SahiInference() = default;

    // Run SAHI inference on a frame
    // Returns merged detections in original image coordinates
    std::vector<DynalgoDetectionResult> infer(const DynalgoFrame& frame);

    // Get slice grid for visualization/debugging
    std::vector<Slice> getSlices(int imgW, int imgH) const;

private:
    SahiConfig cfg_;
    InferFunc inferFunc_;

    // Generate slice grid
    std::vector<Slice> generateSlices(int imgW, int imgH) const;

    // Extract slice from frame
    DynalgoFrame extractSlice(const DynalgoFrame& frame, const Slice& slice) const;

    // Map detection from slice coordinates to original
    DynalgoDetectionResult mapDetection(const DynalgoDetectionResult& det,
                                        const Slice& slice) const;

    // Merge detections from all slices (NMS across slices)
    std::vector<DynalgoDetectionResult> mergeDetections(
        const std::vector<SahiDetection>& sliceDetections,
        int srcW, int srcH) const;

    // NMS for merging
    std::vector<int> nmsMerge(const std::vector<SahiDetection>& dets,
                              float iouThr, float confThr) const;
};

} // namespace dynalgo