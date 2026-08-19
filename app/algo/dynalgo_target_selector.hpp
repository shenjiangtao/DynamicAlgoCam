// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_target_selector.hpp — Pick one detection among a frame's detections
// per a fixed strategy. Used by EngagementLoop to commit to a single target.
//
// NOT called by capture main; reserved for the Phase C engagement loop.

#pragma once

#include "dynalgo_model.hpp"

#include <optional>
#include <vector>

namespace dynalgo {

// Strategy for picking a target when more than one detection is present.
enum class SelectorStrategy {
    HIGHEST_SCORE,   // pick the detection with the largest `score`
    NEAREST_DEPTH,   // pick the detection whose last-known 3D fix has the smallest |Z|
                     // (caller precomputes Z via detectionCenterToCamera3D)
    LARGEST_AREA     // pick the detection with the largest `w * h`
};

// Stateless convenience picker. The optional `depthSortedMeters` is only used
// when strategy == NEAREST_DEPTH; entries are Z-metres aligned 1:1 with
// `detections` (NaN / missing entries are treated as +inf).
//
// Returns std::nullopt when `detections` is empty.
std::optional<DynalgoDetectionResult> pickTarget(const std::vector<DynalgoDetectionResult>& detections,
                                                   SelectorStrategy strategy,
                                                   const std::vector<float>* depthSortedMeters = nullptr);

} // namespace dynalgo
