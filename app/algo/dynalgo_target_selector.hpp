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

// [枚举说明 / Enum Description]
// 中文: 目标选择策略
// English: Target selection strategies
enum class SelectorStrategy {
    // [枚举值说明 / Enum Value Description]
    // 中文: 选择得分最高的检测
    // English: Pick detection with highest score
    HIGHEST_SCORE,   // pick the detection with the largest `score`
    // [枚举值说明 / Enum Value Description]
    // 中文: 选择最近深度的检测
    // English: Pick detection with nearest depth
    NEAREST_DEPTH,   // pick the detection whose last-known 3D fix has the smallest |Z|
                     // (caller precomputes Z via detectionCenterToCamera3D)
    // [枚举值说明 / Enum Value Description]
    // 中文: 选择最大面积的检测
    // English: Pick detection with largest area
    LARGEST_AREA     // pick the detection with the largest `w * h`
};

// Stateless convenience picker. The optional `depthSortedMeters` is only used
// when strategy == NEAREST_DEPTH; entries are Z-metres aligned 1:1 with
// `detections` (NaN / missing entries are treated as +inf).
//
// Returns std::nullopt when `detections` is empty.
// [函数说明 / Function Description]
// 中文: 根据策略从多个检测中选择一个目标
// English: Pick one detection among multiple detections based on strategy
std::optional<DynalgoDetectionResult> pickTarget(const std::vector<DynalgoDetectionResult>& detections,
                                                   SelectorStrategy strategy,
                                                   const std::vector<float>* depthSortedMeters = nullptr);

} // namespace dynalgo
