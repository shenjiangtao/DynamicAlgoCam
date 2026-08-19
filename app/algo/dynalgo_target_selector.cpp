// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_target_selector.cpp — pickTarget() implementation.

#include "dynalgo_target_selector.hpp"

#include <cmath>
#include <limits>

namespace dynalgo {

// [函数说明 / Function Description]
// 中文: 根据策略从多个检测中选择一个目标
// English: Pick one detection among multiple detections based on strategy
std::optional<DynalgoDetectionResult> pickTarget(const std::vector<DynalgoDetectionResult>& detections,
                                                   SelectorStrategy strategy,
                                                   const std::vector<float>* depthSortedMeters)
{
    if (detections.empty())
        return std::nullopt;

    size_t bestIdx = 0;
    float  bestMetric = 0.0f;
    bool   first = true;

    for (size_t i = 0; i < detections.size(); ++i) {
        float m = 0.0f;
        switch (strategy) {
        case SelectorStrategy::HIGHEST_SCORE:
            m = detections[i].score;
            break;
        case SelectorStrategy::LARGEST_AREA:
            m = detections[i].w * detections[i].h;
            break;
        case SelectorStrategy::NEAREST_DEPTH: {
            float Z = std::numeric_limits<float>::infinity();
            if (depthSortedMeters && i < depthSortedMeters->size()) {
                float zCandidate = (*depthSortedMeters)[i];
                if (!std::isnan(zCandidate))
                    Z = zCandidate;
            }
            // Pick the minimum Z; use +Z as the metric so "smallest wins".
            // Invert: we want metric=−Z so the largest-metric argmax gives min-Z.
            m = -Z;
            break;
        }
        }
        if (first || m > bestMetric) {
            bestMetric = m;
            bestIdx = i;
            first = false;
        }
    }

    return detections[bestIdx];
}

} // namespace dynalgo
