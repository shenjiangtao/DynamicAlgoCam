// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_detection_to_3d.hpp — Back-project a 2D detection box center into the
// camera optical frame (meters), given a D2C-aligned depth frame.
//
// [文件说明 / File Description]
// 中文：将2D检测框中心反投影到相机光学坐标系（米），需要D2C对齐的深度帧
// English: Back-project a 2D detection box center into the camera optical frame (meters), given a D2C-aligned depth frame
//
// Header-only; dynalgo_core consumes no extra source. Mirrors the pinhole model
// already used by PointcloudFrameConsumer::backprojectToPointCloud() (see
// app/capture/dynalgo_frame_consumer.cpp:144-174):
//
//     u  = det.x + det.w / 2
//     v  = det.y + det.h / 2
//     Z  = rawDepthAt(u, v) * depthScale          (Y16 source: uint16 millimetres)
//     X  = (u - k.cx) * Z / k.fx
//     Y  = (v - k.cy) * Z / k.fy
//
// PREREQUISITE (must be enforced by the caller):
//   `depthAligned` MUST be D2C-aligned to the color frame on which the
//   detection was produced. If a raw (unaligned) depth frame is passed, the
//   pixel (u, v) refers to a different physical ray than the color pixel at
//   the same (u, v) — X and Y will be wrong. Callers should check
//   pipeline->getAlignMode() == dynalgo::types::DynalgoAlignMode::HW or ::SW
//   before invoking; otherwise the returned 3-D point is untrustworthy.
//
// filterHalf:
//   - 0  → single-pixel raw depth (fastest, most noisy on edge pixels)
//   - k>0 → median of the (2k+1)×(2k+1) window around (u,v), ignoring zero
//           (invalid) depth pixels. If the window has zero valid pixels the
//           function returns false. Recommended value: 2 (5×5 window) for the
//           common case of small targets at near-to-mid range.
//
// Out-param semantics:
//   When the function returns false, outX/outY/outZ are left untouched so the
//   caller can distinguish "no 3D fix this frame" from "fix at (0,0,0)".
//   Z is returned in meters (because depthScale converts Y16→meters).
//
// Coordinate convention:
//   Origin at the camera optical center. +X right, +Y down, +Z forward.
//   Same frame as the cloud produced by backprojectToPointCloud().
//
// Not in scope (per DEVELOPMENT_PLAN.md §1.2 range boundary):
//   - Multi-frame 3D Kalman filtering (XYZ time-series smoothing) — listed
//     as O6 in IMPLEMENTATION_TASKS.md range-out tracking.
//   - "Nearest point" / "mask centroid" depth sampling — detection here is a
//     plain axis-aligned box; centre is the geometric centre (x+w/2, y+h/2).

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_model.hpp"
#include "dynalgo_types.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace dynalgo {

// [方法说明 / Method Description]
// 中文：将检测框中心反投影到相机坐标系，使用针孔模型和中值滤波深度
// English: Back-project detection box center to camera coordinate system using pinhole model and median-filtered depth
//
// Returns false when the centre sits outside the depth frame, the depth frame
// is not Y16 / wrong-sized, the Y16 value(s) in the window are all zero (invalid),
// or the intrinsic is degenerate (fx/fy == 0). Output values are unchanged on
// false so callers can keep the last known 3-D fix.
inline bool detectionCenterToCamera3D(const DynalgoFrame& depthAligned,
                                       const DynalgoIntrinsic& intr,
                                       float depthScale,
                                       int filterHalf,
                                       const DynalgoDetectionResult& det,
                                       float& outX, float& outY, float& outZ)
{
    if (depthAligned.format != DynalgoFormat::Y16)
        return false;
    if (depthScale <= 0.0f || intr.fx == 0.0f || intr.fy == 0.0f)
        return false;
    const int w = depthAligned.width;
    const int h = depthAligned.height;
    if (w <= 0 || h <= 0)
        return false;
    if (depthAligned.data.size() < static_cast<size_t>(w) * h * sizeof(uint16_t))
        return false;

    if (filterHalf < 0)
        filterHalf = 0;

    const int u = static_cast<int>(det.x + 0.5f * det.w);
    const int v = static_cast<int>(det.y + 0.5f * det.h);
    if (u < 0 || u >= w || v < 0 || v >= h)
        return false;

    const auto* y16 = reinterpret_cast<const uint16_t*>(depthAligned.data.data());

    // Compute the window bounds, clamped to [0, w-1] / [0, h-1] so the function
    // works even when the detection centre sits at the frame border.
    int u0 = u - filterHalf, u1 = u + filterHalf;
    int v0 = v - filterHalf, v1 = v + filterHalf;
    if (u0 < 0)  u0 = 0;
    if (v0 < 0)  v0 = 0;
    if (u1 >= w) u1 = w - 1;
    if (v1 >= h) v1 = h - 1;

    // Collect the non-zero depth values in the window. Returns false when the
    // window has zero valid pixels (e.g. the target is too close, or the
    // surface is specular and returns no depth at this region).
    std::vector<uint16_t> window;
    window.reserve(static_cast<size_t>((u1 - u0 + 1) * (v1 - v0 + 1)));
    for (int vv = v0; vv <= v1; ++vv) {
        for (int uu = u0; uu <= u1; ++uu) {
            uint16_t d = y16[static_cast<size_t>(vv) * w + uu];
            if (d != 0)
                window.push_back(d);
        }
    }
    if (window.empty())
        return false;

    std::sort(window.begin(), window.end());
    const uint16_t median = window[window.size() / 2];

    const float Z = static_cast<float>(median) * depthScale;
    const float X = (static_cast<float>(u) - intr.cx) * Z / intr.fx;
    const float Y = (static_cast<float>(v) - intr.cy) * Z / intr.fy;

    outX = X;
    outY = Y;
    outZ = Z;
    return true;
}

} // namespace dynalgo
