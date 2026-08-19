// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_track_bundle.hpp — Single-target trajectory bundle: combines the
// DynalgoKalmanTracker (2D bbox smoothing) with a cached 3D fix computed via
// detectionCenterToCamera3D().
//
// Reserved for the Phase C engagement loop; NOT called by capture main.
// Multi-target association is out of scope (see IMPLEMENTATION_TASKS.md O5).

#pragma once

#include "dynalgo_detection_to_3d.hpp"
#include "dynalgo_kalman_tracker.hpp"
#include "dynalgo_model.hpp"

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 单目标轨迹包，结合卡尔曼滤波器和缓存的3D定位
// English: Single-target trajectory bundle combining Kalman filter and cached 3D fix
class DynalgoTrackBundle
{
public:
    DynalgoTrackBundle() = default;

    // [方法说明 / Method Description]
    // 中文: 从首次检测初始化轨迹包
    // English: Initialize from the first detection
    void init(const DynalgoDetectionResult& det);

    // [方法说明 / Method Description]
    // 中文: 融入新测量值并刷新3D定位
    // English: Incorporate a new measurement and refresh the 3D fix
    void update(const DynalgoDetectionResult& det,
                const DynalgoFrame& depthAligned,
                const DynalgoIntrinsic& intr,
                float depthScale);

    // [方法说明 / Method Description]
    // 中文: 无测量时进行时间传播并返回预测边界框
    // English: Time-propagate without measurement and return predicted bbox
    DynalgoDetectionResult predict();

    bool initialised() const { return tracker_.initialised(); }
    bool hasFix()      const { return lastHasFix_; }

    // Most-recent 3-D fix (valid iff hasFix()).
    float lastX() const { return lastX_; }
    float lastY() const { return lastY_; }
    float lastZ() const { return lastZ_; }

private:
    DynalgoKalmanTracker tracker_;
    bool  lastHasFix_ = false;
    float lastX_ = 0.0f, lastY_ = 0.0f, lastZ_ = 0.0f;
};

} // namespace dynalgo
