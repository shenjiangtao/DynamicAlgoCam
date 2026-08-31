// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_track_bundle.cpp — DynalgoTrackBundle implementation.

#include "dynalgo_track_bundle.hpp"
#include "dynalgo_log.hpp"

namespace dynalgo {

// [方法说明 / Method Description]
// 中文: 初始化轨迹包
// English: Initialize track bundle
void DynalgoTrackBundle::init(const DynalgoDetectionResult& det)
{
    if (tracker_.initialised())
        DYNALGO_LOG_WARN_S("DynalgoTrackBundle::init re-initialising an already-initialised bundle");
    tracker_.init(det);
}

// [方法说明 / Method Description]
// 中文: 更新轨迹包并刷新3D定位
// English: Update track bundle and refresh 3D fix
void DynalgoTrackBundle::update(const DynalgoDetectionResult& det,
                                 const DynalgoFrame& depthAligned,
                                 const DynalgoIntrinsic& intr,
                                 float depthScale)
{
    DynalgoDetectionResult smoothed = tracker_.update(det);

    // Refresh 3D fix from the smoothed bbox centre. If it fails (zero-depth
    // window, etc.) keep the previous cached fix so the engagement loop can
    // decide what to do.
    float X, Y, Z;
    if (detectionCenterToCamera3D(depthAligned, intr, depthScale, /*filterHalf=*/2,
                                   smoothed, X, Y, Z)) {
        lastX_ = X; lastY_ = Y; lastZ_ = Z;
        lastHasFix_ = true;
    }
}

// [方法说明 / Method Description]
// 中文: 预测下一帧位置
// English: Predict position for next frame
DynalgoDetectionResult DynalgoTrackBundle::predict()
{
    return tracker_.predict();
}

} // namespace dynalgo
