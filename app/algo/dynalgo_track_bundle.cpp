// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_track_bundle.cpp — DynalgoTrackBundle implementation.

#include "dynalgo_track_bundle.hpp"
#include "dynalgo_log.hpp"

namespace dynalgo {

void DynalgoTrackBundle::init(const DynalgoDetectionResult& det)
{
    if (tracker_.initialised())
        DYNALGO_LOG_WARN_S("DynalgoTrackBundle::init re-initialising an already-initialised bundle");
    tracker_.init(det);
}

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
    if (detectionCenterToCamera3D(depthAligned, intr, depthScale, /*filterHalf=*/0,
                                   smoothed, X, Y, Z)) {
        lastX_ = X; lastY_ = Y; lastZ_ = Z;
        lastHasFix_ = true;
    }
}

DynalgoDetectionResult DynalgoTrackBundle::predict()
{
    return tracker_.predict();
}

} // namespace dynalgo
