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

class DynalgoTrackBundle
{
public:
    DynalgoTrackBundle() = default;

    // Initialize from the first detection. Calling init() again on an
    // already-initialized bundle logs a WARN and re-initialises.
    void init(const DynalgoDetectionResult& det);

    // Incorporate a new measurement and refresh the 3D fix.
    // - `det`: the latest smoothed/raw target detection (2D)
    // - `depthAligned`: D2C-aligned Y16 depth frame (PRECONDITION of detectionCenterToCamera3D)
    // - `intr`: depth intrinsics
    // - `depthScale`: depthScale from sensorInfo
    // On any false return from detectionCenterToCamera3D the cached last3D is
    // left unchanged so the caller still has the previous fix to act on.
    void update(const DynalgoDetectionResult& det,
                const DynalgoFrame& depthAligned,
                const DynalgoIntrinsic& intr,
                float depthScale);

    // Time-propagate without a measurement and return the predicted bbox.
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
