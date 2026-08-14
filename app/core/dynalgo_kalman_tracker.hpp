// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_kalman_tracker.hpp — Single-target Kalman filter for 2D bounding-box
// trajectory smoothing and one-step position prediction.
//
// State vector (6):  [cx, cy, w, h, vx, vy]   (centre x/y, w/h, velocities)
// Measurement (4):   [cx, cy, w, h]
// Motion model:      constant velocity
//   x(k+1) = F x(k) + process_noise
//   z(k)   = H x(k) + measurement_noise
//
// This is the minimal single-target tracker. Multi-target association
// (Hungarian / greedy match, track lifecycle) is intentionally NOT covered
// here — it is a separate concern to be added on top when the project wires
// up a concrete model backend. Keep this class focused on one trajectory.
//
// Usage (one tracker per tracked object):
//   dynalgo::DynalgoKalmanTracker trk;
//   trk.init(firstDetection);        // optional explicit init, else lazy on first update
//   dynalgo::DynalgoDetectionResult smoothed = trk.update(det);   // incorporate measurement
//   dynalgo::DynalgoDetectionResult predicted = trk.predict();   // propagate without measurement
//
// NOT called by the capture pipeline today; provided as infrastructure for
// future algorithm threads that will sit downstream of DynalgoModelBackend::infer().

#pragma once

#include "dynalgo_model.hpp"

#include <array>

namespace dynalgo {

class DynalgoKalmanTracker
{
public:
    DynalgoKalmanTracker();

    // Seed the state with a measurement. Optional; if skipped the first
    // update() call auto-initialises from the detection.
    void init(const DynalgoDetectionResult& det);

    // Incorporate a measurement and return the smoothed (a-posteriori)
    // bounding box. On the very first call (no init()), initialises state
    // from `det` and returns a lightly-smoothed copy of `det`.
    DynalgoDetectionResult update(const DynalgoDetectionResult& det);

    // Time-propagate the state without a measurement (a-priori prediction)
    // and return the predicted bounding box for the next frame.
    DynalgoDetectionResult predict();

    // True after init() or the first measurement-fed update().
    bool initialised() const { return initialised_; }

private:
    // State vector and covariance. Using std::array<double, N> for fixed
    // size keeps this header dependency-free (no Eigen).
    std::array<double, 6>  x_;          // state estimate
    std::array<double, 36> P_;          // 6x6 covariance, row-major

    bool initialised_ = false;

    // Tunables. Exposed as public members so callers can tweak without
    // subclassing; reasonable defaults are set in the constructor.
    double dt_           = 1.0;          // time step (frames)
    double processNoise_ = 1.0;          // Q scaling
    double measNoise_   = 1.0;           // R scaling
};

} // namespace dynalgo
