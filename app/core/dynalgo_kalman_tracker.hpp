// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_kalman_tracker.hpp — Single-target Kalman filter for 2D bounding-box
// trajectory smoothing and one-step position prediction.
//
// [文件说明 / File Description]
// 中文：单目标卡尔曼滤波器，用于2D边界框轨迹平滑和单步位置预测
// English: Single-target Kalman filter for 2D bounding-box trajectory smoothing and one-step position prediction
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

// [类说明 / Class Description]
// 中文：单目标卡尔曼滤波器，用于2D边界框轨迹平滑和位置预测
// English: Single-target Kalman filter for 2D bounding-box trajectory smoothing and position prediction
class DynalgoKalmanTracker
{
public:
    DynalgoKalmanTracker();

    // [方法说明 / Method Description]
    // 中文：用测量值初始化状态，可选；如果跳过，第一次update()调用会自动初始化
    // English: Seed the state with a measurement. Optional; if skipped the first update() call auto-initialises.
    void init(const DynalgoDetectionResult& det);

    // [方法说明 / Method Description]
    // 中文：合并测量值并返回平滑后的边界框，首次调用时自动初始化
    // English: Incorporate a measurement and return the smoothed bounding box. On first call, auto-initialises.
    DynalgoDetectionResult update(const DynalgoDetectionResult& det);

    // [方法说明 / Method Description]
    // 中文：在没有测量值的情况下时间传播状态（先验预测），返回下一帧的预测边界框
    // English: Time-propagate the state without measurement (a-priori prediction) and return predicted bounding box.
    DynalgoDetectionResult predict();

    // [方法说明 / Method Description]
    // 中文：检查是否已初始化
    // English: Check if tracker is initialised
    bool initialised() const { return initialised_; }

private:
    // [私有成员 / Private Members]
    // 中文：状态向量和协方差，使用固定大小数组保持无依赖（无Eigen）
    // English: State vector and covariance, using fixed-size arrays for dependency-free header (no Eigen)
    std::array<double, 6>  x_;          // state estimate
    std::array<double, 36> P_;          // 6x6 covariance, row-major

    bool initialised_ = false;

    // [可调参数 / Tunable Parameters]
    // 中文：可调参数，暴露为公共成员以便调用者调整，构造函数中设置合理默认值
    // English: Tunables, exposed as public members for caller tweaking, reasonable defaults in constructor
    double dt_           = 1.0;          // time step (frames)
    double processNoise_ = 1.0;          // Q scaling
    double measNoise_   = 1.0;           // R scaling
};

} // namespace dynalgo
