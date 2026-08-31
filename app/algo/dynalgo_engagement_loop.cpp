// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_engagement_loop.cpp — Engagement loop implementation.

#include "dynalgo_engagement_loop.hpp"
#include "dynalgo_detection_to_3d.hpp"
#include "dynalgo_log.hpp"

#include <algorithm>

namespace dynalgo {

// [构造函数说明 / Constructor Description]
// 中文: 初始化交互循环
// English: Initialize engagement loop
DynalgoEngagementLoop::DynalgoEngagementLoop(const Config& cfg,
                                              DynalgoModelBackend* model,
                                              DynalgoActuator* actuator,
                                              const DynalgoIntrinsic& depthIntr,
                                              float depthScale)
    : cfg_(cfg), model_(model), actuator_(actuator), depthIntr_(depthIntr), depthScale_(depthScale)
{
    if (model_)
        DYNALGO_LOG_INFO_S("[engage] model backend: " << model_->name());
    else
        DYNALGO_LOG_WARN_S("[engage] no model backend — inference will be no-op");

    if (actuator_) {
        DYNALGO_LOG_INFO_S("[engage] actuator: " << actuator_->name() << " dryRun=" << (actuator_->config().dryRun ? "on" : "off"));
        if (!actuator_->load(actuator_->config()))
            DYNALGO_LOG_WARN_S("[engage] actuator load() returned false");
        if (!actuator_->open())
            DYNALGO_LOG_WARN_S("[engage] actuator open() returned false");
    } else {
        DYNALGO_LOG_WARN_S("[engage] no actuator — firing will be no-op");
    }
}

// [方法说明 / Method Description]
// 中文: 处理每帧数据
// English: Process each frame
void DynalgoEngagementLoop::onFrame(const DynalgoFrameSet& frameSet)
{
    if (stop_)
        return;

    // 1) Perceive: run model inference if available
    std::vector<DynalgoDetectionResult> detections;
    if (model_) {
        // Use color frame for inference (model expects RGB)
        if (auto* colorFrame = frameSet.getFrame(DynalgoFrameType::COLOR))
            model_->infer(*colorFrame, detections);
    }

    // 2) Locate: pick a target among detections
    std::optional<DynalgoDetectionResult> target;
    if (!detections.empty()) {
        // Precompute Z for NEAREST_DEPTH strategy
        std::vector<float> zMeters;
        if (cfg_.selectorStrategy == SelectorStrategy::NEAREST_DEPTH) {
            DynalgoFrame depthAligned;
            if (depthFrameFromSet(frameSet, depthAligned)) {
                zMeters.reserve(detections.size());
                for (const auto& det : detections) {
                    float X, Y, Z;
                    if (detectionCenterToCamera3D(depthAligned, depthIntr_, depthScale_, 2, det, X, Y, Z))
                        zMeters.push_back(Z);
                    else
                        zMeters.push_back(std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
        target = pickTarget(detections, cfg_.selectorStrategy, zMeters.empty() ? nullptr : &zMeters);
    }

    // 3) Estimate / State machine
    bool hasTarget = target.has_value();
    DynalgoFrame depthAligned;
    bool hasDepth = depthFrameFromSet(frameSet, depthAligned);

    switch (state_) {
    case State::IDLE:
        if (hasTarget) {
            consecutiveDetections_ = 1;
            transitionTo(State::LOCKING);
            bundle_.init(*target);
            if (hasDepth)
                bundle_.update(*target, depthAligned, depthIntr_, depthScale_);
        }
        break;

    case State::LOCKING:
        if (hasTarget) {
            ++consecutiveDetections_;
            consecutiveMisses_ = 0;
            bundle_.update(*target, depthAligned, depthIntr_, depthScale_);
            if (consecutiveDetections_ >= cfg_.lockingFramesRequired)
                transitionTo(State::TRACKING);
        } else {
            consecutiveMisses_ = 0; // LOCKING tolerates misses; just don't advance
            // stay in LOCKING, keep last bundle_ state (predicted)
        }
        break;

    case State::TRACKING:
        if (hasTarget) {
            consecutiveDetections_++;
            consecutiveMisses_ = 0;
            bundle_.update(*target, depthAligned, depthIntr_, depthScale_);
            // Attempt fire if we have a 3D fix
            if (bundle_.hasFix())
                tryFire(*target);
        } else {
            consecutiveDetections_ = 0;
            ++consecutiveMisses_;
            // Predict without measurement
            bundle_.predict();
            if (consecutiveMisses_ >= cfg_.lostFramesAllowed)
                transitionTo(State::LOST);
        }
        break;

    case State::FIRING:
        // FIRING is a transient sub-state; we immediately return to TRACKING
        // after fire() (with cooldown enforced in tryFire). If we lost target
        // during FIRING, fall through to LOST logic next frame.
        if (hasTarget) {
            consecutiveDetections_++;
            consecutiveMisses_ = 0;
            bundle_.update(*target, depthAligned, depthIntr_, depthScale_);
            if (bundle_.hasFix())
                tryFire(*target); // respects cooldown
            if (state_ == State::FIRING)
                transitionTo(State::TRACKING);
        } else {
            consecutiveDetections_ = 0;
            ++consecutiveMisses_;
            bundle_.predict();
            if (consecutiveMisses_ >= cfg_.lostFramesAllowed)
                transitionTo(State::LOST);
            else
                transitionTo(State::TRACKING);
        }
        break;

    case State::LOST:
        // No target, no depth processing. Return to IDLE.
        transitionTo(State::IDLE);
        break;
    }
}

// [方法说明 / Method Description]
// 中文: 从帧集中提取深度帧
// English: Extract depth frame from frame set
bool DynalgoEngagementLoop::depthFrameFromSet(const DynalgoFrameSet& fs, DynalgoFrame& outDepth) const
{
    // Convention: fused frame-set from CaptureSession puts D2C-aligned depth at DEPTH type.
    if (auto* depthFrame = fs.getFrame(DynalgoFrameType::DEPTH)) {
        outDepth = *depthFrame;
        return true;
    }
    return false;
}

// [方法说明 / Method Description]
// 中文: 执行状态转换
// English: Perform state transition
void DynalgoEngagementLoop::transitionTo(State newState)
{
    if (newState == state_)
        return;

    static const char* names[] = { "IDLE", "LOCKING", "TRACKING", "FIRING", "LOST" };
    DYNALGO_LOG_INFO_S("[engage] state: " << names[static_cast<int>(state_)] << " → " << names[static_cast<int>(newState)]);

    state_ = newState;

    if (newState == State::IDLE) {
        consecutiveDetections_ = 0;
        consecutiveMisses_ = 0;
        bundle_ = DynalgoTrackBundle(); // reset
    }
}

// [方法说明 / Method Description]
// 中文: 尝试执行射击动作
// English: Attempt to fire the actuator
bool DynalgoEngagementLoop::tryFire(const DynalgoDetectionResult& det)
{
    if (!actuator_ || !bundle_.hasFix())
        return false;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFireTime_).count();
    if (elapsed < cfg_.fireCooldownMs)
        return false;

    // Aim at cached 3D fix (camera-centric XYZ metres)
    if (!actuator_->aimAt(bundle_.lastX(), bundle_.lastY(), bundle_.lastZ())) {
        DYNALGO_LOG_WARN_S("[engage] actuator aimAt() returned false");
        return false;
    }

    // Fire for a short default duration (configurable later if needed)
    if (!actuator_->fire(10)) {
        DYNALGO_LOG_WARN_S("[engage] actuator fire() returned false");
        return false;
    }

    lastFireTime_ = now;
    transitionTo(State::FIRING);
    return true;
}

} // namespace dynalgo