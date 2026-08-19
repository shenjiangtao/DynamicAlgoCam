// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_engagement_loop.hpp — Perceive → Locate → Estimate → Control loop
// orchestrating ModelBackend, TargetSelector, TrackBundle, and Actuator.
//
// State machine: IDLE → LOCKING → TRACKING → FIRING → LOST → IDLE
// - LOCKING→TRACKING: 3 consecutive frames with valid detection
// - TRACKING→LOST:    5 consecutive frames without detection
// - FIRING cooldown:  1000 ms between fire() calls
// - Only when --engage-* flags are provided.

#pragma once

#include "dynalgo_actuator.hpp"
#include "dynalgo_model.hpp"
#include "dynalgo_target_selector.hpp"
#include "dynalgo_track_bundle.hpp"
#include "dynalgo_types.hpp"

#include <chrono>
#include <memory>
#include <optional>

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 交互循环，协调模型后端、目标选择器、轨迹包和执行器
// English: Engagement loop orchestrating ModelBackend, TargetSelector, TrackBundle, and Actuator
class DynalgoEngagementLoop
{
public:
    // [结构体说明 / Struct Description]
    // 中文: 交互循环配置参数
    // English: Engagement loop configuration parameters
    struct Config
    {
        int lockingFramesRequired = 3;     // consecutive detections to enter TRACKING
        int lostFramesAllowed   = 5;       // consecutive misses to enter LOST
        int fireCooldownMs      = 1000;    // minimum interval between fire() calls
        SelectorStrategy selectorStrategy = SelectorStrategy::HIGHEST_SCORE;
    };

    // All pointers must remain valid for the lifetime of this object.
    // model/actuator may be nullptr (treated as no-op).
    DynalgoEngagementLoop(const Config& cfg,
                          DynalgoModelBackend* model,
                          DynalgoActuator* actuator,
                          const DynalgoIntrinsic& depthIntr,
                          float depthScale);

    ~DynalgoEngagementLoop() = default;

    // [方法说明 / Method Description]
    // 中文: 每帧融合帧集调用一次
    // English: Called once per fused frame-set from the capture pipeline
    void onFrame(const DynalgoFrameSet& frameSet);

    // [方法说明 / Method Description]
    // 中文: 请求优雅关闭
    // English: Request graceful shutdown
    void stop() { stop_ = true; }

    // [枚举说明 / Enum Description]
    // 中文: 交互循环状态
    // English: Engagement loop states
    enum class State { IDLE, LOCKING, TRACKING, FIRING, LOST };
    State state() const { return state_; }

private:

    const Config cfg_;
    DynalgoModelBackend* const model_;
    DynalgoActuator* const actuator_;
    const DynalgoIntrinsic& depthIntr_;
    const float depthScale_;

    DynalgoTrackBundle bundle_;
    std::optional<DynalgoDetectionResult> lastDetection_;
    int consecutiveDetections_ = 0;
    int consecutiveMisses_ = 0;
    std::chrono::steady_clock::time_point lastFireTime_;
    State state_ = State::IDLE;
    bool stop_ = false;

    // [方法说明 / Method Description]
    // 中文: 状态转换处理
    // English: Handle state transition
    void transitionTo(State newState);
    // [方法说明 / Method Description]
    // 中文: 尝试执行射击动作
    // English: Attempt to fire the actuator
    bool tryFire(const DynalgoDetectionResult& det);
    // [方法说明 / Method Description]
    // 中文: 从帧集中提取深度帧
    // English: Extract depth frame from frame set
    bool depthFrameFromSet(const DynalgoFrameSet& fs, DynalgoFrame& outDepth) const;
};

} // namespace dynalgo