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

class DynalgoEngagementLoop
{
public:
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

    // Called once per fused frame-set from the capture pipeline.
    // frameSet must contain a D2C-aligned depth frame (Y16) at index 0 or 1.
    void onFrame(const DynalgoFrameSet& frameSet);

    // Request graceful shutdown (sets internal stop flag; onFrame becomes no-op).
    void stop() { stop_ = true; }

    // Current state for external logging / HMI.
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

    void transitionTo(State newState);
    bool tryFire(const DynalgoDetectionResult& det);
    bool depthFrameFromSet(const DynalgoFrameSet& fs, DynalgoFrame& outDepth) const;
};

} // namespace dynalgo