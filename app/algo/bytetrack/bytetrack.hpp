// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// bytetrack.hpp — ByteTrack C++ multi-object tracker for Dynalgo.

#pragma once

#include "dynalgo_model.hpp"
#include "dynalgo_types.hpp"

#include <vector>
#include <memory>
#include <optional>

namespace dynalgo {

// Track state
enum class TrackState {
    NEW = 0,
    TRACKED = 1,
    LOST = 2,
    REMOVED = 3
};

// Track representation
struct Track {
    int trackId = -1;
    TrackState state = TrackState::NEW;
    DynalgoDetectionResult detection; // last associated detection
    float x = 0, y = 0, w = 0, h = 0; // smoothed bbox (camera pixel coords)
    float vx = 0, vy = 0;             // velocity
    float z = 0;                      // 3D distance (meters) if available
    int age = 0;                      // frames since creation
    int hitStreak = 0;                // consecutive matches
    int timeSinceUpdate = 0;          // frames since last match

    // Kalman filter state (6D: x, y, w, h, vx, vy)
    float kfState[6] = {0};
    float kfCov[6][6] = {{0}};
};

// ByteTrack configuration
struct ByteTrackConfig {
    float trackThresh = 0.5f;      // high-confidence detection threshold
    float lowThresh = 0.1f;        // low-confidence detection threshold (for second association)
    float matchThresh = 0.8f;      // IoU threshold for matching
    float trackBuffer = 30;        // frames to keep lost tracks
    float minBoxArea = 10.0f;      // minimum box area
    bool useByteMatch = true;      // use second association with low-conf detections
    float motThresh = 0.7f;        // MOT matching threshold
};

// Kalman filter for 6D state (x, y, w, h, vx, vy)
class KalmanFilter {
public:
    KalmanFilter();
    void initiate(const float* measurement); // measurement: [x, y, w, h]
    void predict(float* state, float cov[6][6]);
    void update(const float* measurement, float* state, float cov[6][6]);
    float gatingDistance(const float* measurement, const float* state, const float cov[6][6]);

private:
    float stdWeightPosition = 1.0f / 20.0f;
    float stdWeightVelocity = 1.0f / 160.0f;
    float motionMat[6][6];  // state transition
    float updateMat[4][6];  // measurement matrix
};

// ByteTrack main class
class ByteTrack {
public:
    explicit ByteTrack(const ByteTrackConfig& cfg = ByteTrackConfig());
    ~ByteTrack();

    // Update with new detections and optional depth info
    // depthMap: optional 3D distance for each detection (same order)
    std::vector<Track> update(const std::vector<DynalgoDetectionResult>& detections,
                              const float* depthMap = nullptr);

    // Get active tracks (TRACKED state)
    std::vector<Track> getActiveTracks() const;

    // Get all tracks
    const std::vector<Track>& getAllTracks() const { return tracks_; }

    // Reset tracker
    void reset();

private:
    ByteTrackConfig cfg_;
    std::vector<Track> tracks_;
    int nextTrackId_ = 1;
    KalmanFilter kf_;

    // First association: high-conf detections with tracked tracks
    std::vector<std::pair<int, int>> associateFirst(
        const std::vector<DynalgoDetectionResult>& highConfDets,
        const std::vector<int>& trackIndices);

    // Second association: low-conf detections with unmatched tracks
    std::vector<std::pair<int, int>> associateSecond(
        const std::vector<DynalgoDetectionResult>& lowConfDets,
        const std::vector<int>& unmatchedTrackIndices);

    // IoU computation
    float iou(const DynalgoDetectionResult& a, const DynalgoDetectionResult& b) const;
    float iouTrackDet(const Track& track, const DynalgoDetectionResult& det) const;

    // Create new track from detection
    void createTrack(const DynalgoDetectionResult& det, float depth = 0);

    // Remove dead tracks
    void removeDeadTracks();
};

} // namespace dynalgo