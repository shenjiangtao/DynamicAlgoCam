// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// bytetrack.cpp — ByteTrack C++ implementation.

#include "bytetrack.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace dynalgo {

// KalmanFilter implementation
KalmanFilter::KalmanFilter() {
    // State transition matrix (constant velocity model)
    // [x, y, w, h, vx, vy]
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            motionMat[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    // Position updated by velocity
    motionMat[0][4] = 1.0f; // x += vx
    motionMat[1][5] = 1.0f; // y += vy
    // w, h constant velocity (optional, typically 0)
    motionMat[2][2] = 1.0f;
    motionMat[3][3] = 1.0f;

    // Measurement matrix: observe [x, y, w, h]
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 6; ++j) {
            updateMat[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void KalmanFilter::initiate(const float* measurement) {
    // Not used directly, state is initialized in createTrack
}

void KalmanFilter::predict(float* state, float cov[6][6]) {
    // State prediction: x' = F * x
    float newState[6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            newState[i] += motionMat[i][j] * state[j];
        }
    }
    for (int i = 0; i < 6; ++i) state[i] = newState[i];

    // Covariance prediction: P' = F * P * F^T + Q
    float newCov[6][6] = {{0}};
    float FT[6][6];
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) FT[i][j] = motionMat[j][i];

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0;
            for (int k = 0; k < 6; ++k) {
                for (int l = 0; l < 6; ++l) {
                    sum += motionMat[i][k] * cov[k][l] * FT[l][j];
                }
            }
            newCov[i][j] = sum;
        }
    }

    // Process noise Q
    for (int i = 0; i < 6; ++i) {
        float std = (i < 4) ? stdWeightPosition : stdWeightVelocity;
        newCov[i][i] += std * std;
    }

    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) cov[i][j] = newCov[i][j];
}

void KalmanFilter::update(const float* measurement, float* state, float cov[6][6]) {
    // Innovation: y = z - H * x
    float innovation[4];
    for (int i = 0; i < 4; ++i) {
        innovation[i] = measurement[i];
        for (int j = 0; j < 6; ++j) {
            innovation[i] -= updateMat[i][j] * state[j];
        }
    }

    // Innovation covariance: S = H * P * H^T + R
    float S[4][4] = {{0}};
    float HT[6][4];
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 6; ++j) HT[i][j] = updateMat[j][i];

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0;
            for (int k = 0; k < 6; ++k) {
                for (int l = 0; l < 6; ++l) {
                    sum += updateMat[i][k] * cov[k][l] * HT[l][j];
                }
            }
            S[i][j] = sum;
        }
    }

    // Measurement noise R
    for (int i = 0; i < 4; ++i) {
        float std = stdWeightPosition * (i < 2 ? state[i] : state[i]); // rough scale
        S[i][i] += std * std + 1e-2f;
    }

    // Kalman gain: K = P * H^T * S^-1
    // Simplified: solve S * K^T = H * P for K
    float K[6][4] = {{0}};

    // Invert S (4x4) using simple method (for small matrices)
    float SInv[4][4];
    // Use pseudo-inverse via eigenvalue decomposition or simple approach
    // For simplicity, use diagonal approximation
    for (int i = 0; i < 4; ++i) {
        SInv[i][i] = (S[i][i] > 1e-6f) ? 1.0f / S[i][i] : 0.0f;
        for (int j = 0; j < 4; ++j) if (i != j) SInv[i][j] = 0.0f;
    }

    // K = P * H^T * S^-1
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0;
            for (int k = 0; k < 6; ++k) {
                sum += cov[i][k] * HT[j][k]; // H^T = HT
            }
            for (int l = 0; l < 4; ++l) {
                K[i][j] += sum * SInv[l][j];
            }
        }
    }

    // Update state: x = x + K * y
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 4; ++j) {
            state[i] += K[i][j] * innovation[j];
        }
    }

    // Update covariance: P = (I - K * H) * P
    float I_KH[6][6] = {{0}};
    for (int i = 0; i < 6; ++i) I_KH[i][i] = 1.0f;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 4; ++k) {
                I_KH[i][j] -= K[i][k] * updateMat[k][j];
            }
        }
    }

    float newCov[6][6] = {{0}};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                newCov[i][j] += I_KH[i][k] * cov[k][j];
            }
        }
    }
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) cov[i][j] = newCov[i][j];
}

float KalmanFilter::gatingDistance(const float* measurement,
                                    const float* state, const float cov[6][6]) {
    // Mahalanobis distance: (z - Hx)^T S^-1 (z - Hx)
    float innovation[4];
    for (int i = 0; i < 4; ++i) {
        innovation[i] = measurement[i];
        for (int j = 0; j < 6; ++j) {
            innovation[i] -= updateMat[i][j] * state[j];
        }
    }

    // Simplified: use diagonal of S
    float dist = 0;
    for (int i = 0; i < 4; ++i) {
        float var = cov[i][i] + stdWeightPosition * stdWeightPosition;
        if (var > 1e-6f) dist += innovation[i] * innovation[i] / var;
    }
    return dist;
}

// ByteTrack implementation
ByteTrack::ByteTrack(const ByteTrackConfig& cfg) : cfg_(cfg) {}
ByteTrack::~ByteTrack() = default;

float ByteTrack::iou(const DynalgoDetectionResult& a, const DynalgoDetectionResult& b) const {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.w, b.x + b.w);
    float y2 = std::min(a.y + a.h, b.y + b.h);

    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float areaA = a.w * a.h;
    float areaB = b.w * b.h;
    return inter / (areaA + areaB - inter + 1e-6f);
}

float ByteTrack::iouTrackDet(const Track& track, const DynalgoDetectionResult& det) const {
    float x1 = std::max(track.x, det.x);
    float y1 = std::max(track.y, det.y);
    float x2 = std::min(track.x + track.w, det.x + det.w);
    float y2 = std::min(track.y + track.h, det.y + det.h);

    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float areaT = track.w * track.h;
    float areaD = det.w * det.h;
    return inter / (areaT + areaD - inter + 1e-6f);
}

void ByteTrack::createTrack(const DynalgoDetectionResult& det, float depth) {
    Track t;
    t.trackId = nextTrackId_++;
    t.state = TrackState::NEW;
    t.detection = det;
    t.x = det.x;
    t.y = det.y;
    t.w = det.w;
    t.h = det.h;
    t.z = depth;
    t.age = 1;
    t.hitStreak = 1;
    t.timeSinceUpdate = 0;

    // Initialize Kalman state: [x, y, w, h, vx, vy]
    float cx = det.x + det.w * 0.5f;
    float cy = det.y + det.h * 0.5f;
    t.kfState[0] = cx;
    t.kfState[1] = cy;
    t.kfState[2] = det.w;
    t.kfState[3] = det.h;
    t.kfState[4] = 0.0f;
    t.kfState[5] = 0.0f;

    // Initial covariance
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            t.kfCov[i][j] = (i == j) ? 10.0f : 0.0f;
        }
    }

    tracks_.push_back(t);
}

std::vector<std::pair<int, int>> ByteTrack::associateFirst(
    const std::vector<DynalgoDetectionResult>& highConfDets,
    const std::vector<int>& trackIndices) {

    std::vector<std::pair<int, int>> matches;

    // Build cost matrix (1 - IoU)
    std::vector<std::vector<float>> cost(trackIndices.size(),
        std::vector<float>(highConfDets.size(), 1.0f));

    for (size_t i = 0; i < trackIndices.size(); ++i) {
        const Track& track = tracks_[trackIndices[i]];
        for (size_t j = 0; j < highConfDets.size(); ++j) {
            float iouVal = iouTrackDet(track, highConfDets[j]);
            cost[i][j] = 1.0f - iouVal;
        }
    }

    // Simple greedy matching (Hungarian would be better but more complex)
    std::vector<bool> trackUsed(trackIndices.size(), false);
    std::vector<bool> detUsed(highConfDets.size(), false);

    for (size_t iter = 0; iter < std::min(trackIndices.size(), highConfDets.size()); ++iter) {
        float bestCost = 1.0f;
        int bestTrack = -1, bestDet = -1;

        for (size_t i = 0; i < trackIndices.size(); ++i) {
            if (trackUsed[i]) continue;
            for (size_t j = 0; j < highConfDets.size(); ++j) {
                if (detUsed[j]) continue;
                if (cost[i][j] < bestCost && cost[i][j] < (1.0f - cfg_.matchThresh)) {
                    bestCost = cost[i][j];
                    bestTrack = i;
                    bestDet = j;
                }
            }
        }

        if (bestTrack >= 0) {
            matches.emplace_back(trackIndices[bestTrack], bestDet);
            trackUsed[bestTrack] = true;
            detUsed[bestDet] = true;
        } else {
            break;
        }
    }

    return matches;
}

std::vector<std::pair<int, int>> ByteTrack::associateSecond(
    const std::vector<DynalgoDetectionResult>& lowConfDets,
    const std::vector<int>& unmatchedTrackIndices) {

    std::vector<std::pair<int, int>> matches;

    for (int trackIdx : unmatchedTrackIndices) {
        Track& track = tracks_[trackIdx];
        if (track.state != TrackState::TRACKED) continue;

        float bestIou = 0.0f;
        int bestDet = -1;

        for (size_t j = 0; j < lowConfDets.size(); ++j) {
            float iouVal = iouTrackDet(track, lowConfDets[j]);
            if (iouVal > bestIou && iouVal > cfg_.matchThresh) {
                bestIou = iouVal;
                bestDet = j;
            }
        }

        if (bestDet >= 0) {
            matches.emplace_back(trackIdx, bestDet);
        }
    }

    return matches;
}

std::vector<Track> ByteTrack::update(const std::vector<DynalgoDetectionResult>& detections,
                                     const float* depthMap) {
    // Separate high/low confidence detections
    std::vector<DynalgoDetectionResult> highConfDets, lowConfDets;
    std::vector<int> highConfIndices, lowConfIndices;

    for (size_t i = 0; i < detections.size(); ++i) {
        if (detections[i].score >= cfg_.trackThresh) {
            highConfDets.push_back(detections[i]);
            highConfIndices.push_back(i);
        } else if (detections[i].score >= cfg_.lowThresh) {
            lowConfDets.push_back(detections[i]);
            lowConfIndices.push_back(i);
        }
    }

    // Get active track indices
    std::vector<int> activeTrackIndices, lostTrackIndices;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i].state == TrackState::TRACKED) {
            activeTrackIndices.push_back(i);
        } else if (tracks_[i].state == TrackState::LOST) {
            lostTrackIndices.push_back(i);
        }
    }

    // Predict all tracked tracks
    for (int idx : activeTrackIndices) {
        Track& t = tracks_[idx];
        kf_.predict(t.kfState, t.kfCov);
        // Update bbox from Kalman state (center format)
        t.x = t.kfState[0] - t.kfState[2] * 0.5f;
        t.y = t.kfState[1] - t.kfState[3] * 0.5f;
        t.w = t.kfState[2];
        t.h = t.kfState[3];
    }

    // First association: high-conf detections with tracked tracks
    auto matches1 = associateFirst(highConfDets, activeTrackIndices);

    std::vector<bool> detMatched1(highConfDets.size(), false);
    std::vector<bool> trackMatched1(tracks_.size(), false);

    for (auto& m : matches1) {
        int trackIdx = m.first;
        int detIdx = m.second;
        Track& t = tracks_[trackIdx];
        const auto& det = highConfDets[detIdx];

        // Kalman update
        float measurement[4] = {det.x + det.w * 0.5f, det.y + det.h * 0.5f, det.w, det.h};
        kf_.update(measurement, t.kfState, t.kfCov);

        // Update bbox from Kalman
        t.x = t.kfState[0] - t.kfState[2] * 0.5f;
        t.y = t.kfState[1] - t.kfState[3] * 0.5f;
        t.w = t.kfState[2];
        t.h = t.kfState[3];

        t.detection = det;
        t.state = TrackState::TRACKED;
        t.hitStreak++;
        t.timeSinceUpdate = 0;
        t.age++;

        if (depthMap && detIdx < static_cast<int>(highConfIndices.size())) {
            t.z = depthMap[highConfIndices[detIdx]];
        }

        detMatched1[detIdx] = true;
        trackMatched1[trackIdx] = true;
    }

    // Find unmatched tracks for second association
    std::vector<int> unmatchedTracks;
    for (int idx : activeTrackIndices) {
        if (!trackMatched1[idx]) unmatchedTracks.push_back(idx);
    }

    // Second association: low-conf detections with unmatched tracked tracks
    if (cfg_.useByteMatch && !lowConfDets.empty() && !unmatchedTracks.empty()) {
        auto matches2 = associateSecond(lowConfDets, unmatchedTracks);

        std::vector<bool> detMatched2(lowConfDets.size(), false);
        for (auto& m : matches2) {
            int trackIdx = m.first;
            int detIdx = m.second;
            Track& t = tracks_[trackIdx];
            const auto& det = lowConfDets[detIdx];

            // Kalman update with low-conf detection
            float measurement[4] = {det.x + det.w * 0.5f, det.y + det.h * 0.5f, det.w, det.h};
            kf_.update(measurement, t.kfState, t.kfCov);

            t.x = t.kfState[0] - t.kfState[2] * 0.5f;
            t.y = t.kfState[1] - t.kfState[3] * 0.5f;
            t.w = t.kfState[2];
            t.h = t.kfState[3];

            t.detection = det;
            t.state = TrackState::TRACKED;
            t.hitStreak++;
            t.timeSinceUpdate = 0;
            t.age++;

            detMatched2[detIdx] = true;
        }
    }

    // Handle unmatched tracks
    for (int idx : activeTrackIndices) {
        if (!trackMatched1[idx]) {
            Track& t = tracks_[idx];
            t.timeSinceUpdate++;
            if (t.timeSinceUpdate > cfg_.trackBuffer) {
                t.state = TrackState::REMOVED;
            } else {
                t.state = TrackState::LOST;
            }
        }
    }

    // Create new tracks from unmatched high-conf detections
    for (size_t j = 0; j < highConfDets.size(); ++j) {
        if (detMatched1[j]) continue;
        float depth = 0;
        if (depthMap) depth = depthMap[highConfIndices[j]];
        createTrack(highConfDets[j], depth);
    }

    // Remove dead tracks
    removeDeadTracks();

    // Return active tracks
    return getActiveTracks();
}

void ByteTrack::removeDeadTracks() {
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [](const Track& t) { return t.state == TrackState::REMOVED; }),
        tracks_.end());
}

std::vector<Track> ByteTrack::getActiveTracks() const {
    std::vector<Track> active;
    for (const auto& t : tracks_) {
        if (t.state == TrackState::TRACKED) {
            active.push_back(t);
        }
    }
    return active;
}

void ByteTrack::reset() {
    tracks_.clear();
    nextTrackId_ = 1;
}

} // namespace dynalgo