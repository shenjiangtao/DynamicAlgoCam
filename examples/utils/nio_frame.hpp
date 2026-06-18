// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame.hpp — SDK-neutral frame and frame-set value types.
//
// NioFrame: owns a copy of pixel/IMU data + metadata (format, size,
// timestamp, depth scale).  Extracted from SDK frames at the producer
// boundary (ObFrameAdapter), so downstream consumers are SDK-agnostic.
//
// NioFrameSet: collection of NioFrames indexed by NioFrameType — the
// unit that travels through VideoFrameQueue to the consumer thread.
//
// NioImuSample: single IMU measurement (accel or gyro) — the unit
// that travels through ImuFrameQueue.

#pragma once

#include "nio_types.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace nio {

// NioFrame: owns pixel data + metadata for one sensor frame.
// Constructed by the SDK adapter at the producer boundary (copy semantics).
struct NioFrame {
    NioFrameType type = NioFrameType::COLOR;
    NioFormat format = NioFormat::UNKNOWN;
    int width = 0;
    int height = 0;
    uint64_t timestampUs = 0;
    float depthScale = 1.0f;

    std::vector<uint8_t> data;

    // Convenience: size in bytes of the pixel data.
    uint32_t dataSize() const { return static_cast<uint32_t>(data.size()); }

    // Convenience: raw pointer to pixel data.
    const uint8_t *rawData() const { return data.empty() ? nullptr : data.data(); }
};

// NioFrameSet: collection of frames from one synchronized capture.
// Indexed by NioFrameType — replaces ob::FrameSet for consumers.
struct NioFrameSet {
    // Get frame by type, or nullptr if absent.
    NioFrame *getFrame(NioFrameType type);
    const NioFrame *getFrame(NioFrameType type) const;

    // Store a frame (takes ownership).
    void setFrame(NioFrameType type, NioFrame frame);

    // Iterate all frames (non-owning).
    const std::map<NioFrameType, NioFrame> &allFrames() const { return frames_; }

    // Number of frames in the set.
    size_t size() const { return frames_.size(); }

    bool empty() const { return frames_.empty(); }

    // Optional: attached SDK-native FrameSet (type-erased shared_ptr).
    // Used for pipeline operations (e.g. ob::Align) that need the original
    // SDK frame.  Set by ObFrameAdapter; null for non-Orbbec paths.
    // Transitional — will be removed once NioAlign abstracts this.
    std::shared_ptr<void> nativeFrameSet;

private:
    std::map<NioFrameType, NioFrame> frames_;
};

// NioImuSample: single IMU measurement (accel or gyro).
struct NioImuSample {
    NioFrameType type;  // ACCEL or GYRO
    uint64_t timestampUs = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float temperature = 0.0f;
};

} // namespace nio
