// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame.hpp — SDK-neutral frame and frame-set value types.
//
// DynalgoFrame: owns a copy of pixel/IMU data + metadata (format, size,
// timestamp, depth scale).  Extracted from SDK frames at the producer
// boundary (ObFrameAdapter), so downstream consumers are SDK-agnostic.
//
// DynalgoFrameSet: collection of DynalgoFrames indexed by DynalgoFrameType — the
// unit that travels through VideoFrameQueue to the consumer thread.
//
// DynalgoImuSample: single IMU measurement (accel or gyro) — the unit
// that travels through ImuFrameQueue.

#pragma once

#include "dynalgo_types.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <memory>
#include <vector>

namespace dynalgo {

// DynalgoFrame: owns pixel data + metadata for one sensor frame.
// Constructed by the SDK adapter at the producer boundary (copy semantics).
struct DynalgoFrame {
    DynalgoFrameType type = DynalgoFrameType::COLOR;
    DynalgoFormat format = DynalgoFormat::UNKNOWN;
    int width = 0;
    int height = 0;
    uint64_t timestampUs = 0;
    float depthScale = 1.0f;

    std::vector<uint8_t> data;

    // Convenience: size in bytes of the pixel data.
    uint32_t dataSize() const { return static_cast<uint32_t>(data.size()); }

    // Convenience: raw pointer to pixel data.
    const uint8_t *rawData() const { return data.empty() ? nullptr : data.data(); }

    // True when this slot has been populated by setFrame().
    bool present = false;
};

// DynalgoFrameSet: collection of frames from one synchronized capture.
// Indexed by DynalgoFrameType — replaces vendor-specific FrameSet for consumers.
//
// Backed by std::array<DynalgoFrame, COUNT> (slot per DynalgoFrameType) + a presence
// bitset, replacing the previous std::map.  Hot path: getFrame is now O(1)
// array index + bitset test (was O(log N) red-black tree + per-frame heap
// allocation), setFrame is now in-place array assignment (was tree insert).
struct DynalgoFrameSet {
    // Get frame by type, or nullptr if absent.
    DynalgoFrame *getFrame(DynalgoFrameType type);
    const DynalgoFrame *getFrame(DynalgoFrameType type) const;

    // Store a frame (takes ownership).
    void setFrame(DynalgoFrameType type, DynalgoFrame frame);

    // Number of frames in the set.
    size_t size() const { return present_.count(); }

    bool empty() const { return !present_.any(); }

private:
    std::array<DynalgoFrame, static_cast<size_t>(DynalgoFrameType::COUNT)> frames_;
    std::bitset<static_cast<size_t>(DynalgoFrameType::COUNT)> present_;
};

// DynalgoImuSample: single IMU measurement (accel or gyro).
struct DynalgoImuSample {
    DynalgoFrameType type;  // ACCEL or GYRO
    uint64_t timestampUs = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float temperature = 0.0f;
};

} // namespace dynalgo
