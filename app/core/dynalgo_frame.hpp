// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame.hpp — SDK-neutral frame and frame-set value types.
//
// [文件说明 / File Description]
// 中文：SDK中立的帧和帧集值类型，包含DynalgoFrame、DynalgoFrameSet和DynalgoImuSample
// English: SDK-neutral frame and frame-set value types, including DynalgoFrame, DynalgoFrameSet, and DynalgoImuSample
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

// [结构体说明 / Struct Description]
// 中文：单帧数据结构，包含像素/IMU数据和元数据（格式、大小、时间戳、深度缩放）
// English: DynalgoFrame: owns pixel data + metadata for one sensor frame. Constructed by the SDK adapter at the producer boundary (copy semantics).
struct DynalgoFrame {
    DynalgoFrameType type = DynalgoFrameType::COLOR;
    DynalgoFormat format = DynalgoFormat::UNKNOWN;
    int width = 0;
    int height = 0;
    uint64_t timestampUs = 0;
    float depthScale = 1.0f;

    std::vector<uint8_t> data;

    // [方法说明 / Method Description]
    // 中文：获取像素数据大小（字节）
    // English: Convenience: size in bytes of the pixel data.
    uint32_t dataSize() const { return static_cast<uint32_t>(data.size()); }

    // [方法说明 / Method Description]
    // 中文：获取像素数据原始指针
    // English: Convenience: raw pointer to pixel data.
    const uint8_t *rawData() const { return data.empty() ? nullptr : data.data(); }

    // [属性说明 / Property Description]
    // 中文：标记此槽位是否已被setFrame()填充
    // English: True when this slot has been populated by setFrame().
    bool present = false;
};

// [结构体说明 / Struct Description]
// 中文：帧集合结构，存储一次同步捕获的所有帧，使用数组+位集合实现O(1)访问
// English: DynalgoFrameSet: collection of frames from one synchronized capture.
// Indexed by DynalgoFrameType — replaces vendor-specific FrameSet for consumers.
//
// Backed by std::array<DynalgoFrame, COUNT> (slot per DynalgoFrameType) + a presence
// bitset, replacing the previous std::map.  Hot path: getFrame is now O(1)
// array index + bitset test (was O(log N) red-black tree + per-frame heap
// allocation), setFrame is now in-place array assignment (was tree insert).
struct DynalgoFrameSet {
    // [方法说明 / Method Description]
    // 中文：根据帧类型获取帧指针，不存在返回nullptr
    // English: Get frame by type, or nullptr if absent.
    DynalgoFrame *getFrame(DynalgoFrameType type);
    const DynalgoFrame *getFrame(DynalgoFrameType type) const;

    // [方法说明 / Method Description]
    // 中文：存储帧（接管所有权）
    // English: Store a frame (takes ownership).
    void setFrame(DynalgoFrameType type, DynalgoFrame frame);

    // [方法说明 / Method Description]
    // 中文：获取帧集合中的帧数量
    // English: Number of frames in the set.
    size_t size() const { return present_.count(); }

    // [方法说明 / Method Description]
    // 中文：检查帧集合是否为空
    // English: Check if frame set is empty
    bool empty() const { return !present_.any(); }

private:
    std::array<DynalgoFrame, static_cast<size_t>(DynalgoFrameType::COUNT)> frames_;
    std::bitset<static_cast<size_t>(DynalgoFrameType::COUNT)> present_;
};

// [结构体说明 / Struct Description]
// 中文：单个IMU测量值（加速度或陀螺仪）
// English: DynalgoImuSample: single IMU measurement (accel or gyro).
struct DynalgoImuSample {
    DynalgoFrameType type;  // ACCEL or GYRO
    uint64_t timestampUs = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float temperature = 0.0f;
};

} // namespace dynalgo
