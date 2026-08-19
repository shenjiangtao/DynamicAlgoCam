// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_spec.hpp - RoboSense RS-AC1 hardware specifications.
//
// Centralized constexpr constants for sensor resolution, frame rates,
// format, and USB identifiers.  All dimensions use the generic
// Resolution / SensorSpec / DeviceSpec abstractions so adding support for
// a future model is a matter of defining a new struct constant.

#pragma once

#include "dynalgo_types.hpp"
#include <cstdint>

namespace dynalgo::rs {

// [结构体说明 / Struct Description]
// 中文: 分辨率结构体（宽、高）
// English: Resolution struct (width, height)
struct Resolution
{
    int width = 0;
    int height = 0;

    constexpr Resolution() = default;
    constexpr Resolution(int w, int h) : width(w), height(h) {}

    // [函数说明 / Function Description]
    // 中文: 检查分辨率是否有效（宽高>0）
    // English: Check if resolution is valid (width>0 && height>0)
    [[nodiscard]] constexpr bool isValid() const {
        return width > 0 && height > 0;
    }

    // [函数说明 / Function Description]
    // 中文: 计算宽高比
    // English: Calculate aspect ratio
    [[nodiscard]] constexpr float aspectRatio() const {
        return static_cast<float>(width) / static_cast<float>(height);
    }

    [[nodiscard]] constexpr bool operator==(const Resolution& other) const {
        return width == other.width && height == other.height;
    }

    [[nodiscard]] constexpr bool operator!=(const Resolution& other) const {
        return !(*this == other);
    }
};

// [结构体说明 / Struct Description]
// 中文: 传感器规格（分辨率、帧率、格式）
// English: Sensor spec (resolution, fps, format)
struct SensorSpec
{
    Resolution resolution;
    int fps = 0;
    DynalgoFormat format = DynalgoFormat::UNKNOWN;

    constexpr SensorSpec() = default;
    constexpr SensorSpec(const Resolution& res, int frameRate, DynalgoFormat fmt)
    : resolution(res), fps(frameRate), format(fmt) {}

    // [函数说明 / Function Description]
    // 中文: 检查传感器规格是否有效
    // English: Check if sensor spec is valid
    [[nodiscard]] constexpr bool isValid() const {
        return resolution.isValid() && fps > 0 && format != DynalgoFormat::UNKNOWN;
    }
};

// [结构体说明 / Struct Description]
// 中文: USB 标识符（VID、PID）
// English: USB identifier (VID, PID)
struct UsbId
{
    uint16_t vid = 0;
    uint16_t pid = 0;

    constexpr UsbId() = default;
    constexpr UsbId(uint16_t vendor, uint16_t product) : vid(vendor), pid(product) {}

    // [函数说明 / Function Description]
    // 中文: 检查 USB ID 是否有效
    // English: Check if USB ID is valid
    [[nodiscard]] constexpr bool isValid() const {
        return vid != 0 && pid != 0;
    }
};

// ---------------------------------------------------------------------------
// RoboSense AC1 (RS-AC1) specifications
// ---------------------------------------------------------------------------
struct AC1
{
    // USB identifiers / USB 标识符
    static constexpr UsbId USB_ID{ 0x3840, 0x1010 };

    // Color sensor / 颜色传感器
    static constexpr SensorSpec COLOR{ { 1920, 1080 }, 30, DynalgoFormat::NV12 };

    // Depth sensor (shares resolution with color on AC1) / 深度传感器（AC1 上与颜色共享分辨率）
    static constexpr SensorSpec DEPTH{ { 1920, 1080 }, 10, DynalgoFormat::Y16 };

    // Depth scale: raw depth value -> millimeters / 深度比例：原始深度值 -> 毫米
    static constexpr float DEPTH_SCALE = 0.1f;

    // IMU / IMU 帧率
    static constexpr int IMU_FPS = 100;

    // Convenience: total pixels for sanity-checks / 便利：总像素用于合理性检查
    static constexpr int COLOR_PIXELS = COLOR.resolution.width * COLOR.resolution.height;
    static constexpr int DEPTH_PIXELS = DEPTH.resolution.width * DEPTH.resolution.height;

    // [函数说明 / Function Description]
    // 中文: 验证 AC1 规格是否有效
    // English: Verify AC1 spec is valid
    [[nodiscard]] static constexpr bool verify() {
        return USB_ID.isValid() && COLOR.isValid() && DEPTH.isValid() && DEPTH_SCALE > 0.0f && IMU_FPS > 0 &&
               COLOR.resolution == DEPTH.resolution; // AC1: color & depth aligned
    }
};

} // namespace dynalgo::rs
