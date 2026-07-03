// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_rs_spec.hpp - RoboSense RS-AC1 hardware specifications.
//
// Centralized constexpr constants for sensor resolution, frame rates,
// format, and USB identifiers.  All dimensions use the generic
// Resolution / SensorSpec / DeviceSpec abstractions so adding support for
// a future model is a matter of defining a new struct constant.

#pragma once

#include "nio_types.hpp"
#include <cstdint>

namespace nio::rs {

// ---------------------------------------------------------------------------
// Generic helpers
// ---------------------------------------------------------------------------
struct Resolution
{
    int width = 0;
    int height = 0;

    constexpr Resolution() = default;
    constexpr Resolution(int w, int h) : width(w), height(h) {}

    [[nodiscard]] constexpr bool isValid() const {
        return width > 0 && height > 0;
    }

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

struct SensorSpec
{
    Resolution resolution;
    int fps = 0;
    NioFormat format = NioFormat::UNKNOWN;

    constexpr SensorSpec() = default;
    constexpr SensorSpec(const Resolution& res, int frameRate, NioFormat fmt)
    : resolution(res), fps(frameRate), format(fmt) {}

    [[nodiscard]] constexpr bool isValid() const {
        return resolution.isValid() && fps > 0 && format != NioFormat::UNKNOWN;
    }
};

struct UsbId
{
    uint16_t vid = 0;
    uint16_t pid = 0;

    constexpr UsbId() = default;
    constexpr UsbId(uint16_t vendor, uint16_t product) : vid(vendor), pid(product) {}

    [[nodiscard]] constexpr bool isValid() const {
        return vid != 0 && pid != 0;
    }
};

// ---------------------------------------------------------------------------
// RoboSense AC1 (RS-AC1) specifications
// ---------------------------------------------------------------------------
struct AC1
{
    // USB identifiers
    static constexpr UsbId USB_ID{ 0x3840, 0x1010 };

    // Color sensor
    static constexpr SensorSpec COLOR{ { 1920, 1080 }, 30, NioFormat::NV12 };

    // Depth sensor (shares resolution with color on AC1)
    static constexpr SensorSpec DEPTH{ { 1920, 1080 }, 10, NioFormat::Y16 };

    // Depth scale: raw depth value -> millimeters
    static constexpr float DEPTH_SCALE = 0.1f;

    // IMU
    static constexpr int IMU_FPS = 100;

    // Convenience: total pixels for sanity-checks
    static constexpr int COLOR_PIXELS = COLOR.resolution.width * COLOR.resolution.height;
    static constexpr int DEPTH_PIXELS = DEPTH.resolution.width * DEPTH.resolution.height;

    [[nodiscard]] static constexpr bool verify() {
        return USB_ID.isValid() && COLOR.isValid() && DEPTH.isValid() && DEPTH_SCALE > 0.0f && IMU_FPS > 0 &&
               COLOR.resolution == DEPTH.resolution; // AC1: color & depth aligned
    }
};

} // namespace nio::rs
