// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_spec.hpp - Orbbec device specifications and configuration constants.
//
// Unlike RoboSense AC1, Orbbec cameras do not have fixed, model-specific
// resolution constants because profiles are negotiated at runtime through
// the SDK.  However, the following policy-level constants are stable enough
// to be centralized:
//
//  * Depth precision level → depth-scale mapping
//  * Preferred color format per device family
//  * Default fallback values for optional properties
//
// All constants are constexpr and compile-time evaluated.

#pragma once

#include "dynalgo_types.hpp"
#include <array>
#include <cstdint>

namespace dynalgo::orbbec {

// ---------------------------------------------------------------------------
// Depth precision level mapping
// ---------------------------------------------------------------------------
// Orbbec depth precision levels (OB_PROP_DEPTH_PRECISION_LEVEL_INT):
//   0 = 1mm, 1 = 0.5mm, 2 = 0.25mm, 3 = 0.1mm
//
// The table below maps each level to its corresponding depth scale
// (raw value -> meters).  Level count is fixed by the SDK spec.
// ---------------------------------------------------------------------------

inline constexpr float DEFAULT_DEPTH_SCALE = 0.001f;

struct DepthPrecisionLevel
{
    int level;        // OB_PROP_DEPTH_PRECISION_LEVEL_INT value
    float scale;      // raw depth value -> meters
    const char* desc; // human-readable description
};

inline constexpr std::array<DepthPrecisionLevel, 4> DEPTH_PRECISION_TABLE = { {
    { 0, 0.001f, "1.0mm" },
    { 1, 0.0005f, "0.5mm" },
    { 2, 0.00025f, "0.25mm" },
    { 3, 0.0001f, "0.1mm" },
} };

/**
 * @brief Convert Orbbec depth precision level to depth scale (meters per unit).
 * @param level The precision level from the device.
 * @return Depth scale, or DEFAULT_DEPTH_SCALE if the level is out of range.
 */
[[nodiscard]] constexpr float depthLevelToScale(int level) {
    if (level < 0 || level >= static_cast<int>(DEPTH_PRECISION_TABLE.size())) {
        return DEFAULT_DEPTH_SCALE;
    }
    return DEPTH_PRECISION_TABLE[level].scale;
}

// ---------------------------------------------------------------------------
// Color format preference policy
// ---------------------------------------------------------------------------
// Different Orbbec families prefer different native color formats.
// CaptureSession uses this to set the preferred format passed to
// selectBestProfile().
// ---------------------------------------------------------------------------

inline constexpr DynalgoFormat COLOR_FMT_DEFAULT = DynalgoFormat::MJPG;
inline constexpr DynalgoFormat COLOR_FMT_GEMINI305 = DynalgoFormat::YUYV;
inline constexpr DynalgoFormat COLOR_FMT_GEMINI335L336L = DynalgoFormat::YUYV;

/**
 * @brief Determine the preferred color format for a given Orbbec device.
 * @param isGemini305 Whether the device is identified as Gemini 305 series.
 * @param isGemini335L336L Whether the device is identified as Gemini 335L/336L.
 * @return The preferred DynalgoFormat for the color stream.
 */
[[nodiscard]] constexpr DynalgoFormat getPreferredColorFormat(bool isGemini305, bool isGemini335L336L) {
    if (isGemini305 || isGemini335L336L) {
        return COLOR_FMT_GEMINI305;
    }
    return COLOR_FMT_DEFAULT;
}

// ---------------------------------------------------------------------------
// Sensor type enablement policy
// ---------------------------------------------------------------------------
// Some device families have known limitations (e.g., Gemini 305g lacks IR_LEFT).
// These policy flags centralize those quirks.
// ---------------------------------------------------------------------------

inline constexpr bool IR_LEFT_ENABLED_DEFAULT = true;
inline constexpr bool IR_LEFT_ENABLED_GEMINI305G = false;

// ---------------------------------------------------------------------------
// Verification helpers
// ---------------------------------------------------------------------------

[[nodiscard]] constexpr bool isValidDepthScale(float scale) {
    return scale > 0.0f;
}

[[nodiscard]] constexpr bool isValidPrecisionLevel(int level) {
    return level >= 0 && level < static_cast<int>(DEPTH_PRECISION_TABLE.size());
}

} // namespace dynalgo::orbbec
