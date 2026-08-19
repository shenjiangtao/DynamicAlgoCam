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

// [结构体说明 / Struct Description]
// 中文: 深度精度等级映射表项（等级、比例、描述）
// English: Depth precision level mapping table entry (level, scale, description)
struct DepthPrecisionLevel
{
    int level;        // OB_PROP_DEPTH_PRECISION_LEVEL_INT value / OB_PROP_DEPTH_PRECISION_LEVEL_INT 值
    float scale;      // raw depth value -> meters / 原始深度值 -> 米
    const char* desc; // human-readable description / 人类可读描述
};

// [常量说明 / Constant Description]
// 中文: 默认深度比例（1mm = 0.001m）
// English: Default depth scale (1mm = 0.001m)
inline constexpr float DEFAULT_DEPTH_SCALE = 0.001f;

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
// [函数说明 / Function Description]
// 中文: 将 Orbbec 深度精度等级转换为深度比例（米/单位）
// English: Convert Orbbec depth precision level to depth scale (meters per unit)
[[nodiscard]] constexpr float depthLevelToScale(int level) {
    if (level < 0 || level >= static_cast<int>(DEPTH_PRECISION_TABLE.size())) {
        return DEFAULT_DEPTH_SCALE;
    }
    return DEPTH_PRECISION_TABLE[level].scale;
}

// [常量说明 / Constant Description]
// 中文: 默认颜色格式（MJPG）
// English: Default color format (MJPG)
inline constexpr DynalgoFormat COLOR_FMT_DEFAULT = DynalgoFormat::MJPG;
// [常量说明 / Constant Description]
// 中文: Gemini 305 系列首选颜色格式（YUYV）
// English: Gemini 305 series preferred color format (YUYV)
inline constexpr DynalgoFormat COLOR_FMT_GEMINI305 = DynalgoFormat::YUYV;
// [常量说明 / Constant Description]
// 中文: Gemini 335L/336L 系列首选颜色格式（YUYV）
// English: Gemini 335L/336L series preferred color format (YUYV)
inline constexpr DynalgoFormat COLOR_FMT_GEMINI335L336L = DynalgoFormat::YUYV;

/**
 * @brief Determine the preferred color format for a given Orbbec device.
 * @param isGemini305 Whether the device is identified as Gemini 305 series.
 * @param isGemini335L336L Whether the device is identified as Gemini 335L/336L.
 * @return The preferred DynalgoFormat for the color stream.
 */
// [函数说明 / Function Description]
// 中文: 根据设备类型确定首选颜色格式
// English: Determine preferred color format based on device type
[[nodiscard]] constexpr DynalgoFormat getPreferredColorFormat(bool isGemini305, bool isGemini335L336L) {
    if (isGemini305 || isGemini335L336L) {
        return COLOR_FMT_GEMINI305;
    }
    return COLOR_FMT_DEFAULT;
}

// [常量说明 / Constant Description]
// 中文: 默认启用 IR_LEFT
// English: Enable IR_LEFT by default
inline constexpr bool IR_LEFT_ENABLED_DEFAULT = true;
// [常量说明 / Constant Description]
// 中文: Gemini 305g 禁用 IR_LEFT（硬件限制）
// English: Gemini 305g disables IR_LEFT (hardware limitation)
inline constexpr bool IR_LEFT_ENABLED_GEMINI305G = false;

// [函数说明 / Function Description]
// 中文: 验证深度比例是否有效（>0）
// English: Validate depth scale is valid (>0)
[[nodiscard]] constexpr bool isValidDepthScale(float scale) {
    return scale > 0.0f;
}

// [函数说明 / Function Description]
// 中文: 验证精度等级是否在有效范围内
// English: Validate precision level is within valid range
[[nodiscard]] constexpr bool isValidPrecisionLevel(int level) {
    return level >= 0 && level < static_cast<int>(DEPTH_PRECISION_TABLE.size());
}

} // namespace dynalgo::orbbec
