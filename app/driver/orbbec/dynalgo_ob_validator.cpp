// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_validator.cpp — Orbbec configuration validator implementation.

#include "dynalgo_ob_validator.hpp"

#include "dynalgo_ob_spec.hpp"

namespace dynalgo {

// [函数说明 / Function Description]
// 中文: 验证流配置（分辨率、帧率、像素格式）
// English: Validate stream config (resolution, fps, pixel format)
bool ObValidator::validateStream(const DynalgoStreamConfig& config) const {
    if (config.width <= 0 || config.height <= 0) {
        lastError_ = "Invalid resolution: " + std::to_string(config.width) + "x" + std::to_string(config.height);
        return false;
    }
    if (config.fps <= 0) {
        lastError_ = "Invalid FPS: " + std::to_string(config.fps);
        return false;
    }
    if (config.format == types::DynalgoFormat::UNKNOWN) {
        lastError_ = "Invalid pixel format (UNKNOWN)";
        return false;
    }
    lastError_.clear();
    return true;
}

// [函数说明 / Function Description]
// 中文: 验证传感器信息（深度精度比例）
// English: Validate sensor info (depth precision scale)
bool ObValidator::validateSensorInfo(const DynalgoSensorInfo& info) const {
    // Validate depth precision scale
    if (info.hasDepth && !dynalgo::orbbec::isValidDepthScale(info.depthScale)) {
        lastError_ = "Invalid depth scale: " + std::to_string(info.depthScale);
        return false;
    }
    lastError_.clear();
    return true;
}

// [函数说明 / Function Description]
// 中文: 验证设备信息（VID 必须为 Orbbec 的 0x2bc5）
// English: Validate device info (VID must be Orbbec's 0x2bc5)
bool ObValidator::validateDevice(const DynalgoDeviceInfo& info) const {
    if (info.vid != VENDOR_ID) {
        lastError_ =
            "Invalid Orbbec VID: 0x" + std::to_string(info.vid) + " (expected 0x" + std::to_string(VENDOR_ID) + ")";
        return false;
    }
    lastError_.clear();
    return true;
}

} // namespace dynalgo
