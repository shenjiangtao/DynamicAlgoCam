// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_validator.cpp — RoboSense configuration validator implementation.

#include "dynalgo_rs_validator.hpp"

#include "dynalgo_rs_spec.hpp"

namespace dynalgo {

// [函数说明 / Function Description]
// 中文: 验证流配置（颜色/深度分辨率和帧率必须匹配 AC1 规格）
// English: Validate stream config (color/depth resolution and fps must match AC1 spec)
bool RsValidator::validateStream(const DynalgoStreamConfig& config) const {
    switch (config.frameType) {
    case types::DynalgoFrameType::COLOR:
        if (config.width != rs::AC1::COLOR.resolution.width || config.height != rs::AC1::COLOR.resolution.height) {
            lastError_ = "AC1 color resolution must be " + std::to_string(rs::AC1::COLOR.resolution.width) + "x" +
                         std::to_string(rs::AC1::COLOR.resolution.height);
            return false;
        }
        if (config.fps != rs::AC1::COLOR.fps) {
            lastError_ = "AC1 color FPS must be " + std::to_string(rs::AC1::COLOR.fps);
            return false;
        }
        break;
    case types::DynalgoFrameType::DEPTH:
        if (config.width != rs::AC1::DEPTH.resolution.width || config.height != rs::AC1::DEPTH.resolution.height) {
            lastError_ = "AC1 depth resolution must be " + std::to_string(rs::AC1::DEPTH.resolution.width) + "x" +
                         std::to_string(rs::AC1::DEPTH.resolution.height);
            return false;
        }
        if (config.fps != rs::AC1::DEPTH.fps) {
            lastError_ = "AC1 depth FPS must be " + std::to_string(rs::AC1::DEPTH.fps);
            return false;
        }
        break;
    default:
        break;
    }
    lastError_.clear();
    return true;
}

// [函数说明 / Function Description]
// 中文: 验证传感器信息（必须同时有颜色和深度流）
// English: Validate sensor info (must have both color and depth streams)
bool RsValidator::validateSensorInfo(const DynalgoSensorInfo& info) const {
    if (!info.hasColor || !info.hasDepth) {
        lastError_ = "AC1 requires both color and depth streams";
        return false;
    }
    lastError_.clear();
    return true;
}

// [函数说明 / Function Description]
// 中文: 验证设备信息（VID/PID 必须匹配 AC1）
// English: Validate device info (VID/PID must match AC1)
bool RsValidator::validateDevice(const DynalgoDeviceInfo& info) const {
    if (info.vid != VENDOR_ID || info.pid != PRODUCT_ID) {
        lastError_ = "Invalid AC1 device: VID=0x" + std::to_string(info.vid) + ", PID=0x" + std::to_string(info.pid);
        return false;
    }
    lastError_.clear();
    return true;
}

} // namespace dynalgo
