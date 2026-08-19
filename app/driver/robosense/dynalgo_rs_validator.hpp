// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_validator.hpp — RoboSense configuration validator.

#pragma once

#include "dynalgo_config_validator.hpp"

namespace dynalgo {

// [类说明 / Class Description]
// 中文: RoboSense 专用配置验证器（AC1），验证流配置、传感器信息和设备信息
// English: RoboSense-specific configuration validator (AC1) for stream config, sensor info, and device info
class RsValidator : public ConfigValidator
{
public:
    // AC1 USB identifiers / AC1 USB 标识符
    static constexpr uint16_t VENDOR_ID = 0x3840;
    static constexpr uint16_t PRODUCT_ID = 0x1010;

    // [函数说明 / Function Description]
    // 中文: 验证流配置（颜色/深度分辨率和帧率必须匹配 AC1 规格）
    // English: Validate stream config (color/depth resolution and fps must match AC1 spec)
    bool validateStream(const DynalgoStreamConfig& config) const override;
    // [函数说明 / Function Description]
    // 中文: 验证传感器信息（必须同时有颜色和深度流）
    // English: Validate sensor info (must have both color and depth streams)
    bool validateSensorInfo(const DynalgoSensorInfo& info) const override;
    // [函数说明 / Function Description]
    // 中文: 验证设备信息（VID/PID 必须匹配 AC1）
    // English: Validate device info (VID/PID must match AC1)
    bool validateDevice(const DynalgoDeviceInfo& info) const override;
    std::string lastError() const override {
        return lastError_;
    }
    void clearError() override {
        lastError_.clear();
    }

private:
    mutable std::string lastError_;
};

} // namespace dynalgo
