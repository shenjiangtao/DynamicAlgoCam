// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_validator.hpp — Orbbec configuration validator.

#pragma once

#include "dynalgo_config_validator.hpp"

namespace dynalgo {

// [类说明 / Class Description]
// 中文: Orbbec 专用配置验证器，验证流配置、传感器信息和设备信息
// English: Orbbec-specific configuration validator for stream config, sensor info, and device info
class ObValidator : public ConfigValidator
{
public:
    // Orbbec USB Vendor ID / Orbbec USB 厂商 ID
    static constexpr uint16_t VENDOR_ID = 0x2bc5;

    // [函数说明 / Function Description]
    // 中文: 验证流配置（分辨率、帧率、像素格式）
    // English: Validate stream config (resolution, fps, pixel format)
    bool validateStream(const DynalgoStreamConfig& config) const override;
    // [函数说明 / Function Description]
    // 中文: 验证传感器信息（深度精度比例）
    // English: Validate sensor info (depth precision scale)
    bool validateSensorInfo(const DynalgoSensorInfo& info) const override;
    // [函数说明 / Function Description]
    // 中文: 验证设备信息（VID 必须为 Orbbec 的 0x2bc5）
    // English: Validate device info (VID must be Orbbec's 0x2bc5)
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
