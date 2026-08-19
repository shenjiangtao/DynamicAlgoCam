// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_config_validator.hpp — Abstract validation interface for driver configs.
//
// [文件说明 / File Description]
// 中文：驱动配置的抽象验证接口，每个供应商提供具体验证器检查配置有效性
// English: Abstract validation interface for driver configs, each vendor provides concrete validator to check config validity
//
// Each vendor provides a concrete validator that checks whether a given
// stream configuration or device info is valid for that vendor's hardware.

#pragma once

#include "dynalgo_device.hpp"

#include <memory>
#include <string>

namespace dynalgo {

// [类说明 / Class Description]
// 中文：配置验证器抽象接口，用于验证流/设备配置是否符合供应商硬件能力
// English: ConfigValidator abstract interface for validating stream/device configuration against vendor-specific hardware capabilities
//
// Usage:
//   auto validator = createValidator(DriverVendor::ORBBEC);
//   if (!validator->validate(config)) { /* reject */ }
//
class ConfigValidator
{
public:
    virtual ~ConfigValidator() = default;

    // [方法说明 / Method Description]
    // 中文：验证单个流配置（分辨率、帧率、格式），不支持返回false
    // English: Validate a single stream configuration (resolution, fps, format). Returns false if unsupported.
    virtual bool validateStream(const DynalgoStreamConfig& config) const = 0;

    // [方法说明 / Method Description]
    // 中文：验证管道设置后的整个传感器信息，无效组合返回false
    // English: Validate the entire DynalgoSensorInfo after pipeline setup. Returns false if invalid.
    virtual bool validateSensorInfo(const DynalgoSensorInfo& info) const = 0;

    // [方法说明 / Method Description]
    // 中文：验证发现的设备是否为预期的型号/固件
    // English: Validate that the discovered device is the expected model/firmware
    virtual bool validateDevice(const DynalgoDeviceInfo& info) const = 0;

    // [方法说明 / Method Description]
    // 中文：获取最后一次验证失败的人类可读描述
    // English: Human-readable description of the last validation failure
    virtual std::string lastError() const = 0;

    // [方法说明 / Method Description]
    // 中文：重置内部错误状态
    // English: Reset any internal error state
    virtual void clearError() = 0;
};

// [工厂函数 / Factory Function]
// 中文：为给定供应商创建适当的验证器，不支持的供应商返回nullptr
// English: Factory: create the appropriate validator for the given vendor. Returns nullptr if not supported.
std::unique_ptr<ConfigValidator> createValidator(DriverVendor vendor);

} // namespace dynalgo
