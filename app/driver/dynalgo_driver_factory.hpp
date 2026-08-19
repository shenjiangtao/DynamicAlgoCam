// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_driver_factory.hpp — Factory for creating device+pipeline pairs.
//
// Hides SDK-specific context (ObContext, RsContext) behind a single
// discoverDevices() call that returns vector<DynalgoContext::DiscoveredDevice>.
// The application layer never sees concrete SDK types.

#pragma once

#include "dynalgo_device.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dynalgo {

// [枚举说明 / Enum Description]
// 中文: 驱动厂商选择枚举，用于指定探测哪个厂商的设备
// English: Enum for selecting which vendor(s) to probe
enum class DriverVendor {
    ALL = 0, // probe both Orbbec and RoboSense / 探测 Orbbec 和 RoboSense 两个厂商
    ORBBEC,
    ROBOSENSE
};

// [结构体说明 / Struct Description]
// 中文: 驱动配置结构体，包含要探测的厂商信息
// English: Driver configuration struct containing vendor to probe
struct DriverConfig
{
    DriverVendor vendor = DriverVendor::ALL;
};

// [结构体说明 / Struct Description]
// 中文: 发现的设备结构体，包含设备和管道的共享指针
// English: Discovered device struct containing shared pointers to device and pipeline
struct DiscoveredDevice
{
    std::shared_ptr<DynalgoDevice> device;
    std::shared_ptr<DynalgoPipeline> pipeline;
};

// [函数说明 / Function Description]
// 中文: 发现所有可用设备并创建 DynalgoDevice+DynalgoPipeline 对
// English: Discover all available devices and create DynalgoDevice+DynalgoPipeline pairs
std::vector<DiscoveredDevice> discoverDevices(const DriverConfig& cfg = DriverConfig{});

} // namespace dynalgo
