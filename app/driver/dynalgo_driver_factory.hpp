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

// Enum for selecting which vendor(s) to probe.
enum class DriverVendor {
    ALL = 0, // probe both Orbbec and RoboSense
    ORBBEC,
    ROBOSENSE
};

struct DriverConfig
{
    DriverVendor vendor = DriverVendor::ALL;
};

struct DiscoveredDevice
{
    std::shared_ptr<DynalgoDevice> device;
    std::shared_ptr<DynalgoPipeline> pipeline;
};

std::vector<DiscoveredDevice> discoverDevices(const DriverConfig& cfg = DriverConfig{});

} // namespace dynalgo
