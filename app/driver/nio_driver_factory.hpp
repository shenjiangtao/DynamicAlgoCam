// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_driver_factory.hpp — Factory for creating device+pipeline pairs.
//
// Hides SDK-specific context (ObContext, RsContext) behind a single
// discoverDevices() call that returns vector<NioContext::DiscoveredDevice>.
// The application layer never sees concrete SDK types.

#pragma once

#include "nio_device.hpp"

#include <memory>
#include <string>
#include <vector>

namespace nio {

struct DiscoveredDevice
{
    std::shared_ptr<NioDevice> device;
    std::shared_ptr<NioPipeline> pipeline;
};

std::vector<DiscoveredDevice> discoverDevices();

} // namespace nio
