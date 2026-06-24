// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_driver_factory.cpp — Factory: discovers all available devices
// and creates NioDevice+NioPipeline pairs for each.

#include "nio_driver_factory.hpp"

#ifdef ENABLE_ORBBEC
#include "nio_ob_device.hpp"
#endif

#ifdef ENABLE_RS_AC1
#include "nio_rs_device.hpp"
#endif

namespace nio {

std::vector<DiscoveredDevice> discoverDevices() {
    std::vector<DiscoveredDevice> result;

#ifdef ENABLE_ORBBEC
    ObContext obCtx;
    uint32_t obCount = obCtx.getDeviceCount();
    for (uint32_t i = 0; i < obCount; i++) {
        auto nioDev = obCtx.getDevice(i);
        auto obDev = std::dynamic_pointer_cast<ObDevice>(nioDev);
        if (!obDev)
            continue;
        DiscoveredDevice dd;
        dd.device = nioDev;
        dd.pipeline = std::make_shared<ObPipeline>(obDev->obDevice());
        result.push_back(std::move(dd));
    }
#endif

#ifdef ENABLE_RS_AC1
    RsContext rsCtx;
    uint32_t rsCount = rsCtx.getDeviceCount();
    for (uint32_t i = 0; i < rsCount; i++) {
        auto nioDev = rsCtx.getDevice(i);
        auto rsDev = std::dynamic_pointer_cast<RsDevice>(nioDev);
        if (!rsDev)
            continue;
        DiscoveredDevice dd;
        dd.device = nioDev;
        dd.pipeline = std::make_shared<RsPipeline>(rsDev);
        result.push_back(std::move(dd));
    }
#endif

    return result;
}

} // namespace nio
