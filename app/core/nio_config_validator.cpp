// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_config_validator.cpp — Factory for creating vendor-specific validators instances.

#include "nio_config_validator.hpp"

#ifdef ENABLE_ORBBEC
#include "../driver/orbbec/nio_ob_validator.hpp"
#endif

#ifdef ENABLE_RS_AC1
#include "../driver/robosense/nio_rs_validator.hpp"
#endif

namespace nio {

std::unique_ptr<ConfigValidator> createValidator(DriverVendor vendor) {
    switch (vendor) {
    case DriverVendor::ORBBEC:
#ifdef ENABLE_ORBBEC
        return std::make_unique<ObValidator>();
#else
        return nullptr;
#endif
    case DriverVendor::ROBOSENSE:
#ifdef ENABLE_RS_AC1
        return std::make_unique<RsValidator>();
#else
        return nullptr;
#endif
    case DriverVendor::ALL:
    default:
        return nullptr;
    }
}

} // namespace nio
