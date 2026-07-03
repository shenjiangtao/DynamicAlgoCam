// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_config_validator.hpp — Abstract validation interface for driver configs.
//
// Each vendor provides a concrete validator that checks whether a given
// stream configuration or device info is valid for that vendor’s hardware.

#pragma once

#include "nio_device.hpp"

#include <memory>
#include <string>

namespace nio {

// ---------------------------------------------------------------------------
// ConfigValidator
// ---------------------------------------------------------------------------
// Abstract interface for validating stream/device configuration against
// vendor-specific hardware capabilities.
//
// Usage:
//   auto validator = createValidator(DriverVendor::ORBBEC);
//   if (!validator->validate(config)) { /* reject */ }
//
class ConfigValidator
{
public:
    virtual ~ConfigValidator() = default;

    // Validate a single stream configuration (resolution, fps, format).
    // Returns false if the configuration is unsupported by this vendor.
    virtual bool validateStream(const NioStreamConfig& config) const = 0;

    // Validate the entire NioSensorInfo after pipeline setup.
    // Returns false if the sensor combination is invalid.
    virtual bool validateSensorInfo(const NioSensorInfo& info) const = 0;

    // Validate that the discovered device is the expected model/firmware.
    virtual bool validateDevice(const NioDeviceInfo& info) const = 0;

    // Human-readable description of the last validation failure.
    virtual std::string lastError() const = 0;

    // Reset any internal error state.
    virtual void clearError() = 0;
};

// Factory: create the appropriate validator for the given vendor.
// Returns nullptr if the vendor is not supported.
std::unique_ptr<ConfigValidator> createValidator(DriverVendor vendor);

} // namespace nio
