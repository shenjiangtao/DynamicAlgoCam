// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_validator.hpp — RoboSense configuration validator.

#pragma once

#include "dynalgo_config_validator.hpp"

namespace dynalgo {

// RoboSense-specific configuration validator (AC1).
class RsValidator : public ConfigValidator
{
public:
    // AC1 USB identifiers
    static constexpr uint16_t VENDOR_ID = 0x3840;
    static constexpr uint16_t PRODUCT_ID = 0x1010;

    bool validateStream(const DynalgoStreamConfig& config) const override;
    bool validateSensorInfo(const DynalgoSensorInfo& info) const override;
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
