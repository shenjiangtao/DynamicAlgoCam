// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_validator.hpp — Orbbec configuration validator.

#pragma once

#include "dynalgo_config_validator.hpp"

namespace dynalgo {

// Orbbec-specific configuration validator.
class ObValidator : public ConfigValidator
{
public:
    // Orbbec USB Vendor ID
    static constexpr uint16_t VENDOR_ID = 0x2bc5;

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
