// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_validator.hpp — Orbbec configuration validator.

#pragma once

#include "nio_config_validator.hpp"

namespace nio {

// Orbbec-specific configuration validator.
class ObValidator : public ConfigValidator
{
public:
    // Orbbec USB Vendor ID
    static constexpr uint16_t VENDOR_ID = 0x2bc5;

    bool validateStream(const NioStreamConfig& config) const override;
    bool validateSensorInfo(const NioSensorInfo& info) const override;
    bool validateDevice(const NioDeviceInfo& info) const override;
    std::string lastError() const override {
        return lastError_;
    }
    void clearError() override {
        lastError_.clear();
    }

private:
    mutable std::string lastError_;
};

} // namespace nio
