// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_rs_validator.hpp — RoboSense configuration validator.

#pragma once

#include "nio_config_validator.hpp"

namespace nio {

// RoboSense-specific configuration validator (AC1).
class RsValidator : public ConfigValidator
{
public:
    // AC1 USB identifiers
    static constexpr uint16_t VENDOR_ID = 0x3840;
    static constexpr uint16_t PRODUCT_ID = 0x1010;

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
