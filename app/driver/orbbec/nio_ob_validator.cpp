// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_validator.cpp — Orbbec configuration validator implementation.

#include "nio_ob_validator.hpp"

#include "nio_ob_spec.hpp"

namespace nio {

bool ObValidator::validateStream(const NioStreamConfig& config) const {
    if (config.width <= 0 || config.height <= 0) {
        lastError_ = "Invalid resolution: " + std::to_string(config.width) + "x" + std::to_string(config.height);
        return false;
    }
    if (config.fps <= 0) {
        lastError_ = "Invalid FPS: " + std::to_string(config.fps);
        return false;
    }
    if (config.format == types::NioFormat::UNKNOWN) {
        lastError_ = "Invalid pixel format (UNKNOWN)";
        return false;
    }
    lastError_.clear();
    return true;
}

bool ObValidator::validateSensorInfo(const NioSensorInfo& info) const {
    // Validate depth precision scale
    if (info.hasDepth && !nio::orbbec::isValidDepthScale(info.depthScale)) {
        lastError_ = "Invalid depth scale: " + std::to_string(info.depthScale);
        return false;
    }
    lastError_.clear();
    return true;
}

bool ObValidator::validateDevice(const NioDeviceInfo& info) const {
    if (info.vid != VENDOR_ID) {
        lastError_ =
            "Invalid Orbbec VID: 0x" + std::to_string(info.vid) + " (expected 0x" + std::to_string(VENDOR_ID) + ")";
        return false;
    }
    lastError_.clear();
    return true;
}

} // namespace nio
