// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_rs_validator.cpp — RoboSense configuration validator implementation.

#include "nio_rs_validator.hpp"

#include "nio_rs_spec.hpp"

namespace nio {

bool RsValidator::validateStream(const NioStreamConfig& config) const {
    switch (config.frameType) {
    case types::NioFrameType::COLOR:
        if (config.width != rs::AC1::COLOR.resolution.width || config.height != rs::AC1::COLOR.resolution.height) {
            lastError_ = "AC1 color resolution must be " + std::to_string(rs::AC1::COLOR.resolution.width) + "x" +
                         std::to_string(rs::AC1::COLOR.resolution.height);
            return false;
        }
        if (config.fps != rs::AC1::COLOR.fps) {
            lastError_ = "AC1 color FPS must be " + std::to_string(rs::AC1::COLOR.fps);
            return false;
        }
        break;
    case types::NioFrameType::DEPTH:
        if (config.width != rs::AC1::DEPTH.resolution.width || config.height != rs::AC1::DEPTH.resolution.height) {
            lastError_ = "AC1 depth resolution must be " + std::to_string(rs::AC1::DEPTH.resolution.width) + "x" +
                         std::to_string(rs::AC1::DEPTH.resolution.height);
            return false;
        }
        if (config.fps != rs::AC1::DEPTH.fps) {
            lastError_ = "AC1 depth FPS must be " + std::to_string(rs::AC1::DEPTH.fps);
            return false;
        }
        break;
    default:
        break;
    }
    lastError_.clear();
    return true;
}

bool RsValidator::validateSensorInfo(const NioSensorInfo& info) const {
    if (!info.hasColor || !info.hasDepth) {
        lastError_ = "AC1 requires both color and depth streams";
        return false;
    }
    lastError_.clear();
    return true;
}

bool RsValidator::validateDevice(const NioDeviceInfo& info) const {
    if (info.vid != VENDOR_ID || info.pid != PRODUCT_ID) {
        lastError_ = "Invalid AC1 device: VID=0x" + std::to_string(info.vid) + ", PID=0x" + std::to_string(info.pid);
        return false;
    }
    lastError_.clear();
    return true;
}

} // namespace nio
