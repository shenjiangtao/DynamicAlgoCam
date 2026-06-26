// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_rs_adapter.hpp — RS-AC1 ↔ Nio type conversions.
//
// Converts between RoboSense rs_driver types (frame_format_t, ImuData)
// and SDK-neutral Nio types (NioFormat, NioImuSample).

#pragma once

#include "nio_frame.hpp"
#include "nio_types.hpp"

#include <rs_driver/msg/image_data_msg.hpp>
#include <rs_driver/msg/imu_data_msg.hpp>

namespace nio {
// RS frame_format_t → NioFormat
inline NioFormat rsFrameFormatToNio(robosense::lidar::frame_format_t fmt) {
    using F = robosense::lidar::frame_format_t;
    switch (fmt) {
    case F::FRAME_FORMAT_NV12:
        return NioFormat::NV12;
    case F::FRAME_FORMAT_BGR24:
        return NioFormat::BGR;
    case F::FRAME_FORMAT_RGB24:
        return NioFormat::RGB;
    case F::FRAME_FORMAT_YUV422:
        return NioFormat::YUYV;
    default:
        return NioFormat::UNKNOWN;
    }
}

// NioFormat → RS frame_format_t
inline robosense::lidar::frame_format_t nioFormatToRsFrameFormat(NioFormat fmt) {
    using F = robosense::lidar::frame_format_t;
    switch (fmt) {
    case NioFormat::NV12:
        return F::FRAME_FORMAT_NV12;
    case NioFormat::BGR:
        return F::FRAME_FORMAT_BGR24;
    case NioFormat::RGB:
        return F::FRAME_FORMAT_RGB24;
    case NioFormat::YUYV:
        return F::FRAME_FORMAT_YUV422;
    default:
        return F::FRAME_FORMAT_NV12;
    }
}

// RS ImuData → vector of NioImuSample (one ACCEL + one GYRO)
inline std::vector<NioImuSample> rsImuToNioSamples(const std::shared_ptr<robosense::lidar::ImuData>& imu) {
    std::vector<NioImuSample> samples;
    auto tsUs = static_cast<uint64_t>(imu->timestamp * 1e6);

    NioImuSample accel;
    accel.type = NioFrameType::ACCEL;
    accel.timestampUs = tsUs;
    accel.x = imu->linear_acceleration_x;
    accel.y = imu->linear_acceleration_y;
    accel.z = imu->linear_acceleration_z;
    accel.temperature = 0.0f;
    samples.push_back(accel);

    NioImuSample gyro;
    gyro.type = NioFrameType::GYRO;
    gyro.timestampUs = tsUs;
    gyro.x = imu->angular_velocity_x;
    gyro.y = imu->angular_velocity_y;
    gyro.z = imu->angular_velocity_z;
    gyro.temperature = 0.0f;
    samples.push_back(gyro);

    return samples;
}
} // namespace nio
