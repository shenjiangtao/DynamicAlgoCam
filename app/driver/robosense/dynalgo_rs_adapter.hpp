// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_adapter.hpp — RS-AC1 ↔ Nio type conversions.
//
// Converts between RoboSense rs_driver types (frame_format_t, ImuData)
// and SDK-neutral Nio types (DynalgoFormat, DynalgoImuSample).

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"

#include <rs_driver/msg/image_data_msg.hpp>
#include <rs_driver/msg/imu_data_msg.hpp>

namespace dynalgo {
// RS frame_format_t → DynalgoFormat
inline DynalgoFormat rsFrameFormatToNio(robosense::lidar::frame_format_t fmt) {
    using F = robosense::lidar::frame_format_t;
    switch (fmt) {
    case F::FRAME_FORMAT_NV12:
        return DynalgoFormat::NV12;
    case F::FRAME_FORMAT_BGR24:
        return DynalgoFormat::BGR;
    case F::FRAME_FORMAT_RGB24:
        return DynalgoFormat::RGB;
    case F::FRAME_FORMAT_YUV422:
        return DynalgoFormat::YUYV;
    default:
        return DynalgoFormat::UNKNOWN;
    }
}

// DynalgoFormat → RS frame_format_t
inline robosense::lidar::frame_format_t nioFormatToRsFrameFormat(DynalgoFormat fmt) {
    using F = robosense::lidar::frame_format_t;
    switch (fmt) {
    case DynalgoFormat::NV12:
        return F::FRAME_FORMAT_NV12;
    case DynalgoFormat::BGR:
        return F::FRAME_FORMAT_BGR24;
    case DynalgoFormat::RGB:
        return F::FRAME_FORMAT_RGB24;
    case DynalgoFormat::YUYV:
        return F::FRAME_FORMAT_YUV422;
    default:
        return F::FRAME_FORMAT_NV12;
    }
}

// RS ImuData → vector of DynalgoImuSample (one ACCEL + one GYRO)
inline std::vector<DynalgoImuSample> rsImuToNioSamples(const std::shared_ptr<robosense::lidar::ImuData>& imu) {
    std::vector<DynalgoImuSample> samples;
    auto tsUs = static_cast<uint64_t>(imu->timestamp * 1e6);

    DynalgoImuSample accel;
    accel.type = DynalgoFrameType::ACCEL;
    accel.timestampUs = tsUs;
    accel.x = imu->linear_acceleration_x;
    accel.y = imu->linear_acceleration_y;
    accel.z = imu->linear_acceleration_z;
    accel.temperature = 0.0f;
    samples.push_back(accel);

    DynalgoImuSample gyro;
    gyro.type = DynalgoFrameType::GYRO;
    gyro.timestampUs = tsUs;
    gyro.x = imu->angular_velocity_x;
    gyro.y = imu->angular_velocity_y;
    gyro.z = imu->angular_velocity_z;
    gyro.temperature = 0.0f;
    samples.push_back(gyro);

    return samples;
}
} // namespace dynalgo
