// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_adapter.hpp — Conversion functions between Orbbec SDK types and
// Nio neutral types.  Only code that directly uses the Orbbec SDK should
// include this file; all downstream consumers use nio_types.hpp instead.
//
// Also contains selectBestProfile (score-based stream profile selection)
// and isLiDARDevice (moved from app/core/utils.hpp to remove ObSensor
// dependency from the core layer).

#pragma once

#include "nio_types.hpp"
#include <libobsensor/ObSensor.hpp>

namespace nio {

// OBFormat → NioFormat
inline NioFormat obFormatToNio(OBFormat f) {
    switch (f) {
    case OB_FORMAT_Y8:     return NioFormat::Y8;
    case OB_FORMAT_Y16:    return NioFormat::Y16;
    case OB_FORMAT_YUYV:   return NioFormat::YUYV;
    case OB_FORMAT_YUY2:   return NioFormat::YUY2;
    case OB_FORMAT_UYVY:   return NioFormat::UYVY;
    case OB_FORMAT_MJPG:   return NioFormat::MJPG;
    case OB_FORMAT_NV12:   return NioFormat::NV12;
    case OB_FORMAT_NV21:   return NioFormat::NV21;
    case OB_FORMAT_I420:   return NioFormat::I420;
    case OB_FORMAT_RGB:    return NioFormat::RGB;
    case OB_FORMAT_BGR:    return NioFormat::BGR;
    case OB_FORMAT_RGBA:   return NioFormat::RGBA;
    case OB_FORMAT_BGRA:   return NioFormat::BGRA;
    case OB_FORMAT_H264:   return NioFormat::H264;
    case OB_FORMAT_H265:   return NioFormat::H265;
    case OB_FORMAT_HEVC:   return NioFormat::HEVC;
    case OB_FORMAT_POINT:  return NioFormat::POINT;
    default:               return NioFormat::UNKNOWN;
    }
}

// NioFormat → OBFormat
inline OBFormat nioFormatToOb(NioFormat f) {
    switch (f) {
    case NioFormat::Y8:     return OB_FORMAT_Y8;
    case NioFormat::Y16:    return OB_FORMAT_Y16;
    case NioFormat::YUYV:   return OB_FORMAT_YUYV;
    case NioFormat::UYVY:   return OB_FORMAT_UYVY;
    case NioFormat::MJPG:   return OB_FORMAT_MJPG;
    case NioFormat::MJPEG:  return OB_FORMAT_MJPG;
    case NioFormat::NV12:   return OB_FORMAT_NV12;
    case NioFormat::NV21:   return OB_FORMAT_NV21;
    case NioFormat::I420:   return OB_FORMAT_I420;
    case NioFormat::RGB:    return OB_FORMAT_RGB;
    case NioFormat::BGR:    return OB_FORMAT_BGR;
    case NioFormat::RGBA:   return OB_FORMAT_RGBA;
    case NioFormat::BGRA:   return OB_FORMAT_BGRA;
    case NioFormat::H264:   return OB_FORMAT_H264;
    case NioFormat::H265:   return OB_FORMAT_H265;
    case NioFormat::HEVC:   return OB_FORMAT_HEVC;
    default:                return OB_FORMAT_UNKNOWN;
    }
}

// OBFrameType → NioFrameType
inline NioFrameType obFrameTypeToNio(OBFrameType t) {
    switch (t) {
    case OB_FRAME_COLOR:       return NioFrameType::COLOR;
    case OB_FRAME_DEPTH:       return NioFrameType::DEPTH;
    case OB_FRAME_IR:          return NioFrameType::IR;
    case OB_FRAME_IR_LEFT:     return NioFrameType::IR_LEFT;
    case OB_FRAME_IR_RIGHT:    return NioFrameType::IR_RIGHT;
    case OB_FRAME_ACCEL:       return NioFrameType::ACCEL;
    case OB_FRAME_GYRO:        return NioFrameType::GYRO;
    case OB_FRAME_COLOR_LEFT:  return NioFrameType::COLOR_LEFT;
    case OB_FRAME_COLOR_RIGHT: return NioFrameType::COLOR_RIGHT;
    case OB_FRAME_CONFIDENCE:  return NioFrameType::CONFIDENCE;
    case OB_FRAME_POINTS:      return NioFrameType::POINT;
    default:                    return NioFrameType::COLOR;
    }
}

// NioFrameType → OBSensorType (for disableStream etc.)
inline OBSensorType nioFrameTypeToObSensor(NioFrameType t) {
    switch (t) {
    case NioFrameType::COLOR:       return OB_SENSOR_COLOR;
    case NioFrameType::DEPTH:       return OB_SENSOR_DEPTH;
    case NioFrameType::IR:          return OB_SENSOR_IR;
    case NioFrameType::IR_LEFT:     return OB_SENSOR_IR_LEFT;
    case NioFrameType::IR_RIGHT:    return OB_SENSOR_IR_RIGHT;
    case NioFrameType::ACCEL:       return OB_SENSOR_ACCEL;
    case NioFrameType::GYRO:        return OB_SENSOR_GYRO;
    default:                         return OB_SENSOR_COLOR;
    }
}

// NioFrameType → OBFrameType
inline OBFrameType nioFrameTypeToOb(NioFrameType t) {
    switch (t) {
    case NioFrameType::COLOR:       return OB_FRAME_COLOR;
    case NioFrameType::DEPTH:       return OB_FRAME_DEPTH;
    case NioFrameType::IR:          return OB_FRAME_IR;
    case NioFrameType::IR_LEFT:     return OB_FRAME_IR_LEFT;
    case NioFrameType::IR_RIGHT:    return OB_FRAME_IR_RIGHT;
    case NioFrameType::ACCEL:       return OB_FRAME_ACCEL;
    case NioFrameType::GYRO:        return OB_FRAME_GYRO;
    case NioFrameType::COLOR_LEFT:  return OB_FRAME_COLOR_LEFT;
    case NioFrameType::COLOR_RIGHT: return OB_FRAME_COLOR_RIGHT;
    case NioFrameType::CONFIDENCE:  return OB_FRAME_CONFIDENCE;
    case NioFrameType::POINT:       return OB_FRAME_POINTS;
    default:                         return OB_FRAME_COLOR;
    }
}

// OBCameraIntrinsic → NioIntrinsic
inline NioIntrinsic obIntrinsicToNio(const OBCameraIntrinsic &ob) {
    NioIntrinsic n;
    n.fx = ob.fx;
    n.fy = ob.fy;
    n.cx = ob.cx;
    n.cy = ob.cy;
    n.width = ob.width;
    n.height = ob.height;
    return n;
}

// NioIntrinsic → OBCameraIntrinsic
inline OBCameraIntrinsic nioIntrinsicToOb(const NioIntrinsic &n) {
    OBCameraIntrinsic ob = {};
    ob.fx = n.fx;
    ob.fy = n.fy;
    ob.cx = n.cx;
    ob.cy = n.cy;
    ob.width = n.width;
    ob.height = n.height;
    return ob;
}

// selectBestProfile: scoring-based stream profile selector.
// Prefers the requested format (+1000), then favors 640w (+100) /
// 848w (+90) / 1280w (+80), and 30fps (+50) / 25fps (+45) / 15fps (+30).
// Falls back to first profile if no match.
inline std::shared_ptr<ob::VideoStreamProfile> selectBestProfile(std::shared_ptr<ob::StreamProfileList> profiles,
                                                                  OBFormat preferredFormat) {
    std::shared_ptr<ob::VideoStreamProfile> best;
    int bestScore = -1;

    for (uint32_t i = 0; i < profiles->getCount(); i++) {
        try {
            auto sp = profiles->getProfile(i);
            if (!sp)
                continue;
            auto vsp = sp->as<ob::VideoStreamProfile>();
            if (!vsp)
                continue;

            int score = 0;
            if (vsp->getFormat() == preferredFormat)
                score += 1000;
            if (vsp->getWidth() == 1280)
                score += 120;
            else if (vsp->getWidth() == 640)
                score += 100;
            else if (vsp->getWidth() == 848)
                score += 90;
            if (vsp->getFps() == 30)
                score += 50;
            else if (vsp->getFps() == 25)
                score += 45;
            else if (vsp->getFps() == 15)
                score += 30;

            if (score > bestScore) {
                bestScore = score;
                best = vsp;
            }
        } catch (...) {
            continue;
        }
    }

    if (!best && profiles->getCount() > 0) {
        try {
            auto sp = profiles->getProfile(0);
            best = sp->as<ob::VideoStreamProfile>();
        } catch (...) {
        }
    }
    return best;
}

// Orbbec VID (was NIO_DEVICE_VID — specific to Orbbec, not generic).
static constexpr uint16_t OB_DEVICE_VID = 0x2bc5;

inline bool isGemini305Device(int vid, int pid) {
    return vid == OB_DEVICE_VID && (pid == 0x0840 || pid == 0x0841 || pid == 0x0842 || pid == 0x0843);
}

inline bool isGemini305gDevice(int vid, int pid, const char *connectionType) {
    return isGemini305Device(vid, pid) && strcmp(connectionType, "GMSL2") == 0;
}

inline bool isAstraMiniDevice(int vid, int pid) {
    return vid == OB_DEVICE_VID && (pid == 0x069d || pid == 0x065b || pid == 0x065e);
}

inline bool isLiDARDevice(std::shared_ptr<ob::Device> device) {
    auto sensorList = device->getSensorList();
    for (uint32_t i = 0; i < sensorList->getCount(); i++) {
        if (sensorList->getSensorType(i) == OB_SENSOR_LIDAR)
            return true;
    }
    return false;
}

} // namespace nio
