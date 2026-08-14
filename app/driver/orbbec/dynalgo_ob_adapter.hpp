// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_adapter.hpp — Conversion functions between Orbbec SDK types and
// Nio neutral types.  Only code that directly uses the Orbbec SDK should
// include this file; all downstream consumers use dynalgo_types.hpp instead.
//
// Also contains selectBestProfile (score-based stream profile selection)
// and isLiDARDevice (moved from app/core/utils.hpp to remove ObSensor
// dependency from the core layer).

#pragma once

#include "dynalgo_types.hpp"
#include "dynalgo_log.hpp"
#include <libobsensor/ObSensor.hpp>

namespace dynalgo {

// OBFormat → DynalgoFormat
inline DynalgoFormat obFormatToNio(OBFormat f) {
    switch (f) {
    case OB_FORMAT_Y8:
        return DynalgoFormat::Y8;
    case OB_FORMAT_Y16:
        return DynalgoFormat::Y16;
    case OB_FORMAT_YUYV:
        return DynalgoFormat::YUYV;
    case OB_FORMAT_YUY2:
        return DynalgoFormat::YUY2;
    case OB_FORMAT_UYVY:
        return DynalgoFormat::UYVY;
    case OB_FORMAT_MJPG:
        return DynalgoFormat::MJPG;
    case OB_FORMAT_NV12:
        return DynalgoFormat::NV12;
    case OB_FORMAT_NV21:
        return DynalgoFormat::NV21;
    case OB_FORMAT_I420:
        return DynalgoFormat::I420;
    case OB_FORMAT_RGB:
        return DynalgoFormat::RGB;
    case OB_FORMAT_BGR:
        return DynalgoFormat::BGR;
    case OB_FORMAT_RGBA:
        return DynalgoFormat::RGBA;
    case OB_FORMAT_BGRA:
        return DynalgoFormat::BGRA;
    case OB_FORMAT_H264:
        return DynalgoFormat::H264;
    case OB_FORMAT_H265:
        return DynalgoFormat::H265;
    case OB_FORMAT_HEVC:
        return DynalgoFormat::HEVC;
    case OB_FORMAT_POINT:
        return DynalgoFormat::POINT;
    default:
        return DynalgoFormat::UNKNOWN;
    }
}

// DynalgoFormat → OBFormat
inline OBFormat nioFormatToOb(DynalgoFormat f) {
    switch (f) {
    case DynalgoFormat::Y8:
        return OB_FORMAT_Y8;
    case DynalgoFormat::Y16:
        return OB_FORMAT_Y16;
    case DynalgoFormat::YUYV:
        return OB_FORMAT_YUYV;
    case DynalgoFormat::UYVY:
        return OB_FORMAT_UYVY;
    case DynalgoFormat::MJPG:
        return OB_FORMAT_MJPG;
    case DynalgoFormat::MJPEG:
        return OB_FORMAT_MJPG;
    case DynalgoFormat::NV12:
        return OB_FORMAT_NV12;
    case DynalgoFormat::NV21:
        return OB_FORMAT_NV21;
    case DynalgoFormat::I420:
        return OB_FORMAT_I420;
    case DynalgoFormat::RGB:
        return OB_FORMAT_RGB;
    case DynalgoFormat::BGR:
        return OB_FORMAT_BGR;
    case DynalgoFormat::RGBA:
        return OB_FORMAT_RGBA;
    case DynalgoFormat::BGRA:
        return OB_FORMAT_BGRA;
    case DynalgoFormat::H264:
        return OB_FORMAT_H264;
    case DynalgoFormat::H265:
        return OB_FORMAT_H265;
    case DynalgoFormat::HEVC:
        return OB_FORMAT_HEVC;
    default:
        return OB_FORMAT_UNKNOWN;
    }
}

// OBFrameType → DynalgoFrameType
inline DynalgoFrameType obFrameTypeToNio(OBFrameType t) {
    switch (t) {
    case OB_FRAME_COLOR:
        return DynalgoFrameType::COLOR;
    case OB_FRAME_DEPTH:
        return DynalgoFrameType::DEPTH;
    case OB_FRAME_IR:
        return DynalgoFrameType::IR;
    case OB_FRAME_IR_LEFT:
        return DynalgoFrameType::IR_LEFT;
    case OB_FRAME_IR_RIGHT:
        return DynalgoFrameType::IR_RIGHT;
    case OB_FRAME_ACCEL:
        return DynalgoFrameType::ACCEL;
    case OB_FRAME_GYRO:
        return DynalgoFrameType::GYRO;
    case OB_FRAME_COLOR_LEFT:
        return DynalgoFrameType::COLOR_LEFT;
    case OB_FRAME_COLOR_RIGHT:
        return DynalgoFrameType::COLOR_RIGHT;
    case OB_FRAME_CONFIDENCE:
        return DynalgoFrameType::CONFIDENCE;
    case OB_FRAME_POINTS:
        return DynalgoFrameType::POINT;
    default:
        return DynalgoFrameType::COLOR;
    }
}

// DynalgoFrameType → OBSensorType (for disableStream etc.)
inline OBSensorType nioFrameTypeToObSensor(DynalgoFrameType t) {
    switch (t) {
    case DynalgoFrameType::COLOR:
        return OB_SENSOR_COLOR;
    case DynalgoFrameType::DEPTH:
        return OB_SENSOR_DEPTH;
    case DynalgoFrameType::IR:
        return OB_SENSOR_IR;
    case DynalgoFrameType::IR_LEFT:
        return OB_SENSOR_IR_LEFT;
    case DynalgoFrameType::IR_RIGHT:
        return OB_SENSOR_IR_RIGHT;
    case DynalgoFrameType::ACCEL:
        return OB_SENSOR_ACCEL;
    case DynalgoFrameType::GYRO:
        return OB_SENSOR_GYRO;
    default:
        return OB_SENSOR_COLOR;
    }
}

// DynalgoFrameType → OBFrameType
inline OBFrameType nioFrameTypeToOb(DynalgoFrameType t) {
    switch (t) {
    case DynalgoFrameType::COLOR:
        return OB_FRAME_COLOR;
    case DynalgoFrameType::DEPTH:
        return OB_FRAME_DEPTH;
    case DynalgoFrameType::IR:
        return OB_FRAME_IR;
    case DynalgoFrameType::IR_LEFT:
        return OB_FRAME_IR_LEFT;
    case DynalgoFrameType::IR_RIGHT:
        return OB_FRAME_IR_RIGHT;
    case DynalgoFrameType::ACCEL:
        return OB_FRAME_ACCEL;
    case DynalgoFrameType::GYRO:
        return OB_FRAME_GYRO;
    case DynalgoFrameType::COLOR_LEFT:
        return OB_FRAME_COLOR_LEFT;
    case DynalgoFrameType::COLOR_RIGHT:
        return OB_FRAME_COLOR_RIGHT;
    case DynalgoFrameType::CONFIDENCE:
        return OB_FRAME_CONFIDENCE;
    case DynalgoFrameType::POINT:
        return OB_FRAME_POINTS;
    default:
        return OB_FRAME_COLOR;
    }
}

// OBCameraIntrinsic → DynalgoIntrinsic
inline DynalgoIntrinsic obIntrinsicToNio(const OBCameraIntrinsic& ob) {
    DynalgoIntrinsic n;
    n.fx = ob.fx;
    n.fy = ob.fy;
    n.cx = ob.cx;
    n.cy = ob.cy;
    n.width = ob.width;
    n.height = ob.height;
    return n;
}

// DynalgoIntrinsic → OBCameraIntrinsic
inline OBCameraIntrinsic nioIntrinsicToOb(const DynalgoIntrinsic& n) {
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
// Prefers the requested format (+1000), then favors 1280w (+120) /
// 640w (+100) / 848w (+90), 800h (+110) / 720h (+100) / 480h (+80),
// and 30fps (+50) / 25fps (+45) / 15fps (+30).
// When `hwD2CSupportedProfiles` is non-null, profiles that appear in it
// receive an extra +800 so depth selection prefers HW-D2C-capable profiles,
// reducing software-align fallbacks.
// Falls back to first profile if no match.
inline std::shared_ptr<ob::VideoStreamProfile> selectBestProfile(std::shared_ptr<ob::StreamProfileList> profiles,
                                                                 OBFormat preferredFormat,
                                                                 std::shared_ptr<ob::StreamProfileList> hwD2CSupportedProfiles = nullptr) {
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
            if (vsp->getHeight() == 800)
                score += 110;
            else if (vsp->getHeight() == 720)
                score += 100;
            else if (vsp->getHeight() == 480)
                score += 80;
            if (vsp->getFps() == 30)
                score += 50;
            else if (vsp->getFps() == 25)
                score += 45;
            else if (vsp->getFps() == 15)
                score += 30;

            if (hwD2CSupportedProfiles) {
                for (uint32_t j = 0; j < hwD2CSupportedProfiles->getCount(); j++) {
                    try {
                        auto hp = hwD2CSupportedProfiles->getProfile(j)->as<ob::VideoStreamProfile>();
                        if (hp && hp->getWidth() == vsp->getWidth() && hp->getHeight() == vsp->getHeight() &&
                            hp->getFormat() == vsp->getFormat() && hp->getFps() == vsp->getFps()) {
                            score += 800;
                            break;
                        }
                    } catch (...) {
                        continue;
                    }
                }
            }

            if (score > bestScore) {
                bestScore = score;
                best = vsp;
            }
        } catch (...) {
            DYNALGO_LOG_WARN_S("selectBestProfile: profile scoring threw, skipping candidate");
            continue;
        }
    }

    if (!best && profiles->getCount() > 0) {
        try {
            auto sp = profiles->getProfile(0);
            best = sp->as<ob::VideoStreamProfile>();
        } catch (...) {
            DYNALGO_LOG_WARN_S("selectBestProfile: fallback getProfile(0) threw, no profile selected");
        }
    }
    return best;
}

// Orbbec VID (was DYNALGO_DEVICE_VID — specific to Orbbec, not generic).
static constexpr uint16_t OB_DEVICE_VID = 0x2bc5;

inline bool isGemini305Device(int vid, int pid) {
    return vid == OB_DEVICE_VID && (pid == 0x0840 || pid == 0x0841 || pid == 0x0842 || pid == 0x0843);
}

inline bool isGemini305gDevice(int vid, int pid, const char* connectionType) {
    return isGemini305Device(vid, pid) && strcmp(connectionType, "GMSL2") == 0;
}

inline bool isAstraMiniDevice(int vid, int pid) {
    return vid == OB_DEVICE_VID && (pid == 0x069d || pid == 0x065b || pid == 0x065e);
}

inline bool isGemini335L336LDevice(int vid, int pid) {
    return vid == OB_DEVICE_VID && (pid == 0x0804 || pid == 0x0807);
}

inline bool isLiDARDevice(std::shared_ptr<ob::Device> device) {
    auto sensorList = device->getSensorList();
    for (uint32_t i = 0; i < sensorList->getCount(); i++) {
        if (sensorList->getSensorType(i) == OB_SENSOR_LIDAR)
            return true;
    }
    return false;
}

} // namespace dynalgo
