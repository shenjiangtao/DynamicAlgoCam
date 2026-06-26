// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_types.hpp — SDK-neutral value types for multi-SDK capture.
//
// Replaces direct usage of Orbbec SDK types (OBFormat, OBFrameType,
// OBCameraIntrinsic, etc.) with SDK-agnostic equivalents.  Conversion
// functions between NioFormat↔OBFormat and NioFrameType↔OBFrameType
// are provided in nio_ob_adapter.hpp / nio_rs_adapter.hpp (driver layer only).

#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace nio {

// Pixel / frame format — SDK-independent.
enum class NioFormat {
    UNKNOWN = 0,
    Y8,
    Y16,
    YUYV,
    UYVY,
    YUY2,
    MJPG,
    MJPEG,
    NV12,
    NV21,
    I420,
    RGB,
    BGR,
    RGBA,
    BGRA,
    H264,
    H265,
    HEVC,
    POINT,
    RGB888,
};

// Bytes per pixel for single-plane formats (Y16=2, RGB=3, RGBA=4, etc.).
// Returns 0 for multi-plane or compressed formats (NV12/NV21/I420/MJPG/H264/POINT/UNKNOWN).
inline int nioFormatBpp(NioFormat f) {
    switch (f) {
    case NioFormat::Y8:
        return 1;
    case NioFormat::Y16:
        return 2;
    case NioFormat::YUYV:
    case NioFormat::UYVY:
    case NioFormat::YUY2:
        return 2;
    case NioFormat::RGB:
    case NioFormat::BGR:
    case NioFormat::RGB888:
        return 3;
    case NioFormat::RGBA:
    case NioFormat::BGRA:
        return 4;
    default:
        return 0;
    }
}

// Raw buffer size in bytes for a given format + resolution.
// Covers: Y8, Y16, YUYV/UYVY/YUY2, RGB/BGR/RGB888, RGBA/BGRA, NV12, NV21, I420.
// Returns 0 for MJPEG, H264, POINT, UNKNOWN (variable-size).
inline size_t nioFormatRawSize(NioFormat f, int w, int h) {
    switch (f) {
    case NioFormat::Y8:
        return static_cast<size_t>(w * h);
    case NioFormat::Y16:
        return static_cast<size_t>(w * h * 2);
    case NioFormat::YUYV:
    case NioFormat::UYVY:
    case NioFormat::YUY2:
        return static_cast<size_t>(w * h * 2);
    case NioFormat::RGB:
    case NioFormat::BGR:
    case NioFormat::RGB888:
        return static_cast<size_t>(w * h * 3);
    case NioFormat::RGBA:
    case NioFormat::BGRA:
        return static_cast<size_t>(w * h * 4);
    case NioFormat::NV12:
    case NioFormat::NV21:
        return static_cast<size_t>(w * h * 3 / 2);
    case NioFormat::I420:
        return static_cast<size_t>(w * h * 3 / 2);
    default:
        return 0;
    }
}

// Frame type — identifies the sensor source of a frame.
enum class NioFrameType {
    COLOR = 0,
    DEPTH,
    IR,
    IR_LEFT,
    IR_RIGHT,
    ACCEL,
    GYRO,
    COLOR_LEFT,
    COLOR_RIGHT,
    CONFIDENCE,
    POINT,
    COUNT
};

// Camera intrinsic parameters (3×3 pinhole model).
struct NioIntrinsic
{
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int width = 0;
    int height = 0;
};

// Stream profile — resolution + fps + format for one sensor stream.
struct NioStreamProfile
{
    int width = 0;
    int height = 0;
    int fps = 30;
    NioFormat format = NioFormat::UNKNOWN;
};

// Per-device sensor presence + profile summary.
// Replaces SensorInfo (which used OBFormat / ob::VideoStreamProfile).
struct NioSensorInfo
{
    bool hasColor = false;
    bool hasDepth = false;
    bool hasIR = false;
    bool hasIRLeft = false;
    bool hasIRRight = false;
    bool hasAccel = false;
    bool hasGyro = false;

    NioFormat colorFormat = NioFormat::UNKNOWN;
    NioFormat depthFormat = NioFormat::UNKNOWN;
    NioFormat irFormat = NioFormat::UNKNOWN;
    NioFormat irLeftFormat = NioFormat::UNKNOWN;
    NioFormat irRightFormat = NioFormat::UNKNOWN;

    int colorW = 0, colorH = 0, colorFps = 30;
    int depthW = 0, depthH = 0, depthFps = 30;
    int irW = 0, irH = 0, irFps = 30;
    int irLW = 0, irLH = 0, irLFps = 30;
    int irRW = 0, irRH = 0, irRFps = 30;

    NioIntrinsic depthIntrinsic;
    NioIntrinsic colorIntrinsic;

    float depthScale = 0.001f;
};

// Frame count map — replaces std::map<OBFrameType, uint64_t>.
using NioFrameCounts = std::map<NioFrameType, uint64_t>;

// NioFormat → string (for logging / FPS reports).
inline const char* nioFormatToStr(NioFormat f) {
    switch (f) {
    case NioFormat::Y8:
        return "Y8";
    case NioFormat::Y16:
        return "Y16";
    case NioFormat::YUYV:
        return "YUYV";
    case NioFormat::UYVY:
        return "UYVY";
    case NioFormat::YUY2:
        return "YUY2";
    case NioFormat::MJPG:
        return "MJPG";
    case NioFormat::MJPEG:
        return "MJPEG";
    case NioFormat::NV12:
        return "NV12";
    case NioFormat::NV21:
        return "NV21";
    case NioFormat::I420:
        return "I420";
    case NioFormat::RGB:
        return "RGB";
    case NioFormat::BGR:
        return "BGR";
    case NioFormat::RGBA:
        return "RGBA";
    case NioFormat::BGRA:
        return "BGRA";
    case NioFormat::H264:
        return "H264";
    case NioFormat::H265:
        return "H265";
    case NioFormat::HEVC:
        return "HEVC";
    case NioFormat::POINT:
        return "POINT";
    case NioFormat::RGB888:
        return "RGB888";
    default:
        return "UNKNOWN";
    }
}

// NioFrameType → string (for logging / FPS reports).
inline const char* nioFrameTypeToStr(NioFrameType t) {
    switch (t) {
    case NioFrameType::COLOR:
        return "COLOR";
    case NioFrameType::DEPTH:
        return "DEPTH";
    case NioFrameType::IR:
        return "IR";
    case NioFrameType::IR_LEFT:
        return "IR_LEFT";
    case NioFrameType::IR_RIGHT:
        return "IR_RIGHT";
    case NioFrameType::ACCEL:
        return "ACCEL";
    case NioFrameType::GYRO:
        return "GYRO";
    case NioFrameType::COLOR_LEFT:
        return "COLOR_LEFT";
    case NioFrameType::COLOR_RIGHT:
        return "COLOR_RIGHT";
    case NioFrameType::CONFIDENCE:
        return "CONFIDENCE";
    case NioFrameType::POINT:
        return "POINT";
    default:
        return "UNKNOWN";
    }
}

} // namespace nio
