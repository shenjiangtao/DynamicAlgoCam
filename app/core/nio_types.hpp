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
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace nio {

// PcdFieldDesc: describes one field in a packed point cloud wire layout.
// Used by writePcdFile() to generate PCD headers and transform source data.
// Wire format for POINT frames:
//   [4B pointCount (uint32)] [4B srcPointSize (uint32)] [4B numFields (uint32)]
//   [numFields * sizeof(PcdFieldDesc)]
//   [pointCount * srcPointSize — packed point data]
struct PcdFieldDesc
{
    char name[16] = {};    // field name, e.g. "x", "intensity", "ring", "timestamp"
    uint8_t srcSize = 0;   // bytes in source (wire) data: 1/2/4/8
    uint8_t srcOffset = 0; // byte offset within the source point struct
    uint8_t pcdSize = 0;   // bytes in output PCD: e.g. 4 when src uint8→float32
    char pcdType = 'F';    // PCD TYPE char: 'F'=float, 'U'=unsigned, 'I'=signed
    uint8_t pad_[4] = {};  // zero-padded to 24 bytes
};
static_assert(sizeof(PcdFieldDesc) == 24, "PcdFieldDesc must be 24 bytes for wire compatibility");

// PcdLayout: convenience wrapper around a vector of PcdFieldDesc.
struct PcdLayout
{
    std::vector<PcdFieldDesc> fields;
    uint32_t srcPointSize = 0;

    void addField(const char* name, uint8_t srcSize, uint8_t pcdSize, char pcdType) {
        PcdFieldDesc d;
        std::strncpy(d.name, name, sizeof(d.name) - 1);
        d.srcSize = srcSize;
        d.srcOffset = srcPointSize;
        d.pcdSize = pcdSize;
        d.pcdType = pcdType;
        fields.push_back(d);
        srcPointSize += srcSize;
    }

    uint32_t pcdPointSize() const {
        uint32_t s = 0;
        for (auto& f : fields)
            s += f.pcdSize;
        return s;
    }

    // Serialize this layout into a byte buffer (wire header prefix).
    void serialize(std::vector<uint8_t>& out) const {
        uint32_t n = static_cast<uint32_t>(fields.size());
        size_t hdrSize = 12 + n * sizeof(PcdFieldDesc);
        size_t oldSize = out.size();
        out.resize(oldSize + hdrSize);
        uint8_t* p = out.data() + oldSize;
        std::memcpy(p, &srcPointSize, 4);
        p += 4;
        std::memcpy(p, &n, 4);
        p += 4;
        // placeholder for pointCount — caller writes it before the point data
        std::memset(p, 0, 4);
        p += 4;
        if (!fields.empty())
            std::memcpy(p, fields.data(), n * sizeof(PcdFieldDesc));
    }

    // Deserialize a PcdLayout from wire data. Returns bytes consumed (0 on error).
    // Expects: [srcPointSize(4)] [numFields(4)] [pointCount(4)] [fields...]
    static size_t deserialize(const uint8_t* data, size_t size, PcdLayout& layout, uint32_t& pointCount) {
        if (size < 12)
            return 0;
        const uint8_t* p = data;
        std::memcpy(&layout.srcPointSize, p, 4);
        p += 4;
        uint32_t n = 0;
        std::memcpy(&n, p, 4);
        p += 4;
        std::memcpy(&pointCount, p, 4);
        p += 4;
        size_t fieldsBytes = static_cast<size_t>(n) * sizeof(PcdFieldDesc);
        if (size < 12 + fieldsBytes)
            return 0;
        layout.fields.resize(n);
        std::memcpy(layout.fields.data(), p, fieldsBytes);
        p += fieldsBytes;
        return static_cast<size_t>(p - data);
    }

    // RS-AC1 layout: matches PointXYZIRT struct layout (24 bytes with 1B padding after intensity)
    // Offsets must match the compiler's layout, not a packed sum.
    static PcdLayout rsAc1() {
        PcdLayout l;
        l.srcPointSize = 24;
        l.fields.resize(6);
        auto& f = l.fields;
        std::strncpy(f[0].name, "x", 15);
        f[0].srcSize = 4;
        f[0].srcOffset = 0;
        f[0].pcdSize = 4;
        f[0].pcdType = 'F';
        std::strncpy(f[1].name, "y", 15);
        f[1].srcSize = 4;
        f[1].srcOffset = 4;
        f[1].pcdSize = 4;
        f[1].pcdType = 'F';
        std::strncpy(f[2].name, "z", 15);
        f[2].srcSize = 4;
        f[2].srcOffset = 8;
        f[2].pcdSize = 4;
        f[2].pcdType = 'F';
        std::strncpy(f[3].name, "intensity", 15);
        f[3].srcSize = 1;
        f[3].srcOffset = 12;
        f[3].pcdSize = 4;
        f[3].pcdType = 'F';
        std::strncpy(f[4].name, "ring", 15);
        f[4].srcSize = 2;
        f[4].srcOffset = 14;
        f[4].pcdSize = 2;
        f[4].pcdType = 'U';
        std::strncpy(f[5].name, "timestamp", 15);
        f[5].srcSize = 8;
        f[5].srcOffset = 16;
        f[5].pcdSize = 8;
        f[5].pcdType = 'F';
        return l;
    }

    // Orbbec XYZ-only layout: xyz(float3), 12 bytes per point, no transform needed
    static PcdLayout obXyz() {
        PcdLayout l;
        l.addField("x", 4, 4, 'F');
        l.addField("y", 4, 4, 'F');
        l.addField("z", 4, 4, 'F');
        return l;
    }
};

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
// Replaces vendor-specific SensorInfo types.
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
