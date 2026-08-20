// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_types.hpp — SDK-neutral value types for multi-SDK capture.
//
// [文件说明 / File Description]
// 中文：SDK中立的值类型，用于多SDK捕获，替代直接使用供应商SDK类型
// English: SDK-neutral value types for multi-SDK capture, replaces direct usage of vendor SDK types
//
// Replaces direct usage of Orbbec SDK types (OBFormat, OBFrameType,
// OBCameraIntrinsic, etc.) with SDK-agnostic equivalents.  Conversion
// functions between DynalgoFormat↔OBFormat and DynalgoFrameType↔OBFrameType
// are provided in dynalgo_ob_adapter.hpp / dynalgo_rs_adapter.hpp (driver layer only).

#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace dynalgo {

// [结构体说明 / Struct Description]
// 中文：点云字段描述符，用于描述打包点云线布局中的一个字段
// English: PcdFieldDesc: describes one field in a packed point cloud wire layout
//
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

// [结构体说明 / Struct Description]
// 中文：点云布局，PcdFieldDesc向量的便捷包装器
// English: PcdLayout: convenience wrapper around a vector of PcdFieldDesc
struct PcdLayout
{
    std::vector<PcdFieldDesc> fields;
    uint32_t srcPointSize = 0;

    // [方法说明 / Method Description]
    // 中文：添加字段到布局
    // English: Add field to layout
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

    // [方法说明 / Method Description]
    // 中文：将布局序列化到字节缓冲区（线头前缀）
    // English: Serialize this layout into a byte buffer (wire header prefix)
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

    // [方法说明 / Method Description]
    // 中文：从线数据反序列化PcdLayout，返回消耗的字节数（错误返回0）
    // English: Deserialize a PcdLayout from wire data. Returns bytes consumed (0 on error)
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

    // [静态方法 / Static Method]
    // 中文：RS-AC1布局，匹配PointXYZIRT结构体布局（24字节，强度后1字节填充）
    // English: RS-AC1 layout: matches PointXYZIRT struct layout (24 bytes with 1B padding after intensity)
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

    // [静态方法 / Static Method]
    // 中文：Orbbec XYZ-only布局，xyz(float3)，每点12字节，无需转换
    // English: Orbbec XYZ-only layout: xyz(float3), 12 bytes per point, no transform needed
    static PcdLayout obXyz() {
        PcdLayout l;
        l.addField("x", 4, 4, 'F');
        l.addField("y", 4, 4, 'F');
        l.addField("z", 4, 4, 'F');
        return l;
    }
};

// ---------------------------------------------------------------------------
// SDK-neutral type system — kept in a dedicated namespace for clarity.
// ---------------------------------------------------------------------------
namespace types {

// [枚举说明 / Enum Description]
// 中文：像素/帧格式，SDK中立
// English: Pixel / frame format — SDK-independent.
enum class DynalgoFormat {
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

// [函数说明 / Function Description]
// 中文：获取单平面格式的每像素字节数，多平面或压缩格式返回0
// English: Bytes per pixel for single-plane formats, returns 0 for multi-plane or compressed formats
inline int dynalgoFormatBpp(DynalgoFormat f) {
    switch (f) {
    case DynalgoFormat::Y8:
        return 1;
    case DynalgoFormat::Y16:
        return 2;
    case DynalgoFormat::YUYV:
    case DynalgoFormat::UYVY:
    case DynalgoFormat::YUY2:
        return 2;
    case DynalgoFormat::RGB:
    case DynalgoFormat::BGR:
    case DynalgoFormat::RGB888:
        return 3;
    case DynalgoFormat::RGBA:
    case DynalgoFormat::BGRA:
        return 4;
    default:
        return 0;
    }
}

// [函数说明 / Function Description]
// 中文：给定格式和分辨率的原始缓冲区大小（字节），可变大小格式返回0
// English: Raw buffer size in bytes for a given format + resolution, returns 0 for variable-size formats
inline size_t dynalgoFormatRawSize(DynalgoFormat f, int w, int h) {
    switch (f) {
    case DynalgoFormat::Y8:
        return static_cast<size_t>(w * h);
    case DynalgoFormat::Y16:
        return static_cast<size_t>(w * h * 2);
    case DynalgoFormat::YUYV:
    case DynalgoFormat::UYVY:
    case DynalgoFormat::YUY2:
        return static_cast<size_t>(w * h * 2);
    case DynalgoFormat::RGB:
    case DynalgoFormat::BGR:
    case DynalgoFormat::RGB888:
        return static_cast<size_t>(w * h * 3);
    case DynalgoFormat::RGBA:
    case DynalgoFormat::BGRA:
        return static_cast<size_t>(w * h * 4);
    case DynalgoFormat::NV12:
    case DynalgoFormat::NV21:
        return static_cast<size_t>(w * h * 3 / 2);
    case DynalgoFormat::I420:
        return static_cast<size_t>(w * h * 3 / 2);
    default:
        return 0;
    }
}

// [枚举说明 / Enum Description]
// 中文：帧类型，标识帧的传感器来源
// English: Frame type — identifies the sensor source of a frame.
enum class DynalgoFrameType {
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

} // namespace types

using types::DynalgoFormat;
using types::DynalgoFrameType;

// [结构体说明 / Struct Description]
// 中文：相机内参（3×3针孔模型）
// English: Camera intrinsic parameters (3×3 pinhole model)
struct DynalgoIntrinsic
{
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int width = 0;
    int height = 0;
};

// [结构体说明 / Struct Description]
// 中文：流配置文件，包含分辨率、帧率和格式
// English: Stream profile — resolution + fps + format for one sensor stream
struct DynalgoStreamProfile
{
    int width = 0;
    int height = 0;
    int fps = 30;
    DynalgoFormat format = DynalgoFormat::UNKNOWN;
};

// [结构体说明 / Struct Description]
// 中文：每设备传感器存在性和配置文件摘要，替代供应商特定的SensorInfo类型
// English: Per-device sensor presence + profile summary, replaces vendor-specific SensorInfo types
struct DynalgoSensorInfo
{
    bool hasColor = false;
    bool hasDepth = false;
    bool hasIR = false;
    bool hasIRLeft = false;
    bool hasIRRight = false;
    bool hasAccel = false;
    bool hasGyro = false;

    DynalgoFormat colorFormat = DynalgoFormat::UNKNOWN;
    DynalgoFormat depthFormat = DynalgoFormat::UNKNOWN;
    DynalgoFormat irFormat = DynalgoFormat::UNKNOWN;
    DynalgoFormat irLeftFormat = DynalgoFormat::UNKNOWN;
    DynalgoFormat irRightFormat = DynalgoFormat::UNKNOWN;

    int colorW = 0, colorH = 0, colorFps = 30;
    int depthW = 0, depthH = 0, depthFps = 30;
    int irW = 0, irH = 0, irFps = 30;
    int irLW = 0, irLH = 0, irLFps = 30;
    int irRW = 0, irRH = 0, irRFps = 30;

    DynalgoIntrinsic depthIntrinsic;
    DynalgoIntrinsic colorIntrinsic;

    float depthScale = 0.001f;
};

// [类型别名 / Type Alias]
// 中文：帧计数映射，替代std::map<OBFrameType, uint64_t>
// English: Frame count map — replaces std::map<OBFrameType, uint64_t>
using DynalgoFrameCounts = std::map<DynalgoFrameType, uint64_t>;

// [函数说明 / Function Description]
// 中文：DynalgoFormat转字符串，用于日志和FPS报告
// English: DynalgoFormat → string (for logging / FPS reports)
inline const char* dynalgoFormatToStr(DynalgoFormat f) {
    switch (f) {
    case DynalgoFormat::Y8:
        return "Y8";
    case DynalgoFormat::Y16:
        return "Y16";
    case DynalgoFormat::YUYV:
        return "YUYV";
    case DynalgoFormat::UYVY:
        return "UYVY";
    case DynalgoFormat::YUY2:
        return "YUY2";
    case DynalgoFormat::MJPG:
        return "MJPG";
    case DynalgoFormat::MJPEG:
        return "MJPEG";
    case DynalgoFormat::NV12:
        return "NV12";
    case DynalgoFormat::NV21:
        return "NV21";
    case DynalgoFormat::I420:
        return "I420";
    case DynalgoFormat::RGB:
        return "RGB";
    case DynalgoFormat::BGR:
        return "BGR";
    case DynalgoFormat::RGBA:
        return "RGBA";
    case DynalgoFormat::BGRA:
        return "BGRA";
    case DynalgoFormat::H264:
        return "H264";
    case DynalgoFormat::H265:
        return "H265";
    case DynalgoFormat::HEVC:
        return "HEVC";
    case DynalgoFormat::POINT:
        return "POINT";
    case DynalgoFormat::RGB888:
        return "RGB888";
    default:
        return "UNKNOWN";
    }
}

// [函数说明 / Function Description]
// 中文：DynalgoFrameType转字符串，用于日志和FPS报告
// English: DynalgoFrameType → string (for logging / FPS reports)
inline const char* dynalgoFrameTypeToStr(DynalgoFrameType t) {
    switch (t) {
    case DynalgoFrameType::COLOR:
        return "COLOR";
    case DynalgoFrameType::DEPTH:
        return "DEPTH";
    case DynalgoFrameType::IR:
        return "IR";
    case DynalgoFrameType::IR_LEFT:
        return "IR_LEFT";
    case DynalgoFrameType::IR_RIGHT:
        return "IR_RIGHT";
    case DynalgoFrameType::ACCEL:
        return "ACCEL";
    case DynalgoFrameType::GYRO:
        return "GYRO";
    case DynalgoFrameType::COLOR_LEFT:
        return "COLOR_LEFT";
    case DynalgoFrameType::COLOR_RIGHT:
        return "COLOR_RIGHT";
    case DynalgoFrameType::CONFIDENCE:
        return "CONFIDENCE";
    case DynalgoFrameType::POINT:
        return "POINT";
    default:
        return "UNKNOWN";
    }
}

} // namespace dynalgo
