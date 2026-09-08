#pragma once
#ifndef DYNALOGO_HAL_TYPES_HPP
#define DYNALOGO_HAL_TYPES_HPP

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

namespace dynalgo::hal {

constexpr uint32_t HAL_VERSION_MAJOR = 1;
constexpr uint32_t HAL_VERSION_MINOR = 0;
constexpr uint32_t HAL_VERSION_PATCH = 0;

inline constexpr uint32_t HAL_VERSION() {
    return (HAL_VERSION_MAJOR << 16) | (HAL_VERSION_MINOR << 8) | HAL_VERSION_PATCH;
}

enum ErrorCode : uint32_t {
    OK = 0,
    INVALID_ARG = 1,
    NOT_INITIALIZED = 2,
    ALREADY_INITIALIZED = 3,
    NOT_SUPPORTED = 4,
    TIMEOUT = 5,
    OUT_OF_MEMORY = 6,
    DEVICE_ERROR = 7,
    PERMISSION_DENIED = 8,
    BUSY = 9,
    NO_DATA = 10,
    INVALID_STATE = 11,
    NOT_FOUND = 12,
    IO_ERROR = 13,
    NOT_IMPLEMENTED = 14,
    VENDOR_BASE = 0x10000
};

inline const char* error_code_to_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "OK";
        case ErrorCode::INVALID_ARG: return "INVALID_ARG";
        case ErrorCode::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case ErrorCode::ALREADY_INITIALIZED: return "ALREADY_INITIALIZED";
        case ErrorCode::NOT_SUPPORTED: return "NOT_SUPPORTED";
        case ErrorCode::TIMEOUT: return "TIMEOUT";
        case ErrorCode::OUT_OF_MEMORY: return "OUT_OF_MEMORY";
        case ErrorCode::DEVICE_ERROR: return "DEVICE_ERROR";
        case ErrorCode::PERMISSION_DENIED: return "PERMISSION_DENIED";
        case ErrorCode::BUSY: return "BUSY";
        case ErrorCode::NO_DATA: return "NO_DATA";
        case ErrorCode::INVALID_STATE: return "INVALID_STATE";
        case ErrorCode::NOT_FOUND: return "NOT_FOUND";
        case ErrorCode::IO_ERROR: return "IO_ERROR";
        default: return "UNKNOWN";
    }
}

template<typename T>
class Result {
    uint32_t error_code_;
    T value_;
public:
    Result() : error_code_(static_cast<uint32_t>(ErrorCode::OK)), value_() {}
    Result(uint32_t err) : error_code_(err), value_() {}
    Result(uint32_t err, T val) : error_code_(err), value_(std::move(val)) {}

    static Result ok(T v) { return Result(static_cast<uint32_t>(ErrorCode::OK), std::move(v)); }
    static Result err(uint32_t code) { return Result(code); }
    static Result err(ErrorCode code) { return Result(static_cast<uint32_t>(code)); }
    static Result make_error(ErrorCode code) { return Result(static_cast<uint32_t>(code)); }

    bool is_ok() const { return error_code_ == static_cast<uint32_t>(ErrorCode::OK); }
    bool is_err() const { return error_code_ != static_cast<uint32_t>(ErrorCode::OK); }
    uint32_t error() const { return error_code_; }
    ErrorCode error_code() const { return static_cast<ErrorCode>(error_code_); }

    T& value() { return value_; }
    const T& value() const { return value_; }
    T* operator->() { return &value_; }
    const T* operator->() const { return &value_; }
    T& operator*() { return value_; }
    const T& operator*() const { return value_; }
};

template<>
class Result<void> {
    uint32_t error_code_;
public:
    Result() : error_code_(static_cast<uint32_t>(ErrorCode::OK)) {}
    Result(uint32_t err) : error_code_(err) {}

    static Result ok() { return Result(static_cast<uint32_t>(ErrorCode::OK)); }
    static Result err(uint32_t code) { return Result(code); }
    static Result err(ErrorCode code) { return Result(static_cast<uint32_t>(code)); }
    static Result make_error(uint32_t code) { return Result(code); }
    static Result make_error(ErrorCode code) { return Result(static_cast<uint32_t>(code)); }

    bool is_ok() const { return error_code_ == static_cast<uint32_t>(ErrorCode::OK); }
    bool is_err() const { return error_code_ != static_cast<uint32_t>(ErrorCode::OK); }
    uint32_t error() const { return error_code_; }
    ErrorCode error_code() const { return static_cast<ErrorCode>(error_code_); }
};

using ResultVoid = Result<void>;

struct BufferHandle {
    uint64_t handle = 0;
    uint32_t platform_id = 0;
    void* reserved = nullptr;

    bool is_valid() const { return handle != 0; }
    bool operator==(const BufferHandle& other) const {
        return handle == other.handle && platform_id == other.platform_id;
    }
    bool operator!=(const BufferHandle& other) const { return !(*this == other); }
};

struct FenceHandle {
    uint64_t handle = 0;
    uint32_t platform_id = 0;

    bool is_valid() const { return handle != 0; }
    bool operator==(const FenceHandle& other) const {
        return handle == other.handle && platform_id == other.platform_id;
    }
    bool operator!=(const FenceHandle& other) const { return !(*this == other); }
};

enum class PixelFormat : uint32_t {
    UNKNOWN = 0,
    Y8 = 0x38595659,
    Y16 = 0x36315959,
    NV12 = 0x3231564E,
    NV21 = 0x3132564E,
    YUYV = 0x56595559,
    UYVY = 0x59565955,
    YVYU = 0x55595659,
    VYUY = 0x59555956,
    RGB24 = 0x33324247,
    BGR24 = 0x33324242,
    RGBA32 = 0x41524742,
    BGRA32 = 0x42475241,
    ARGB32 = 0x41424752,
    ABGR32 = 0x41475242,
    RAW8 = 0x38574152,
    RAW10 = 0x30315741,
    RAW12 = 0x32315741,
    RAW14 = 0x34315741,
    RAW16 = 0x36315741,
    H264 = 0x34363248,
    H265 = 0x35363248,
    MJPEG = 0x47454A4D,
    METADATA = 0x4154414D,
    DEPTH16 = 0x36315044,
    DEPTH32F = 0x46323344,
    P010 = 0x30313050,
    P016 = 0x36313050,
};

inline uint32_t pixel_format_bpp(PixelFormat fmt) {
    switch (fmt) {
        case PixelFormat::Y8: case PixelFormat::RAW8: return 8;
        case PixelFormat::Y16: case PixelFormat::RAW10: case PixelFormat::RAW12: case PixelFormat::RAW16: case PixelFormat::DEPTH16: return 16;
        case PixelFormat::NV12: case PixelFormat::NV21: case PixelFormat::P010: return 12;
        case PixelFormat::P016: return 16;
        case PixelFormat::YUYV: case PixelFormat::UYVY: case PixelFormat::YVYU: case PixelFormat::VYUY: return 16;
        case PixelFormat::RGB24: case PixelFormat::BGR24: return 24;
        case PixelFormat::RGBA32: case PixelFormat::BGRA32: case PixelFormat::ARGB32: case PixelFormat::ABGR32: return 32;
        case PixelFormat::DEPTH32F: return 32;
        case PixelFormat::H264: case PixelFormat::H265: case PixelFormat::MJPEG: return 0;
        default: return 0;
    }
}

inline bool is_yuv_format(PixelFormat fmt) {
    return fmt == PixelFormat::NV12 || fmt == PixelFormat::NV21 ||
           fmt == PixelFormat::YUYV || fmt == PixelFormat::UYVY ||
           fmt == PixelFormat::YVYU || fmt == PixelFormat::VYUY ||
           fmt == PixelFormat::P010 || fmt == PixelFormat::P016;
}

inline bool is_raw_format(PixelFormat fmt) {
    return fmt == PixelFormat::RAW8 || fmt == PixelFormat::RAW10 ||
           fmt == PixelFormat::RAW12 || fmt == PixelFormat::RAW14 ||
           fmt == PixelFormat::RAW16;
}

inline bool is_compressed_format(PixelFormat fmt) {
    return fmt == PixelFormat::H264 || fmt == PixelFormat::H265 || fmt == PixelFormat::MJPEG;
}

struct PlaneInfo {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    uint32_t offset = 0;
    uint32_t size = 0;
    uint32_t bpp = 0;
};

struct FrameMetadata {
    uint64_t timestamp_ns = 0;
    uint32_t frame_id = 0;
    uint32_t sensor_id = 0;
    PixelFormat format = PixelFormat::UNKNOWN;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    BufferHandle buffer;
    FenceHandle acquire_fence;
    FenceHandle release_fence;
    std::vector<PlaneInfo> planes;
    void* vendor_data = nullptr;
    size_t vendor_data_size = 0;

    uint32_t get_bpp() const { return pixel_format_bpp(format); }
    size_t get_frame_size() const {
        if (is_compressed_format(format)) return 0;
        size_t size = 0;
        for (const auto& p : planes) size += p.size;
        if (size == 0 && stride && height) size = stride * height;
        return size;
    }
};

struct StreamConfig {
    uint32_t stream_id = 0;
    PixelFormat format = PixelFormat::NV12;
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t fps = 30;
    uint32_t buffer_count = 4;
    bool hardware_sync = true;
};

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>;

inline uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline TimePoint now_tp() {
    return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
}

} // namespace dynalgo::hal

#endif // DYNALOGO_HAL_TYPES_HPP