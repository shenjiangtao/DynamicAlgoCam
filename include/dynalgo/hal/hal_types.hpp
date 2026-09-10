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
#include <variant>
#include <array>
#include <atomic>
#include <map>
#include <cmath>

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

enum class FrameType : uint32_t {
    UNKNOWN = 0,
    IMAGE = 1,
    POINT_CLOUD = 2,
    DEPTH_MAP = 3,
    DISPARITY = 4,
    IMU = 5,
    FUSED = 6,
};

struct FrameMetadata {
    uint64_t timestamp_ns = 0;
    uint32_t frame_id = 0;
    uint32_t sensor_id = 0;
    FrameType frame_type = FrameType::UNKNOWN;
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

// ============================================================================
// UPEP: Coordinate Frames and Transformations
// ============================================================================

enum class CoordinateFrame : uint32_t {
    UNKNOWN = 0,
    SENSOR = 1,           // Native sensor frame
    VEHICLE = 2,          // Vehicle body frame (FLU: Forward-Left-Up)
    WORLD = 3,            // World frame (ENU: East-North-Up)
    CAMERA = 4,           // Camera optical frame
    LIDAR = 5,            // LiDAR frame
    IMU = 6,              // IMU frame
};

struct Transform3D {
    std::array<float, 9> rotation;      // 3x3 rotation matrix (row-major)
    std::array<float, 3> translation;   // Translation vector (meters)
    
    Transform3D() {
        rotation = {1,0,0, 0,1,0, 0,0,1};
        translation = {0,0,0};
    }
    
    // Apply transform: point_out = R * point_in + t
    void transformPoint(const float* in, float* out) const {
        out[0] = rotation[0]*in[0] + rotation[1]*in[1] + rotation[2]*in[2] + translation[0];
        out[1] = rotation[3]*in[0] + rotation[4]*in[1] + rotation[5]*in[2] + translation[1];
        out[2] = rotation[6]*in[0] + rotation[7]*in[1] + rotation[8]*in[2] + translation[2];
    }
    
    // Inverse transform
    Transform3D inverse() const {
        Transform3D inv;
        // Transpose rotation
        inv.rotation[0] = rotation[0]; inv.rotation[1] = rotation[3]; inv.rotation[2] = rotation[6];
        inv.rotation[3] = rotation[1]; inv.rotation[4] = rotation[4]; inv.rotation[5] = rotation[7];
        inv.rotation[6] = rotation[2]; inv.rotation[7] = rotation[5]; inv.rotation[8] = rotation[8];
        // -R^T * t
        inv.translation[0] = -(inv.rotation[0]*translation[0] + inv.rotation[1]*translation[1] + inv.rotation[2]*translation[2]);
        inv.translation[1] = -(inv.rotation[3]*translation[0] + inv.rotation[4]*translation[1] + inv.rotation[5]*translation[2]);
        inv.translation[2] = -(inv.rotation[6]*translation[0] + inv.rotation[7]*translation[1] + inv.rotation[8]*translation[2]);
        return inv;
    }
    
    // Compose: this * other
    Transform3D compose(const Transform3D& other) const {
        Transform3D result;
        // R = R1 * R2
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.rotation[i*3+j] = 0;
                for (int k = 0; k < 3; ++k) {
                    result.rotation[i*3+j] += rotation[i*3+k] * other.rotation[k*3+j];
                }
            }
        }
        // t = R1 * t2 + t1
        result.translation[0] = rotation[0]*other.translation[0] + rotation[1]*other.translation[1] + rotation[2]*other.translation[2] + translation[0];
        result.translation[1] = rotation[3]*other.translation[0] + rotation[4]*other.translation[1] + rotation[5]*other.translation[2] + translation[1];
        result.translation[2] = rotation[6]*other.translation[0] + rotation[7]*other.translation[1] + rotation[8]*other.translation[2] + translation[2];
        return result;
    }
    
    static Transform3D fromEuler(float roll, float pitch, float yaw) {
        Transform3D t;
        float cr = std::cos(roll), sr = std::sin(roll);
        float cp = std::cos(pitch), sp = std::sin(pitch);
        float cy = std::cos(yaw), sy = std::sin(yaw);
        
        t.rotation[0] = cy * cp;
        t.rotation[1] = cy * sp * sr - sy * cr;
        t.rotation[2] = cy * sp * cr + sy * sr;
        t.rotation[3] = sy * cp;
        t.rotation[4] = sy * sp * sr + cy * cr;
        t.rotation[5] = sy * sp * cr - cy * sr;
        t.rotation[6] = -sp;
        t.rotation[7] = cp * sr;
        t.rotation[8] = cp * cr;
        return t;
    }
};

// Camera intrinsic parameters
struct CameraIntrinsic {
    float fx = 0.0f, fy = 0.0f;
    float cx = 0.0f, cy = 0.0f;
    uint32_t width = 0, height = 0;
    
    // Project 3D point to 2D pixel
    bool project(const float* point3d, float* u, float* v) const {
        if (point3d[2] <= 0) return false;
        *u = fx * point3d[0] / point3d[2] + cx;
        *v = fy * point3d[1] / point3d[2] + cy;
        return (*u >= 0 && *u < width && *v >= 0 && *v < height);
    }
    
    // Unproject 2D pixel + depth to 3D point
    void unproject(float u, float v, float depth, float* point3d) const {
        point3d[0] = (u - cx) * depth / fx;
        point3d[1] = (v - cy) * depth / fy;
        point3d[2] = depth;
    }
};

struct CalibrationData {
    std::string sensor_id;
    CoordinateFrame frame;
    CameraIntrinsic intrinsic;
    Transform3D extrinsics;           // Sensor to vehicle frame
    Transform3D sensor_to_camera;     // For LiDAR-Camera calibration
    
    // Temporal calibration
    int64_t time_offset_ns = 0;       // Sensor clock offset from reference
    float time_scale = 1.0f;          // Sensor clock scale factor
    
    // Validation
    bool is_valid() const { return !sensor_id.empty() && intrinsic.width > 0 && intrinsic.height > 0; }
};

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>;

inline uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline TimePoint now_tp() {
    return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now());
}

// ============================================================================
// UPEP: Unified FrameBuffer for zero-copy multi-modal data (Image + PointCloud)
// ============================================================================

enum class DataLayout : uint32_t {
    UNKNOWN = 0,
    ROW_MAJOR = 1,      // Images: row-major pixel layout
    COLUMN_MAJOR = 2,   // Column-major (rare)
    BLOCK_LINEAR = 3,   // GPU tiled layouts
    PITCH_LINEAR = 4,   // Linear with pitch
    STRUCTURED = 5,     // Structured buffer (point cloud with fields)
    CUSTOM = 0xFFFF,
};

struct PointCloudField {
    std::string name;           // e.g., "x", "y", "z", "intensity", "ring", "timestamp"
    uint32_t offset = 0;        // Byte offset within point struct
    uint32_t size = 0;          // Size in bytes (4 for float32, 2 for uint16, etc.)
    uint32_t count = 1;         // Number of elements (1 for scalar, 3 for vec3)
    std::string datatype;       // "FLOAT32", "UINT16", "INT32", etc.
};

struct PointCloudLayout {
    std::vector<PointCloudField> fields;
    uint32_t point_size = 0;        // Total bytes per point
    uint32_t point_count = 0;       // Number of points
    DataLayout layout = DataLayout::STRUCTURED;
    bool is_dense = true;           // No invalid points
    float min_range = 0.0f;
    float max_range = 0.0f;
};

// Unified FrameBuffer supporting both images and point clouds
struct FrameBuffer {
    FrameMetadata metadata;
    
    // Image data (when frame_type == IMAGE or DEPTH_MAP)
    std::vector<uint8_t> image_data;  // Raw pixel data (owned for CPU, handle for GPU)
    bool image_owns_data = true;
    
    // Point cloud data (when frame_type == POINT_CLOUD)
    PointCloudLayout pc_layout;
    std::vector<uint8_t> point_data;  // Raw point data (owned for CPU, handle for GPU)
    bool pc_owns_data = true;
    
    // Synchronization
    FenceHandle acquire_fence;
    FenceHandle release_fence;
    
    // Multi-sensor sync info
    uint64_t sync_group_id = 0;       // Frames with same sync_group_id are temporally synced
    uint64_t reference_timestamp_ns = 0;  // Reference time for sync
    
    // Reference counting for zero-copy sharing
    std::atomic<uint32_t> ref_count{1};
    
    FrameBuffer() = default;
    FrameBuffer(const FrameBuffer&) = delete;
    FrameBuffer& operator=(const FrameBuffer&) = delete;
    FrameBuffer(FrameBuffer&& other) noexcept = default;
    FrameBuffer& operator=(FrameBuffer&& other) noexcept = default;
    
    void addRef() { ref_count.fetch_add(1, std::memory_order_relaxed); }
    bool release() { 
        auto prev = ref_count.fetch_sub(1, std::memory_order_acq_rel);
        return prev == 1;
    }
    uint32_t getRefCount() const { return ref_count.load(std::memory_order_relaxed); }
    
    bool isImage() const { return metadata.frame_type == FrameType::IMAGE || metadata.frame_type == FrameType::DEPTH_MAP; }
    bool isPointCloud() const { return metadata.frame_type == FrameType::POINT_CLOUD; }
    bool isFused() const { return metadata.frame_type == FrameType::FUSED; }
    
    size_t getImageSize() const {
        if (!isImage()) return 0;
        return image_data.size();
    }
    
    size_t getPointCloudSize() const {
        if (!isPointCloud()) return 0;
        return point_data.size();
    }
    
    uint32_t getPointCount() const {
        if (!isPointCloud()) return 0;
        return pc_layout.point_count;
    }
    
    const PointCloudField* getField(const std::string& name) const {
        for (const auto& f : pc_layout.fields) {
            if (f.name == name) return &f;
        }
        return nullptr;
    }
    
    template<typename T>
    const T* getPointFieldPtr(uint32_t point_idx, const std::string& field_name) const {
        if (!isPointCloud()) return nullptr;
        const auto* field = getField(field_name);
        if (!field) return nullptr;
        if (point_idx >= pc_layout.point_count) return nullptr;
        const uint8_t* base = point_data.data() + point_idx * pc_layout.point_size + field->offset;
        return reinterpret_cast<const T*>(base);
    }
};

using FrameBufferPtr = std::shared_ptr<FrameBuffer>;

// ============================================================================
// UPEP: Multi-sensor synchronization types
// ============================================================================

enum class SyncMethod : uint32_t {
    HARDWARE_TRIGGER = 0,    // Hardware GPIO/PPS sync
    PTP = 1,                 // IEEE 1588 PTP
    SOFTWARE_TIMESTAMP = 2,  // Software timestamp matching
    NTP = 3,                 // NTP sync
    GPS_PPS = 4,             // GPS 1PPS
};

struct SyncConfig {
    SyncMethod method = SyncMethod::HARDWARE_TRIGGER;
    uint32_t sync_group_id = 0;        // Sensors in same group sync together
    uint64_t max_time_diff_ns = 1000000; // 1ms max time diff for sync
    bool enable_interpolation = true;  // Interpolate if timestamps don't match exactly
    std::string ptp_interface = "eth0"; // PTP network interface
    uint32_t ptp_domain = 0;
    int gpio_sync_pin = -1;
    std::string gpio_sync_polarity = "rising";
    bool hardware_trigger = false;
};

struct SynchronizedFrameSet {
    uint64_t sync_timestamp_ns = 0;   // Reference timestamp (e.g., PTP master)
    uint64_t sync_group_id = 0;
    
    FrameBufferPtr camera_frame;      // Main camera
    FrameBufferPtr depth_frame;       // Depth/stereo
    FrameBufferPtr lidar_frame;       // LiDAR point cloud
    FrameBufferPtr imu_frame;         // IMU data
    
    // Per-sensor calibration
    std::map<std::string, CalibrationData> calibrations;
    
    // Convenience
    bool hasCamera() const { return camera_frame && camera_frame->isImage(); }
    bool hasDepth() const { return depth_frame && depth_frame->isImage(); }
    bool hasLidar() const { return lidar_frame && lidar_frame->isPointCloud(); }
    bool hasImu() const { return imu_frame && imu_frame->isImage(); }
    
    size_t sensorCount() const {
        size_t count = 0;
        if (hasCamera()) ++count;
        if (hasDepth()) ++count;
        if (hasLidar()) ++count;
        if (hasImu()) ++count;
        return count;
    }
    
    // Legacy vector-based access (for backward compatibility)
    std::vector<FrameBufferPtr> frames() const {
        std::vector<FrameBufferPtr> result;
        if (camera_frame) result.push_back(camera_frame);
        if (depth_frame) result.push_back(depth_frame);
        if (lidar_frame) result.push_back(lidar_frame);
        if (imu_frame) result.push_back(imu_frame);
        return result;
    }
    
    FrameBufferPtr getImage(uint32_t sensor_id = 0) const {
        if (camera_frame && camera_frame->metadata.sensor_id == sensor_id) return camera_frame;
        if (depth_frame && depth_frame->metadata.sensor_id == sensor_id) return depth_frame;
        return nullptr;
    }
    
    FrameBufferPtr getDepth(uint32_t sensor_id = 0) const {
        if (depth_frame && depth_frame->metadata.sensor_id == sensor_id) return depth_frame;
        return nullptr;
    }
    
    FrameBufferPtr getPointCloud(uint32_t sensor_id = 0) const {
        if (lidar_frame && lidar_frame->metadata.sensor_id == sensor_id) return lidar_frame;
        return nullptr;
    }
    
    };

using SynchronizedFrameSetPtr = std::shared_ptr<SynchronizedFrameSet>;

} // namespace dynalgo::hal

#endif // DYNALOGO_HAL_TYPES_HPP