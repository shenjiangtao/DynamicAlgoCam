#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <functional>
#include <variant>

namespace dynalgo::hal {

enum class SensorType : uint32_t {
    UNKNOWN = 0,
    IMU = 1,
    ACCELEROMETER = 2,
    GYROSCOPE = 3,
    MAGNETOMETER = 4,
    GPS = 5,
    BAROMETER = 6,
    THERMOMETER = 7,
    HUMIDITY = 8,
    PROXIMITY = 9,
    LIGHT = 10,
    CAMERA = 11,
    LIDAR = 12,
    RADAR = 13,
    ULTRASONIC = 14,
    ENCODER = 15,
    FORCE_TORQUE = 16,
    CUSTOM = 0xFFFF,
};

struct SensorConfig {
    std::string device_id;
    std::string vendor;
    SensorType type = SensorType::UNKNOWN;
    uint32_t sample_rate_hz = 100;
    uint32_t buffer_size = 1000;
    bool hardware_timestamp = true;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct IMUData {
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
    float temperature = 0.0f;
    uint64_t timestamp_ns = 0;
    uint32_t sample_id = 0;
};

struct GPSData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    float speed = 0.0f;
    float heading = 0.0f;
    float hdop = 0.0f;
    float vdop = 0.0f;
    uint32_t satellites = 0;
    uint32_t fix_type = 0;
    uint64_t timestamp_ns = 0;
};

struct BarometerData {
    float pressure = 0.0f;
    float temperature = 0.0f;
    float altitude = 0.0f;
    uint64_t timestamp_ns = 0;
};

// UPEP: LiDAR point cloud data
struct LidarPoint {
    float x = 0.0f, y = 0.0f, z = 0.0f;      // Cartesian coordinates (meters)
    float intensity = 0.0f;                   // Reflectivity/intensity
    uint16_t ring = 0;                        // Laser ring/channel index
    float azimuth = 0.0f;                     // Azimuth angle (degrees)
    float elevation = 0.0f;                   // Elevation angle (degrees)
    float range = 0.0f;                       // Distance from sensor (meters)
    uint64_t timestamp_ns = 0;                // Point timestamp (for time-sync)
};

struct LidarData {
    std::vector<LidarPoint> points;
    uint32_t point_count = 0;
    uint32_t ring_count = 0;
    float h_fov_start = 0.0f;      // Horizontal FOV start (degrees)
    float h_fov_end = 360.0f;      // Horizontal FOV end (degrees)
    float v_fov_start = -25.0f;    // Vertical FOV start (degrees)
    float v_fov_end = 15.0f;       // Vertical FOV end (degrees)
    float angular_resolution = 0.0f;
    uint64_t timestamp_ns = 0;
    uint32_t frame_id = 0;
    std::string coordinate_frame;  // e.g., "sensor", "vehicle", "world"
    void* vendor_data = nullptr;
    size_t vendor_data_size = 0;
};

using SensorDataVariant = std::variant<
    std::monostate,
    IMUData,
    GPSData,
    BarometerData,
    LidarData
>;

struct SensorData {
    SensorType type = SensorType::UNKNOWN;
    uint64_t timestamp_ns = 0;
    uint32_t sample_id = 0;
    SensorDataVariant data;
    void* vendor_data = nullptr;
    size_t vendor_data_size = 0;
};

// UPEP: Unified sensor interface using FrameBuffer for zero-copy
class ISensorHAL {
public:
    virtual ~ISensorHAL() = default;

    virtual ResultVoid initialize(const SensorConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual ResultVoid start() = 0;
    virtual ResultVoid stop() = 0;

    // Legacy API (for backward compatibility)
    virtual Result<SensorData> read(uint32_t timeout_ms = 100) = 0;
    virtual Result<std::vector<SensorData>> readBatch(uint32_t count, uint32_t timeout_ms = 100) = 0;

    using DataCallback = std::function<void(const SensorData&)>;
    virtual ResultVoid registerCallback(DataCallback cb) = 0;
    virtual ResultVoid unregisterCallback() = 0;

    // UPEP: New zero-copy FrameBuffer API
    virtual Result<FrameBufferPtr> acquireFrameBuffer(uint32_t timeout_ms = 100) = 0;
    virtual Result<std::vector<FrameBufferPtr>> acquireFrameBuffers(uint32_t count, uint32_t timeout_ms = 100) = 0;
    virtual ResultVoid releaseFrameBuffer(const FrameBufferPtr& frame) = 0;

    using FrameBufferCallback = std::function<void(const FrameBufferPtr&)>;
    virtual ResultVoid registerFrameBufferCallback(FrameBufferCallback cb) = 0;
    virtual ResultVoid unregisterFrameBufferCallback() = 0;

    virtual ResultVoid setSampleRate(uint32_t hz) = 0;
    virtual Result<uint32_t> getSampleRate() const = 0;

    virtual ResultVoid calibrate() = 0;
    virtual ResultVoid setOffset(const SensorData& offset) = 0;
    virtual Result<SensorData> getOffset() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;
    virtual Result<SensorType> getType() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isRunning() const = 0;
    
    // UPEP: Multi-sensor sync support
    virtual ResultVoid setSyncConfig(const SyncConfig& config) = 0;
    virtual Result<SyncConfig> getSyncConfig() const = 0;
};

class SensorHALFactory {
public:
    using CreateFunc = ISensorHAL* (*)();
    using DestroyFunc = void (*)(ISensorHAL*);

    static Result<ISensorHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<ISensorHAL*> create(const SensorConfig& config);
    static void destroy(ISensorHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

} // namespace dynalgo::hal