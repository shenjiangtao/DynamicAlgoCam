#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace dynalgo::hal {

enum class CameraControl : uint32_t {
    EXPOSURE_TIME = 0x00980911,
    GAIN = 0x00980913,
    FRAME_RATE = 0x00980914,
    AUTO_EXPOSURE = 0x00980915,
    AUTO_GAIN = 0x00980916,
    AUTO_WHITE_BALANCE = 0x00980917,
    WHITE_BALANCE_TEMPERATURE = 0x00980918,
    FOCUS_ABSOLUTE = 0x00980919,
    FOCUS_AUTO = 0x0098091A,
    IRIS_ABSOLUTE = 0x0098091B,
    IRIS_AUTO = 0x0098091C,
    ZOOM_ABSOLUTE = 0x0098091D,
    PAN_ABSOLUTE = 0x0098091E,
    TILT_ABSOLUTE = 0x0098091F,
    TRIGGER_MODE = 0x00980920,
    TRIGGER_SOURCE = 0x00980921,
    TRIGGER_DELAY = 0x00980922,
    EXPOSURE_AUTO_PRIORITY = 0x00980923,
    BRIGHTNESS = 0x00980924,
    CONTRAST = 0x00980925,
    SATURATION = 0x00980926,
    HUE = 0x00980927,
    SHARPNESS = 0x00980928,
    GAMMA = 0x00980929,
    BACKLIGHT_COMPENSATION = 0x0098092A,
    VENDOR_BASE = 0x10000
};

struct CameraConfig {
    std::string device_id;
    std::string vendor;
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t fps = 30;
    PixelFormat format = PixelFormat::NV12;
    uint32_t buffer_count = 4;
    bool hardware_sync = true;
    uint32_t sync_master_id = 0;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;

    CameraConfig() = default;
    CameraConfig(const std::string& dev_id, const std::string& vend)
        : device_id(dev_id), vendor(vend) {}
};

struct MultiCameraConfig {
    std::vector<CameraConfig> cameras;
    bool synchronized = true;
    uint32_t master_camera = 0;
    uint32_t sync_timeout_ms = 100;
    std::string sync_method = "hardware";
};

struct SensorInfo {
    std::string name;
    std::string vendor;
    std::string serial_number;
    uint32_t max_width = 0;
    uint32_t max_height = 0;
    std::vector<PixelFormat> supported_formats;
    std::vector<std::pair<uint32_t, uint32_t>> supported_resolutions;
    std::vector<uint32_t> supported_fps;
    bool supports_hdr = false;
    bool supports_hw_sync = false;
    float pixel_size_um = 0.0f;
    std::string color_filter = "RGGB";
    std::string lens_mount = "M12";
};

struct CameraDeviceInfo {
    std::string device_id;
    std::string display_name;
    std::string vendor;
    std::string bus_info;
    SensorInfo sensor_info;
    bool is_available = true;
    std::string unavailable_reason;
};

class ICameraHAL {
public:
    virtual ~ICameraHAL() = default;

    virtual Result<std::vector<CameraDeviceInfo>> enumerateDevices() = 0;

    virtual Result<std::vector<StreamConfig>> getSupportedStreams(const std::string& device_id) = 0;

    virtual Result<SensorInfo> getSensorInfo(const std::string& device_id) = 0;

    virtual ResultVoid initialize(const CameraConfig& config) = 0;

    virtual ResultVoid initializeMulti(const MultiCameraConfig& config) = 0;

    virtual ResultVoid start() = 0;

    virtual ResultVoid stop() = 0;

    virtual ResultVoid deinitialize() = 0;

    virtual Result<FrameMetadata> acquireFrame(uint32_t timeout_ms = 1000) = 0;

    virtual Result<std::vector<FrameMetadata>> acquireFrames(uint32_t timeout_ms = 1000) = 0;

    using FrameCallback = std::function<void(const FrameMetadata&)>;
    virtual ResultVoid registerCallback(FrameCallback cb) = 0;
    virtual ResultVoid unregisterCallback() = 0;

    virtual Result<BufferHandle> importBuffer(const BufferHandle& external) = 0;
    virtual ResultVoid releaseBuffer(const BufferHandle& buffer) = 0;
    virtual ResultVoid releaseFrames(const std::vector<FrameMetadata>& frames) = 0;

    virtual ResultVoid setControl(const std::string& device_id, CameraControl control, int64_t value) = 0;
    virtual Result<int64_t> getControl(const std::string& device_id, CameraControl control) = 0;
    virtual Result<std::vector<std::pair<CameraControl, int64_t>>> getAllControls(const std::string& device_id) = 0;
    virtual Result<std::pair<int64_t, int64_t>> getControlRange(const std::string& device_id, CameraControl control) = 0;

    virtual ResultVoid triggerFrame(const std::string& device_id) = 0;
    virtual ResultVoid triggerFrames(const std::vector<std::string>& device_ids) = 0;

    virtual ResultVoid flush() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isRunning() const = 0;
    virtual uint32_t getDroppedFrameCount() const = 0;
    virtual void resetDroppedFrameCount() = 0;
};

class CameraHALFactory {
public:
    using CreateFunc = ICameraHAL* (*)();
    using DestroyFunc = void (*)(ICameraHAL*);

    static Result<ICameraHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<ICameraHAL*> create(const CameraConfig& config);
    static void destroy(ICameraHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static std::vector<CameraDeviceInfo> discoverAllDevices(const std::string& platform);

    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

inline Result<std::vector<FrameMetadata>> acquireFramesWithTimeout(
    ICameraHAL* hal, uint32_t count, uint32_t timeout_ms) {
    if (!hal) return Result<std::vector<FrameMetadata>>::err(ErrorCode::INVALID_ARG);

    std::vector<FrameMetadata> frames;
    frames.reserve(count);

    auto start = now_tp();
    while (frames.size() < count) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            start + std::chrono::milliseconds(timeout_ms) - now_tp());
        if (remaining.count() <= 0) break;

        auto result = hal->acquireFrame(static_cast<uint32_t>(remaining.count()));
        if (result.is_ok()) {
            frames.push_back(std::move(result.value()));
        } else if (result.error_code() == static_cast<uint32_t>(ErrorCode::TIMEOUT)) {
            break;
        } else {
            return Result<std::vector<FrameMetadata>>::err(result.error());
        }
    }

    return Result<std::vector<FrameMetadata>>::ok(std::move(frames));
}

} // namespace dynalgo::hal