#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <functional>

namespace dynalgo::hal {

enum class DisplayMode : uint32_t {
    WINDOWED = 0,
    FULLSCREEN = 1,
    BORDERLESS = 2,
    EXCLUSIVE = 3,
    OVERLAY = 4,
    HEADLESS = 5,
};

enum class DisplayRotation : uint32_t {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,
};

struct DisplayConfig {
    std::string device_id;
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t refresh_rate = 60;
    DisplayMode mode = DisplayMode::WINDOWED;
    DisplayRotation rotation = DisplayRotation::ROTATE_0;
    bool vsync = true;
    bool hw_overlay = false;
    uint32_t layer = 0;
    float opacity = 1.0f;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
    void* native_window = nullptr;
};

struct DisplayInfo {
    std::string device_id;
    std::string name;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t refresh_rate = 0;
    uint32_t physical_width_mm = 0;
    uint32_t physical_height_mm = 0;
    bool is_primary = false;
    bool supports_hw_overlay = false;
    bool supports_hdr = false;
    std::vector<PixelFormat> supported_formats;
};

struct PresentConfig {
    uint64_t target_timestamp_ns = 0;
    FenceHandle wait_fence;
    FenceHandle signal_fence;
    uint32_t src_x = 0;
    uint32_t src_y = 0;
    uint32_t src_width = 0;
    uint32_t src_height = 0;
    uint32_t dst_x = 0;
    uint32_t dst_y = 0;
    uint32_t dst_width = 0;
    uint32_t dst_height = 0;
    float alpha = 1.0f;
    bool flip_h = false;
    bool flip_v = false;
    DisplayRotation rotation = DisplayRotation::ROTATE_0;
};

class IDisplayHAL {
public:
    virtual ~IDisplayHAL() = default;

    virtual Result<std::vector<DisplayInfo>> enumerateDisplays() = 0;

    virtual ResultVoid initialize(const DisplayConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual ResultVoid present(const FrameMetadata& frame, const PresentConfig& config = {}) = 0;
    virtual ResultVoid presentAsync(const FrameMetadata& frame, const PresentConfig& config,
                                    std::function<void(ResultVoid)> callback) = 0;

    virtual ResultVoid presentBuffer(const BufferHandle& buffer, PixelFormat format,
                                     uint32_t width, uint32_t height, uint32_t stride,
                                     const PresentConfig& config = {}) = 0;

    virtual ResultVoid setLayer(uint32_t layer) = 0;
    virtual ResultVoid setOpacity(float opacity) = 0;
    virtual ResultVoid setRotation(DisplayRotation rotation) = 0;
    virtual ResultVoid setPosition(int32_t x, int32_t y) = 0;
    virtual ResultVoid setSize(uint32_t width, uint32_t height) = 0;

    virtual ResultVoid show() = 0;
    virtual ResultVoid hide() = 0;

    virtual ResultVoid setVSync(bool enable) = 0;
    virtual ResultVoid setColorSpace(const std::string& colorspace) = 0;
    virtual ResultVoid setHDRMetadata(const void* metadata, size_t size) = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isVisible() const = 0;
    virtual Result<DisplayConfig> getConfig() const = 0;
};

class DisplayHALFactory {
public:
    using CreateFunc = IDisplayHAL* (*)();
    using DestroyFunc = void (*)(IDisplayHAL*);

    static Result<IDisplayHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<IDisplayHAL*> create(const DisplayConfig& config);
    static void destroy(IDisplayHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

} // namespace dynalgo::hal