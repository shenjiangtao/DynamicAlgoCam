#include <dynalgo/hal/display_hal.hpp>

namespace dynalgo {
class SDL2DisplayHAL : public hal::IDisplayHAL {
public:
    SDL2DisplayHAL() = default;
    ~SDL2DisplayHAL() override = default;

    hal::Result<std::vector<hal::DisplayInfo>> enumerateDisplays() override { return hal::Result<std::vector<hal::DisplayInfo>>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid initialize(const hal::DisplayConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid present(const hal::FrameMetadata&, const hal::PresentConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid presentAsync(const hal::FrameMetadata&, const hal::PresentConfig&, std::function<void(hal::ResultVoid)>) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid presentBuffer(const hal::BufferHandle&, hal::PixelFormat, uint32_t, uint32_t, uint32_t, const hal::PresentConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setLayer(uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setOpacity(float) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setRotation(hal::DisplayRotation) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setPosition(int32_t, int32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setSize(uint32_t, uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid show() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid hide() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setVSync(bool) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setColorSpace(const std::string&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setHDRMetadata(const void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("SDL2 Display HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("generic"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    bool isVisible() const override { return false; }
    hal::Result<hal::DisplayConfig> getConfig() const override { return hal::Result<hal::DisplayConfig>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IDisplayHAL* dynalgo_hal_display_create() { return new dynalgo::SDL2DisplayHAL(); }
void dynalgo_hal_display_destroy(dynalgo::hal::IDisplayHAL* ptr) { delete ptr; }
}
