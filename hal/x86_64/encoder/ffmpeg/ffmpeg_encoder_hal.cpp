#include <dynalgo/hal/encoder_hal.hpp>

namespace dynalgo {
class FfmpegEncoderHAL : public hal::IEncoderHAL {
public:
    FfmpegEncoderHAL() = default;
    ~FfmpegEncoderHAL() override = default;

    hal::ResultVoid initialize(const hal::EncodeConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid start() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid stop() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid encodeFrame(const hal::FrameMetadata&, hal::BitstreamBuffer&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid encodeFrameAsync(const hal::FrameMetadata&, std::function<void(hal::Result<hal::BitstreamBuffer>)>) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid flush() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid forceIDR() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setBitrate(uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setFramerate(uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setGopSize(uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setQP(uint32_t, uint32_t, uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::EncoderStats> getStats() override { return hal::Result<hal::EncoderStats>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid resetStats() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("FFmpeg Encoder HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("generic"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    bool isRunning() const override { return false; }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IEncoderHAL* dynalgo_hal_encoder_create() { return new dynalgo::FfmpegEncoderHAL(); }
void dynalgo_hal_encoder_destroy(dynalgo::hal::IEncoderHAL* ptr) { delete ptr; }
}
