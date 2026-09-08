#include <dynalgo/hal/logger_hal.hpp>

namespace dynalgo {
class SpdlogLoggerHAL : public hal::ILoggerHAL {
public:
    SpdlogLoggerHAL() = default;
    ~SpdlogLoggerHAL() override = default;

    hal::ResultVoid initialize(const hal::LogConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    void log(const hal::LogEntry&) override {}
    void log(hal::LogLevel, const std::string&, const std::string&, const std::string&, int, const std::string&) override {}
    hal::ResultVoid setLevel(hal::LogLevel) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::LogLevel> getLevel() override { return hal::Result<hal::LogLevel>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setLevel(const std::string&, hal::LogLevel) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid flush() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("Spdlog Logger HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("spdlog"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ILoggerHAL* dynalgo_hal_logger_create() { return new dynalgo::SpdlogLoggerHAL(); }
void dynalgo_hal_logger_destroy(dynalgo::hal::ILoggerHAL* ptr) { delete ptr; }
}
