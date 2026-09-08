#include <dynalgo/hal/time_hal.hpp>

namespace dynalgo {
class ChronoTimeHAL : public hal::ITimeHAL {
public:
    ChronoTimeHAL() = default;
    ~ChronoTimeHAL() override = default;

    hal::ResultVoid initialize(const hal::TimeConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    uint64_t now_ns() override { return hal::now_ns(); }
    hal::TimePoint now_tp() override { return hal::now_tp(); }
    hal::Result<uint64_t> now_ns(hal::TimeSource) override { return hal::Result<uint64_t>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid sync() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::TimeInfo> getTimeInfo() override { return hal::Result<hal::TimeInfo>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setTime(uint64_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid adjustOffset(int64_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid startPTP(const hal::PTPConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid stopPTP() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::TimeInfo> getPTPStatus() override { return hal::Result<hal::TimeInfo>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setAlarm(uint64_t, std::function<void()>) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid cancelAlarm() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("Chrono Time HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("generic"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    bool isSynchronized() const override { return false; }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ITimeHAL* dynalgo_hal_time_create() { return new dynalgo::ChronoTimeHAL(); }
void dynalgo_hal_time_destroy(dynalgo::hal::ITimeHAL* ptr) { delete ptr; }
}
