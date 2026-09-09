#include <dynalgo/hal/logger_hal.hpp>

namespace dynalgo {
class SpdlogLoggerHAL : public hal::ILoggerHAL {
public:
    SpdlogLoggerHAL() = default;
    ~SpdlogLoggerHAL() override = default;
    // TODO: Implement all pure virtual methods
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ILoggerHAL* dynalgo_hal_logger_create() { return new dynalgo::SpdlogLoggerHAL(); }
void dynalgo_hal_logger_destroy(dynalgo::hal::ILoggerHAL* ptr) { delete ptr; }
}