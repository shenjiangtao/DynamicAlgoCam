#include <dynalgo/hal/time_hal.hpp>

namespace dynalgo {
class PTPTimeHAL : public hal::ITimeHAL {
public:
    PTPTimeHAL() = default;
    ~PTPTimeHAL() override = default;
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ITimeHAL* dynalgo_hal_time_create() { return new dynalgo::PTPTimeHAL(); }
void dynalgo_hal_time_destroy(dynalgo::hal::ITimeHAL* ptr) { delete ptr; }
}
