#include <dynalgo/hal/display_hal.hpp>

namespace dynalgo {
class NvDisplayHAL : public hal::IDisplayHAL {
public:
    NvDisplayHAL() = default;
    ~NvDisplayHAL() override = default;
    // TODO: Implement all pure virtual methods
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IDisplayHAL* dynalgo_hal_display_create() { return new dynalgo::NvDisplayHAL(); }
void dynalgo_hal_display_destroy(dynalgo::hal::IDisplayHAL* ptr) { delete ptr; }
}