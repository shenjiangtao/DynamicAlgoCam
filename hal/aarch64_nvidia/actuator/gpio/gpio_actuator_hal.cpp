#include <dynalgo/hal/actuator_hal.hpp>

namespace dynalgo {
class GPIOActuatorHAL : public hal::IActuatorHAL {
public:
    GPIOActuatorHAL() = default;
    ~GPIOActuatorHAL() override = default;
    // TODO: Implement all pure virtual methods
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IActuatorHAL* dynalgo_hal_actuator_create() { return new dynalgo::GPIOActuatorHAL(); }
void dynalgo_hal_actuator_destroy(dynalgo::hal::IActuatorHAL* ptr) { delete ptr; }
}