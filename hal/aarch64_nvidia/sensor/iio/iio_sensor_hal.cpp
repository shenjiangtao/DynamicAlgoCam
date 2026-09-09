#include <dynalgo/hal/sensor_hal.hpp>

namespace dynalgo {
class IIOSensorHAL : public hal::ISensorHAL {
public:
    IIOSensorHAL() = default;
    ~IIOSensorHAL() override = default;
    // TODO: Implement all pure virtual methods
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ISensorHAL* dynalgo_hal_sensor_create() { return new dynalgo::IIOSensorHAL(); }
void dynalgo_hal_sensor_destroy(dynalgo::hal::ISensorHAL* ptr) { delete ptr; }
}