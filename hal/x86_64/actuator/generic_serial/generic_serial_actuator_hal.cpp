#include <dynalgo/hal/actuator_hal.hpp>

namespace dynalgo {
class GenericSerialActuatorHAL : public hal::IActuatorHAL {
public:
    GenericSerialActuatorHAL() = default;
    ~GenericSerialActuatorHAL() override = default;

    hal::ResultVoid initialize(const hal::ActuatorConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid enable() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid disable() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid home() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setHomePosition(float) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid move(const hal::ActuatorCommand&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid moveAsync(const hal::ActuatorCommand&, std::function<void(hal::ResultVoid)>) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid stop(bool) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::ActuatorState> getState() override { return hal::Result<hal::ActuatorState>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::ActuatorLimits> getLimits() override { return hal::Result<hal::ActuatorLimits>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setControlMode(hal::ActuatorControlMode) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setLimits(const hal::ActuatorLimits&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setPID(float, float, float, float) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::tuple<float, float, float, float>> getPID() override { return hal::Result<std::tuple<float, float, float, float>>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setZeroPosition(float) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid calibrate() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("Generic Serial Actuator HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("generic"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::Result<hal::ActuatorType> getType() const override { return hal::Result<hal::ActuatorType>::ok(hal::ActuatorType::CUSTOM); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    bool isEnabled() const override { return false; }
    bool isMoving() const override { return false; }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::IActuatorHAL* dynalgo_hal_actuator_create() { return new dynalgo::GenericSerialActuatorHAL(); }
void dynalgo_hal_actuator_destroy(dynalgo::hal::IActuatorHAL* ptr) { delete ptr; }
}
