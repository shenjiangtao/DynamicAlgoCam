#include <dynalgo/hal/sensor_hal.hpp>

namespace dynalgo {
class GenericSensorHAL : public hal::ISensorHAL {
public:
    GenericSensorHAL() = default;
    ~GenericSensorHAL() override = default;

    hal::ResultVoid initialize(const hal::SensorConfig&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid deinitialize() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid start() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid stop() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::SensorData> read(uint32_t) override { return hal::Result<hal::SensorData>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::vector<hal::SensorData>> readBatch(uint32_t, uint32_t) override { return hal::Result<std::vector<hal::SensorData>>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid registerCallback(hal::ISensorHAL::DataCallback) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid unregisterCallback() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setSampleRate(uint32_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<uint32_t> getSampleRate() const override { return hal::Result<uint32_t>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid calibrate() override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::ResultVoid setOffset(const hal::SensorData&) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<hal::SensorData> getOffset() override { return hal::Result<hal::SensorData>::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("Generic Sensor HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("generic"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::Result<hal::SensorType> getType() const override { return hal::Result<hal::SensorType>::ok(hal::SensorType::CUSTOM); }
    hal::ResultVoid vendorCommand(uint32_t, const void*, size_t, void*, size_t) override { return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED); }
    bool isRunning() const override { return false; }
};
} // namespace dynalgo

extern "C" {
dynalgo::hal::ISensorHAL* dynalgo_hal_sensor_create() { return new dynalgo::GenericSensorHAL(); }
void dynalgo_hal_sensor_destroy(dynalgo::hal::ISensorHAL* ptr) { delete ptr; }
}
