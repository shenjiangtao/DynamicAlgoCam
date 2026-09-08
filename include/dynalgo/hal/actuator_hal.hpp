#pragma once

#include "hal_types.hpp"
#include <string>
#include <vector>
#include <functional>

namespace dynalgo::hal {

enum class ActuatorType : uint32_t {
    UNKNOWN = 0,
    GIMBAL_PAN_TILT = 1,
    GIMBAL_3AXIS = 2,
    SERVO = 3,
    STEPPER = 4,
    DC_MOTOR = 5,
    BRUSHLESS = 6,
    SOLENOID = 7,
    RELAY = 8,
    LED = 9,
    LASER = 10,
    TRIGGER = 11,
    CUSTOM = 0xFFFF,
};

enum class ActuatorControlMode : uint32_t {
    POSITION = 0,
    VELOCITY = 1,
    TORQUE = 2,
    PWM = 3,
    VOLTAGE = 4,
    CURRENT = 5,
    OPEN_LOOP = 6,
};

struct ActuatorConfig {
    std::string device_id;
    std::string vendor;
    ActuatorType type = ActuatorType::UNKNOWN;
    ActuatorControlMode control_mode = ActuatorControlMode::POSITION;
    float min_position = -180.0f;
    float max_position = 180.0f;
    float max_velocity = 360.0f;
    float max_acceleration = 720.0f;
    float home_position = 0.0f;
    bool invert_direction = false;
    uint32_t update_rate_hz = 100;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct ActuatorState {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    float voltage = 0.0f;
    float current = 0.0f;
    bool is_enabled = false;
    bool is_homed = false;
    bool has_error = false;
    uint32_t error_code = 0;
    std::string error_message;
    uint64_t timestamp_ns = 0;
};

struct ActuatorCommand {
    ActuatorControlMode mode = ActuatorControlMode::POSITION;
    float target = 0.0f;
    float velocity_limit = 0.0f;
    float acceleration_limit = 0.0f;
    float torque_limit = 0.0f;
    uint32_t duration_ms = 0;
    bool blocking = false;
};

struct ActuatorLimits {
    float min_position = -180.0f;
    float max_position = 180.0f;
    float max_velocity = 360.0f;
    float max_acceleration = 720.0f;
    float max_torque = 10.0f;
    float max_current = 5.0f;
    float max_temperature = 85.0f;
};

class IActuatorHAL {
public:
    virtual ~IActuatorHAL() = default;

    virtual ResultVoid initialize(const ActuatorConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual ResultVoid enable() = 0;
    virtual ResultVoid disable() = 0;

    virtual ResultVoid home() = 0;
    virtual ResultVoid setHomePosition(float position) = 0;

    virtual ResultVoid move(const ActuatorCommand& cmd) = 0;
    virtual ResultVoid moveAsync(const ActuatorCommand& cmd,
                                 std::function<void(ResultVoid)> callback) = 0;

    virtual ResultVoid stop(bool emergency = false) = 0;

    virtual Result<ActuatorState> getState() = 0;
    virtual Result<ActuatorLimits> getLimits() = 0;

    virtual ResultVoid setControlMode(ActuatorControlMode mode) = 0;
    virtual ResultVoid setLimits(const ActuatorLimits& limits) = 0;

    virtual ResultVoid setPID(float kp, float ki, float kd, float ff = 0.0f) = 0;
    virtual Result<std::tuple<float, float, float, float>> getPID() = 0;

    virtual ResultVoid setZeroPosition(float offset = 0.0f) = 0;
    virtual ResultVoid calibrate() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;
    virtual Result<ActuatorType> getType() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isEnabled() const = 0;
    virtual bool isMoving() const = 0;
};

class ActuatorHALFactory {
public:
    using CreateFunc = IActuatorHAL* (*)();
    using DestroyFunc = void (*)(IActuatorHAL*);

    static Result<IActuatorHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<IActuatorHAL*> create(const ActuatorConfig& config);
    static void destroy(IActuatorHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);
};

} // namespace dynalgo::hal