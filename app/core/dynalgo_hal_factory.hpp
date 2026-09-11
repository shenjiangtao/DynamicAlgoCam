/*
 * dynalgo_hal_factory.hpp - Dynamic HAL plugin loader
 */
#pragma once

#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/encoder_hal.hpp>
#include <dynalgo/hal/inference_hal.hpp>
#include <dynalgo/hal/display_hal.hpp>
#include <dynalgo/hal/actuator_hal.hpp>
#include <dynalgo/hal/sensor_hal.hpp>
#include <dynalgo/hal/logger_hal.hpp>
#include <dynalgo/hal/time_hal.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <dlfcn.h>

#include <functional>

namespace dynalgo {

// Forward declarations
struct PlatformConfig;
struct BoardConfig;
using VendorConfigMap = std::map<std::string, std::unique_ptr<struct VendorConfig>>;

// HALBundle - aggregate of all HAL interfaces (movable but not copyable)
struct HALBundle {
    std::shared_ptr<hal::ICameraHAL> camera;
    std::shared_ptr<hal::IEncoderHAL> encoder;
    std::shared_ptr<hal::IInferenceHAL> inference;
    std::shared_ptr<hal::IDisplayHAL> display;
    std::shared_ptr<hal::IActuatorHAL> actuator;
    std::shared_ptr<hal::ISensorHAL> sensor;
    std::shared_ptr<hal::ILoggerHAL> logger;
    std::shared_ptr<hal::ITimeHAL> time;

    HALBundle() = default;
    HALBundle(HALBundle&&) = default;
    HALBundle& operator=(HALBundle&&) = default;
    HALBundle(const HALBundle&) = delete;
    HALBundle& operator=(const HALBundle&) = delete;
};

class HALFactory {
public:
    // Load all HALs based on platform/vendor config
    static hal::Result<HALBundle> createFromConfig(
        const PlatformConfig& platform_cfg,
        const BoardConfig& board_cfg,
        const VendorConfigMap& vendor_configs);

    // Individual HAL loaders
    static hal::Result<std::shared_ptr<hal::ICameraHAL>> loadCameraHAL(
        const std::string& vendor, const hal::CameraConfig& cam_config);
    
    static hal::Result<std::shared_ptr<hal::IEncoderHAL>> loadEncoderHAL(
        const std::string& vendor, const hal::EncodeConfig& config);
    
    static hal::Result<std::shared_ptr<hal::IInferenceHAL>> loadInferenceHAL(
        const std::string& vendor, const hal::ModelConfig& config);
    
    static hal::Result<std::shared_ptr<hal::IDisplayHAL>> loadDisplayHAL(
        const std::string& vendor, const hal::DisplayConfig& config);
    
    static hal::Result<std::shared_ptr<hal::IActuatorHAL>> loadActuatorHAL(
        const std::string& vendor, const hal::ActuatorConfig& config);
    
    static hal::Result<std::shared_ptr<hal::ISensorHAL>> loadSensorHAL(
        const std::string& vendor, const hal::SensorConfig& config);
    
    static hal::Result<std::shared_ptr<hal::ILoggerHAL>> loadLoggerHAL(
        const std::string& vendor, const hal::LogConfig& config);
    
    static hal::Result<std::shared_ptr<hal::ITimeHAL>> loadTimeHAL(
        const std::string& vendor, const hal::TimeConfig& config);

    // Register x86_64 vendors for dynamic loading
    static void registerX86_64Vendors();

private:
    // Plugin handle with custom deleter
    struct PluginHandle {
        void* handle = nullptr;
        std::string name;
        
        ~PluginHandle() {
            if (handle) dlclose(handle);
        }
    };
    
    static std::string getPluginName(const std::string& hal_type, const std::string& vendor);
    static std::string getPluginPath(const std::string& plugin_name);
    
    template<typename Interface, typename CreateFunc, typename DestroyFunc>
    static hal::Result<std::shared_ptr<Interface>> 
    loadPlugin(const std::string& hal_type, const std::string& vendor);
    
    static std::vector<std::string> s_loaded_plugins;
    static std::mutex s_plugin_mutex;
};

} // namespace dynalgo