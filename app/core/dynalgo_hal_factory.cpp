/*
 * dynalgo_hal_factory.cpp - Dynamic HAL plugin loader implementation
 */
#include "dynalgo_hal_factory.hpp"

#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/encoder_hal.hpp>
#include <dynalgo/hal/inference_hal.hpp>
#include <dynalgo/hal/display_hal.hpp>
#include <dynalgo/hal/actuator_hal.hpp>
#include <dynalgo/hal/sensor_hal.hpp>
#include <dynalgo/hal/logger_hal.hpp>
#include <dynalgo/hal/time_hal.hpp>

#include "dynalgo_config_manager.hpp"


#include <dlfcn.h>
#include <filesystem>
#include <mutex>
#include <algorithm>

namespace dynalgo {

std::vector<std::string> HALFactory::s_loaded_plugins;
std::mutex HALFactory::s_plugin_mutex;

std::string HALFactory::getPluginName(const std::string& hal_type, const std::string& vendor) {
    return "libdynalgo_hal_" + hal_type + "_" + vendor + ".so";
}

std::string HALFactory::getPluginPath(const std::string& plugin_name) {
    static const std::vector<std::string> search_paths = {
        "./lib/dynalgo/hal",
        "/usr/local/lib/dynalgo/hal",
        "/opt/dynalgo/lib/hal",
        "/usr/lib/dynalgo/hal",
        "/usr/lib/aarch64-linux-gnu/dynalgo/hal"
    };
    
    for (const auto& path : search_paths) {
        std::filesystem::path p = std::filesystem::path(path) / plugin_name;
        if (std::filesystem::exists(p)) {
            return p.string();
        }
    }
    return "";
}

template<typename Interface, typename CreateFunc, typename DestroyFunc>
hal::Result<std::shared_ptr<Interface>> 
HALFactory::loadPlugin(const std::string& hal_type, const std::string& vendor) {
    std::lock_guard<std::mutex> lock(s_plugin_mutex);
    
    std::string plugin_name = "libdynalgo_hal_" + std::string("camera") + "_" + "nvsipl" + ".so"; // TODO: fix
    // This is a simplified version - in reality we'd construct the plugin name properly
    
    return hal::Result<std::shared_ptr<Interface>>::err(
        hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::shared_ptr<hal::ICameraHAL>> HALFactory::loadCameraHAL(
    const std::string& vendor, const hal::CameraConfig& cam_config) {
    
    std::string plugin_name = "libdynalgo_hal_camera_" + vendor + ".so";
    
    // Search for plugin
    static const std::vector<std::string> search_paths = {
        "./lib/dynalgo/hal",
        "/usr/local/lib/dynalgo/hal",
        "/opt/dynalgo/lib/hal",
        "/usr/lib/dynalgo/hal"
    };
    
    std::string plugin_path;
    for (const auto& path : search_paths) {
        std::filesystem::path p = std::filesystem::path(path) / ("libdynalgo_hal_camera_" + vendor + ".so");
        if (std::filesystem::exists(p)) {
            plugin_path = p.string();
            break;
        }
    }
    
    if (plugin_path.empty()) {
        return hal::Result<std::shared_ptr<hal::ICameraHAL>>::err(hal::ErrorCode::NOT_FOUND);
    }
    
    void* handle = dlopen(plugin_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        return hal::Result<std::shared_ptr<hal::ICameraHAL>>::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    using CreateFunc = hal::ICameraHAL* (* )();
    using DestroyFunc = void (*)(hal::ICameraHAL*);
    
    CreateFunc create = reinterpret_cast<CreateFunc>(dlsym(handle, "dynalgo_hal_camera_create"));
    DestroyFunc destroy = reinterpret_cast<DestroyFunc>(dlsym(handle, "dynalgo_hal_camera_destroy"));
    
    if (!create || !destroy) {
        dlclose(handle);
        return hal::Result<std::shared_ptr<hal::ICameraHAL>>::err(hal::ErrorCode::NOT_SUPPORTED);
    }
    
    auto hal = std::shared_ptr<hal::ICameraHAL>(create(), 
        [handle, destroy](hal::ICameraHAL* ptr) {
            destroy(ptr);
            dlclose(handle);
        });
    
    // Initialize with config
    hal::CameraConfig cfg;
    cfg.device_id = "nvsipl://" + std::to_string(0); // TODO: get from config
    cfg.width = 3840;
    cfg.height = 2160;
    cfg.fps = 30;
    cfg.format = dynalgo::hal::PixelFormat::NV12;
    cfg.buffer_count = 4;
    cfg.hardware_sync = true;
    
    auto init_result = hal->initialize(cfg);
    if (!init_result.is_ok()) {
        return hal::Result<std::shared_ptr<hal::ICameraHAL>>::err(init_result.error());
    }
    
    return hal::Result<std::shared_ptr<hal::ICameraHAL>>::ok(std::move(hal));
}

hal::Result<std::shared_ptr<hal::IEncoderHAL>> HALFactory::loadEncoderHAL(
    const std::string& vendor, const hal::EncodeConfig& config) {
    return hal::Result<std::shared_ptr<hal::IEncoderHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<std::shared_ptr<hal::IInferenceHAL>> HALFactory::loadInferenceHAL(
    const std::string& vendor, const hal::ModelConfig& config) {
    return hal::Result<std::shared_ptr<hal::IInferenceHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<std::shared_ptr<hal::IDisplayHAL>> HALFactory::loadDisplayHAL(
    const std::string& vendor, const hal::DisplayConfig& config) {
    return hal::Result<std::shared_ptr<hal::IDisplayHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<std::shared_ptr<hal::IActuatorHAL>> HALFactory::loadActuatorHAL(
    const std::string& vendor, const hal::ActuatorConfig& config) {
    return hal::Result<std::shared_ptr<hal::IActuatorHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<std::shared_ptr<hal::ISensorHAL>> HALFactory::loadSensorHAL(
    const std::string& vendor, const hal::SensorConfig& config) {
    return hal::Result<std::shared_ptr<hal::ISensorHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<std::shared_ptr<hal::ILoggerHAL>> HALFactory::loadLoggerHAL(
    const std::string& vendor, const hal::LogConfig& config) {
    return hal::Result<std::shared_ptr<hal::ILoggerHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<std::shared_ptr<hal::ITimeHAL>> HALFactory::loadTimeHAL(
    const std::string& vendor, const hal::TimeConfig& config) {
    return hal::Result<std::shared_ptr<hal::ITimeHAL>>::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

hal::Result<HALBundle> HALFactory::createFromConfig(
    const PlatformConfig& platform_cfg,
    const BoardConfig& board_cfg,
    const VendorConfigMap& vendor_configs) {
    
    HALBundle bundle;
    
    // Load camera HAL
    auto cam_result = loadCameraHAL(platform_cfg.getVendor("camera"), {});
    if (!cam_result.is_ok()) {
        return hal::Result<HALBundle>::err(cam_result.error());
    }
    bundle.camera = std::move(cam_result.value());
    
    // TODO: Load other HALs
    
    return hal::Result<HALBundle>::ok(std::move(bundle));
}

void HALFactory::registerX86_64Vendors() {
    // The vendors are registered via the plugin system - each HAL is a shared library
    // that gets loaded dynamically by the CameraHALFactory/SensorHALFactory.
    // The libraries are: libdynalgo_hal_camera_orbbec.so, libdynalgo_hal_camera_robosense.so,
    // libdynalgo_hal_camera_stereo.so, libdynalgo_hal_sensor_robosense_lidar.so
    // They should be placed in ./lib/dynalgo/hal/ or the build output directory.
}

} // namespace dynalgo