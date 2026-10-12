/*
 * Platform configuration parser for NvSIPL
 * Supports JSON format for GMSL camera configurations
 */
#pragma once

#include <dynalgo/hal/camera_hal.hpp>
#include <NvSIPLCommon.hpp>
#include <NvSIPLCamera.hpp>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <optional>
#include <nlohmann/json.hpp>

namespace dynalgo {

class NvSIPLPlatformConfig {
public:
    struct GmslCameraParams {
        std::string camera_name;          // e.g., "NIO_P02F120A1_20BIT_RGGB_FLIP"
        std::string interface;            // e.g., "csi-e"
        std::string deserializer;         // e.g., "MAX96712DES1"
        uint32_t link = 0;                // Link index
        std::string output_format;        // e.g., "raw+processed"
        uint32_t cphy_mode = 1;           // CPHY mode
        bool skip_eeprom = true;          // Skip EEPROM read
        uint32_t fifo_size = 4;           // FIFO size
        bool enable_subframe = true;      // Enable subframe
        std::string output_timestamp;     // e.g., "raw"
        
        // Derived/optional
        std::string sensor_name;          // Sensor name (IMX728, IMX623, etc.)
        std::string module_name;          // Module name
        uint32_t sensor_id = 0;
    };

    struct SensorInfo {
        uint32_t sensor_id = 0;
        std::string name;
        std::string module_name;
        nvsipl::SurfaceFormat input_format = nvsipl::SurfaceFormat::SF_RAW12;
        uint32_t width = 3840;
        uint32_t height = 2160;
        uint32_t fps = 30;
        bool enable_isp0 = true;
        bool enable_isp1 = true;
        bool enable_icp = true;
        
        // GMSL-specific
        std::optional<GmslCameraParams> gmsl_params;
    };

    struct DeviceBlockInfo {
        std::string name;              // e.g., "deser0"
        std::string cdi;               // e.g., "max96712"
        std::vector<SensorInfo> sensors;
    };

    // Parse from JSON file
    static bool ParseFromJson(const std::string& json_file, 
                               std::vector<DeviceBlockInfo>& device_blocks);
    
    // Parse from YAML file (legacy)
    static bool ParseFromYaml(const std::string& yaml_file, 
                               std::vector<DeviceBlockInfo>& device_blocks);

    // Create default config for backward compatibility
    static std::vector<DeviceBlockInfo> CreateDefaultConfig();

    // Parse sensor parameters from JSON string
    static std::optional<GmslCameraParams> ParseGmslParams(const std::string& json_str);

    // Convert GmslCameraParams to JSON string
    static std::string GmslParamsToJson(const GmslCameraParams& params);

    // Create platform config from camera configs
    static bool CreatePlatformConfig(const std::vector<hal::CameraConfig>& camera_configs,
                                     std::vector<DeviceBlockInfo>& device_blocks);

private:
    static void CreateDefaultIMX728(DeviceBlockInfo& db);
    static void CreateDefaultIMX623(DeviceBlockInfo& db);
    
    // Parse GMSL params from JSON object
    static bool ParseGmslParamsFromJson(const nlohmann::json& json, GmslCameraParams& params);
};

} // namespace dynalgo