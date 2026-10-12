/*
 * Platform configuration parser implementation for NvSIPL
 * Supports JSON format for GMSL camera configurations
 * Based on NVIDIA CSiplCamera reference implementation (nvsipl-multicast-6.0.8.0)
 */
#include "nvsipl_platform_cfg.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace dynalgo {

using json = nlohmann::json;

std::vector<NvSIPLPlatformConfig::DeviceBlockInfo> NvSIPLPlatformConfig::CreateDefaultConfig() {
    std::vector<DeviceBlockInfo> device_blocks;
    DeviceBlockInfo db;
    db.name = "deser0";
    db.cdi = "max96712";
    
    // IMX728 front camera
    SensorInfo imx728;
    imx728.sensor_id = 0;
    imx728.name = "IMX728";
    imx728.module_name = "imx728_front";
    imx728.input_format = nvsipl::SurfaceFormat::SF_RAW12;
    imx728.width = 3840;
    imx728.height = 2160;
    imx728.fps = 30;
    imx728.enable_isp0 = true;
    imx728.enable_isp1 = true;
    imx728.enable_icp = true;
    db.sensors.push_back(imx728);
    
    // IMX623 rear camera
    SensorInfo imx623;
    imx623.sensor_id = 1;
    imx623.name = "IMX623";
    imx623.module_name = "imx623_rear";
    imx623.input_format = nvsipl::SurfaceFormat::SF_RAW12;
    imx623.width = 1920;
    imx623.height = 1080;
    imx623.fps = 60;
    imx623.enable_isp0 = true;
    imx623.enable_isp1 = true;
    imx623.enable_icp = true;
    db.sensors.push_back(imx623);
    
    device_blocks.push_back(db);
    return device_blocks;
}

bool NvSIPLPlatformConfig::ParseFromYaml(const std::string& yaml_file, 
                                          std::vector<DeviceBlockInfo>& device_blocks) {
    if (yaml_file.empty()) {
        device_blocks = CreateDefaultConfig();
        return true;
    }
    
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        if (!config["platform"]) {
            LOG_WARN("No 'platform' section in config file, using default config");
            device_blocks = CreateDefaultConfig();
            return true;
        }
        
        auto platform_node = config["platform"];
        
        // Parse device blocks
        if (platform_node["device_blocks"]) {
            for (const auto& db_node : platform_node["device_blocks"]) {
                DeviceBlockInfo db;
                db.name = db_node["name"].as<std::string>("deser0");
                db.cdi = db_node["cdi"].as<std::string>("max96712");
                
                if (db_node["sensors"]) {
                    for (const auto& sensor_node : db_node["sensors"]) {
                        SensorInfo sensor;
                        sensor.sensor_id = sensor_node["sensor_id"].as<uint32_t>(0);
                        sensor.name = sensor_node["name"].as<std::string>();
                        sensor.module_name = sensor_node["module_name"].as<std::string>();
                        
                        std::string fmt_str = sensor_node["input_format"].as<std::string>("RAW12");
                        if (fmt_str == "RAW12") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW12;
                        else if (fmt_str == "RAW10") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW10;
                        else if (fmt_str == "RAW14") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW14;
                        else if (fmt_str == "RAW16") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW16;
                        else if (fmt_str == "RAW20") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW20;
                        else sensor.input_format = nvsipl::SurfaceFormat::SF_RAW12;
                        
                        sensor.width = sensor_node["width"].as<uint32_t>(3840);
                        sensor.height = sensor_node["height"].as<uint32_t>(2160);
                        sensor.fps = sensor_node["fps"].as<uint32_t>(30);
                        sensor.enable_isp0 = sensor_node["enable_isp0"].as<bool>(true);
                        sensor.enable_isp1 = sensor_node["enable_isp1"].as<bool>(true);
                        sensor.enable_icp = sensor_node["enable_icp"].as<bool>(true);
                        
                        // Optional sensor position and orientation
                        if (sensor_node["position"]) sensor.position = sensor_node["position"].as<std::string>();
                        if (sensor_node["orientation"]) sensor.orientation = sensor_node["orientation"].as<int>(0);
                        
                        // Parse GMSL params if present
                        if (sensor_node["gmsl_params"]) {
                            if (ParseGmslParamsFromJson(sensor_node["gmsl_params"], sensor.gmsl_params)) {
                                sensor.gmsl_params = sensor.gmsl_params; // Already parsed
                            }
                        }
                        
                        db.sensors.push_back(sensor);
                    }
                }
                
                device_blocks.push_back(db);
            }
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        LOG_ERR("Failed to parse YAML config: %s", e.what());
        device_blocks = CreateDefaultConfig();
        return false;
    }
}

bool NvSIPLPlatformConfig::ParseFromJson(const std::string& json_file, 
                                          std::vector<DeviceBlockInfo>& device_blocks) {
    if (json_file.empty()) {
        device_blocks = CreateDefaultConfig();
        return true;
    }
    
    try {
        std::ifstream file(json_file);
        if (!file.is_open()) {
            LOG_ERR("Failed to open JSON config file: %s", json_file.c_str());
            device_blocks = CreateDefaultConfig();
            return false;
        }
        
        json config;
        file >> config;
        
        if (!config.contains("platform")) {
            LOG_WARN("No 'platform' section in config file, using default config");
            device_blocks = CreateDefaultConfig();
            return true;
        }
        
        auto platform = config["platform"];
        
        // Parse device blocks
        if (platform.contains("device_blocks")) {
            for (const auto& db_json : platform["device_blocks"]) {
                DeviceBlockInfo db;
                db.name = db_json.value("name", "deser0");
                db.cdi = db_json.value("cdi", "max96712");
                
                if (db_json.contains("sensors")) {
                    for (const auto& sensor_json : db_json["sensors"]) {
                        SensorInfo sensor;
                        sensor.sensor_id = sensor_json.value("sensor_id", 0u);
                        sensor.name = sensor_json.value("name", "");
                        sensor.module_name = sensor_json.value("module_name", "");
                        
                        std::string fmt_str = sensor_json.value("input_format", "RAW12");
                        if (fmt_str == "RAW12") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW12;
                        else if (fmt_str == "RAW10") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW10;
                        else if (fmt_str == "RAW14") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW14;
                        else if (fmt_str == "RAW16") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW16;
                        else if (fmt_str == "RAW20") sensor.input_format = nvsipl::SurfaceFormat::SF_RAW20;
                        else sensor.input_format = nvsipl::SurfaceFormat::SF_RAW12;
                        
                        sensor.width = sensor_json.value("width", 3840u);
                        sensor.height = sensor_json.value("height", 2160u);
                        sensor.fps = sensor_json.value("fps", 30u);
                        sensor.enable_isp0 = sensor_json.value("enable_isp0", true);
                        sensor.enable_isp1 = sensor_json.value("enable_isp1", true);
                        sensor.enable_icp = sensor_json.value("enable_icp", true);
                        
                        if (sensor_json.contains("position")) sensor.position = sensor_json["position"].get<std::string>();
                        if (sensor_json.contains("orientation")) sensor.orientation = sensor_json["orientation"].get<int>();
                        
                        // Parse GMSL params if present
                        if (sensor_json.contains("gmsl_params")) {
                            if (ParseGmslParamsFromJson(sensor_json["gmsl_params"], sensor.gmsl_params)) {
                                sensor.gmsl_params = sensor.gmsl_params; // Already parsed
                            }
                        }
                        
                        db.sensors.push_back(sensor);
                    }
                }
                
                device_blocks.push_back(db);
            }
        }
        
        return true;
    } catch (const json::exception& e) {
        LOG_ERR("Failed to parse JSON config: %s", e.what());
        device_blocks = CreateDefaultConfig();
        return false;
    }
}

std::optional<NvSIPLPlatformConfig::GmslCameraParams> NvSIPLPlatformConfig::ParseGmslParams(const std::string& json_str) {
    try {
        json json = json::parse(json_str);
        GmslCameraParams params;
        if (ParseGmslParamsFromJson(json, params)) {
            return params;
        }
    } catch (const json::exception& e) {
        LOG_ERR("Failed to parse GMSL params JSON: %s", e.what());
    }
    return std::nullopt;
}

std::string NvSIPLPlatformConfig::GmslParamsToJson(const GmslCameraParams& params) {
    json json;
    json["camera-name"] = params.camera_name;
    json["interface"] = params.interface;
    json["deserializer"] = params.deserializer;
    json["link"] = params.link;
    json["output-format"] = params.output_format;
    json["CPHY-mode"] = params.cphy_mode;
    json["skip-eeprom"] = params.skip_eeprom;
    json["fifo-size"] = params.fifo_size;
    json["enable-subframe"] = params.enable_subframe;
    json["output-timestamp"] = params.output_timestamp;
    return json.dump(4);
}

bool NvSIPLPlatformConfig::CreatePlatformConfig(const std::vector<hal::CameraConfig>& camera_configs,
                                                 std::vector<DeviceBlockInfo>& device_blocks) {
    DeviceBlockInfo db;
    db.name = "deser0";
    db.cdi = "max96712";
    
    for (size_t i = 0; i < camera_configs.size(); ++i) {
        SensorInfo sensor;
        sensor.sensor_id = static_cast<uint32_t>(i);
        sensor.name = camera_configs[i].vendor + "_" + camera_configs[i].device_id;
        sensor.module_name = camera_configs[i].device_id;
        sensor.width = camera_configs[i].width;
        sensor.height = camera_configs[i].height;
        sensor.fps = camera_configs[i].fps;
        sensor.enable_isp0 = true;
        sensor.enable_isp1 = true;
        sensor.enable_icp = true;
        
        // Parse vendor_params if it contains GMSL params
        if (camera_configs[i].vendor_params && camera_configs[i].vendor_params_size > 0) {
            std::string params_str(static_cast<const char*>(camera_configs[i].vendor_params), 
                                   camera_configs[i].vendor_params_size);
            if (auto gmsl_params = ParseGmslParams(params_str)) {
                sensor.gmsl_params = gmsl_params;
            }
        }
        
        db.sensors.push_back(sensor);
    }
    
    device_blocks.push_back(db);
    return true;
}

bool NvSIPLPlatformConfig::ParseGmslParamsFromJson(const nlohmann::json& json, GmslCameraParams& params) {
    try {
        params.camera_name = json.value("camera-name", "");
        params.interface = json.value("interface", "csi-e");
        params.deserializer = json.value("deserializer", "MAX96712DES1");
        params.link = json.value("link", 0u);
        params.output_format = json.value("output-format", "raw+processed");
        params.cphy_mode = json.value("CPHY-mode", 1u);
        params.skip_eeprom = json.value("skip-eeprom", true);
        params.fifo_size = json.value("fifo-size", 4u);
        params.enable_subframe = json.value("enable-subframe", true);
        params.output_timestamp = json.value("output-timestamp", "raw");
        
        // Log parsed params for debugging
        LOG_INFO("Parsed GMSL params: camera-name=%s, interface=%s, deserializer=%s, link=%u, "
                 "output-format=%s, CPHY-mode=%u, skip-eeprom=%s, fifo-size=%u, "
                 "enable-subframe=%s, output-timestamp=%s",
                 params.camera_name.c_str(), params.interface.c_str(), params.deserializer.c_str(),
                 params.link, params.output_format.c_str(), params.cphy_mode,
                 params.skip_eeprom ? "true" : "false", params.fifo_size,
                 params.enable_subframe ? "true" : "false", params.output_timestamp.c_str());
        
        return true;
    } catch (const json::exception& e) {
        LOG_ERR("Failed to parse GMSL params: %s", e.what());
        return false;
    }
}

} // namespace dynalgo