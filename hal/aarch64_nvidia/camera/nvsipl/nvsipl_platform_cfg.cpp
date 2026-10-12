/*
 * Platform configuration parser implementation
 * Based on NVIDIA CSiplCamera reference implementation (nvsipl-multicast-6.0.8.0)
 */
#include "nvsipl_platform_cfg.hpp"

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace dynalgo {

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

bool NvSIPLPlatformConfig::ParseFromYAML(const std::string& yaml_file, 
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
                        
                        db.sensors.push_back(sensor);
                    }
                }
                
                device_blocks.push_back(db);
            }
        }
        
        // Parse platform config
        if (platform_node["config"]) {
            // Additional platform-level settings can be parsed here
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        LOG_ERR("Failed to parse YAML config: %s", e.what());
        device_blocks = CreateDefaultConfig();
        return false;
    }
}

} // namespace dynalgo