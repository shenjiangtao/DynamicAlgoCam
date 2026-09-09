/*
 * Platform configuration parser implementation
 */
#include "nvsipl_platform_cfg.hpp"

#include <fstream>
#include <iostream>

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
    // TODO: Implement YAML parsing using yaml-cpp
    // For now, return default config
    device_blocks = CreateDefaultConfig();
    return true;
}

} // namespace dynalgo