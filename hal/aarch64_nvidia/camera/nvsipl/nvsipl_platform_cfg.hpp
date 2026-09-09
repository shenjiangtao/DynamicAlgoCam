/*
 * Platform configuration parser for NvSIPL
 */
#pragma once

#include <dynalgo/hal/camera_hal.hpp>
#include <NvSIPLCommon.hpp>
#include <NvSIPLCamera.hpp>

#include <string>
#include <vector>
#include <memory>

namespace dynalgo {

class NvSIPLPlatformConfig {
public:
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
    };

    struct DeviceBlockInfo {
        std::string name;
        std::string cdi;
        std::vector<SensorInfo> sensors;
    };

    static bool ParseFromYAML(const std::string& yaml_file, 
                               std::vector<DeviceBlockInfo>& device_blocks);

    static std::vector<DeviceBlockInfo> CreateDefaultConfig();

private:
    static void CreateDefaultIMX728(DeviceBlockInfo& db);
    static void CreateDefaultIMX623(DeviceBlockInfo& db);
};

} // namespace dynalgo