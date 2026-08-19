// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_driver_factory.cpp — Factory: discovers all available devices
// and creates DynalgoDevice+DynalgoPipeline pairs for each.

#include "dynalgo_driver_factory.hpp"
#include "utils.hpp"

#ifdef ENABLE_ORBBEC
#include "libobsensor/hpp/Context.hpp"
#include "dynalgo_ob_device.hpp"
#include <unistd.h>
#endif

#ifdef ENABLE_RS_AC1
#include "dynalgo_rs_device.hpp"
#endif

namespace dynalgo {

// [函数说明 / Function Description]
// 中文: 解析 Orbbec 扩展目录路径，优先使用环境变量，其次尝试相对路径
// English: Resolve Orbbec extensions directory path, prefer env var then relative paths
#ifdef ENABLE_ORBBEC
static std::string resolveOrbbecExtensionsDir() {
    const char* envDir = getenv("ORBBEC_EXTENSIONS_DIR");
    if (envDir && envDir[0] != '\0')
        return envDir;

    std::string exeDir = getExeDir();
    std::string candidates[] = {
        exeDir + "/../extensions",
        exeDir + "/extensions",
        "./extensions",
    };
    for (auto& dir : candidates) {
        if (access(dir.c_str(), F_OK) == 0)
            return dir;
    }
    return "";
}

// [函数说明 / Function Description]
// 中文: 解析 Orbbec SDK 配置文件路径，优先使用环境变量，其次尝试相对路径
// English: Resolve Orbbec SDK config file path, prefer env var then relative paths
static std::string resolveOrbbecConfigPath() {
    const char* envCfg = getenv("ORBBEC_CONFIG_PATH");
    if (envCfg && envCfg[0] != '\0')
        return envCfg;

    std::string exeDir = getExeDir();
    std::string candidates[] = {
        exeDir + "/../OrbbecSDKConfig.xml",
        exeDir + "/../config/OrbbecSDKConfig.xml",
        exeDir + "/OrbbecSDKConfig.xml",
        "./OrbbecSDKConfig.xml",
    };
    for (auto& path : candidates) {
        if (access(path.c_str(), F_OK) == 0)
            return path;
    }
    return "";
}
#endif

// [函数说明 / Function Description]
// 中文: 发现所有可用设备并创建 DynalgoDevice+DynalgoPipeline 对
// English: Discover all available devices and create DynalgoDevice+DynalgoPipeline pairs
std::vector<DiscoveredDevice> discoverDevices(const DriverConfig& cfg) {
    std::vector<DiscoveredDevice> result;

#ifdef ENABLE_ORBBEC
    if (cfg.vendor == DriverVendor::ALL || cfg.vendor == DriverVendor::ORBBEC) {
        std::string extensionsDir = resolveOrbbecExtensionsDir();
        std::string configPath = resolveOrbbecConfigPath();
        ObContext::initSDK(extensionsDir);
        ObContext obCtx(configPath);
        uint32_t obCount = obCtx.getDeviceCount();
        for (uint32_t i = 0; i < obCount; i++) {
            auto nioDev = obCtx.getDevice(i);
            auto obDev = std::dynamic_pointer_cast<ObDevice>(nioDev);
            if (!obDev)
                continue;
            DiscoveredDevice dd;
            dd.device = nioDev;
            dd.pipeline = std::make_shared<ObPipeline>(obDev->obDevice());
            result.push_back(std::move(dd));
        }
    }
#endif

#ifdef ENABLE_RS_AC1
    if (cfg.vendor == DriverVendor::ALL || cfg.vendor == DriverVendor::ROBOSENSE) {
        RsContext rsCtx;
        uint32_t rsCount = rsCtx.getDeviceCount();
        for (uint32_t i = 0; i < rsCount; i++) {
            auto nioDev = rsCtx.getDevice(i);
            auto rsDev = std::dynamic_pointer_cast<RsDevice>(nioDev);
            if (!rsDev)
                continue;
            DiscoveredDevice dd;
            dd.device = nioDev;
            dd.pipeline = std::make_shared<RsPipeline>(rsDev);
            result.push_back(std::move(dd));
        }
    }
#endif

    return result;
}

} // namespace dynalgo
