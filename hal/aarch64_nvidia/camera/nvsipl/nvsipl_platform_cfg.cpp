/*
 * Platform configuration parser implementation for NvSIPL
 * Supports JSON format for GMSL camera configurations (NIO/AD camera.json format)
 * Based on NVIDIA CSiplCamera reference implementation (nvsipl-multicast-6.0.8.0) and NIO/AD nvidia_camera
 */
#include "nvsipl_platform_cfg.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <regex>
#include <algorithm>

#include <json/json.h>  // Use jsoncpp from vendors/OrbbecSDK

#include <NvSIPLQuery.hpp>

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
    imx728.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
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
    imx623.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
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
    (void)yaml_file;
    // YAML parsing not available in aarch64 container (no yaml-cpp)
    // Return default config with a warning
    std::fprintf(stderr, "[NVPL_WARN] YAML parsing not supported in aarch64 HAL, using default config\n");
    device_blocks = CreateDefaultConfig();
    return true;
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
            std::fprintf(stderr, "[NVPL_ERR] Failed to open JSON config file: %s\n", json_file.c_str());
            device_blocks = CreateDefaultConfig();
            return false;
        }
        
        Json::CharReaderBuilder reader;
        reader["collectComments"] = false;
        Json::Value config;
        std::string errs;
        if (!Json::parseFromStream(reader, file, &config, &errs)) {
            std::fprintf(stderr, "[NVPL_ERR] Failed to parse JSON config: %s\n", errs.c_str());
            device_blocks = CreateDefaultConfig();
            return false;
        }
        
        // Support both platform-based format and direct sensorConfig array format
        if (config.isMember("sensorConfig")) {
            // Direct sensorConfig array format (NIO/AD camera.json format)
            return ParseSensorConfigArray(config["sensorConfig"], device_blocks);
        }
        
        // Legacy platform format
        if (config.isMember("platform")) {
            return ParsePlatformFormat(config["platform"], device_blocks);
        }
        
        std::fprintf(stderr, "[NVPL_WARN] No recognized config format, using default config\n");
        device_blocks = CreateDefaultConfig();
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[NVPL_ERR] Failed to parse JSON config: %s\n", e.what());
        device_blocks = CreateDefaultConfig();
        return false;
    }
}

bool NvSIPLPlatformConfig::ParseSensorConfigArray(const Json::Value& sensor_config_array, 
                                                   std::vector<DeviceBlockInfo>& device_blocks) {
    // Group cameras by deserializer (MAX96712DES1, DES2, etc.) for device blocks
    std::unordered_map<std::string, DeviceBlockInfo> deserializer_map;
    uint32_t sensor_id_counter = 0;
    
    try {
        for (const auto& cam_json : sensor_config_array) {
            // Skip if not a GMSL camera
            std::string protocol = cam_json.get("sensorProtocol", "").asString();
            if (protocol != "camera.gmsl") {
                continue;
            }
            
            std::string data_source_name = cam_json.get("dataSourceName", "").asString();
            std::string camera_name = cam_json.get("cameraName", "").asString();
            
            // Parse sensorParameters (primary GMSL params for standard hardware)
            std::string sensor_params = cam_json.get("sensorParameters", "").asString();
            
            if (sensor_params.empty()) {
                std::fprintf(stderr, "[NVPL_WARN] Camera %s has no sensorParameters\n", camera_name.c_str());
                continue;
            }
            
            // Parse GMSL params
            auto gmsl_params_opt = ParseGmslParams(sensor_params);
            if (!gmsl_params_opt.has_value()) {
                std::fprintf(stderr, "[NVPL_WARN] Failed to parse sensor params for camera %s\n", camera_name.c_str());
                continue;
            }
            
            auto gmsl_params = gmsl_params_opt.value();
            
            // Get deserializer name
            std::string deserializer = gmsl_params.deserializer;
            if (deserializer.empty()) {
                std::fprintf(stderr, "[NVPL_WARN] Camera %s has no deserializer\n", camera_name.c_str());
                continue;
            }
            
            // Get or create device block for this deserializer
            std::string deser_key = deserializer;
            if (deserializer.find("MAX96712DES") != std::string::npos) {
                // Extract DES number for grouping
                std::regex des_regex("MAX96712DES(\\d+)|DES_(\\w+)");
                std::smatch match;
                if (std::regex_search(deserializer, match, des_regex)) {
                    deser_key = match[0]; // Use full match
                }
            }
            
            // Create/get device block
            auto& db = deserializer_map[deser_key];
            if (db.name.empty()) {
                db.name = "deser_" + std::to_string(deserializer_map.size());
                db.cdi = "max96712"; // Default CDI for MAX96712 deserializers
            }
            
            // Create sensor info
            SensorInfo sensor;
            sensor.sensor_id = sensor_id_counter++;
            sensor.name = cam_json.get("cameraName", "unknown").asString();
            sensor.module_name = cam_json.get("dataSourceName", "").asString();
            
            // Parse resolution and fps
            sensor.width = cam_json.get("width", 3840u).asUInt();
            sensor.height = cam_json.get("height", 2160u).asUInt();
            sensor.fps = static_cast<uint32_t>(cam_json.get("frameRate", 30u).asUInt());
            
            // Determine input format from sensorParameters (20BIT/12BIT/24BIT)
            if (sensor_params.find("20BIT") != std::string::npos) {
                sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW20;
            } else if (sensor_params.find("12BIT") != std::string::npos) {
                sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
            } else if (sensor_params.find("24BIT") != std::string::npos) {
                sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW20; // 24BIT not in enum, use RAW20 as closest
            } else {
                sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
            }
            
            // Pipeline configuration from output-format
            std::string output_format = cam_json.get("output-format", "").asString();
            
            // Check for raw image enable
            sensor.enable_icp = (cam_json.get("rawImageEnable", 1u).asUInt() == 1);
            sensor.enable_isp0 = (cam_json.get("encoderEnable", 0u).asUInt() == 1);
            sensor.enable_isp1 = (cam_json.get("encoderEnable", 0u).asUInt() == 1);
            
            // Parse GMSL params for sensor struct (already parsed above, reuse)
            sensor.gmsl_params = gmsl_params;
            
            // Set sensor name from cameraName
            sensor.name = cam_json.get("cameraName", "unknown").asString();
            sensor.module_name = cam_json.get("dataSourceName", "").asString();
            
            // Set position and orientation if present
            if (cam_json.isMember("position")) sensor.position = cam_json.get("position", "").asString();
            if (cam_json.isMember("orientation")) sensor.orientation = cam_json.get("orientation", 0).asInt();
            
            db.sensors.push_back(sensor);
        }
        
        // Convert map to vector
        for (auto& [key, db] : deserializer_map) {
            if (!db.sensors.empty()) {
                device_blocks.push_back(std::move(db));
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[NVPL_ERR] Failed to parse sensorConfig array: %s\n", e.what());
        device_blocks = CreateDefaultConfig();
        return false;
    }
}

bool NvSIPLPlatformConfig::ParsePlatformFormat(const Json::Value& platform, 
                                                std::vector<DeviceBlockInfo>& device_blocks) {
    // Legacy platform format
    if (platform.isMember("device_blocks")) {
        for (const auto& db_json : platform["device_blocks"]) {
            DeviceBlockInfo db;
            db.name = db_json.get("name", "deser0").asString();
            db.cdi = db_json.get("cdi", "max96712").asString();
            
            if (db_json.isMember("sensors")) {
                for (const auto& sensor_json : db_json["sensors"]) {
                    SensorInfo sensor;
                    sensor.sensor_id = sensor_json.get("sensor_id", 0u).asUInt();
                    sensor.name = sensor_json.get("name", "").asString();
                    sensor.module_name = sensor_json.get("module_name", "").asString();
                    
                    std::string fmt_str = sensor_json.get("input_format", "RAW12").asString();
                    if (fmt_str == "RAW12") sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
                    else if (fmt_str == "RAW10") sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW10;
                    else if (fmt_str == "RAW14") sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW14;
                    else if (fmt_str == "RAW16") sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW16;
                    else if (fmt_str == "RAW20") sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW20;
                    else sensor.input_format = NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
                    
                    sensor.width = sensor_json.get("width", 3840u).asUInt();
                    sensor.height = sensor_json.get("height", 2160u).asUInt();
                    sensor.fps = sensor_json.get("fps", 30u).asUInt();
                    sensor.enable_isp0 = sensor_json.get("enable_isp0", true).asBool();
                    sensor.enable_isp1 = sensor_json.get("enable_isp1", true).asBool();
                    sensor.enable_icp = sensor_json.get("enable_icp", true).asBool();
                    
                    if (sensor_json.isMember("position")) sensor.position = sensor_json["position"].asString();
                    if (sensor_json.isMember("orientation")) sensor.orientation = sensor_json["orientation"].asInt();
                    
                    if (sensor_json.isMember("gmsl_params")) {
                        GmslCameraParams params;
                        std::string json_str = sensor_json["gmsl_params"].toStyledString();
                        if (ParseGmslParamsFromJson(json_str, params)) {
                            sensor.gmsl_params = params;
                        }
                    }
                    
                    db.sensors.push_back(sensor);
                }
            }
            
            device_blocks.push_back(db);
        }
        
        return true;
    }
    
    return false;
}

std::optional<NvSIPLPlatformConfig::GmslCameraParams> NvSIPLPlatformConfig::ParseGmslParams(const std::string& json_str) {
    try {
        // Handle comma-separated key=value format (NIO/AD format)
        std::string str = json_str;
        if (str.empty()) {
            return std::nullopt;
        }
        
        GmslCameraParams params;
        std::regex param_regex("([^,=]+)=([^,]*)(?:,|$)");
        std::smatch match;
        std::string::const_iterator search_start(str.cbegin());
        
        while (std::regex_search(search_start, str.cend(), match, param_regex)) {
            std::string key = match[1].str();
            std::string value = match[2].str();
            
            // Trim whitespace
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            if (key == "camera-name") params.camera_name = value;
            else if (key == "interface") params.interface = value;
            else if (key == "deserializer") params.deserializer = value;
            else if (key == "link") params.link = static_cast<uint32_t>(std::stoi(value));
            else if (key == "output-format") params.output_format = value;
            else if (key == "CPHY-mode") params.cphy_mode = static_cast<uint32_t>(std::stoi(value));
            else if (key == "skip-eeprom") params.skip_eeprom = (std::stoi(value) == 1);
            else if (key == "fifo-size") params.fifo_size = static_cast<uint32_t>(std::stoi(value));
            else if (key == "enable-subframe") params.enable_subframe = (std::stoi(value) == 1);
            else if (key == "output-timestamp") params.output_timestamp = value;
            else if (key == "disable-subframe") params.disable_subframe = (std::stoi(value) == 1);
            else if (key == "slave") params.slave_mode = (std::stoi(value) == 1);
            else if (key == "isp-mode") params.isp_mode = value;
            else if (key == "output-format") params.output_format = value;
            else if (key == "CPHY-mode") params.cphy_mode = static_cast<uint32_t>(std::stoi(value));
            else if (key == "disable-subframe") params.disable_subframe = (std::stoi(value) == 0);
            
            search_start = match.suffix().first;
        }
        
        // Log parsed params for debugging
        std::fprintf(stderr, "[NVPL_INFO] Parsed GMSL params: camera-name=%s, interface=%s, deserializer=%s, link=%u, "
                 "output-format=%s, CPHY-mode=%u, skip-eeprom=%s, fifo-size=%u, "
                 "enable-subframe=%s, output-timestamp=%s, disable-subframe=%s, slave_mode=%s, isp_mode=%s\n",
                 params.camera_name.c_str(), params.interface.c_str(), params.deserializer.c_str(),
                 params.link, params.output_format.c_str(), params.cphy_mode,
                 params.skip_eeprom ? "true" : "false", params.fifo_size,
                 params.enable_subframe ? "true" : "false", params.output_timestamp.c_str(),
                 params.disable_subframe ? "true" : "false", params.slave_mode ? "true" : "false", params.isp_mode.c_str());
        
        return params;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[NVPL_ERR] Failed to parse GMSL params: %s\n", e.what());
        return std::nullopt;
    }
}

std::string NvSIPLPlatformConfig::GmslParamsToJson(const GmslCameraParams& params) {
    Json::Value json;
    json["camera-name"] = params.camera_name;
    json["interface"] = params.interface;
    json["deserializer"] = params.deserializer;
    json["link"] = static_cast<Json::UInt>(params.link);
    json["output-format"] = params.output_format;
    json["CPHY-mode"] = static_cast<Json::UInt>(params.cphy_mode);
    json["skip-eeprom"] = params.skip_eeprom;
    json["fifo-size"] = static_cast<Json::UInt>(params.fifo_size);
    json["enable-subframe"] = params.enable_subframe;
    json["output-timestamp"] = params.output_timestamp;
    json["disable-subframe"] = params.disable_subframe;
    json["slave"] = params.slave_mode;
    json["isp-mode"] = params.isp_mode;
    return json.toStyledString();
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

bool NvSIPLPlatformConfig::ParseGmslParamsFromJson(const Json::Value& json_obj, GmslCameraParams& params) {
    try {
        params.camera_name = json_obj.get("camera-name", "").asString();
        params.interface = json_obj.get("interface", "csi-e").asString();
        params.deserializer = json_obj.get("deserializer", "MAX96712DES1").asString();
        params.link = json_obj.get("link", 0u).asUInt();
        params.output_format = json_obj.get("output-format", "raw+processed").asString();
        params.cphy_mode = json_obj.get("CPHY-mode", 1u).asUInt();
        params.skip_eeprom = json_obj.get("skip-eeprom", true).asBool();
        params.fifo_size = json_obj.get("fifo-size", 4u).asUInt();
        params.enable_subframe = json_obj.get("enable-subframe", true).asBool();
        params.output_timestamp = json_obj.get("output-timestamp", "raw").asString();
        params.disable_subframe = json_obj.get("disable-subframe", false).asBool();
        params.slave_mode = json_obj.get("slave", false).asBool();
        params.isp_mode = json_obj.get("isp-mode", "").asString();
        
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[NVPL_ERR] Failed to parse GMSL params JSON: %s\n", e.what());
        return false;
    }
}

// Helper: Map interface string to NvSIPL CSI port enum
static NvSiplCapInterfaceType MapInterfaceToCsiPort(const std::string& interface) {
    if (interface.find("csi-a") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_A;
    if (interface.find("csi-b") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_B;
    if (interface.find("csi-c") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_C;
    if (interface.find("csi-d") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_D;
    if (interface.find("csi-e") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_E;
    if (interface.find("csi-f") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_F;
    if (interface.find("csi-g") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_G;
    if (interface.find("csi-h") != std::string::npos) return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_H;
    return NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_A;
}

// Helper: Map camera name to input format
static NvSiplCapInputFormatType MapCameraNameToInputFormat(const std::string& camera_name) {
    if (camera_name.find("20BIT") != std::string::npos) return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW20;
    if (camera_name.find("12BIT") != std::string::npos) return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
    if (camera_name.find("24BIT") != std::string::npos) return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW20;
    if (camera_name.find("10BIT") != std::string::npos) return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW10;
    if (camera_name.find("14BIT") != std::string::npos) return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW14;
    if (camera_name.find("16BIT") != std::string::npos) return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW16;
    return NVSIPL_CAP_INPUT_FORMAT_TYPE_RAW12;
}

// Build real NvSIPL PlatformCfg via NIO-style query (preferred for real hardware)
nvsipl::PlatformCfg NvSIPLPlatformConfig::BuildPlatformCfgViaQuery(const std::vector<DeviceBlockInfo>& device_blocks) {
    nvsipl::PlatformCfg platform_cfg;
    platform_cfg.platform = "x86_64";
    platform_cfg.platformConfig = "dynalgo";
    platform_cfg.description = "DynamicAlgoCam platform config";
    platform_cfg.numDeviceBlocks = 0;
    
    try {
#ifdef DYNALGO_HAVE_NVSIPL
        // Try NIO-style query flow
        auto query = nvsipl::INvSIPLQuery::GetInstance();
        if (!query) {
            return nvsipl::PlatformCfg();
        }
        
        nvsipl::SIPLStatus status = query->ParseDatabase();
        if (status != nvsipl::NVSIPL_STATUS_OK) {
            return nvsipl::PlatformCfg();
        }
        
        // Build mask from deserializers
        std::vector<uint32_t> masks(5, 0);
        for (const auto& db : device_blocks) {
            for (const auto& sensor : db.sensors) {
                if (sensor.gmsl_params.has_value()) {
                    const auto& gmsl = sensor.gmsl_params.value();
                    if (gmsl.deserializer.find("MAX96712DES") != std::string::npos) {
                        // Extract DES number
                        std::regex des_regex("MAX96712DES(\\d+)");
                        std::smatch match;
                        if (std::regex_search(gmsl.deserializer, match, std::regex("MAX96712DES(\\d+)"))) {
                            int slot = std::stoi(match[1].str()) - 1;
                            if (slot >= 0 && slot < 5) {
                                masks[slot] |= (1u << (gmsl.link * 4));
                            }
                        }
                    } else if (gmsl.deserializer.find("DES_") != std::string::npos) {
                        // CCC board DES mapping
                        if (gmsl.deserializer.find("DES_F120") != std::string::npos || 
                            gmsl.deserializer.find("DES_F30") != std::string::npos) {
                            masks[0] |= (1u << (gmsl.link * 4));
                        } else if (gmsl.deserializer.find("DES_F30") != std::string::npos) {
                            masks[1] |= (1u << (gmsl.link * 4));
                        } else if (gmsl.deserializer.find("DES_SVC") != std::string::npos) {
                            masks[2] |= (1u << (gmsl.link * 4));
                        } else if (gmsl.deserializer.find("DES_SD") != std::string::npos) {
                            masks[3] |= (1u << (gmsl.link * 4));
                        }
                    }
                }
            }
        }
        
        auto query2 = nvsipl::INvSIPLQuery::GetInstance();
        if (query2) {
            if (query2->ParseDatabase() == nvsipl::NVSIPL_STATUS_OK) {
                // Try NIO_CCC first
                nvsipl::SIPLStatus status = query2->GetPlatformCfg("NIO_CCC", platform_cfg);
                if (status == nvsipl::NVSIPL_STATUS_OK) {
                    // Apply masks - ApplyMask takes a vector of masks
                    query2->ApplyMask(platform_cfg, masks);
                    return platform_cfg;
                }
            }
        }
#endif
    } catch (...) {
        // Ignore exceptions, fall back to direct construction
    }
    return nvsipl::PlatformCfg();
}

nvsipl::PlatformCfg NvSIPLPlatformConfig::BuildPlatformCfgDirect(const std::vector<DeviceBlockInfo>& device_blocks) {
    nvsipl::PlatformCfg platform_cfg;
    platform_cfg.platform = "dynalgo";
    platform_cfg.platformConfig = "dynalgo";
    platform_cfg.description = "DynamicAlgoCam platform config";
    platform_cfg.numDeviceBlocks = 0;
    
    // Set up device blocks from parsed model
    for (size_t db_idx = 0; db_idx < device_blocks.size() && db_idx < 16; ++db_idx) {
        const auto& src_db = device_blocks[db_idx];
        nvsipl::DeviceBlockInfo& dst_db = platform_cfg.deviceBlockList[platform_cfg.numDeviceBlocks];
        
        dst_db.csiPort = NVSIPL_CAP_CSI_INTERFACE_TYPE_CSI_A;
        dst_db.phyMode = NVSIPL_CAP_CSI_DPHY_MODE;
        dst_db.i2cDevice = 0;
        dst_db.desI2CPort = 0;
        dst_db.desTxPort = 0;
        dst_db.pwrPort = 0;
        dst_db.isPassiveModeEnabled = false;
        dst_db.isGroupInitProg = false;
        dst_db.resetAll = false;
        
        // Set deserializer info
        nvsipl::DeserInfo deser;
        deser.name = "";
        deser.i2cAddress = 0;
        deser.useCDIv2API = false;
        dst_db.deserInfo = deser;
        
        dst_db.numCameraModules = 0;
        
        for (size_t cam_idx = 0; cam_idx < src_db.sensors.size() && cam_idx < 4; ++cam_idx) {
            const auto& sensor = src_db.sensors[cam_idx];
            nvsipl::CameraModuleInfo& cam = dst_db.cameraModuleInfoList[dst_db.numCameraModules];
            
            cam.sensorInfo.id = sensor.sensor_id;
            cam.sensorInfo.name = sensor.name;
            // Note: moduleName doesn't exist in real SensorInfo, using name instead
            cam.sensorInfo.vcInfo.inputFormat = sensor.input_format;
            cam.sensorInfo.vcInfo.resolution.width = sensor.width;
            cam.sensorInfo.vcInfo.resolution.height = sensor.height;
            cam.sensorInfo.vcInfo.fps = static_cast<float>(sensor.fps);
            cam.sensorInfo.i2cAddress = 0;
            cam.sensorInfo.useCDIv2API = false;
            cam.linkIndex = sensor.gmsl_params.has_value() ? sensor.gmsl_params.value().link : 0;
            cam.isSimulatorModeEnabled = false;
            
            // Set serializer info
            cam.serInfo.name = "";
            cam.serInfo.i2cAddress = 0;
            cam.serInfo.useCDIv2API = false;
            
            // Set EEPROM info
            cam.isEEPROMSupported = false;
            cam.eepromInfo.name = "";
            cam.eepromInfo.i2cAddress = 0;
            cam.eepromInfo.useCDIv2API = false;
            
            dst_db.numCameraModules++;
        }
        
        platform_cfg.numDeviceBlocks++;
    }
    
    return platform_cfg;
}

nvsipl::PlatformCfg NvSIPLPlatformConfig::BuildRealPlatformCfg(const std::vector<DeviceBlockInfo>& device_blocks) {
    // Try NIO-style query first (preferred for real hardware)
    nvsipl::PlatformCfg cfg = BuildPlatformCfgViaQuery(device_blocks);
    
    // Fallback to direct construction if query fails
    if (cfg.numDeviceBlocks == 0) {
        cfg = BuildPlatformCfgDirect(device_blocks);
    }
    
    return cfg;
}

} // namespace dynalgo