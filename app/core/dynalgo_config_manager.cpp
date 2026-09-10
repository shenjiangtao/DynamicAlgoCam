/*
 * dynalgo_config_manager.cpp - Configuration manager implementation
 */
#include "dynalgo_config_manager.hpp"

#include <fstream>
#include <iostream>
#include <filesystem>
#include <stdexcept>

#if defined(HAVE_YAML_CPP)
#include <yaml-cpp/yaml.h>
#endif

namespace dynalgo {

std::unique_ptr<PlatformConfig> PlatformConfig::loadFromFile(const std::string& filepath) {
#if defined(HAVE_YAML_CPP)
    try {
        YAML::Node node = YAML::LoadFile(filepath);
        if (!node["platform"]) {
            throw std::runtime_error("Missing 'platform' section in config");
        }
        
        auto cfg = std::make_unique<PlatformConfig>();
        auto& p = node["platform"];
        
        cfg->m_name = p["name"].as<std::string>();
        cfg->m_arch = p["arch"].as<std::string>();
        cfg->m_vendor = p["vendor"].as<std::string>();
        cfg->m_soc = p["soc"].as<std::string>();
        cfg->m_soc_revision = p["soc_revision"].as<std::string>();
        
        if (p["defaults"]) {
            for (auto it = p["defaults"].begin(); it != p["defaults"].end(); ++it) {
                cfg->m_defaults[it->first.as<std::string>()] = it->second.as<std::string>();
            }
        }
        
        if (p["features"]) {
            for (auto it = p["features"].begin(); it != p["features"].end(); ++it) {
                cfg->m_features[it->first.as<std::string>()] = it->second.as<bool>();
            }
        }
        
        if (p["resources"]) {
            for (auto it = p["resources"].begin(); it != p["resources"].end(); ++it) {
                cfg->m_resources[it->first.as<std::string>()] = it->second.as<int>();
            }
        }
        
        return cfg;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load platform config: " << e.what() << std::endl;
        return nullptr;
    }
#else
    (void)filepath;
    std::cerr << "YAML support not available, cannot load platform config" << std::endl;
    return nullptr;
#endif
}

std::unique_ptr<BoardConfig> BoardConfig::loadFromFile(const std::string& filepath) {
#if defined(HAVE_YAML_CPP)
    try {
        YAML::Node node = YAML::LoadFile(filepath);
        if (!node["board"]) {
            throw std::runtime_error("Missing 'board' section in config");
        }
        
        auto cfg = std::make_unique<BoardConfig>();
        auto& b = node["board"];
        
        cfg->m_name = b["name"].as<std::string>();
        cfg->m_platform = b["platform"].as<std::string>();
        cfg->m_revision = b["revision"].as<std::string>();
        
        if (b["cameras"]) {
            for (const auto& cam : b["cameras"]) {
                CameraConfigYAML cam_cfg;
                cam_cfg.id = cam["id"].as<std::string>();
                cam_cfg.connector = cam["connector"].as<std::string>();
                cam_cfg.vendor = cam["vendor"].as<std::string>();
                cam_cfg.sensor_config = cam["sensor_config"].as<std::string>();
                cam_cfg.position = cam["position"].as<std::string>();
                cam_cfg.orientation = cam["orientation"].as<int>();
                cam_cfg.intrinsics = cam["intrinsics"].as<std::string>();
                cam_cfg.extrinsics = cam["extrinsics"].as<std::string>();
                cam_cfg.gpio_power = cam["gpio_power"].as<int>();
                cam_cfg.gpio_reset = cam["gpio_reset"].as<int>();
                cam_cfg.gpio_sync = cam["gpio_sync"].as<int>();
                cfg->m_cameras.push_back(cam_cfg);
            }
        }
        
        if (b["actuators"]) {
            for (const auto& act : b["actuators"]) {
                ActuatorConfigYAML act_cfg;
                act_cfg.id = act["id"].as<std::string>();
                act_cfg.type = act["type"].as<std::string>();
                act_cfg.interface = act["interface"].as<std::string>();
                if (act["params"]) {
                    for (auto it = act["params"].begin(); it != act["params"].end(); ++it) {
                        act_cfg.params[it->first.as<std::string>()] = it->second.as<std::string>();
                    }
                }
                act_cfg.vendor = act["vendor"].as<std::string>();
                cfg->m_actuators.push_back(act_cfg);
            }
        }
        
        if (b["sync"]) {
            auto& s = b["sync"];
            cfg->m_sync.ptp_enabled = s["ptp_enabled"].as<bool>();
            cfg->m_sync.ptp_interface = s["ptp_interface"].as<std::string>();
            cfg->m_sync.ptp_domain = s["ptp_domain"].as<int>();
            cfg->m_sync.gpio_sync_pin = s["gpio_sync_pin"].as<int>();
            cfg->m_sync.gpio_sync_polarity = s["gpio_sync_polarity"].as<std::string>();
            cfg->m_sync.hardware_trigger = s["hardware_trigger"].as<bool>();
        }
        
        if (b["power"]) {
            auto& p = b["power"];
            cfg->m_power.mode = p["mode"].as<std::string>();
            cfg->m_power.thermal_zone = p["thermal_zone"].as<std::string>();
            cfg->m_power.max_temp_c = p["max_temp_c"].as<int>();
        }
        
        if (b["storage"]) {
            auto& s = b["storage"];
            cfg->m_storage.data_path = s["data_path"].as<std::string>();
            cfg->m_storage.cache_path = s["cache_path"].as<std::string>();
            cfg->m_storage.log_path = s["log_path"].as<std::string>();
            cfg->m_storage.calibration_path = s["calibration_path"].as<std::string>();
        }
        
        if (b["network"]) {
            auto& n = b["network"];
            cfg->m_network.ptp_interface = n["ptp_interface"].as<std::string>();
            cfg->m_network.multicast_group = n["multicast_group"].as<std::string>();
            cfg->m_network.multicast_port = n["multicast_port"].as<int>();
        }
        
        return cfg;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load board config: " << e.what() << std::endl;
        return nullptr;
    }
#else
    (void)filepath;
    std::cerr << "YAML support not available, cannot load board config" << std::endl;
    return nullptr;
#endif
}

std::unique_ptr<VendorConfig> VendorConfig::loadFromFile(const std::string& filepath) {
#if defined(HAVE_YAML_CPP)
    try {
        YAML::Node node = YAML::LoadFile(filepath);
        if (!node["vendor"]) {
            throw std::runtime_error("Missing 'vendor' section in config");
        }
        
        auto cfg = std::make_unique<VendorConfig>();
        auto& v = node["vendor"];
        
        cfg->m_name = v["name"].as<std::string>();
        cfg->m_platform = v["platform"].as<std::string>();
        cfg->m_library = v["library"].as<std::string>();
        
        if (v["sensor"]) {
            auto& s = v["sensor"];
            VendorConfig::SensorInfo si;
            si.name = s["name"].as<std::string>();
            si.resolution = s["resolution"].as<std::string>();
            si.fps = s["fps"].as<int>();
            si.interface = s["interface"].as<std::string>();
            si.pixel_format = s["pixel_format"].as<std::string>();
            si.hdr = s["hdr"].as<std::string>();
            cfg->m_sensor = si;
        }
        
        cfg->m_platform_cfg = v["platform_cfg"].as<std::string>();
        
        if (v["isp"]) {
            auto& i = v["isp"];
            VendorConfig::ISPConfig isp_cfg;
            isp_cfg.mode = i["mode"].as<std::string>();
            isp_cfg.ae_target = i["ae_target"].as<int>();
            isp_cfg.awb_mode = i["awb_mode"].as<std::string>();
            isp_cfg.ccm = i["ccm"].as<std::string>();
            cfg->m_isp = isp_cfg;
        }
        
        if (v["sync"]) {
            auto& s = v["sync"];
            VendorConfig::SyncConfig sync_cfg;
            sync_cfg.master = s["master"].as<bool>();
            sync_cfg.gpio_pin = s["gpio_pin"].as<int>();
            sync_cfg.trigger_mode = s["trigger_mode"].as<std::string>();
            cfg->m_sync = sync_cfg;
        }
        
        if (v["buffers"]) {
            auto& b = v["buffers"];
            VendorConfig::BufferConfig buf_cfg;
            buf_cfg.count = b["count"].as<int>();
            buf_cfg.layout = b["layout"].as<std::string>();
            buf_cfg.cuda_compatible = b["cuda_compatible"].as<bool>();
            buf_cfg.nvscibuf = b["nvscibuf"].as<bool>();
            cfg->m_buffers = buf_cfg;
        }
        
        return cfg;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load vendor config: " << e.what() << std::endl;
        return nullptr;
    }
#else
    (void)filepath;
    std::cerr << "YAML support not available, cannot load vendor config" << std::endl;
    return nullptr;
#endif
}

bool ConfigManager::loadConfigs(const std::string& config_root) {
    m_config_root = config_root;
    
    // Load platform config
    std::string platform_file = config_root + "/platform.yaml";
    if (!std::filesystem::exists(platform_file)) {
        platform_file = config_root + "/platforms/platform.yaml";
    }
    m_platform_cfg = PlatformConfig::loadFromFile(platform_file);
    if (!m_platform_cfg) {
        std::cerr << "Failed to load platform config from " << platform_file << std::endl;
        return false;
    }
    
    // Load board config
    std::string board_file = config_root + "/board.yaml";
    if (!std::filesystem::exists(board_file)) {
        board_file = config_root + "/boards/board.yaml";
    }
    m_board_cfg = BoardConfig::loadFromFile(board_file);
    if (!m_board_cfg) {
        std::cerr << "Failed to load board config from " << board_file << std::endl;
        return false;
    }
    
    // Load vendor configs
    std::string vendor_dir = config_root + "/vendor";
    if (std::filesystem::exists(vendor_dir) && std::filesystem::is_directory(vendor_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(vendor_dir)) {
            if (entry.is_directory()) {
                std::string vendor_name = entry.path().filename().string();
                for (const auto& file_entry : std::filesystem::directory_iterator(entry.path())) {
                    if (file_entry.path().extension() == ".yaml") {
                        auto vendor_cfg = VendorConfig::loadFromFile(file_entry.path().string());
                        if (vendor_cfg) {
                            m_vendors[vendor_cfg->name()] = std::move(vendor_cfg);
                        }
                    }
                }
            }
        }
    }
    
    return true;
}

} // namespace dynalgo