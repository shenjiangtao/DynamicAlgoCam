/*
 * dynalgo_config_manager.hpp - Configuration manager for platform/vendor/board
 */
#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <map>
#include <optional>
#include <memory>
#include <filesystem>

#if defined(HAVE_YAML_CPP)
#define DYNALGO_HAVE_YAML 1
#endif

namespace dynalgo {

struct CameraConfigYAML {
    std::string id;
    std::string connector;
    std::string vendor;
    std::string sensor_config;
    std::string position;
    int orientation = 0;
    std::string intrinsics;
    std::string extrinsics;
    int gpio_power = -1;
    int gpio_reset = -1;
    int gpio_sync = -1;
};

struct ActuatorConfigYAML {
    std::string id;
    std::string type;
    std::string interface;
    std::map<std::string, std::string> params;
    std::string vendor;
};

struct SyncConfigYAML {
    bool ptp_enabled = false;
    std::string ptp_interface;
    int ptp_domain = 0;
    int gpio_sync_pin = -1;
    std::string gpio_sync_polarity = "rising";
    bool hardware_trigger = false;
};

struct PowerConfigYAML {
    std::string mode = "MAXN";
    std::string thermal_zone = "cpu-gpu";
    int max_temp_c = 85;
};

struct StorageConfigYAML {
    std::string data_path = "/data/dynalgo";
    std::string cache_path = "/tmp/dynalgo";
    std::string log_path = "/var/log/dynalgo";
    std::string calibration_path = "/opt/dynalgo/calibration";
};

struct NetworkConfigYAML {
    std::string ptp_interface = "eth0";
    std::string multicast_group = "239.255.0.1";
    int multicast_port = 5000;
};

class BoardConfig {
public:
    static std::unique_ptr<BoardConfig> loadFromFile(const std::string& filepath);

    std::string name() const { return m_name; }
    std::string platform() const { return m_platform; }
    std::string revision() const { return m_revision; }

    const std::vector<CameraConfigYAML>& cameras() const { return m_cameras; }
    const std::vector<ActuatorConfigYAML>& actuators() const { return m_actuators; }
    const SyncConfigYAML& sync() const { return m_sync; }
    const PowerConfigYAML& power() const { return m_power; }
    const StorageConfigYAML& storage() const { return m_storage; }
    const NetworkConfigYAML& network() const { return m_network; }

    std::optional<CameraConfigYAML> getCameraConfig(size_t index) const {
        if (index < m_cameras.size()) return m_cameras[index];
        return std::nullopt;
    }

    std::optional<CameraConfigYAML> getCameraById(const std::string& id) const {
        for (const auto& cam : m_cameras) {
            if (cam.id == id) return cam;
        }
        return std::nullopt;
    }

private:
    std::string m_name;
    std::string m_platform;
    std::string m_revision;
    std::vector<CameraConfigYAML> m_cameras;
    std::vector<ActuatorConfigYAML> m_actuators;
    SyncConfigYAML m_sync;
    PowerConfigYAML m_power;
    StorageConfigYAML m_storage;
    NetworkConfigYAML m_network;
};

class PlatformConfig {
public:
    static std::unique_ptr<PlatformConfig> loadFromFile(const std::string& filepath);

    std::string name() const { return m_name; }
    std::string arch() const { return m_arch; }
    std::string vendor() const { return m_vendor; }
    std::string soc() const { return m_soc; }
    std::string soc_revision() const { return m_soc_revision; }

    std::string getVendor(const std::string& hal_type) const {
        auto it = m_defaults.find(hal_type);
        return it != m_defaults.end() ? it->second : "";
    }

    bool hasFeature(const std::string& feature) const {
        auto it = m_features.find(feature);
        return it != m_features.end() && it->second;
    }

    const std::map<std::string, std::string>& defaults() const { return m_defaults; }
    const std::map<std::string, bool>& features() const { return m_features; }
    const std::map<std::string, int>& resources() const { return m_resources; }

private:
    std::string m_name;
    std::string m_arch;
    std::string m_vendor;
    std::string m_soc;
    std::string m_soc_revision;
    std::map<std::string, std::string> m_defaults;
    std::map<std::string, bool> m_features;
    std::map<std::string, int> m_resources;
};

class VendorConfig {
public:
    static std::unique_ptr<VendorConfig> loadFromFile(const std::string& filepath);

    std::string name() const { return m_name; }
    std::string platform() const { return m_platform; }
    std::string library() const { return m_library; }

    struct SensorInfo {
        std::string name;
        std::string resolution;
        int fps = 30;
        std::string interface;
        std::string pixel_format;
        std::string hdr;
    };
    std::optional<SensorInfo> sensor() const { return m_sensor; }

    std::string platform_cfg() const { return m_platform_cfg; }

    struct ISPConfig {
        std::string mode = "auto";
        int ae_target = 18;
        std::string awb_mode = "auto";
        std::string ccm = "standard";
    };
    std::optional<ISPConfig> isp() const { return m_isp; }

    struct SyncConfig {
        bool master = false;
        int gpio_pin = -1;
        std::string trigger_mode = "hardware";
    };
    std::optional<SyncConfig> sync() const { return m_sync; }

    struct BufferConfig {
        int count = 4;
        std::string layout = "block_linear";
        bool cuda_compatible = false;
        bool nvscibuf = false;
    };
    std::optional<BufferConfig> buffers() const { return m_buffers; }

private:
    std::string m_name;
    std::string m_platform;
    std::string m_library;
    std::optional<SensorInfo> m_sensor;
    std::string m_platform_cfg;
    std::optional<ISPConfig> m_isp;
    std::optional<SyncConfig> m_sync;
    std::optional<BufferConfig> m_buffers;
};

using VendorConfigMap = std::map<std::string, std::unique_ptr<VendorConfig>>;

class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager instance;
        return instance;
    }

    bool loadConfigs(const std::string& config_root = "/opt/dynalgo/config");

    const PlatformConfig& platform() const { return *m_platform_cfg; }
    const BoardConfig& board() const { return *m_board_cfg; }
    const VendorConfigMap& vendors() const { return m_vendors; }

    std::unique_ptr<VendorConfig> getVendorConfig(const std::string& vendor_name) const {
        auto it = m_vendors.find(vendor_name);
        if (it != m_vendors.end()) return std::make_unique<VendorConfig>(*it->second);
        return nullptr;
    }

private:
    ConfigManager() = default;

    std::unique_ptr<PlatformConfig> m_platform_cfg;
    std::unique_ptr<BoardConfig> m_board_cfg;
    std::map<std::string, std::unique_ptr<VendorConfig>> m_vendors;
    std::string m_config_root;
};

} // namespace dynalgo