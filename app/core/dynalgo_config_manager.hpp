/*
 * dynalgo_config_manager.hpp - Configuration manager for platform/vendor/board
 * UPEP: 3-level hierarchy - Platform (SoC arch) -> Vendor (Silicon vendor) -> Board (Carrier board)
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

struct StreamConfigYAML {
    std::string type;              // "color", "depth", "ir", "ir_left", "ir_right", "points"
    int width = 1280;
    int height = 720;
    int fps = 30;
    std::string format;            // "NV12", "Y16", "YUYV", "POINT", "Y8"
    bool enabled = true;
    bool hw_d2c = false;           // Hardware D2C alignment
};

struct CameraConfigYAML {
    std::string id;                    // "cam0", "cam0_ir_left"
    std::string connector;             // "J13", "/dev/video0"
    std::string vendor;                // "orbbec", "robosense"
    std::string sensor_config;         // "gemini_305", "gemini_335l", "robosense_ac1"
    std::string connection_type;       // "usb3", "gmsl2", "ethernet", "pcie"
    std::string role;                  // "main", "stereo_left", "stereo_right", "depth", "ir"
    std::string position;              // "front", "rear", "left", "right"
    int orientation = 0;
    
    std::vector<StreamConfigYAML> streams;  // Multiple streams per camera
    
    std::string intrinsics;
    std::string extrinsics;
    int gpio_power = -1;
    int gpio_reset = -1;
    int gpio_sync = -1;
    
    // Orbbec-specific
    std::string orbbec_mode;           // "standard", "305g_gmsl2"
    bool disable_ir_left = false;      // For 305g
};

struct LidarConfigYAML {
    std::string id;
    std::string connector;
    std::string vendor;
    std::string sensor_config;
    std::string connection_type;       // "usb3", "ethernet", "pcie"
    std::string position;
    int orientation = 0;
    
    std::vector<StreamConfigYAML> streams;  // LiDAR streams (points, intensity, etc.)
    
    std::string intrinsics;
    std::string extrinsics;
    int gpio_power = -1;
    int gpio_reset = -1;
    int gpio_sync = -1;
    std::string coordinate_frame = "sensor";
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
    
    // UPEP: Multi-sensor sync config
    std::string sync_method = "hardware_trigger";  // hardware_trigger, ptp, software_timestamp, ntp, gps_pps
    uint32_t sync_group_id = 0;
    uint64_t max_time_diff_ns = 1000000;
    bool enable_interpolation = true;
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

// UPEP: Fusion configuration
struct FusionConfigYAML {
    bool enabled = false;
    std::string fusion_type = "early";  // early, late, deep
    float lidar_to_camera_extrinsics_x = 0.0f;
    float lidar_to_camera_extrinsics_y = 0.0f;
    float lidar_to_camera_extrinsics_z = 0.0f;
    float lidar_to_camera_roll = 0.0f;
    float lidar_to_camera_pitch = 0.0f;
    float lidar_to_camera_yaw = 0.0f;
    float min_depth_m = 0.1f;
    float max_depth_m = 100.0f;
    std::string depth_interpolation = "bilinear";
};

class BoardConfig {
public:
    static std::unique_ptr<BoardConfig> loadFromFile(const std::string& filepath);

    std::string name() const { return m_name; }
    std::string platform() const { return m_platform; }
    std::string revision() const { return m_revision; }

    const std::vector<CameraConfigYAML>& cameras() const { return m_cameras; }
    const std::vector<LidarConfigYAML>& lidars() const { return m_lidars; }
    const std::vector<ActuatorConfigYAML>& actuators() const { return m_actuators; }
    const SyncConfigYAML& sync() const { return m_sync; }
    const PowerConfigYAML& power() const { return m_power; }
    const StorageConfigYAML& storage() const { return m_storage; }
    const NetworkConfigYAML& network() const { return m_network; }
    const FusionConfigYAML& fusion() const { return m_fusion; }

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
    
    std::optional<LidarConfigYAML> getLidarConfig(size_t index) const {
        if (index < m_lidars.size()) return m_lidars[index];
        return std::nullopt;
    }
    
    std::optional<LidarConfigYAML> getLidarById(const std::string& id) const {
        for (const auto& lidar : m_lidars) {
            if (lidar.id == id) return lidar;
        }
        return std::nullopt;
    }

private:
    std::string m_name;
    std::string m_platform;
    std::string m_revision;
    std::vector<CameraConfigYAML> m_cameras;
    std::vector<LidarConfigYAML> m_lidars;
    std::vector<ActuatorConfigYAML> m_actuators;
    SyncConfigYAML m_sync;
    PowerConfigYAML m_power;
    StorageConfigYAML m_storage;
    NetworkConfigYAML m_network;
    FusionConfigYAML m_fusion;
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
    
    // UPEP: Heterogeneous compute resources
    int getGpuMemoryMB() const { return m_gpu_memory_mb; }
    int getDlaCores() const { return m_dla_cores; }
    bool hasDla() const { return m_has_dla; }
    bool hasPtp() const { return m_has_ptp; }
    bool hasGpio() const { return m_has_gpio; }
    bool hasCan() const { return m_has_can; }

private:
    std::string m_name;
    std::string m_arch;
    std::string m_vendor;
    std::string m_soc;
    std::string m_soc_revision;
    std::map<std::string, std::string> m_defaults;
    std::map<std::string, bool> m_features;
    std::map<std::string, int> m_resources;
    
    // UPEP: Hardware resources
    int m_gpu_memory_mb = 0;
    int m_dla_cores = 0;
    bool m_has_dla = false;
    bool m_has_ptp = false;
    bool m_has_gpio = false;
    bool m_has_can = false;
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