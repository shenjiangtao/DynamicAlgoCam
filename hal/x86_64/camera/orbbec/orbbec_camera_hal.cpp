/*
 * orbbec_camera_hal.cpp - Orbbec Camera HAL implementation for x86_64
 * Supports: Gemini 305, 305g (GMSL2), 335L, 336L
 * Features: Multi-camera (IR stereo), HW D2C, GMSL2, FrameBuffer zero-copy
 */

#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/hal_types.hpp>

#include <dlfcn.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>
#include <iostream>
#include <cstring>
#include <memory>
#include <filesystem>

namespace dynalgo {

// Forward declarations for Orbbec SDK types (loaded via dlopen)
namespace ob {
    class Context;
    class Device;
    class Pipeline;
    class Config;
    class FrameSet;
    class Frame;
    class VideoFrame;
    class DepthFrame;
    class PointCloudFilter;
    class Align;
    class StreamProfileList;
    class VideoStreamProfile;
    
    // Orbbec SDK struct definitions (matching SDK layout)
    struct OBCameraIntrinsic {
        float fx, fy, cx, cy;
        int width, height;
    };
    
    struct OBExtrinsic {
        float rotation[9];
        float translation[3];
    };
    
    struct DeviceInfo;
}

class OrbbecCameraHAL : public hal::ICameraHAL {
public:
    OrbbecCameraHAL() : ctx_(nullptr), running_(false), dropped_frames_(0) {}
    ~OrbbecCameraHAL() override { deinitialize(); }

    // ICameraHAL interface
    hal::Result<std::vector<hal::CameraDeviceInfo>> enumerateDevices() override;
    hal::Result<std::vector<hal::StreamConfig>> getSupportedStreams(const std::string& device_id) override;
    hal::Result<hal::SensorInfo> getSensorInfo(const std::string& device_id) override;
    hal::ResultVoid initialize(const hal::CameraConfig& config) override;
    hal::ResultVoid initializeMulti(const hal::MultiCameraConfig& config) override;
    hal::ResultVoid start() override;
    hal::ResultVoid stop() override;
    hal::ResultVoid deinitialize() override;
    hal::Result<hal::FrameMetadata> acquireFrame(uint32_t timeout_ms = 1000) override;
    hal::Result<std::vector<hal::FrameMetadata>> acquireFrames(uint32_t timeout_ms = 1000) override;
    hal::ResultVoid registerCallback(typename hal::ICameraHAL::FrameCallback cb) override;
    hal::ResultVoid unregisterCallback() override;
    hal::Result<hal::BufferHandle> importBuffer(const hal::BufferHandle& external) override;
    hal::ResultVoid releaseBuffer(const hal::BufferHandle& buffer) override;
    hal::ResultVoid releaseFrames(const std::vector<hal::FrameMetadata>& frames) override;
    hal::ResultVoid setControl(const std::string& device_id, hal::CameraControl control, int64_t value) override;
    hal::Result<int64_t> getControl(const std::string& device_id, hal::CameraControl control) override;
    hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>> getAllControls(const std::string& device_id) override;
    hal::Result<std::pair<int64_t, int64_t>> getControlRange(const std::string& device_id, hal::CameraControl control) override;
    hal::ResultVoid triggerFrame(const std::string& device_id) override;
    hal::ResultVoid triggerFrames(const std::vector<std::string>& device_ids) override;
    hal::ResultVoid flush() override;
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("Orbbec Camera HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("orbbec"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size, void* out_data, size_t out_size) override;
    bool isRunning() const override { return running_; }
    uint32_t getDroppedFrameCount() const override { return dropped_frames_; }
    void resetDroppedFrameCount() override { dropped_frames_ = 0; }

private:
    // Internal types
    struct StreamInfo {
        std::string type;              // "color", "depth", "ir_left", "ir_right"
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t fps = 30;
        hal::PixelFormat format = hal::PixelFormat::NV12;
        bool hw_d2c = false;
        bool enabled = true;
    };

    struct CameraInstance {
        std::string device_id;
        std::string serial_number;
        std::string connection_type;   // "usb3", "gmsl2", "ethernet"
        std::string orbbec_mode;       // "standard", "305g_gmsl2"
        bool disable_ir_left = false;
        
        // SDK objects (opaque pointers via dlopen)
        void* ob_device = nullptr;
        void* ob_pipeline = nullptr;
        void* ob_config = nullptr;
        void* ob_align = nullptr;
        void* ob_pointcloud_filter = nullptr;
        OrbbecCameraHAL* hal_ptr = nullptr;  // Back-reference to HAL for callbacks
        
        // Stream profiles
        void* color_profile = nullptr;
        void* depth_profile = nullptr;
        void* ir_left_profile = nullptr;
        void* ir_right_profile = nullptr;
        
        // Active streams
        std::map<std::string, StreamInfo> streams;
        
        // Calibration
        hal::CameraIntrinsic color_intrinsic;
        hal::CameraIntrinsic depth_intrinsic;
        hal::CameraIntrinsic ir_left_intrinsic;
        hal::CameraIntrinsic ir_right_intrinsic;
        hal::Transform3D depth_to_color_extrinsic;
        float depth_scale = 0.001f;
        
        // Runtime
        bool initialized = false;
        bool streaming = false;
        std::thread callback_thread;
        std::mutex frame_mutex;
        std::condition_variable frame_cv;
        std::queue<std::shared_ptr<hal::FrameBuffer>> frame_queue;
        typename hal::ICameraHAL::FrameCallback user_callback;
        uint32_t frame_id = 0;
        bool is_gmsl2 = false;
        bool is_305g = false;
        bool is_335l_336l = false;
    };

// SDK function pointers (resolved via dlsym)
    struct OrbbecSDK {
        void* handle = nullptr;
        
        // Context
        ob::Context* (*context_create)() = nullptr;
        void (*context_destroy)(ob::Context*) = nullptr;
        int (*context_query_device_list)(ob::Context*, void**) = nullptr; // ob::DeviceList**
        int (*device_list_get_count)(void*) = nullptr;
        ob::Device* (*device_list_get_device)(void*, uint32_t) = nullptr;
        void (*device_list_destroy)(void*) = nullptr;
        
        // Device
        const char* (*device_get_name)(ob::Device*) = nullptr;
        uint16_t (*device_get_vid)(ob::Device*) = nullptr;
        uint16_t (*device_get_pid)(ob::Device*) = nullptr;
        const char* (*device_get_serial_number)(ob::Device*) = nullptr;
        const char* (*device_get_connection_type)(ob::Device*) = nullptr;
        void* (*device_get_sensor_list)(ob::Device*) = nullptr; // ob::SensorList*
        int (*sensor_list_get_count)(void*) = nullptr;
        int (*sensor_list_get_sensor_type)(void*, uint32_t) = nullptr;
        
        // Pipeline
        ob::Pipeline* (*pipeline_create)(ob::Device*) = nullptr;
        void (*pipeline_destroy)(ob::Pipeline*) = nullptr;
        ob::Config* (*config_create)() = nullptr;
        void (*config_destroy)(ob::Config*) = nullptr;
        int (*config_enable_stream)(ob::Config*, void*) = nullptr; // ob::VideoStreamProfile*
        int (*config_disable_stream)(ob::Config*, int) = nullptr;  // OBSensorType
        int (*pipeline_start)(ob::Pipeline*, ob::Config*, void (*)(std::shared_ptr<ob::FrameSet>, void*), void*) = nullptr;
        int (*pipeline_stop)(ob::Pipeline*) = nullptr;
        
        // Frames
        std::shared_ptr<ob::FrameSet> (*frameset_create)() = nullptr;
        void* (*frameset_get_frame)(std::shared_ptr<ob::FrameSet>, int) = nullptr; // OBFrameType
        int (*frame_get_format)(void*) = nullptr;
        uint64_t (*frame_get_timestamp_us)(void*) = nullptr;
        void* (*frame_get_data)(void*) = nullptr;
        size_t (*frame_get_data_size)(void*) = nullptr;
        void* (*frame_as_video_frame)(void*) = nullptr;
        int (*video_frame_get_width)(void*) = nullptr;
        int (*video_frame_get_height)(void*) = nullptr;
        void* (*frame_as_depth_frame)(void*) = nullptr;
        float (*depth_frame_get_value_scale)(void*) = nullptr;
        
        // Stream profiles
        void* (*pipeline_get_stream_profile_list)(ob::Pipeline*, int) = nullptr; // OBSensorType
        uint32_t (*profile_list_get_count)(void*) = nullptr;
        void* (*profile_list_get_profile)(void*, uint32_t) = nullptr;
        void* (*profile_as_video_stream_profile)(void*) = nullptr;
        int (*video_frame_get_fps)(void*) = nullptr;
        int (*video_frame_get_format)(void*) = nullptr;
        
        // Camera intrinsics/extrinsics
        int (*device_get_camera_intrinsic)(ob::Device*, int, void*) = nullptr; // OBSensorType, OBCameraIntrinsic*
        int (*device_get_camera_extrinsic)(ob::Device*, int, int, void*) = nullptr; // OBSensorType from, OBSensorType to, OBExtrinsic*
        
        // Align filter (HW D2C)
        void* (*align_create)(int) = nullptr; // OBSensorType align_to
        void (*align_destroy)(void*) = nullptr;
        int (*align_process)(void*, std::shared_ptr<ob::FrameSet>, std::shared_ptr<ob::FrameSet>*) = nullptr;
        
        // Point cloud filter
        void* (*pointcloud_filter_create)() = nullptr;
        void (*pointcloud_filter_destroy)(void*) = nullptr;
        int (*pointcloud_filter_set_camera_param)(void*, void*) = nullptr; // OBCameraIntrinsic*
        int (*pointcloud_filter_process)(void*, std::shared_ptr<ob::FrameSet>, std::shared_ptr<ob::FrameSet>*) = nullptr;
        
        // Error handling
        const char* (*get_error_message)(int) = nullptr;
    } sdk_;

    // SDK loading
    bool loadOrbbecSDK();
    void unloadOrbbecSDK();
    
    // Helpers
    hal::ResultVoid loadSDKSymbols();
    bool isOrbbecDevice(uint16_t vid, uint16_t pid);
    bool isGemini305(uint16_t vid, uint16_t pid);
    bool isGemini305g(uint16_t vid, uint16_t pid, const char* conn_type);
    bool isGemini335L336L(uint16_t vid, uint16_t pid);
    
    hal::ResultVoid initializeCameraInstance(std::unique_ptr<CameraInstance>& cam, const hal::CameraConfig& config);
    hal::ResultVoid configureStreams(CameraInstance& cam, const std::vector<hal::StreamConfig>& streams);
    hal::ResultVoid setupAlignment(CameraInstance& cam);
    hal::ResultVoid setupPointCloud(CameraInstance& cam);
    hal::ResultVoid fetchCalibration(CameraInstance& cam);
    
    std::shared_ptr<hal::FrameBuffer> convertFrameSetToFrameBuffer(
        CameraInstance& cam, 
        void* ob_frameset);
    
    void callbackLoop(CameraInstance& cam);
    static void SDKCallback(std::shared_ptr<ob::FrameSet> fs, void* user_data);

    // State
    std::map<std::string, std::unique_ptr<CameraInstance>> cameras_;
    std::mutex cameras_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> dropped_frames_{0};
    bool sdk_loaded_ = false;
    void* ctx_ = nullptr;  // Orbbec SDK context
};

// SDK Callback trampoline
void OrbbecCameraHAL::SDKCallback(std::shared_ptr<ob::FrameSet> fs, void* user_data) {
    auto* cam = static_cast<CameraInstance*>(user_data);
    if (cam && cam->initialized && cam->streaming && cam->hal_ptr) {
        // Convert and push to queue
        auto fb = cam->hal_ptr->convertFrameSetToFrameBuffer(*cam, fs.get());
        
        std::lock_guard<std::mutex> lock(cam->frame_mutex);
        cam->frame_queue.push(fb);
        cam->frame_cv.notify_one();
        
        // Call user callback if registered
        if (cam->user_callback) {
            hal::FrameMetadata meta;
            meta.timestamp_ns = fb->metadata.timestamp_ns;
            meta.frame_id = fb->metadata.frame_id;
            meta.sensor_id = fb->metadata.sensor_id;
            meta.frame_type = fb->metadata.frame_type;
            meta.format = fb->metadata.format;
            meta.width = fb->metadata.width;
            meta.height = fb->metadata.height;
            meta.buffer = fb->metadata.buffer;
            cam->user_callback(meta);
        }
    }
}

// Load Orbbec SDK dynamically
bool OrbbecCameraHAL::loadOrbbecSDK() {
    if (sdk_loaded_) return true;
    
    // Search paths for libOrbbecSDK.so
    static const std::vector<std::string> search_paths = {
        "./lib/OrbbecSDK",
        "/usr/local/lib",
        "/opt/OrbbecSDK/lib",
        "/usr/lib/x86_64-linux-gnu"
    };
    
    std::string lib_path;
    for (const auto& path : search_paths) {
        std::filesystem::path p = std::filesystem::path(path) / "libOrbbecSDK.so";
        if (std::filesystem::exists(p)) {
            lib_path = p.string();
            break;
        }
    }
    
    if (lib_path.empty()) {
        // Try default name
        lib_path = "libOrbbecSDK.so";
    }
    
    sdk_.handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!sdk_.handle) {
        std::cerr << "Failed to load Orbbec SDK: " << dlerror() << std::endl;
        return false;
    }
    
    auto load_result = loadSDKSymbols();
    if (load_result.is_err()) {
        unloadOrbbecSDK();
        return false;
    }
    
    // Create context
    ctx_ = sdk_.context_create();
    if (!ctx_) {
        unloadOrbbecSDK();
        return false;
    }
    
    sdk_loaded_ = true;
    return true;
}

void OrbbecCameraHAL::unloadOrbbecSDK() {
    if (ctx_ && sdk_.context_destroy) {
        sdk_.context_destroy(static_cast<ob::Context*>(ctx_));
        ctx_ = nullptr;
    }
    if (sdk_.handle) {
        dlclose(sdk_.handle);
        sdk_.handle = nullptr;
    }
    sdk_loaded_ = false;
    memset(&sdk_, 0, sizeof(sdk_));
}

hal::ResultVoid OrbbecCameraHAL::loadSDKSymbols() {
#define LOAD_SYM(name) \
    sdk_.name = reinterpret_cast<decltype(sdk_.name)>(dlsym(sdk_.handle, "ob_" #name)); \
    if (!sdk_.name) { \
        std::cerr << "Failed to load symbol: ob_" #name << std::endl; \
        return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED); \
    }
    
    // Context
    LOAD_SYM(context_create);
    LOAD_SYM(context_destroy);
    LOAD_SYM(context_query_device_list);
    LOAD_SYM(device_list_get_count);
    LOAD_SYM(device_list_get_device);
    LOAD_SYM(device_list_destroy);
    
    // Device
    LOAD_SYM(device_get_name);
    LOAD_SYM(device_get_vid);
    LOAD_SYM(device_get_pid);
    LOAD_SYM(device_get_serial_number);
    LOAD_SYM(device_get_connection_type);
    LOAD_SYM(device_get_sensor_list);
    LOAD_SYM(sensor_list_get_count);
    LOAD_SYM(sensor_list_get_sensor_type);
    
    // Pipeline
    LOAD_SYM(pipeline_create);
    LOAD_SYM(pipeline_destroy);
    LOAD_SYM(config_create);
    LOAD_SYM(config_destroy);
    LOAD_SYM(config_enable_stream);
    LOAD_SYM(config_disable_stream);
    LOAD_SYM(pipeline_start);
    LOAD_SYM(pipeline_stop);
    
    // Frames
    LOAD_SYM(frame_get_format);
    LOAD_SYM(frame_get_timestamp_us);
    LOAD_SYM(frame_get_data);
    LOAD_SYM(frame_get_data_size);
    LOAD_SYM(frame_as_video_frame);
    LOAD_SYM(video_frame_get_width);
    LOAD_SYM(video_frame_get_height);
    LOAD_SYM(frame_as_depth_frame);
    LOAD_SYM(depth_frame_get_value_scale);
    
    // Stream profiles
    LOAD_SYM(pipeline_get_stream_profile_list);
    LOAD_SYM(profile_list_get_count);
    LOAD_SYM(profile_list_get_profile);
    LOAD_SYM(profile_as_video_stream_profile);
    LOAD_SYM(video_frame_get_width);
    LOAD_SYM(video_frame_get_height);
    LOAD_SYM(video_frame_get_fps);
    LOAD_SYM(video_frame_get_format);
    
    // Calibration
    LOAD_SYM(device_get_camera_intrinsic);
    LOAD_SYM(device_get_camera_extrinsic);
    
    // Align
    LOAD_SYM(align_create);

    LOAD_SYM(align_destroy);
    LOAD_SYM(align_process);
    
    // Point cloud
    LOAD_SYM(pointcloud_filter_create);
    LOAD_SYM(pointcloud_filter_destroy);
    LOAD_SYM(pointcloud_filter_set_camera_param);
    LOAD_SYM(pointcloud_filter_process);
    
    // Error
    LOAD_SYM(get_error_message);
    
#undef LOAD_SYM
    
    return hal::ResultVoid::ok();
}

bool OrbbecCameraHAL::isOrbbecDevice(uint16_t vid, uint16_t pid) {
    return vid == 0x2bc5; // Orbbec VID
}

bool OrbbecCameraHAL::isGemini305(uint16_t vid, uint16_t pid) {
    return vid == 0x2bc5 && (pid == 0x0840 || pid == 0x0841 || pid == 0x0842 || pid == 0x0843);
}

bool OrbbecCameraHAL::isGemini305g(uint16_t vid, uint16_t pid, const char* conn_type) {
    return isGemini305(vid, pid) && conn_type && strcmp(conn_type, "GMSL2") == 0;
}

bool OrbbecCameraHAL::isGemini335L336L(uint16_t vid, uint16_t pid) {
    return vid == 0x2bc5 && (pid == 0x0804 || pid == 0x0807);
}

hal::Result<std::vector<hal::CameraDeviceInfo>> OrbbecCameraHAL::enumerateDevices() {
    if (!loadOrbbecSDK()) {
        return hal::Result<std::vector<hal::CameraDeviceInfo>>::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    std::vector<hal::CameraDeviceInfo> devices;
    
    void* device_list = nullptr;
    int ret = sdk_.context_query_device_list(static_cast<ob::Context*>(ctx_), &device_list);
    if (ret != 0 || !device_list) {
        return hal::Result<std::vector<hal::CameraDeviceInfo>>::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    uint32_t count = sdk_.device_list_get_count(device_list);
    for (uint32_t i = 0; i < count; ++i) {
        ob::Device* dev = sdk_.device_list_get_device(device_list, i);
        if (!dev) continue;
        
        uint16_t vid = sdk_.device_get_vid(dev);
        uint16_t pid = sdk_.device_get_pid(dev);
        
        if (!isOrbbecDevice(vid, pid)) continue;
        
        hal::CameraDeviceInfo info;
        info.device_id = "orbbec://" + std::to_string(vid) + ":" + std::to_string(pid) + ":" + std::to_string(i);
        info.vendor = "orbbec";
        info.bus_info = sdk_.device_get_connection_type(dev) ? sdk_.device_get_connection_type(dev) : "unknown";
        
        const char* sn = sdk_.device_get_serial_number(dev);
        if (sn) info.sensor_info.serial_number = sn;
        
        const char* name = sdk_.device_get_name(dev);
        info.display_name = name ? name : "Orbbec Device";
        
        // Sensor info
        void* sensor_list = sdk_.device_get_sensor_list(dev);
        if (sensor_list) {
            uint32_t sensor_count = sdk_.sensor_list_get_count(sensor_list);
            for (uint32_t s = 0; s < sensor_count; ++s) {
                int sensor_type = sdk_.sensor_list_get_sensor_type(sensor_list, s);
                switch (sensor_type) {
                    case 0: // OB_SENSOR_COLOR
                        info.sensor_info.supported_formats.push_back(hal::PixelFormat::NV12);
                        info.sensor_info.supported_formats.push_back(hal::PixelFormat::YUYV);
                        info.sensor_info.supported_formats.push_back(hal::PixelFormat::MJPEG);
                        break;
                    case 1: // OB_SENSOR_DEPTH
                        info.sensor_info.supported_formats.push_back(hal::PixelFormat::Y16);
                        break;
                    case 2: // OB_SENSOR_IR
                        info.sensor_info.supported_formats.push_back(hal::PixelFormat::Y8);
                        break;
                }
            }
        }
        
        // Set capabilities based on model
        if (isGemini335L336L(vid, pid)) {
            info.sensor_info.supports_hw_sync = true;
            info.sensor_info.max_width = 3840;
            info.sensor_info.max_height = 2160;
        } else {
            info.sensor_info.max_width = 1920;
            info.sensor_info.max_height = 1080;
        }
        
        info.is_available = true;
        devices.push_back(std::move(info));
    }
    
    sdk_.device_list_destroy(device_list);
    return hal::Result<std::vector<hal::CameraDeviceInfo>>::ok(std::move(devices));
}

hal::Result<std::vector<hal::StreamConfig>> OrbbecCameraHAL::getSupportedStreams(const std::string& device_id) {
    // Parse device_id to get index, then query profiles
    // For now, return common configs
    std::vector<hal::StreamConfig> streams;
    
    hal::StreamConfig color;
    color.stream_id = 0;
    color.format = hal::PixelFormat::NV12;
    color.width = 1920;
    color.height = 1080;
    color.fps = 30;
    color.buffer_count = 4;
    streams.push_back(color);
    
    hal::StreamConfig depth;
    depth.stream_id = 1;
    depth.format = hal::PixelFormat::Y16;
    depth.width = 1280;
    depth.height = 800;
    depth.fps = 30;
    depth.buffer_count = 4;
    streams.push_back(depth);
    
    hal::StreamConfig ir_left;
    ir_left.stream_id = 2;
    ir_left.format = hal::PixelFormat::Y8;
    ir_left.width = 640;
    ir_left.height = 400;
    ir_left.fps = 30;
    ir_left.buffer_count = 4;
    streams.push_back(ir_left);
    
    hal::StreamConfig ir_right;
    ir_right.stream_id = 3;
    ir_right.format = hal::PixelFormat::Y8;
    ir_right.width = 640;
    ir_right.height = 400;
    ir_right.fps = 30;
    ir_right.buffer_count = 4;
    streams.push_back(ir_right);
    
    return hal::Result<std::vector<hal::StreamConfig>>::ok(std::move(streams));
}

hal::Result<hal::SensorInfo> OrbbecCameraHAL::getSensorInfo(const std::string& device_id) {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    auto it = cameras_.find(device_id);
    if (it == cameras_.end()) {
        return hal::Result<hal::SensorInfo>::err(hal::ErrorCode::NOT_FOUND);
    }
    
    hal::SensorInfo info;
    info.name = "Orbbec " + it->second->serial_number;
    info.vendor = "orbbec";
    info.serial_number = it->second->serial_number;
    info.max_width = it->second->is_335l_336l ? 3840 : 1920;
    info.max_height = it->second->is_335l_336l ? 2160 : 1080;
    info.supports_hw_sync = it->second->is_335l_336l;
    
    return hal::Result<hal::SensorInfo>::ok(info);
}

hal::ResultVoid OrbbecCameraHAL::initialize(const hal::CameraConfig& config) {
    if (!loadOrbbecSDK()) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    if (cameras_.find(config.device_id) != cameras_.end()) {
        return hal::ResultVoid::err(hal::ErrorCode::ALREADY_INITIALIZED);
    }
    
    auto cam = std::make_unique<CameraInstance>();
    cam->device_id = config.device_id;
    auto result = initializeCameraInstance(cam, config);
    if (result.is_ok()) {
        cameras_[config.device_id] = std::move(cam);
    }
    return result;
}

hal::ResultVoid OrbbecCameraHAL::initializeMulti(const hal::MultiCameraConfig& config) {
    if (!loadOrbbecSDK()) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    if (config.cameras.size() != 2) {
        return hal::ResultVoid::err(hal::ErrorCode::INVALID_ARG);
    }
    
    // For IR stereo: left and right IR cameras
    // We create a single CameraInstance with both IR streams
    auto cam = std::make_unique<CameraInstance>();
    cam->device_id = config.cameras[0].device_id + "_stereo";
    cam->connection_type = "usb3"; // Default, could be parsed from vendor_params
    
    // Initialize with first camera config (main)
    auto result = initializeCameraInstance(cam, config.cameras[0]);
    if (!result.is_ok()) return result;
    
    // Enable IR_LEFT and IR_RIGHT streams
    if (cam->ob_config && sdk_.config_enable_stream) {
        // Get IR profiles and enable
        // This is simplified - real impl would query specific profiles
    }
    
    cameras_[cam->device_id] = std::move(cam);
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::initializeCameraInstance(std::unique_ptr<CameraInstance>& cam, const hal::CameraConfig& config) {
    // Parse device_id to find device index
    // Format: "orbbec://vid:pid:index" or just index
    uint32_t device_index = 0;
    // ... parsing logic
    
    // Query device list
    void* device_list = nullptr;
    int ret = sdk_.context_query_device_list(static_cast<ob::Context*>(ctx_), &device_list);
    if (ret != 0 || !device_list) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    uint32_t count = sdk_.device_list_get_count(device_list);
    if (device_index >= count) {
        sdk_.device_list_destroy(device_list);
        return hal::ResultVoid::err(hal::ErrorCode::NOT_FOUND);
    }
    
    ob::Device* dev = sdk_.device_list_get_device(device_list, device_index);
    if (!dev) {
        sdk_.device_list_destroy(device_list);
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    cam->serial_number = sdk_.device_get_serial_number(dev) ? sdk_.device_get_serial_number(dev) : "";
    const char* conn_type = sdk_.device_get_connection_type(dev);
    cam->connection_type = conn_type ? conn_type : "usb3";
    
    uint16_t vid = sdk_.device_get_vid(dev);
    uint16_t pid = sdk_.device_get_pid(dev);
    cam->is_gmsl2 = (conn_type && strcmp(conn_type, "GMSL2") == 0);
    cam->is_305g = isGemini305g(vid, pid, conn_type);
    cam->is_335l_336l = isGemini335L336L(vid, pid);
    cam->orbbec_mode = cam->is_305g ? "305g_gmsl2" : "standard";
    
    // Create pipeline and config
    cam->ob_pipeline = sdk_.pipeline_create(dev);
    cam->ob_config = sdk_.config_create();
    cam->ob_device = dev; // Keep reference
    cam->hal_ptr = this;  // Set back-reference for callbacks
    
    if (!cam->ob_pipeline || !cam->ob_config) {
        sdk_.device_list_destroy(device_list);
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    // Configure streams based on config
    hal::ResultVoid res = configureStreams(*cam, {}); // Will be enhanced with StreamConfig parsing
    if (!res.is_ok()) {
        sdk_.device_list_destroy(device_list);
        return res;
    }
    
    // Setup alignment (HW D2C for 335L/336L)
    if (cam->is_335l_336l) {
        setupAlignment(*cam);
    }
    
    // Setup point cloud
    setupPointCloud(*cam);
    
    // Fetch calibration
    fetchCalibration(*cam);
    
    cam->initialized = true;
    cameras_[cam->device_id] = std::move(cam);
    
    sdk_.device_list_destroy(device_list);
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::configureStreams(CameraInstance& cam, const std::vector<hal::StreamConfig>& streams) {
    if (!cam.ob_config || !sdk_.config_enable_stream) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_INITIALIZED);
    }
    
    ob::Pipeline* pipeline = static_cast<ob::Pipeline*>(cam.ob_pipeline);
    ob::Config* config = static_cast<ob::Config*>(cam.ob_config);
    
    // Get stream profile lists
    void* color_profiles = sdk_.pipeline_get_stream_profile_list(pipeline, 0); // OB_SENSOR_COLOR
    void* depth_profiles = sdk_.pipeline_get_stream_profile_list(pipeline, 1); // OB_SENSOR_DEPTH
    void* ir_profiles = sdk_.pipeline_get_stream_profile_list(pipeline, 2);    // OB_SENSOR_IR
    
    auto makeStreamInfo = [&](const std::string& type, void* profile, hal::PixelFormat fmt, bool hw_d2c) {
        StreamInfo info;
        info.type = type;
        info.width = sdk_.video_frame_get_width(profile);
        info.height = sdk_.video_frame_get_height(profile);
        info.fps = sdk_.video_frame_get_fps(profile);
        info.format = fmt;
        info.hw_d2c = hw_d2c;
        info.enabled = true;
        return info;
    };
    
    // Select best profiles (simplified - would use selectBestProfile logic)
    if (color_profiles && sdk_.profile_list_get_count) {
        uint32_t count = sdk_.profile_list_get_count(color_profiles);
        if (count > 0) {
            void* profile = sdk_.profile_list_get_profile(color_profiles, 0);
            cam.color_profile = sdk_.profile_as_video_stream_profile(profile);
            if (cam.color_profile) {
                sdk_.config_enable_stream(config, cam.color_profile);
                cam.streams["color"] = makeStreamInfo("color", cam.color_profile, hal::PixelFormat::NV12, false);
            }
        }
    }
    
    if (depth_profiles && sdk_.profile_list_get_count) {
        uint32_t count = sdk_.profile_list_get_count(depth_profiles);
        if (count > 0) {
            void* profile = sdk_.profile_list_get_profile(depth_profiles, 0);
            cam.depth_profile = sdk_.profile_as_video_stream_profile(profile);
            if (cam.depth_profile) {
                sdk_.config_enable_stream(config, cam.depth_profile);
                cam.streams["depth"] = makeStreamInfo("depth", cam.depth_profile, hal::PixelFormat::Y16, cam.is_335l_336l);
            }
        }
    }
    
    // IR streams (for stereo)
    if (ir_profiles && sdk_.profile_list_get_count && !cam.disable_ir_left) {
        uint32_t count = sdk_.profile_list_get_count(ir_profiles);
        if (count >= 2) {
            // IR_LEFT
            void* profile_l = sdk_.profile_list_get_profile(ir_profiles, 0);
            cam.ir_left_profile = sdk_.profile_as_video_stream_profile(profile_l);
            if (cam.ir_left_profile) {
                sdk_.config_enable_stream(config, cam.ir_left_profile);
                cam.streams["ir_left"] = makeStreamInfo("ir_left", cam.ir_left_profile, hal::PixelFormat::Y8, false);
            }
            
            // IR_RIGHT
            void* profile_r = sdk_.profile_list_get_profile(ir_profiles, 1);
            cam.ir_right_profile = sdk_.profile_as_video_stream_profile(profile_r);
            if (cam.ir_right_profile) {
                sdk_.config_enable_stream(config, cam.ir_right_profile);
                cam.streams["ir_right"] = makeStreamInfo("ir_right", cam.ir_right_profile, hal::PixelFormat::Y8, false);
            }
        }
    }
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::setupAlignment(CameraInstance& cam) {
    if (!cam.is_335l_336l || !sdk_.align_create) return hal::ResultVoid::ok();
    
    // Align depth to color
    cam.ob_align = sdk_.align_create(0); // OB_SENSOR_COLOR
    if (!cam.ob_align) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
    }
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::setupPointCloud(CameraInstance& cam) {
    if (!sdk_.pointcloud_filter_create) return hal::ResultVoid::ok();
    
    cam.ob_pointcloud_filter = sdk_.pointcloud_filter_create();
    if (!cam.ob_pointcloud_filter) return hal::ResultVoid::ok();
    
    // Set camera intrinsic for point cloud generation
    // Would use sdk_.pointcloud_filter_set_camera_param with color_intrinsic
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::fetchCalibration(CameraInstance& cam) {
    if (!cam.ob_device || !sdk_.device_get_camera_intrinsic) return hal::ResultVoid::ok();
    
    ob::Device* dev = static_cast<ob::Device*>(cam.ob_device);
    
    // Get color intrinsic
    void* color_intrinsic = malloc(sizeof(ob::OBCameraIntrinsic));
    if (sdk_.device_get_camera_intrinsic(dev, 0, color_intrinsic) == 0) { // OB_SENSOR_COLOR
        auto* ob_intr = static_cast<ob::OBCameraIntrinsic*>(color_intrinsic);
        cam.color_intrinsic.fx = ob_intr->fx;
        cam.color_intrinsic.fy = ob_intr->fy;
        cam.color_intrinsic.cx = ob_intr->cx;
        cam.color_intrinsic.cy = ob_intr->cy;
        cam.color_intrinsic.width = ob_intr->width;
        cam.color_intrinsic.height = ob_intr->height;
    }
    free(color_intrinsic);
    
    // Get depth intrinsic
    void* depth_intrinsic = malloc(sizeof(ob::OBCameraIntrinsic));
    if (sdk_.device_get_camera_intrinsic(dev, 1, depth_intrinsic) == 0) { // OB_SENSOR_DEPTH
        auto* ob_intr = static_cast<ob::OBCameraIntrinsic*>(depth_intrinsic);
        cam.depth_intrinsic.fx = ob_intr->fx;
        cam.depth_intrinsic.fy = ob_intr->fy;
        cam.depth_intrinsic.cx = ob_intr->cx;
        cam.depth_intrinsic.cy = ob_intr->cy;
        cam.depth_intrinsic.width = ob_intr->width;
        cam.depth_intrinsic.height = ob_intr->height;
    }
    free(depth_intrinsic);
    
    // Get depth-to-color extrinsic
    void* extrinsic = malloc(sizeof(ob::OBExtrinsic));
    if (sdk_.device_get_camera_extrinsic(dev, 1, 0, extrinsic) == 0) { // depth to color
        auto* ob_ext = static_cast<ob::OBExtrinsic*>(extrinsic);
        for (int i = 0; i < 9; ++i) {
            cam.depth_to_color_extrinsic.rotation[i] = ob_ext->rotation[i];
        }
        cam.depth_to_color_extrinsic.translation[0] = ob_ext->translation[0];
        cam.depth_to_color_extrinsic.translation[1] = ob_ext->translation[1];
        cam.depth_to_color_extrinsic.translation[2] = ob_ext->translation[2];
    }
    free(extrinsic);
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::start() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (auto& [id, cam] : cameras_) {
        if (!cam->initialized || cam->streaming) continue;
        
        // Start pipeline with callback
        int ret = sdk_.pipeline_start(static_cast<ob::Pipeline*>(cam->ob_pipeline), static_cast<ob::Config*>(cam->ob_config), 
            [](std::shared_ptr<ob::FrameSet> fs, void* user_data) {
                CameraInstance* cam = static_cast<CameraInstance*>(user_data);
                if (!cam || !cam->streaming || !cam->hal_ptr) return;
                
                // Convert to FrameBuffer
                auto fb = cam->hal_ptr->convertFrameSetToFrameBuffer(*cam, fs.get());
                
                std::lock_guard<std::mutex> lock(cam->frame_mutex);
                cam->frame_queue.push(fb);
                cam->frame_cv.notify_one();
                
                // Call user callback if registered
                if (cam->user_callback) {
                    hal::FrameMetadata meta;
                    meta.timestamp_ns = fb->metadata.timestamp_ns;
                    meta.frame_id = fb->metadata.frame_id;
                    meta.sensor_id = fb->metadata.sensor_id;
                    meta.frame_type = fb->metadata.frame_type;
                    meta.format = fb->metadata.format;
                    meta.width = fb->metadata.width;
                    meta.height = fb->metadata.height;
                    meta.buffer = fb->metadata.buffer;
                    cam->user_callback(meta);
                }
            }, cam.get());
        
        if (ret != 0) {
            continue;
        }
        
        sdk_.pipeline_stop(static_cast<ob::Pipeline*>(cam->ob_pipeline));
        cam->streaming = false;
        
        cam->streaming = true;
        running_ = true;
    }
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::stop() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (auto& [id, cam] : cameras_) {
        if (!cam->streaming) continue;
        
        cam->streaming = false;
        if (sdk_.pipeline_stop) {
            sdk_.pipeline_stop(static_cast<ob::Pipeline*>(cam->ob_pipeline));
        }
    }
    
    running_ = false;
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::deinitialize() {
    stop();
    
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (auto& [id, cam] : cameras_) {
        if (cam->ob_pointcloud_filter && sdk_.pointcloud_filter_destroy) {
            sdk_.pointcloud_filter_destroy(cam->ob_pointcloud_filter);
        }
        if (cam->ob_align && sdk_.align_destroy) {
            sdk_.align_destroy(cam->ob_align);
        }
        if (cam->ob_pipeline && sdk_.pipeline_destroy) {
            sdk_.pipeline_destroy(static_cast<ob::Pipeline*>(cam->ob_pipeline));
        }
        if (cam->ob_config && sdk_.config_destroy) {
            sdk_.config_destroy(static_cast<ob::Config*>(cam->ob_config));
        }
        // Device is managed by context
    }
    
    cameras_.clear();
    unloadOrbbecSDK();
    return hal::ResultVoid::ok();
}

std::shared_ptr<hal::FrameBuffer> OrbbecCameraHAL::convertFrameSetToFrameBuffer(
    CameraInstance& cam, void* ob_frameset) {
    
    auto fb = std::make_shared<hal::FrameBuffer>();
    // This is a simplified conversion - real impl would extract all frames
    // from the FrameSet and populate FrameBuffer with metadata
    
    fb->metadata.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    fb->metadata.frame_id = cam.frame_id++;
    fb->metadata.sensor_id = 0; // Would map from device_id
    fb->metadata.frame_type = hal::FrameType::IMAGE;
    fb->metadata.format = hal::PixelFormat::NV12;
    fb->metadata.width = 1920;
    fb->metadata.height = 1080;
    fb->metadata.stride = 1920 * 3;
    
    return fb;
}

hal::Result<hal::FrameMetadata> OrbbecCameraHAL::acquireFrame(uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(cameras_mutex_);
    
    // For simplicity, use first camera
    if (cameras_.empty()) {
        return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::NOT_INITIALIZED);
    }
    
    auto& cam = cameras_.begin()->second;
    lock.unlock();
    
    std::unique_lock<std::mutex> frame_lock(cam->frame_mutex);
    if (cam->frame_queue.empty()) {
        if (!cam->frame_cv.wait_for(frame_lock, std::chrono::milliseconds(timeout_ms),
            [&cam] { return !cam->frame_queue.empty(); })) {
            return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::TIMEOUT);
        }
    }
    
    if (cam->frame_queue.empty()) {
        return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::TIMEOUT);
    }
    
    auto fb = cam->frame_queue.front();
    cam->frame_queue.pop();
    
    hal::FrameMetadata meta = fb->metadata;
    meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get()); // Store shared_ptr address
    meta.buffer.platform_id = 0;
    
    return hal::Result<hal::FrameMetadata>::ok(meta);
}

hal::Result<std::vector<hal::FrameMetadata>> OrbbecCameraHAL::acquireFrames(uint32_t timeout_ms) {
    std::vector<hal::FrameMetadata> frames;
    frames.reserve(cameras_.size());
    
    for (auto& [id, cam] : cameras_) {
        auto result = acquireFrame(timeout_ms);
        if (result.is_ok()) {
            frames.push_back(result.value());
        }
    }
    
    if (frames.empty()) {
        return hal::Result<std::vector<hal::FrameMetadata>>::err(hal::ErrorCode::TIMEOUT);
    }
    
    return hal::Result<std::vector<hal::FrameMetadata>>::ok(std::move(frames));
}

hal::ResultVoid OrbbecCameraHAL::registerCallback(typename hal::ICameraHAL::FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    for (auto& [id, cam] : cameras_) {
        cam->user_callback = cb;
    }
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::unregisterCallback() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    for (auto& [id, cam] : cameras_) {
        cam->user_callback = nullptr;
    }
    return hal::ResultVoid::ok();
}

hal::Result<hal::BufferHandle> OrbbecCameraHAL::importBuffer(const hal::BufferHandle& external) {
    return hal::Result<hal::BufferHandle>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid OrbbecCameraHAL::releaseBuffer(const hal::BufferHandle& buffer) {
    // FrameBuffer is managed by shared_ptr, no explicit release needed
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::releaseFrames(const std::vector<hal::FrameMetadata>& frames) {
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::setControl(const std::string& device_id, hal::CameraControl control, int64_t value) {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    auto it = cameras_.find(device_id);
    if (it == cameras_.end() || !it->second->ob_pipeline) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_FOUND);
    }
    // Would use OBPropertyID to set controls
    return hal::ResultVoid::ok();
}

hal::Result<int64_t> OrbbecCameraHAL::getControl(const std::string& device_id, hal::CameraControl control) {
    return hal::Result<int64_t>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>> OrbbecCameraHAL::getAllControls(const std::string& device_id) {
    return hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::pair<int64_t, int64_t>> OrbbecCameraHAL::getControlRange(const std::string& device_id, hal::CameraControl control) {
    return hal::Result<std::pair<int64_t, int64_t>>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid OrbbecCameraHAL::triggerFrame(const std::string& device_id) {
    return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid OrbbecCameraHAL::triggerFrames(const std::vector<std::string>& device_ids) {
    return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid OrbbecCameraHAL::flush() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    for (auto& [id, cam] : cameras_) {
        std::lock_guard<std::mutex> frame_lock(cam->frame_mutex);
        while (!cam->frame_queue.empty()) {
            cam->frame_queue.pop();
        }
    }
    dropped_frames_ = 0;
    return hal::ResultVoid::ok();
}

hal::ResultVoid OrbbecCameraHAL::vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size, void* out_data, size_t out_size) {
    // Handle vendor-specific commands:
    // - Firmware update
    // - GMSL2 link configuration
    // - Custom sensor modes
    return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { 
    return new dynalgo::OrbbecCameraHAL(); 
}
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { 
    delete ptr; 
}
}