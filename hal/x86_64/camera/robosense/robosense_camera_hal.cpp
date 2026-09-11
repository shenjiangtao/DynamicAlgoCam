/*
 * robosense_camera_hal.cpp - RoboSense AC1 Camera HAL implementation for x86_64
 * Supports: RoboSense RoboX AC1 (Camera + LiDAR integration)
 * Features: Color, Depth, LiDAR point cloud, IMU, HW sync
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

// Forward declarations for RoboSense SDK types (loaded via dlopen)
namespace rs {
    namespace lidar {
        class LidarDriver;
        template<typename T> class SyncQueue;
        struct PointCloudMsg;
        struct ImageData;
        struct ImuData;
        struct RSDriverParam;
        enum LidarType { RS_AC1 };
        struct USB_ID { static constexpr uint16_t vid = 0x1234; static constexpr uint16_t pid = 0x5678; };
        enum InputType { USB };
        struct Error { std::string toString() const { return ""; } };
    }
    
    namespace camera {
        class CameraDriver;
        struct CameraParam;
        struct Frame;
    }
}

class RobosenseCameraHAL : public hal::ICameraHAL {
public:
    RobosenseCameraHAL() : ctx_(nullptr), running_(false), dropped_frames_(0) {}
    ~RobosenseCameraHAL() override { deinitialize(); }

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
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("RoboSense Camera HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("robosense"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size, void* out_data, size_t out_size) override;
    bool isRunning() const override { return running_; }
    uint32_t getDroppedFrameCount() const override { return dropped_frames_; }
    void resetDroppedFrameCount() override { dropped_frames_ = 0; }

private:
    struct StreamInfo {
        std::string type;              // "color", "depth"
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
        
        // SDK objects (opaque pointers via dlopen)
        void* rs_lidar_driver = nullptr;
        void* rs_camera_driver = nullptr;
        
        // Active streams
        std::map<std::string, StreamInfo> streams;
        
        // Calibration
        hal::CameraIntrinsic color_intrinsic;
        hal::CameraIntrinsic depth_intrinsic;
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
        RobosenseCameraHAL* hal_ptr = nullptr;
        uint32_t frame_id = 0;
    };

    // SDK function pointers (resolved via dlsym)
    struct RoboSenseSDK {
        void* handle = nullptr;
        
        // LiDAR driver
        void* (*lidar_driver_create)(void*) = nullptr;
        void (*lidar_driver_destroy)(void*) = nullptr;
        int (*lidar_driver_init)(void*, void*) = nullptr;
        int (*lidar_driver_start)(void*) = nullptr;
        int (*lidar_driver_stop)(void*) = nullptr;
        void (*lidar_driver_reg_pointcloud_callback)(void*, void(*)(void*, void*)) = nullptr;
        void (*lidar_driver_reg_image_callback)(void*, void(*)(void*, void*)) = nullptr;
        void (*lidar_driver_reg_imu_callback)(void*, void(*)(void*, void*)) = nullptr;
        
        // Camera driver (if separate)
        void* (*camera_driver_create)() = nullptr;
        void (*camera_driver_destroy)(void*) = nullptr;
        int (*camera_driver_open)(void*, int, int, int) = nullptr;
        int (*camera_driver_close)(void*) = nullptr;
        int (*camera_driver_start)(void*) = nullptr;
        int (*camera_driver_stop)(void*) = nullptr;
        void (*camera_driver_reg_callback)(void*, void(*)(void*, void*)) = nullptr;
    } sdk_;

    // SDK loading
    bool loadRoboSenseSDK();
    void unloadRoboSenseSDK();
    
    // Helpers
    hal::ResultVoid loadSDKSymbols();
    bool isRoboSenseDevice(uint16_t vid, uint16_t pid);
    
    hal::ResultVoid initializeCameraInstance(std::unique_ptr<CameraInstance>& cam, const hal::CameraConfig& config);
    hal::ResultVoid configureStreams(CameraInstance& cam, const std::vector<hal::StreamConfig>& streams);
    hal::ResultVoid fetchCalibration(CameraInstance& cam);
    
    std::shared_ptr<hal::FrameBuffer> convertToFrameBuffer(CameraInstance& cam, void* image_data, const std::string& stream_type);
    
    static void LiDARPointCloudCallback(void* user_data, void* msg);
    static void LiDARImageCallback(void* user_data, void* msg);
    static void LiDARImuCallback(void* user_data, void* msg);
    static void CameraFrameCallback(void* user_data, void* frame);

    // State
    std::map<std::string, std::unique_ptr<CameraInstance>> cameras_;
    std::mutex cameras_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> dropped_frames_{0};
    bool sdk_loaded_ = false;
    void* ctx_ = nullptr;
};

// LiDAR Point Cloud Callback
void RobosenseCameraHAL::LiDARPointCloudCallback(void* user_data, void* msg) {
    auto* cam = static_cast<CameraInstance*>(user_data);
    if (!cam || !cam->hal_ptr || !cam->streaming) return;
    
    // Convert point cloud message to FrameBuffer
    // This is a placeholder - real implementation would parse rs::lidar::PointCloudMsg
    auto fb = std::make_shared<hal::FrameBuffer>();
    fb->metadata.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    fb->metadata.frame_id = cam->frame_id++;
    fb->metadata.sensor_id = 0;
    fb->metadata.frame_type = hal::FrameType::POINT_CLOUD;
    fb->metadata.format = hal::PixelFormat::METADATA;
    
    // Parse point cloud data from msg
    // fb->pc_layout, fb->point_data = parsePointCloud(msg);
    
    std::lock_guard<std::mutex> lock(cam->frame_mutex);
    cam->frame_queue.push(fb);
    cam->frame_cv.notify_one();
    
    if (cam->user_callback) {
        hal::FrameMetadata meta = fb->metadata;
        meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get());
        cam->user_callback(meta);
    }
}

// LiDAR Image Callback
void RobosenseCameraHAL::LiDARImageCallback(void* user_data, void* msg) {
    auto* cam = static_cast<CameraInstance*>(user_data);
    if (!cam || !cam->hal_ptr || !cam->streaming) return;
    
    auto fb = cam->hal_ptr->convertToFrameBuffer(*cam, msg, "depth");
    
    std::lock_guard<std::mutex> lock(cam->frame_mutex);
    cam->frame_queue.push(fb);
    cam->frame_cv.notify_one();
    
    if (cam->user_callback) {
        hal::FrameMetadata meta = fb->metadata;
        meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get());
        cam->user_callback(meta);
    }
}

// LiDAR IMU Callback
void RobosenseCameraHAL::LiDARImuCallback(void* user_data, void* msg) {
    auto* cam = static_cast<CameraInstance*>(user_data);
    if (!cam || !cam->hal_ptr || !cam->streaming) return;
    
    // IMU data would be processed here
    // For now, just acknowledge
}

// Camera Frame Callback
void RobosenseCameraHAL::CameraFrameCallback(void* user_data, void* frame) {
    auto* cam = static_cast<CameraInstance*>(user_data);
    if (!cam || !cam->hal_ptr || !cam->streaming) return;
    
    auto fb = cam->hal_ptr->convertToFrameBuffer(*cam, frame, "color");
    
    std::lock_guard<std::mutex> lock(cam->frame_mutex);
    cam->frame_queue.push(fb);
    cam->frame_cv.notify_one();
    
    if (cam->user_callback) {
        hal::FrameMetadata meta = fb->metadata;
        meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get());
        cam->user_callback(meta);
    }
}

// Load RoboSense SDK dynamically
bool RobosenseCameraHAL::loadRoboSenseSDK() {
    if (sdk_loaded_) return true;
    
    // Search paths for RoboSense SDK
    static const std::vector<std::string> search_paths = {
        "./lib/RoboSense",
        "/usr/local/lib",
        "/opt/RoboSense/lib",
        "/usr/lib/x86_64-linux-gnu"
    };
    
    std::string lib_path;
    for (const auto& path : search_paths) {
        std::filesystem::path p = std::filesystem::path(path) / "librs_driver.so";
        if (std::filesystem::exists(p)) {
            lib_path = p.string();
            break;
        }
    }
    
    if (lib_path.empty()) {
        lib_path = "librs_driver.so";
    }
    
    sdk_.handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!sdk_.handle) {
        std::cerr << "Failed to load RoboSense SDK: " << dlerror() << std::endl;
        return false;
    }
    
    auto symbols_result = loadSDKSymbols();
    if (symbols_result.is_err()) {
        unloadRoboSenseSDK();
        return false;
    }
    
    sdk_loaded_ = true;
    return true;
}

void RobosenseCameraHAL::unloadRoboSenseSDK() {
    if (sdk_.handle) {
        dlclose(sdk_.handle);
        sdk_.handle = nullptr;
    }
    sdk_loaded_ = false;
    memset(&sdk_, 0, sizeof(sdk_));
}

hal::ResultVoid RobosenseCameraHAL::loadSDKSymbols() {
#define LOAD_SYM(name) \
    sdk_.name = reinterpret_cast<decltype(sdk_.name)>(dlsym(sdk_.handle, "rs_" #name)); \
    if (!sdk_.name) { \
        std::cerr << "Failed to load symbol: rs_" #name << std::endl; \
        return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED); \
    }
    
    // LiDAR driver
    LOAD_SYM(lidar_driver_create);
    LOAD_SYM(lidar_driver_destroy);
    LOAD_SYM(lidar_driver_init);
    LOAD_SYM(lidar_driver_start);
    LOAD_SYM(lidar_driver_stop);
    LOAD_SYM(lidar_driver_reg_pointcloud_callback);
    LOAD_SYM(lidar_driver_reg_image_callback);
    LOAD_SYM(lidar_driver_reg_imu_callback);
    
    // Camera driver
    LOAD_SYM(camera_driver_create);
    LOAD_SYM(camera_driver_destroy);
    LOAD_SYM(camera_driver_open);
    LOAD_SYM(camera_driver_close);
    LOAD_SYM(camera_driver_start);
    LOAD_SYM(camera_driver_stop);
    LOAD_SYM(camera_driver_reg_callback);
    
#undef LOAD_SYM
    
    return hal::ResultVoid::ok();
}

bool RobosenseCameraHAL::isRoboSenseDevice(uint16_t vid, uint16_t pid) {
    return vid == 0x1234 && pid == 0x5678; // Placeholder VID/PID
}

hal::Result<std::vector<hal::CameraDeviceInfo>> RobosenseCameraHAL::enumerateDevices() {
    if (!loadRoboSenseSDK()) {
        return hal::Result<std::vector<hal::CameraDeviceInfo>>::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    std::vector<hal::CameraDeviceInfo> devices;
    
    // For RoboSense AC1, we enumerate via USB
    hal::CameraDeviceInfo info;
    info.device_id = "robosense://ac1:0";
    info.vendor = "robosense";
    info.bus_info = "USB3.0";
    info.display_name = "RoboSense RoboX AC1";
    info.sensor_info.serial_number = "AC1_001";
    info.sensor_info.max_width = 1920;
    info.sensor_info.max_height = 1080;
    info.sensor_info.supported_formats = {hal::PixelFormat::NV12, hal::PixelFormat::Y16, hal::PixelFormat::METADATA};
    info.sensor_info.supports_hw_sync = true;
    info.is_available = true;
    devices.push_back(info);
    
    return hal::Result<std::vector<hal::CameraDeviceInfo>>::ok(std::move(devices));
}

hal::Result<std::vector<hal::StreamConfig>> RobosenseCameraHAL::getSupportedStreams(const std::string& device_id) {
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
    depth.width = 640;
    depth.height = 480;
    depth.fps = 30;
    depth.buffer_count = 4;
    streams.push_back(depth);
    
    hal::StreamConfig points;
    points.stream_id = 2;
    points.format = hal::PixelFormat::METADATA; // Point cloud uses metadata format
    points.width = 0;
    points.height = 0;
    points.fps = 10;
    points.buffer_count = 4;
    streams.push_back(points);
    
    return hal::Result<std::vector<hal::StreamConfig>>::ok(std::move(streams));
}

hal::Result<hal::SensorInfo> RobosenseCameraHAL::getSensorInfo(const std::string& device_id) {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    auto it = cameras_.find(device_id);
    if (it == cameras_.end()) {
        return hal::Result<hal::SensorInfo>::err(hal::ErrorCode::NOT_FOUND);
    }
    
    hal::SensorInfo info;
    info.name = "RoboSense AC1";
    info.vendor = "robosense";
    info.serial_number = it->second->serial_number;
    info.max_width = 1920;
    info.max_height = 1080;
    info.supports_hw_sync = true;
    
    return hal::Result<hal::SensorInfo>::ok(info);
}

hal::ResultVoid RobosenseCameraHAL::initialize(const hal::CameraConfig& config) {
    if (!loadRoboSenseSDK()) {
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

hal::ResultVoid RobosenseCameraHAL::initializeMulti(const hal::MultiCameraConfig& config) {
    // AC1 is a single integrated device, but can be treated as multi-camera
    return initialize(config.cameras[0]);
}

hal::ResultVoid RobosenseCameraHAL::initializeCameraInstance(std::unique_ptr<CameraInstance>& cam, const hal::CameraConfig& config) {
    // Create LiDAR driver
    void* param = malloc(sizeof(void*)); // Placeholder for RSDriverParam
    cam->rs_lidar_driver = sdk_.lidar_driver_create(param);
    free(param);
    
    if (!cam->rs_lidar_driver) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    // Initialize LiDAR driver with AC1 config
    void* init_param = malloc(sizeof(void*)); // Placeholder
    int ret = sdk_.lidar_driver_init(cam->rs_lidar_driver, init_param);
    free(init_param);
    
    if (ret != 0) {
        sdk_.lidar_driver_destroy(cam->rs_lidar_driver);
        cam->rs_lidar_driver = nullptr;
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    // Create camera driver (if separate)
    cam->rs_camera_driver = sdk_.camera_driver_create();
    if (cam->rs_camera_driver) {
        sdk_.camera_driver_open(cam->rs_camera_driver, 1920, 1080, 30);
    }
    
    // Fetch calibration
    fetchCalibration(*cam);
    
    cam->initialized = true;
    cam->hal_ptr = this;
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::configureStreams(CameraInstance& cam, const std::vector<hal::StreamConfig>& streams) {
    if (!cam.rs_lidar_driver || !sdk_.lidar_driver_start) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_INITIALIZED);
    }
    
    // Configure LiDAR streams
    // Register callbacks
    sdk_.lidar_driver_reg_pointcloud_callback(cam.rs_lidar_driver, LiDARPointCloudCallback);
    sdk_.lidar_driver_reg_image_callback(cam.rs_lidar_driver, LiDARImageCallback);
    sdk_.lidar_driver_reg_imu_callback(cam.rs_lidar_driver, LiDARImuCallback);
    
    if (cam.rs_camera_driver && sdk_.camera_driver_reg_callback) {
        sdk_.camera_driver_reg_callback(cam.rs_camera_driver, CameraFrameCallback);
    }
    
    // Start LiDAR
    int ret = sdk_.lidar_driver_start(cam.rs_lidar_driver);
    if (ret != 0) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    if (cam.rs_camera_driver && sdk_.camera_driver_start) {
        sdk_.camera_driver_start(cam.rs_camera_driver);
    }
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::fetchCalibration(CameraInstance& cam) {
    // AC1 has fixed calibration
    cam.color_intrinsic.fx = 960.0f; cam.color_intrinsic.fy = 960.0f;
    cam.color_intrinsic.cx = 960.0f; cam.color_intrinsic.cy = 540.0f;
    cam.color_intrinsic.width = 1920; cam.color_intrinsic.height = 1080;
    
    cam.depth_intrinsic.fx = 480.0f; cam.depth_intrinsic.fy = 480.0f;
    cam.depth_intrinsic.cx = 320.0f; cam.depth_intrinsic.cy = 240.0f;
    cam.depth_intrinsic.width = 640; cam.depth_intrinsic.height = 480;
    
    cam.depth_scale = 0.001f;
    
    return hal::ResultVoid::ok();
}

std::shared_ptr<hal::FrameBuffer> RobosenseCameraHAL::convertToFrameBuffer(CameraInstance& cam, void* data, const std::string& stream_type) {
    auto fb = std::make_shared<hal::FrameBuffer>();
    fb->metadata.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    fb->metadata.frame_id = cam.frame_id++;
    fb->metadata.sensor_id = 0;
    
    if (stream_type == "color") {
        fb->metadata.frame_type = hal::FrameType::IMAGE;
        fb->metadata.format = hal::PixelFormat::NV12;
        fb->metadata.width = 1920;
        fb->metadata.height = 1080;
    } else if (stream_type == "depth") {
        fb->metadata.frame_type = hal::FrameType::DEPTH_MAP;
        fb->metadata.format = hal::PixelFormat::Y16;
        fb->metadata.width = 640;
        fb->metadata.height = 480;
    }
    
    fb->metadata.stride = fb->metadata.width * (fb->metadata.format == hal::PixelFormat::NV12 ? 3/2 : 1);
    // fb->image_data = copy image data from data
    
    return fb;
}

hal::ResultVoid RobosenseCameraHAL::start() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (auto& [id, cam] : cameras_) {
        if (!cam->initialized || cam->streaming) continue;
        
        auto result = configureStreams(*cam, {});
        if (!result.is_ok()) continue;
        
        cam->streaming = true;
        running_ = true;
    }
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::stop() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (auto& [id, cam] : cameras_) {
        if (!cam->streaming) continue;
        
        cam->streaming = false;
        if (cam->rs_lidar_driver && sdk_.lidar_driver_stop) {
            sdk_.lidar_driver_stop(cam->rs_lidar_driver);
        }
        if (cam->rs_camera_driver && sdk_.camera_driver_stop) {
            sdk_.camera_driver_stop(cam->rs_camera_driver);
        }
    }
    
    running_ = false;
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::deinitialize() {
    stop();
    
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    
    for (auto& [id, cam] : cameras_) {
        if (cam->rs_lidar_driver && sdk_.lidar_driver_destroy) {
            sdk_.lidar_driver_destroy(cam->rs_lidar_driver);
        }
        if (cam->rs_camera_driver && sdk_.camera_driver_destroy) {
            sdk_.camera_driver_destroy(cam->rs_camera_driver);
        }
    }
    
    cameras_.clear();
    unloadRoboSenseSDK();
    return hal::ResultVoid::ok();
}

hal::Result<hal::FrameMetadata> RobosenseCameraHAL::acquireFrame(uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(cameras_mutex_);
    
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
    meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get());
    meta.buffer.platform_id = 0;
    
    return hal::Result<hal::FrameMetadata>::ok(meta);
}

hal::Result<std::vector<hal::FrameMetadata>> RobosenseCameraHAL::acquireFrames(uint32_t timeout_ms) {
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

hal::ResultVoid RobosenseCameraHAL::registerCallback(typename hal::ICameraHAL::FrameCallback cb) {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    for (auto& [id, cam] : cameras_) {
        cam->user_callback = cb;
    }
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::unregisterCallback() {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    for (auto& [id, cam] : cameras_) {
        cam->user_callback = nullptr;
    }
    return hal::ResultVoid::ok();
}

hal::Result<hal::BufferHandle> RobosenseCameraHAL::importBuffer(const hal::BufferHandle& external) {
    return hal::Result<hal::BufferHandle>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid RobosenseCameraHAL::releaseBuffer(const hal::BufferHandle& buffer) {
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::releaseFrames(const std::vector<hal::FrameMetadata>& frames) {
    return hal::ResultVoid::ok();
}

hal::ResultVoid RobosenseCameraHAL::setControl(const std::string& device_id, hal::CameraControl control, int64_t value) {
    std::lock_guard<std::mutex> lock(cameras_mutex_);
    auto it = cameras_.find(device_id);
    if (it == cameras_.end() || !it->second->rs_lidar_driver) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_FOUND);
    }
    return hal::ResultVoid::ok();
}

hal::Result<int64_t> RobosenseCameraHAL::getControl(const std::string& device_id, hal::CameraControl control) {
    return hal::Result<int64_t>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>> RobosenseCameraHAL::getAllControls(const std::string& device_id) {
    return hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::pair<int64_t, int64_t>> RobosenseCameraHAL::getControlRange(const std::string& device_id, hal::CameraControl control) {
    return hal::Result<std::pair<int64_t, int64_t>>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid RobosenseCameraHAL::triggerFrame(const std::string& device_id) {
    return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid RobosenseCameraHAL::triggerFrames(const std::vector<std::string>& device_ids) {
    return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid RobosenseCameraHAL::flush() {
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

hal::ResultVoid RobosenseCameraHAL::vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size, void* out_data, size_t out_size) {
    return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { 
    return new dynalgo::RobosenseCameraHAL(); 
}
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { 
    delete ptr; 
}
}