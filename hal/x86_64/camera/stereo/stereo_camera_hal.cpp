/*
 * stereo_camera_hal.cpp - Generic UVC Stereo Camera HAL implementation for x86_64
 * Bridges UvcStereoCamera driver to ICameraHAL interface
 * Supports: Dual UVC cameras, hardware sync, stereo rectification, depth computation
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
#include <map>

#include "../../../../app/driver/stereo/stereo_adapter.hpp"

namespace dynalgo {

class StereoCameraHAL : public hal::ICameraHAL {
public:
    StereoCameraHAL() : running_(false), dropped_frames_(0) {}
    ~StereoCameraHAL() override { deinitialize(); }

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
    hal::Result<std::string> getName() const override { return hal::Result<std::string>::ok("Stereo Camera HAL"); }
    hal::Result<std::string> getVendor() const override { return hal::Result<std::string>::ok("stereo"); }
    hal::Result<std::string> getVersion() const override { return hal::Result<std::string>::ok("1.0.0"); }
    hal::ResultVoid vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size, void* out_data, size_t out_size) override;
    bool isRunning() const override { return running_; }
    uint32_t getDroppedFrameCount() const override { return dropped_frames_; }
    void resetDroppedFrameCount() override { dropped_frames_ = 0; }

private:
    struct StreamInfo {
        std::string type;              // "left", "right", "depth", "disparity"
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t fps = 30;
        hal::PixelFormat format = hal::PixelFormat::NV12;
        bool hw_d2c = false;
        bool enabled = true;
    };

    struct StereoPair {
        std::string pair_id;
        std::string left_device;
        std::string right_device;
        std::unique_ptr<IStereoCamera> driver;
        std::string calibration_file;
        bool compute_depth = true;
        
        // Calibration
        hal::CameraIntrinsic left_intrinsic;
        hal::CameraIntrinsic right_intrinsic;
        hal::Transform3D left_to_right;
        float baseline_meters = 0.12f;
        
        // Active streams
        std::map<std::string, StreamInfo> streams;
        
        // Runtime
        bool initialized = false;
        bool streaming = false;
        std::mutex frame_mutex;
        std::condition_variable frame_cv;
        std::queue<std::shared_ptr<hal::FrameBuffer>> frame_queue;
        std::function<void(const hal::FrameMetadata&)> user_callback;
        uint32_t frame_id = 0;
    };

    // Helpers
    hal::ResultVoid initializeStereoPair(std::unique_ptr<StereoPair>& pair, const hal::MultiCameraConfig& config);
    hal::ResultVoid configureStreams(StereoPair& pair, const std::vector<hal::StreamConfig>& streams);
    hal::ResultVoid loadCalibration(StereoPair& pair);
    
    std::shared_ptr<hal::FrameBuffer> convertStereoFrameSet(const StereoPair& pair, const StereoFrameSet& frame_set);
    
    // State
    std::map<std::string, std::unique_ptr<StereoPair>> pairs_;
    std::mutex pairs_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> dropped_frames_{0};
};

// ICameraHAL implementation

hal::Result<std::vector<hal::CameraDeviceInfo>> StereoCameraHAL::enumerateDevices() {
    std::vector<hal::CameraDeviceInfo> devices;
    
    // Scan for UVC stereo pairs
    // This is a simplified implementation - real impl would scan /dev/video* for pairs
    hal::CameraDeviceInfo info;
    info.device_id = "stereo://uvc:0";
    info.vendor = "stereo";
    info.bus_info = "USB3.0";
    info.display_name = "Generic UVC Stereo Pair";
    info.sensor_info.serial_number = "uvc_stereo_0";
    info.sensor_info.max_width = 1280;
    info.sensor_info.max_height = 720;
    info.sensor_info.supported_formats = {hal::PixelFormat::NV12, hal::PixelFormat::YUYV, hal::PixelFormat::Y16, hal::PixelFormat::METADATA};
    info.sensor_info.supports_hw_sync = true;
    info.is_available = true;
    devices.push_back(info);
    
    return hal::Result<std::vector<hal::CameraDeviceInfo>>::ok(std::move(devices));
}

hal::Result<std::vector<hal::StreamConfig>> StereoCameraHAL::getSupportedStreams(const std::string& device_id) {
    std::vector<hal::StreamConfig> streams;
    
    hal::StreamConfig left;
    left.stream_id = 0;
    left.format = hal::PixelFormat::NV12;
    left.width = 1280;
    left.height = 720;
    left.fps = 30;
    left.buffer_count = 4;
    streams.push_back(left);
    
    hal::StreamConfig right;
    right.stream_id = 1;
    right.format = hal::PixelFormat::NV12;
    right.width = 1280;
    right.height = 720;
    right.fps = 30;
    right.buffer_count = 4;
    streams.push_back(right);
    
    hal::StreamConfig depth;
    depth.stream_id = 2;
    depth.format = hal::PixelFormat::Y16;
    depth.width = 1280;
    depth.height = 720;
    depth.fps = 30;
    depth.buffer_count = 4;
    streams.push_back(depth);
    
    hal::StreamConfig disparity;
    disparity.stream_id = 3;
    disparity.format = hal::PixelFormat::Y16;
    disparity.width = 1280;
    disparity.height = 720;
    disparity.fps = 30;
    disparity.buffer_count = 4;
    streams.push_back(disparity);
    
    return hal::Result<std::vector<hal::StreamConfig>>::ok(std::move(streams));
}

hal::Result<hal::SensorInfo> StereoCameraHAL::getSensorInfo(const std::string& device_id) {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    auto it = pairs_.find(device_id);
    if (it == pairs_.end()) {
        return hal::Result<hal::SensorInfo>::err(hal::ErrorCode::NOT_FOUND);
    }
    
    hal::SensorInfo info;
    info.name = "Stereo Pair";
    info.vendor = "stereo";
    info.serial_number = it->second->pair_id;
    info.max_width = 1280;
    info.max_height = 720;
    info.supports_hw_sync = true;
    
    return hal::Result<hal::SensorInfo>::ok(info);
}

hal::ResultVoid StereoCameraHAL::initialize(const hal::CameraConfig& config) {
    // Single camera initialize not typically used for stereo
    // Use initializeMulti for stereo pairs
    return hal::ResultVoid::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid StereoCameraHAL::initializeMulti(const hal::MultiCameraConfig& config) {
    if (config.cameras.size() != 2) {
        return hal::ResultVoid::err(hal::ErrorCode::INVALID_ARG);
    }
    
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    
    std::string pair_id = config.cameras[0].device_id + "_stereo";
    if (pairs_.find(pair_id) != pairs_.end()) {
        return hal::ResultVoid::err(hal::ErrorCode::ALREADY_INITIALIZED);
    }
    
    auto pair = std::make_unique<StereoPair>();
    pair->pair_id = pair_id;
    pair->left_device = config.cameras[0].device_id;
    pair->right_device = config.cameras[1].device_id;
    
    // Load calibration if provided
    if (!config.cameras[0].vendor_params || !config.cameras[1].vendor_params) {
        // Could extract calibration file from vendor_params
    }
    
    auto result = initializeStereoPair(pair, config);
    if (!result.is_ok()) {
        return result;
    }
    
    pairs_[pair_id] = std::move(pair);
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::initializeStereoPair(std::unique_ptr<StereoPair>& pair, const hal::MultiCameraConfig& config) {
    // Create stereo driver
    pair->driver = StereoCameraFactory::create(StereoCameraFactory::Vendor::GENERIC_UVC);
    if (!pair->driver) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    // Configure stereo
    StereoConfig stereo_cfg;
    stereo_cfg.devicePath = pair->left_device; // Base path, driver will append _left/_right
    stereo_cfg.width = config.cameras[0].width > 0 ? config.cameras[0].width : 1280;
    stereo_cfg.height = config.cameras[0].height > 0 ? config.cameras[0].height : 720;
    stereo_cfg.fps = config.cameras[0].fps > 0 ? config.cameras[0].fps : 30;
    stereo_cfg.hardwareSync = config.synchronized;
    stereo_cfg.computeDepth = true;
    
    // Load calibration if provided in vendor_params
    // This would need to be extracted from camera configs
    
    if (!pair->driver->open(stereo_cfg)) {
        return hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
    }
    
    // Get calibration info
    const auto& info = pair->driver->getInfo();
    pair->left_intrinsic = {info.leftIntrinsic.fx, info.leftIntrinsic.fy, info.leftIntrinsic.cx, info.leftIntrinsic.cy,
                            static_cast<uint32_t>(info.leftIntrinsic.width), static_cast<uint32_t>(info.leftIntrinsic.height)};
    pair->right_intrinsic = {info.rightIntrinsic.fx, info.rightIntrinsic.fy, info.rightIntrinsic.cx, info.rightIntrinsic.cy,
                             static_cast<uint32_t>(info.rightIntrinsic.width), static_cast<uint32_t>(info.rightIntrinsic.height)};
    for (int i = 0; i < 9; ++i) {
        pair->left_to_right.rotation[i] = info.leftToRight.r[i];
    }
    for (int i = 0; i < 3; ++i) {
        pair->left_to_right.translation[i] = info.leftToRight.t[i];
    }
    pair->baseline_meters = info.baselineMeters;
    
    // Set up streams
    pair->streams["left"] = {"left", stereo_cfg.width, stereo_cfg.height, stereo_cfg.fps, hal::PixelFormat::NV12, false, true};
    pair->streams["right"] = {"right", stereo_cfg.width, stereo_cfg.height, stereo_cfg.fps, hal::PixelFormat::NV12, false, true};
    pair->streams["depth"] = {"depth", stereo_cfg.width, stereo_cfg.height, stereo_cfg.fps, hal::PixelFormat::Y16, false, true};
    pair->streams["disparity"] = {"disparity", stereo_cfg.width, stereo_cfg.height, stereo_cfg.fps, hal::PixelFormat::Y16, false, true};
    
    pair->initialized = true;
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::configureStreams(StereoPair& pair, const std::vector<hal::StreamConfig>& streams) {
    if (!pair.initialized || !pair.driver) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_INITIALIZED);
    }
    
    // Start streaming with callback
    pair.streaming = true;
    auto& stereo_pair_ref = pair;
    pair.driver->startStreaming([this, &stereo_pair_ref](const StereoFrameSet& frame_set) {
        if (!stereo_pair_ref.streaming) return;
        
        auto fb = convertStereoFrameSet(stereo_pair_ref, frame_set);
        
        std::lock_guard<std::mutex> lock(stereo_pair_ref.frame_mutex);
        stereo_pair_ref.frame_queue.push(fb);
        stereo_pair_ref.frame_cv.notify_one();
        
        if (stereo_pair_ref.user_callback) {
            hal::FrameMetadata meta = fb->metadata;
            meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get());
            stereo_pair_ref.user_callback(meta);
        }
    });
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::loadCalibration(StereoPair& pair) {
    if (!pair.calibration_file.empty() && pair.driver) {
        // Load calibration from file
        // This would use the driver's rectifier
    }
    return hal::ResultVoid::ok();
}

std::shared_ptr<hal::FrameBuffer> StereoCameraHAL::convertStereoFrameSet(const StereoPair& pair, const StereoFrameSet& frame_set) {
    auto fb = std::make_shared<hal::FrameBuffer>();
    fb->metadata.timestamp_ns = frame_set.timestampUs * 1000;
    fb->metadata.frame_id = frame_set.frameId;
    fb->metadata.sensor_id = 0;
    fb->metadata.frame_type = hal::FrameType::FUSED; // Stereo pair as fused
    fb->metadata.format = hal::PixelFormat::METADATA;
    fb->metadata.width = frame_set.leftRect.width;
    fb->metadata.height = frame_set.leftRect.height;
    
    // Store stereo data in vendor_data or as fused frame
    // This is a simplified implementation
    
    return fb;
}

hal::ResultVoid StereoCameraHAL::start() {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    
    for (auto& [id, pair] : pairs_) {
        if (!pair->initialized || pair->streaming) continue;
        
        auto result = configureStreams(*pair, {});
        if (!result.is_ok()) continue;
        
        pair->streaming = true;
        running_ = true;
    }
    
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::stop() {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    
    for (auto& [id, pair] : pairs_) {
        if (!pair->streaming) continue;
        
        pair->streaming = false;
        if (pair->driver) {
            pair->driver->stopStreaming();
        }
    }
    
    running_ = false;
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::deinitialize() {
    stop();
    
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    
    for (auto& [id, pair] : pairs_) {
        if (pair->driver) {
            pair->driver->close();
        }
    }
    
    pairs_.clear();
    return hal::ResultVoid::ok();
}

hal::Result<hal::FrameMetadata> StereoCameraHAL::acquireFrame(uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(pairs_mutex_);
    
    if (pairs_.empty()) {
        return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::NOT_INITIALIZED);
    }
    
    auto& pair = pairs_.begin()->second;
    lock.unlock();
    
    std::unique_lock<std::mutex> frame_lock(pair->frame_mutex);
    if (pair->frame_queue.empty()) {
        if (!pair->frame_cv.wait_for(frame_lock, std::chrono::milliseconds(timeout_ms),
            [&pair] { return !pair->frame_queue.empty(); })) {
            return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::TIMEOUT);
        }
    }
    
    if (pair->frame_queue.empty()) {
        return hal::Result<hal::FrameMetadata>::err(hal::ErrorCode::TIMEOUT);
    }
    
    auto fb = pair->frame_queue.front();
    pair->frame_queue.pop();
    
    hal::FrameMetadata meta = fb->metadata;
    meta.buffer.handle = reinterpret_cast<uint64_t>(fb.get());
    meta.buffer.platform_id = 0;
    
    return hal::Result<hal::FrameMetadata>::ok(meta);
}

hal::Result<std::vector<hal::FrameMetadata>> StereoCameraHAL::acquireFrames(uint32_t timeout_ms) {
    std::vector<hal::FrameMetadata> frames;
    frames.reserve(pairs_.size());
    
    for (auto& [id, pair] : pairs_) {
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

hal::ResultVoid StereoCameraHAL::registerCallback(typename hal::ICameraHAL::FrameCallback cb) {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    for (auto& [id, stereo_pair] : pairs_) {
        stereo_pair->user_callback = cb;
    }
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::unregisterCallback() {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    for (auto& [id, stereo_pair] : pairs_) {
        stereo_pair->user_callback = nullptr;
    }
    return hal::ResultVoid::ok();
}

hal::Result<hal::BufferHandle> StereoCameraHAL::importBuffer(const hal::BufferHandle& external) {
    return hal::Result<hal::BufferHandle>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid StereoCameraHAL::releaseBuffer(const hal::BufferHandle& buffer) {
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::releaseFrames(const std::vector<hal::FrameMetadata>& frames) {
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::setControl(const std::string& device_id, hal::CameraControl control, int64_t value) {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    auto it = pairs_.find(device_id);
    if (it == pairs_.end() || !it->second->driver) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_FOUND);
    }
    
    if (control == hal::CameraControl::EXPOSURE_TIME) {
        it->second->driver->setExposure(value / 1000.0f);
    } else if (control == hal::CameraControl::GAIN) {
        it->second->driver->setGain(value / 100.0f);
    }
    return hal::ResultVoid::ok();
}

hal::Result<int64_t> StereoCameraHAL::getControl(const std::string& device_id, hal::CameraControl control) {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    auto it = pairs_.find(device_id);
    if (it == pairs_.end() || !it->second->driver) {
        return hal::Result<int64_t>::err(hal::ErrorCode::NOT_FOUND);
    }
    
    if (control == hal::CameraControl::EXPOSURE_TIME) {
        return hal::Result<int64_t>::ok(static_cast<int64_t>(it->second->driver->getExposure() * 1000));
    } else if (control == hal::CameraControl::GAIN) {
        return hal::Result<int64_t>::ok(static_cast<int64_t>(it->second->driver->getGain() * 100));
    }
    return hal::Result<int64_t>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>> StereoCameraHAL::getAllControls(const std::string& device_id) {
    return hal::Result<std::vector<std::pair<hal::CameraControl, int64_t>>>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::Result<std::pair<int64_t, int64_t>> StereoCameraHAL::getControlRange(const std::string& device_id, hal::CameraControl control) {
    return hal::Result<std::pair<int64_t, int64_t>>::err(hal::ErrorCode::NOT_SUPPORTED);
}

hal::ResultVoid StereoCameraHAL::triggerFrame(const std::string& device_id) {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    auto it = pairs_.find(device_id);
    if (it == pairs_.end() || !it->second->driver) {
        return hal::ResultVoid::err(hal::ErrorCode::NOT_FOUND);
    }
    return it->second->driver->triggerCapture() ? hal::ResultVoid::ok() : hal::ResultVoid::err(hal::ErrorCode::DEVICE_ERROR);
}

hal::ResultVoid StereoCameraHAL::triggerFrames(const std::vector<std::string>& device_ids) {
    for (const auto& id : device_ids) {
        auto result = triggerFrame(id);
        if (!result.is_ok()) return result;
    }
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::flush() {
    std::lock_guard<std::mutex> lock(pairs_mutex_);
    for (auto& [id, pair] : pairs_) {
        std::lock_guard<std::mutex> frame_lock(pair->frame_mutex);
        while (!pair->frame_queue.empty()) {
            pair->frame_queue.pop();
        }
    }
    dropped_frames_ = 0;
    return hal::ResultVoid::ok();
}

hal::ResultVoid StereoCameraHAL::vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size, void* out_data, size_t out_size) {
    return hal::ResultVoid::err(hal::ErrorCode::NOT_IMPLEMENTED);
}

} // namespace dynalgo

extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() { 
    return new dynalgo::StereoCameraHAL(); 
}
void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) { 
    delete ptr; 
}
}