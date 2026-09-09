/*
 * Copyright (c) 2024, DynamicAlgoCam. All rights reserved.
 *
 * NvSIPL Camera HAL Implementation for NVIDIA Jetson/Drive platforms.
 * Based on NVIDIA CSiplCamera reference implementation.
 */

#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/hal_types.hpp>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <fstream>
#include <iostream>
#include <queue>

// NvSIPL headers - only available on NVIDIA platforms
#ifdef DYNALGO_HAVE_NVSIPL
#include "NvSIPLVersion.hpp"
#include "NvSIPLCommon.hpp"
#include "NvSIPLCamera.hpp"
#include "NvSIPLPipelineMgr.hpp"
#include "NvSIPLClient.hpp"
#include "NvSIPLQuery.hpp"

// NvSciBuf/Sync/Stream
#include "nvscibuf.h"
#include "nvscisync.h"
#include "nvscistream.h"
#else
// Forward declarations for mock types
#endif

// Mock types for compilation on non-NVIDIA platforms (always available)
#ifndef DYNALGO_HAVE_NVSIPL
namespace nvsipl {
    using SIPLStatus = int;
    constexpr int NVSIPL_STATUS_OK = 0;
    constexpr int NVSIPL_STATUS_BAD_ARGUMENT = -1;
    constexpr int NVSIPL_STATUS_ERROR = -2;
    constexpr int NVSIPL_STATUS_TIMED_OUT = -3;
    constexpr int NVSIPL_STATUS_EOF = -4;
    constexpr int NVSIPL_STATUS_OUT_OF_MEMORY = -5;
    
    enum class SurfaceFormat { SF_RAW12 };
    struct SensorInfo { 
        uint32_t id; 
        std::string name; 
        struct {
            nvsipl::SurfaceFormat inputFormat = nvsipl::SurfaceFormat::SF_RAW12;
        } vcInfo;
    };
    struct CameraModuleInfo { SensorInfo sensorInfo; std::string name; };
    struct DeviceBlockInfo { std::string name; std::string cdi; uint32_t numCameraModules; CameraModuleInfo* cameraModuleInfoList; };
    struct PlatformCfg { uint32_t numDeviceBlocks; DeviceBlockInfo* deviceBlockList; };
    
    // Forward declarations
    class INvSIPLFrameCompletionQueue;
    class INvSIPLNotificationQueue;
    
    struct NvSIPLDeviceBlockQueues { std::vector<nvsipl::INvSIPLNotificationQueue*> notificationQueue; };
    struct NvSIPLPipelineConfiguration { bool captureOutputRequested; bool isp0OutputRequested; bool isp1OutputRequested; };
    struct NvSIPLPipelineQueues { 
        nvsipl::INvSIPLFrameCompletionQueue* captureCompletionQueue; 
        nvsipl::INvSIPLFrameCompletionQueue* isp0CompletionQueue; 
        nvsipl::INvSIPLFrameCompletionQueue* isp1CompletionQueue; 
        nvsipl::INvSIPLNotificationQueue* notificationQueue; 
    };
    struct NotificationData { int eNotifType; };
    constexpr int NOTIF_ERROR_ICP_CAPTURE_FAILURE = 1;
    constexpr int NOTIF_ERROR_ISP_PROCESSING_FAILURE = 2;
    constexpr int NOTIF_ERROR_INTERNAL_FAILURE = 3;
    constexpr int NOTIF_ERROR_ICP_BAD_INPUT_STREAM = 4;
    
    enum class ConsumerDesc { ICP, ISP0, ISP1, ISP2 };
    
    enum class SensorControl { EXPOSURE_TIME, GAIN, FRAME_RATE };
    
    class INvSIPLCamera {
    public:
        static std::unique_ptr<INvSIPLCamera> GetInstance() { return nullptr; }
        virtual SIPLStatus SetPlatformCfg(void*, NvSIPLDeviceBlockQueues&) { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus SetPipelineCfg(uint32_t, NvSIPLPipelineConfiguration&, NvSIPLPipelineQueues&) { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus Init() { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus Deinit() { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus Start() { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus Stop() { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus SetSensorControl(uint32_t, nvsipl::SensorControl, int64_t) { return NVSIPL_STATUS_OK; }
        virtual SIPLStatus GetSensorControl(uint32_t, nvsipl::SensorControl, int64_t&) { return NVSIPL_STATUS_OK; }
    };
    
    class INvSIPLClient {
    public:
        class ConsumerDesc {
        public:
            enum class OutputType { ICP, ISP0, ISP1, ISP2 };
        };
        class INvSIPLBuffer {
        public:
            virtual void Release() {}
            virtual void* GetNvSciBufImage() { return nullptr; }
            struct ImageAttributes {
                uint32_t width = 3840;
                uint32_t height = 2160;
            };
            virtual ImageAttributes GetImageAttributes() { return ImageAttributes{}; }
        };
        class INvSIPLFrameCompletionQueue {
        public:
            virtual SIPLStatus Get(INvSIPLBuffer**, uint64_t) { return NVSIPL_STATUS_OK; }
        };
        class INvSIPLNotificationQueue {
        public:
            virtual SIPLStatus Get(NotificationData&, uint64_t) { return NVSIPL_STATUS_OK; }
        };
    };
    
    class INvSIPLFrameCompletionQueue {
    public:
        virtual SIPLStatus Get(INvSIPLClient::INvSIPLBuffer**, uint64_t) { return NVSIPL_STATUS_OK; }
    };
    
    class INvSIPLNotificationQueue {
    public:
        virtual SIPLStatus Get(NotificationData&, uint64_t) { return NVSIPL_STATUS_OK; }
    };
}
#endif

namespace dynalgo {

using nvsipl::SIPLStatus;
using nvsipl::NVSIPL_STATUS_OK;
using nvsipl::NVSIPL_STATUS_BAD_ARGUMENT;
using nvsipl::NVSIPL_STATUS_ERROR;
using nvsipl::NVSIPL_STATUS_TIMED_OUT;
using nvsipl::NVSIPL_STATUS_EOF;
using nvsipl::NVSIPL_STATUS_OUT_OF_MEMORY;
using nvsipl::NOTIF_ERROR_ICP_CAPTURE_FAILURE;
using nvsipl::NOTIF_ERROR_ISP_PROCESSING_FAILURE;
using nvsipl::NOTIF_ERROR_INTERNAL_FAILURE;
using nvsipl::NOTIF_ERROR_ICP_BAD_INPUT_STREAM;

// Forward declarations for internal handler classes
class PipelineFrameQueueHandler;
class PipelineNotificationHandler;
class DeviceBlockNotificationHandler;

// Internal frame queue handler - wraps NvSIPL frame completion queue
class PipelineFrameQueueHandler {
public:
    using FrameCallback = std::function<void(const hal::FrameMetadata&)>;

    PipelineFrameQueueHandler(uint32_t sensor_id, FrameCallback cb)
        : m_sensor_id(sensor_id), m_callback(std::move(cb)) {}

    ~PipelineFrameQueueHandler() { Deinit(); }

    SIPLStatus Init(uint32_t sensor_id,
                    const std::vector<std::pair<nvsipl::INvSIPLClient::ConsumerDesc::OutputType,
                                                 nvsipl::INvSIPLFrameCompletionQueue*>>& queues) {
        m_sensor_id = sensor_id;
        m_queues = queues;
        m_quit = false;
        m_thread = std::thread(&PipelineFrameQueueHandler::ThreadFunc, this);
        return NVSIPL_STATUS_OK;
    }

    void Deinit() {
        m_quit = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }

    uint32_t GetSensorId() const { return m_sensor_id; }

private:
    void ThreadFunc() {
        pthread_setname_np(pthread_self(), "FrameQueue");

        std::vector<nvsipl::INvSIPLClient::INvSIPLBuffer*> buffers(m_queues.size(), nullptr);
        hal::FrameMetadata frame;
        frame.sensor_id = m_sensor_id;

        while (!m_quit) {
            bool all_ok = true;
            for (size_t i = 0; i < m_queues.size(); ++i) {
                nvsipl::INvSIPLClient::INvSIPLBuffer* buf = nullptr;
                SIPLStatus status = m_queues[i].second->Get(&buf, 1000000); // 1s timeout
                if (status == NVSIPL_STATUS_OK) {
                    buffers[i] = buf;
                } else if (status == NVSIPL_STATUS_TIMED_OUT) {
                    all_ok = false;
                    break;
                } else {
                    all_ok = false;
                    break;
                }
            }

            if (all_ok && m_callback) {
                // Convert buffers to FrameMetadata
                frame.timestamp_ns = dynalgo::hal::now_ns();
                frame.frame_id = m_frame_counter++;
                frame.format = hal::PixelFormat::NV12; // Will be updated per buffer
                
                // For simplicity, use first buffer as primary
                if (!buffers.empty() && buffers[0]) {
                    auto* nvbuf = buffers[0];
                    auto img_attrs = nvbuf->GetImageAttributes();
                    frame.width = img_attrs.width;
                    frame.height = img_attrs.height;
                    frame.format = hal::PixelFormat::NV12;
                    
                    // Wrap NvSciBufObj in BufferHandle
                    frame.buffer.handle = reinterpret_cast<uint64_t>(nvbuf->GetNvSciBufImage());
                    frame.buffer.platform_id = 0x1; // NVIDIA platform ID
                }
                
                m_callback(frame);
                
                // Release buffers
                for (auto* buf : buffers) {
                    if (buf) {
                        buf->Release();
                    }
                }
            }
        }
    }

    uint32_t m_sensor_id = 0;
    std::vector<std::pair<nvsipl::INvSIPLClient::ConsumerDesc::OutputType, nvsipl::INvSIPLFrameCompletionQueue*>> m_queues;
    FrameCallback m_callback;
    std::thread m_thread;
    std::atomic<bool> m_quit{false};
    std::atomic<uint32_t> m_frame_counter{0};
};

// Pipeline notification handler
class PipelineNotificationHandler {
public:
    PipelineNotificationHandler(uint32_t sensor_id) : m_sensor_id(sensor_id) {}
    
    SIPLStatus Init(uint32_t sensor_id, nvsipl::INvSIPLNotificationQueue* queue) {
        m_sensor_id = sensor_id;
        m_queue = queue;
        m_quit = false;
        m_thread = std::thread(&PipelineNotificationHandler::ThreadFunc, this);
        return NVSIPL_STATUS_OK;
    }

    void Deinit() {
        m_quit = true;
        if (m_thread.joinable()) m_thread.join();
    }

    bool IsPipelineInError() const { return m_error; }

private:
    void ThreadFunc() {
        pthread_setname_np(pthread_self(), "PipelineEvent");
        nvsipl::NotificationData notif;
        
        while (!m_quit) {
            SIPLStatus status = m_queue->Get(notif, 1000000);
            if (status == NVSIPL_STATUS_OK) {
                HandleEvent(notif);
            } else if (status == NVSIPL_STATUS_EOF) {
                break;
            }
        }
    }

    void HandleEvent(const nvsipl::NotificationData& notif) {
        switch (notif.eNotifType) {
            case nvsipl::NOTIF_ERROR_ICP_CAPTURE_FAILURE:
            case nvsipl::NOTIF_ERROR_ISP_PROCESSING_FAILURE:
            case nvsipl::NOTIF_ERROR_INTERNAL_FAILURE:
                m_error = true;
                break;
            default:
                break;
        }
    }

    uint32_t m_sensor_id = 0;
    nvsipl::INvSIPLNotificationQueue* m_queue = nullptr;
    std::thread m_thread;
    std::atomic<bool> m_quit{false};
    std::atomic<bool> m_error{false};
};

// Device block notification handler
class DeviceBlockNotificationHandler {
public:
    DeviceBlockNotificationHandler(uint32_t index) : m_index(index) {}
    
    SIPLStatus Init(uint32_t index, const nvsipl::DeviceBlockInfo* info,
                    nvsipl::INvSIPLNotificationQueue* queue,
                    nvsipl::INvSIPLCamera* camera) {
        m_index = index;
        m_info = info;
        m_queue = queue;
        m_camera = camera;
        m_quit = false;
        m_thread = std::thread(&DeviceBlockNotificationHandler::ThreadFunc, this);
        return NVSIPL_STATUS_OK;
    }

    void Deinit() {
        m_quit = true;
        if (m_thread.joinable()) m_thread.join();
    }

    bool IsDeviceBlockInError() const { return m_error; }

private:
    void ThreadFunc() {
        pthread_setname_np(pthread_self(), "DevBlkEvent");
        nvsipl::NotificationData notif;
        
        while (!m_quit) {
            SIPLStatus status = m_queue->Get(notif, 1000000);
            if (status == NVSIPL_STATUS_OK) {
                // Handle device block events
            } else if (status == NVSIPL_STATUS_EOF) {
                break;
            }
        }
    }

    uint32_t m_index = 0;
    const nvsipl::DeviceBlockInfo* m_info = nullptr;
    nvsipl::INvSIPLNotificationQueue* m_queue = nullptr;
    nvsipl::INvSIPLCamera* m_camera = nullptr;
    std::thread m_thread;
    std::atomic<bool> m_quit{false};
    std::atomic<bool> m_error{false};
};

// Platform configuration parser
class PlatformConfigParser {
public:
    static SIPLStatus ParsePlatformConfig(const std::string& config_file, 
                                          nvsipl::PlatformCfg* platform_cfg) {
        // For now, create a default platform config programmatically
        // In production, this would parse the YAML PlatformCfg file
        platform_cfg->numDeviceBlocks = 1;
        platform_cfg->deviceBlockList = new nvsipl::DeviceBlockInfo[1];
        
        auto& db = platform_cfg->deviceBlockList[0];
        db.name = "deser0";
        db.cdi = "max96712";
        db.numCameraModules = 2;
        db.cameraModuleInfoList = new nvsipl::CameraModuleInfo[2];
        
        // Camera 0 - IMX728 front
        auto& cam0 = db.cameraModuleInfoList[0];
        cam0.sensorInfo.id = 0;
        cam0.sensorInfo.name = "imx728";
        cam0.sensorInfo.vcInfo.inputFormat = nvsipl::SurfaceFormat::SF_RAW12;
        cam0.name = "imx728_front";
        
        // Camera 1 - IMX623 rear
        auto& cam1 = db.cameraModuleInfoList[1];
        cam1.sensorInfo.id = 1;
        cam1.sensorInfo.name = "imx623";
        cam1.sensorInfo.vcInfo.inputFormat = nvsipl::SurfaceFormat::SF_RAW12;
        cam1.name = "imx623_rear";
        
        return NVSIPL_STATUS_OK;
    }
    
    static void FreePlatformConfig(nvsipl::PlatformCfg* cfg) {
        if (cfg && cfg->deviceBlockList) {
            for (uint32_t i = 0; i < cfg->numDeviceBlocks; ++i) {
                delete[] cfg->deviceBlockList[i].cameraModuleInfoList;
            }
            delete[] cfg->deviceBlockList;
        }
    }
};

// NvSIPL Camera HAL Implementation
class NvSIPLCameraHAL : public dynalgo::hal::ICameraHAL {
public:
    NvSIPLCameraHAL() = default;
    ~NvSIPLCameraHAL() override { deinitialize(); }

    // ICameraHAL interface
    dynalgo::hal::Result<std::vector<dynalgo::hal::CameraDeviceInfo>> enumerateDevices() override {
        std::vector<dynalgo::hal::CameraDeviceInfo> devices;
        
        if (m_platform_cfg && m_platform_cfg->numDeviceBlocks > 0) {
            for (uint32_t d = 0; d < m_platform_cfg->numDeviceBlocks; ++d) {
                const auto& db = m_platform_cfg->deviceBlockList[d];
                for (uint32_t m = 0; m < db.numCameraModules; ++m) {
                    const auto& cam = db.cameraModuleInfoList[m];
                    dynalgo::hal::CameraDeviceInfo info;
                    info.device_id = "nvsipl://" + std::to_string(cam.sensorInfo.id);
                    info.display_name = cam.name;
                    info.vendor = "nvidia";
                    info.bus_info = "MIPI CSI-2";
                    info.is_available = true;
                    info.sensor_info.name = cam.sensorInfo.name;
                    info.sensor_info.vendor = "nvidia";
                    info.sensor_info.max_width = 3840;
                    info.sensor_info.max_height = 2160;
                    info.sensor_info.supported_formats = {dynalgo::hal::PixelFormat::RAW12, dynalgo::hal::PixelFormat::NV12};
                    info.sensor_info.supported_resolutions = {{3840, 2160}, {1920, 1080}};
                    info.sensor_info.supported_fps = {30, 60};
                    info.sensor_info.supports_hdr = true;
                    info.sensor_info.supports_hw_sync = true;
                    info.sensor_info.pixel_size_um = 2.74f;
                    devices.push_back(std::move(info));
                }
            }
        }
        
        return dynalgo::hal::Result<std::vector<dynalgo::hal::CameraDeviceInfo>>::ok(std::move(devices));
    }

    dynalgo::hal::Result<std::vector<dynalgo::hal::StreamConfig>> getSupportedStreams(const std::string& device_id) override {
        std::vector<dynalgo::hal::StreamConfig> streams;
        
        dynalgo::hal::StreamConfig s1;
        s1.stream_id = 0;
        s1.format = dynalgo::hal::PixelFormat::NV12;
        s1.width = 3840;
        s1.height = 2160;
        s1.fps = 30;
        s1.buffer_count = 4;
        streams.push_back(s1);
        
        dynalgo::hal::StreamConfig s2;
        s2.stream_id = 1;
        s2.format = dynalgo::hal::PixelFormat::NV12;
        s2.width = 1920;
        s2.height = 1080;
        s2.fps = 60;
        s2.buffer_count = 4;
        streams.push_back(s2);
        
        return dynalgo::hal::Result<std::vector<dynalgo::hal::StreamConfig>>::ok(std::move(streams));
    }

    dynalgo::hal::Result<dynalgo::hal::SensorInfo> getSensorInfo(const std::string& device_id) override {
        dynalgo::hal::SensorInfo info;
        info.name = "IMX728";
        info.vendor = "nvidia";
        info.serial_number = "nvsipl_0";
        info.max_width = 3840;
        info.max_height = 2160;
        info.supported_formats = {dynalgo::hal::PixelFormat::RAW12, dynalgo::hal::PixelFormat::NV12};
        info.supported_resolutions = {{3840, 2160}, {1920, 1080}};
        info.supported_fps = {30, 60};
        info.supports_hdr = true;
        info.supports_hw_sync = true;
        info.pixel_size_um = 2.74f;
        info.color_filter = "RGGB";
        info.lens_mount = "M12";
        return dynalgo::hal::Result<dynalgo::hal::SensorInfo>::ok(std::move(info));
    }

    dynalgo::hal::ResultVoid initialize(const dynalgo::hal::CameraConfig& config) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        if (m_initialized) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::ALREADY_INITIALIZED);
        }

        m_config = config;
        
        // Parse platform config from file
        if (!config.device_id.empty() && config.device_id.find("nvsipl://") == 0) {
            // Extract config file path from device_id or use default
            std::string config_file = "/opt/dynalgo/config/platform.yaml";
            SIPLStatus status = PlatformConfigParser::ParsePlatformConfig(config_file, m_platform_cfg);
            if (status != NVSIPL_STATUS_OK) {
                return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
            }
        } else {
            // Create default config
            SIPLStatus status = PlatformConfigParser::ParsePlatformConfig("", m_platform_cfg);
            if (status != NVSIPL_STATUS_OK) {
                return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
            }
        }

        // Initialize NvSIPL Camera
        m_camera = nvsipl::INvSIPLCamera::GetInstance();
        if (!m_camera) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }

        // Set platform config
        nvsipl::NvSIPLDeviceBlockQueues device_block_queues;
        SIPLStatus status = m_camera->SetPlatformCfg(m_platform_cfg, device_block_queues);
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }

        // Setup pipelines for each camera module
        for (uint32_t d = 0; d < m_platform_cfg->numDeviceBlocks; ++d) {
            const auto& db = m_platform_cfg->deviceBlockList[d];
            for (uint32_t m = 0; m < db.numCameraModules; ++m) {
                const auto& module = db.cameraModuleInfoList[m];
                uint32_t sensor_id = module.sensorInfo.id;
                
                nvsipl::NvSIPLPipelineConfiguration pipeline_cfg{};
                nvsipl::NvSIPLPipelineQueues pipeline_queues;
                
                // Configure pipeline
                pipeline_cfg.captureOutputRequested = true;
                pipeline_cfg.isp0OutputRequested = true;
                pipeline_cfg.isp1OutputRequested = true;
                
                status = m_camera->SetPipelineCfg(sensor_id, pipeline_cfg, pipeline_queues);
                if (status != NVSIPL_STATUS_OK) {
                    return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
                }

                // Create frame queue handler
                auto frame_handler = std::make_unique<PipelineFrameQueueHandler>(sensor_id, 
                    [this](const dynalgo::hal::FrameMetadata& frame) {
                        std::lock_guard<std::mutex> lock(m_frame_mutex);
                        m_frame_queue.push(frame);
                        m_frame_cv.notify_one();
                    });
                
                std::vector<std::pair<nvsipl::INvSIPLClient::ConsumerDesc::OutputType, 
                                      nvsipl::INvSIPLFrameCompletionQueue*>> queues;
                queues.push_back(std::make_pair(nvsipl::INvSIPLClient::ConsumerDesc::OutputType::ICP, 
                                  pipeline_queues.captureCompletionQueue));
                queues.push_back(std::make_pair(nvsipl::INvSIPLClient::ConsumerDesc::OutputType::ISP0, 
                                  pipeline_queues.isp0CompletionQueue));
                queues.push_back(std::make_pair(nvsipl::INvSIPLClient::ConsumerDesc::OutputType::ISP1, 
                                  pipeline_queues.isp1CompletionQueue));
                
                frame_handler->Init(sensor_id, queues);
                m_frame_handlers[sensor_id] = std::move(frame_handler);

                // Create notification handler
                auto notif_handler = std::make_unique<PipelineNotificationHandler>(sensor_id);
                notif_handler->Init(sensor_id, pipeline_queues.notificationQueue);
                m_notif_handlers[sensor_id] = std::move(notif_handler);
            }
        }

        // Initialize camera
        status = m_camera->Init();
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }

        // Create device block handlers
        nvsipl::NvSIPLDeviceBlockQueues db_queues;
        m_camera->SetPlatformCfg(m_platform_cfg, db_queues); // Get queues again
        
        for (uint32_t d = 0; d < m_platform_cfg->numDeviceBlocks; ++d) {
            auto db_handler = std::make_unique<DeviceBlockNotificationHandler>(d);
            db_handler->Init(d, &m_platform_cfg->deviceBlockList[d],
                            db_queues.notificationQueue[d], m_camera.get());
            m_db_handlers.push_back(std::move(db_handler));
        }

        m_initialized = true;
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid initializeMulti(const dynalgo::hal::MultiCameraConfig& config) override {
        // For now, initialize each camera sequentially
        for (const auto& cam_config : config.cameras) {
            auto result = initialize(cam_config);
            if (!result.is_ok()) return result;
        }
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid start() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_INITIALIZED);
        if (m_running) return dynalgo::hal::ResultVoid::ok();
        
        SIPLStatus status = m_camera->Start();
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }
        m_running = true;
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid stop() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_running) return dynalgo::hal::ResultVoid::ok();
        
        SIPLStatus status = m_camera->Stop();
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }
        m_running = false;
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid deinitialize() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) return dynalgo::hal::ResultVoid::ok();
        
        if (m_running) {
            stop();
        }
        
        // Deinit handlers
        for (auto& [id, handler] : m_frame_handlers) {
            handler->Deinit();
        }
        m_frame_handlers.clear();
        
        for (auto& [id, handler] : m_notif_handlers) {
            handler->Deinit();
        }
        m_notif_handlers.clear();
        
        for (auto& handler : m_db_handlers) {
            handler->Deinit();
        }
        m_db_handlers.clear();
        
        if (m_camera) {
            m_camera->Deinit();
            m_camera.reset();
        }
        
        // Clear frame queue
        std::queue<dynalgo::hal::FrameMetadata> empty;
        std::swap(m_frame_queue, empty);
        
        if (m_platform_cfg) {
            PlatformConfigParser::FreePlatformConfig(m_platform_cfg);
            m_platform_cfg = nullptr;
        }
        
        m_initialized = false;
        m_running = false;
        
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::Result<dynalgo::hal::FrameMetadata> acquireFrame(uint32_t timeout_ms = 1000) override {
        std::unique_lock<std::mutex> lock(m_frame_mutex);
        if (m_frame_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                [this] { return !m_frame_queue.empty(); })) {
            auto frame = std::move(m_frame_queue.front());
            m_frame_queue.pop();
            return dynalgo::hal::Result<dynalgo::hal::FrameMetadata>::ok(std::move(frame));
        }
        return dynalgo::hal::Result<dynalgo::hal::FrameMetadata>::err(dynalgo::hal::ErrorCode::TIMEOUT);
    }

    dynalgo::hal::Result<std::vector<dynalgo::hal::FrameMetadata>> acquireFrames(uint32_t timeout_ms = 1000) override {
        std::unique_lock<std::mutex> lock(m_frame_mutex);
        std::vector<dynalgo::hal::FrameMetadata> frames;
        
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        
        while (frames.size() < m_config.buffer_count) {
            auto remaining = deadline - std::chrono::steady_clock::now();
            if (remaining.count() <= 0) break;
            
            if (m_frame_cv.wait_for(lock, remaining, [this] { return !m_frame_queue.empty(); })) {
                auto frame = std::move(m_frame_queue.front());
                m_frame_queue.pop();
                frames.push_back(std::move(frame));
            } else {
                break;
            }
        }
        
        if (frames.empty()) {
            return dynalgo::hal::Result<std::vector<dynalgo::hal::FrameMetadata>>::err(dynalgo::hal::ErrorCode::TIMEOUT);
        }
        
        return dynalgo::hal::Result<std::vector<dynalgo::hal::FrameMetadata>>::ok(std::move(frames));
    }

    dynalgo::hal::ResultVoid registerCallback(dynalgo::hal::ICameraHAL::FrameCallback cb) override {
        m_user_callback = std::move(cb);
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid unregisterCallback() override {
        m_user_callback = nullptr;
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::Result<dynalgo::hal::BufferHandle> importBuffer(const dynalgo::hal::BufferHandle& external) override {
        // Import external buffer as NvSciBufObj
        dynalgo::hal::BufferHandle handle;
        handle.handle = external.handle;
        handle.platform_id = external.platform_id;
        return dynalgo::hal::Result<dynalgo::hal::BufferHandle>::ok(handle);
    }

    dynalgo::hal::ResultVoid releaseBuffer(const dynalgo::hal::BufferHandle& buffer) override {
        // Buffer release handled by NvSIPL frame completion
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid releaseFrames(const std::vector<dynalgo::hal::FrameMetadata>& frames) override {
        for (const auto& frame : frames) {
            releaseBuffer(frame.buffer);
        }
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid setControl(const std::string& device_id, dynalgo::hal::CameraControl control, int64_t value) override {
        if (!m_camera) return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_INITIALIZED);
        
        // Map HAL controls to NvSIPL controls
        uint32_t sensor_id = std::stoi(device_id.substr(device_id.find_last_of('/') + 1));
        
        switch (control) {
            case dynalgo::hal::CameraControl::EXPOSURE_TIME:
                return m_camera->SetSensorControl(sensor_id, nvsipl::SensorControl::EXPOSURE_TIME, value);
            case dynalgo::hal::CameraControl::GAIN:
                return m_camera->SetSensorControl(sensor_id, nvsipl::SensorControl::GAIN, value);
            case dynalgo::hal::CameraControl::FRAME_RATE:
                return m_camera->SetSensorControl(sensor_id, nvsipl::SensorControl::FRAME_RATE, value);
            default:
                return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
        }
    }

    dynalgo::hal::Result<int64_t> getControl(const std::string& device_id, dynalgo::hal::CameraControl control) override {
        if (!m_camera) return dynalgo::hal::Result<int64_t>::err(dynalgo::hal::ErrorCode::NOT_INITIALIZED);
        
        uint32_t sensor_id = std::stoi(device_id.substr(device_id.find_last_of('/') + 1));
        int64_t value = 0;
        
        switch (control) {
            case dynalgo::hal::CameraControl::EXPOSURE_TIME:
                m_camera->GetSensorControl(sensor_id, nvsipl::SensorControl::EXPOSURE_TIME, value);
                return dynalgo::hal::Result<int64_t>::ok(value);
            case dynalgo::hal::CameraControl::GAIN:
                m_camera->GetSensorControl(sensor_id, nvsipl::SensorControl::GAIN, value);
                return dynalgo::hal::Result<int64_t>::ok(value);
            default:
                return dynalgo::hal::Result<int64_t>::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
        }
    }

    dynalgo::hal::Result<std::vector<std::pair<dynalgo::hal::CameraControl, int64_t>>> getAllControls(const std::string& device_id) override {
        return dynalgo::hal::Result<std::vector<std::pair<dynalgo::hal::CameraControl, int64_t>>>::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
    }

    dynalgo::hal::Result<std::pair<int64_t, int64_t>> getControlRange(const std::string& device_id, dynalgo::hal::CameraControl control) override {
        return dynalgo::hal::Result<std::pair<int64_t, int64_t>>::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
    }

    dynalgo::hal::ResultVoid triggerFrame(const std::string& device_id) override {
        return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
    }

    dynalgo::hal::ResultVoid triggerFrames(const std::vector<std::string>& device_ids) override {
        return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
    }

    dynalgo::hal::ResultVoid flush() override {
        std::lock_guard<std::mutex> lock(m_frame_mutex);
        std::queue<dynalgo::hal::FrameMetadata> empty;
        std::swap(m_frame_queue, empty);
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::Result<std::string> getName() const override {
        return dynalgo::hal::Result<std::string>::ok("NvSIPL Camera HAL");
    }

    dynalgo::hal::Result<std::string> getVendor() const override {
        return dynalgo::hal::Result<std::string>::ok("nvidia");
    }

    dynalgo::hal::Result<std::string> getVersion() const override {
        return dynalgo::hal::Result<std::string>::ok("6.0.8");
    }

    dynalgo::hal::ResultVoid vendorCommand(uint32_t cmd_id, const void* in_data, size_t in_size,
                                           void* out_data, size_t out_size) override {
        return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
    }

    bool isRunning() const override { return m_running; }
    uint32_t getDroppedFrameCount() const override { return 0; }
    void resetDroppedFrameCount() override {}

private:
    std::mutex m_mutex;
    dynalgo::hal::CameraConfig m_config;
    
    nvsipl::PlatformCfg* m_platform_cfg = nullptr;
    std::unique_ptr<nvsipl::INvSIPLCamera> m_camera;
    
    std::unordered_map<uint32_t, std::unique_ptr<PipelineFrameQueueHandler>> m_frame_handlers;
    std::unordered_map<uint32_t, std::unique_ptr<PipelineNotificationHandler>> m_notif_handlers;
    std::vector<std::unique_ptr<DeviceBlockNotificationHandler>> m_db_handlers;
    
    // Frame queue for blocking acquire
    std::mutex m_frame_mutex;
    std::condition_variable m_frame_cv;
    std::queue<dynalgo::hal::FrameMetadata> m_frame_queue;
    dynalgo::hal::ICameraHAL::FrameCallback m_user_callback;
    
    bool m_initialized = false;
    bool m_running = false;
};

// Factory registration
extern "C" {
dynalgo::hal::ICameraHAL* dynalgo_hal_camera_create() {
    return new dynalgo::NvSIPLCameraHAL();
}

void dynalgo_hal_camera_destroy(dynalgo::hal::ICameraHAL* ptr) {
    delete ptr;
}
}

} // namespace dynalgo