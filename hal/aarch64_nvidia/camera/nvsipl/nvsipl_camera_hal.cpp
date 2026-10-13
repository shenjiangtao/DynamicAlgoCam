/*
 * Copyright (c) 2024, DynamicAlgoCam. All rights reserved.
 *
 * NvSIPL Camera HAL Implementation for NVIDIA Jetson/Drive platforms.
 * Based on NVIDIA CSiplCamera reference implementation (nvsipl-multicast-6.0.8.0).
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

// Logging macros
#define LOG_DBG(fmt, ...) fprintf(stderr, "[NVPL_DBG] " fmt "\n", ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) fprintf(stderr, "[NVPL_INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) fprintf(stderr, "[NVPL_WARN] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) fprintf(stderr, "[NVPL_ERR] " fmt "\n", ##__VA_ARGS__)

// Type aliases for real NvSIPL API
using NotificationType = nvsipl::NvSIPLPipelineNotifier::NotificationType;
using NotificationData = nvsipl::NvSIPLPipelineNotifier::NotificationData;
using INvSIPLFrameCompletionQueue = nvsipl::INvSIPLFrameCompletionQueue;
using INvSIPLNotificationQueue = nvsipl::INvSIPLNotificationQueue;
using INvSIPLClient = nvsipl::INvSIPLClient;
using INvSIPLCamera = nvsipl::INvSIPLCamera;
using INvSIPLBuffer = nvsipl::INvSIPLClient::INvSIPLBuffer;
using INvSIPLNvMBuffer = nvsipl::INvSIPLClient::INvSIPLNvMBuffer;
using NvSIPLDeviceBlockQueues = nvsipl::NvSIPLDeviceBlockQueues;
using NvSIPLPipelineConfiguration = nvsipl::NvSIPLPipelineConfiguration;
using NvSIPLPipelineQueues = nvsipl::NvSIPLPipelineQueues;
using ConsumerDesc = nvsipl::INvSIPLClient::ConsumerDesc;
using ImageMetaData = nvsipl::INvSIPLClient::ImageMetaData;

#else
// Mock types for compilation on non-NVIDIA platforms
namespace nvsipl {
    using SIPLStatus = int;
    constexpr int NVSIPL_STATUS_OK = 0;
    constexpr int NVSIPL_STATUS_BAD_ARGUMENT = -1;
    constexpr int NVSIPL_STATUS_ERROR = -2;
    constexpr int NVSIPL_STATUS_TIMED_OUT = -3;
    constexpr int NVSIPL_STATUS_EOF = -4;
    constexpr int NVSIPL_STATUS_OUT_OF_MEMORY = -5;
    
    using NvSiplCapInputFormatType = int;
    
    struct SensorInfo { 
        uint32_t id; 
        std::string name; 
        struct {
            NvSiplCapInputFormatType inputFormat = 0;
        } vcInfo;
    };
    struct CameraModuleInfo { SensorInfo sensorInfo; std::string name; };
    struct DeviceBlockInfo { std::string name; std::string cdi; uint32_t numCameraModules; CameraModuleInfo* cameraModuleInfoList; };
    struct PlatformCfg { uint32_t numDeviceBlocks; DeviceBlockInfo* deviceBlockList; };
    
    // Forward declarations
    class INvSIPLFrameCompletionQueue;
    class INvSIPLNotificationQueue;
    
    struct NvSIPLDeviceBlockQueues { std::vector<INvSIPLNotificationQueue*> notificationQueue; };
    struct NvSIPLPipelineConfiguration { bool captureOutputRequested; bool isp0OutputRequested; bool isp1OutputRequested; };
    struct NvSIPLPipelineQueues { 
        INvSIPLFrameCompletionQueue* captureCompletionQueue; 
        INvSIPLFrameCompletionQueue* isp0CompletionQueue; 
        INvSIPLFrameCompletionQueue* isp1CompletionQueue; 
        INvSIPLNotificationQueue* notificationQueue; 
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
        class INvSIPLBuffer {
        public:
            virtual void AddRef() = 0;
            virtual SIPLStatus Release() = 0;
        };
    };
    
    class INvSIPLFrameCompletionQueue {
    public:
        virtual SIPLStatus Get(INvSIPLClient::INvSIPLBuffer*& item, size_t timeoutUsec) = 0;
    };
    
    class INvSIPLNotificationQueue {
    public:
        virtual SIPLStatus Get(NotificationData& item, size_t timeoutUsec) = 0;
    };
    
    // Logging macros for mock
    #define LOG_DBG(fmt, ...) fprintf(stderr, "[NVPL_DBG] " fmt "\n", ##__VA_ARGS__)
    #define LOG_INFO(fmt, ...) fprintf(stderr, "[NVPL_INFO] " fmt "\n", ##__VA_ARGS__)
    #define LOG_WARN(fmt, ...) fprintf(stderr, "[NVPL_WARN] " fmt "\n", ##__VA_ARGS__)
    #define LOG_ERR(fmt, ...) fprintf(stderr, "[NVPL_ERR] " fmt "\n", ##__VA_ARGS__)
}
#endif

namespace dynalgo {

// Use nvsipl types
using nvsipl::SIPLStatus;
using nvsipl::NVSIPL_STATUS_OK;
using nvsipl::NVSIPL_STATUS_BAD_ARGUMENT;
using nvsipl::NVSIPL_STATUS_ERROR;
using nvsipl::NVSIPL_STATUS_TIMED_OUT;
using nvsipl::NVSIPL_STATUS_EOF;
using nvsipl::NVSIPL_STATUS_OUT_OF_MEMORY;

// Notification constants - use the real NvSIPL types when available
#ifdef DYNALGO_HAVE_NVSIPL
// In real NvSIPL, these are in nvsipl::NvSIPLPipelineMgr::NotificationType enum
// We'll use them directly in switch statements
#else
// Mock constants already defined in nvsipl namespace
using nvsipl::NOTIF_ERROR_ICP_CAPTURE_FAILURE;
using nvsipl::NOTIF_ERROR_ISP_PROCESSING_FAILURE;
using nvsipl::NOTIF_ERROR_INTERNAL_FAILURE;
using nvsipl::NOTIF_ERROR_ICP_BAD_INPUT_STREAM;
using nvsipl::NOTIF_INFO_ICP_PROCESSING_DONE;
using nvsipl::NOTIF_INFO_ISP_PROCESSING_DONE;
using nvsipl::NOTIF_INFO_ACP_PROCESSING_DONE;
using nvsipl::NOTIF_INFO_ICP_AUTH_SUCCESS;
using nvsipl::NOTIF_INFO_CDI_PROCESSING_DONE;
using nvsipl::NOTIF_WARN_ICP_FRAME_DROP;
using nvsipl::NOTIF_WARN_ICP_FRAME_DISCONTINUITY;
using nvsipl::NOTIF_WARN_ICP_CAPTURE_TIMEOUT;
using nvsipl::NOTIF_ERROR_ICP_EMB_DATA_PARSE_FAILURE;
using nvsipl::NOTIF_ERROR_ACP_PROCESSING_FAILURE;
using nvsipl::NOTIF_ERROR_CDI_SET_SENSOR_CTRL_FAILURE;
using nvsipl::NOTIF_ERROR_ICP_AUTH_FAILURE;
using nvsipl::NOTIF_ERROR_DESERIALIZER_FAILURE;
using nvsipl::NOTIF_ERROR_SERIALIZER_FAILURE;
using nvsipl::NOTIF_ERROR_SENSOR_FAILURE;
using nvsipl::NOTIF_ERROR_ISP_PROCESSING_FAILURE_RECOVERABLE;
#endif

// Pipeline frame queue handler
class PipelineFrameQueueHandler {
public:
    PipelineFrameQueueHandler(uint32_t sensor_id) : m_sensor_id(sensor_id) {}

    SIPLStatus Init(uint32_t sensor_id, 
                    const std::vector<std::pair<ConsumerDesc::OutputType, INvSIPLFrameCompletionQueue*>>& queues,
                    dynalgo::hal::ICameraHAL::FrameCallback callback) {
        m_sensor_id = sensor_id;
        m_queues = queues;
        m_callback = std::move(callback);
        m_quit = false;
        m_thread = std::thread(&PipelineFrameQueueHandler::ThreadFunc, this);
        return NVSIPL_STATUS_OK;
    }

    void Deinit() {
        m_quit = true;
        if (m_thread.joinable()) m_thread.join();
    }

    uint32_t GetSensorId() const { return m_sensor_id; }

private:
    void ThreadFunc() {
        pthread_setname_np(pthread_self(), "FrameQueue");

        std::vector<INvSIPLBuffer*> buffers(m_queues.size(), nullptr);
        hal::FrameMetadata frame;
        frame.sensor_id = m_sensor_id;

        while (!m_quit) {
            bool all_ok = true;
            for (size_t i = 0; i < m_queues.size(); ++i) {
                INvSIPLBuffer* buf = nullptr;
                SIPLStatus status = m_queues[i].second->Get(buf, 1000000); // 1s timeout
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
                    // Use INvSIPLNvMBuffer methods
                    auto* nvm_buf = dynamic_cast<INvSIPLNvMBuffer*>(nvbuf);
                    if (nvm_buf) {
                        // Get NvSciBufObj for the buffer
                        frame.buffer.handle = reinterpret_cast<uint64_t>(nvm_buf->GetNvSciBufImage());
                        frame.buffer.platform_id = 0x1; // NVIDIA platform ID
                        // Note: width/height should come from platform config, not image metadata
                        frame.width = 3840;
                        frame.height = 2160;
                    }
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
    std::vector<std::pair<ConsumerDesc::OutputType, INvSIPLFrameCompletionQueue*>> m_queues;
    dynalgo::hal::ICameraHAL::FrameCallback m_callback;
    std::thread m_thread;
    std::atomic<bool> m_quit{false};
    std::atomic<uint32_t> m_frame_counter{0};;
};

// Pipeline notification handler
class PipelineNotificationHandler {
public:
    PipelineNotificationHandler(uint32_t sensor_id) : m_sensor_id(sensor_id) {}
    
    SIPLStatus Init(uint32_t sensor_id, INvSIPLNotificationQueue* queue) {
        m_sensor_id = sensor_id;
        m_queue = queue;
        m_quit = false;
        m_upThread = std::make_unique<std::thread>(&PipelineNotificationHandler::ThreadFunc, this);
        return NVSIPL_STATUS_OK;
    }

    void Deinit() {
        m_quit = true;
        if (m_upThread && m_upThread->joinable()) m_upThread->join();
    }

    bool IsPipelineInError() const { return m_error; }

private:
    void ThreadFunc() {
        pthread_setname_np(pthread_self(), "PipelineEvent");
        NotificationData notif;
        
        while (!m_quit) {
            SIPLStatus status = m_queue->Get(notif, 1000000);
            if (status == NVSIPL_STATUS_OK) {
                HandleEvent(notif);
            } else if (status == NVSIPL_STATUS_EOF) {
                break;
            }
        }
    }

    void HandleEvent(const NotificationData& notif) {
        switch (notif.eNotifType) {
            case NotificationType::NOTIF_INFO_ICP_PROCESSING_DONE:
                LOG_INFO("Pipeline: %u, NOTIF_INFO_ICP_PROCESSING_DONE\n", notif.uIndex);
                break;
            case NotificationType::NOTIF_INFO_ISP_PROCESSING_DONE:
                LOG_INFO("Pipeline: %u, NOTIF_INFO_ISP_PROCESSING_DONE\n", notif.uIndex);
                break;
            case NotificationType::NOTIF_INFO_ACP_PROCESSING_DONE:
                LOG_INFO("Pipeline: %u, NOTIF_INFO_ACP_PROCESSING_DONE\n", notif.uIndex);
                break;
            case NotificationType::NOTIF_INFO_ICP_AUTH_SUCCESS:
                LOG_DBG("Pipeline: %u, ICP_AUTH_SUCCESS frame=%lu\n", notif.uIndex,
                        notif.frameSeqNumber);
                break;
            case NotificationType::NOTIF_INFO_CDI_PROCESSING_DONE:
                LOG_INFO("Pipeline: %u, NOTIF_INFO_CDI_PROCESSING_DONE\n", notif.uIndex);
                break;
            case NotificationType::NOTIF_WARN_ICP_FRAME_DROP:
                LOG_WARN("Pipeline: %u, NOTIF_WARN_ICP_FRAME_DROP\n", notif.uIndex);
                m_frame_drops++;
                break;
            case NotificationType::NOTIF_WARN_ICP_FRAME_DISCONTINUITY:
                LOG_WARN("Pipeline: %u, NOTIF_WARN_ICP_FRAME_DISCONTINUITY\n", notif.uIndex);
                break;
            case NotificationType::NOTIF_WARN_ICP_CAPTURE_TIMEOUT:
                LOG_WARN("Pipeline: %u, NOTIF_WARN_ICP_CAPTURE_TIMEOUT\n", notif.uIndex);
                break;
            case NotificationType::NOTIF_ERROR_ICP_BAD_INPUT_STREAM:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_ICP_BAD_INPUT_STREAM\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_ICP_CAPTURE_FAILURE:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_ICP_CAPTURE_FAILURE\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_ICP_EMB_DATA_PARSE_FAILURE:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_ICP_EMB_DATA_PARSE_FAILURE\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_ISP_PROCESSING_FAILURE:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_ISP_PROCESSING_FAILURE\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_ACP_PROCESSING_FAILURE:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_ACP_PROCESSING_FAILURE\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_CDI_SET_SENSOR_CTRL_FAILURE:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_CDI_SET_SENSOR_CTRL_FAILURE\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_INTERNAL_FAILURE:
                LOG_ERR("Pipeline: %u, NOTIF_ERROR_INTERNAL_FAILURE\n", notif.uIndex);
                m_error = true;
                break;
            case NotificationType::NOTIF_ERROR_ICP_AUTH_FAILURE:
                LOG_ERR("Pipeline: %u, ICP_AUTH_FAILURE frame=%lu\n", notif.uIndex,
                        notif.frameSeqNumber);
                break;
            default:
                LOG_WARN("Pipeline: %u, Unknown/Invalid notification\n", notif.uIndex);
                break;
        }
    }

    uint32_t m_sensor_id = 0;
    uint32_t m_frame_drops = 0;
    bool m_error = false;
    INvSIPLNotificationQueue* m_queue = nullptr;
    std::unique_ptr<std::thread> m_upThread = nullptr;
    std::atomic<bool> m_quit{false};
};

// Device block notification handler (simplified)
class DeviceBlockNotificationHandler {
public:
    DeviceBlockNotificationHandler(uint32_t index, const void* db_info) 
        : m_index(index), m_device_block_info(db_info) {}
    
    SIPLStatus Init(const void* db_info, void* queue, void* camera, bool ignore_error) {
        m_device_block_info = db_info;
        m_queue = queue;
        m_camera = camera;
        m_ignore_error = ignore_error;
        m_quit = false;
        m_error = false;
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
        
        while (!m_quit) {
            // In real implementation, use NvSIPL notification queue
            // For now, just sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    uint32_t m_index = 0;
    const void* m_device_block_info = nullptr;
    void* m_queue = nullptr;
    void* m_camera = nullptr;
    bool m_ignore_error = false;
    std::thread m_thread;
    std::atomic<bool> m_quit{false};
    std::atomic<bool> m_error{false};
};

// Platform configuration parser
class PlatformConfigParser {
public:
    static SIPLStatus ParsePlatformConfig(const std::string& config_file, 
                                          void* platform_cfg) {
        // For now, create a default platform config programmatically
        // In production, this would parse the YAML PlatformCfg file
        return NVSIPL_STATUS_OK;
    }
    
    static void FreePlatformConfig(void* cfg) {
        // Free platform config memory
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
        
        // In real implementation, use NvSIPL Query to enumerate devices
        // For now, return default devices
        dynalgo::hal::CameraDeviceInfo info;
        info.device_id = "nvsipl://0";
        info.vendor = "nvidia";
        info.bus_info = "MIPI CSI-2";
        info.display_name = "IMX728 Front Camera";
        info.sensor_info.name = "IMX728";
        info.sensor_info.vendor = "nvidia";
        info.sensor_info.serial_number = "NVSIPL_0";
        info.sensor_info.max_width = 3840;
        info.sensor_info.max_height = 2160;
        info.sensor_info.supported_formats = {dynalgo::hal::PixelFormat::RAW12, dynalgo::hal::PixelFormat::NV12};
        info.sensor_info.supported_resolutions = {{3840, 2160}, {1920, 1080}};
        info.sensor_info.supported_fps = {30, 60};
        info.sensor_info.supports_hdr = true;
        info.sensor_info.supports_hw_sync = true;
        info.sensor_info.pixel_size_um = 2.74f;
        info.sensor_info.color_filter = "RGGB";
        info.sensor_info.lens_mount = "M12";
        info.is_available = true;
        devices.push_back(std::move(info));
        
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
        
        // Initialize NvSIPL Camera
#ifdef DYNALGO_HAVE_NVSIPL
        m_camera = nvsipl::INvSIPLCamera::GetInstance();
        if (!m_camera) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }

        // Create default platform config (in real implementation, parse from file)
        m_platform_cfg.platform = "dynalgo";
        m_platform_cfg.platformConfig = "default";
        m_platform_cfg.description = "DynamicAlgoCam default config";
        m_platform_cfg.numDeviceBlocks = 0;
        
        // Set platform config - returns device block queues
        SIPLStatus status = m_camera->SetPlatformCfg(&m_platform_cfg, m_device_block_queues);
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }

        // Setup pipelines for each camera module
        // In real implementation, iterate through platform config and set up pipelines
        
        // Initialize camera
        status = m_camera->Init();
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }
#endif
        
        m_initialized = true;
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid initializeMulti(const dynalgo::hal::MultiCameraConfig& config) override {
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
        
#ifdef DYNALGO_HAVE_NVSIPL
        SIPLStatus status = m_camera->Start();
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }
#endif
        
        m_running = true;
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid stop() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_running) return dynalgo::hal::ResultVoid::ok();
        
#ifdef DYNALGO_HAVE_NVSIPL
        SIPLStatus status = m_camera->Stop();
        if (status != NVSIPL_STATUS_OK) {
            return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::DEVICE_ERROR);
        }
#endif
        
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
        
#ifdef DYNALGO_HAVE_NVSIPL
        if (m_camera) {
            m_camera->Deinit();
            m_camera.reset();
        }
#endif
        
        // Clear frame queue
        std::queue<dynalgo::hal::FrameMetadata> empty;
        std::swap(m_frame_queue, empty);
        
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
        frames.reserve(m_config.buffer_count);
        
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
        dynalgo::hal::BufferHandle handle;
        handle.handle = external.handle;
        handle.platform_id = external.platform_id;
        return dynalgo::hal::Result<dynalgo::hal::BufferHandle>::ok(handle);
    }

    dynalgo::hal::ResultVoid releaseBuffer(const dynalgo::hal::BufferHandle& buffer) override {
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid releaseFrames(const std::vector<dynalgo::hal::FrameMetadata>& frames) override {
        for (const auto& frame : frames) {
            releaseBuffer(frame.buffer);
        }
        return dynalgo::hal::ResultVoid::ok();
    }

    dynalgo::hal::ResultVoid setControl(const std::string& device_id, dynalgo::hal::CameraControl control, int64_t value) override {
        (void)device_id; (void)control; (void)value;
        // Sensor control is handled via auto control plugins in NvSIPL, not directly via camera API
        return dynalgo::hal::ResultVoid::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
    }

    dynalgo::hal::Result<int64_t> getControl(const std::string& device_id, dynalgo::hal::CameraControl control) override {
        (void)device_id; (void)control;
        return dynalgo::hal::Result<int64_t>::err(dynalgo::hal::ErrorCode::NOT_SUPPORTED);
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
    
#ifdef DYNALGO_HAVE_NVSIPL
    nvsipl::PlatformCfg m_platform_cfg;
    std::unique_ptr<nvsipl::INvSIPLCamera> m_camera;
    nvsipl::NvSIPLDeviceBlockQueues m_device_block_queues;
#else
    void* m_platform_cfg = nullptr;
    void* m_camera = nullptr;
    void* m_device_block_queues = nullptr;
#endif
    
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
} // extern "C"

} // namespace dynalgo