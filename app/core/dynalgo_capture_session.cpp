/*
 * dynalgo_capture_session.cpp - HAL-based capture session implementation
 */
#include "dynalgo_capture_session.hpp"
#include "dynalgo_hal_factory.hpp"
#include "dynalgo_log.hpp"
#include "dynalgo_device.hpp"
#include "dynalgo_config_manager.hpp"

#include <filesystem>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <functional>

namespace dynalgo {

using ::std::shared_ptr;
using ::std::unique_ptr;
using ::std::string;
using ::std::move;
using ::std::lock_guard;
using ::std::mutex;
using ::std::function;
using ::std::make_shared;
using ::std::chrono::steady_clock;
using ::std::chrono::nanoseconds;
using ::std::chrono::duration_cast;

HALCaptureSession::HALCaptureSession(HALBundle&& hal_bundle, const Config& config)
    : m_hal(std::move(hal_bundle)), m_config(config) {
    m_output_dir = config.device_output_dir;
}

std::shared_ptr<HALCaptureSession> HALCaptureSession::create(DynalgoDevice* device, DynalgoPipeline* pipeline,
                                                             const std::string& name, const std::string& output_dir,
                                                             int width, int height, int fps) {
    // Convert to HALCaptureSession::Config
    Config session_config;
    session_config.device_output_dir = output_dir;
    session_config.width = width;
    session_config.height = height;
    session_config.fps = fps;
    session_config.buffer_count = 4;
    session_config.enable_encoder = false;
    session_config.enable_inference = false;
    session_config.enable_display = false;
    session_config.enable_fusion = false;
    session_config.fusion_alpha = 0.5f;
    session_config.depth_min_m = 0.1f;
    session_config.depth_max_m = 10.0f;
    session_config.file_prefix = "recording";
    
    // Create HAL bundle using factory
    auto hal_bundle_result = HALFactory::createFromConfig({}, {}, {}); // TODO: pass proper configs
    if (!hal_bundle_result.is_ok()) {
        return nullptr;
    }
    
    auto session = std::make_shared<HALCaptureSession>(std::move(hal_bundle_result.value()), session_config);
    // TODO: initialize device/pipeline specific setup
    return session;
}

HALCaptureSession::~HALCaptureSession() {
    stop();
}

bool HALCaptureSession::setup() {
    if (!m_hal.camera) {
        DYNALGO_LOG_ERROR("Camera HAL not available");
        return false;
    }
    
    // Initialize camera with config
    hal::CameraConfig cam_config;
    cam_config.device_id = "nvsipl://0"; // TODO: get from config
    cam_config.width = m_config.width;
    cam_config.height = m_config.height;
    cam_config.fps = m_config.fps;
    cam_config.format = hal::PixelFormat::NV12;
    cam_config.buffer_count = m_config.buffer_count;
    cam_config.hardware_sync = true;
    
    auto result = m_hal.camera->initialize(cam_config);
    if (!result.is_ok()) {
        DYNALGO_LOG_ERROR_S("Camera initialize failed: " << result.error());
        return false;
    }
    
    // Initialize encoder if enabled
    if (m_config.enable_encoder && m_hal.encoder) {
        hal::EncodeConfig enc_config;
        enc_config.width = m_config.width;
        enc_config.height = m_config.height;
        enc_config.fps = m_config.fps;
        enc_config.bitrate_bps = 8000000;
        enc_config.gop_size = 30;
        enc_config.codec = hal::EncoderCodec::H264_HIGH;
        enc_config.rate_control = hal::EncoderRateControl::CBR;
        enc_config.input_format = hal::PixelFormat::NV12;
        
        auto enc_result = m_hal.encoder->initialize(enc_config);
        if (!enc_result.is_ok()) {
            DYNALGO_LOG_WARN_S("Encoder initialize failed: " << enc_result.error());
        }
    }
    
    // Initialize inference if enabled
    if (m_config.enable_inference && m_hal.inference) {
        hal::ModelConfig inf_config;
        inf_config.model_path = "model.trt"; // TODO: from config
        inf_config.precision = hal::InferencePrecision::FP16;
        inf_config.device = hal::InferenceDevice::AUTO;
        inf_config.max_batch_size = 1;
        inf_config.workspace_size_mb = 512;
        
        auto inf_result = m_hal.inference->initialize(inf_config);
        if (!inf_result.is_ok()) {
            DYNALGO_LOG_WARN_S("Inference initialize failed: " << inf_result.error());
        }
    }
    
    // Initialize display if enabled
    if (m_config.enable_display && m_hal.display) {
        hal::DisplayConfig disp_config;
        disp_config.width = m_config.width;
        disp_config.height = m_config.height;
        disp_config.mode = hal::DisplayMode::WINDOWED;
        disp_config.vsync = true;
        
        auto disp_result = m_hal.display->initialize(disp_config);
        if (!disp_result.is_ok()) {
            DYNALGO_LOG_WARN_S("Display initialize failed: " << disp_result.error());
        }
    }
    
    // Start camera streaming
    auto start_result = m_hal.camera->start();
    if (!start_result.is_ok()) {
        DYNALGO_LOG_ERROR_S("Camera start failed: " << start_result.error());
        return false;
    }
    
    // Start encoder if available
    if (m_config.enable_encoder && m_hal.encoder) {
        m_hal.encoder->start();
    }
    
    // Start display if available
    if (m_config.enable_display && m_hal.display) {
        m_hal.display->show();
    }
    
    return true;
}

void HALCaptureSession::stop() {
    // Stop video pipeline
    if (m_hal.camera) {
        m_hal.camera->stop();
    }
    
    // Stop encoder
    if (m_config.enable_encoder && m_hal.encoder) {
        m_hal.encoder->stop();
    }
    
    // Stop inference
    if (m_config.enable_inference && m_hal.inference) {
        m_hal.inference->deinitialize();
    }
    
    // Stop display
    if (m_config.enable_display && m_hal.display) {
        m_hal.display->hide();
    }
    
    // Join threads
    // TODO: implement proper thread joining
}

void HALCaptureSession::addFrameConsumer(FrameConsumer consumer) {
    lock_guard<mutex> lock(m_frame_mutex);
    m_frame_consumers.push_back(move(consumer));
}

void HALCaptureSession::setEngagementLoop(void* engage_loop, void* model_backend, void* actuator) {
    // TODO: Store engagement loop pointers for frame processing
}

void HALCaptureSession::reportFps(uint64_t duration_ms) {
    // TODO: Implement FPS reporting
}

bool HALCaptureSession::startImuPipeline() {
    // TODO: Implement IMU pipeline start
    return false;
}

bool HALCaptureSession::startVideoPipeline(bool show_preview) {
    // TODO: Implement video pipeline start
    (void)show_preview;
    return false;
}

} // namespace dynalgo