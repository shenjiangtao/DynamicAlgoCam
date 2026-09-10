/*
 * dynalgo_capture_session.hpp - HAL-based capture session
 */
#pragma once

#include <dynalgo/hal/camera_hal.hpp>
#include <dynalgo/hal/encoder_hal.hpp>
#include <dynalgo/hal/inference_hal.hpp>
#include <dynalgo/hal/display_hal.hpp>
#include <dynalgo/hal/hal_types.hpp>
#include <dynalgo/hal/logger_hal.hpp>
#include <dynalgo_hal_factory.hpp>
#include "dynalgo_device.hpp"
#include "dynalgo_types.hpp"

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

#include <functional>

namespace dynalgo {

class HALCaptureSession {
public:
    struct Config {
        std::string device_output_dir;
        int width = 3840;
        int height = 2160;
        int fps = 30;
        int buffer_count = 4;
        bool enable_encoder = false;
        bool enable_inference = false;
        bool enable_display = false;
        bool enable_fusion = false;
        float fusion_alpha = 0.5f;
        float depth_min_m = 0.1f;
        float depth_max_m = 10.0f;
        std::string file_prefix = "recording";
    };

    using FrameConsumer = ::std::function<void(const hal::FrameMetadata&)>;

    HALCaptureSession(HALBundle&& hal_bundle, const Config& config);
    ~HALCaptureSession();

    // Setup and teardown
    bool setup();
    void stop();
    
    // Pipeline control
    bool startVideoPipeline(bool show_preview = true);
    bool startImuPipeline();
    
    bool hasVideoPipeline() const { return m_video_running; }
    
    // Frame consumers for engagement loop, etc.
    void addFrameConsumer(FrameConsumer consumer);
    
    // Engagement loop integration
    void setEngagementLoop(void* engage_loop, void* model_backend, void* actuator);
    
    // Getters
    const DynalgoIntrinsic& depthIntrinsic() const { return m_depth_intrinsic; }
    float depthScale() const { return m_depth_scale; }
    
    // Stats
    void reportFps(uint64_t duration_ms);
    
    // File prefix for event-based naming
    void setFilePrefix(const std::string& prefix) { m_file_prefix = prefix; }

    // Factory method
    static std::shared_ptr<HALCaptureSession> create(DynalgoDevice* device, DynalgoPipeline* pipeline,
                                                     const std::string& name, const std::string& output_dir,
                                                     int width = 3840, int height = 2160, int fps = 30);

private:
    // Pipeline threads
    void videoCaptureLoop();
    void processFrame(const hal::FrameMetadata& frame);
    
    // Encoder thread
    void encoderLoop();
    
    // Inference thread
    void inferenceLoop();
    
    // Display thread
    void displayLoop();

private:
    HALBundle m_hal;
    Config m_config;
    
    // Camera intrinsics
    DynalgoIntrinsic m_depth_intrinsic;
    float m_depth_scale = 0.001f;
    
    // Threads
    std::thread m_video_thread;
    std::thread m_encoder_thread;
    std::thread m_inference_thread;
    std::thread m_display_thread;
    std::atomic<bool> m_video_running{false};
    std::atomic<bool> m_encoder_running{false};
    std::atomic<bool> m_inference_running{false};
    std::atomic<bool> m_display_running{false};
    
    // Frame queues
    std::mutex m_frame_mutex;
    std::condition_variable m_frame_cv;
    std::queue<hal::FrameMetadata> m_frame_queue;
    std::vector<FrameConsumer> m_frame_consumers;
    
    // Encoder
    std::mutex m_encoder_mutex;
    std::condition_variable m_encoder_cv;
    std::queue<hal::FrameMetadata> m_encoder_queue;
    
    // Inference
    std::mutex m_inference_mutex;
    std::condition_variable m_inference_cv;
    std::queue<hal::FrameMetadata> m_inference_queue;
    
    // Display
    std::mutex m_display_mutex;
    std::condition_variable m_display_cv;
    std::queue<hal::FrameMetadata> m_display_queue;
    
    // File output
    std::string m_file_prefix = "recording";
    std::string m_output_dir;
    int m_frame_counter = 0;
    
    // Stats
    uint64_t m_frames_captured = 0;
    uint64_t m_frames_encoded = 0;
    uint64_t m_frames_inferred = 0;
    uint64_t m_last_report_time = 0;
    
    // Engagement loop
    void* m_engage_loop = nullptr;
    void* m_engage_model = nullptr;
    void* m_engage_actuator = nullptr;
};

} // namespace dynalgo