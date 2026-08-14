// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_capture_session.hpp — Per-device capture session: sensor enumeration,
// encoder/file creation, fusion setup, pipeline start/stop, and teardown.
//
// Producer-consumer threading model:
//   - SDK video callback (producer): converts → DynalgoFrameSet,
//     enqueues to VideoFrameQueue, returns immediately — zero blocking.
//   - Video consumer thread: dequeues FrameSets, dispatches to encode tasks,
//     fusion task, viewer pushFrame, frame counter updates.
//   - SDK IMU callback (producer): formats CSV line, enqueues to ImuFrameQueue.
//   - IMU consumer thread: dequeues CSV lines, writes to ImuStreamTask.

#pragma once

#include "dynalgo_capture_config.hpp"
#include "dynalgo_color_convert.hpp"
#include "dynalgo_common.hpp"
#include "dynalgo_device.hpp"
#include "dynalgo_frame.hpp"
#include "dynalgo_frame_consumer.hpp"
#include "dynalgo_frame_queue.hpp"
#include "dynalgo_h264_encoder.hpp"
#include "dynalgo_stream_io.hpp"
#include "dynalgo_stream_tasks.hpp"
#include "dynalgo_thread.hpp"
#include "dynalgo_types.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace dynalgo {

class CaptureSession
{
public:
    // Existing public members ...
    // New method to set filename prefix (event‑driven)
    void setFilePrefix(const std::string& prefix) { filePrefix_ = prefix; }
    CaptureSession(std::shared_ptr<DynalgoDevice> device, std::shared_ptr<DynalgoPipeline> pipeline,
                   const std::string& safeName, const std::string& deviceOutputDir, const CaptureConfig& cfg);

    bool setup();

    void startVideoPipeline(SDLViewer& viewer, bool noShow);
    void startImuPipeline();
    void stop();
    void reportFps(uint64_t reportDurationMs);

    const std::string& deviceName() const {
        return safeName_;
    }
    bool hasIMU() const {
        return sensorInfo_.hasAccel && sensorInfo_.hasGyro;
    }
    bool canFuse() const {
        return canFuse_;
    }
    bool hasVideoPipeline() const {
        return pipeline_ != nullptr;
    }
    uint64_t getAndResetFrameCount(DynalgoFrameType type);
    uint64_t getAndResetFusionCount();

private:
    void createEncodersAndTasks();
    void createColorEncoder();
    void createDepthEncoder();
    void createIREncoder(DynalgoFrameType type, const std::string& suffix, DynalgoFormat fmt, int w, int h, int fps,
                         ViewerChannel ch);
    void createImuTask();
    void createPcdTask();

    void setupFusion();
    void writeIntrinsicJson();
    void setupViewerSlot(SDLViewer& viewer);

    void videoConsumerLoop();
    void imuConsumerLoop();

    std::shared_ptr<DynalgoDevice> device_;
    std::shared_ptr<DynalgoPipeline> pipeline_;
    std::string safeName_;
    std::string deviceOutputDir_;
    CaptureConfig cfg_;

    std::shared_ptr<SensorFiles> sensorFiles_;
    DynalgoSensorInfo sensorInfo_;

    float depthScale_ = 0.001f;
    std::string startTs_;
    std::string baseName_; // base path without prefix
    std::string filePrefix_; // e.g. "E0_1713145678901"


    std::mutex fusedMtx_;
    std::shared_ptr<MjpgDecoderRes> mjpgRes_;
    int fusedFps_ = 30;
    bool hwD2CMode_ = false;
    bool canFuse_ = false;
    int viewerIdx_ = -1;

    std::vector<std::unique_ptr<FrameConsumer>> frameConsumers_;
    std::shared_ptr<FusionStreamTask> fusionTask_;
    std::shared_ptr<ImuStreamTask> imuTask_;
    std::shared_ptr<StreamTask> pcdTask_;

    std::string devId_;

    VideoFrameQueue videoQueue_{ 8 };
    ImuFrameQueue imuQueue_{ 32 };
    std::thread videoConsumerThread_;
    std::thread imuConsumerThread_;
    std::atomic<bool> consumersRunning_{ false };

    // Replaces the previous hard-coded 500ms post-start sleep with an
    // event-driven wait — the SDK callback notifies firstFrameCv_ as
    // soon as the first frame arrives, or wait_for times out (2s) as a
    // safety net for slow-starting devices.
    std::mutex firstFrameMtx_;
    std::condition_variable firstFrameCv_;
    bool firstFrameSeen_ = false;

    SDLViewer* viewer_ = nullptr;
};

} // namespace dynalgo
