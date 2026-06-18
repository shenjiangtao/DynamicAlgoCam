// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_capture_session.hpp — Per-device capture session: sensor enumeration,
// encoder/file creation, fusion setup, pipeline start/stop, and teardown.
//
// Producer-consumer threading model:
//   - SDK video callback (producer): converts ob::FrameSet → NioFrameSet,
//     enqueues to VideoFrameQueue, returns immediately — zero blocking.
//   - Video consumer thread: dequeues FrameSets, dispatches to encode tasks,
//     fusion task, viewer pushFrame, frame counter updates.
//   - SDK IMU callback (producer): formats CSV line, enqueues to ImuFrameQueue.
//   - IMU consumer thread: dequeues CSV lines, writes to ImuStreamTask.

#pragma once

#include "nio_capture_config.hpp"
#include "nio_color_convert.hpp"
#include "nio_common.hpp"
#include "nio_frame.hpp"
#include "nio_frame_consumer.hpp"
#include "nio_frame_queue.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_ob_adapter.hpp"
#include "nio_ob_frame_adapter.hpp"
#include "nio_sdl_viewer.hpp"
#include "nio_stream_io.hpp"
#include "nio_stream_tasks.hpp"
#include "nio_thread.hpp"
#include "nio_types.hpp"
#include "utils.hpp"

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
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <libobsensor/ObSensor.hpp>

namespace nio {

struct SensorInfo {
    bool hasColor = false;
    bool hasDepth = false;
    bool hasIR = false;
    bool hasIRLeft = false;
    bool hasIRRight = false;
    bool hasAccel = false;
    bool hasGyro = false;

    NioFormat colorFormat = NioFormat::UNKNOWN;
    NioFormat depthFormat = NioFormat::UNKNOWN;
    NioFormat irFormat = NioFormat::UNKNOWN;
    NioFormat irLeftFormat = NioFormat::UNKNOWN;
    NioFormat irRightFormat = NioFormat::UNKNOWN;

    int colorW = 0, colorH = 0, colorFps = 30;
    int depthW = 0, depthH = 0, depthFps = 30;
    int irW = 0, irH = 0, irFps = 30;
    int irLW = 0, irLH = 0, irLFps = 30;
    int irRW = 0, irRH = 0, irRFps = 30;

    std::shared_ptr<ob::VideoStreamProfile> colorProfile;
    std::shared_ptr<ob::VideoStreamProfile> depthProfile;
    std::shared_ptr<ob::VideoStreamProfile> irProfile;
    std::shared_ptr<ob::VideoStreamProfile> irLeftProfile;
    std::shared_ptr<ob::VideoStreamProfile> irRightProfile;

    NioIntrinsic depthIntrinsic;
    NioIntrinsic colorIntrinsic;
};

class CaptureSession {
public:
    CaptureSession(std::shared_ptr<ob::Device> device,
                   const std::string &safeName,
                   const std::string &deviceOutputDir,
                   const CaptureConfig &cfg);

    // Initialize device: sync timer, enumerate sensors, create encoders/tasks/fusion.
    bool setup();

    // Start video pipeline with SDL viewer (or headless if noShow).
    // Spawns video consumer thread; SDK callback pushes to VideoFrameQueue.
    void startVideoPipeline(SDLViewer &viewer, bool noShow);

    // Start IMU pipeline. Spawns IMU consumer thread; SDK callback pushes to ImuFrameQueue.
    void startImuPipeline();

    // Stop all pipelines, join consumer threads, stop tasks, close files.
    void stop();

    // Print per-stream FPS to stdout for the last reportDurationMs interval.
    void reportFps(uint64_t reportDurationMs);

    const std::string &deviceName() const { return safeName_; }
    bool hasIMU() const { return sensorInfo_.hasAccel && sensorInfo_.hasGyro; }
    bool canFuse() const { return canFuse_; }
    bool hasVideoPipeline() const { return videoPipeline_ != nullptr; }
    uint64_t getAndResetFrameCount(NioFrameType type);
    uint64_t getAndResetFusionCount();

private:
    // --- Sensor enumeration helpers (called from enumerateSensors) ---
    bool enumerateSensors();
    void enumerateColorSensor(const std::shared_ptr<ob::StreamProfileList> &profiles, NioFormat preferredFmt);
    void enumerateDepthSensor(const std::shared_ptr<ob::StreamProfileList> &profiles);
    void enumerateIRSensor(const std::shared_ptr<ob::StreamProfileList> &profiles);
    void enumerateIRLeftSensor(const std::shared_ptr<ob::StreamProfileList> &profiles);
    void enumerateIRRightSensor(const std::shared_ptr<ob::StreamProfileList> &profiles);
    void detectDepthScale();
    void applyDeviceQuirks();

    // --- Encoder / task creation helpers ---
    void createEncodersAndTasks();
    void createColorEncoder();
    void createDepthEncoder();
    void createIREncoder(NioFrameType type, const std::string &suffix, NioFormat fmt,
                         int w, int h, int fps, ViewerChannel ch);
    void createImuTask();

    // --- Fusion ---
    void setupFusion();

    // --- Intrinsic JSON ---
    void writeIntrinsicJson();

    // --- HW alignment check ---
    bool checkHWD2CAlign();

    // --- Viewer setup ---
    void setupViewerSlot(SDLViewer &viewer);

    // --- IMU callback helper ---
    void onImuFrameSet(std::shared_ptr<ob::FrameSet> frameSet);

    // --- Teardown helpers ---
    void closeEncoders();
    void closeFiles();

    // --- Consumer thread loops ---
    // Video consumer thread: dequeues FrameSets, dispatches to fusion + frameConsumers_.
    void videoConsumerLoop();
    // IMU consumer thread: dequeues CSV lines, forwards to ImuStreamTask.
    void imuConsumerLoop();

    std::shared_ptr<ob::Device> device_;
    std::string safeName_;
    std::string deviceOutputDir_;
    CaptureConfig cfg_;

    std::shared_ptr<ob::Pipeline> videoPipeline_;
    std::shared_ptr<ob::Pipeline> imuPipeline_;
    std::shared_ptr<ob::Config> videoConfig_;
    std::shared_ptr<SensorFiles> sensorFiles_;
    SensorInfo sensorInfo_;

    float depthScale_ = 0.001f;
    std::string startTs_;
    std::string baseName_;

    std::shared_ptr<ob::Align> alignFilter_;
    std::mutex fusedMtx_;
    std::shared_ptr<MjpgDecoderRes> mjpgRes_;
    int fusedFps_ = 30;
    bool hwD2CMode_ = false;
    bool canFuse_ = false;
    int viewerIdx_ = -1;

    // Per-stream consumers (color/depth/IR/IR-left/IR-right).
    // Built in createEncodersAndTasks(); viewer bound in startVideoPipeline().
    std::vector<std::unique_ptr<FrameConsumer>> frameConsumers_;
    std::shared_ptr<FusionStreamTask> fusionTask_;
    std::shared_ptr<ImuStreamTask> imuTask_;

    std::string devId_;

    VideoFrameQueue videoQueue_{8};
    ImuFrameQueue imuQueue_{32};
    std::thread videoConsumerThread_;
    std::thread imuConsumerThread_;
    std::atomic<bool> consumersRunning_{false};

    SDLViewer *viewer_ = nullptr;
};

} // namespace nio
