// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_capture_session.hpp — Per-device capture session: sensor enumeration,
// encoder/file creation, fusion setup, pipeline start/stop, and teardown.
//
// Producer-consumer threading model:
//   - SDK video callback (producer): converts → NioFrameSet,
//     enqueues to VideoFrameQueue, returns immediately — zero blocking.
//   - Video consumer thread: dequeues FrameSets, dispatches to encode tasks,
//     fusion task, viewer pushFrame, frame counter updates.
//   - SDK IMU callback (producer): formats CSV line, enqueues to ImuFrameQueue.
//   - IMU consumer thread: dequeues CSV lines, writes to ImuStreamTask.

#pragma once

#include "nio_capture_config.hpp"
#include "nio_color_convert.hpp"
#include "nio_common.hpp"
#include "nio_device.hpp"
#include "nio_frame.hpp"
#include "nio_frame_consumer.hpp"
#include "nio_frame_queue.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_stream_io.hpp"
#include "nio_stream_tasks.hpp"
#include "nio_thread.hpp"
#include "nio_types.hpp"

#ifdef ENABLE_RS_AC1
#include "nio_rs_adapter.hpp"
#endif

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

namespace nio {

class CaptureSession {
public:
    CaptureSession(std::shared_ptr<NioDevice> device,
                   std::shared_ptr<NioPipeline> pipeline,
                   const std::string &safeName,
                   const std::string &deviceOutputDir,
                   const CaptureConfig &cfg);

    bool setup();

    void startVideoPipeline(SDLViewer &viewer, bool noShow);
    void startImuPipeline();
    void stop();
    void reportFps(uint64_t reportDurationMs);

    const std::string &deviceName() const { return safeName_; }
    bool hasIMU() const { return sensorInfo_.hasAccel && sensorInfo_.hasGyro; }
    bool canFuse() const { return canFuse_; }
    bool hasVideoPipeline() const { return pipeline_ != nullptr; }
    uint64_t getAndResetFrameCount(NioFrameType type);
    uint64_t getAndResetFusionCount();

private:
    void createEncodersAndTasks();
    void createColorEncoder();
    void createDepthEncoder();
    void createIREncoder(NioFrameType type, const std::string &suffix, NioFormat fmt,
                         int w, int h, int fps, ViewerChannel ch);
    void createImuTask();
    void createPcdTask();

    void setupFusion();
    void writeIntrinsicJson();
    void setupViewerSlot(SDLViewer &viewer);

    void videoConsumerLoop();
    void imuConsumerLoop();

    std::shared_ptr<NioDevice> device_;
    std::shared_ptr<NioPipeline> pipeline_;
    std::string safeName_;
    std::string deviceOutputDir_;
    CaptureConfig cfg_;

    std::shared_ptr<SensorFiles> sensorFiles_;
    NioSensorInfo sensorInfo_;

    float depthScale_ = 0.001f;
    std::string startTs_;
    std::string baseName_;

    std::mutex fusedMtx_;
    std::shared_ptr<MjpgDecoderRes> mjpgRes_;
    int fusedFps_ = 30;
    bool hwD2CMode_ = false;
    bool canFuse_ = false;
    int viewerIdx_ = -1;

    std::vector<std::unique_ptr<FrameConsumer>> frameConsumers_;
    std::shared_ptr<FusionStreamTask> fusionTask_;
    std::shared_ptr<ImuStreamTask> imuTask_;
    std::shared_ptr<PcdStreamTask> pcdTask_;

    std::string devId_;

    VideoFrameQueue videoQueue_{8};
    ImuFrameQueue imuQueue_{32};
    std::thread videoConsumerThread_;
    std::thread imuConsumerThread_;
    std::atomic<bool> consumersRunning_{false};

    SDLViewer *viewer_ = nullptr;
};

} // namespace nio
