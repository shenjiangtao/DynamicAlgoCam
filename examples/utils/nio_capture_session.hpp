// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_capture_session.hpp — Per-device capture session: sensor enumeration,
// encoder/file creation, fusion setup, pipeline start/stop, and teardown.
//
// CaptureSession owns all per-device state (pipelines, encoders, tasks,
// files) and manages their lifecycle from setup() through stop().

#pragma once

#include "nio_capture_config.hpp"
#include "nio_color_convert.hpp"
#include "nio_common.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_sdl_viewer.hpp"
#include "nio_stream_io.hpp"
#include "nio_stream_tasks.hpp"
#include "nio_thread.hpp"
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

    OBFormat colorFormat = OB_FORMAT_UNKNOWN;
    OBFormat depthFormat = OB_FORMAT_UNKNOWN;
    OBFormat irFormat = OB_FORMAT_UNKNOWN;
    OBFormat irLeftFormat = OB_FORMAT_UNKNOWN;
    OBFormat irRightFormat = OB_FORMAT_UNKNOWN;

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

    OBCameraIntrinsic depthIntrinsic = {};
    OBCameraIntrinsic colorIntrinsic = {};
};

class CaptureSession {
public:
    CaptureSession(std::shared_ptr<ob::Device> device,
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
    bool hasVideoPipeline() const { return videoPipeline_ != nullptr; }
    uint64_t getAndResetFrameCount(OBFrameType type);
    uint64_t getAndResetFusionCount();

private:
    bool enumerateSensors();
    void createEncodersAndTasks();
    void setupFusion();
    void writeIntrinsicJson();
    bool checkHWD2CAlign();

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

    std::shared_ptr<EncodeStreamTask> colorEncodeTask_;
    std::shared_ptr<EncodeStreamTask> depthEncodeTask_;
    std::shared_ptr<EncodeStreamTask> irEncodeTask_;
    std::shared_ptr<EncodeStreamTask> irLeftEncodeTask_;
    std::shared_ptr<EncodeStreamTask> irRightEncodeTask_;
    std::shared_ptr<DepthRawTask> depthRawTask_;
    std::shared_ptr<FusionStreamTask> fusionTask_;
    std::shared_ptr<ImuStreamTask> imuTask_;

    std::string devId_;
};

} // namespace nio
