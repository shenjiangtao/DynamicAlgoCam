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
#include "dynalgo_actuator.hpp"
#include "dynalgo_model.hpp"

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

// [类说明 / Class Description]
// 中文: 单设备采集会话 - 负责传感器枚举、编码器/文件创建、融合设置、管道启停和清理
// English: Per-device capture session: sensor enumeration, encoder/file creation, fusion setup, pipeline start/stop, and teardown
// Producer-consumer threading model:
//   - SDK video callback (producer): converts -> DynalgoFrameSet, enqueues to VideoFrameQueue
//   - Video consumer thread: dequeues FrameSets, dispatches to encode tasks, fusion task, viewer pushFrame, frame counter updates
//   - SDK IMU callback (producer): formats CSV line, enqueues to ImuFrameQueue
//   - IMU consumer thread: dequeues CSV lines, writes to ImuStreamTask

// Forward declaration (defined in algo/dynalgo_engagement_loop.hpp)
class DynalgoEngagementLoop;

class CaptureSession
{
public:
    // [方法说明 / Method Description]
    // 中文: 设置文件名前缀（事件驱动）
    // English: Set filename prefix (event-driven)
    void setFilePrefix(const std::string& prefix) { filePrefix_ = prefix; }
    
    // [方法说明 / Method Description]
    // 中文: 构造函数
    // English: Constructor
    CaptureSession(std::shared_ptr<DynalgoDevice> device, std::shared_ptr<DynalgoPipeline> pipeline,
                   const std::string& safeName, const std::string& deviceOutputDir, const CaptureConfig& cfg);

    // [方法说明 / Method Description]
    // 中文: 析构函数，清理资源
    // English: Destructor, cleanup resources
    ~CaptureSession();

    // [方法说明 / Method Description]
    // 中文: 初始化会话，创建编码器和任务
    // English: Initialize session, create encoders and tasks
    bool setup();

    // [方法说明 / Method Description]
    // 中文: 启动视频管道
    // English: Start video pipeline
    void startVideoPipeline(SDLViewer& viewer, bool noShow);
    
    // [方法说明 / Method Description]
    // 中文: 启动IMU管道
    // English: Start IMU pipeline
    void startImuPipeline();
    
    // [方法说明 / Method Description]
    // 中文: 停止所有管道和线程
    // English: Stop all pipelines and threads
    void stop();
    
    // [方法说明 / Method Description]
    // 中文: 报告FPS统计
    // English: Report FPS statistics
    void reportFps(uint64_t reportDurationMs);

    // [方法说明 / Method Description]
    // 中文: 获取设备安全名称
    // English: Get device safe name
    const std::string& deviceName() const {
        return safeName_;
    }

    // [方法说明 / Method Description]
    // 中文: 添加帧消费者到消费者链，需在startVideoPipeline前调用。线程安全
    // English: Append frame consumer to consumer chain. Call before startVideoPipeline(). Thread-safe
    void addFrameConsumer(std::unique_ptr<FrameConsumer> consumer);

    // [方法说明 / Method Description]
    // 中文: 获取深度内参（用于闭环Phase C）
    // English: Get depth intrinsic (for Phase C engagement loop)
    const DynalgoIntrinsic& depthIntrinsic() const { return sensorInfo_.depthIntrinsic; }
    
    // [方法说明 / Method Description]
    // 中文: 获取深度缩放因子
    // English: Get depth scale factor
    float depthScale() const { return depthScale_; }

    // [方法说明 / Method Description]
    // 中文: 存储闭环组件（模型/执行器），会话生命周期内保持存活。使用原始指针避免头文件依赖
    // English: Store engagement loop components (keeps model/actuator alive for session lifetime). Raw pointers avoid header dependency on dynalgo_engagement_loop.hpp. Ownership transferred; session deletes in destructor
    void setEngagementLoop(DynalgoEngagementLoop* loop,
                           DynalgoModelBackend* model,
                           DynalgoActuator* actuator);

    // [方法说明 / Method Description]
    // 中文: 是否有IMU传感器
    // English: Has IMU sensor
    bool hasIMU() const {
        return sensorInfo_.hasAccel && sensorInfo_.hasGyro;
    }
    
    // [方法说明 / Method Description]
    // 中文: 是否支持融合
    // English: Supports fusion
    bool canFuse() const {
        return canFuse_;
    }
    
    // [方法说明 / Method Description]
    // 中文: 是否有视频管道
    // English: Has video pipeline
    bool hasVideoPipeline() const {
        return pipeline_ != nullptr;
    }
    
    // [方法说明 / Method Description]
    // 中文: 获取并重置指定类型帧计数
    // English: Get and reset frame count for type
    uint64_t getAndResetFrameCount(DynalgoFrameType type);
    
    // [方法说明 / Method Description]
    // 中文: 获取并重置融合帧计数
    // English: Get and reset fusion frame count
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

    // Engagement loop (Phase C) — owned by session for lifetime management
    // Raw pointers (header avoids depending on dynalgo_engagement_loop.hpp).
    // Deleted in ~CaptureSession() / resetEngagementLoop().
    DynalgoEngagementLoop* engageLoop_ = nullptr;
    DynalgoModelBackend* engageModelBackend_ = nullptr;
    DynalgoActuator* engageActuator_ = nullptr;

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
