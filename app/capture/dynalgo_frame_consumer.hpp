// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame_consumer.hpp — FrameConsumer base class + per-stream subclasses.
//
// Each subclass encapsulates the encode/dispatch/viewer/count logic for one
// stream type. CaptureSession holds a vector of FrameConsumer pointers and
// iterates them in the video consumer thread.
//
// Common state (viewer, viewerIdx, channel, sensorFiles) lives in the base
// class, along with helper methods for viewer binding, counter increment, and
// the encode→viewer→count dispatch template.  Subclasses only override
// consume() to extract their frame and (optionally) transform it before
// enqueue + viewer push + count.

#pragma once

#include "dynalgo_color_convert.hpp"
#include "dynalgo_frame.hpp"
#include "dynalgo_sdl_viewer.hpp"
#include "dynalgo_stream_io.hpp"
#include "dynalgo_stream_tasks.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 帧消费者抽象基类，用于视频消费线程中的逐流派发。各子类处理特定传感器类型的编码、查看器推送和帧计数
// English: FrameConsumer abstract base for per-stream frame dispatch in video consumer thread. Subclasses handle encode, viewer push, and frame counting for one sensor type
class FrameConsumer
{
public:
    virtual ~FrameConsumer() = default;

    // [方法说明 / Method Description]
    // 中文: 派发一个DynalgoFrameSet：提取相关帧、入队编码任务、推送查看器、增加帧计数
    // English: Dispatch one DynalgoFrameSet: extract relevant frame, enqueue to encode task, push to viewer, increment frame counter
    virtual void consume(std::shared_ptr<DynalgoFrameSet> frameSet) = 0;

    // [方法说明 / Method Description]
    // 中文: 后期绑定查看器指针和设备槽位索引
    // English: Late-bind the viewer pointer and device slot index
    void setViewer(SDLViewer* viewer, int viewerIdx) {
        viewer_ = viewer;
        viewerIdx_ = viewerIdx;
    }

    // [方法说明 / Method Description]
    // 中文: 停止底层StreamTask工作线程
    // English: Stop the underlying StreamTask worker thread(s)
    virtual void stopTask() = 0;

protected:
    // [方法说明 / Method Description]
    // 中文: 推送一帧到查看器（若已绑定），使用子类通过setChannel设置的通道。线程安全
    // English: Push one frame to viewer (if bound) with channel stored in subclass via setChannel(). Thread-safe
    void dispatchViewIncr_(const DynalgoFrame* frame) {
        if (viewerIdx_ >= 0 && viewer_)
            viewer_->pushFrame(viewerIdx_, channel_, frame->rawData(), frame->dataSize());
    }

    // [方法说明 / Method Description]
    // 中文: 推送深度帧到查看器，带深度元数据
    // English: Push depth frame to viewer with extra metadata
    void dispatchViewIncrDepth_(const DynalgoFrame* frame, float depthMinM, float depthMaxM) {
        if (viewerIdx_ >= 0 && viewer_)
            viewer_->pushFrame(viewerIdx_, channel_, frame->rawData(), frame->dataSize(), frame->depthScale,
                               depthMinM, depthMaxM);
    }

    // [方法说明 / Method Description]
    // 中文: 在SensorFiles::countMtx保护下增加类型计数器
    // English: Increment per-type counter under SensorFiles::countMtx
    void incrCount_(DynalgoFrameType type) {
        std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
        sensorFiles_->frameCounts[type]++;
    }

    // [方法说明 / Method Description]
    // 中文: 设置查看器通道
    // English: Set viewer channel
    void setChannel_(ViewerChannel ch) { channel_ = ch; }

    SDLViewer* viewer_ = nullptr;
    int viewerIdx_ = -1;
    ViewerChannel channel_{};
    std::shared_ptr<SensorFiles> sensorFiles_;
};

// [类说明 / Class Description]
// 中文: 彩色帧消费者 - H264编码 + 查看器显示 + 计数
// English: ColorFrameConsumer: handles COLOR frames — H264 encode + viewer display + count
class ColorFrameConsumer : public FrameConsumer
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数
    // English: Constructor
    ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer, int viewerIdx,
                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles);

    // [方法说明 / Method Description]
    // 中文: 消费帧集，提取彩色帧处理
    // English: Consume frame set, extract and process color frame
    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    // [方法说明 / Method Description]
    // 中文: 停止编码任务
    // English: Stop encode task
    void stopTask() override;

private:
    std::shared_ptr<EncodeStreamTask> encodeTask_;
};

// [类说明 / Class Description]
// 中文: 深度帧消费者 - H264编码 + Y16原始写入 + 查看器显示(深度着色) + 计数
// English: DepthFrameConsumer: handles DEPTH frames — H264 encode + Y16 raw write + viewer display (with depthScale/depthRange colormap) + count
class DepthFrameConsumer : public FrameConsumer
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数
    // English: Constructor
    DepthFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, std::shared_ptr<DepthRawTask> rawTask,
                       SDLViewer* viewer, int viewerIdx, ViewerChannel channel,
                       std::shared_ptr<SensorFiles> sensorFiles, float depthScale, float depthMinM, float depthMaxM,
                       int depthW, int depthH);

    // [方法说明 / Method Description]
    // 中文: 消费帧集，提取深度帧处理
    // English: Consume frame set, extract and process depth frame
    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    // [方法说明 / Method Description]
    // 中文: 停止编码和原始写入任务
    // English: Stop encode and raw write tasks
    void stopTask() override;

private:
    std::shared_ptr<EncodeStreamTask> encodeTask_;
    std::shared_ptr<DepthRawTask> rawTask_;
    float depthScale_;
    float depthMinM_;
    float depthMaxM_;
    int depthW_;
    int depthH_;
    std::vector<uint8_t> jetRgbBuf_;
};

// [类说明 / Class Description]
// 中文: 红外帧消费者 - 处理IR/IR-LEFT/IR-RIGHT三种变体，通过frameType_/channel_参数区分
// English: IRFrameConsumer: handles IR / IR-LEFT / IR-RIGHT frames — H264 encode + viewer display + count. Shared by all three IR variants via frameType_/channel_ params
class IRFrameConsumer : public FrameConsumer
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数
    // English: Constructor
    IRFrameConsumer(DynalgoFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer,
                    int viewerIdx, ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles);

    // [方法说明 / Method Description]
    // 中文: 消费帧集，根据frameType_提取对应红外帧
    // English: Consume frame set, extract corresponding IR frame based on frameType_
    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    // [方法说明 / Method Description]
    // 中文: 停止编码任务
    // English: Stop encode task
    void stopTask() override;

private:
    DynalgoFrameType frameType_;
    std::shared_ptr<EncodeStreamTask> encodeTask_;
};

// [类说明 / Class Description]
// 中文: 点云帧消费者 - 用于SDK生成的POINT帧回投验证
// English: PointcloudFrameConsumer: used for self-computed back-projection cross-check against SDK-produced POINT frame
class PointcloudFrameConsumer : public FrameConsumer
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数。depthIntrinsic.fx为0时跳过回投验证
    // English: Constructor. Cross-check skipped silently when depthIntrinsic.fx is 0
    PointcloudFrameConsumer(std::shared_ptr<StreamTask> pcdTask, std::shared_ptr<SensorFiles> sensorFiles,
                            DynalgoIntrinsic depthIntrinsic = {}, float depthScale = 0.0f);

    // [方法说明 / Method Description]
    // 中文: 消费帧集，处理点云数据
    // English: Consume frame set, process pointcloud data
    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    // [方法说明 / Method Description]
    // 中文: 停止点云任务
    // English: Stop pointcloud task
    void stopTask() override;

private:
    std::shared_ptr<StreamTask> pcdTask_;
    DynalgoIntrinsic depthIntrinsic_;
    float depthScale_;
};

} // namespace dynalgo
