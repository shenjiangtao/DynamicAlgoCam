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

// FrameConsumer: abstract base for per-stream frame dispatch in the video
// consumer thread. Each subclass handles encode, viewer push, and frame
// counting for one sensor type (color / depth / IR variants).
//
// Common viewer/sensorFiles state is held here.  Helper dispatchViewIncr_()
// runs the (viewer push + count increment) tail shared by every consumer.
class FrameConsumer
{
public:
    virtual ~FrameConsumer() = default;

    // Dispatch one DynalgoFrameSet: extract the relevant frame, enqueue to encode
    // task, push to viewer, and increment frame counter.
    virtual void consume(std::shared_ptr<DynalgoFrameSet> frameSet) = 0;

    // Late-bind the viewer pointer and device slot index.
    void setViewer(SDLViewer* viewer, int viewerIdx) {
        viewer_ = viewer;
        viewerIdx_ = viewerIdx;
    }

    // Stop the underlying StreamTask worker thread(s).
    virtual void stopTask() = 0;

protected:
    // Push one frame to the viewer (if bound) with the channel stored in the
    // subclass via setChannel().  Thread-safe: viewer handles its own lock.
    void dispatchViewIncr_(const DynalgoFrame* frame) {
        if (viewerIdx_ >= 0 && viewer_)
            viewer_->pushFrame(viewerIdx_, channel_, frame->rawData(), frame->dataSize());
    }

    // Push depth frame to viewer with extra metadata.
    void dispatchViewIncrDepth_(const DynalgoFrame* frame, float depthMinM, float depthMaxM) {
        if (viewerIdx_ >= 0 && viewer_)
            viewer_->pushFrame(viewerIdx_, channel_, frame->rawData(), frame->dataSize(), frame->depthScale,
                               depthMinM, depthMaxM);
    }

    // Increment the per-type counter under SensorFiles::countMtx.
    void incrCount_(DynalgoFrameType type) {
        std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
        sensorFiles_->frameCounts[type]++;
    }

    void setChannel_(ViewerChannel ch) { channel_ = ch; }

    SDLViewer* viewer_ = nullptr;
    int viewerIdx_ = -1;
    ViewerChannel channel_{};
    std::shared_ptr<SensorFiles> sensorFiles_;
};

// ColorFrameConsumer: handles COLOR frames — H264 encode + viewer display + count.
class ColorFrameConsumer : public FrameConsumer
{
public:
    ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer, int viewerIdx,
                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles);

    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    void stopTask() override;

private:
    std::shared_ptr<EncodeStreamTask> encodeTask_;
};

// DepthFrameConsumer: handles DEPTH frames — H264 encode + Y16 raw write +
// viewer display (with depthScale/depthRange colormap) + count.
class DepthFrameConsumer : public FrameConsumer
{
public:
    DepthFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, std::shared_ptr<DepthRawTask> rawTask,
                       SDLViewer* viewer, int viewerIdx, ViewerChannel channel,
                       std::shared_ptr<SensorFiles> sensorFiles, float depthScale, float depthMinM, float depthMaxM,
                       int depthW, int depthH);

    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
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

// IRFrameConsumer: handles IR / IR-LEFT / IR-RIGHT frames — H264 encode +
// viewer display + count. Shared by all three IR variants via frameType_/channel_ params.
class IRFrameConsumer : public FrameConsumer
{
public:
    IRFrameConsumer(DynalgoFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer,
                    int viewerIdx, ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles);

    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    void stopTask() override;

private:
    DynalgoFrameType frameType_;
    std::shared_ptr<EncodeStreamTask> encodeTask_;
};

class PointcloudFrameConsumer : public FrameConsumer
{
public:
    // depthIntrinsic / depthScale: used for the self-computed back-projection
    // cross-check against the SDK-produced POINT frame. When depthIntrinsic.fx
    // is 0 the cross-check is skipped silently.
    PointcloudFrameConsumer(std::shared_ptr<StreamTask> pcdTask, std::shared_ptr<SensorFiles> sensorFiles,
                            DynalgoIntrinsic depthIntrinsic = {}, float depthScale = 0.0f);

    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    void stopTask() override;

private:
    std::shared_ptr<StreamTask> pcdTask_;
    DynalgoIntrinsic depthIntrinsic_;
    float depthScale_;
};

} // namespace dynalgo
