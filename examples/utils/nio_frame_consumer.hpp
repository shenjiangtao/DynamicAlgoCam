// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame_consumer.hpp — FrameConsumer base class + per-stream subclasses.
//
// Each subclass encapsulates the encode/dispatch/viewer/count logic for one
// stream type. CaptureSession holds a vector of FrameConsumer pointers and
// iterates them in the video consumer thread.

#pragma once

#include "nio_frame.hpp"
#include "nio_sdl_viewer.hpp"
#include "nio_stream_io.hpp"
#include "nio_stream_tasks.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace nio {

// FrameConsumer: abstract base for per-stream frame dispatch in the video
// consumer thread. Each subclass handles encode, viewer push, and frame
// counting for one sensor type (color / depth / IR variants).

class FrameConsumer {
public:
    virtual ~FrameConsumer() = default;

    // Dispatch one NioFrameSet: extract the relevant frame, enqueue to encode
    // task, push to viewer, and increment frame counter.
    virtual void consume(std::shared_ptr<NioFrameSet> frameSet) = 0;

    // Late-bind the viewer pointer and device slot index.
    virtual void setViewer(SDLViewer *viewer, int viewerIdx) = 0;

    // Stop the underlying StreamTask worker thread(s).
    virtual void stopTask() = 0;
};

// ColorFrameConsumer: handles COLOR frames — H264 encode + viewer display + count.
class ColorFrameConsumer : public FrameConsumer {
public:
    ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask,
                       SDLViewer *viewer, int viewerIdx, ViewerChannel channel,
                       std::shared_ptr<SensorFiles> sensorFiles);

    void consume(std::shared_ptr<NioFrameSet> frameSet) override;
    void setViewer(SDLViewer *viewer, int viewerIdx) override;
    void stopTask() override;

private:
    std::shared_ptr<EncodeStreamTask> encodeTask_;
    SDLViewer *viewer_;
    int viewerIdx_;
    ViewerChannel channel_;
    std::shared_ptr<SensorFiles> sensorFiles_;
};

// DepthFrameConsumer: handles DEPTH frames — H264 encode + Y16 raw write +
// viewer display (with depthScale/depthRange colormap) + count.
class DepthFrameConsumer : public FrameConsumer {
public:
    DepthFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask,
                       std::shared_ptr<DepthRawTask> rawTask,
                       SDLViewer *viewer, int viewerIdx, ViewerChannel channel,
                       std::shared_ptr<SensorFiles> sensorFiles,
                       float depthScale, float depthMinM, float depthMaxM);

    void consume(std::shared_ptr<NioFrameSet> frameSet) override;
    void setViewer(SDLViewer *viewer, int viewerIdx) override;
    void stopTask() override;

private:
    std::shared_ptr<EncodeStreamTask> encodeTask_;
    std::shared_ptr<DepthRawTask> rawTask_;
    SDLViewer *viewer_;
    int viewerIdx_;
    ViewerChannel channel_;
    std::shared_ptr<SensorFiles> sensorFiles_;
    float depthScale_;
    float depthMinM_;
    float depthMaxM_;
};

// IRFrameConsumer: handles IR / IR-LEFT / IR-RIGHT frames — H264 encode +
// viewer display + count. Shared by all three IR variants via frameType_/channel_ params.
class IRFrameConsumer : public FrameConsumer {
public:
    IRFrameConsumer(NioFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask,
                    SDLViewer *viewer, int viewerIdx, ViewerChannel channel,
                    std::shared_ptr<SensorFiles> sensorFiles);

    void consume(std::shared_ptr<NioFrameSet> frameSet) override;
    void setViewer(SDLViewer *viewer, int viewerIdx) override;
    void stopTask() override;

private:
    NioFrameType frameType_;
    std::shared_ptr<EncodeStreamTask> encodeTask_;
    SDLViewer *viewer_;
    int viewerIdx_;
    ViewerChannel channel_;
    std::shared_ptr<SensorFiles> sensorFiles_;
};

} // namespace nio
