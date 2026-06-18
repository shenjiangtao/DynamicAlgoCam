// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame_consumer.hpp — FrameConsumer base class + per-stream subclasses.
//
// Each subclass encapsulates the encode/dispatch/viewer/count logic for one
// stream type. CaptureSession holds a vector of FrameConsumer pointers and
// iterates them in the video consumer thread.

#pragma once

#include "nio_sdl_viewer.hpp"
#include "nio_stream_io.hpp"
#include "nio_stream_tasks.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <libobsensor/ObSensor.hpp>

namespace nio {

// FrameConsumer: abstract base for per-stream frame dispatch in the video
// consumer thread. Each subclass handles encode, viewer push, and frame
// counting for one sensor type (color / depth / IR variants).
//
// Lifecycle:
//   1. Constructed in CaptureSession::createEncodersAndTasks() with
//      encodeTask + sensorFiles (viewer is null at this point).
//   2. setViewer() called in startVideoPipeline() after the SDLViewer
//      slot index is assigned — delayed because the viewer is created later.
//   3. consume() called repeatedly by the video consumer thread.
//   4. stopTask() called during CaptureSession::stop() to join worker threads.

class FrameConsumer {
public:
    virtual ~FrameConsumer() = default;

    // Dispatch one FrameSet: extract the relevant frame, enqueue to encode
    // task, push to viewer, and increment frame counter.
    virtual void consume(std::shared_ptr<ob::FrameSet> frameSet) = 0;

    // Late-bind the viewer pointer and device slot index. Called after
    // addDevice() assigns a slot, before the consumer thread starts.
    virtual void setViewer(SDLViewer *viewer, int viewerIdx) = 0;

    // Stop the underlying StreamTask worker thread(s). Called during teardown.
    virtual void stopTask() = 0;
};

// ColorFrameConsumer: handles COLOR frames — H264 encode + viewer display + count.
class ColorFrameConsumer : public FrameConsumer {
public:
    ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask,
                       SDLViewer *viewer, int viewerIdx, ViewerChannel channel,
                       std::shared_ptr<SensorFiles> sensorFiles);

    void consume(std::shared_ptr<ob::FrameSet> frameSet) override;
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

    void consume(std::shared_ptr<ob::FrameSet> frameSet) override;
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
    IRFrameConsumer(OBFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask,
                    SDLViewer *viewer, int viewerIdx, ViewerChannel channel,
                    std::shared_ptr<SensorFiles> sensorFiles);

    void consume(std::shared_ptr<ob::FrameSet> frameSet) override;
    void setViewer(SDLViewer *viewer, int viewerIdx) override;
    void stopTask() override;

private:
    OBFrameType frameType_;
    std::shared_ptr<EncodeStreamTask> encodeTask_;
    SDLViewer *viewer_;
    int viewerIdx_;
    ViewerChannel channel_;
    std::shared_ptr<SensorFiles> sensorFiles_;
};

} // namespace nio
