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

class FrameConsumer {
public:
    virtual ~FrameConsumer() = default;

    virtual void consume(std::shared_ptr<ob::FrameSet> frameSet) = 0;
    virtual void setViewer(SDLViewer *viewer, int viewerIdx) = 0;
    virtual void stopTask() = 0;
};

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
