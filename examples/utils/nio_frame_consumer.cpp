// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame_consumer.cpp — FrameConsumer subclass implementations.

#include "nio_frame_consumer.hpp"

namespace nio {

// === ColorFrameConsumer ===

ColorFrameConsumer::ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer, int viewerIdx,
                                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles)
: encodeTask_(std::move(encodeTask))
, viewer_(viewer)
, viewerIdx_(viewerIdx)
, channel_(channel)
, sensorFiles_(std::move(sensorFiles)) {}

// Extract color frame from FrameSet; enqueue to H264 encode task,
// push raw data to SDLViewer, and increment frame counter.
void ColorFrameConsumer::consume(std::shared_ptr<ob::FrameSet> frameSet) {
    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
    if (!colorFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(colorFrame->getData(), colorFrame->getDataSize(), colorFrame->getTimeStampUs());
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, channel_, colorFrame->getData(), colorFrame->getDataSize());
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    sensorFiles_->frameCounts[OB_FRAME_COLOR]++;
}

// Stop the H264 encode worker thread.
void ColorFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
}

void ColorFrameConsumer::setViewer(SDLViewer* viewer, int viewerIdx) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
}

// === DepthFrameConsumer ===

DepthFrameConsumer::DepthFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask,
                                       std::shared_ptr<DepthRawTask> rawTask, SDLViewer* viewer, int viewerIdx,
                                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles,
                                       float depthScale, float depthMinM, float depthMaxM)
: encodeTask_(std::move(encodeTask))
, rawTask_(std::move(rawTask))
, viewer_(viewer)
, viewerIdx_(viewerIdx)
, channel_(channel)
, sensorFiles_(std::move(sensorFiles))
, depthScale_(depthScale)
, depthMinM_(depthMinM)
, depthMaxM_(depthMaxM) {}

// Extract depth frame from FrameSet; enqueue Y16 raw to DepthRawTask (unless
// native H264/H265), enqueue to H264 encode task, push to SDLViewer with
// depth scale and range for colormap, and increment frame counter.
void DepthFrameConsumer::consume(std::shared_ptr<ob::FrameSet> frameSet) {
    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
    if (!depthFrame)
        return;

    auto format = depthFrame->getFormat();
    auto data = depthFrame->getData();
    auto size = depthFrame->getDataSize();

    if (format != OB_FORMAT_H264 && format != OB_FORMAT_H265 && format != OB_FORMAT_HEVC) {
        if (rawTask_)
            rawTask_->enqueue(data, size, depthFrame->getTimeStampUs());
    }

    if (encodeTask_)
        encodeTask_->enqueue(data, size, depthFrame->getTimeStampUs());

    float viewerDepthScale = depthScale_;
    try {
        auto depthF = depthFrame->as<ob::DepthFrame>();
        if (depthF)
            viewerDepthScale = depthF->getValueScale();
    } catch (...) {
    }
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, channel_, data, size, viewerDepthScale, depthMinM_, depthMaxM_);
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    sensorFiles_->frameCounts[OB_FRAME_DEPTH]++;
}

// Stop both H264 encode and raw-write worker threads.
void DepthFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
    if (rawTask_)
        rawTask_->stop();
}

void DepthFrameConsumer::setViewer(SDLViewer* viewer, int viewerIdx) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
}

// === IRFrameConsumer (used for IR, IR-Left, IR-Right) ===

IRFrameConsumer::IRFrameConsumer(OBFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer,
                                 int viewerIdx, ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles)
: frameType_(frameType)
, encodeTask_(std::move(encodeTask))
, viewer_(viewer)
, viewerIdx_(viewerIdx)
, channel_(channel)
, sensorFiles_(std::move(sensorFiles)) {}

// Extract IR frame (type determined by frameType_) from FrameSet;
// enqueue to H264 encode task, push to SDLViewer, increment frame counter.
void IRFrameConsumer::consume(std::shared_ptr<ob::FrameSet> frameSet) {
    auto irFrame = frameSet->getFrame(frameType_);
    if (!irFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(irFrame->getData(), irFrame->getDataSize(), irFrame->getTimeStampUs());
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, channel_, irFrame->getData(), irFrame->getDataSize());
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    sensorFiles_->frameCounts[frameType_]++;
}

void IRFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
}

void IRFrameConsumer::setViewer(SDLViewer* viewer, int viewerIdx) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
}

} // namespace nio
