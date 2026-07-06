// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame_consumer.cpp — FrameConsumer subclass implementations.

#include "nio_frame_consumer.hpp"

namespace nio {

// === ColorFrameConsumer ===

ColorFrameConsumer::ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer, int viewerIdx,
                                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles)
: encodeTask_(std::move(encodeTask)) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
    channel_ = channel;
    sensorFiles_ = std::move(sensorFiles);
}

void ColorFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* colorFrame = frameSet->getFrame(NioFrameType::COLOR);
    if (!colorFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(colorFrame->rawData(), colorFrame->dataSize(), colorFrame->timestampUs);
    dispatchViewIncr_(colorFrame);
    incrCount_(NioFrameType::COLOR);
}

void ColorFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
}

// === DepthFrameConsumer ===

DepthFrameConsumer::DepthFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask,
                                       std::shared_ptr<DepthRawTask> rawTask, SDLViewer* viewer, int viewerIdx,
                                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles,
                                       float depthScale, float depthMinM, float depthMaxM, int depthW, int depthH)
: encodeTask_(std::move(encodeTask))
, rawTask_(std::move(rawTask))
, depthScale_(depthScale)
, depthMinM_(depthMinM)
, depthMaxM_(depthMaxM)
, depthW_(depthW)
, depthH_(depthH) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
    channel_ = channel;
    sensorFiles_ = std::move(sensorFiles);
}

void DepthFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* depthFrame = frameSet->getFrame(NioFrameType::DEPTH);
    if (!depthFrame)
        return;

    auto format = depthFrame->format;
    auto* data = depthFrame->rawData();
    auto size = depthFrame->dataSize();

    if (format != NioFormat::H264 && format != NioFormat::H265 && format != NioFormat::HEVC) {
        if (rawTask_)
            rawTask_->enqueue(data, size, depthFrame->timestampUs);
    }

    if (encodeTask_) {
        if (format == NioFormat::Y16 && depthW_ > 0 && depthH_ > 0) {
            int w = depthFrame->width > 0 ? depthFrame->width : depthW_;
            int h = depthFrame->height > 0 ? depthFrame->height : depthH_;
            size_t rgbSize = static_cast<size_t>(w * h * 3);
            if (jetRgbBuf_.size() != rgbSize)
                jetRgbBuf_.resize(rgbSize);
            const uint16_t* y16 = reinterpret_cast<const uint16_t*>(data);
            depthY16ToJetRgb(y16, w, h, depthScale_, depthMinM_, depthMaxM_, jetRgbBuf_.data());
            encodeTask_->enqueue(jetRgbBuf_.data(), static_cast<uint32_t>(rgbSize), depthFrame->timestampUs);
        } else {
            encodeTask_->enqueue(data, size, depthFrame->timestampUs);
        }
    }

    dispatchViewIncrDepth_(depthFrame, depthMinM_, depthMaxM_);
    incrCount_(NioFrameType::DEPTH);
}

void DepthFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
    if (rawTask_)
        rawTask_->stop();
}

// === IRFrameConsumer (used for IR, IR-Left, IR-Right) ===

IRFrameConsumer::IRFrameConsumer(NioFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask,
                                 SDLViewer* viewer, int viewerIdx, ViewerChannel channel,
                                 std::shared_ptr<SensorFiles> sensorFiles)
: frameType_(frameType)
, encodeTask_(std::move(encodeTask)) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
    channel_ = channel;
    sensorFiles_ = std::move(sensorFiles);
}

void IRFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* irFrame = frameSet->getFrame(frameType_);
    if (!irFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(irFrame->rawData(), irFrame->dataSize(), irFrame->timestampUs);
    dispatchViewIncr_(irFrame);
    incrCount_(frameType_);
}

void IRFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
}

// === PointcloudFrameConsumer ===

PointcloudFrameConsumer::PointcloudFrameConsumer(std::shared_ptr<StreamTask> pcdTask,
                                                 std::shared_ptr<SensorFiles> sensorFiles)
: pcdTask_(std::move(pcdTask)) {
    viewerIdx_ = -1;
    sensorFiles_ = std::move(sensorFiles);
}

void PointcloudFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* pointFrame = frameSet->getFrame(NioFrameType::POINT);
    if (!pointFrame)
        return;
    if (pcdTask_)
        pcdTask_->enqueue(pointFrame->rawData(), pointFrame->dataSize(), pointFrame->timestampUs);
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, ViewerChannel::POINT, pointFrame->rawData(), pointFrame->dataSize());
    incrCount_(NioFrameType::POINT);
}

void PointcloudFrameConsumer::stopTask() {
    if (pcdTask_)
        pcdTask_->stop();
}

} // namespace nio
