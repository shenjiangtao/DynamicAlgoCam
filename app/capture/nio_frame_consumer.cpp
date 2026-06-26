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

void ColorFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* colorFrame = frameSet->getFrame(NioFrameType::COLOR);
    if (!colorFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(colorFrame->rawData(), colorFrame->dataSize(), colorFrame->timestampUs);
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, channel_, colorFrame->rawData(), colorFrame->dataSize());
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    sensorFiles_->frameCounts[NioFrameType::COLOR]++;
}

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
                                       float depthScale, float depthMinM, float depthMaxM, int depthW, int depthH)
: encodeTask_(std::move(encodeTask))
, rawTask_(std::move(rawTask))
, viewer_(viewer)
, viewerIdx_(viewerIdx)
, channel_(channel)
, sensorFiles_(std::move(sensorFiles))
, depthScale_(depthScale)
, depthMinM_(depthMinM)
, depthMaxM_(depthMaxM)
, depthW_(depthW)
, depthH_(depthH) {}

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
            for (int i = 0; i < w * h; i++) {
                uint16_t raw = y16[i];
                if (raw == 0) {
                    jetRgbBuf_[i * 3 + 0] = 0;
                    jetRgbBuf_[i * 3 + 1] = 0;
                    jetRgbBuf_[i * 3 + 2] = 0;
                } else {
                    float distM = raw * depthScale_ / 1000.0f;
                    float norm = (distM - depthMinM_) / (depthMaxM_ - depthMinM_);
                    norm = std::max(0.0f, std::min(1.0f, norm));
                    uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                    jetColormap(v, jetRgbBuf_[i * 3 + 0], jetRgbBuf_[i * 3 + 1], jetRgbBuf_[i * 3 + 2]);
                }
            }
            encodeTask_->enqueue(jetRgbBuf_.data(), static_cast<uint32_t>(rgbSize), depthFrame->timestampUs);
        } else {
            encodeTask_->enqueue(data, size, depthFrame->timestampUs);
        }
    }

    float viewerDepthScale = depthFrame->depthScale;
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, channel_, data, size, viewerDepthScale, depthMinM_, depthMaxM_);
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    sensorFiles_->frameCounts[NioFrameType::DEPTH]++;
}

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

IRFrameConsumer::IRFrameConsumer(NioFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask,
                                 SDLViewer* viewer, int viewerIdx, ViewerChannel channel,
                                 std::shared_ptr<SensorFiles> sensorFiles)
: frameType_(frameType)
, encodeTask_(std::move(encodeTask))
, viewer_(viewer)
, viewerIdx_(viewerIdx)
, channel_(channel)
, sensorFiles_(std::move(sensorFiles)) {}

void IRFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* irFrame = frameSet->getFrame(frameType_);
    if (!irFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(irFrame->rawData(), irFrame->dataSize(), irFrame->timestampUs);
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, channel_, irFrame->rawData(), irFrame->dataSize());
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

// === PointcloudFrameConsumer ===

PointcloudFrameConsumer::PointcloudFrameConsumer(std::shared_ptr<PcdStreamTask> pcdTask,
                                                 std::shared_ptr<SensorFiles> sensorFiles)
: pcdTask_(std::move(pcdTask)), sensorFiles_(std::move(sensorFiles)) {}

void PointcloudFrameConsumer::consume(std::shared_ptr<NioFrameSet> frameSet) {
    auto* pointFrame = frameSet->getFrame(NioFrameType::POINT);
    if (!pointFrame)
        return;
    if (pcdTask_)
        pcdTask_->enqueue(pointFrame->rawData(), pointFrame->dataSize(), pointFrame->timestampUs);
    if (viewerIdx_ >= 0 && viewer_)
        viewer_->pushFrame(viewerIdx_, ViewerChannel::POINT, pointFrame->rawData(), pointFrame->dataSize());
    std::lock_guard<std::mutex> lock(sensorFiles_->countMtx);
    sensorFiles_->frameCounts[NioFrameType::POINT]++;
}

void PointcloudFrameConsumer::stopTask() {
    if (pcdTask_)
        pcdTask_->stop();
}

void PointcloudFrameConsumer::setViewer(SDLViewer* viewer, int viewerIdx) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
}

} // namespace nio
