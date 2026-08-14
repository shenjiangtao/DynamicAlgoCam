// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame_consumer.cpp — FrameConsumer subclass implementations.

#include "dynalgo_frame_consumer.hpp"
#include "dynalgo_log.hpp"

#include <algorithm>
#include <cstring>

namespace dynalgo {

// === ColorFrameConsumer ===

ColorFrameConsumer::ColorFrameConsumer(std::shared_ptr<EncodeStreamTask> encodeTask, SDLViewer* viewer, int viewerIdx,
                                       ViewerChannel channel, std::shared_ptr<SensorFiles> sensorFiles)
: encodeTask_(std::move(encodeTask)) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
    channel_ = channel;
    sensorFiles_ = std::move(sensorFiles);
}

void ColorFrameConsumer::consume(std::shared_ptr<DynalgoFrameSet> frameSet) {
    auto* colorFrame = frameSet->getFrame(DynalgoFrameType::COLOR);
    if (!colorFrame)
        return;
    if (encodeTask_)
        encodeTask_->enqueue(colorFrame->rawData(), colorFrame->dataSize(), colorFrame->timestampUs);
    dispatchViewIncr_(colorFrame);
    incrCount_(DynalgoFrameType::COLOR);
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

void DepthFrameConsumer::consume(std::shared_ptr<DynalgoFrameSet> frameSet) {
    auto* depthFrame = frameSet->getFrame(DynalgoFrameType::DEPTH);
    if (!depthFrame)
        return;

    auto format = depthFrame->format;
    auto* data = depthFrame->rawData();
    auto size = depthFrame->dataSize();

    if (format != DynalgoFormat::H264 && format != DynalgoFormat::H265 && format != DynalgoFormat::HEVC) {
        if (rawTask_)
            rawTask_->enqueue(data, size, depthFrame->timestampUs);
    }

    if (encodeTask_) {
        if (format == DynalgoFormat::Y16 && depthW_ > 0 && depthH_ > 0) {
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
    incrCount_(DynalgoFrameType::DEPTH);
}

void DepthFrameConsumer::stopTask() {
    if (encodeTask_)
        encodeTask_->stop();
    if (rawTask_)
        rawTask_->stop();
}

// === IRFrameConsumer (used for IR, IR-Left, IR-Right) ===

IRFrameConsumer::IRFrameConsumer(DynalgoFrameType frameType, std::shared_ptr<EncodeStreamTask> encodeTask,
                                 SDLViewer* viewer, int viewerIdx, ViewerChannel channel,
                                 std::shared_ptr<SensorFiles> sensorFiles)
: frameType_(frameType)
, encodeTask_(std::move(encodeTask)) {
    viewer_ = viewer;
    viewerIdx_ = viewerIdx;
    channel_ = channel;
    sensorFiles_ = std::move(sensorFiles);
}

void IRFrameConsumer::consume(std::shared_ptr<DynalgoFrameSet> frameSet) {
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
                                                 std::shared_ptr<SensorFiles> sensorFiles,
                                                 DynalgoIntrinsic depthIntrinsic, float depthScale)
: pcdTask_(std::move(pcdTask))
, depthIntrinsic_(depthIntrinsic)
, depthScale_(depthScale) {
    viewerIdx_ = -1;
    sensorFiles_ = std::move(sensorFiles);
}

// Self-computed back-projection: pinhole model with intrinsic (fx,fy,cx,cy) and
// per-pixel depth scale. Returns a vector of packed float3 (X,Y,Z), one triple
// per valid pixel (Z > 0). This is the fallback path used when the SDK has not
// attached a POINT frame; it is also reused for the dual-track comparison when
// a driver POINT frame is present.
static std::vector<float> backprojectToPointCloud(const uint16_t* y16, int w, int h, const DynalgoIntrinsic& k,
                                                  float scale, size_t* validCount = nullptr) {
    std::vector<float> cloud;
    if (validCount)
        *validCount = 0;
    if (!y16 || w <= 0 || h <= 0 || k.fx == 0.0f || k.fy == 0.0f || scale == 0.0f)
        return cloud;
    cloud.reserve(static_cast<size_t>(w) * h * 3);
    const float invFx = 1.0f / k.fx;
    const float invFy = 1.0f / k.fy;
    size_t valid = 0;
    for (int v = 0; v < h; v++) {
        for (int u = 0; u < w; u++) {
            uint16_t d = y16[v * w + u];
            if (d == 0)
                continue;
            float Z = d * scale;
            if (Z > 0.0f) {
                float X = (u - k.cx) * Z * invFx;
                float Y = (v - k.cy) * Z * invFy;
                cloud.push_back(X);
                cloud.push_back(Y);
                cloud.push_back(Z);
                valid++;
            }
        }
    }
    if (validCount)
        *validCount = valid;
    return cloud;
}

// Pack a self-computed XYZ cloud into the same self-describing wire layout the
// SDK's POINT frames use (see PcdLayout::obXyz): 12B header + N*24B field
// descriptors + N*12B packed XYZ. Used when the driver provides no POINT frame
// so the existing PcdWriterTask can ingest the fallback cloud unchanged.
static void packObXyzWire(std::vector<uint8_t>& out, const std::vector<float>& xyz, uint64_t ts) {
    PcdLayout layout = PcdLayout::obXyz();
    uint32_t nPts = static_cast<uint32_t>(xyz.size() / 3);
    uint32_t nFields = static_cast<uint32_t>(layout.fields.size());
    size_t hdrSize = 12 + nFields * sizeof(PcdFieldDesc);
    out.resize(hdrSize + static_cast<size_t>(nPts) * layout.srcPointSize);
    uint8_t* dst = out.data();
    std::memcpy(dst, &layout.srcPointSize, 4); dst += 4;
    std::memcpy(dst, &nFields, 4); dst += 4;
    std::memcpy(dst, &nPts, 4); dst += 4;
    std::memcpy(dst, layout.fields.data(), nFields * sizeof(PcdFieldDesc));
    dst += nFields * sizeof(PcdFieldDesc);
    std::memcpy(dst, xyz.data(), static_cast<size_t>(nPts) * layout.srcPointSize);
}

void PointcloudFrameConsumer::consume(std::shared_ptr<DynalgoFrameSet> frameSet) {
    auto* driverPoint = frameSet->getFrame(DynalgoFrameType::POINT);

    // Always attempt the self-computed back-projection from the DEPTH Y16 frame:
    // (a) it is the fallback when the driver did not produce a POINT frame, and
    // (b) it feeds the dual-track comparison when the driver frame IS present.
    std::vector<float> ownCloud;
    size_t ownValid = 0;
    uint64_t depthTs = 0;
    if (depthIntrinsic_.fx != 0.0f && depthIntrinsic_.fy != 0.0f && depthScale_ > 0.0f) {
        auto* depthFrame = frameSet->getFrame(DynalgoFrameType::DEPTH);
        if (depthFrame && depthFrame->format == DynalgoFormat::Y16) {
            int w = depthFrame->width > 0 ? depthFrame->width : depthIntrinsic_.width;
            int h = depthFrame->height > 0 ? depthFrame->height : depthIntrinsic_.height;
            ownCloud = backprojectToPointCloud(reinterpret_cast<const uint16_t*>(depthFrame->rawData()),
                                               w, h, depthIntrinsic_, depthScale_, &ownValid);
            depthTs = depthFrame->timestampUs;
        }
    }

    if (driverPoint) {
        // Primary path: use the driver's POINT frame for downstream PCD output.
        if (pcdTask_)
            pcdTask_->enqueue(driverPoint->rawData(), driverPoint->dataSize(), driverPoint->timestampUs);
        if (viewerIdx_ >= 0 && viewer_)
            viewer_->pushFrame(viewerIdx_, ViewerChannel::POINT, driverPoint->rawData(), driverPoint->dataSize());
        incrCount_(DynalgoFrameType::POINT);

        // Dual-track comparison: parse the driver wire header (PcdLayout::obXyz)
        // to count its points and sample a few XYZ entries, then log the delta
        // vs. the self-computed cloud. Only a sanity check — does NOT feed PcdTask.
        if (!ownCloud.empty()) {
            PcdLayout driverLayout;
            uint32_t driverPts = 0;
            size_t consumed = PcdLayout::deserialize(driverPoint->rawData(), driverPoint->dataSize(),
                                                    driverLayout, driverPts);
            if (consumed > 0 && driverPts > 0 && driverLayout.srcPointSize >= 12) {
                const uint8_t* pts = driverPoint->rawData() + consumed;
                // Sample first 8 points' XYZ mean for a stable scalar comparison.
                size_t sampleN = std::min<uint32_t>(driverPts, 8u);
                double dx = 0, dy = 0, dz = 0;
                for (size_t i = 0; i < sampleN; i++) {
                    const float* p = reinterpret_cast<const float*>(pts + i * driverLayout.srcPointSize);
                    dx += p[0]; dy += p[1]; dz += p[2];
                }
                double ox = 0, oy = 0, oz = 0;
                size_t ownN = std::min<size_t>(ownValid, sampleN);
                for (size_t i = 0; i < ownN; i++) {
                    ox += ownCloud[i * 3 + 0];
                    oy += ownCloud[i * 3 + 1];
                    oz += ownCloud[i * 3 + 2];
                }
                if (ownN) { ox /= ownN; oy /= ownN; oz /= ownN; }
                if (sampleN) { dx /= sampleN; dy /= sampleN; dz /= sampleN; }
                DYNALGO_LOG_DEBUG_S("PCD dual-track: driverPts=" << driverPts << " ownPts=" << ownValid
                               << " | sample-mean[driver]=(" << dx << "," << dy << "," << dz
                               << ") vs own=(" << ox << "," << oy << "," << oz << ")");
            } else {
                DYNALGO_LOG_DEBUG_S("PCD dual-track: ownPts=" << ownValid
                               << " but driver POINT wire failed to parse (size=" << driverPoint->dataSize() << ")");
            }
        }
        return;
    }

    // Fallback path: no driver POINT frame this cycle — synthesize one with the
    // self-computed back-projection cloud using PcdLayout::obXyz() so PcdWriterTask
    // can ingest it unchanged. Downstream algorithms stay on this wire format.
    if (!ownCloud.empty() && pcdTask_) {
        std::vector<uint8_t> wire;
        packObXyzWire(wire, ownCloud, depthTs);
        pcdTask_->enqueue(wire.data(), static_cast<uint32_t>(wire.size()), depthTs);
        if (viewerIdx_ >= 0 && viewer_)
            viewer_->pushFrame(viewerIdx_, ViewerChannel::POINT, wire.data(), static_cast<uint32_t>(wire.size()));
        incrCount_(DynalgoFrameType::POINT);
        DYNALGO_LOG_DEBUG_S("PCD fallback: self-computed back-projection fed pcdTask (ownPts=" << ownValid << ")");
    }
}

void PointcloudFrameConsumer::stopTask() {
    if (pcdTask_)
        pcdTask_->stop();
}

} // namespace dynalgo
