// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_tasks.cpp — StreamTask subclass implementations.

#include "nio_stream_tasks.hpp"
#include "nio_color_convert.hpp"
#include "nio_log.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace nio {

// === EncodeStreamTask ===

EncodeStreamTask::EncodeStreamTask(const std::string& name, std::shared_ptr<StreamEncoder> se)
: StreamTask(name), se_(std::move(se)) {}

void EncodeStreamTask::processFrame(const FrameBlob& blob) {
    if (se_ && se_->file && se_->file->is_open()) {
        writeStreamFrame(se_.get(), blob.data.data(), blob.size, blob.timestampUs);
    }
    frameCount++;
}

// === DepthRawTask ===

DepthRawTask::DepthRawTask(const std::string& name, std::shared_ptr<std::ofstream> file, int width, int height,
                           float depthScale)
: StreamTask(name), file_(std::move(file)), width_(width), height_(height), depthScale_(depthScale) {}

void DepthRawTask::processFrame(const FrameBlob& blob) {
    if (!file_ || !file_->is_open())
        return;

    writeDepthRawWithHeader(*file_, blob.data.data(), blob.size, width_, height_, depthScale_, frameIdx_++, fileMtx_,
                            blob.timestampUs);
}

// === FusionStreamTask ===

FusionStreamTask::FusionStreamTask(const std::string& name, int colorW, int colorH, NioFormat colorFormat,
                                   int /*fusedFps*/, std::shared_ptr<H264Encoder> fusedEncoder,
                                   std::shared_ptr<std::ofstream> fusedFile, std::mutex& fusedMtx,
                                   std::shared_ptr<NioD2CAlign> alignFilter, bool hwD2CMode, float alpha,
                                   float depthMinM, float depthMaxM, float depthScale,
                                   std::shared_ptr<MjpgDecoderRes> mjpgRes)
: StreamTask(name, 4)
, colorW_(colorW)
, colorH_(colorH)
, colorFormat_(colorFormat)
, fusedEncoder_(std::move(fusedEncoder))
, fusedFile_(std::move(fusedFile))
, fusedMtx_(fusedMtx)
, alignFilter_(std::move(alignFilter))
, hwD2CMode_(hwD2CMode)
, alpha_(alpha)
, depthMinM_(depthMinM)
, depthMaxM_(depthMaxM)
, depthScale_(depthScale)
, mjpgRes_(std::move(mjpgRes)) {
    int rgbBufSize = colorW * colorH * 3;
    colorRGBBuf_ = std::make_shared<std::vector<uint8_t>>(rgbBufSize, 0);
    fusedRGBBuf_ = std::make_shared<std::vector<uint8_t>>(rgbBufSize, 0);
    if (hwD2CMode_) {
        latestColor_.data.resize(colorW * colorH * 4 + 1024);
        latestDepth_.data.resize(colorW * colorH * 2 + 1024);
    }
}

void FusionStreamTask::enqueueNioFrameSet(std::shared_ptr<NioFrameSet> frameSet) {
    {
        std::lock_guard<std::mutex> lock(frameSetMtx_);
        latestFrameSet_ = std::move(frameSet);
        frameSetReady_ = true;
    }
    wakeup();
}

void FusionStreamTask::enqueueColor(const uint8_t* data, uint32_t size, uint64_t timestampUs) {
    {
        std::lock_guard<std::mutex> lock(colorMtx_);
        if (latestColor_.data.size() < size)
            latestColor_.data.resize(size);
        std::memcpy(latestColor_.data.data(), data, size);
        latestColor_.size = size;
        latestColor_.timestampUs = timestampUs;
        colorReady_ = true;
    }
    wakeup();
}

void FusionStreamTask::enqueueDepth(const uint8_t* data, uint32_t size, uint64_t timestampUs, float depthScale) {
    {
        std::lock_guard<std::mutex> lock(depthMtx_);
        if (latestDepth_.data.size() < size)
            latestDepth_.data.resize(size);
        std::memcpy(latestDepth_.data.data(), data, size);
        latestDepth_.size = size;
        latestDepth_.timestampUs = timestampUs;
        latestDepth_.depthScale = depthScale;
        depthReady_ = true;
    }
    wakeup();
}

void FusionStreamTask::processFrame(const FrameBlob&) {
    onIdle();
}

// onIdle for hardware D2C mode: blend from latest color+depth buffers
// 硬件D2C模式空闲回调：从最新color+depth缓冲进行融合
void FusionStreamTask::onIdleHwD2C() {
    if (!colorReady_.load() || !depthReady_.load())
        return;

    FrameBuf colorBuf, depthBuf;
    {
        std::lock_guard<std::mutex> lock(colorMtx_);
        colorBuf = latestColor_;
        colorReady_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(depthMtx_);
        depthBuf = latestDepth_;
        depthReady_ = false;
    }

    doBlend(colorBuf.data.data(), colorBuf.size, colorBuf.timestampUs, depthBuf.data.data(), depthBuf.size,
            depthBuf.timestampUs, depthBuf.depthScale);
}

// onIdle for software D2C mode: align FrameSet then blend color+depth
// 软件D2C模式空闲回调：对齐FrameSet后融合color+depth
void FusionStreamTask::onIdleSwD2C() {
    if (!frameSetReady_.load())
        return;

    std::shared_ptr<NioFrameSet> nioFrameSet;
    {
        std::lock_guard<std::mutex> lock(frameSetMtx_);
        nioFrameSet = std::move(latestFrameSet_);
        frameSetReady_ = false;
    }

    if (!nioFrameSet)
        return;

    if (alignFilter_ && nioFrameSet->nativeFrameSet) {
        NioAlignedFrame aligned;
        if (alignFilter_->process(nioFrameSet->nativeFrameSet, aligned)) {
            doBlend(aligned.colorData, aligned.colorSize, aligned.colorTs, aligned.depthData, aligned.depthSize,
                    aligned.depthTs, aligned.depthScale);
            return;
        }
    }

    auto* colorFrame = nioFrameSet->getFrame(NioFrameType::COLOR);
    auto* depthFrame = nioFrameSet->getFrame(NioFrameType::DEPTH);
    if (!colorFrame || !depthFrame)
        return;

    doBlend(colorFrame->rawData(), colorFrame->dataSize(), colorFrame->timestampUs, depthFrame->rawData(),
            depthFrame->dataSize(), depthFrame->timestampUs, depthFrame->depthScale);
}

// onIdle: dispatch to HW or SW D2C path
// 空闲回调：分派至硬件或软件D2C路径
void FusionStreamTask::onIdle() {
    if (hwD2CMode_) {
        onIdleHwD2C();
    } else {
        onIdleSwD2C();
    }
}

void FusionStreamTask::doBlend(const uint8_t* colorData, uint32_t colorSize, uint64_t /*colorTs*/,
                               const uint8_t* depthData, uint32_t depthSize, uint64_t depthTs, float depthScale) {
    int w = colorW_;
    int h = colorH_;

    bool colorOk = decodeColorToRGB(colorData, colorSize, colorFormat_, w, h, colorRGBBuf_->data(), mjpgRes_);
    if (!colorOk) {
        std::memset(colorRGBBuf_->data(), 128, w * h * 3);
    }

    float scale = depthScale_;
    if (depthScale > 0.0f)
        scale = depthScale;

    float minDist = depthMinM_;
    float maxDist = depthMaxM_;
    float al = alpha_;

    if (depthSize >= static_cast<uint32_t>(w * h * 2)) {
        const uint16_t* depthPtr = reinterpret_cast<const uint16_t*>(depthData);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                uint16_t rawVal = depthPtr[y * w + x];
                int idx = (y * w + x) * 3;
                if (rawVal == 0) {
                    (*fusedRGBBuf_)[idx + 0] = (*colorRGBBuf_)[idx + 0];
                    (*fusedRGBBuf_)[idx + 1] = (*colorRGBBuf_)[idx + 1];
                    (*fusedRGBBuf_)[idx + 2] = (*colorRGBBuf_)[idx + 2];
                } else {
                    float distM = rawVal * scale / 1000.0f;
                    float norm = (distM - minDist) / (maxDist - minDist);
                    norm = std::max(0.0f, std::min(1.0f, norm));
                    uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                    uint8_t cr, cg, cb;
                    jetColormap(v, cr, cg, cb);
                    float inv = 1.0f - al;
                    (*fusedRGBBuf_)[idx + 0] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 0] + al * cr);
                    (*fusedRGBBuf_)[idx + 1] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 1] + al * cg);
                    (*fusedRGBBuf_)[idx + 2] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 2] + al * cb);
                }
            }
        }
    } else if (depthSize >= 2) {
        int dw = static_cast<int>(depthSize / 2);
        int dh = 1;
        while (dw > 1 && dw * dh * 2 < static_cast<int>(depthSize)) {
            dh++;
        }
        if (depthSize >= static_cast<uint32_t>(96 * 288 * 2)) {
            dw = 96;
            dh = 288;
        } else {
            dh = 1;
            for (int cand = 1; cand <= dw; cand++) {
                if (depthSize >= static_cast<uint32_t>(cand * (dw / cand) * 2) && dw % cand == 0) {
                    dh = dw / cand;
                    dw = cand;
                    break;
                }
            }
        }

        if (dw > 0 && dh > 0) {
            const uint16_t* depthPtr = reinterpret_cast<const uint16_t*>(depthData);
            for (int y = 0; y < h; y++) {
                int sy = y * dh / h;
                if (sy >= dh)
                    sy = dh - 1;
                for (int x = 0; x < w; x++) {
                    int sx = x * dw / w;
                    if (sx >= dw)
                        sx = dw - 1;
                    uint16_t rawVal = depthPtr[sy * dw + sx];
                    int idx = (y * w + x) * 3;
                    if (rawVal == 0) {
                        (*fusedRGBBuf_)[idx + 0] = (*colorRGBBuf_)[idx + 0];
                        (*fusedRGBBuf_)[idx + 1] = (*colorRGBBuf_)[idx + 1];
                        (*fusedRGBBuf_)[idx + 2] = (*colorRGBBuf_)[idx + 2];
                    } else {
                        float distM = rawVal * scale / 1000.0f;
                        float norm = (distM - minDist) / (maxDist - minDist);
                        norm = std::max(0.0f, std::min(1.0f, norm));
                        uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                        uint8_t cr, cg, cb;
                        jetColormap(v, cr, cg, cb);
                        float inv = 1.0f - al;
                        (*fusedRGBBuf_)[idx + 0] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 0] + al * cr);
                        (*fusedRGBBuf_)[idx + 1] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 1] + al * cg);
                        (*fusedRGBBuf_)[idx + 2] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 2] + al * cb);
                    }
                }
            }
        } else {
            std::memcpy(fusedRGBBuf_->data(), colorRGBBuf_->data(), w * h * 3);
        }
    } else {
        std::memcpy(fusedRGBBuf_->data(), colorRGBBuf_->data(), w * h * 3);
    }

    fusedEncoder_->encodeRGB(fusedRGBBuf_->data(), *fusedFile_, fusedMtx_, depthTs);
    frameCount++;
}

// === ImuStreamTask ===

ImuStreamTask::ImuStreamTask(const std::string& name, std::shared_ptr<std::ofstream> imuFile)
: StreamTask(name, 8), imuFile_(std::move(imuFile)) {}

void ImuStreamTask::processFrame(const FrameBlob& blob) {
    if (!imuFile_ || !imuFile_->is_open())
        return;
    std::string line(reinterpret_cast<const char*>(blob.data.data()), blob.size);
    std::lock_guard<std::mutex> lock(fileMtx_);
    *imuFile_ << line;
    imuFile_->flush();
}

void ImuStreamTask::enqueueLine(std::string line) {
    enqueue(reinterpret_cast<const uint8_t*>(line.data()), static_cast<uint32_t>(line.size()), 0);
}

// === PcdStreamTask ===

PcdStreamTask::PcdStreamTask(const std::string& name, std::shared_ptr<std::ofstream> pcdFile)
: StreamTask(name, 4), pcdFile_(std::move(pcdFile)) {}

void PcdStreamTask::processFrame(const FrameBlob& blob) {
    if (!pcdFile_ || !pcdFile_->is_open())
        return;
    writePointRawWithHeader(*pcdFile_, blob.data.data(), blob.size, frameIdx_++, fileMtx_, blob.timestampUs);
    frameCount++;
}

} // namespace nio
