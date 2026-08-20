// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_stream_tasks.cpp — StreamTask subclass implementations.

#include "dynalgo_stream_tasks.hpp"
#include "dynalgo_color_convert.hpp"
#include "dynalgo_log.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace dynalgo {

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

FusionStreamTask::FusionStreamTask(const std::string& name, int colorW, int colorH, DynalgoFormat colorFormat,
                                   int /*fusedFps*/, std::shared_ptr<H264Encoder> fusedEncoder,
                                   std::shared_ptr<std::ofstream> fusedFile, std::mutex& fusedMtx, bool hwD2CMode,
float alpha, float depthMinM, float depthMaxM, float depthScale,
                                 std::shared_ptr<MjpgDecoderRes> mjpgRes)
: StreamTask(name, 8)
, colorW_(colorW)
, colorH_(colorH)
, colorFormat_(colorFormat)
, fusedEncoder_(std::move(fusedEncoder))
, fusedFile_(std::move(fusedFile))
, fusedMtx_(fusedMtx)
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

void FusionStreamTask::enqueueDynalgoFrameSet(std::shared_ptr<DynalgoFrameSet> frameSet) {
    {
        std::lock_guard<std::mutex> lock(frameSetMtx_);
        latestFrameSet_ = std::move(frameSet);
        frameSetReady_ = true;
    }
    // Push a 0-size sentinel blob into the StreamTask queue so run()'s cv_.wait_for
    // predicate (count_ > 0) becomes true and the worker wakes immediately, rather
    // than waiting for the 100ms wait_for timeout.  processFrame() forwards to
    // onIdle() which performs the actual doBlend.  Use a non-null pointer to
    // stay within std::memcpy's defined behavior (nullptr+0 is UB even though
    // the size is zero).
    static const uint8_t zeroSentinel = 0;
    enqueue(&zeroSentinel, 0, 0);
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
    static const uint8_t zeroSentinel = 0;
    enqueue(&zeroSentinel, 0, 0);
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
    static const uint8_t zeroSentinel = 0;
    enqueue(&zeroSentinel, 0, 0);
}

void FusionStreamTask::processFrame(const FrameBlob&) {
    onIdle();
}

// onIdle for hardware D2C mode: blend from latest color+depth buffers.
// Color-driven fusion: emit a fused frame whenever color is ready,
// reusing the most-recent depth frame. Depth arrival wakes the loop
// but does not reset readiness — only color consumption clears it.
void FusionStreamTask::onIdleHwD2C() {
    if (!colorReady_.load() || !depthReady_.load())
        return;

    std::lock(colorMtx_, depthMtx_);
    std::lock_guard<std::mutex> colorLk(colorMtx_, std::adopt_lock);
    std::lock_guard<std::mutex> depthLk(depthMtx_, std::adopt_lock);

    doBlend(latestColor_.data.data(), latestColor_.size, latestColor_.timestampUs, latestDepth_.data.data(),
            latestDepth_.size, latestDepth_.timestampUs, latestDepth_.depthScale);

    // Reset only colour — keep depthReady_ so the next colour frame reuses it
    colorReady_ = false;
}

// onIdle for software D2C mode: frames are already aligned in the driver,
// blend color+depth directly from DynalgoFrameSet
void FusionStreamTask::onIdleSwD2C() {
    if (!frameSetReady_.load())
        return;

    std::shared_ptr<DynalgoFrameSet> dynalgoFrameSet;
    {
        std::lock_guard<std::mutex> lock(frameSetMtx_);
        dynalgoFrameSet = std::move(latestFrameSet_);
        frameSetReady_ = false;
    }

    if (!dynalgoFrameSet)
        return;

    auto* colorFrame = dynalgoFrameSet->getFrame(DynalgoFrameType::COLOR);
    auto* depthFrame = dynalgoFrameSet->getFrame(DynalgoFrameType::DEPTH);
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
    float rangeM = maxDist - minDist;
    if (rangeM <= 0.0f)
        rangeM = 1.0f;

    // Per-pixel fusion: rawVal==0 keeps color, else alpha-blend color + jet(depth).
    auto blendPixel = [&](uint16_t rawVal, int idx) {
        if (rawVal == 0) {
            (*fusedRGBBuf_)[idx + 0] = (*colorRGBBuf_)[idx + 0];
            (*fusedRGBBuf_)[idx + 1] = (*colorRGBBuf_)[idx + 1];
            (*fusedRGBBuf_)[idx + 2] = (*colorRGBBuf_)[idx + 2];
        } else {
            float distM = rawVal * scale / 1000.0f;
            float norm = (distM - minDist) / rangeM;
            if (norm < 0.0f)
                norm = 0.0f;
            else if (norm > 1.0f)
                norm = 1.0f;
            uint8_t v = static_cast<uint8_t>(norm * 255.0f);
            uint8_t cr, cg, cb;
            jetColormap(v, cr, cg, cb);
            float inv = 1.0f - al;
            (*fusedRGBBuf_)[idx + 0] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 0] + al * cr);
            (*fusedRGBBuf_)[idx + 1] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 1] + al * cg);
            (*fusedRGBBuf_)[idx + 2] = static_cast<uint8_t>(inv * (*colorRGBBuf_)[idx + 2] + al * cb);
        }
    };

    if (depthSize >= static_cast<uint32_t>(w * h * 2)) {
        const uint16_t* depthPtr = reinterpret_cast<const uint16_t*>(depthData);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                uint16_t rawVal = depthPtr[y * w + x];
                int idx = (y * w + x) * 3;
                blendPixel(rawVal, idx);
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
                    blendPixel(rawVal, idx);
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
}

void ImuStreamTask::enqueueLine(std::string line) {
    enqueue(reinterpret_cast<const uint8_t*>(line.data()), static_cast<uint32_t>(line.size()), 0);
}

// === PcdWriterTask ===

PcdWriterTask::PcdWriterTask(const std::string& name, const std::string& outputDir, const std::string& baseName,
                             Mode mode)
: StreamTask(name, 4), outputDir_(outputDir), baseName_(baseName), mode_(mode) {}

PcdWriterTask::~PcdWriterTask() {
    if (mode_ == Mode::Stream && pcdStream_.file && pcdStream_.file->is_open()) {
        std::lock_guard<std::mutex> lock(fileMtx_);
        writePcdStreamIndex(pcdStream_);
    }
}

void PcdWriterTask::processFrame(const FrameBlob& blob) {
    if (mode_ == Mode::Single) {
        writePcdFile(outputDir_, baseName_, blob.data.data(), blob.size, fileMtx_, blob.timestampUs);
        frameCount++;
        return;
    }

    // Stream mode: lazy-open the .pcs file, append frames + index entries.
    std::lock_guard<std::mutex> lock(fileMtx_);
    if (!pcdStream_.file || !pcdStream_.file->is_open()) {
        mkdirRecursive(outputDir_);
        std::string path = outputDir_ + "/" + baseName_ + ".pcs";
        pcdStream_.file = openBufferedFile(path, std::ios::binary, DYNALGO_FILE_BUF_SIZE, &pcdStream_.fileBuf);
        if (!pcdStream_.file || !pcdStream_.file->is_open())
            return;
    }
    writePcdStreamFrame(pcdStream_, blob.data.data(), blob.size, blob.timestampUs);
    frameCount++;
}

void PcdWriterTask::onStop() {
    if (mode_ != Mode::Stream)
        return;
    std::lock_guard<std::mutex> lock(fileMtx_);
    if (pcdStream_.file && pcdStream_.file->is_open())
        writePcdStreamIndex(pcdStream_);
}

} // namespace dynalgo
