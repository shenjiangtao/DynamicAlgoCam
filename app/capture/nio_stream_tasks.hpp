// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_stream_tasks.hpp — StreamTask subclasses for per-stream capture threads.
//
// EncodeStreamTask: H264 encode + file write for color/depth/IR streams.
// DepthRawTask: Y16 raw file write with per-frame header.
// FusionStreamTask: D2C alignment + alpha-blend + fused H264 encode.
// ImuStreamTask: IMU accel/gyro CSV line writer.

#pragma once

#include "nio_color_convert.hpp"
#include "nio_common.hpp"
#include "nio_device.hpp"
#include "nio_frame.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_stream_io.hpp"
#include "nio_thread.hpp"
#include "nio_types.hpp"

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nio {

class EncodeStreamTask : public StreamTask {
public:
    EncodeStreamTask(const std::string &name, std::shared_ptr<StreamEncoder> se);
    std::atomic<uint64_t> frameCount{0};

protected:
    void processFrame(const FrameBlob &blob) override;

private:
    std::shared_ptr<StreamEncoder> se_;
};

class DepthRawTask : public StreamTask {
public:
    DepthRawTask(const std::string &name, std::shared_ptr<std::ofstream> file,
                 int width, int height, float depthScale);

protected:
    void processFrame(const FrameBlob &blob) override;

private:
    std::shared_ptr<std::ofstream> file_;
    int width_;
    int height_;
    float depthScale_;
    std::mutex fileMtx_;
    uint64_t frameIdx_ = 0;
};

class FusionStreamTask : public StreamTask {
public:
    FusionStreamTask(const std::string &name,
                     int colorW, int colorH, NioFormat colorFormat, int fusedFps,
                     std::shared_ptr<H264Encoder> fusedEncoder,
                     std::shared_ptr<std::ofstream> fusedFile,
                     std::mutex &fusedMtx,
                     std::shared_ptr<NioD2CAlign> alignFilter,
                     bool hwD2CMode,
                     float alpha, float depthMinM, float depthMaxM, float depthScale,
                     std::shared_ptr<MjpgDecoderRes> mjpgRes);

    std::atomic<uint64_t> frameCount{0};

    void enqueueNioFrameSet(std::shared_ptr<NioFrameSet> frameSet);
    void enqueueColor(const uint8_t *data, uint32_t size, uint64_t timestampUs);
    void enqueueDepth(const uint8_t *data, uint32_t size, uint64_t timestampUs,
                      float depthScale = 1.0f);

protected:
    void processFrame(const FrameBlob &blob) override;
    void onIdle() override;
    void onIdleHwD2C();
    void onIdleSwD2C();

private:
    void doBlend(const uint8_t *colorData, uint32_t colorSize, uint64_t colorTs,
                 const uint8_t *depthData, uint32_t depthSize, uint64_t depthTs,
                 float depthScale);

    int colorW_;
    int colorH_;
    NioFormat colorFormat_;
    std::shared_ptr<H264Encoder> fusedEncoder_;
    std::shared_ptr<std::ofstream> fusedFile_;
    std::mutex &fusedMtx_;
    std::shared_ptr<NioD2CAlign> alignFilter_;
    bool hwD2CMode_;

    float alpha_;
    float depthMinM_;
    float depthMaxM_;
    float depthScale_;

    std::shared_ptr<MjpgDecoderRes> mjpgRes_;
    std::shared_ptr<std::vector<uint8_t>> colorRGBBuf_;
    std::shared_ptr<std::vector<uint8_t>> fusedRGBBuf_;

    struct FrameBuf {
        std::vector<uint8_t> data;
        uint32_t size = 0;
        uint64_t timestampUs = 0;
        float depthScale = 1.0f;
    };

    std::mutex colorMtx_;
    FrameBuf latestColor_;
    std::atomic<bool> colorReady_{false};

    std::mutex depthMtx_;
    FrameBuf latestDepth_;
    std::atomic<bool> depthReady_{false};

    std::mutex frameSetMtx_;
    std::shared_ptr<NioFrameSet> latestFrameSet_;
    std::atomic<bool> frameSetReady_{false};
};

class ImuStreamTask : public StreamTask {
public:
    ImuStreamTask(const std::string &name, std::shared_ptr<std::ofstream> imuFile);

    void enqueueLine(std::string line);

protected:
    void processFrame(const FrameBlob &blob) override;

private:
    std::shared_ptr<std::ofstream> imuFile_;
    std::mutex fileMtx_;
};

class PcdStreamTask : public StreamTask {
public:
    PcdStreamTask(const std::string &name, const std::string &outputDir, const std::string &baseName);
    std::atomic<uint64_t> frameCount{0};

protected:
    void processFrame(const FrameBlob &blob) override;

private:
    std::string outputDir_;
    std::string baseName_;
    std::mutex fileMtx_;
    uint64_t frameIdx_ = 0;
};

} // namespace nio
