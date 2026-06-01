// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_h264_encoder.hpp — Unified H.264 encoder wrapping FFmpeg libavcodec.
// Supports RGB, BGR, Y16, and MJPEG input formats with configurable
// bit-rate and SEI copyright UUID.

#pragma once

#include <fstream>
#include <mutex>
#include <cstdint>
#include <string>

#include <libobsensor/ObSensor.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace nio {

class H264Encoder {
public:
    H264Encoder();
    ~H264Encoder();

    bool init(int width, int height, int fps, OBFormat srcFormat,
              int bitRate = 4000000, const char *seiUuid = "nio@orbbec-fusio");

    bool initRGB(int width, int height, int fps,
                 int bitRate = 4000000, const char *seiUuid = "nio@orbbec-fusio");

    bool initBGR(int width, int height, int fps,
                 int bitRate = 4000000, const char *seiUuid = "nio@orbbec-fusio");

    void close();

    bool encode(const uint8_t *data, uint32_t size,
                std::ofstream &outFile, std::mutex &mtx,
                uint64_t deviceTimestampUs = 0, bool writeSEI = true);

    bool encodeRGB(const uint8_t *rgbData, std::ofstream &outFile, std::mutex &mtx,
                   uint64_t deviceTimestampUs);

    bool encodeBGR(const uint8_t *bgrData, std::ofstream &outFile, std::mutex &mtx,
                   uint64_t deviceTimestampUs);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    bool isInitialized() const { return initialized_; }

private:
    AVFrame *decodeMjpg(const uint8_t *data, uint32_t size);
    bool initEncoder(int width, int height, int fps, int bitRate);
    bool initSws(AVPixelFormat srcFmt, int width, int height);
    bool writeFrame(std::ofstream &outFile, std::mutex &mtx, uint64_t deviceTimestampUs, bool writeSEI);

    AVCodecContext *codecCtx_;
    AVFrame *frame_;
    AVPacket *pkt_;
    SwsContext *swsCtx_;
    int64_t pts_;
    int width_, height_;
    OBFormat srcFormat_;
    bool initialized_;
    bool seiWritten_;
    std::string seiUuid_;

    const AVCodec *mjpgCodec_;
    AVCodecContext *mjpgCtx_;
    AVPacket *mjpgPkt_;
    AVFrame *mjpgDecFrame_;
};

} // namespace nio
