// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_h264_encoder.hpp — Unified H.264 encoder wrapping FFmpeg libavcodec.
// Supports RGB, BGR, Y16, YUYV, MJPEG and other input formats with configurable
// bit-rate and SEI copyright UUID.
//
// Key design decisions:
// - Encoder always outputs YUV420P (x264 ultrafast/zerolatency preset)
// - Color space: BT.709 full range (AVCOL_RANGE_JPEG) — ensures decoded
//   output matches the original camera colors without the limited-range
//   (16-235) luma squeeze that causes washed-out colors
// - For MJPEG input: sws context is lazily created on first decoded frame
//   because the MJPEG decoder may output YUVJ422P (4:2:2) rather than
//   YUV420P (4:2:0); creating sws with the wrong source format causes
//   chroma plane misalignment and completely broken colors

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
// H264Encoder: wraps FFmpeg x264 encoder + optional MJPEG decoder.
// Usage: init() → encode()* → close().  Thread-unsafe — one instance per stream.
class H264Encoder {
public:
    H264Encoder();
    ~H264Encoder();

    // init: create encoder + sws context for the given OBFormat.
    // For MJPEG, sws is deferred to first decodeMjpg() call.
    bool init(int width, int height, int fps, OBFormat srcFormat,
              int bitRate = 4000000, const char *seiUuid = "nio@orbbec-fusio");

    // initRGB / initBGR: convenience wrappers (no MJPEG decoder created)
    bool initRGB(int width, int height, int fps,
                 int bitRate = 4000000, const char *seiUuid = "nio@orbbec-fusio");

    bool initBGR(int width, int height, int fps,
                 int bitRate = 4000000, const char *seiUuid = "nio@orbbec-fusio");

    void close();

    // encode: convert raw frame data to H.264 and write NALs to outFile.
    // For MJPEG input, decodes JPEG first via decodeMjpg().
    bool encode(const uint8_t *data, uint32_t size,
                std::ofstream &outFile, std::mutex &mtx,
                uint64_t deviceTimestampUs = 0, bool writeSEI = true);

    // encodeRGB / encodeBGR: encode from packed RGB/BGR data
    bool encodeRGB(const uint8_t *rgbData, std::ofstream &outFile, std::mutex &mtx,
                   uint64_t deviceTimestampUs);

    bool encodeBGR(const uint8_t *bgrData, std::ofstream &outFile, std::mutex &mtx,
                   uint64_t deviceTimestampUs);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    bool isInitialized() const { return initialized_; }

private:
    // decodeMjpg: decode MJPEG bytes → YUV420P frame (lazy sws init)
    AVFrame *decodeMjpg(const uint8_t *data, uint32_t size);
    bool initMjpgSws(AVPixelFormat decFmt);
    bool initEncoder(int width, int height, int fps, int bitRate);
    bool initEncoderFrame(int width, int height);
    void setupEncoderVui();
    bool initSws(AVPixelFormat srcFmt, int width, int height);
    AVPixelFormat mapOBFormatToAV(OBFormat srcFormat);
    void initMjpgDecoder(int width, int height);
    bool writeFrame(std::ofstream &outFile, std::mutex &mtx,
                    uint64_t deviceTimestampUs, bool writeSEI);
    bool swsConvertFrame(const uint8_t *data, uint32_t size);
    void computeSrcStrides(int srcStride[4]);
    void computeSrcSlices(const uint8_t *data, const uint8_t *srcSlice[4]);

    // --- H.264 encoder state ---
    AVCodecContext *codecCtx_;      // x264 encoder context
    AVFrame *frame_;                // reusable YUV420P frame for encoding
    AVPacket *pkt_;                 // reusable packet for encoder output
    SwsContext *swsCtx_;            // pixel format converter (src → YUV420P)
    int64_t pts_;                   // monotonic presentation timestamp counter
    int width_, height_;
    OBFormat srcFormat_;
    bool initialized_;
    bool seiWritten_;               // true after first copyright SEI is written
    std::string seiUuid_;

    // --- MJPEG decoder state (only used for OB_FORMAT_MJPG/MJPEG) ---
    const AVCodec *mjpgCodec_;          // MJPEG decoder codec
    AVPixelFormat mjpgDecFmt_;          // actual decoder output format (lazy)
    bool mjpgSwsInitialized_;           // true after first-frame sws creation
    AVCodecContext *mjpgCtx_;           // MJPEG decoder context
    AVPacket *mjpgPkt_;                 // reusable packet for MJPEG decode
    AVFrame *mjpgDecFrame_;             // reusable frame for MJPEG decode output
};

} // namespace nio
