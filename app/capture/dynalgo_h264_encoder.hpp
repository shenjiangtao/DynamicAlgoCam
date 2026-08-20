// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_h264_encoder.hpp — Unified H.264 encoder wrapping FFmpeg libavcodec.
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

#include "dynalgo_types.hpp"

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace dynalgo {
// [类说明 / Class Description]
// 中文: H.264编码器封装，集成FFmpeg x264编码器和可选MJPEG解码器。支持RGB/BGR/Y16/YUYV/MJPEG等输入格式
// English: Unified H.264 encoder wrapping FFmpeg libavcodec + optional MJPEG decoder. Supports RGB, BGR, Y16, YUYV, MJPEG input
// Usage: init() -> encode()* -> close(). Thread-unsafe — one instance per stream.
class H264Encoder
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数
    // English: Constructor
    H264Encoder();
    // [方法说明 / Method Description]
    // 中文: 析构函数，释放编码器资源
    // English: Destructor, releases encoder resources
    ~H264Encoder();

    // [方法说明 / Method Description]
    // 中文: 初始化编码器和SWS上下文。MJPEG格式的SWS延迟到首次解码时创建
    // English: Initialize encoder and sws context for given DynalgoFormat. For MJPEG, sws is deferred to first decodeMjpg() call
    bool init(int width, int height, int fps, DynalgoFormat srcFormat, int bitRate = 4000000,
              const char* seiUuid = "jiangtao.shen@ad");

    // [方法说明 / Method Description]
    // 中文: RGB格式便捷初始化（不创建MJPEG解码器）
    // English: RGB format convenience wrapper (no MJPEG decoder created)
    bool initRGB(int width, int height, int fps, int bitRate = 4000000, const char* seiUuid = "jiangtao.shen@ad");

    // [方法说明 / Method Description]
    // 中文: BGR格式便捷初始化（不创建MJPEG解码器）
    // English: BGR format convenience wrapper (no MJPEG decoder created)
    bool initBGR(int width, int height, int fps, int bitRate = 4000000, const char* seiUuid = "jiangtao.shen@ad");

    // [方法说明 / Method Description]
    // 中文: 关闭编码器，释放所有资源
    // English: Close encoder, release all resources
    void close();

    // [方法说明 / Method Description]
    // 中文: 将原始帧数据编码为H.264并写入文件。MJPEG输入需先通过decodeMjpg解码
    // English: Convert raw frame data to H.264 and write NALs to outFile. For MJPEG input, decodes JPEG first via decodeMjpg()
    bool encode(const uint8_t* data, uint32_t size, std::ofstream& outFile, std::mutex& mtx,
                uint64_t deviceTimestampUs = 0, bool writeSEI = true);

    // [方法说明 / Method Description]
    // 中文: 从RGB打包数据编码
    // English: Encode from packed RGB data
    bool encodeRGB(const uint8_t* rgbData, std::ofstream& outFile, std::mutex& mtx, uint64_t deviceTimestampUs);

    // [方法说明 / Method Description]
    // 中文: 从BGR打包数据编码
    // English: Encode from packed BGR data
    bool encodeBGR(const uint8_t* bgrData, std::ofstream& outFile, std::mutex& mtx, uint64_t deviceTimestampUs);

    int getWidth() const {
        return width_;
    }
    int getHeight() const {
        return height_;
    }
    bool isInitialized() const {
        return initialized_;
    }

private:
    // [方法说明 / Method Description]
    // 中文: 解码MJPEG字节为YUV420P帧（延迟SWS初始化）
    // English: Decode MJPEG bytes to YUV420P frame (lazy sws init)
    AVFrame* decodeMjpg(const uint8_t* data, uint32_t size);
    // [方法说明 / Method Description]
    // 中文: 初始化MJPEG专用SWS转换器
    // English: Initialize MJPEG-specific sws converter
    bool initMjpgSws(AVPixelFormat decFmt);
    // [方法说明 / Method Description]
    // 中文: 初始化核心编码器参数
    // English: Initialize core encoder parameters
    bool initEncoder(int width, int height, int fps, int bitRate);
    // [方法说明 / Method Description]
    // 中文: 初始化编码器可复用帧结构
    // English: Initialize reusable encoder frame structure
    bool initEncoderFrame(int width, int height);
    // [方法说明 / Method Description]
    // 中文: 配置编码器VUI参数
    // English: Configure encoder VUI parameters
    void setupEncoderVui();
    // [方法说明 / Method Description]
    // 中文: 初始化SWS上下文（源格式转YUV420P）
    // English: Initialize sws context (src format -> YUV420P)
    bool initSws(AVPixelFormat srcFmt, int width, int height);
    // [方法说明 / Method Description]
    // 中文: 将DynalgoFormat映射为FFmpeg AVPixelFormat
    // English: Map DynalgoFormat to FFmpeg AVPixelFormat
    AVPixelFormat mapDynalgoFormatToAV(DynalgoFormat srcFormat);
    // [方法说明 / Method Description]
    // 中文: 初始化MJPEG解码器
    // English: Initialize MJPEG decoder
    void initMjpgDecoder(int width, int height);
    // [方法说明 / Method Description]
    // 中文: 将编码后的帧写入文件（含SEI版权UUID）
    // English: Write encoded frame to file (with SEI copyright UUID)
    bool writeFrame(std::ofstream& outFile, std::mutex& mtx, uint64_t deviceTimestampUs, bool writeSEI);
    // [方法说明 / Method Description]
    // 中文: SWS像素格式转换
    // English: SWS pixel format conversion
    bool swsConvertFrame(const uint8_t* data, uint32_t size);
    // [方法说明 / Method Description]
    // 中文: 计算源数据stride
    // English: Compute source strides
    void computeSrcStrides(int srcStride[4]);
    // [方法说明 / Method Description]
    // 中文: 计算源数据slice指针
    // English: Compute source slice pointers
    void computeSrcSlices(const uint8_t* data, const uint8_t* srcSlice[4]);

    // --- H.264 encoder state ---
    AVCodecContext* codecCtx_; // x264 encoder context
    AVFrame* frame_;           // reusable YUV420P frame for encoding
    AVPacket* pkt_;            // reusable packet for encoder output
    SwsContext* swsCtx_;       // pixel format converter (src -> YUV420P)
    int64_t pts_;              // monotonic presentation timestamp counter
    int width_, height_;
    DynalgoFormat srcFormat_;
    bool initialized_;
    bool seiWritten_; // true after first copyright SEI is written
    std::string seiUuid_;

    // --- MJPEG decoder state (only used for DynalgoFormat::MJPG/MJPEG) ---
    const AVCodec* mjpgCodec_; // MJPEG decoder codec
    AVPixelFormat mjpgDecFmt_; // actual decoder output format (lazy)
    bool mjpgSwsInitialized_;  // true after first-frame sws creation
    AVCodecContext* mjpgCtx_;  // MJPEG decoder context
    AVPacket* mjpgPkt_;        // reusable packet for MJPEG decode
    AVFrame* mjpgDecFrame_;    // reusable frame for MJPEG decode output
};

} // namespace dynalgo
