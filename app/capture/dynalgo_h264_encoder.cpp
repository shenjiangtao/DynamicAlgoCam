// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_h264_encoder.cpp — H264Encoder implementation using FFmpeg x264.
//
// Encoding pipeline per input format:
//   RGB/BGR/RGBA/BGRA/YUYV/UYVY/NV12/NV21/I420/Y16/Y8:
//     raw pixels → sws_scale (srcFmt → YUV420P) → avcodec_encode → H.264 NAL
//   MJPEG:
//     JPEG bytes → mjpgCtx_ decode → YUVJ422P (or similar) frame →
//     sws_scale (actual decoder fmt → YUV420P) → avcodec_encode → H.264 NAL
//
// Critical color-space fix (2025-06):
//   MJPEG decoder may output YUVJ422P (4:2:2 full range) instead of the
//   requested YUV420P (4:2:0).  If sws_getContext is created with the wrong
//   source format, sws_scale reads U/V planes with 4:2:0 stride from 4:2:2
//   data, causing chroma misalignment and completely wrong colors.
//   Fix: sws context is lazily created in decodeMjpg() using the actual
//   decoder output format from the first decoded frame.

#include "dynalgo_h264_encoder.hpp"
#include "dynalgo_common.hpp"
#include "dynalgo_log.hpp"

#include <cstring>
#include <iostream>
#include <sstream>

namespace dynalgo {

H264Encoder::H264Encoder()
: codecCtx_(nullptr)
, frame_(nullptr)
, pkt_(nullptr)
, swsCtx_(nullptr)
, pts_(0)
, width_(0)
, height_(0)
, srcFormat_(DynalgoFormat::UNKNOWN)
, initialized_(false)
, seiWritten_(false)
, seiUuid_("jiangtao.shen@ad")
, mjpgCodec_(nullptr)
, mjpgDecFmt_(AV_PIX_FMT_NONE)
, mjpgSwsInitialized_(false)
, mjpgCtx_(nullptr)
, mjpgPkt_(nullptr)
, mjpgDecFrame_(nullptr) {}

H264Encoder::~H264Encoder() {
    close();
}

// Set VUI color-space signals on codecCtx_: BT.709 full range
// 设置VUI色彩空间信号：BT.709全范围，避免解码后颜色偏白
void H264Encoder::setupEncoderVui() {
    codecCtx_->color_range = AVCOL_RANGE_JPEG;
    codecCtx_->color_primaries = AVCOL_PRI_BT709;
    codecCtx_->color_trc = AVCOL_TRC_BT709;
    codecCtx_->colorspace = AVCOL_SPC_BT709;
}

// Allocate AVFrame + AVPacket for the encoder
// 为编码器分配AVFrame + AVPacket
bool H264Encoder::initEncoderFrame(int width, int height) {
    frame_ = av_frame_alloc();
    if (!frame_) {
        close();
        return false;
    }
    frame_->format = AV_PIX_FMT_YUV420P;
    frame_->width = width;
    frame_->height = height;
    if (av_frame_get_buffer(frame_, 0) < 0) {
        close();
        return false;
    }

    pkt_ = av_packet_alloc();
    if (!pkt_) {
        close();
        return false;
    }
    return true;
}

// Create x264 AVCodecContext, configure params, allocate frame/packet
// 创建x264编码器上下文，配置参数，分配帧/包
bool H264Encoder::initEncoder(int width, int height, int fps, int bitRate) {
    width_ = width;
    height_ = height;

    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        DYNALGO_LOG_ERROR_S("H264 encoder not found, " << width << "x" << height);
        return false;
    }

    codecCtx_ = avcodec_alloc_context3(codec);
    if (!codecCtx_)
        return false;

    codecCtx_->bit_rate = bitRate;
    codecCtx_->width = width;
    codecCtx_->height = height;
    codecCtx_->time_base = { 1, fps };
    codecCtx_->framerate = { fps, 1 };
    codecCtx_->gop_size = fps;
    codecCtx_->max_b_frames = 0;
    codecCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
    codecCtx_->qmin = 10;
    codecCtx_->qmax = 30;
    av_opt_set(codecCtx_->priv_data, "preset", "ultrafast", 0);
    av_opt_set(codecCtx_->priv_data, "tune", "zerolatency", 0);
    setupEncoderVui();

    if (avcodec_open2(codecCtx_, codec, nullptr) < 0) {
        DYNALGO_LOG_ERROR_S("Failed to open H264 encoder, " << width << "x" << height);
        avcodec_free_context(&codecCtx_);
        codecCtx_ = nullptr;
        return false;
    }

    return initEncoderFrame(width, height);
}

// --- initSws: create SwsContext with BT.709 full-range color-space details ---
// Used by initRGB/initBGR paths.  srcRange=1 (full range) for RGB input,
// dstRange=1 (full range) for YUV420P output to encoder.
bool H264Encoder::initSws(AVPixelFormat srcFmt, int width, int height) {
    AVPixelFormat dstFmt = AV_PIX_FMT_YUV420P;
    if (srcFmt != dstFmt) {
        swsCtx_ = sws_getContext(width, height, srcFmt, width, height, dstFmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!swsCtx_) {
            DYNALGO_LOG_ERROR_S("Failed to create sws context for H264 encoder");
            close();
            return false;
        }
        const int* srcCoeffs = sws_getCoefficients(SWS_CS_ITU709);
        const int* dstCoeffs = sws_getCoefficients(SWS_CS_ITU709);
        // sws_setColorspaceDetails: 8-arg signature
        // (ctx, srcCoeffs, srcRange, dstCoeffs, dstRange, brightness, contrast, saturation)
        // brightness=0, contrast=1<<16 (1.0), saturation=1<<16 (1.0)
        sws_setColorspaceDetails(swsCtx_, srcCoeffs, 1, dstCoeffs, 1, 0, 1 << 16, 1 << 16);
    }
    return true;
}

// Map DynalgoFormat to AVPixelFormat for sws_getContext
// 将DynalgoFormat映射为FFmpeg AVPixelFormat
AVPixelFormat H264Encoder::mapDynalgoFormatToAV(DynalgoFormat srcFormat) {
    switch (srcFormat) {
    case DynalgoFormat::YUYV:
        return AV_PIX_FMT_YUYV422;
    case DynalgoFormat::UYVY:
        return AV_PIX_FMT_UYVY422;
    case DynalgoFormat::RGB:
    case DynalgoFormat::RGB888:
        return AV_PIX_FMT_RGB24;
    case DynalgoFormat::BGR:
        return AV_PIX_FMT_BGR24;
    case DynalgoFormat::RGBA:
        return AV_PIX_FMT_RGBA;
    case DynalgoFormat::BGRA:
        return AV_PIX_FMT_BGRA;
    case DynalgoFormat::NV12:
        return AV_PIX_FMT_NV12;
    case DynalgoFormat::NV21:
        return AV_PIX_FMT_NV21;
    case DynalgoFormat::Y16:
        return AV_PIX_FMT_GRAY16LE;
    case DynalgoFormat::Y8:
        return AV_PIX_FMT_GRAY8;
    case DynalgoFormat::I420:
        return AV_PIX_FMT_YUV420P;
    case DynalgoFormat::MJPG:
    case DynalgoFormat::MJPEG:
        return AV_PIX_FMT_YUV420P;
    default:
        return AV_PIX_FMT_NONE;
    }
}

// Pre-create MJPEG decoder context (pix_fmt hint only, actual format determined at decode)
// 预创建MJPEG解码器上下文（pix_fmt仅为提示，实际格式在解码时确定）
void H264Encoder::initMjpgDecoder(int width, int height) {
    mjpgCodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (mjpgCodec_) {
        mjpgCtx_ = avcodec_alloc_context3(mjpgCodec_);
        if (mjpgCtx_) {
            mjpgCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
            mjpgCtx_->width = width;
            mjpgCtx_->height = height;
            if (avcodec_open2(mjpgCtx_, mjpgCodec_, nullptr) < 0) {
                avcodec_free_context(&mjpgCtx_);
                mjpgCtx_ = nullptr;
            }
        }
    }
    mjpgPkt_ = av_packet_alloc();
    mjpgDecFrame_ = av_frame_alloc();
}

// init: main entry point — create encoder + sws for the given DynalgoFormat
// init：主入口 — 根据DynalgoFormat创建编码器 + sws转换上下文
bool H264Encoder::init(int width, int height, int fps, DynalgoFormat srcFormat, int bitRate, const char* seiUuid) {
    srcFormat_ = srcFormat;
    seiUuid_ = seiUuid ? seiUuid : "jiangtao.shen@ad";

    if (!initEncoder(width, height, fps, bitRate))
        return false;

    AVPixelFormat srcFmt = mapDynalgoFormatToAV(srcFormat);
    if (srcFmt == AV_PIX_FMT_NONE) {
        DYNALGO_LOG_ERROR_S("Unsupported format for H264 encoding: " << dynalgoFormatToStr(srcFormat) << " " << width << "x"
                                                                 << height);
        close();
        return false;
    }

    // Defer sws creation for MJPEG: actual decoder output format unknown until first frame
    bool isMjpeg = (srcFormat == DynalgoFormat::MJPG || srcFormat == DynalgoFormat::MJPEG);
    AVPixelFormat swsSrcFmt = isMjpeg ? AV_PIX_FMT_NONE : srcFmt;

    AVPixelFormat dstFmt = AV_PIX_FMT_YUV420P;
    if (swsSrcFmt != AV_PIX_FMT_NONE && swsSrcFmt != dstFmt) {
        swsCtx_ =
            sws_getContext(width, height, swsSrcFmt, width, height, dstFmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!swsCtx_) {
            DYNALGO_LOG_ERROR_S("Failed to create sws context for H264 encoder, format=" << dynalgoFormatToStr(srcFormat));
            close();
            return false;
        }
        const int* srcCoeffs = sws_getCoefficients(SWS_CS_ITU709);
        const int* dstCoeffs = sws_getCoefficients(SWS_CS_ITU709);
        sws_setColorspaceDetails(swsCtx_, srcCoeffs, 1, dstCoeffs, 1, 0, 1 << 16, 1 << 16);
    }

    initMjpgDecoder(width, height);
    initialized_ = true;
    return true;
}

// --- initRGB / initBGR: convenience wrappers for RGB/BGR-only encoding ---
bool H264Encoder::initRGB(int width, int height, int fps, int bitRate, const char* seiUuid) {
    srcFormat_ = DynalgoFormat::RGB;
    seiUuid_ = seiUuid ? seiUuid : "jiangtao.shen@ad";

    if (!initEncoder(width, height, fps, bitRate))
        return false;
    if (!initSws(AV_PIX_FMT_RGB24, width, height))
        return false;

    initialized_ = true;
    return true;
}

bool H264Encoder::initBGR(int width, int height, int fps, int bitRate, const char* seiUuid) {
    srcFormat_ = DynalgoFormat::BGR;
    seiUuid_ = seiUuid ? seiUuid : "jiangtao.shen@ad";

    if (!initEncoder(width, height, fps, bitRate))
        return false;
    if (!initSws(AV_PIX_FMT_BGR24, width, height))
        return false;

    initialized_ = true;
    return true;
}

// --- close: release all FFmpeg resources in reverse creation order ---
void H264Encoder::close() {
    if (swsCtx_) {
        sws_freeContext(swsCtx_);
        swsCtx_ = nullptr;
    }
    if (frame_) {
        av_frame_free(&frame_);
    }
    if (pkt_) {
        av_packet_free(&pkt_);
    }
    if (codecCtx_) {
        avcodec_free_context(&codecCtx_);
    }
    if (mjpgDecFrame_) {
        av_frame_free(&mjpgDecFrame_);
    }
    if (mjpgPkt_) {
        av_packet_free(&mjpgPkt_);
    }
    if (mjpgCtx_) {
        avcodec_free_context(&mjpgCtx_);
    }
    mjpgCodec_ = nullptr;
    initialized_ = false;
}

// Lazily create sws context for MJPEG decoder output format on first frame
// 首帧时根据MJPEG解码器实际输出格式延迟创建sws上下文
bool H264Encoder::initMjpgSws(AVPixelFormat decFmt) {
    const char* pixFmtName = av_get_pix_fmt_name(decFmt);
    DYNALGO_LOG_INFO_S("[MJPEG_DEC_FORMAT_CHECK] H264Encoder::decodeMjpg"
                   << " decoder_output_fmt=" << (pixFmtName ? pixFmtName : "unknown") << " (" << decFmt << ")"
                   << " color_range=" << mjpgDecFrame_->color_range << " w=" << mjpgDecFrame_->width
                   << " h=" << mjpgDecFrame_->height << " stride_Y=" << mjpgDecFrame_->linesize[0]
                   << " stride_U=" << mjpgDecFrame_->linesize[1] << " stride_V=" << mjpgDecFrame_->linesize[2]);
    if (mjpgDecFrame_->data[0]) {
        const uint8_t* y0 = mjpgDecFrame_->data[0];
        const uint8_t* u0 = mjpgDecFrame_->data[1];
        const uint8_t* v0 = mjpgDecFrame_->data[2];
        DYNALGO_LOG_INFO_S("[MJPEG_DEC_FORMAT_CHECK] H264Encoder sample Y[0..4]={"
                       << (int)y0[0] << "," << (int)y0[1] << "," << (int)y0[2] << "," << (int)y0[3] << "} U[0..4]={"
                       << (int)u0[0] << "," << (int)u0[1] << "," << (int)u0[2] << "," << (int)u0[3] << "} V[0..4]={"
                       << (int)v0[0] << "," << (int)v0[1] << "," << (int)v0[2] << "," << (int)v0[3] << "}");
    }

    if (swsCtx_) {
        sws_freeContext(swsCtx_);
        swsCtx_ = nullptr;
    }
    mjpgDecFmt_ = decFmt;
    swsCtx_ = sws_getContext(width_, height_, decFmt, width_, height_, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr,
                             nullptr, nullptr);
    if (!swsCtx_) {
        DYNALGO_LOG_ERROR_S("Failed to create sws context for MJPEG, srcFmt=" << (pixFmtName ? pixFmtName : "unknown"));
        return false;
    }
    const int* srcCoeffs = sws_getCoefficients(SWS_CS_ITU709);
    const int* dstCoeffs = sws_getCoefficients(SWS_CS_ITU709);
    int srcRange = (mjpgDecFrame_->color_range == AVCOL_RANGE_JPEG) ? 1 : 0;
    sws_setColorspaceDetails(swsCtx_, srcCoeffs, srcRange, dstCoeffs, 1, 0, 1 << 16, 1 << 16);
    DYNALGO_LOG_INFO_S("[MJPEG_DEC_FORMAT_CHECK] sws recreated: srcFmt=" << (pixFmtName ? pixFmtName : "unknown")
                                                                     << " dstFmt=YUV420P srcRange=" << srcRange
                                                                     << " dstRange=1 cs=BT709");
    mjpgSwsInitialized_ = true;
    return true;
}

// decodeMjpg: decode MJPEG bytes → YUV420P frame with lazy sws init
// decodeMjpg：解码MJPEG字节 → YUV420P帧，延迟初始化sws
AVFrame* H264Encoder::decodeMjpg(const uint8_t* data, uint32_t size) {
    if (!mjpgCtx_)
        return nullptr;

    mjpgPkt_->data = const_cast<uint8_t*>(data);
    mjpgPkt_->size = size;

    int ret = avcodec_send_packet(mjpgCtx_, mjpgPkt_);
    if (ret < 0)
        return nullptr;

    ret = avcodec_receive_frame(mjpgCtx_, mjpgDecFrame_);
    if (ret < 0)
        return nullptr;

    AVPixelFormat decFmt = static_cast<AVPixelFormat>(mjpgDecFrame_->format);

    if (!mjpgSwsInitialized_) {
        if (!initMjpgSws(decFmt))
            return nullptr;
    }

    if (!swsCtx_)
        return nullptr;

    if (av_frame_make_writable(frame_) < 0)
        return nullptr;
    sws_scale(swsCtx_, mjpgDecFrame_->data, mjpgDecFrame_->linesize, 0, mjpgDecFrame_->height, frame_->data,
              frame_->linesize);

    static bool postSwsLogged = false;
    if (!postSwsLogged && frame_->data[0] && frame_->data[1] && frame_->data[2]) {
        const uint8_t* yOut = frame_->data[0];
        const uint8_t* uOut = frame_->data[1];
        const uint8_t* vOut = frame_->data[2];
        DYNALGO_LOG_INFO_S("[MJPEG_DEC_FORMAT_CHECK] H264Encoder post-sws_scale"
                       << " Y[0..4]={" << (int)yOut[0] << "," << (int)yOut[1] << "," << (int)yOut[2] << ","
                       << (int)yOut[3] << "} U[0..4]={" << (int)uOut[0] << "," << (int)uOut[1] << "," << (int)uOut[2]
                       << "," << (int)uOut[3] << "} V[0..4]={" << (int)vOut[0] << "," << (int)vOut[1] << ","
                       << (int)vOut[2] << "," << (int)vOut[3] << "}");
        postSwsLogged = true;
    }

    return frame_;
}

// --- writeFrame: drain encoded packets from encoder, write to file with SEI ---
bool H264Encoder::writeFrame(std::ofstream& outFile, std::mutex& mtx, uint64_t deviceTimestampUs, bool writeSEI) {
    if (writeSEI) {
        if (!seiWritten_) {
            dynalgo::writeSEINalUnit(outFile, SEI_COPYRIGHT, mtx, seiUuid_.c_str());
            seiWritten_ = true;
        }
        std::ostringstream tsStr;
        tsStr << "dts=" << deviceTimestampUs;
        dynalgo::writeSEINalUnit(outFile, tsStr.str(), mtx, seiUuid_.c_str());
    }

    bool wrote = false;
    int ret = 0;
    while (ret >= 0) {
        ret = avcodec_receive_packet(codecCtx_, pkt_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        if (ret < 0)
            break;

        {
            std::lock_guard<std::mutex> lock(mtx);
            outFile.write(reinterpret_cast<const char*>(pkt_->data), pkt_->size);
        }
        wrote = true;
        av_packet_unref(pkt_);
    }
    return wrote;
}

// Compute source strides based on srcFormat_ for sws_scale
// 根据srcFormat_计算sws_scale所需的源步幅
void H264Encoder::computeSrcStrides(int srcStride[4]) {
    switch (srcFormat_) {
    case DynalgoFormat::YUYV:
    case DynalgoFormat::UYVY:
    case DynalgoFormat::YUY2:
    case DynalgoFormat::Y16:
        srcStride[0] = width_ * 2;
        break;
    case DynalgoFormat::RGB:
    case DynalgoFormat::BGR:
    case DynalgoFormat::RGB888:
        srcStride[0] = width_ * 3;
        break;
    case DynalgoFormat::RGBA:
    case DynalgoFormat::BGRA:
        srcStride[0] = width_ * 4;
        break;
    case DynalgoFormat::Y8:
        srcStride[0] = width_;
        break;
    case DynalgoFormat::I420:
        srcStride[0] = width_;
        srcStride[1] = width_ / 2;
        srcStride[2] = width_ / 2;
        break;
    case DynalgoFormat::NV12:
    case DynalgoFormat::NV21:
        srcStride[0] = width_;
        srcStride[1] = width_;
        break;
    default:
        break;
    }
}

// Compute source slice pointers for I420/NV12/NV21 or single-plane formats
// 计算I420/NV12/NV21或多平面格式的源切片指针
void H264Encoder::computeSrcSlices(const uint8_t* data, const uint8_t* srcSlice[4]) {
    srcSlice[0] = data;
    srcSlice[1] = nullptr;
    srcSlice[2] = nullptr;
    srcSlice[3] = nullptr;
    if (srcFormat_ == DynalgoFormat::I420) {
        srcSlice[0] = data;
        srcSlice[1] = data + width_ * height_;
        srcSlice[2] = data + width_ * height_ * 5 / 4;
    } else if (srcFormat_ == DynalgoFormat::NV12 || srcFormat_ == DynalgoFormat::NV21) {
        srcSlice[0] = data;
        srcSlice[1] = data + width_ * height_;
    }
}

// Convert raw frame data to YUV420P via sws_scale (non-MJPEG paths)
// 通过sws_scale将原始帧数据转为YUV420P（非MJPEG路径）
bool H264Encoder::swsConvertFrame(const uint8_t* data, uint32_t /*size*/) {
    if (swsCtx_) {
        int srcStride[4] = { 0, 0, 0, 0 };
        computeSrcStrides(srcStride);

        const uint8_t* srcSlice[4] = { nullptr, nullptr, nullptr, nullptr };
        computeSrcSlices(data, srcSlice);

        if (av_frame_make_writable(frame_) < 0)
            return false;
        sws_scale(swsCtx_, srcSlice, srcStride, 0, height_, frame_->data, frame_->linesize);
    } else {
        if (av_frame_make_writable(frame_) < 0)
            return false;
        if (frame_->linesize[0] == width_) {
            memcpy(frame_->data[0], data, static_cast<size_t>(width_) * height_);
        } else {
            for (int i = 0; i < height_; i++)
                memcpy(frame_->data[0] + i * frame_->linesize[0], data + i * width_, width_);
        }
        int uvWidth = width_ / 2;
        int uvHeight = height_ / 2;
        int uvPlaneSize = uvWidth * uvHeight;
        if (frame_->linesize[1] == uvWidth && frame_->linesize[2] == uvWidth) {
            memcpy(frame_->data[1], data + width_ * height_, uvPlaneSize);
            memcpy(frame_->data[2], data + width_ * height_ + uvPlaneSize, uvPlaneSize);
        } else {
            for (int i = 0; i < uvHeight; i++) {
                memcpy(frame_->data[1] + i * frame_->linesize[1], data + width_ * height_ + i * uvWidth, uvWidth);
                memcpy(frame_->data[2] + i * frame_->linesize[2], data + width_ * height_ + uvPlaneSize + i * uvWidth,
                       uvWidth);
            }
        }
    }
    return true;
}

// encode: top-level entry — decode MJPEG or sws_convert, then send to H264 encoder
// encode：顶层入口 — 解码MJPEG或sws_convert，然后发送至H264编码器
bool H264Encoder::encode(const uint8_t* data, uint32_t size, std::ofstream& outFile, std::mutex& mtx,
                         uint64_t deviceTimestampUs, bool writeSEI) {
    if (!initialized_ || !codecCtx_)
        return false;

    AVFrame* srcFrame = nullptr;

    if (srcFormat_ == DynalgoFormat::MJPG || srcFormat_ == DynalgoFormat::MJPEG) {
        srcFrame = decodeMjpg(data, size);
        if (!srcFrame)
            return false;
    } else {
        if (!swsConvertFrame(data, size))
            return false;
        srcFrame = frame_;
    }

    srcFrame->pts = pts_++;

    int ret = avcodec_send_frame(codecCtx_, srcFrame);
    if (ret < 0) {
        if (srcFrame != frame_)
            av_frame_free(&srcFrame);
        return false;
    }

    bool wrote = writeFrame(outFile, mtx, deviceTimestampUs, writeSEI);

    if (srcFrame != frame_)
        av_frame_free(&srcFrame);
    return wrote;
}

// --- encodeRGB / encodeBGR: direct RGB/BGR encode (sws already created) ---
bool H264Encoder::encodeRGB(const uint8_t* rgbData, std::ofstream& outFile, std::mutex& mtx,
                            uint64_t deviceTimestampUs) {
    if (!initialized_ || !codecCtx_ || !swsCtx_)
        return false;

    int srcStride = width_ * 3;
    const uint8_t* srcSlice[1] = { rgbData };
    if (av_frame_make_writable(frame_) < 0)
        return false;
    sws_scale(swsCtx_, srcSlice, &srcStride, 0, height_, frame_->data, frame_->linesize);

    frame_->pts = pts_++;

    int ret = avcodec_send_frame(codecCtx_, frame_);
    if (ret < 0)
        return false;

    return writeFrame(outFile, mtx, deviceTimestampUs, true);
}

bool H264Encoder::encodeBGR(const uint8_t* bgrData, std::ofstream& outFile, std::mutex& mtx,
                            uint64_t deviceTimestampUs) {
    if (!initialized_ || !codecCtx_ || !swsCtx_)
        return false;

    int srcStride = width_ * 3;
    const uint8_t* srcSlice[1] = { bgrData };
    if (av_frame_make_writable(frame_) < 0)
        return false;
    sws_scale(swsCtx_, srcSlice, &srcStride, 0, height_, frame_->data, frame_->linesize);

    frame_->pts = pts_++;

    int ret = avcodec_send_frame(codecCtx_, frame_);
    if (ret < 0)
        return false;

    return writeFrame(outFile, mtx, deviceTimestampUs, true);
}

} // namespace dynalgo
