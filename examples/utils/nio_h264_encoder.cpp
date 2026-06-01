// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_h264_encoder.cpp — H264Encoder implementation using FFmpeg x264.

#include "nio_h264_encoder.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"

#include <iostream>
#include <sstream>
#include <cstring>

namespace nio {

H264Encoder::H264Encoder()
    : codecCtx_(nullptr), frame_(nullptr), pkt_(nullptr), swsCtx_(nullptr),
      pts_(0), width_(0), height_(0), srcFormat_(OB_FORMAT_UNKNOWN),
      initialized_(false), seiWritten_(false), seiUuid_("nio@orbbec-fusio"),
      mjpgCodec_(nullptr), mjpgCtx_(nullptr), mjpgPkt_(nullptr), mjpgDecFrame_(nullptr) {}

H264Encoder::~H264Encoder() { close(); }

bool H264Encoder::initEncoder(int width, int height, int fps, int bitRate) {
    width_ = width;
    height_ = height;

    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if(!codec) {
        std::cerr << "H264 encoder not found" << std::endl;
        NIO_LOG_ERROR_S("H264 encoder not found, " << width << "x" << height);
        return false;
    }

    codecCtx_ = avcodec_alloc_context3(codec);
    if(!codecCtx_) return false;

    codecCtx_->bit_rate = bitRate;
    codecCtx_->width = width;
    codecCtx_->height = height;
    AVRational tb = {1, fps};
    AVRational fr = {fps, 1};
    codecCtx_->time_base = tb;
    codecCtx_->framerate = fr;
    codecCtx_->gop_size = fps;
    codecCtx_->max_b_frames = 0;
    codecCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
    codecCtx_->qmin = 10;
    codecCtx_->qmax = 30;

    av_opt_set(codecCtx_->priv_data, "preset", "ultrafast", 0);
    av_opt_set(codecCtx_->priv_data, "tune", "zerolatency", 0);

    if(avcodec_open2(codecCtx_, codec, nullptr) < 0) {
        std::cerr << "Failed to open H264 encoder" << std::endl;
        NIO_LOG_ERROR_S("Failed to open H264 encoder, " << width << "x" << height);
        avcodec_free_context(&codecCtx_);
        codecCtx_ = nullptr;
        return false;
    }

    frame_ = av_frame_alloc();
    if(!frame_) { close(); return false; }
    frame_->format = AV_PIX_FMT_YUV420P;
    frame_->width = width;
    frame_->height = height;
    if(av_frame_get_buffer(frame_, 0) < 0) { close(); return false; }

    pkt_ = av_packet_alloc();
    if(!pkt_) { close(); return false; }

    return true;
}

bool H264Encoder::initSws(AVPixelFormat srcFmt, int width, int height) {
    AVPixelFormat dstFmt = AV_PIX_FMT_YUV420P;
    if(srcFmt != dstFmt) {
        swsCtx_ = sws_getContext(width, height, srcFmt,
                                 width, height, dstFmt,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!swsCtx_) {
            std::cerr << "Failed to create sws context" << std::endl;
            NIO_LOG_ERROR_S("Failed to create sws context for H264 encoder");
            close();
            return false;
        }
    }
    return true;
}

bool H264Encoder::init(int width, int height, int fps, OBFormat srcFormat,
                       int bitRate, const char *seiUuid) {
    srcFormat_ = srcFormat;
    seiUuid_ = seiUuid ? seiUuid : "nio@orbbec-fusio";

    if(!initEncoder(width, height, fps, bitRate)) return false;

    AVPixelFormat srcFmt = AV_PIX_FMT_NONE;
    switch(srcFormat) {
    case OB_FORMAT_YUYV: srcFmt = AV_PIX_FMT_YUYV422; break;
    case OB_FORMAT_UYVY: srcFmt = AV_PIX_FMT_UYVY422; break;
    case OB_FORMAT_RGB:  srcFmt = AV_PIX_FMT_RGB24; break;
    case OB_FORMAT_BGR:  srcFmt = AV_PIX_FMT_BGR24; break;
    case OB_FORMAT_RGBA: srcFmt = AV_PIX_FMT_RGBA; break;
    case OB_FORMAT_BGRA: srcFmt = AV_PIX_FMT_BGRA; break;
    case OB_FORMAT_NV12: srcFmt = AV_PIX_FMT_NV12; break;
    case OB_FORMAT_NV21: srcFmt = AV_PIX_FMT_NV21; break;
    case OB_FORMAT_Y16:  srcFmt = AV_PIX_FMT_GRAY16LE; break;
    case OB_FORMAT_Y8:   srcFmt = AV_PIX_FMT_GRAY8; break;
    case OB_FORMAT_I420: srcFmt = AV_PIX_FMT_YUV420P; break;
    case OB_FORMAT_MJPG: srcFmt = AV_PIX_FMT_YUV420P; break;
    default:
        std::cerr << "Unsupported format for H264 encoding: " << srcFormat << std::endl;
        NIO_LOG_ERROR_S("Unsupported format for H264 encoding: " << srcFormat << " " << width << "x" << height);
        close();
        return false;
    }

    AVPixelFormat swsSrcFmt = srcFmt;
    if(srcFormat == OB_FORMAT_MJPG || srcFormat == OB_FORMAT_MJPEG) {
        swsSrcFmt = AV_PIX_FMT_YUV420P;
    }

    AVPixelFormat dstFmt = AV_PIX_FMT_YUV420P;
    if(swsSrcFmt != dstFmt || srcFormat == OB_FORMAT_MJPG || srcFormat == OB_FORMAT_MJPEG) {
        swsCtx_ = sws_getContext(width, height, swsSrcFmt,
                                 width, height, dstFmt,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!swsCtx_) {
            std::cerr << "Failed to create sws context" << std::endl;
            NIO_LOG_ERROR_S("Failed to create sws context for H264 encoder, format=" << srcFormat);
            close();
            return false;
        }
    }

    mjpgCodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if(mjpgCodec_) {
        mjpgCtx_ = avcodec_alloc_context3(mjpgCodec_);
        if(mjpgCtx_) {
            mjpgCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
            mjpgCtx_->width = width;
            mjpgCtx_->height = height;
            if(avcodec_open2(mjpgCtx_, mjpgCodec_, nullptr) < 0) {
                avcodec_free_context(&mjpgCtx_);
                mjpgCtx_ = nullptr;
            }
        }
    }
    mjpgPkt_ = av_packet_alloc();
    mjpgDecFrame_ = av_frame_alloc();

    initialized_ = true;
    return true;
}

bool H264Encoder::initRGB(int width, int height, int fps, int bitRate, const char *seiUuid) {
    srcFormat_ = OB_FORMAT_RGB;
    seiUuid_ = seiUuid ? seiUuid : "nio@orbbec-fusio";

    if(!initEncoder(width, height, fps, bitRate)) return false;
    if(!initSws(AV_PIX_FMT_RGB24, width, height)) return false;

    initialized_ = true;
    return true;
}

bool H264Encoder::initBGR(int width, int height, int fps, int bitRate, const char *seiUuid) {
    srcFormat_ = OB_FORMAT_BGR;
    seiUuid_ = seiUuid ? seiUuid : "nio@orbbec-fusio";

    if(!initEncoder(width, height, fps, bitRate)) return false;
    if(!initSws(AV_PIX_FMT_BGR24, width, height)) return false;

    initialized_ = true;
    return true;
}

void H264Encoder::close() {
    if(swsCtx_) { sws_freeContext(swsCtx_); swsCtx_ = nullptr; }
    if(frame_) { av_frame_free(&frame_); }
    if(pkt_) { av_packet_free(&pkt_); }
    if(codecCtx_) { avcodec_free_context(&codecCtx_); }
    if(mjpgDecFrame_) { av_frame_free(&mjpgDecFrame_); }
    if(mjpgPkt_) { av_packet_free(&mjpgPkt_); }
    if(mjpgCtx_) { avcodec_free_context(&mjpgCtx_); }
    mjpgCodec_ = nullptr;
    initialized_ = false;
}

AVFrame *H264Encoder::decodeMjpg(const uint8_t *data, uint32_t size) {
    if(!mjpgCtx_) return nullptr;

    mjpgPkt_->data = const_cast<uint8_t *>(data);
    mjpgPkt_->size = size;

    int ret = avcodec_send_packet(mjpgCtx_, mjpgPkt_);
    if(ret < 0) return nullptr;

    ret = avcodec_receive_frame(mjpgCtx_, mjpgDecFrame_);
    if(ret < 0) return nullptr;

    if(!swsCtx_) return nullptr;

    if(av_frame_make_writable(frame_) < 0) return nullptr;
    sws_scale(swsCtx_, mjpgDecFrame_->data, mjpgDecFrame_->linesize, 0,
              mjpgDecFrame_->height, frame_->data, frame_->linesize);
    return frame_;
}

bool H264Encoder::writeFrame(std::ofstream &outFile, std::mutex &mtx,
                              uint64_t frameTimestampMs, bool writeSEI) {
    if(writeSEI) {
        if(!seiWritten_) {
            nio::writeSEINalUnit(outFile, SEI_COPYRIGHT, mtx, seiUuid_.c_str());
            seiWritten_ = true;
        }
        std::ostringstream tsStr;
        tsStr << "ts=" << frameTimestampMs;
        nio::writeSEINalUnit(outFile, tsStr.str(), mtx, seiUuid_.c_str());
    }

    bool wrote = false;
    int ret = 0;
    while(ret >= 0) {
        ret = avcodec_receive_packet(codecCtx_, pkt_);
        if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
        if(ret < 0) break;

        {
            std::lock_guard<std::mutex> lock(mtx);
            outFile.write(reinterpret_cast<const char *>(pkt_->data), pkt_->size);
            outFile.flush();
        }
        wrote = true;
        av_packet_unref(pkt_);
    }
    return wrote;
}

bool H264Encoder::encode(const uint8_t *data, uint32_t size,
                          std::ofstream &outFile, std::mutex &mtx,
                          uint64_t frameTimestampMs, bool writeSEI) {
    if(!initialized_ || !codecCtx_) return false;

    AVFrame *srcFrame = nullptr;

    if(srcFormat_ == OB_FORMAT_MJPG || srcFormat_ == OB_FORMAT_MJPEG) {
        srcFrame = decodeMjpg(data, size);
        if(!srcFrame) return false;
    } else {
        if(swsCtx_) {
            int srcStride[4] = {0, 0, 0, 0};
            switch(srcFormat_) {
            case OB_FORMAT_YUYV:
            case OB_FORMAT_UYVY: srcStride[0] = width_ * 2; break;
            case OB_FORMAT_RGB:
            case OB_FORMAT_BGR: srcStride[0] = width_ * 3; break;
            case OB_FORMAT_RGBA:
            case OB_FORMAT_BGRA: srcStride[0] = width_ * 4; break;
            case OB_FORMAT_Y16: srcStride[0] = width_ * 2; break;
            case OB_FORMAT_Y8: srcStride[0] = width_; break;
            case OB_FORMAT_I420:
                srcStride[0] = width_;
                srcStride[1] = width_ / 2;
                srcStride[2] = width_ / 2;
                break;
            case OB_FORMAT_NV12:
            case OB_FORMAT_NV21:
                srcStride[0] = width_;
                srcStride[1] = width_;
                break;
            default: break;
            }

            const uint8_t *srcSlice[4] = { data, nullptr, nullptr, nullptr };
            if(srcFormat_ == OB_FORMAT_I420) {
                srcSlice[0] = data;
                srcSlice[1] = data + width_ * height_;
                srcSlice[2] = data + width_ * height_ * 5 / 4;
            } else if(srcFormat_ == OB_FORMAT_NV12 || srcFormat_ == OB_FORMAT_NV21) {
                srcSlice[0] = data;
                srcSlice[1] = data + width_ * height_;
            }

            if(av_frame_make_writable(frame_) < 0) return false;
            sws_scale(swsCtx_, srcSlice, srcStride, 0, height_,
                      frame_->data, frame_->linesize);
        } else {
            if(av_frame_make_writable(frame_) < 0) return false;
            for(int i = 0; i < height_; i++)
                memcpy(frame_->data[0] + i * frame_->linesize[0],
                       data + i * width_, width_);
            for(int i = 0; i < height_ / 2; i++) {
                memcpy(frame_->data[1] + i * frame_->linesize[1],
                       data + width_ * height_ + i * width_ / 2, width_ / 2);
                memcpy(frame_->data[2] + i * frame_->linesize[2],
                       data + width_ * height_ * 5 / 4 + i * width_ / 2, width_ / 2);
            }
        }
        srcFrame = frame_;
    }

    srcFrame->pts = pts_++;

    int ret = avcodec_send_frame(codecCtx_, srcFrame);
    if(ret < 0) {
        if(srcFrame != frame_) av_frame_free(&srcFrame);
        return false;
    }

    bool wrote = writeFrame(outFile, mtx, frameTimestampMs, writeSEI);

    if(srcFrame != frame_) av_frame_free(&srcFrame);
    return wrote;
}

bool H264Encoder::encodeRGB(const uint8_t *rgbData, std::ofstream &outFile, std::mutex &mtx,
                             uint64_t frameTimestampMs) {
    if(!initialized_ || !codecCtx_ || !swsCtx_) return false;

    int srcStride = width_ * 3;
    const uint8_t *srcSlice[1] = { rgbData };
    if(av_frame_make_writable(frame_) < 0) return false;
    sws_scale(swsCtx_, srcSlice, &srcStride, 0, height_,
              frame_->data, frame_->linesize);

    frame_->pts = pts_++;

    int ret = avcodec_send_frame(codecCtx_, frame_);
    if(ret < 0) return false;

    return writeFrame(outFile, mtx, frameTimestampMs, true);
}

bool H264Encoder::encodeBGR(const uint8_t *bgrData, std::ofstream &outFile, std::mutex &mtx,
                             uint64_t frameTimestampMs) {
    if(!initialized_ || !codecCtx_ || !swsCtx_) return false;

    int srcStride = width_ * 3;
    const uint8_t *srcSlice[1] = { bgrData };
    if(av_frame_make_writable(frame_) < 0) return false;
    sws_scale(swsCtx_, srcSlice, &srcStride, 0, height_,
              frame_->data, frame_->linesize);

    frame_->pts = pts_++;

    int ret = avcodec_send_frame(codecCtx_, frame_);
    if(ret < 0) return false;

    return writeFrame(outFile, mtx, frameTimestampMs, true);
}

} // namespace nio
