// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert.cpp — Color conversion and text rendering implementation.

#include "nio_color_convert.hpp"

#include <cstring>
#include <cmath>
#include <algorithm>

extern "C" {
#include <libavutil/imgutils.h>
}

namespace nio {

MjpgDecoderRes::MjpgDecoderRes()
    : ctx(nullptr), pkt(nullptr), decFrame(nullptr), sws(nullptr) {}

MjpgDecoderRes::~MjpgDecoderRes() {
    if(sws) sws_freeContext(sws);
    if(decFrame) av_frame_free(&decFrame);
    if(pkt) av_packet_free(&pkt);
    if(ctx) avcodec_free_context(&ctx);
}

bool MjpgDecoderRes::init(int w, int h, OBFormat fmt) {
    pkt = av_packet_alloc();
    decFrame = av_frame_alloc();
    if(fmt == OB_FORMAT_MJPG || fmt == OB_FORMAT_MJPEG) {
        auto codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
        if(codec) {
            ctx = avcodec_alloc_context3(codec);
            if(ctx) {
                ctx->pix_fmt = AV_PIX_FMT_YUV420P;
                ctx->width = w;
                ctx->height = h;
                if(avcodec_open2(ctx, codec, nullptr) < 0) {
                    avcodec_free_context(&ctx);
                    ctx = nullptr;
                }
            }
        }
        sws = sws_getContext(w, h, AV_PIX_FMT_YUV420P,
                             w, h, AV_PIX_FMT_RGB24,
                             SWS_BILINEAR, nullptr, nullptr, nullptr);
    }
    return true;
}

void jetColormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
    float t = v / 255.0f;
    float rv, gv, bv;
    if(t < 0.125f) { rv = 0.0f; gv = 0.0f; bv = 0.5f + t * 4.0f; }
    else if(t < 0.375f) { rv = 0.0f; gv = (t - 0.125f) * 4.0f; bv = 1.0f; }
    else if(t < 0.625f) { rv = (t - 0.375f) * 4.0f; gv = 1.0f; bv = 1.0f - (t - 0.375f) * 4.0f; }
    else if(t < 0.875f) { rv = 1.0f; gv = 1.0f - (t - 0.625f) * 4.0f; bv = 0.0f; }
    else { rv = 1.0f - (t - 0.875f) * 4.0f; gv = 0.0f; bv = 0.0f; }
    r = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, rv * 255.0f)));
    g = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, gv * 255.0f)));
    b = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bv * 255.0f)));
}

bool decodeColorToRGB(const uint8_t *data, uint32_t size, OBFormat format,
                      int w, int h, uint8_t *rgbBuf,
                      std::shared_ptr<MjpgDecoderRes> mjpg) {
    if(format == OB_FORMAT_RGB) {
        memcpy(rgbBuf, data, w * h * 3);
        return true;
    }
    if(format == OB_FORMAT_BGR) {
        for(int i = 0; i < w * h; i++) {
            rgbBuf[i * 3 + 0] = data[i * 3 + 2];
            rgbBuf[i * 3 + 1] = data[i * 3 + 1];
            rgbBuf[i * 3 + 2] = data[i * 3 + 0];
        }
        return true;
    }
    if(format == OB_FORMAT_RGBA) {
        for(int i = 0; i < w * h; i++) {
            rgbBuf[i * 3 + 0] = data[i * 4 + 0];
            rgbBuf[i * 3 + 1] = data[i * 4 + 1];
            rgbBuf[i * 3 + 2] = data[i * 4 + 2];
        }
        return true;
    }
    if(format == OB_FORMAT_BGRA) {
        for(int i = 0; i < w * h; i++) {
            rgbBuf[i * 3 + 0] = data[i * 4 + 2];
            rgbBuf[i * 3 + 1] = data[i * 4 + 1];
            rgbBuf[i * 3 + 2] = data[i * 4 + 0];
        }
        return true;
    }
    if(format == OB_FORMAT_MJPG || format == OB_FORMAT_MJPEG) {
        if(!mjpg->ctx || !mjpg->sws) return false;
        mjpg->pkt->data = const_cast<uint8_t *>(data);
        mjpg->pkt->size = size;
        int ret = avcodec_send_packet(mjpg->ctx, mjpg->pkt);
        if(ret < 0) return false;
        ret = avcodec_receive_frame(mjpg->ctx, mjpg->decFrame);
        if(ret < 0) return false;

        AVFrame *tmpFrame = av_frame_alloc();
        if(!tmpFrame) return false;
        tmpFrame->format = AV_PIX_FMT_RGB24;
        tmpFrame->width = w;
        tmpFrame->height = h;
        if(av_frame_get_buffer(tmpFrame, 0) < 0) { av_frame_free(&tmpFrame); return false; }

        sws_scale(mjpg->sws, mjpg->decFrame->data, mjpg->decFrame->linesize,
                  0, h, tmpFrame->data, tmpFrame->linesize);

        for(int row = 0; row < h; row++) {
            memcpy(rgbBuf + row * w * 3,
                   tmpFrame->data[0] + row * tmpFrame->linesize[0], w * 3);
        }
        av_frame_free(&tmpFrame);
        return true;
    }
    if(format == OB_FORMAT_YUYV || format == OB_FORMAT_UYVY ||
       format == OB_FORMAT_NV12 || format == OB_FORMAT_NV21 ||
       format == OB_FORMAT_I420) {
        AVPixelFormat srcPixFmt = AV_PIX_FMT_NONE;
        if(format == OB_FORMAT_YUYV) srcPixFmt = AV_PIX_FMT_YUYV422;
        else if(format == OB_FORMAT_UYVY) srcPixFmt = AV_PIX_FMT_UYVY422;
        else if(format == OB_FORMAT_NV12) srcPixFmt = AV_PIX_FMT_NV12;
        else if(format == OB_FORMAT_NV21) srcPixFmt = AV_PIX_FMT_NV21;
        else if(format == OB_FORMAT_I420) srcPixFmt = AV_PIX_FMT_YUV420P;

        SwsContext *tmpSws = sws_getContext(w, h, srcPixFmt,
                                            w, h, AV_PIX_FMT_RGB24,
                                            SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!tmpSws) return false;

        AVFrame *tmpFrame = av_frame_alloc();
        if(!tmpFrame) { sws_freeContext(tmpSws); return false; }
        tmpFrame->format = AV_PIX_FMT_RGB24;
        tmpFrame->width = w;
        tmpFrame->height = h;
        if(av_frame_get_buffer(tmpFrame, 0) < 0) {
            av_frame_free(&tmpFrame);
            sws_freeContext(tmpSws);
            return false;
        }

        int srcStride[4] = {0, 0, 0, 0};
        const uint8_t *srcSlice[4] = { data, nullptr, nullptr, nullptr };

        if(format == OB_FORMAT_YUYV || format == OB_FORMAT_UYVY) {
            srcStride[0] = w * 2;
        } else if(format == OB_FORMAT_NV12 || format == OB_FORMAT_NV21) {
            srcStride[0] = w;
            srcStride[1] = w;
            srcSlice[1] = data + w * h;
        } else if(format == OB_FORMAT_I420) {
            srcStride[0] = w;
            srcStride[1] = w / 2;
            srcStride[2] = w / 2;
            srcSlice[0] = data;
            srcSlice[1] = data + w * h;
            srcSlice[2] = data + w * h * 5 / 4;
        }

        sws_scale(tmpSws, srcSlice, srcStride, 0, h,
                  tmpFrame->data, tmpFrame->linesize);

        for(int row = 0; row < h; row++) {
            memcpy(rgbBuf + row * w * 3,
                   tmpFrame->data[0] + row * tmpFrame->linesize[0], w * 3);
        }
        sws_freeContext(tmpSws);
        av_frame_free(&tmpFrame);
        return true;
    }
    return false;
}

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x03,0x00,0x03,0x00},
    {0x14,0x3E,0x14,0x3E,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},
    {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},
    {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08},
    {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},
    {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},
    {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},
    {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41},
    {0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00},
    {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x09,0x09,0x09,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x01,0x01},
    {0x3E,0x41,0x41,0x49,0x3A},
    {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x09,0x19,0x66},
    {0x26,0x49,0x49,0x49,0x32},
    {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},
    {0x7F,0x20,0x10,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43},
};

void drawChar5x7(uint8_t *buf, int bufW, int bufH, int x0, int y0, char c,
                 uint8_t r, uint8_t g, uint8_t b) {
    int idx = c - ' ';
    if(idx < 0 || idx >= static_cast<int>(sizeof(font5x7) / sizeof(font5x7[0]))) return;
    const uint8_t *glyph = font5x7[idx];
    for(int col = 0; col < 5; col++) {
        uint8_t colBits = glyph[col];
        for(int row = 0; row < 7; row++) {
            if(colBits & (1 << row)) {
                int px = x0 + col;
                int py = y0 + row;
                if(px >= 0 && px < bufW && py >= 0 && py < bufH) {
                    int i = (py * bufW + px) * 3;
                    buf[i + 0] = r;
                    buf[i + 1] = g;
                    buf[i + 2] = b;
                }
            }
        }
    }
}

void drawText5x7(uint8_t *buf, int bufW, int bufH, int x0, int y0,
                 const std::string &text, uint8_t r, uint8_t g, uint8_t b) {
    int cx = x0;
    for(size_t i = 0; i < text.size(); i++) {
        drawChar5x7(buf, bufW, bufH, cx, y0, text[i], r, g, b);
        cx += 6;
    }
}

void fillQuadrant(uint8_t *outBuf, int outW, int outH,
                  int quadX, int quadY, int quadW, int quadH,
                  const uint8_t *srcRGB, int srcW, int srcH) {
    for(int y = 0; y < quadH; y++) {
        int srcY = y * srcH / quadH;
        if(srcY >= srcH) srcY = srcH - 1;
        for(int x = 0; x < quadW; x++) {
            int srcX = x * srcW / quadW;
            if(srcX >= srcW) srcX = srcW - 1;
            int ox = quadX + x;
            int oy = quadY + y;
            if(ox >= 0 && ox < outW && oy >= 0 && oy < outH) {
                int di = (oy * outW + ox) * 3;
                int si = (srcY * srcW + srcX) * 3;
                outBuf[di + 0] = srcRGB[si + 0];
                outBuf[di + 1] = srcRGB[si + 1];
                outBuf[di + 2] = srcRGB[si + 2];
            }
        }
    }
}

void fillQuadrantJetDepth(uint8_t *outBuf, int outW, int outH,
                           int quadX, int quadY, int quadW, int quadH,
                           const uint16_t *depthData, int depthW, int depthH,
                           float scale, float depthMinM, float depthMaxM) {
    float rangeM = depthMaxM - depthMinM;
    if(rangeM <= 0.0f) rangeM = 1.0f;
    for(int y = 0; y < quadH; y++) {
        int srcY = y * depthH / quadH;
        if(srcY >= depthH) srcY = depthH - 1;
        for(int x = 0; x < quadW; x++) {
            int srcX = x * depthW / quadW;
            if(srcX >= depthW) srcX = depthW - 1;
            int ox = quadX + x;
            int oy = quadY + y;
            if(ox >= 0 && ox < outW && oy >= 0 && oy < outH) {
                int di = (oy * outW + ox) * 3;
                uint16_t rawVal = depthData[srcY * depthW + srcX];
                if(rawVal == 0) {
                    outBuf[di + 0] = 0;
                    outBuf[di + 1] = 0;
                    outBuf[di + 2] = 0;
                } else {
                    float meters = rawVal * scale * 0.001f;
                    float norm = (meters - depthMinM) / rangeM;
                    if(norm < 0.0f) norm = 0.0f;
                    if(norm > 1.0f) norm = 1.0f;
                    uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                    jetColormap(v, outBuf[di + 0], outBuf[di + 1], outBuf[di + 2]);
                }
            }
        }
    }
}

} // namespace nio
