// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert.hpp — Color conversion utilities: MJPEG decoding,
// jet colormap, RGB quadrant compositing, 5×7 bitmap text rendering.
//
// MjpgDecoderRes: RAII wrapper for FFmpeg MJPEG decoder + sws context.
//   - sws context is lazily created on first decoded frame (see .cpp for
//     the YUVJ422P vs YUV420P format mismatch fix).
//   - decFmt / swsInitialized track the lazy init state.
//
// decodeColorToRGB(): converts any supported NioFormat frame to RGB24.
//   Used by the D2C fusion preview to compose color + depth + IR into
//   a single RGB canvas.
//
// fillQuadrant / fillQuadrantJetDepth: composite sub-images into a 2×2
//   quadrant layout (used for multi-sensor preview display).
//
// drawChar5x7 / drawText5x7: tiny bitmap font overlay for timestamp/sensor
//   labels drawn directly onto RGB pixel buffers.

#pragma once

#include "nio_types.hpp"

#include <cstdint>
#include <memory>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace nio {

// MjpgDecoderRes: owns MJPEG decoder + sws context for YUV→RGB conversion.
// Sws is NOT created in init() — it is lazily created in decodeColorToRGB()
// after the first frame is decoded, because the actual decoder output format
// (e.g. YUVJ422P) may differ from the requested YUV420P.
struct MjpgDecoderRes {
    AVCodecContext *ctx;         // MJPEG decoder context
    AVPacket *pkt;               // reuse packet for each decode call
    AVFrame *decFrame;           // reuse decoded frame buffer
    SwsContext *sws;             // lazily created YUV→RGB sws context
    AVPixelFormat decFmt;        // actual decoder output pixel format
    bool swsInitialized;         // true after first-frame lazy init

  MjpgDecoderRes();
  ~MjpgDecoderRes();

  bool init(int w, int h, NioFormat fmt);
};

void jetColormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b);

bool decodeColorToRGB(const uint8_t *data, uint32_t size, NioFormat format,
                      int w, int h, uint8_t *rgbBuf,
                      std::shared_ptr<MjpgDecoderRes> mjpg);

void drawChar5x7(uint8_t *buf, int bufW, int bufH, int x0, int y0, char c,
                 uint8_t r, uint8_t g, uint8_t b);

void drawText5x7(uint8_t *buf, int bufW, int bufH, int x0, int y0,
                 const std::string &text, uint8_t r, uint8_t g, uint8_t b);

void fillQuadrant(uint8_t *outBuf, int outW, int outH,
                  int quadX, int quadY, int quadW, int quadH,
                  const uint8_t *srcRGB, int srcW, int srcH);

void fillQuadrantJetDepth(uint8_t *outBuf, int outW, int outH,
                           int quadX, int quadY, int quadW, int quadH,
                           const uint16_t *depthData, int depthW, int depthH,
                           float scale, float depthMinM, float depthMaxM);

} // namespace nio
