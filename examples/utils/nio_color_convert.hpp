// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert.hpp — Color conversion utilities: MJPEG decoding,
// jet colormap, RGB quadrant compositing, 5×7 bitmap text rendering.

#pragma once

#include <cstdint>
#include <memory>

#include <libobsensor/ObSensor.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace nio {

struct MjpgDecoderRes {
  AVCodecContext *ctx;
  AVPacket *pkt;
  AVFrame *decFrame;
  SwsContext *sws;
  AVPixelFormat decFmt;
  bool swsInitialized;

  MjpgDecoderRes();
  ~MjpgDecoderRes();

  bool init(int w, int h, OBFormat fmt);
};

void jetColormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b);

bool decodeColorToRGB(const uint8_t *data, uint32_t size, OBFormat format,
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
