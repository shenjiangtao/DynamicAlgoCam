// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_sdl_viewer.hpp — SDL2-based live preview for multi-device capture.
//
// Layout per device row:
//   型号_序列号
//   ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
//   │Color │ │Depth │ │IR-L  │ │IR-R  │
//   │      │ │      │ │      │ │      │
//   └──────┘ └──────┘ └──────┘ └──────┘
//    MJPG      Y16      Y8       Y8
//
// Threading model (zero impact on SDK recording):
// - pushFrame(): SDK callback → memcpy raw data into slot.rawBuf (~us)
// - decodeThread_(): converts rawBuf → RGB24 into slot.renderBuf (parallel)
// - renderLoop_(): uploads renderBuf to textures + draws text overlays (30fps)

#pragma once

#include <SDL2/SDL.h>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>
#include <vector>
#include <atomic>
#include <chrono>

#include <libobsensor/ObSensor.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include "nio_color_convert.hpp"

namespace nio {

enum class ViewerChannel { COLOR, DEPTH, IR, IR_LEFT, IR_RIGHT };

struct ViewerSlot {
    std::string label;
    std::string formatStr;
    OBFormat format = OB_FORMAT_UNKNOWN;
    int w = 0;
    int h = 0;

    // Raw frame buffer (written by pushFrame, read by decodeThread)
    std::vector<uint8_t> rawBuf;
    uint32_t rawSize = 0;
    float depthScale = 1.0f; // raw→mm (same as DepthFrame::getValueScale)
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    std::mutex rawMtx;
    std::atomic<bool> rawUpdated{false};

    // RGB render buffer (written by decodeThread, read by renderLoop)
    std::vector<uint8_t> renderBuf;
    std::mutex renderMtx;
    std::atomic<bool> renderUpdated{false};

    // MJPG decoder
    std::shared_ptr<MjpgDecoderRes> mjpgRes;

    // YUYV sws state
    SwsContext* yuyvSws = nullptr;
    AVFrame* yuyvSrcFrame = nullptr;
    AVFrame* yuyvDstFrame = nullptr;
    bool yuyvSwsInit = false;
};

class SDLViewer {
public:
    SDLViewer();
    ~SDLViewer();

    bool init();
    void close();

    int addDevice(const std::string& name,
                  const std::string& cameraType,
                  const std::string& serialNumber,
                  bool hasColor, OBFormat colorFmt, int cw, int ch,
                  bool hasDepth, OBFormat depthFmt, int dw, int dh,
                  bool hasIR, int irw, int irh,
                  bool hasIRLeft, int ilw, int ilh,
                  bool hasIRRight, int irw2, int irh2);

    bool createWindow();

    void pushFrame(int devIdx, ViewerChannel ch,
                   const uint8_t* data, uint32_t size,
                   float depthScale = 1.0f,
                   float depthMinM = 0.3f, float depthMaxM = 5.0f);

private:
    void decodeThreadFunc();
    void renderLoop();
    void decodeSlot(ViewerSlot& slot);
    void cleanupSlot(ViewerSlot& slot);
    static std::string obFormatToString(OBFormat fmt);

    void rebuildLabelTextures(int winW, int winH);

    struct DeviceRow {
        std::string name;
        std::string cameraType;
        std::string serialNumber;
        std::vector<int> slotIndices;
        int colorSlot = -1;
        int depthSlot = -1;
        int irSlot = -1;
        int irLeftSlot = -1;
        int irRightSlot = -1;
    };

    std::vector<DeviceRow> devices_;
    std::vector<std::unique_ptr<ViewerSlot>> slots_;

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    std::vector<SDL_Texture*> textures_;      // video textures (per slot)

    // Cached label textures (rebuilt when window size changes)
    struct LabelTex {
        SDL_Texture* tex = nullptr;
        int w = 0;
        int h = 0;
    };
    std::vector<LabelTex> titleTexs_;         // per-device: "型号_序列号"
    std::vector<LabelTex> formatTexs_;        // per-slot: "MJPG"/"Y16"/"Y8"
    std::vector<LabelTex> channelTexs_;       // per-slot: "Color"/"Depth"/etc.
    int cachedWinW_ = 0;
    int cachedWinH_ = 0;

    std::thread decodeThread_;
    std::thread renderThread_;
    std::atomic<bool> running_{false};
    bool initialized_ = false;

    int tileW_ = 0;
    int tileH_ = 0;
    int maxSlotsPerRow_ = 0;

    std::mutex decodeCvMtx_;
    std::condition_variable decodeCv_;
    std::atomic<bool> decodeWakeup_{false};

    static const int RENDER_FPS = 30;
    static const int ROW_HEADER_H = 36;
    static const int FORMAT_BAR_H = 26;
    static const int FONT_SCALE = 3;
};

} // namespace nio
