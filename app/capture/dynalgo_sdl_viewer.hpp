// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_sdl_viewer.hpp — SDL2-based live preview for multi-device capture.
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

#include "dynalgo_types.hpp"
#include <utility>
#include <SDL2/SDL.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include "dynalgo_color_convert.hpp"
#include "dynalgo_thread.hpp"

namespace dynalgo {

enum class ViewerChannel {
    COLOR,
    DEPTH,
    IR,
    IR_LEFT,
    IR_RIGHT,
    POINT
};

struct ViewerSlot
{
    std::string label;
    std::string formatStr;
    DynalgoFormat format = DynalgoFormat::UNKNOWN;
    int w = 0;
    int h = 0;

    // Raw frame buffer (written by pushFrame, read by decodeThread)
    std::vector<uint8_t> rawBuf;
    uint32_t rawSize = 0;
    float depthScale = 1.0f; // raw→mm (same as DepthFrame::getValueScale)
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    std::mutex rawMtx;
    std::atomic<bool> rawUpdated{ false };

    // RGB render buffer (written by decodeThread, read by renderLoop)
    std::vector<uint8_t> renderBuf;
    std::mutex renderMtx;
    std::atomic<bool> renderUpdated{ false };

    // MJPG decoder
    std::shared_ptr<MjpgDecoderRes> mjpgRes;

    // YUYV sws state
    SwsContext* yuyvSws = nullptr;
    AVFrame* yuyvSrcFrame = nullptr;
    AVFrame* yuyvDstFrame = nullptr;
    bool yuyvSwsInit = false;

    // Per-slot decode worker — replaces the prior single global decode
    // thread.  Each slot has its own thread so sws_scale/jet/point decode
    // paths run in parallel across slots (was serialized before).  The
    // worker waits on decodeCv until pushFrame signals, then calls
    // SDLViewer::decodeSlot(*this).
    std::mutex decodeMtx;
    std::condition_variable decodeCv;
    std::atomic<bool> decodeQueued{ false };
    std::thread decodeThread;
};

class SDLViewer
{
public:
    SDLViewer();
    ~SDLViewer();

    bool init();
    void close();

    int addDevice(const std::string& name, const std::string& cameraType, const std::string& serialNumber,
                  bool hasColor, DynalgoFormat colorFmt, int cw, int ch, bool hasDepth, DynalgoFormat depthFmt, int dw, int dh,
                  bool hasIR, int irw, int irh, bool hasIRLeft, int ilw, int ilh, bool hasIRRight, int irw2, int irh2,
                  bool hasPoint = false, int pw = 640, int ph = 480);

    int addViewerSlot(const std::string& label, DynalgoFormat fmt, int w, int h);
    // Compute slot display size (preserving aspect) to fit within slotW x slotH bounds
    std::pair<int, int> computeSlotDisplaySize(int slotIdx, int slotW, int slotH) const;
    int addPointSlot(const std::string& label, int w, int h);

    bool createWindow();

    void pushFrame(int devIdx, ViewerChannel ch, const uint8_t* data, uint32_t size, float depthScale = 1.0f,
                   float depthMinM = 0.3f, float depthMaxM = 5.0f);

    struct LabelTex
    {
        SDL_Texture* tex = nullptr;
        int w = 0;
        int h = 0;
    };

private:
    void slotDecodeLoop(ViewerSlot* slot);
    void renderLoop();
    void renderDeviceRow(int di, int rowY, float scale, int colW, int winW);
    void renderSlotLabel(int slotIdx, int xOff, int videoY, int dstW, int dstH, float scale);
    void decodeSlot(ViewerSlot& slot);
    bool decodePointSlot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                         std::vector<uint8_t>& rgb);
    bool decodeY16Slot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                       std::vector<uint8_t>& rgb);
    bool decodeY8Slot(const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h, std::vector<uint8_t>& rgb);
    bool decodeYuyvSlot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                        std::vector<uint8_t>& rgb);
    bool decodeMjpgSlot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                        std::vector<uint8_t>& rgb);
    void cleanupSlot(ViewerSlot& slot);
    static std::string dynalgoFormatToString(DynalgoFormat fmt);

    void rebuildLabelTextures(int winW, int winH);
    void destroyLabelTextures();
    void makeLabelTex(LabelTex& out, const std::string& text, uint8_t fgR, uint8_t fgG, uint8_t fgB, uint8_t bgR,
                      uint8_t bgG, uint8_t bgB, int fscale);

    struct DeviceRow
    {
        std::string name;
        std::string cameraType;
        std::string serialNumber;
        std::vector<int> slotIndices;
        int colorSlot = -1;
        int depthSlot = -1;
        int irSlot = -1;
        int irLeftSlot = -1;
        int irRightSlot = -1;
        int pointSlot = -1;
    };

    std::vector<DeviceRow> devices_;
    std::vector<std::unique_ptr<ViewerSlot>> slots_;

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    std::vector<SDL_Texture*> textures_; // video textures (per slot)

    // Cached label textures (rebuilt when window size changes)
    std::vector<LabelTex> titleTexs_;   // per-device: "型号_序列号"
    std::vector<LabelTex> formatTexs_;  // per-slot: "MJPG"/"Y16"/"Y8"
    std::vector<LabelTex> channelTexs_; // per-slot: "Color"/"Depth"/etc.
    int cachedWinW_ = 0;
    int cachedWinH_ = 0;

    std::thread renderThread_;
    std::atomic<bool> running_{ false };
    bool initialized_ = false;

    int tileW_ = 0;
    int tileH_ = 0;
    int maxSlotsPerRow_ = 0;
    int targetWinW_ = 1080;
    int targetWinH_ = 1920;

    static const int RENDER_FPS = 30;
    static const int ROW_HEADER_H = 36;
    static const int FORMAT_BAR_H = 26;
    static const int FONT_SCALE = 3;
};

} // namespace dynalgo
