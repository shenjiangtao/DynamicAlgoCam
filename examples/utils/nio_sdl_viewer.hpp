// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_sdl_viewer.hpp — SDL2-based live preview for multi-device capture.
//
// Displays color/depth/IR streams from all Orbbec devices in a single window.
// Layout: one row per device, channels tiled horizontally within each row.
//
// Format handling:
// - Depth Y16 → jet colormap (same palette as D2C fusion)
// - IR Y8 → grayscale (R=G=B=Y8)
// - Color YUYV → RGB24 via libswscale (YUYV422→RGB24, preserves 4:2:2)
// - Color MJPG → RGB24 via decodeColorToRGB (JPEG decode + sws)
//
// Threading model:
// - pushFrame() is called from the OrbbecSDK callback thread (one per device)
// - renderLoop() runs on a dedicated thread at RENDER_FPS (default 30 fps)
// - Per-slot mutex + updated flag: pushFrame converts to RGB under lock,
//   renderLoop only uploads texture when updated==true (avoids redundant work)

#pragma once

#include <SDL2/SDL.h>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>
#include <vector>
#include <atomic>

#include <libobsensor/ObSensor.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include "nio_color_convert.hpp"

namespace nio {

// ViewerChannel: identifies which sensor slot to push a frame into.
// IR is for Gemini 305 (single IR sensor); IR_LEFT/IR_RIGHT for 335L/336L.
enum class ViewerChannel { COLOR, DEPTH, IR, IR_LEFT, IR_RIGHT };

// ViewerSlot: holds one channel's preview state (e.g. "Gemini_305 Color").
// Stored as unique_ptr in SDLViewer because std::mutex is non-movable.
struct ViewerSlot {
    std::string label;             // display label (e.g. "Gemini_305 Depth")
    OBFormat format = OB_FORMAT_UNKNOWN; // original pixel format from device
    int w = 0;                     // frame width in pixels
    int h = 0;                     // frame height in pixels
    std::vector<uint8_t> rgbBuf;   // converted RGB24 buffer (w*h*3 bytes)
    std::mutex mtx;                // protects rgbBuf + updated during push/render
    bool updated = false;          // true after pushFrame, cleared by renderLoop

    // MJPG decoder resources (only allocated for OB_FORMAT_MJPG slots)
    std::shared_ptr<MjpgDecoderRes> mjpgRes;

    // YUYV→RGB conversion state (lazily initialized on first pushFrame)
    SwsContext* yuyvSws = nullptr;     // sws context: YUYV422 → RGB24
    AVFrame* yuyvSrcFrame = nullptr;   // source frame for sws_scale input
    AVFrame* yuyvDstFrame = nullptr;   // destination frame for sws_scale output
    bool yuyvSwsInit = false;          // true after first-frame sws creation
};

// SDLViewer: multi-device live preview window.
// Usage: init() → addDevice()* → pushFrame()* (from callback) → close()
// The render thread starts automatically when the first device is added.
class SDLViewer {
public:
    SDLViewer();
    ~SDLViewer();

    // init: initialize SDL2 video subsystem. Safe to call even if no
    // display is available (e.g. SSH) — returns false in that case.
    bool init();

    // close: stop render thread, destroy SDL resources. Called by destructor.
    void close();

    // addDevice: register a device and its available sensor channels.
    // Returns device index (>=0) for use in pushFrame(), or -1 on failure.
    // Creates the SDL window + render thread on first call.
    int addDevice(const std::string& name,
                  bool hasColor, OBFormat colorFmt, int cw, int ch,
                  bool hasDepth, int dw, int dh,
                  bool hasIR, int irw, int irh,
                  bool hasIRLeft, int ilw, int ilh,
                  bool hasIRRight, int irw2, int irh2);

    // pushFrame: convert a raw sensor frame to RGB and store in the slot.
    // Called from OrbbecSDK callback — must be fast (no encoding, just conversion).
    // depthScale/depthMinM/depthMaxM are only used for Y16 depth → jet colormap.
    void pushFrame(int devIdx, ViewerChannel ch,
                   const uint8_t* data, uint32_t size,
                   float depthScale = 0.001f,
                   float depthMinM = 0.3f, float depthMaxM = 5.0f);

private:
    // renderLoop: main loop of the render thread. Polls SDL events,
    // uploads updated textures, and renders all slots at RENDER_FPS.
    void renderLoop();

    // Format conversion helpers (called from pushFrame under slot mutex):
    void y16ToJetRGB(const uint16_t* y16, int w, int h,
                     uint8_t* rgb, float scale, float minM, float maxM);
    void y8ToRGB(const uint8_t* y8, int w, int h, uint8_t* rgb);
    bool yuyvToRGB(const uint8_t* yuyv, int w, int h, uint8_t* rgb, ViewerSlot& slot);
    bool mjpgToRGB(const uint8_t* data, uint32_t size, int w, int h,
                   uint8_t* rgb, std::shared_ptr<MjpgDecoderRes> mjpg);

    // DeviceRow: one row in the viewer layout. Maps channel types to slot indices.
    struct DeviceRow {
        std::string name;
        std::vector<int> slotIndices; // all slot indices for this row (left→right)
        int colorSlot = -1;           // index into slots_ (-1 if no color sensor)
        int depthSlot = -1;
        int irSlot = -1;              // Gemini 305 single IR
        int irLeftSlot = -1;          // Gemini 335L/336L IR left
        int irRightSlot = -1;         // Gemini 335L/336L IR right
    };

    std::vector<DeviceRow> devices_;
    std::vector<std::unique_ptr<ViewerSlot>> slots_; // unique_ptr because mutex is non-movable

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    std::vector<SDL_Texture*> textures_; // one per slot, RGB24 STREAMING

    std::thread renderThread_;         // independent render thread
    std::atomic<bool> running_{false}; // false → render thread exits

    int tileW_ = 0;           // max slot width (used for scaling)
    int tileH_ = 0;           // max slot height (used for scaling)
    int maxSlotsPerRow_ = 0;  // max channels across all devices (for column count)

    static const int RENDER_FPS = 30; // render thread frame rate
};

} // namespace nio
