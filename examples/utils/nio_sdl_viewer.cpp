// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_sdl_viewer.cpp — SDL2 live preview implementation for multi-device capture.
//
// Sections:
// 1. init / close — SDL2 lifecycle + render thread management
// 2. addDevice — register device + create window/textures/render thread
// 3. pushFrame — format-specific conversion (Y16→jet, Y8→gray, YUYV→RGB, MJPG→RGB)
// 4. Format conversion helpers (y16ToJetRGB, y8ToRGB, yuyvToRGB, mjpgToRGB)
// 5. renderLoop — independent thread that uploads textures + renders at 30 fps

#include "nio_sdl_viewer.hpp"
#include "nio_log.hpp"
#include <algorithm>
#include <cstring>

namespace nio {

// === Section 1: SDL2 lifecycle ===

SDLViewer::SDLViewer() {}
SDLViewer::~SDLViewer() {
    close();
}

// init: initialize SDL2 video subsystem. Returns false if no display available
// (e.g. headless SSH session) — caller should continue without preview.
bool SDLViewer::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL init failed: " << SDL_GetError() << std::endl;
        return false;
    }
    running_ = true;
    return true;
}

// === Section 2: Device registration ===

// addDevice: register a device and create slots for its available sensors.
// - Creates one ViewerSlot per available channel (color/depth/IR/IR-L/IR-R)
// - On first device: creates SDL window, renderer, textures, and render thread
// - Window size = maxSlotsPerRow * maxSlotW × numDevices * maxSlotH
// - Uses SDL_RENDERER_SOFTWARE to avoid GPU dependency on headless servers
// Returns device index for pushFrame(), or -1 on SDL creation failure.
int SDLViewer::addDevice(const std::string& name, bool hasColor, OBFormat colorFmt, int cw, int ch, bool hasDepth,
                         int dw, int dh, bool hasIR, int irw, int irh, bool hasIRLeft, int ilw, int ilh,
                         bool hasIRRight, int irw2, int irh2) {
    DeviceRow row;
    row.name = name;

    // Track max slot dimensions for window sizing and scaling
    int maxW = 0, maxH = 0;

    // Helper: create a ViewerSlot and return its index in slots_
    auto addSlot = [&](const std::string& label, OBFormat fmt, int w, int h) -> int {
        int idx = slots_.size();
        slots_.push_back(std::unique_ptr<ViewerSlot>(new ViewerSlot()));
        auto& s = *slots_.back();
        s.label = label;
        s.format = fmt;
        s.w = w;
        s.h = h;
        s.rgbBuf.resize(w * h * 3, 0); // pre-allocate RGB24 buffer
        // Pre-create MJPEG decoder for MJPG color slots
        if (fmt == OB_FORMAT_MJPG)
            s.mjpgRes = std::make_shared<MjpgDecoderRes>();
        maxW = std::max(maxW, w);
        maxH = std::max(maxH, h);
        return idx;
    };

    // Add slots in left-to-right order: Color | Depth | IR[-L] | IR[-R]
    if (hasColor) {
        int idx = addSlot(name + " Color", colorFmt, cw, ch);
        row.slotIndices.push_back(idx);
        row.colorSlot = idx;
    }
    if (hasDepth) {
        int idx = addSlot(name + " Depth", OB_FORMAT_Y16, dw, dh);
        row.slotIndices.push_back(idx);
        row.depthSlot = idx;
    }
    // Gemini 305: single IR sensor (OB_FRAME_IR, Y8 format)
    if (hasIR) {
        int idx = addSlot(name + " IR", OB_FORMAT_Y8, irw, irh);
        row.slotIndices.push_back(idx);
        row.irSlot = idx;
    }
    // Gemini 335L/336L: stereo IR (OB_FRAME_IR_LEFT + IR_RIGHT, Y8 format)
    if (hasIRLeft) {
        int idx = addSlot(name + " IR-L", OB_FORMAT_Y8, ilw, ilh);
        row.slotIndices.push_back(idx);
        row.irLeftSlot = idx;
    }
    if (hasIRRight) {
        int idx = addSlot(name + " IR-R", OB_FORMAT_Y8, irw2, irh2);
        row.slotIndices.push_back(idx);
        row.irRightSlot = idx;
    }

    devices_.push_back(row);

    // Create SDL window + render thread on first device registration
    if (!window_ && !slots_.empty()) {
        // Recalculate max slots per row across all devices (for column count)
        maxSlotsPerRow_ = 0;
        for (auto& d : devices_)
            maxSlotsPerRow_ = std::max(maxSlotsPerRow_, static_cast<int>(d.slotIndices.size()));
        tileW_ = maxW;
        tileH_ = maxH;

        // Window dimensions: columns = maxSlotsPerRow, rows = numDevices
        int winW = maxSlotsPerRow_ * tileW_;
        int winH = devices_.size() * tileH_;
        window_ = SDL_CreateWindow("NIO Capture Monitor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, winW, winH,
                                   SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (!window_) {
            std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
            return -1;
        }
        // SOFTWARE renderer: avoids GPU dependency, works on headless/Xvfb
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
        if (!renderer_) {
            std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << std::endl;
            return -1;
        }

        // Create one streaming RGB24 texture per slot (updated by renderLoop)
        for (auto& s : slots_) {
            auto* tex = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, s->w, s->h);
            textures_.push_back(tex);
        }

        // Start independent render thread (decoupled from SDK callbacks)
        renderThread_ = std::thread(&SDLViewer::renderLoop, this);
    }

    int devIdx = devices_.size() - 1;
    NIO_LOG_INFO_S("SDLViewer: added device " << name << " devIdx=" << devIdx << " slots=" << row.slotIndices.size());
    return devIdx;
}

// close: stop render thread, destroy all SDL resources, free sws/AVFrame.
// Called by destructor or explicitly at program exit.
void SDLViewer::close() {
    running_ = false;
    if (renderThread_.joinable())
        renderThread_.join();

    for (auto* tex : textures_) {
        if (tex)
            SDL_DestroyTexture(tex);
    }
    textures_.clear();

    // Free per-slot YUYV sws context + AVFrames (MJPEG decoder freed by MjpgDecoderRes)
    for (auto& s : slots_) {
        if (s->yuyvSws) {
            sws_freeContext(s->yuyvSws);
            s->yuyvSws = nullptr;
        }
        if (s->yuyvSrcFrame) {
            av_frame_free(&s->yuyvSrcFrame);
        }
        if (s->yuyvDstFrame) {
            av_frame_free(&s->yuyvDstFrame);
        }
    }

    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    SDL_Quit();
}

// === Section 3: Frame push (called from SDK callback) ===

// pushFrame: convert raw sensor data to RGB24 and store in the matching slot.
// Dispatches to format-specific converter (Y16→jet, Y8→gray, YUYV→RGB, MJPG→RGB).
// All conversion happens under the slot mutex to avoid tearing with renderLoop.
// After conversion, sets slot.updated=true so renderLoop uploads the new texture.
void SDLViewer::pushFrame(int devIdx, ViewerChannel ch, const uint8_t* data, uint32_t size, float depthScale,
                          float depthMinM, float depthMaxM) {
    if (devIdx < 0 || devIdx >= (int)devices_.size())
        return;
    auto& dev = devices_[devIdx];

    // Map channel enum → slot index for this device
    int slotIdx = -1;
    switch (ch) {
    case ViewerChannel::COLOR:
        slotIdx = dev.colorSlot;
        break;
    case ViewerChannel::DEPTH:
        slotIdx = dev.depthSlot;
        break;
    case ViewerChannel::IR:
        slotIdx = dev.irSlot;
        break;
    case ViewerChannel::IR_LEFT:
        slotIdx = dev.irLeftSlot;
        break;
    case ViewerChannel::IR_RIGHT:
        slotIdx = dev.irRightSlot;
        break;
    }
    if (slotIdx < 0 || slotIdx >= (int)slots_.size())
        return;
    auto& slot = *slots_[slotIdx];

    // Dispatch based on original pixel format
    if (slot.format == OB_FORMAT_Y16) {
        // Depth Y16 → jet colormap (same palette as D2C fusion preview)
        if (size < static_cast<uint32_t>(slot.w * slot.h * 2))
            return;
        const uint16_t* y16 = reinterpret_cast<const uint16_t*>(data);
        std::lock_guard<std::mutex> lock(slot.mtx);
        y16ToJetRGB(y16, slot.w, slot.h, slot.rgbBuf.data(), depthScale, depthMinM, depthMaxM);
        slot.updated = true;
    } else if (slot.format == OB_FORMAT_Y8) {
        // IR Y8 → grayscale (R=G=B=Y8)
        if (size < static_cast<uint32_t>(slot.w * slot.h))
            return;
        std::lock_guard<std::mutex> lock(slot.mtx);
        y8ToRGB(data, slot.w, slot.h, slot.rgbBuf.data());
        slot.updated = true;
    } else if (slot.format == OB_FORMAT_YUYV) {
        // Gemini 305 color: YUYV422 → RGB24 via libswscale
        std::lock_guard<std::mutex> lock(slot.mtx);
        if (yuyvToRGB(data, slot.w, slot.h, slot.rgbBuf.data(), slot)) {
            slot.updated = true;
        }
    } else if (slot.format == OB_FORMAT_MJPG) {
        // Gemini 335L/336L color: MJPG → RGB24 via decodeColorToRGB
        std::lock_guard<std::mutex> lock(slot.mtx);
        if (mjpgToRGB(data, size, slot.w, slot.h, slot.rgbBuf.data(), slot.mjpgRes)) {
            slot.updated = true;
        }
    }
}

// === Section 4: Format conversion helpers ===

// y16ToJetRGB: convert 16-bit depth to RGB24 using jet colormap.
// raw==0 pixels are rendered as black (invalid depth / no return).
// Non-zero pixels: raw * scale / 1000.0f → meters, then normalize to [minM, maxM].
// This matches the fusion preview's jet colormap normalization exactly.
void SDLViewer::y16ToJetRGB(const uint16_t* y16, int w, int h, uint8_t* rgb, float scale, float minM, float maxM) {
    for (int i = 0; i < w * h; i++) {
        uint16_t raw = y16[i];
        if (raw == 0) {
            // Invalid depth: render as black
            rgb[i * 3 + 0] = 0;
            rgb[i * 3 + 1] = 0;
            rgb[i * 3 + 2] = 0;
        } else {
            // Convert raw → meters → normalize to [0,1] → jet colormap
            float distM = raw * scale / 1000.0f;
            float norm = (distM - minM) / (maxM - minM);
            norm = std::max(0.0f, std::min(1.0f, norm));
            uint8_t v = static_cast<uint8_t>(norm * 255.0f);
            jetColormap(v, rgb[i * 3 + 0], rgb[i * 3 + 1], rgb[i * 3 + 2]);
        }
    }
}

// y8ToRGB: convert 8-bit IR to RGB24 grayscale (R=G=B=Y8).
void SDLViewer::y8ToRGB(const uint8_t* y8, int w, int h, uint8_t* rgb) {
    for (int i = 0; i < w * h; i++) {
        rgb[i * 3 + 0] = y8[i];
        rgb[i * 3 + 1] = y8[i];
        rgb[i * 3 + 2] = y8[i];
    }
}

// yuyvToRGB: convert YUYV422 packed data to RGB24 via libswscale.
// SwsContext + AVFrames are lazily created on first call per slot (yuyvSwsInit flag).
// Returns false if sws initialization fails.
bool SDLViewer::yuyvToRGB(const uint8_t* yuyv, int w, int h, uint8_t* rgb, ViewerSlot& slot) {
    if (!slot.yuyvSwsInit) {
        // Create sws context: YUYV422 → RGB24 (preserves 4:2:2 chroma)
        slot.yuyvSws =
            sws_getContext(w, h, AV_PIX_FMT_YUYV422, w, h, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!slot.yuyvSws)
            return false;
        // Allocate source/destination AVFrames for sws_scale
        slot.yuyvSrcFrame = av_frame_alloc();
        slot.yuyvDstFrame = av_frame_alloc();
        slot.yuyvSrcFrame->format = AV_PIX_FMT_YUYV422;
        slot.yuyvSrcFrame->width = w;
        slot.yuyvSrcFrame->height = h;
        av_frame_get_buffer(slot.yuyvSrcFrame, 0);
        slot.yuyvDstFrame->format = AV_PIX_FMT_RGB24;
        slot.yuyvDstFrame->width = w;
        slot.yuyvDstFrame->height = h;
        av_frame_get_buffer(slot.yuyvDstFrame, 0);
        slot.yuyvSwsInit = true;
    }
    // Copy YUYV data into source frame and convert
    memcpy(slot.yuyvSrcFrame->data[0], yuyv, w * h * 2);
    sws_scale(slot.yuyvSws, slot.yuyvSrcFrame->data, slot.yuyvSrcFrame->linesize, 0, h, slot.yuyvDstFrame->data,
              slot.yuyvDstFrame->linesize);
    // Copy from AVFrame (may have padding) to tightly-packed rgbBuf
    int rgbStride = w * 3;
    for (int y = 0; y < h; y++) {
        memcpy(rgb + y * rgbStride, slot.yuyvDstFrame->data[0] + y * slot.yuyvDstFrame->linesize[0], rgbStride);
    }
    return true;
}

// mjpgToRGB: decode MJPEG frame to RGB24 via decodeColorToRGB().
// Uses the slot's MjpgDecoderRes (lazily initialized inside decodeColorToRGB).
// Returns false if decoder init or decode fails.
bool SDLViewer::mjpgToRGB(const uint8_t* data, uint32_t size, int w, int h, uint8_t* rgb,
                          std::shared_ptr<MjpgDecoderRes> mjpg) {
    if (!mjpg)
        return false;
    if (!mjpg->swsInitialized) {
        if (!mjpg->init(w, h, OB_FORMAT_MJPG))
            return false;
    }
    return decodeColorToRGB(data, size, OB_FORMAT_MJPG, w, h, rgb, mjpg);
}

// === Section 5: Render thread ===

// renderLoop: runs on a dedicated thread at RENDER_FPS (30 fps).
// For each frame:
// 1. Poll SDL events (handle window close)
// 2. For each slot: if updated, upload rgbBuf to SDL texture under lock
// 3. Render all slots in the per-device-row layout with uniform scaling
// The texture upload only happens when slot.updated==true, avoiding
// redundant SDL_UpdateTexture calls for unchanged frames.
void SDLViewer::renderLoop() {
    while (running_) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running_ = false;
                return;
            }
        }

        // Clear with dark background
        SDL_SetRenderDrawColor(renderer_, 30, 30, 30, 255);
        SDL_RenderClear(renderer_);

        int winW, winH;
        SDL_GetWindowSize(window_, &winW, &winH);

        // Render each device's row of slots
        for (size_t di = 0; di < devices_.size(); di++) {
            auto& dev = devices_[di];
            for (size_t si = 0; si < dev.slotIndices.size(); si++) {
                int slotIdx = dev.slotIndices[si];
                auto& slot = *slots_[slotIdx];
                auto* tex = textures_[slotIdx];

                // Upload texture only if new frame data is available
                {
                    std::lock_guard<std::mutex> lock(slot.mtx);
                    if (slot.updated) {
                        SDL_UpdateTexture(tex, nullptr, slot.rgbBuf.data(), slot.w * 3);
                        slot.updated = false;
                    }
                }

                // Uniform scaling: fit all slots into the window preserving aspect ratio
                float scaleX = static_cast<float>(winW) / (maxSlotsPerRow_ * tileW_);
                float scaleY = static_cast<float>(winH) / (devices_.size() * tileH_);
                float scale = std::min(scaleX, scaleY);

                int dstW = static_cast<int>(slot.w * scale);
                int dstH = static_cast<int>(slot.h * scale);
                // Tile position: column = slot index within row, row = device index
                int xOff = static_cast<int>(si) * (winW / maxSlotsPerRow_);
                int yOff = static_cast<int>(di) * (winH / devices_.size());

                SDL_Rect dstRect = { xOff, yOff, dstW, dstH };
                SDL_RenderCopy(renderer_, tex, nullptr, &dstRect);
            }
        }

        SDL_RenderPresent(renderer_);
        SDL_Delay(1000 / RENDER_FPS);
    }
}

} // namespace nio
