// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_sdl_viewer.cpp — SDL2 live preview implementation for multi-device capture.
//
// Architecture:
// - pushFrame(): SDK callback → memcpy raw data into slot.rawBuf (microseconds)
// - decodeThread(): converts rawBuf → RGB24 into slot.renderBuf (parallel)
// - renderLoop(): uploads renderBuf to SDL textures + draws text overlays (30fps)
//
// Text rendering uses scaled-up 5x7 bitmap font (no SDL_ttf dependency).

#include "nio_sdl_viewer.hpp"
#include "nio_log.hpp"
#include <algorithm>
#include <cstring>

namespace nio {

// === Bitmap font (5x7 glyphs, scaled 3x → 15x21 pixels per char) ===

// 5x7 font data: each byte is a row, 5 LSB bits are pixel columns.
// Covers: A-Z, 0-9, underscore, dash, dot, slash, space, bracket, colon
static const uint8_t font5x7[][7] = {
    // Space (index 0)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // A
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    // B
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    // C
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    // D
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    // E
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    // F
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    // G
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E},
    // H
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    // I
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    // J
    {0x01,0x01,0x01,0x01,0x01,0x11,0x0E},
    // K
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    // L
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    // M
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    // N
    {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    // O
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    // P
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    // Q
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    // R
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    // S
    {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
    // T
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    // U
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    // V
    {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04},
    // W
    {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    // X
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    // Y
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    // Z
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
    // 0
    {0x0E,0x13,0x15,0x15,0x15,0x19,0x0E},
    // 1
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    // 2
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
    // 3
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    // 4
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    // 5
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    // 6
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    // 7
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    // 8
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    // 9
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    // _ (underscore)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    // - (dash)
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    // . (dot)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x04},
    // / (slash)
    {0x01,0x01,0x02,0x04,0x08,0x10,0x10},
    // [ (left bracket)
    {0x06,0x04,0x04,0x04,0x04,0x04,0x06},
    // ] (right bracket)
    {0x0C,0x04,0x04,0x04,0x04,0x04,0x0C},
    // : (colon)
    {0x00,0x00,0x04,0x00,0x04,0x00,0x00},
};

// Map ASCII char to font5x7 index
static int charToFontIdx(char c) {
    if (c == ' ')  return 0;
    if (c >= 'A' && c <= 'Z') return 1 + (c - 'A');
    if (c >= 'a' && c <= 'z') return 1 + (c - 'a'); // lowercase → uppercase
    if (c >= '0' && c <= '9') return 27 + (c - '0');
    if (c == '_') return 37;
    if (c == '-') return 38;
    if (c == '.') return 39;
    if (c == '/') return 40;
    if (c == '[') return 41;
    if (c == ']') return 42;
    if (c == ':') return 43;
    return 0; // unknown → space
}

// Render a text string into an RGBA surface using the scaled bitmap font.
// Returns a new SDL_Surface (caller must free), or nullptr.
static SDL_Surface* renderBitmapText(const std::string& text, int scale,
    uint8_t fgR, uint8_t fgG, uint8_t fgB,
    uint8_t bgR, uint8_t bgG, uint8_t bgB) {

    int charW = 5 * scale;
    int charH = 7 * scale;
    int gap = scale; // 1-pixel gap between chars (scaled)
    int surfW = static_cast<int>(text.size()) * (charW + gap) + gap;
    int surfH = charH + scale * 2; // top/bottom padding

    SDL_Surface* surf = SDL_CreateRGBSurface(0, surfW, surfH, 32,
        0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);
    if (!surf) return nullptr;

    // Fill background
    uint32_t bgPixel = (0xFF << 24) | (bgB << 16) | (bgG << 8) | bgR;
    SDL_FillRect(surf, nullptr, bgPixel);

    // Draw characters
    for (size_t ci = 0; ci < text.size(); ci++) {
        int idx = charToFontIdx(text[ci]);
        const uint8_t* glyph = font5x7[idx];
        int baseX = gap + static_cast<int>(ci) * (charW + gap);
        int baseY = scale; // top padding

        for (int row = 0; row < 7; row++) {
            uint8_t bits = glyph[row];
            for (int col = 0; col < 5; col++) {
                if (bits & (0x10 >> col)) {
                    // Draw scaled pixel block
                    for (int sy = 0; sy < scale; sy++) {
                        for (int sx = 0; sx < scale; sx++) {
                            int px = baseX + col * scale + sx;
                            int py = baseY + row * scale + sy;
                            if (px < surfW && py < surfH) {
                                uint32_t* pixels = static_cast<uint32_t*>(surf->pixels);
                                pixels[py * surfW + px] =
                                    (0xFF << 24) | (fgB << 16) | (fgG << 8) | fgR;
                            }
                        }
                    }
                }
            }
        }
    }

    return surf;
}

// === Helpers ===

std::string SDLViewer::obFormatToString(OBFormat fmt) {
    switch (fmt) {
    case OB_FORMAT_MJPG:   return "MJPG";
    case OB_FORMAT_YUYV:   return "YUYV";
    case OB_FORMAT_Y16:    return "Y16";
    case OB_FORMAT_Y8:     return "Y8";
    case OB_FORMAT_RGB:    return "RGB";
    case OB_FORMAT_BGR:    return "BGR";
    case OB_FORMAT_H264:   return "H264";
    case OB_FORMAT_H265:   return "H265";
    default:               return "???";
    }
}

// === Section 1: Lifecycle ===

SDLViewer::SDLViewer() {}
SDLViewer::~SDLViewer() { close(); }

bool SDLViewer::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL init failed: " << SDL_GetError() << std::endl;
        return false;
    }
    running_ = true;
    return true;
}

// === Section 2: Device registration ===

int SDLViewer::addDevice(const std::string& name, const std::string& cameraType,
    const std::string& serialNumber, bool hasColor, OBFormat colorFmt, int cw, int ch,
    bool hasDepth, OBFormat depthFmt, int dw, int dh,
    bool hasIR, int irw, int irh,
    bool hasIRLeft, int ilw, int ilh,
    bool hasIRRight, int irw2, int irh2) {

    DeviceRow row;
    row.name = name;
    row.cameraType = cameraType;
    row.serialNumber = serialNumber;

    auto addSlot = [&](const std::string& label, OBFormat fmt, int w, int h) -> int {
        int idx = static_cast<int>(slots_.size());
        slots_.push_back(std::unique_ptr<ViewerSlot>(new ViewerSlot()));
        auto& s = *slots_.back();
        s.label = label;
        s.format = fmt;
        s.formatStr = obFormatToString(fmt);
        s.w = w;
        s.h = h;
        size_t rawMax = 0;
        if (fmt == OB_FORMAT_Y16)       rawMax = w * h * 2;
        else if (fmt == OB_FORMAT_Y8)   rawMax = w * h;
        else if (fmt == OB_FORMAT_YUYV) rawMax = w * h * 2;
        else                             rawMax = w * h * 4;
        s.rawBuf.resize(rawMax, 0);
        s.renderBuf.resize(w * h * 3, 0);
        if (fmt == OB_FORMAT_MJPG)
            s.mjpgRes = std::make_shared<MjpgDecoderRes>();
        return idx;
    };

    if (hasColor) {
        int idx = addSlot("Color", colorFmt, cw, ch);
        row.slotIndices.push_back(idx);
        row.colorSlot = idx;
    }
    if (hasDepth) {
        int idx = addSlot("Depth", depthFmt, dw, dh);
        row.slotIndices.push_back(idx);
        row.depthSlot = idx;
    }
    if (hasIR) {
        int idx = addSlot("IR", OB_FORMAT_Y8, irw, irh);
        row.slotIndices.push_back(idx);
        row.irSlot = idx;
    }
    if (hasIRLeft) {
        int idx = addSlot("IR-L", OB_FORMAT_Y8, ilw, ilh);
        row.slotIndices.push_back(idx);
        row.irLeftSlot = idx;
    }
    if (hasIRRight) {
        int idx = addSlot("IR-R", OB_FORMAT_Y8, irw2, irh2);
        row.slotIndices.push_back(idx);
        row.irRightSlot = idx;
    }

    devices_.push_back(row);
    int devIdx = static_cast<int>(devices_.size()) - 1;
    NIO_LOG_INFO_S("SDLViewer: added device " << name << " devIdx=" << devIdx
        << " slots=" << row.slotIndices.size());
    return devIdx;
}

// === Section 3: Create window, textures, start threads ===

bool SDLViewer::createWindow() {
    if (initialized_ || slots_.empty())
        return false;

    int maxW = 0, maxH = 0;
    for (auto& s : slots_) {
        maxW = std::max(maxW, s->w);
        maxH = std::max(maxH, s->h);
    }
    maxSlotsPerRow_ = 0;
    for (auto& d : devices_)
        maxSlotsPerRow_ = std::max(maxSlotsPerRow_, static_cast<int>(d.slotIndices.size()));
    tileW_ = maxW;
    tileH_ = maxH;

    int winW = maxSlotsPerRow_ * tileW_;
    int rows = static_cast<int>(devices_.size());
    int winH = rows * (tileH_ + ROW_HEADER_H + FORMAT_BAR_H);
    window_ = SDL_CreateWindow("NIO Capture Monitor",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, winW, winH,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window_) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
        return false;
    }
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer_) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window_);
        window_ = nullptr;
        return false;
    }

    for (auto& s : slots_) {
        auto* tex = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24,
            SDL_TEXTUREACCESS_STREAMING, s->w, s->h);
        textures_.push_back(tex);
    }

    // Pre-render label textures at initial window size
    rebuildLabelTextures(winW, winH);

    // Start decode + render threads
    decodeThread_ = std::thread(&SDLViewer::decodeThreadFunc, this);
    renderThread_ = std::thread(&SDLViewer::renderLoop, this);
    initialized_ = true;
    NIO_LOG_INFO_S("SDLViewer: window created " << winW << "x" << winH
        << " slots=" << slots_.size() << " devices=" << devices_.size());
    return true;
}

// Rebuild all cached label textures when window size changes
void SDLViewer::rebuildLabelTextures(int winW, int winH) {
    // Destroy old textures
    for (auto& lt : titleTexs_)   { if (lt.tex) SDL_DestroyTexture(lt.tex); }
    for (auto& lt : formatTexs_)  { if (lt.tex) SDL_DestroyTexture(lt.tex); }
    for (auto& lt : channelTexs_) { if (lt.tex) SDL_DestroyTexture(lt.tex); }
    titleTexs_.clear();
    formatTexs_.clear();
    channelTexs_.clear();

    if (!renderer_) return;

    // Compute scale factor (same as renderLoop)
    int numRows = static_cast<int>(devices_.size());
    if (numRows == 0 || maxSlotsPerRow_ == 0) return;
    int rowTotalH = ROW_HEADER_H + tileH_ + FORMAT_BAR_H;
    int totalContentH = numRows * rowTotalH;
    float scaleX = static_cast<float>(winW) / (maxSlotsPerRow_ * tileW_);
    float scaleY = static_cast<float>(winH) / totalContentH;
    float scale = std::min(scaleX, scaleY);

    // Choose font scale proportional to window scale, minimum 2x
    int fscale = std::max(2, static_cast<int>(FONT_SCALE * scale));
    if (fscale < 1) fscale = 1;

    auto makeLabel = [&](const std::string& text,
        uint8_t fgR, uint8_t fgG, uint8_t fgB,
        uint8_t bgR, uint8_t bgG, uint8_t bgB) -> LabelTex {
        LabelTex lt;
        SDL_Surface* surf = renderBitmapText(text, fscale, fgR, fgG, fgB, bgR, bgG, bgB);
        if (surf) {
            lt.tex = SDL_CreateTextureFromSurface(renderer_, surf);
            lt.w = surf->w;
            lt.h = surf->h;
            SDL_FreeSurface(surf);
        }
        return lt;
    };

    // Title textures: "型号_序列号" (white on dark bg)
    for (auto& dev : devices_) {
        std::string title = dev.cameraType + "_" + dev.serialNumber;
        titleTexs_.push_back(makeLabel(title, 255, 255, 255, 30, 30, 30));
    }

    // Channel + format label textures per slot
    for (auto& slot : slots_) {
        channelTexs_.push_back(makeLabel(slot->label, 200, 200, 255, 30, 30, 30));
        formatTexs_.push_back(makeLabel(slot->formatStr, 100, 255, 100, 30, 30, 30));
    }

    cachedWinW_ = winW;
    cachedWinH_ = winH;
}

// === Section 4: Cleanup ===

void SDLViewer::cleanupSlot(ViewerSlot& s) {
    if (s.yuyvSws) { sws_freeContext(s.yuyvSws); s.yuyvSws = nullptr; }
    if (s.yuyvSrcFrame) { av_frame_free(&s.yuyvSrcFrame); }
    if (s.yuyvDstFrame) { av_frame_free(&s.yuyvDstFrame); }
    s.yuyvSwsInit = false;
}

void SDLViewer::close() {
    running_ = false;
    decodeCv_.notify_all();
    if (decodeThread_.joinable()) decodeThread_.join();
    if (renderThread_.joinable()) renderThread_.join();

    for (auto& lt : titleTexs_)   { if (lt.tex) SDL_DestroyTexture(lt.tex); }
    for (auto& lt : formatTexs_)  { if (lt.tex) SDL_DestroyTexture(lt.tex); }
    for (auto& lt : channelTexs_) { if (lt.tex) SDL_DestroyTexture(lt.tex); }
    titleTexs_.clear();
    formatTexs_.clear();
    channelTexs_.clear();

    for (auto* tex : textures_) { if (tex) SDL_DestroyTexture(tex); }
    textures_.clear();
    for (auto& s : slots_) cleanupSlot(*s);

    if (renderer_) { SDL_DestroyRenderer(renderer_); renderer_ = nullptr; }
    if (window_) { SDL_DestroyWindow(window_); window_ = nullptr; }
    if (initialized_) {
        SDL_Quit();
        initialized_ = false;
    }
}

// === Section 5: pushFrame (SDK callback — memcpy only) ===

void SDLViewer::pushFrame(int devIdx, ViewerChannel ch, const uint8_t* data, uint32_t size,
    float depthScale, float depthMinM, float depthMaxM) {
    if (!initialized_) return;
    if (devIdx < 0 || devIdx >= static_cast<int>(devices_.size())) return;
    auto& dev = devices_[devIdx];

    int slotIdx = -1;
    switch (ch) {
    case ViewerChannel::COLOR:    slotIdx = dev.colorSlot;    break;
    case ViewerChannel::DEPTH:    slotIdx = dev.depthSlot;    break;
    case ViewerChannel::IR:       slotIdx = dev.irSlot;       break;
    case ViewerChannel::IR_LEFT:  slotIdx = dev.irLeftSlot;   break;
    case ViewerChannel::IR_RIGHT: slotIdx = dev.irRightSlot;  break;
    }
    if (slotIdx < 0 || slotIdx >= static_cast<int>(slots_.size())) return;
    auto& slot = *slots_[slotIdx];

    {
        std::lock_guard<std::mutex> lock(slot.rawMtx);
        if (size > slot.rawBuf.size()) return;
        memcpy(slot.rawBuf.data(), data, size);
        slot.rawSize = size;
        slot.depthScale = depthScale;
        slot.depthMinM = depthMinM;
        slot.depthMaxM = depthMaxM;
        slot.rawUpdated = true;
    }
    {
        std::lock_guard<std::mutex> lk(decodeCvMtx_);
        decodeWakeup_ = true;
    }
    decodeCv_.notify_one();
}

// === Section 6: Decode thread (raw → RGB24 conversion) ===

void SDLViewer::decodeSlot(ViewerSlot& slot) {
    std::vector<uint8_t> rawCopy;
    uint32_t rawSz = 0;
    OBFormat fmt = slot.format;
    int w = slot.w, h = slot.h;
    float ds, dmin, dmax;
    {
        std::lock_guard<std::mutex> lock(slot.rawMtx);
        if (!slot.rawUpdated) return;
        rawCopy.assign(slot.rawBuf.data(), slot.rawBuf.data() + slot.rawSize);
        rawSz = slot.rawSize;
        ds = slot.depthScale;
        dmin = slot.depthMinM;
        dmax = slot.depthMaxM;
        slot.rawUpdated = false;
    }

    std::vector<uint8_t> rgb(w * h * 3, 0);
    bool ok = false;

    if (fmt == OB_FORMAT_Y16) {
        if (rawSz >= static_cast<uint32_t>(w * h * 2)) {
            const uint16_t* y16 = reinterpret_cast<const uint16_t*>(rawCopy.data());
            for (int i = 0; i < w * h; i++) {
                uint16_t raw = y16[i];
                if (raw == 0) {
                    rgb[i * 3 + 0] = 0; rgb[i * 3 + 1] = 0; rgb[i * 3 + 2] = 0;
                } else {
                    float distM = raw * ds / 1000.0f;
                    float norm = (distM - dmin) / (dmax - dmin);
                    norm = std::max(0.0f, std::min(1.0f, norm));
                    uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                    jetColormap(v, rgb[i * 3 + 0], rgb[i * 3 + 1], rgb[i * 3 + 2]);
                }
            }
            ok = true;
        }
    } else if (fmt == OB_FORMAT_Y8) {
        if (rawSz >= static_cast<uint32_t>(w * h)) {
            for (int i = 0; i < w * h; i++) {
                rgb[i * 3 + 0] = rawCopy[i];
                rgb[i * 3 + 1] = rawCopy[i];
                rgb[i * 3 + 2] = rawCopy[i];
            }
            ok = true;
        }
    } else if (fmt == OB_FORMAT_YUYV) {
        std::lock_guard<std::mutex> lock(slot.rawMtx);
        if (!slot.yuyvSwsInit) {
            slot.yuyvSws = sws_getContext(w, h, AV_PIX_FMT_YUYV422, w, h,
                AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (!slot.yuyvSws) return;
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
        if (rawSz >= static_cast<uint32_t>(w * h * 2)) {
            memcpy(slot.yuyvSrcFrame->data[0], rawCopy.data(), w * h * 2);
            sws_scale(slot.yuyvSws,
                slot.yuyvSrcFrame->data, slot.yuyvSrcFrame->linesize, 0, h,
                slot.yuyvDstFrame->data, slot.yuyvDstFrame->linesize);
            int stride = w * 3;
            for (int y = 0; y < h; y++) {
                memcpy(rgb.data() + y * stride,
                    slot.yuyvDstFrame->data[0] + y * slot.yuyvDstFrame->linesize[0], stride);
            }
            ok = true;
        }
    } else if (fmt == OB_FORMAT_MJPG) {
        auto mjpg = slot.mjpgRes;
        if (mjpg) {
            if (!mjpg->swsInitialized) {
                if (!mjpg->init(w, h, OB_FORMAT_MJPG))
                    return;
            }
            if (decodeColorToRGB(rawCopy.data(), rawSz, OB_FORMAT_MJPG, w, h, rgb.data(), mjpg))
                ok = true;
        }
    }

    if (ok) {
        std::lock_guard<std::mutex> lock(slot.renderMtx);
        memcpy(slot.renderBuf.data(), rgb.data(), w * h * 3);
        slot.renderUpdated = true;
    }
}

void SDLViewer::decodeThreadFunc() {
    setThreadName("nio-sdl-decode");
    while (running_) {
        {
            std::unique_lock<std::mutex> lk(decodeCvMtx_);
            decodeCv_.wait_for(lk, std::chrono::milliseconds(50),
                [this]() { return decodeWakeup_.load() || !running_.load(); });
            decodeWakeup_ = false;
        }
        if (!running_) break;

        for (auto& slot : slots_) {
            if (slot->rawUpdated) {
                decodeSlot(*slot);
            }
        }
    }
}

// === Section 7: Render thread (textures + text overlays at 30fps) ===

void SDLViewer::renderLoop() {
    setThreadName("nio-sdl-render");
    while (running_) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running_ = false;
                return;
            }
        }

        SDL_SetRenderDrawColor(renderer_, 20, 20, 20, 255);
        SDL_RenderClear(renderer_);

        int winW, winH;
        SDL_GetWindowSize(window_, &winW, &winH);

        // Rebuild label textures if window resized
        if (winW != cachedWinW_ || winH != cachedWinH_) {
            rebuildLabelTextures(winW, winH);
        }

        int numRows = static_cast<int>(devices_.size());
        if (numRows == 0 || maxSlotsPerRow_ == 0) {
            SDL_RenderPresent(renderer_);
            SDL_Delay(1000 / RENDER_FPS);
            continue;
        }

        int rowTotalH = ROW_HEADER_H + tileH_ + FORMAT_BAR_H;
        int totalContentH = numRows * rowTotalH;
        float scaleX = static_cast<float>(winW) / (maxSlotsPerRow_ * tileW_);
        float scaleY = static_cast<float>(winH) / totalContentH;
        float scale = std::min(scaleX, scaleY);

        for (int di = 0; di < numRows; di++) {
            auto& dev = devices_[di];
            int rowY = static_cast<int>(di * rowTotalH * scale);

            // Device title: 型号_序列号
            if (di < static_cast<int>(titleTexs_.size()) && titleTexs_[di].tex) {
                auto& lt = titleTexs_[di];
                int dstH = std::min(lt.h, static_cast<int>(ROW_HEADER_H * scale));
                int dstW = static_cast<int>(lt.w * static_cast<float>(dstH) / lt.h);
                if (dstW > winW) dstW = winW;
                SDL_Rect dst = {0, rowY, dstW, dstH};
                SDL_RenderCopy(renderer_, lt.tex, nullptr, &dst);
            }

            int videoY = rowY + static_cast<int>(ROW_HEADER_H * scale);
            int colW = static_cast<int>(winW / maxSlotsPerRow_);

            for (int si = 0; si < static_cast<int>(dev.slotIndices.size()); si++) {
                int slotIdx = dev.slotIndices[si];
                auto& slot = *slots_[slotIdx];
                auto* tex = textures_[slotIdx];

                // Upload RGB texture from decode thread
                {
                    std::lock_guard<std::mutex> lock(slot.renderMtx);
                    if (slot.renderUpdated) {
                        SDL_UpdateTexture(tex, nullptr, slot.renderBuf.data(), slot.w * 3);
                        slot.renderUpdated = false;
                    }
                }

                // Scaled destination rect (preserve aspect ratio, center in column)
                int dstW = static_cast<int>(slot.w * scale);
                int dstH = static_cast<int>(slot.h * scale);
                int xOff = si * colW + (colW - dstW) / 2;

                SDL_Rect dstRect = {xOff, videoY, dstW, dstH};
                SDL_RenderCopy(renderer_, tex, nullptr, &dstRect);

                // Channel label (top-left of tile)
                if (slotIdx < static_cast<int>(channelTexs_.size()) && channelTexs_[slotIdx].tex) {
                    auto& clt = channelTexs_[slotIdx];
                    int clH = std::min(clt.h, static_cast<int>(ROW_HEADER_H * scale * 0.7f));
                    int clW = static_cast<int>(clt.w * static_cast<float>(clH) / clt.h);
                    SDL_Rect cdst = {xOff + 2, videoY + 2, clW, clH};
                    SDL_RenderCopy(renderer_, clt.tex, nullptr, &cdst);
                }

                // Format label (below tile)
                if (slotIdx < static_cast<int>(formatTexs_.size()) && formatTexs_[slotIdx].tex) {
                    auto& flt = formatTexs_[slotIdx];
                    int flH = std::min(flt.h, static_cast<int>(FORMAT_BAR_H * scale));
                    int flW = static_cast<int>(flt.w * static_cast<float>(flH) / flt.h);
                    int fmtX = xOff + (dstW - flW) / 2;
                    int fmtY = videoY + dstH + 1;
                    SDL_Rect fdst = {fmtX, fmtY, flW, flH};
                    SDL_RenderCopy(renderer_, flt.tex, nullptr, &fdst);
                }
            }
        }

        SDL_RenderPresent(renderer_);
        SDL_Delay(1000 / RENDER_FPS);
    }
}

} // namespace nio
