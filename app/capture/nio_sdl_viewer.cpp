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

#include "nio_sdl_viewer.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace nio {

// === Bitmap font (5x7 glyphs, scaled 3x → 15x21 pixels per char) ===

// 5x7 font data: each byte is a row, 5 LSB bits are pixel columns.
// Covers: A-Z, 0-9, underscore, dash, dot, slash, space, bracket, colon
static const uint8_t font5x7[][7] = {
    // Space (index 0)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    // A
    { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },
    // B
    { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E },
    // C
    { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E },
    // D
    { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E },
    // E
    { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F },
    // F
    { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 },
    // G
    { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E },
    // H
    { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },
    // I
    { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E },
    // J
    { 0x01, 0x01, 0x01, 0x01, 0x01, 0x11, 0x0E },
    // K
    { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },
    // L
    { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F },
    // M
    { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 },
    // N
    { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 },
    // O
    { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },
    // P
    { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 },
    // Q
    { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D },
    // R
    { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 },
    // S
    { 0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E },
    // T
    { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
    // U
    { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },
    // V
    { 0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04 },
    // W
    { 0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11 },
    // X
    { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 },
    // Y
    { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 },
    // Z
    { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F },
    // 0
    { 0x0E, 0x13, 0x15, 0x15, 0x15, 0x19, 0x0E },
    // 1
    { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E },
    // 2
    { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F },
    // 3
    { 0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E },
    // 4
    { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 },
    // 5
    { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E },
    // 6
    { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E },
    // 7
    { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },
    // 8
    { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E },
    // 9
    { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C },
    // _ (underscore)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F },
    // - (dash)
    { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 },
    // . (dot)
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 },
    // / (slash)
    { 0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10 },
    // [ (left bracket)
    { 0x06, 0x04, 0x04, 0x04, 0x04, 0x04, 0x06 },
    // ] (right bracket)
    { 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0C },
    // : (colon)
    { 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00 },
};

// Map ASCII char to font5x7 index
static int charToFontIdx(char c) {
    if (c == ' ')
        return 0;
    if (c >= 'A' && c <= 'Z')
        return 1 + (c - 'A');
    if (c >= 'a' && c <= 'z')
        return 1 + (c - 'a'); // lowercase → uppercase
    if (c >= '0' && c <= '9')
        return 27 + (c - '0');
    if (c == '_')
        return 37;
    if (c == '-')
        return 38;
    if (c == '.')
        return 39;
    if (c == '/')
        return 40;
    if (c == '[')
        return 41;
    if (c == ']')
        return 42;
    if (c == ':')
        return 43;
    return 0; // unknown → space
}

// Render a text string into an RGBA surface using the scaled bitmap font.
// Returns a new SDL_Surface (caller must free), or nullptr.
static SDL_Surface* renderBitmapText(const std::string& text, int scale, uint8_t fgR, uint8_t fgG, uint8_t fgB,
                                     uint8_t bgR, uint8_t bgG, uint8_t bgB) {

    int charW = 5 * scale;
    int charH = 7 * scale;
    int gap = scale; // 1-pixel gap between chars (scaled)
    int surfW = static_cast<int>(text.size()) * (charW + gap) + gap;
    int surfH = charH + scale * 2; // top/bottom padding

    SDL_Surface* surf = SDL_CreateRGBSurface(0, surfW, surfH, 32, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);
    if (!surf)
        return nullptr;

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
                                pixels[py * surfW + px] = (0xFF << 24) | (fgB << 16) | (fgG << 8) | fgR;
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

std::string SDLViewer::nioFormatToString(NioFormat fmt) {
    switch (fmt) {
    case NioFormat::MJPG:
        return "MJPG";
    case NioFormat::YUYV:
        return "YUYV";
    case NioFormat::Y16:
        return "Y16";
    case NioFormat::Y8:
        return "Y8";
    case NioFormat::RGB:
        return "RGB";
    case NioFormat::BGR:
        return "BGR";
    case NioFormat::NV12:
        return "NV12";
    case NioFormat::H264:
        return "H264";
    case NioFormat::H265:
        return "H265";
    default:
        return "???";
    }
}

// === Section 1: Lifecycle ===

SDLViewer::SDLViewer() {}
SDLViewer::~SDLViewer() {
    close();
}

bool SDLViewer::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        NIO_LOG_ERROR_S("SDL init failed: " << SDL_GetError());
        return false;
    }
    running_ = true;
    return true;
}

// === Section 2: Device registration ===

// Add a single viewer slot (one video panel) and return its index
// 添加单个viewer slot（一个视频面板）并返回其索引
int SDLViewer::addViewerSlot(const std::string& label, NioFormat fmt, int w, int h) {
    int idx = static_cast<int>(slots_.size());
    slots_.push_back(std::unique_ptr<ViewerSlot>(new ViewerSlot()));
    auto& s = *slots_.back();
    s.label = label;
    s.format = fmt;
    s.formatStr = nioFormatToString(fmt);
    s.w = w;
    s.h = h;
    size_t rawMax = nioFormatRawSize(fmt, w, h);
    if (rawMax == 0)
        rawMax = w * h * 4;
    s.rawBuf.resize(rawMax, 0);
    s.renderBuf.resize(w * h * 3, 0);
    if (fmt == NioFormat::MJPG || fmt == NioFormat::MJPEG)
        s.mjpgRes = std::make_shared<MjpgDecoderRes>();
    s.decodeThread = std::thread(&SDLViewer::slotDecodeLoop, this, slots_.back().get());
    return idx;
}

int SDLViewer::addPointSlot(const std::string& label, int w, int h) {
    int idx = static_cast<int>(slots_.size());
    slots_.push_back(std::unique_ptr<ViewerSlot>(new ViewerSlot()));
    auto& s = *slots_.back();
    s.label = label;
    s.format = NioFormat::POINT;
    s.formatStr = "PCD";
    s.w = w;
    s.h = h;
    s.rawBuf.resize(12 + 6 * sizeof(PcdFieldDesc) + 27648 * 24 + 4, 0);
    s.renderBuf.resize(w * h * 3, 0);
    s.decodeThread = std::thread(&SDLViewer::slotDecodeLoop, this, slots_.back().get());
    return idx;
}

// Add a device row with selected video slots (color/depth/IR/IR-L/IR-R)
// 添加一个设备行，包含选定的视频slot（彩色/深度/红外/左红外/右红外）
int SDLViewer::addDevice(const std::string& name, const std::string& cameraType, const std::string& serialNumber,
                         bool hasColor, NioFormat colorFmt, int cw, int ch, bool hasDepth, NioFormat depthFmt, int dw,
                         int dh, bool hasIR, int irw, int irh, bool hasIRLeft, int ilw, int ilh, bool hasIRRight,
                         int irw2, int irh2, bool hasPoint, int pw, int ph) {
    DeviceRow row;
    row.name = name;
    row.cameraType = cameraType;
    row.serialNumber = serialNumber;

    if (hasColor) {
        int idx = addViewerSlot("Color", colorFmt, cw, ch);
        row.slotIndices.push_back(idx);
        row.colorSlot = idx;
    }
    if (hasDepth) {
        int idx = addViewerSlot("Depth", depthFmt, dw, dh);
        row.slotIndices.push_back(idx);
        row.depthSlot = idx;
    }
    if (hasIR) {
        int idx = addViewerSlot("IR", NioFormat::Y8, irw, irh);
        row.slotIndices.push_back(idx);
        row.irSlot = idx;
    }
    if (hasIRLeft) {
        int idx = addViewerSlot("IR-L", NioFormat::Y8, ilw, ilh);
        row.slotIndices.push_back(idx);
        row.irLeftSlot = idx;
    }
    if (hasIRRight) {
        int idx = addViewerSlot("IR-R", NioFormat::Y8, irw2, irh2);
        row.slotIndices.push_back(idx);
        row.irRightSlot = idx;
    }
    if (hasPoint) {
        int idx = addPointSlot("Point", pw, ph);
        row.slotIndices.push_back(idx);
        row.pointSlot = idx;
    }

    devices_.push_back(row);
    int devIdx = static_cast<int>(devices_.size()) - 1;
    NIO_LOG_INFO_S("SDLViewer: added device " << name << " devIdx=" << devIdx << " slots=" << row.slotIndices.size());
    return devIdx;
}

// === Section 3: Create window, textures, start threads ===

bool SDLViewer::createWindow() {
    if (initialized_ || slots_.empty())
        return false;

    maxSlotsPerRow_ = 0;
    for (auto& d : devices_)
        maxSlotsPerRow_ = std::max(maxSlotsPerRow_, static_cast<int>(d.slotIndices.size()));
    tileW_ = targetWinW_ / std::max(1, maxSlotsPerRow_);
    tileH_ = targetWinH_ / std::max(1, static_cast<int>(devices_.size()));

    SDL_DisplayMode dm;
    if (SDL_GetCurrentDisplayMode(0, &dm) != 0) {
        NIO_LOG_ERROR_S("SDL_GetCurrentDisplayMode failed: " << SDL_GetError());
        return false;
    }

    int screenW = dm.w;
    int screenH = dm.h;

    int winW = screenW * 2 / 3; // 这里可以改成 screenW / 2
    int winH = screenH * 2 / 3; // 这里可以改成 screenH / 2

    winW = std::min(winW, targetWinW_);
    winH = std::min(winH, targetWinH_);

    window_ = SDL_CreateWindow("NIO Capture Monitor", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, winW, winH,
                               SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window_) {
        NIO_LOG_ERROR_S("SDL_CreateWindow failed: " << SDL_GetError());
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer_) {
        NIO_LOG_ERROR_S("SDL_CreateRenderer failed: " << SDL_GetError());
        SDL_DestroyWindow(window_);
        window_ = nullptr;
        return false;
    }

    for (auto& s : slots_) {
        auto* tex = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, s->w, s->h);
        textures_.push_back(tex);
    }

    rebuildLabelTextures(winW, winH);

    renderThread_ = std::thread(&SDLViewer::renderLoop, this);
    initialized_ = true;
    NIO_LOG_INFO_S("SDLViewer: window created " << winW << "x" << winH << " slots=" << slots_.size()
                                                << " devices=" << devices_.size());
    return true;
}

// Destroy all cached label textures and clear vectors
// 销毁所有缓存的标签纹理并清空向量
void SDLViewer::destroyLabelTextures() {
    for (auto& lt : titleTexs_) {
        if (lt.tex)
            SDL_DestroyTexture(lt.tex);
    }
    for (auto& lt : formatTexs_) {
        if (lt.tex)
            SDL_DestroyTexture(lt.tex);
    }
    for (auto& lt : channelTexs_) {
        if (lt.tex)
            SDL_DestroyTexture(lt.tex);
    }
    titleTexs_.clear();
    formatTexs_.clear();
    channelTexs_.clear();
}

// Create a single LabelTex from text with specified fg/bg colors and font scale
// 用指定前景/背景色和字体缩放创建单个标签纹理
void SDLViewer::makeLabelTex(LabelTex& out, const std::string& text, uint8_t fgR, uint8_t fgG, uint8_t fgB, uint8_t bgR,
                             uint8_t bgG, uint8_t bgB, int fscale) {
    SDL_Surface* surf = renderBitmapText(text, fscale, fgR, fgG, fgB, bgR, bgG, bgB);
    if (surf) {
        out.tex = SDL_CreateTextureFromSurface(renderer_, surf);
        out.w = surf->w;
        out.h = surf->h;
        SDL_FreeSurface(surf);
    }
}

// Rebuild all cached label textures when window size changes
// 窗口尺寸变化时重建所有缓存的标签纹理
void SDLViewer::rebuildLabelTextures(int winW, int winH) {
    destroyLabelTextures();
    if (!renderer_)
        return;

    int numRows = static_cast<int>(devices_.size());
    if (numRows == 0 || maxSlotsPerRow_ == 0)
        return;
    int rowTotalH = ROW_HEADER_H + tileH_ + FORMAT_BAR_H;
    int totalContentH = numRows * rowTotalH;
    float scaleX = static_cast<float>(winW) / (maxSlotsPerRow_ * tileW_);
    float scaleY = static_cast<float>(winH) / totalContentH;
    float scale = std::min(scaleX, scaleY);
    int fscale = std::max(2, static_cast<int>(FONT_SCALE * scale));
    if (fscale < 1)
        fscale = 1;

    // Title textures: "型号_序列号" (white on dark bg)
    for (auto& dev : devices_) {
        std::string title = dev.cameraType + "_" + dev.serialNumber;
        LabelTex lt;
        makeLabelTex(lt, title, 255, 255, 255, 30, 30, 30, fscale);
        titleTexs_.push_back(lt);
    }

    // Channel + format label textures per slot
    for (auto& slot : slots_) {
        LabelTex clt, flt;
        makeLabelTex(clt, slot->label, 200, 200, 255, 30, 30, 30, fscale);
        makeLabelTex(flt, slot->formatStr, 100, 255, 100, 30, 30, 30, fscale);
        channelTexs_.push_back(clt);
        formatTexs_.push_back(flt);
    }

    cachedWinW_ = winW;
    cachedWinH_ = winH;
}

// === Section 4: Cleanup ===

void SDLViewer::cleanupSlot(ViewerSlot& s) {
    if (s.yuyvSws) {
        sws_freeContext(s.yuyvSws);
        s.yuyvSws = nullptr;
    }
    if (s.yuyvSrcFrame) {
        av_frame_free(&s.yuyvSrcFrame);
    }
    if (s.yuyvDstFrame) {
        av_frame_free(&s.yuyvDstFrame);
    }
    s.yuyvSwsInit = false;
}

void SDLViewer::close() {
    running_ = false;
    // Wake every slot's decode worker so they observe running_==false and exit.
    for (auto& s : slots_) {
        {
            std::lock_guard<std::mutex> lk(s->decodeMtx);
            s->decodeQueued = false;
        }
        s->decodeCv.notify_one();
    }
    for (auto& s : slots_) {
        if (s->decodeThread.joinable())
            s->decodeThread.join();
    }
    if (renderThread_.joinable())
        renderThread_.join();

    destroyLabelTextures();

    for (auto* tex : textures_) {
        if (tex)
            SDL_DestroyTexture(tex);
    }
    textures_.clear();
    for (auto& s : slots_)
        cleanupSlot(*s);

    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    if (initialized_) {
        SDL_Quit();
        initialized_ = false;
    }
}

// === Section 5: pushFrame (SDK callback — memcpy only) ===

void SDLViewer::pushFrame(int devIdx, ViewerChannel ch, const uint8_t* data, uint32_t size, float depthScale,
                          float depthMinM, float depthMaxM) {
    if (!initialized_)
        return;
    if (devIdx < 0 || devIdx >= static_cast<int>(devices_.size()))
        return;
    auto& dev = devices_[devIdx];

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
    case ViewerChannel::POINT:
        slotIdx = dev.pointSlot;
        break;
    }
    if (slotIdx < 0 || slotIdx >= static_cast<int>(slots_.size()))
        return;
    auto& slot = *slots_[slotIdx];

    {
        std::lock_guard<std::mutex> lock(slot.rawMtx);
        if (size > slot.rawBuf.size())
            return;
        memcpy(slot.rawBuf.data(), data, size);
        slot.rawSize = size;
        slot.depthScale = depthScale;
        slot.depthMinM = depthMinM;
        slot.depthMaxM = depthMaxM;
        slot.rawUpdated = true;
    }
    {
        std::lock_guard<std::mutex> lk(slot.decodeMtx);
        slot.decodeQueued = true;
    }
    slot.decodeCv.notify_one();
}

// === Section 6: Decode thread (raw → RGB24 conversion) ===

// Decode Y16 depth frame → jet colormap RGB
// Y16深度帧解码 → jet色图RGB
bool SDLViewer::decodeY16Slot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                              std::vector<uint8_t>& rgb) {
    if (rawSz < static_cast<uint32_t>(w * h * 2))
        return false;
    const uint16_t* y16 = reinterpret_cast<const uint16_t*>(rawCopy.data());
    depthY16ToJetRgb(y16, w, h, slot.depthScale, slot.depthMinM, slot.depthMaxM, rgb.data());
    return true;
}

// Decode Y8 IR frame → grayscale RGB
// Y8红外帧解码 → 灰度RGB
bool SDLViewer::decodeY8Slot(const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                             std::vector<uint8_t>& rgb) {
    if (rawSz < static_cast<uint32_t>(w * h))
        return false;
    for (int i = 0; i < w * h; i++) {
        rgb[i * 3 + 0] = rawCopy[i];
        rgb[i * 3 + 1] = rawCopy[i];
        rgb[i * 3 + 2] = rawCopy[i];
    }
    return true;
}

// Decode YUYV frame → RGB24 via sws_scale (lazy sws init)
// YUYV帧解码 → sws_scale转RGB24（延迟初始化sws）
bool SDLViewer::decodeYuyvSlot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                               std::vector<uint8_t>& rgb) {
    std::lock_guard<std::mutex> lock(slot.rawMtx);
    if (!slot.yuyvSwsInit) {
        slot.yuyvSws =
            sws_getContext(w, h, AV_PIX_FMT_YUYV422, w, h, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!slot.yuyvSws)
            return false;
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
    if (rawSz < static_cast<uint32_t>(w * h * 2))
        return false;
    memcpy(slot.yuyvSrcFrame->data[0], rawCopy.data(), w * h * 2);
    sws_scale(slot.yuyvSws, slot.yuyvSrcFrame->data, slot.yuyvSrcFrame->linesize, 0, h, slot.yuyvDstFrame->data,
              slot.yuyvDstFrame->linesize);
    int stride = w * 3;
    if (slot.yuyvDstFrame->linesize[0] == stride) {
        memcpy(rgb.data(), slot.yuyvDstFrame->data[0], static_cast<size_t>(stride) * h);
    } else {
        for (int y = 0; y < h; y++) {
            memcpy(rgb.data() + y * stride, slot.yuyvDstFrame->data[0] + y * slot.yuyvDstFrame->linesize[0], stride);
        }
    }
    return true;
}

// Decode MJPG frame → RGB24 via MjpgDecoderRes
// MJPG帧解码 → 通过MjpgDecoderRes转RGB24
bool SDLViewer::decodeMjpgSlot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                               std::vector<uint8_t>& rgb) {
    auto mjpg = slot.mjpgRes;
    if (!mjpg)
        return false;
    if (!mjpg->swsInitialized) {
        if (!mjpg->init(w, h, NioFormat::MJPG))
            return false;
    }
    return decodeColorToRGB(rawCopy.data(), rawSz, NioFormat::MJPG, w, h, rgb.data(), mjpg);
}

bool SDLViewer::decodePointSlot(ViewerSlot& slot, const std::vector<uint8_t>& rawCopy, uint32_t rawSz, int w, int h,
                                std::vector<uint8_t>& rgb) {
    PcdLayout layout;
    uint32_t nPts = 0;
    size_t hdrBytes = PcdLayout::deserialize(rawCopy.data(), rawSz, layout, nPts);
    if (hdrBytes == 0 || nPts == 0)
        return false;

    const uint8_t* pointData = rawCopy.data() + hdrBytes;
    if (rawSz < hdrBytes + static_cast<size_t>(nPts) * layout.srcPointSize)
        return false;

    // Find xyz field offsets in the source point struct
    int xOff = -1, yOff = -1, zOff = -1, iOff = -1;
    for (const auto& f : layout.fields) {
        if (std::string(f.name) == "x")
            xOff = f.srcOffset;
        else if (std::string(f.name) == "y")
            yOff = f.srcOffset;
        else if (std::string(f.name) == "z")
            zOff = f.srcOffset;
        else if (std::string(f.name) == "intensity")
            iOff = f.srcOffset;
    }
    if (xOff < 0 || yOff < 0 || zOff < 0)
        return false;

    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    float minDist = 0.3f, maxDist = 50.0f;

    struct P
    {
        float x, y, z;
        uint8_t intensity;
    };
    std::vector<P> valid;
    valid.reserve(nPts);
    for (uint32_t i = 0; i < nPts; i++) {
        const uint8_t* pt = pointData + static_cast<size_t>(i) * layout.srcPointSize;
        float px, py, pz;
        memcpy(&px, pt + xOff, 4);
        memcpy(&py, pt + yOff, 4);
        memcpy(&pz, pt + zOff, 4);
        if (std::isnan(px) || std::isnan(py) || std::isnan(pz))
            continue;
        uint8_t intens = (iOff >= 0) ? pt[iOff] : 0;
        valid.push_back({ px, py, pz, intens });
        if (px < minX)
            minX = px;
        if (px > maxX)
            maxX = px;
        if (py < minY)
            minY = py;
        if (py > maxY)
            maxY = py;
    }

    memset(rgb.data(), 0, w * h * 3);

    if (valid.empty())
        return true;

    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    if (rangeX < 0.01f)
        rangeX = 0.01f;
    if (rangeY < 0.01f)
        rangeY = 0.01f;

    float scaleX = (w - 1) / rangeX;
    float scaleY = (h - 1) / rangeY;
    float sc = std::min(scaleX, scaleY) * 0.9f;
    float offX = (w - rangeX * sc) * 0.5f;
    float offY = (h - rangeY * sc) * 0.5f;

    // Perspective‑like scaling: start from orthographic layout and shrink points with distance
    // Points behind the camera (z <= 0) are skipped.
    for (const auto& pt : valid) {
        if (pt.z <= 0.0f) continue; // skip points behind camera
        // Orthographic projection to pixel coordinates
        int px = static_cast<int>((pt.x - minX) * sc + offX);
        int py = static_cast<int>((pt.y - minY) * sc + offY);
        // Apply simple perspective scaling based on depth (farther points appear smaller)
        float depthFactor = 1.0f / (pt.z * 2.0f + 1.0f);
        px = static_cast<int>((px - w / 2) * depthFactor + w / 2);
        py = static_cast<int>((py - h / 2) * depthFactor + h / 2);
        // Flip Y axis for SDL coordinate system (origin top-left)
        py = h - 1 - py;
        if (px >= 0 && px < w && py >= 0 && py < h) {
            // Color based on distance for visual cue (same as previous jet colormap)
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            float norm = (dist - minDist) / (maxDist - minDist);
            norm = std::max(0.0f, std::min(1.0f, norm));
            uint8_t v = static_cast<uint8_t>(norm * 255.0f);
            uint8_t cr, cg, cb;
            jetColormap(v, cr, cg, cb);
            int idx = (py * w + px) * 3;
            rgb[idx + 0] = cr;
            rgb[idx + 1] = cg;
            rgb[idx + 2] = cb;
        }
    }
    return true;
}

// Decode raw frame data → RGB24 based on slot format, copy to renderBuf
// 根据slot格式解码原始帧数据 → RGB24，拷贝至渲染缓冲
void SDLViewer::decodeSlot(ViewerSlot& slot) {
    std::vector<uint8_t> rawCopy;
    uint32_t rawSz = 0;
    int w = slot.w, h = slot.h;
    {
        std::lock_guard<std::mutex> lock(slot.rawMtx);
        if (!slot.rawUpdated)
            return;
        rawCopy.assign(slot.rawBuf.data(), slot.rawBuf.data() + slot.rawSize);
        rawSz = slot.rawSize;
        slot.rawUpdated = false;
    }

    std::vector<uint8_t> rgb(w * h * 3, 0);
    bool ok = false;

    if (slot.format == NioFormat::Y16) {
        ok = decodeY16Slot(slot, rawCopy, rawSz, w, h, rgb);
    } else if (slot.format == NioFormat::POINT) {
        ok = decodePointSlot(slot, rawCopy, rawSz, w, h, rgb);
    } else if (slot.format == NioFormat::Y8) {
        ok = decodeY8Slot(rawCopy, rawSz, w, h, rgb);
    } else if (slot.format == NioFormat::YUYV) {
        ok = decodeYuyvSlot(slot, rawCopy, rawSz, w, h, rgb);
    } else if (slot.format == NioFormat::MJPG || slot.format == NioFormat::MJPEG) {
        ok = decodeMjpgSlot(slot, rawCopy, rawSz, w, h, rgb);
    } else if (slot.format == NioFormat::NV12 || slot.format == NioFormat::NV21 || slot.format == NioFormat::I420 ||
               slot.format == NioFormat::UYVY || slot.format == NioFormat::YUY2 || slot.format == NioFormat::RGB ||
               slot.format == NioFormat::RGB888 || slot.format == NioFormat::BGR || slot.format == NioFormat::RGBA ||
               slot.format == NioFormat::BGRA) {
        if (!slot.mjpgRes) {
            slot.mjpgRes = std::make_shared<MjpgDecoderRes>();
            slot.mjpgRes->init(w, h, slot.format);
        }
        ok = decodeColorToRGB(rawCopy.data(), rawSz, slot.format, w, h, rgb.data(), slot.mjpgRes);
    }

    if (ok) {
        std::lock_guard<std::mutex> lock(slot.renderMtx);
        memcpy(slot.renderBuf.data(), rgb.data(), w * h * 3);
        slot.renderUpdated = true;
    }
}

void SDLViewer::slotDecodeLoop(ViewerSlot* slot) {
    setThreadName("nio-sdl-dec");
    while (running_) {
        {
            std::unique_lock<std::mutex> lk(slot->decodeMtx);
            slot->decodeCv.wait_for(lk, std::chrono::milliseconds(100),
                                    [this, slot]() { return slot->decodeQueued.load() || !running_.load(); });
            slot->decodeQueued = false;
        }
        if (!running_)
            break;
        if (slot->rawUpdated)
            decodeSlot(*slot);
    }
}

// === Section 7: Render thread (textures + text overlays at 30fps) ===

// Render channel label (top-left) and format label (below tile) for one slot
// 渲染单个slot的通道标签（左上）和格式标签（瓦片下方）
void SDLViewer::renderSlotLabel(int slotIdx, int xOff, int videoY, int dstW, int dstH, float scale) {
    // Existing implementation unchanged

    if (slotIdx < static_cast<int>(channelTexs_.size()) && channelTexs_[slotIdx].tex) {
        auto& clt = channelTexs_[slotIdx];
        int clH = std::min(clt.h, static_cast<int>(ROW_HEADER_H * scale * 0.7f));
        int clW = static_cast<int>(clt.w * static_cast<float>(clH) / clt.h);
        SDL_Rect cdst = { xOff + 2, videoY + 2, clW, clH };
        SDL_RenderCopy(renderer_, clt.tex, nullptr, &cdst);
    }

    if (slotIdx < static_cast<int>(formatTexs_.size()) && formatTexs_[slotIdx].tex) {
        auto& flt = formatTexs_[slotIdx];
        int flH = std::min(flt.h, static_cast<int>(FORMAT_BAR_H * scale));
        int flW = static_cast<int>(flt.w * static_cast<float>(flH) / flt.h);
        int fmtX = xOff + (dstW - flW) / 2;
        int fmtY = videoY + dstH + 1;
        SDL_Rect fdst = { fmtX, fmtY, flW, flH };
        SDL_RenderCopy(renderer_, flt.tex, nullptr, &fdst);
    }
}

// Render one device row: title label + all slot tiles with channel/format labels
// 渲染单设备行：标题标签 + 所有slot瓦片及通道/格式标签
void SDLViewer::renderDeviceRow(int di, int rowY, float scale, int colW, int winW) {
    auto& dev = devices_[di];

    if (di < static_cast<int>(titleTexs_.size()) && titleTexs_[di].tex) {
        auto& lt = titleTexs_[di];
        int dstH = std::min(lt.h, static_cast<int>(ROW_HEADER_H * scale));
        int dstW = static_cast<int>(lt.w * static_cast<float>(dstH) / lt.h);
        if (dstW > winW)
            dstW = winW;
        SDL_Rect dst = { 0, rowY, dstW, dstH };
        SDL_RenderCopy(renderer_, lt.tex, nullptr, &dst);
    }

    int videoY = rowY + static_cast<int>(ROW_HEADER_H * scale);
    int videoH = static_cast<int>(tileH_ * scale) - static_cast<int>((ROW_HEADER_H + FORMAT_BAR_H) * scale);

    for (int si = 0; si < static_cast<int>(dev.slotIndices.size()); si++) {
        int slotIdx = dev.slotIndices[si];
        auto& slot = *slots_[slotIdx];
        auto* tex = textures_[slotIdx];

        {
            std::lock_guard<std::mutex> lock(slot.renderMtx);
            if (slot.renderUpdated) {
                SDL_UpdateTexture(tex, nullptr, slot.renderBuf.data(), slot.w * 3);
                slot.renderUpdated = false;
            }
        }

        auto [dstW, dstH] = computeSlotDisplaySize(slotIdx, colW, videoH);
        int xOff = si * colW + (colW - dstW) / 2;

        SDL_Rect dstRect = { xOff, videoY, dstW, dstH };
        SDL_RenderCopy(renderer_, tex, nullptr, &dstRect);
        renderSlotLabel(slotIdx, xOff, videoY, dstW, dstH, scale);
    }
}

// Render loop: clear screen, draw all device rows at 30fps
// 渲染循环：清屏、绘制所有设备行，30fps刷新
void SDLViewer::renderLoop() {
    setThreadName("nio-sdl-render");
    while (running_) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running_ = false;
                g_running = false;
                return;
            }
        }

        SDL_SetRenderDrawColor(renderer_, 20, 20, 20, 255);
        SDL_RenderClear(renderer_);

        int winW, winH;
        SDL_GetWindowSize(window_, &winW, &winH);
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
        float scale = 1.0f;
        if (totalContentH > winH) {
            scale = static_cast<float>(winH) / totalContentH;
        }

        for (int di = 0; di < numRows; di++) {
            int rowY = static_cast<int>(di * rowTotalH * scale);
            int slotsInRow = static_cast<int>(devices_[di].slotIndices.size());
            int colW = (slotsInRow > 0) ? (winW / slotsInRow) : winW;
            renderDeviceRow(di, rowY, scale, colW, winW);
        }

        SDL_RenderPresent(renderer_);
        SDL_Delay(1000 / RENDER_FPS);
    }
}

// Compute the displayed size of a slot while preserving aspect ratio
std::pair<int, int> SDLViewer::computeSlotDisplaySize(int slotIdx, int slotW, int slotH) const {
    if (slotIdx < 0 || slotIdx >= static_cast<int>(slots_.size()))
        return { 0, 0 };
    const auto& slot = *slots_[slotIdx];
    int dstW = 0, dstH = 0;
    if (slot.w > 0 && slot.h > 0 && slotW > 0 && slotH > 0) {
        float aspScale = std::min(static_cast<float>(slotW) / slot.w, static_cast<float>(slotH) / slot.h);
        dstW = static_cast<int>(slot.w * aspScale);
        dstH = static_cast<int>(slot.h * aspScale);
    }
    return { dstW, dstH };
}

} // namespace nio
