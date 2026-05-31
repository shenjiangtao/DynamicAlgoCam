#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"
#include "nio_log.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <mutex>
#include <thread>
#include <atomic>
#include <map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <csignal>
#include <sys/stat.h>
#include <chrono>
#include <cmath>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

static std::atomic<bool> g_running{true};
static void signalHandler(int) { g_running = false; }

static std::string getTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return std::to_string(ms);
}

static uint64_t getTimestampMsInt() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

static std::string getTimestampIso() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    time_t secs = static_cast<time_t>(ms / 1000);
    int millis = static_cast<int>(ms % 1000);
    struct tm t;
    localtime_r(&secs, &t);
    char buf[64];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
        t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
        t.tm_hour, t.tm_min, t.tm_sec, millis);
    return std::string(buf);
}

static void mkdirp(const std::string &path) {
    size_t pos = 0;
    std::string tmp;
    while((pos = path.find('/', pos + 1)) != std::string::npos) {
        tmp = path.substr(0, pos);
        mkdir(tmp.c_str(), 0755);
    }
    mkdir(path.c_str(), 0755);
}

static void jetColormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
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

struct MjpgDecoderRes {
    AVCodecContext *ctx;
    AVPacket *pkt;
    AVFrame *decFrame;
    SwsContext *sws;

    MjpgDecoderRes() : ctx(nullptr), pkt(nullptr), decFrame(nullptr), sws(nullptr) {}

    ~MjpgDecoderRes() {
        if(sws) sws_freeContext(sws);
        if(decFrame) av_frame_free(&decFrame);
        if(pkt) av_packet_free(&pkt);
        if(ctx) avcodec_free_context(&ctx);
    }

    bool init(int w, int h, OBFormat fmt) {
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
                w, h, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
        }
        return true;
    }
};

static const char *SEI_COPYRIGHT = "Copyright jiangtao.shen@nio.com";

static void writeSEINalUnit(std::ofstream &outFile, const std::string &payload, std::mutex &mtx) {
    std::vector<uint8_t> rbsp;
    const char *uuid = "nio@orbbec-fusio";
    for(int i = 0; i < 16; i++) rbsp.push_back(static_cast<uint8_t>(uuid[i]));
    for(size_t i = 0; i < payload.size(); i++) rbsp.push_back(static_cast<uint8_t>(payload[i]));

    size_t payloadSize = rbsp.size();

    std::vector<uint8_t> nal;
    nal.push_back(0x00); nal.push_back(0x00);
    nal.push_back(0x00); nal.push_back(0x01);
    nal.push_back(0x06);
    nal.push_back(0x05);

    while(payloadSize >= 255) { nal.push_back(0xFF); payloadSize -= 255; }
    nal.push_back(static_cast<uint8_t>(payloadSize));

    int zeroCount = 0;
    for(size_t i = 0; i < rbsp.size(); i++) {
        uint8_t b = rbsp[i];
        if(zeroCount >= 2 && b <= 0x03) {
            nal.push_back(0x03);
            zeroCount = 0;
        }
        nal.push_back(b);
        if(b == 0x00) zeroCount++; else zeroCount = 0;
    }

    nal.push_back(0x80);

    {
        std::lock_guard<std::mutex> lock(mtx);
        outFile.write(reinterpret_cast<const char *>(nal.data()), nal.size());
    }
}

static void drawChar5x7(uint8_t *buf, int bufW, int bufH, int x0, int y0, char c,
    uint8_t r, uint8_t g, uint8_t b) {
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
    int idx = c - ' ';
    if(idx < 0 || idx >= (int)(sizeof(font5x7) / sizeof(font5x7[0]))) return;
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

static void drawText5x7(uint8_t *buf, int bufW, int bufH, int x0, int y0,
    const std::string &text, uint8_t r, uint8_t g, uint8_t b) {
    int cx = x0;
    for(size_t i = 0; i < text.size(); i++) {
        drawChar5x7(buf, bufW, bufH, cx, y0, text[i], r, g, b);
        cx += 6;
    }
}

class H264Encoder {
public:
    H264Encoder() : codecCtx_(nullptr), frame_(nullptr), pkt_(nullptr), swsCtx_(nullptr),
        pts_(0), width_(0), height_(0), initialized_(false), seiWritten_(false) {}
    ~H264Encoder() { close(); }

    bool init(int width, int height, int fps) {
        width_ = width;
        height_ = height;
        const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if(!codec) return false;
        codecCtx_ = avcodec_alloc_context3(codec);
        if(!codecCtx_) return false;
        codecCtx_->bit_rate = 6000000;
        codecCtx_->width = width;
        codecCtx_->height = height;
        AVRational tb = {1, fps}; AVRational fr = {fps, 1};
        codecCtx_->time_base = tb; codecCtx_->framerate = fr;
        codecCtx_->gop_size = fps;
        codecCtx_->max_b_frames = 0;
        codecCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
        codecCtx_->qmin = 10; codecCtx_->qmax = 30;
        av_opt_set(codecCtx_->priv_data, "preset", "ultrafast", 0);
        av_opt_set(codecCtx_->priv_data, "tune", "zerolatency", 0);
        if(avcodec_open2(codecCtx_, codec, nullptr) < 0) {
            avcodec_free_context(&codecCtx_); codecCtx_ = nullptr; return false;
        }
        frame_ = av_frame_alloc();
        if(!frame_) { close(); return false; }
        frame_->format = AV_PIX_FMT_YUV420P;
        frame_->width = width; frame_->height = height;
        if(av_frame_get_buffer(frame_, 0) < 0) { close(); return false; }
        pkt_ = av_packet_alloc();
        if(!pkt_) { close(); return false; }
        swsCtx_ = sws_getContext(width, height, AV_PIX_FMT_RGB24,
            width, height, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!swsCtx_) { close(); return false; }
        initialized_ = true;
        return true;
    }

    void close() {
        if(swsCtx_) { sws_freeContext(swsCtx_); swsCtx_ = nullptr; }
        if(frame_) av_frame_free(&frame_);
        if(pkt_) av_packet_free(&pkt_);
        if(codecCtx_) avcodec_free_context(&codecCtx_);
        initialized_ = false;
    }

    bool encodeRGB(const uint8_t *rgbData, std::ofstream &outFile, std::mutex &mtx,
        uint64_t frameTimestampMs) {
        if(!initialized_ || !codecCtx_ || !swsCtx_) return false;
        int srcStride = width_ * 3;
        const uint8_t *srcSlice[1] = { rgbData };
        if(av_frame_make_writable(frame_) < 0) return false;
        sws_scale(swsCtx_, srcSlice, &srcStride, 0, height_, frame_->data, frame_->linesize);
        frame_->pts = pts_++;
        int ret = avcodec_send_frame(codecCtx_, frame_);
        if(ret < 0) return false;

        if(!seiWritten_) {
            writeSEINalUnit(outFile, SEI_COPYRIGHT, mtx);
            seiWritten_ = true;
        }

        std::ostringstream tsStr;
        tsStr << "ts=" << frameTimestampMs;
        writeSEINalUnit(outFile, tsStr.str(), mtx);

        bool wrote = false;
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

private:
    AVCodecContext *codecCtx_;
    AVFrame *frame_;
    AVPacket *pkt_;
    SwsContext *swsCtx_;
    int64_t pts_;
    int width_, height_;
    bool initialized_;
    bool seiWritten_;
};

static bool decodeColorToRGB(const uint8_t *data, uint32_t size, OBFormat format,
    int w, int h, uint8_t *rgbBuf, std::shared_ptr<MjpgDecoderRes> mjpg) {
    if(format == OB_FORMAT_RGB) { memcpy(rgbBuf, data, w * h * 3); return true; }
    if(format == OB_FORMAT_BGR) {
        for(int i = 0; i < w * h; i++) {
            rgbBuf[i*3+0] = data[i*3+2]; rgbBuf[i*3+1] = data[i*3+1]; rgbBuf[i*3+2] = data[i*3+0];
        }
        return true;
    }
    if(format == OB_FORMAT_MJPG || format == OB_FORMAT_MJPEG) {
        if(!mjpg->ctx || !mjpg->sws) return false;
        mjpg->pkt->data = const_cast<uint8_t *>(data); mjpg->pkt->size = size;
        if(avcodec_send_packet(mjpg->ctx, mjpg->pkt) < 0) return false;
        if(avcodec_receive_frame(mjpg->ctx, mjpg->decFrame) < 0) return false;
        AVFrame *tmp = av_frame_alloc();
        if(!tmp) return false;
        tmp->format = AV_PIX_FMT_RGB24; tmp->width = w; tmp->height = h;
        if(av_frame_get_buffer(tmp, 0) < 0) { av_frame_free(&tmp); return false; }
        sws_scale(mjpg->sws, mjpg->decFrame->data, mjpg->decFrame->linesize, 0, h, tmp->data, tmp->linesize);
        for(int row = 0; row < h; row++) memcpy(rgbBuf + row * w * 3, tmp->data[0] + row * tmp->linesize[0], w * 3);
        av_frame_free(&tmp);
        return true;
    }
    return false;
}

static std::shared_ptr<ob::VideoStreamProfile> selectBestProfile(
    std::shared_ptr<ob::StreamProfileList> profiles, OBFormat preferredFormat) {
    std::shared_ptr<ob::VideoStreamProfile> best;
    int bestScore = -1;
    for(uint32_t i = 0; i < profiles->getCount(); i++) {
        try {
            auto sp = profiles->getProfile(i); if(!sp) continue;
            auto vsp = sp->as<ob::VideoStreamProfile>(); if(!vsp) continue;
            int score = 0;
            if(vsp->getFormat() == preferredFormat) score += 1000;
            if(vsp->getWidth() == 640) score += 100;
            else if(vsp->getWidth() == 848) score += 90;
            else if(vsp->getWidth() == 1280) score += 80;
            if(vsp->getFps() == 30) score += 50;
            else if(vsp->getFps() == 25) score += 45;
            if(score > bestScore) { bestScore = score; best = vsp; }
        } catch(...) { continue; }
    }
    if(!best && profiles->getCount() > 0) {
        try { best = profiles->getProfile(0)->as<ob::VideoStreamProfile>(); } catch(...) {}
    }
    return best;
}

struct FusionConfig {
    float depthMinM;
    float depthMaxM;
    std::vector<std::string> deviceFilter;
};

static FusionConfig parseArgs(int argc, char **argv) {
    FusionConfig cfg;
    cfg.depthMinM = 0.3f;
    cfg.depthMaxM = 5.0f;
    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--depth-min" && i + 1 < argc) cfg.depthMinM = std::stof(argv[++i]);
        else if(arg == "--depth-max" && i + 1 < argc) cfg.depthMaxM = std::stof(argv[++i]);
        else if(arg == "--help") {
            std::cout << "Usage: nio_multi_sensor_fusion [device_filter...] [options]\n"
                << "Options:\n"
                << "  --depth-min M  Min depth in meters for colormap (default: 0.3)\n"
                << "  --depth-max M  Max depth in meters for colormap (default: 5.0)\n"
                << "  --help         Show this help\n"
                << "\nFuses Color + IR_Left + Depth(jet) + IR_Right into 2x2 quad view\n"
                << "with IMU overlay and per-frame timestamp + SEI copyright\n";
            exit(0);
        }
        else if(arg.substr(0, 2) != "--") cfg.deviceFilter.push_back(arg);
    }
    return cfg;
}

static bool deviceMatches(const std::string &name, const std::vector<std::string> &filter) {
    if(filter.empty()) return true;
    for(const auto &f : filter) if(name.find(f) != std::string::npos) return true;
    return false;
}

struct IMUState {
    std::mutex mtx;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    bool valid;

    IMUState() : accelX(0), accelY(0), accelZ(0),
        gyroX(0), gyroY(0), gyroZ(0), valid(false) {}

    void update(float ax, float ay, float az, float gx, float gy, float gz) {
        std::lock_guard<std::mutex> lock(mtx);
        accelX = ax; accelY = ay; accelZ = az;
        gyroX = gx; gyroY = gy; gyroZ = gz;
        valid = true;
    }

    bool get(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        std::lock_guard<std::mutex> lock(mtx);
        ax = accelX; ay = accelY; az = accelZ;
        gx = gyroX; gy = gyroY; gz = gyroZ;
        return valid;
    }
};

struct FusionOutput {
    std::shared_ptr<H264Encoder> encoder;
    std::shared_ptr<std::ofstream> file;
    std::shared_ptr<std::mutex> mtx;
};

struct DeviceFusion {
    std::shared_ptr<ob::Pipeline> videoPipeline;
    std::shared_ptr<ob::Pipeline> imuPipeline;
    std::string deviceName;
    bool hasIMU;
    float depthScale;
    float depthMinM;
    float depthMaxM;
    OBFormat colorFormat;
    int colorW, colorH;
    int depthW, depthH;
    int irLeftW, irLeftH;
    int irRightW, irRightH;
    bool hasColor, hasDepth, hasIRLeft, hasIRRight;

    std::shared_ptr<MjpgDecoderRes> mjpgRes;
    std::shared_ptr<IMUState> imuState;
    std::shared_ptr<std::atomic<uint64_t>> fusedFrameCount;
    std::shared_ptr<FusionOutput> fusionOutput;

    std::shared_ptr<ob::Align> alignFilter;

    std::mutex latestDataMtx;
    bool latestDataReady;
    std::vector<uint8_t> latestColorRGB;
    std::vector<uint16_t> latestDepth;
    float latestScale;
    std::vector<uint8_t> latestIRLeft;
    std::vector<uint8_t> latestIRRight;

    DeviceFusion() : hasIMU(false), depthScale(0.001f), depthMinM(0.3f), depthMaxM(5.0f),
        colorFormat(OB_FORMAT_UNKNOWN),
        colorW(0), colorH(0), depthW(0), depthH(0),
        irLeftW(0), irLeftH(0), irRightW(0), irRightH(0),
        hasColor(false), hasDepth(false), hasIRLeft(false), hasIRRight(false),
        latestDataReady(false), latestScale(0.001f) {}
};

static void fillQuadrant(uint8_t *outBuf, int outW, int outH,
    int quadX, int quadY, int quadW, int quadH,
    const uint8_t *srcRGB, int srcW, int srcH) {
    if(!srcRGB || srcW <= 0 || srcH <= 0) return;
    float scaleX = static_cast<float>(srcW) / quadW;
    float scaleY = static_cast<float>(srcH) / quadH;
    for(int y = 0; y < quadH; y++) {
        int srcY = std::min(static_cast<int>(y * scaleY), srcH - 1);
        for(int x = 0; x < quadW; x++) {
            int srcX = std::min(static_cast<int>(x * scaleX), srcW - 1);
            int dstPx = quadX + x;
            int dstPy = quadY + y;
            if(dstPx >= 0 && dstPx < outW && dstPy >= 0 && dstPy < outH) {
                int di = (dstPy * outW + dstPx) * 3;
                int si = (srcY * srcW + srcX) * 3;
                outBuf[di + 0] = srcRGB[si + 0];
                outBuf[di + 1] = srcRGB[si + 1];
                outBuf[di + 2] = srcRGB[si + 2];
            }
        }
    }
}

static void fillQuadrantJetDepth(uint8_t *outBuf, int outW, int outH,
    int quadX, int quadY, int quadW, int quadH,
    const uint16_t *depthData, int depthW, int depthH, float scale,
    float depthMinM, float depthMaxM) {
    if(!depthData || depthW <= 0 || depthH <= 0) return;
    float scaleX = static_cast<float>(depthW) / quadW;
    float scaleY = static_cast<float>(depthH) / quadH;
    float minDist = depthMinM;
    float maxDist = depthMaxM;
    for(int y = 0; y < quadH; y++) {
        int srcY = std::min(static_cast<int>(y * scaleY), depthH - 1);
        for(int x = 0; x < quadW; x++) {
            int srcX = std::min(static_cast<int>(x * scaleX), depthW - 1);
            int dstPx = quadX + x;
            int dstPy = quadY + y;
            if(dstPx >= 0 && dstPx < outW && dstPy >= 0 && dstPy < outH) {
                int di = (dstPy * outW + dstPx) * 3;
                uint16_t rawVal = depthData[srcY * depthW + srcX];
                if(rawVal == 0) {
                    outBuf[di + 0] = 0; outBuf[di + 1] = 0; outBuf[di + 2] = 0;
                } else {
                    float distM = rawVal * scale / 1000.0f;
                    float norm = (distM - minDist) / (maxDist - minDist);
                    norm = std::max(0.0f, std::min(1.0f, norm));
                    uint8_t v8 = static_cast<uint8_t>(norm * 255.0f);
                    uint8_t cr, cg, cb;
                    jetColormap(v8, cr, cg, cb);
                    outBuf[di + 0] = cr; outBuf[di + 1] = cg; outBuf[di + 2] = cb;
                }
            }
        }
    }
}

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    FusionConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_multi_sensor_fusion", "multi_sensor_fusion_output");
    NIO_LOG_SET_LEVEL(nio::LogLevel::TRACE);
    NIO_LOG_INFO_S("Process started, depthMin=" << cfg.depthMinM << " depthMax=" << cfg.depthMaxM
        << " device_filter_count=" << cfg.deviceFilter.size());

    ob::Context context;
    auto deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1) {
        std::cerr << "No Orbbec device found!" << std::endl;
        NIO_LOG_FATAL("No Orbbec device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputRootDir = "multi_sensor_fusion_output/" + sessionTimestamp;
    mkdirp(outputRootDir);
    NIO_LOG_INFO_S("Session timestamp=" << sessionTimestamp << " outputDir=" << outputRootDir);

    std::vector<std::shared_ptr<DeviceFusion>> fusions;

    for(uint32_t i = 0; i < deviceList->getCount(); i++) {
        auto device = deviceList->getDevice(i);
        auto devInfo = device->getDeviceInfo();
        std::string name = devInfo->getName();

        if(!deviceMatches(name, cfg.deviceFilter)) {
            std::cout << "Skipping device: " << name << std::endl;
            NIO_LOG_DEBUG_S("Skipping device: " << name);
            continue;
        }

        std::cout << "Found device: " << name
            << " (SN: " << devInfo->getSerialNumber()
            << ", PID: 0x" << std::hex << std::setw(4) << std::setfill('0')
            << devInfo->getPid() << std::dec << ")" << std::endl;
        NIO_LOG_INFO_S("Found device: " << name << " SN=" << devInfo->getSerialNumber()
            << " PID=0x" << std::hex << devInfo->getPid() << std::dec);

        auto safeName = name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');

        auto df = std::make_shared<DeviceFusion>();
        df->deviceName = safeName;
        df->depthMinM = cfg.depthMinM;
        df->depthMaxM = cfg.depthMaxM;
        df->fusedFrameCount = std::make_shared<std::atomic<uint64_t>>(0);
        df->imuState = std::make_shared<IMUState>();

        try { device->timerSyncWithHost(); }
        catch(ob::Error &e) {
            std::cerr << " Timer sync warning: " << e.what() << std::endl;
            NIO_LOG_WARN_S("Timer sync failed for " << safeName << ": " << e.what());
        }
        if(device->isGlobalTimestampSupported()) {
            try { device->enableGlobalTimestamp(true); } catch(...) {}
        }

        df->videoPipeline = std::make_shared<ob::Pipeline>(device);
        auto config = std::make_shared<ob::Config>();
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

        auto sensorList = device->getSensorList();
        bool hasColor = false, hasDepth = false;
        bool hasIRLeft = false, hasIRRight = false;
        bool hasAccel = false, hasGyro = false;
        OBFormat colorFormat = OB_FORMAT_UNKNOWN;
        OBFormat depthFormat = OB_FORMAT_UNKNOWN;
        OBFormat irLeftFormat = OB_FORMAT_UNKNOWN;
        OBFormat irRightFormat = OB_FORMAT_UNKNOWN;
        int colorW = 0, colorH = 0, colorFps = 30;
        int depthW = 0, depthH = 0, depthFps = 30;
        int irLW = 0, irLH = 0, irLFps = 30;
        int irRW = 0, irRH = 0, irRFps = 30;
        std::shared_ptr<ob::VideoStreamProfile> colorProfile, depthProfile, irLeftProfile, irRightProfile;

        for(uint32_t s = 0; s < sensorList->getCount(); s++) {
            auto sensorType = sensorList->getSensorType(s);
            auto sensor = sensorList->getSensor(s);
            auto profileList = sensor->getStreamProfileList();

            switch(sensorType) {
            case OB_SENSOR_COLOR:
                hasColor = true;
                colorProfile = selectBestProfile(profileList, OB_FORMAT_MJPG);
                if(colorProfile) {
                    colorFormat = colorProfile->getFormat();
                    if(colorFormat == OB_FORMAT_UNKNOWN) {
                        for(uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if(p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    colorProfile = p; colorFormat = p->getFormat(); break;
                                }
                            } catch(...) {}
                        }
                    }
                    if(colorFormat != OB_FORMAT_UNKNOWN) {
                        config->enableStream(colorProfile);
                        colorW = colorProfile->getWidth(); colorH = colorProfile->getHeight();
                        colorFps = colorProfile->getFps();
                    } else { hasColor = false; }
                } else { hasColor = false; }
                if(hasColor) std::cout << " Color: " << colorW << "x" << colorH << "@" << colorFps << " fmt=" << colorFormat << std::endl;
    if(hasColor) NIO_LOG_INFO_S("Color stream: " << colorW << "x" << colorH << "@" << colorFps << " fmt=" << colorFormat);
                break;
            case OB_SENSOR_DEPTH:
                hasDepth = true;
                depthProfile = selectBestProfile(profileList, OB_FORMAT_Y16);
                if(depthProfile) {
                    depthFormat = depthProfile->getFormat();
                    if(depthFormat == OB_FORMAT_UNKNOWN) {
                        for(uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if(p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    depthProfile = p; depthFormat = p->getFormat(); break;
                                }
                            } catch(...) {}
                        }
                    }
                    if(depthFormat != OB_FORMAT_UNKNOWN) {
                        config->enableStream(depthProfile);
                        depthW = depthProfile->getWidth(); depthH = depthProfile->getHeight();
                        depthFps = depthProfile->getFps();
                    } else { hasDepth = false; }
                } else { hasDepth = false; }
                if(hasDepth) std::cout << " Depth: " << depthW << "x" << depthH << "@" << depthFps << " fmt=" << depthFormat << std::endl;
    if(hasDepth) NIO_LOG_INFO_S("Depth stream: " << depthW << "x" << depthH << "@" << depthFps << " fmt=" << depthFormat << " scale=" << df->depthScale);
                try {
                    int32_t pl = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch(pl) {
                        case 0: df->depthScale = 0.001f; break;
                        case 1: df->depthScale = 0.0005f; break;
                        case 2: df->depthScale = 0.00025f; break;
                        case 3: df->depthScale = 0.0001f; break;
                        default: df->depthScale = 0.001f; break;
                    }
                    std::cout << " Depth scale: " << df->depthScale << std::endl;
                } catch(...) { df->depthScale = 0.001f; }
                break;
            case OB_SENSOR_IR_LEFT:
                hasIRLeft = true;
                irLeftProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irLeftProfile) {
                    irLeftFormat = irLeftProfile->getFormat();
                    if(irLeftFormat == OB_FORMAT_UNKNOWN) irLeftFormat = OB_FORMAT_Y8;
                    config->enableStream(irLeftProfile);
                    irLW = irLeftProfile->getWidth(); irLH = irLeftProfile->getHeight();
                    irLFps = irLeftProfile->getFps();
                } else { hasIRLeft = false; }
                if(hasIRLeft) std::cout << " IR Left: " << irLW << "x" << irLH << "@" << irLFps << " fmt=" << irLeftFormat << std::endl;
    if(hasIRLeft) NIO_LOG_INFO_S("IR Left stream: " << irLW << "x" << irLH << "@" << irLFps << " fmt=" << irLeftFormat);
                break;
            case OB_SENSOR_IR_RIGHT:
                hasIRRight = true;
                irRightProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irRightProfile) {
                    irRightFormat = irRightProfile->getFormat();
                    if(irRightFormat == OB_FORMAT_UNKNOWN) irRightFormat = OB_FORMAT_Y8;
                    config->enableStream(irRightProfile);
                    irRW = irRightProfile->getWidth(); irRH = irRightProfile->getHeight();
                    irRFps = irRightProfile->getFps();
                } else { hasIRRight = false; }
                if(hasIRRight) std::cout << " IR Right: " << irRW << "x" << irRH << "@" << irRFps << " fmt=" << irRightFormat << std::endl;
    if(hasIRRight) NIO_LOG_INFO_S("IR Right stream: " << irRW << "x" << irRH << "@" << irRFps << " fmt=" << irRightFormat);
                break;
            case OB_SENSOR_ACCEL: hasAccel = true; break;
            case OB_SENSOR_GYRO: hasGyro = true; break;
            default: break;
            }
        }

        auto pid = devInfo->getPid(); auto vid = devInfo->getVid();
        if(ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
            hasIRLeft = false;
            std::cout << " Gemini 305g: disabled IR_LEFT" << std::endl;
        NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
        }

    if(!hasDepth) {
        std::cerr << " " << safeName << " has no depth sensor, skipping" << std::endl;
        NIO_LOG_WARN_S(safeName << " has no depth sensor, skipping multi-sensor fusion");
        continue;
        }

        df->hasColor = hasColor;
        df->hasDepth = hasDepth;
        df->hasIRLeft = hasIRLeft;
        df->hasIRRight = hasIRRight;
        df->colorFormat = colorFormat;
        df->colorW = colorW; df->colorH = colorH;
        df->depthW = depthW; df->depthH = depthH;
        df->irLeftW = irLW; df->irLeftH = irLH;
        df->irRightW = irRW; df->irRightH = irRH;

        df->mjpgRes = std::make_shared<MjpgDecoderRes>();
        df->mjpgRes->init(colorW, colorH, colorFormat);

        if(hasColor && hasDepth) {
            df->alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        }

        try { df->videoPipeline->enableFrameSync(); } catch(...) {}

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);

        std::string startTs = getTimestampMs();
        std::string fusedPath = deviceOutputDir + "/" + safeName + "_multi_fused_" + startTs + ".h264";
        auto fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);
        auto fusedMtx = std::make_shared<std::mutex>();

        int outW = colorW > 0 ? colorW * 2 : depthW * 2;
        int outH = colorH > 0 ? colorH * 2 : depthH * 2;
        int outFps = std::min(colorFps, depthFps);

        auto fusedEncoder = std::make_shared<H264Encoder>();
    if(!fusedEncoder->init(outW, outH, outFps)) {
        std::cerr << " Failed to init H264 encoder for " << safeName << std::endl;
        NIO_LOG_ERROR_S("Failed to init H264 encoder for " << safeName << " " << outW << "x" << outH << "@" << outFps);
        continue;
        }

        auto fusedRGB = std::make_shared<std::vector<uint8_t>>(outW * outH * 3, 0);
        auto colorRGB = std::make_shared<std::vector<uint8_t>>(outW * outH * 3, 0);
        auto depthJetRGB = std::make_shared<std::vector<uint8_t>>(outW * outH * 3, 0);
        auto irLeftRGB = std::make_shared<std::vector<uint8_t>>(outW * outH * 3, 0);
        auto irRightRGB = std::make_shared<std::vector<uint8_t>>(outW * outH * 3, 0);

        auto dfCapture = df;
        try {
            df->videoPipeline->start(config,
                [dfCapture, colorRGB, depthJetRGB, irLeftRGB, irRightRGB,
                 fusedRGB, fusedEncoder, fusedFile, fusedMtx, colorFormat,
                 irLeftFormat, irRightFormat,
                 hasColor, hasDepth, hasIRLeft, hasIRRight, outW, outH]
                (std::shared_ptr<ob::FrameSet> frameSet) {
                    if(!frameSet) return;

                    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                    if(!depthFrame) return;

                    auto alignedFS = frameSet;
                    if(dfCapture->alignFilter && hasColor) {
                        auto aligned = dfCapture->alignFilter->process(frameSet);
                        if(aligned) {
                            auto fs = std::dynamic_pointer_cast<ob::FrameSet>(aligned);
                            if(fs) alignedFS = fs;
                        }
                    }

                    memset(colorRGB->data(), 0, outW * outH * 3);
                    memset(depthJetRGB->data(), 0, outW * outH * 3);
                    memset(irLeftRGB->data(), 0, outW * outH * 3);
                    memset(irRightRGB->data(), 0, outW * outH * 3);

                    if(hasColor) {
                        auto colorFrame = alignedFS->getFrame(OB_FRAME_COLOR);
                        if(colorFrame) {
                            decodeColorToRGB(colorFrame->getData(), colorFrame->getDataSize(),
                                colorFormat, dfCapture->colorW, dfCapture->colorH,
                                colorRGB->data(), dfCapture->mjpgRes);
                        }
                    }

                    float scale = dfCapture->depthScale;
                    try {
                        auto df2 = depthFrame->as<ob::DepthFrame>();
                        if(df2) scale = df2->getValueScale();
                    } catch(...) {}

                    {
                        const uint16_t *depthPtr = reinterpret_cast<const uint16_t *>(depthFrame->getData());
                        int dW = dfCapture->depthW;
                        int dH = dfCapture->depthH;
                        float minDist = dfCapture->depthMinM;
                        float maxDist = dfCapture->depthMaxM;
                        for(int y = 0; y < dH; y++) {
                            for(int x = 0; x < dW; x++) {
                                uint16_t rawVal = depthPtr[y * dW + x];
                                int idx = (y * dW + x) * 3;
                                if(rawVal == 0) {
                                    (*depthJetRGB)[idx + 0] = 0;
                                    (*depthJetRGB)[idx + 1] = 0;
                                    (*depthJetRGB)[idx + 2] = 0;
                                } else {
                                    float distM = rawVal * scale / 1000.0f;
                                    float norm = (distM - minDist) / (maxDist - minDist);
                                    norm = std::max(0.0f, std::min(1.0f, norm));
                                    uint8_t v8 = static_cast<uint8_t>(norm * 255.0f);
                                    uint8_t cr, cg, cb;
                                    jetColormap(v8, cr, cg, cb);
                                    (*depthJetRGB)[idx + 0] = cr;
                                    (*depthJetRGB)[idx + 1] = cg;
                                    (*depthJetRGB)[idx + 2] = cb;
                                }
                            }
                        }
                    }

            if(hasIRLeft) {
                auto irLFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
                if(irLFrame) {
                    const uint8_t *irData = irLFrame->getData();
                    int irW = dfCapture->irLeftW;
                    int irH = dfCapture->irLeftH;
                    if(irLeftFormat == OB_FORMAT_Y16 || irLeftFormat == OB_FORMAT_YUY2 || irLeftFormat == OB_FORMAT_UYVY) {
                        const uint16_t *ir16 = reinterpret_cast<const uint16_t *>(irData);
                        for(int y = 0; y < irH; y++) {
                            for(int x = 0; x < irW; x++) {
                                uint8_t v = static_cast<uint8_t>(ir16[y * irW + x] >> 8);
                                int idx = (y * irW + x) * 3;
                                (*irLeftRGB)[idx + 0] = v;
                                (*irLeftRGB)[idx + 1] = v;
                                (*irLeftRGB)[idx + 2] = v;
                            }
                        }
                    } else {
                        for(int y = 0; y < irH; y++) {
                            for(int x = 0; x < irW; x++) {
                                uint8_t v = irData[y * irW + x];
                                int idx = (y * irW + x) * 3;
                                (*irLeftRGB)[idx + 0] = v;
                                (*irLeftRGB)[idx + 1] = v;
                                (*irLeftRGB)[idx + 2] = v;
                            }
                        }
                    }
                }
            }

            if(hasIRRight) {
                auto irRFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
                if(irRFrame) {
                    const uint8_t *irData = irRFrame->getData();
                    int irW = dfCapture->irRightW;
                    int irH = dfCapture->irRightH;
                    if(irRightFormat == OB_FORMAT_Y16 || irRightFormat == OB_FORMAT_YUY2 || irRightFormat == OB_FORMAT_UYVY) {
                        const uint16_t *ir16 = reinterpret_cast<const uint16_t *>(irData);
                        for(int y = 0; y < irH; y++) {
                            for(int x = 0; x < irW; x++) {
                                uint8_t v = static_cast<uint8_t>(ir16[y * irW + x] >> 8);
                                int idx = (y * irW + x) * 3;
                                (*irRightRGB)[idx + 0] = v;
                                (*irRightRGB)[idx + 1] = v;
                                (*irRightRGB)[idx + 2] = v;
                            }
                        }
                    } else {
                        for(int y = 0; y < irH; y++) {
                            for(int x = 0; x < irW; x++) {
                                uint8_t v = irData[y * irW + x];
                                int idx = (y * irW + x) * 3;
                                (*irRightRGB)[idx + 0] = v;
                                (*irRightRGB)[idx + 1] = v;
                                (*irRightRGB)[idx + 2] = v;
                            }
                        }
                    }
                }
            }

                    int halfW = outW / 2;
                    int halfH = outH / 2;

                    memset(fusedRGB->data(), 0, outW * outH * 3);

                    if(hasColor)
                        fillQuadrant(fusedRGB->data(), outW, outH, 0, 0, halfW, halfH,
                            colorRGB->data(), dfCapture->colorW, dfCapture->colorH);
                    else
                        fillQuadrantJetDepth(fusedRGB->data(), outW, outH, 0, 0, halfW, halfH,
                            reinterpret_cast<const uint16_t *>(depthFrame->getData()),
                            dfCapture->depthW, dfCapture->depthH, scale,
                            dfCapture->depthMinM, dfCapture->depthMaxM);

            if(hasIRLeft)
                fillQuadrant(fusedRGB->data(), outW, outH, halfW, 0, halfW, halfH,
                    irLeftRGB->data(), dfCapture->irLeftW, dfCapture->irLeftH);
            else
                        fillQuadrant(fusedRGB->data(), outW, outH, halfW, 0, halfW, halfH,
                            depthJetRGB->data(), dfCapture->depthW, dfCapture->depthH);

                    fillQuadrantJetDepth(fusedRGB->data(), outW, outH, 0, halfH, halfW, halfH,
                        reinterpret_cast<const uint16_t *>(depthFrame->getData()),
                        dfCapture->depthW, dfCapture->depthH, scale,
                        dfCapture->depthMinM, dfCapture->depthMaxM);

            if(hasIRRight)
                fillQuadrant(fusedRGB->data(), outW, outH, halfW, halfH, halfW, halfH,
                    irRightRGB->data(), dfCapture->irRightW, dfCapture->irRightH);
            else
                        fillQuadrant(fusedRGB->data(), outW, outH, halfW, halfH, halfW, halfH,
                            depthJetRGB->data(), dfCapture->depthW, dfCapture->depthH);

                    for(int lx = 0; lx < outW; lx++) {
                        int idx1 = (halfH * outW + lx) * 3;
                        fusedRGB->at(idx1 + 0) = 255;
                        fusedRGB->at(idx1 + 1) = 255;
                        fusedRGB->at(idx1 + 2) = 255;
                    }
                    for(int ly = 0; ly < outH; ly++) {
                        int idx2 = (ly * outW + halfW) * 3;
                        fusedRGB->at(idx2 + 0) = 255;
                        fusedRGB->at(idx2 + 1) = 255;
                        fusedRGB->at(idx2 + 2) = 255;
                    }

                    uint64_t tsMs = getTimestampMsInt();
                    std::string tsIso = getTimestampIso();

                    drawText5x7(fusedRGB->data(), outW, outH, 4, 2, "COLOR", 255, 255, 255);
                    drawText5x7(fusedRGB->data(), outW, outH, halfW + 4, 2,
                        hasIRLeft ? "IR_LEFT" : "DEPTH", 255, 255, 255);
                    drawText5x7(fusedRGB->data(), outW, outH, 4, halfH + 2, "DEPTH", 255, 255, 255);
                    drawText5x7(fusedRGB->data(), outW, outH, halfW + 4, halfH + 2,
                        hasIRRight ? "IR_RIGHT" : "DEPTH", 255, 255, 255);

                    std::string tsLabel = "TS:" + tsIso + " (" + std::to_string(tsMs) + ")";
                    drawText5x7(fusedRGB->data(), outW, outH, 4, halfH - 10, tsLabel, 255, 255, 0);

                    float ax, ay, az, gx, gy, gz;
                    bool imuValid = dfCapture->imuState->get(ax, ay, az, gx, gy, gz);
                    if(imuValid) {
                        char imuBuf[128];
                        snprintf(imuBuf, sizeof(imuBuf), "IMU: A=%.2f,%.2f,%.2f G=%.1f,%.1f,%.1f",
                            ax, ay, az, gx, gy, gz);
                        drawText5x7(fusedRGB->data(), outW, outH, 4, halfH - 20, imuBuf, 0, 255, 0);
                    }

                    fusedEncoder->encodeRGB(fusedRGB->data(), *fusedFile, *fusedMtx, tsMs);
                    (*dfCapture->fusedFrameCount)++;
                });
    } catch(ob::Error &e) {
        std::cerr << " Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
        NIO_LOG_ERROR_S("Pipeline start failed for " << safeName << ": " << e.what());
        df->videoPipeline.reset();
            continue;
        }

        df->hasIMU = (hasAccel && hasGyro);
        if(df->hasIMU) {
            auto imuDev = df->videoPipeline->getDevice();
            df->imuPipeline = std::make_shared<ob::Pipeline>(imuDev);
            auto imuConfig = std::make_shared<ob::Config>();
            imuConfig->enableAccelStream();
            imuConfig->enableGyroStream();
            imuConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            try {
                df->imuPipeline->start(imuConfig, [df](std::shared_ptr<ob::FrameSet> frameSet) {
                    if(!frameSet) return;
                    auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
                    auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
                    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
                    if(accelFrame) {
                        try {
                            auto af = accelFrame->as<ob::AccelFrame>();
                            if(af) { auto v = af->getValue(); ax = v.x; ay = v.y; az = v.z; }
                        } catch(...) {}
                    }
                    if(gyroFrame) {
                        try {
                            auto gf = gyroFrame->as<ob::GyroFrame>();
                            if(gf) { auto v = gf->getValue(); gx = v.x; gy = v.y; gz = v.z; }
                        } catch(...) {}
                    }
                    df->imuState->update(ax, ay, az, gx, gy, gz);
                });
    } catch(ob::Error &e) {
        std::cerr << " IMU pipeline start failed: " << e.what() << std::endl;
        NIO_LOG_WARN_S("IMU pipeline start failed for " << safeName << ": " << e.what());
        df->hasIMU = false;
            }
        }

        auto fout = std::make_shared<FusionOutput>();
        fout->encoder = fusedEncoder;
        fout->file = fusedFile;
        fout->mtx = fusedMtx;
        df->fusionOutput = fout;

        fusions.push_back(df);
    }

    if(fusions.empty()) {
        std::cerr << "No suitable devices found for multi-sensor fusion!" << std::endl;
        NIO_LOG_FATAL("No suitable devices found for multi-sensor fusion!");
        return -1;
    }

    std::cout << "\n=== Multi-Sensor Fusion recording started ===" << std::endl;
    std::cout << "Output: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << fusions.size() << " device(s)" << std::endl;
    std::cout << "Layout: Color | IR_Left / Depth(jet) | IR_Right" << std::endl;
    std::cout << "Overlays: timestamp + IMU + SEI copyright" << std::endl;
    std::cout << "Press Ctrl+C or 'q' to stop.\n" << std::endl;
    NIO_LOG_INFO_S("=== Multi-Sensor Fusion recording started === devices=" << fusions.size()
        << " depthRange=" << cfg.depthMinM << "m-" << cfg.depthMaxM << "m");
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    auto lastReportTime = ob_smpl::getNowTimesMs();

    while(g_running) {
        auto key = ob_smpl::waitForKeyPressed(2000);
        if(key == ESC_KEY || key == 'q' || key == 'Q') { g_running = false; break; }

        auto currentTime = ob_smpl::getNowTimesMs();
        if(currentTime >= lastReportTime + 2000) {
            uint64_t reportDuration = currentTime - lastReportTime;
            lastReportTime = currentTime;
            for(auto &df : fusions) {
                uint64_t count = df->fusedFrameCount->exchange(0);
                float rate = (reportDuration > 0) ? (count / (reportDuration / 1000.0f)) : 0.0f;
                std::cout << "[" << df->deviceName << "] Fusion FPS: "
                    << std::fixed << std::setprecision(1) << rate
                    << " | IMU: " << (df->hasIMU ? "ON" : "OFF") << std::endl;
                NIO_LOG_TRACE_S("[" << df->deviceName << "] Fusion FPS: "
                    << std::fixed << std::setprecision(1) << rate
                    << " IMU=" << (df->hasIMU ? "ON" : "OFF"));
            }
        }
    }

    std::cout << "\n=== Stopping multi-sensor fusion ===" << std::endl;
    NIO_LOG_INFO("=== Stopping multi-sensor fusion ===");
    for(auto &df : fusions) {
        if(df->videoPipeline) df->videoPipeline->stop();
        if(df->hasIMU && df->imuPipeline) df->imuPipeline->stop();
        if(df->fusionOutput) {
            if(df->fusionOutput->encoder) df->fusionOutput->encoder->close();
            if(df->fusionOutput->file) df->fusionOutput->file->close();
        }
        df->mjpgRes.reset();
        std::cout << "Stopped: " << df->deviceName << std::endl;
        NIO_LOG_INFO_S("Stopped device: " << df->deviceName);
    }

    std::cout << "Fusion outputs saved to: " << outputRootDir << "/" << std::endl;
    NIO_LOG_INFO_S("Fusion outputs saved to: " << outputRootDir << "/");
    NIO_LOG_SHUTDOWN();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "OB Error: " << e.getFunction() << "\n " << e.what()
        << "\n status: " << e.getStatus() << std::endl;
    NIO_LOG_FATAL_S("OB Error: " << e.getFunction() << " " << e.what() << " status=" << e.getStatus());
    NIO_LOG_SHUTDOWN();
    return -1;
}
catch(std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    NIO_LOG_FATAL_S("Exception: " << e.what());
    NIO_LOG_SHUTDOWN();
    return -1;
}
