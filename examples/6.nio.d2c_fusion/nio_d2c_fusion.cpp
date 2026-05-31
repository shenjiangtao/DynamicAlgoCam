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

static void mkdirp(const std::string &path) {
    size_t pos = 0;
    std::string tmp;
    while((pos = path.find('/', pos + 1)) != std::string::npos) {
        tmp = path.substr(0, pos);
        mkdir(tmp.c_str(), 0755);
    }
    mkdir(path.c_str(), 0755);
}

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

static void jetColormap(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
    float t = v / 255.0f;
    float rv, gv, bv;
    if(t < 0.125f) {
        rv = 0.0f; gv = 0.0f; bv = 0.5f + t * 4.0f;
    } else if(t < 0.375f) {
        rv = 0.0f; gv = (t - 0.125f) * 4.0f; bv = 1.0f;
    } else if(t < 0.625f) {
        rv = (t - 0.375f) * 4.0f; gv = 1.0f; bv = 1.0f - (t - 0.375f) * 4.0f;
    } else if(t < 0.875f) {
        rv = 1.0f; gv = 1.0f - (t - 0.625f) * 4.0f; bv = 0.0f;
    } else {
        rv = 1.0f - (t - 0.875f) * 4.0f; gv = 0.0f; bv = 0.0f;
    }
    r = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, rv * 255.0f)));
    g = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, gv * 255.0f)));
    b = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bv * 255.0f)));
}

struct MjpgDecoderResources {
    AVCodecContext *mjpgCtx = nullptr;
    AVPacket *mjpgPkt = nullptr;
    AVFrame *mjpgDecFrame = nullptr;
    SwsContext *swsColorToRGB = nullptr;

    MjpgDecoderResources() = default;

    ~MjpgDecoderResources() {
        if(swsColorToRGB) sws_freeContext(swsColorToRGB);
        if(mjpgDecFrame) av_frame_free(&mjpgDecFrame);
        if(mjpgPkt) av_packet_free(&mjpgPkt);
        if(mjpgCtx) avcodec_free_context(&mjpgCtx);
    }

    bool init(int width, int height, OBFormat colorFormat) {
        mjpgPkt = av_packet_alloc();
        mjpgDecFrame = av_frame_alloc();

        if(colorFormat == OB_FORMAT_MJPG || colorFormat == OB_FORMAT_MJPEG) {
            auto mjpgCodec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
            if(mjpgCodec) {
                mjpgCtx = avcodec_alloc_context3(mjpgCodec);
                if(mjpgCtx) {
                    mjpgCtx->pix_fmt = AV_PIX_FMT_YUV420P;
                    mjpgCtx->width = width;
                    mjpgCtx->height = height;
                    if(avcodec_open2(mjpgCtx, mjpgCodec, nullptr) < 0) {
                        avcodec_free_context(&mjpgCtx);
                        mjpgCtx = nullptr;
                    }
                }
            }
            swsColorToRGB = sws_getContext(width, height, AV_PIX_FMT_YUV420P,
                width, height, AV_PIX_FMT_RGB24,
                SWS_BILINEAR, nullptr, nullptr, nullptr);
        }
        return true;
    }
};

class H264Encoder {
public:
    H264Encoder() : codecCtx_(nullptr), frame_(nullptr), pkt_(nullptr), swsCtx_(nullptr),
        pts_(0), width_(0), height_(0), initialized_(false), seiWritten_(false) {}

    ~H264Encoder() { close(); }

    bool init(int width, int height, int fps) {
        width_ = width;
        height_ = height;

        const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if(!codec) {
            std::cerr << "H264 encoder not found" << std::endl;
            return false;
        }

        codecCtx_ = avcodec_alloc_context3(codec);
        if(!codecCtx_) return false;

        codecCtx_->bit_rate = 4000000;
        codecCtx_->width = width;
        codecCtx_->height = height;
        AVRational tb = {1, fps};
        AVRational fr = {fps, 1};
        codecCtx_->time_base = tb;
        codecCtx_->framerate = fr;
        codecCtx_->gop_size = fps;
        codecCtx_->max_b_frames = 0;
        codecCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
        codecCtx_->qmin = 10;
        codecCtx_->qmax = 30;

        av_opt_set(codecCtx_->priv_data, "preset", "ultrafast", 0);
        av_opt_set(codecCtx_->priv_data, "tune", "zerolatency", 0);

        if(avcodec_open2(codecCtx_, codec, nullptr) < 0) {
            std::cerr << "Failed to open H264 encoder" << std::endl;
            avcodec_free_context(&codecCtx_);
            codecCtx_ = nullptr;
            return false;
        }

        frame_ = av_frame_alloc();
        if(!frame_) { close(); return false; }
        frame_->format = AV_PIX_FMT_YUV420P;
        frame_->width = width;
        frame_->height = height;
        if(av_frame_get_buffer(frame_, 0) < 0) { close(); return false; }

        pkt_ = av_packet_alloc();
        if(!pkt_) { close(); return false; }

        // RGB24 -> YUV420P converter
        swsCtx_ = sws_getContext(width, height, AV_PIX_FMT_RGB24,
            width, height, AV_PIX_FMT_YUV420P,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!swsCtx_) {
            std::cerr << "Failed to create sws context" << std::endl;
            close();
            return false;
        }

        initialized_ = true;
        return true;
    }

    void close() {
        if(swsCtx_) { sws_freeContext(swsCtx_); swsCtx_ = nullptr; }
        if(frame_) { av_frame_free(&frame_); }
        if(pkt_) { av_packet_free(&pkt_); }
        if(codecCtx_) { avcodec_free_context(&codecCtx_); }
        initialized_ = false;
    }

    bool encodeRGB(const uint8_t *rgbData, std::ofstream &outFile, std::mutex &mtx,
                   uint64_t frameTimestampMs) {
        if(!initialized_ || !codecCtx_ || !swsCtx_) return false;

        int srcStride = width_ * 3;
        const uint8_t *srcSlice[1] = { rgbData };
        if(av_frame_make_writable(frame_) < 0) return false;
        sws_scale(swsCtx_, srcSlice, &srcStride, 0, height_,
            frame_->data, frame_->linesize);

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

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    AVCodecContext *codecCtx_; AVFrame *frame_; AVPacket *pkt_;
    SwsContext *swsCtx_; int64_t pts_; int width_, height_;
    bool initialized_; bool seiWritten_;
};

struct DeviceFusion {
    std::shared_ptr<ob::Pipeline> pipeline;
    std::string deviceName;

    std::shared_ptr<H264Encoder> fusedEncoder;
    std::shared_ptr<std::ofstream> fusedFile;
    std::mutex fusedMtx;

    std::shared_ptr<MjpgDecoderResources> mjpgRes;

    int colorW;
    int colorH;
    int fps;
    float depthScale;
    float alpha;
    float depthMinM;
    float depthMaxM;

    std::shared_ptr<std::atomic<uint64_t>> fusedFrameCount;
};

static std::shared_ptr<ob::VideoStreamProfile> selectBestProfile(
    std::shared_ptr<ob::StreamProfileList> profiles, OBFormat preferredFormat) {
    std::shared_ptr<ob::VideoStreamProfile> best;
    int bestScore = -1;

    for(uint32_t i = 0; i < profiles->getCount(); i++) {
        try {
            auto sp = profiles->getProfile(i);
            if(!sp) continue;
            auto vsp = sp->as<ob::VideoStreamProfile>();
            if(!vsp) continue;

            int score = 0;
            if(vsp->getFormat() == preferredFormat) score += 1000;
            if(vsp->getWidth() == 640) score += 100;
            else if(vsp->getWidth() == 848) score += 90;
            else if(vsp->getWidth() == 1280) score += 80;
            if(vsp->getFps() == 30) score += 50;
            else if(vsp->getFps() == 25) score += 45;
            else if(vsp->getFps() == 15) score += 30;

            if(score > bestScore) {
                bestScore = score;
                best = vsp;
            }
        } catch(...) { continue; }
    }

    if(!best && profiles->getCount() > 0) {
        try {
            auto sp = profiles->getProfile(0);
            best = sp->as<ob::VideoStreamProfile>();
        } catch(...) {}
    }
    return best;
}

static void printUsage() {
    std::cout << "Usage: nio_d2c_fusion [device_name_filter...] [options]\n"
              << "Options:\n"
              << "  --alpha VALUE    Depth overlay opacity 0.0-1.0 (default: 0.5)\n"
              << "  --depth-min M    Min depth in meters for colormap (default: 0.3)\n"
              << "  --depth-max M    Max depth in meters for colormap (default: 5.0)\n"
              << "  --help           Show this help\n"
              << "\nExample:\n"
              << "  nio_d2c_fusion                    # all devices, default alpha=0.5\n"
              << "  nio_d2c_fusion 336L --alpha 0.7   # device with '336L' in name\n"
              << std::endl;
}

struct FusionConfig {
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    std::vector<std::string> deviceFilter;
};

static FusionConfig parseArgs(int argc, char **argv) {
    FusionConfig cfg;
    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--alpha" && i + 1 < argc) {
            cfg.alpha = std::stof(argv[++i]);
            cfg.alpha = std::max(0.0f, std::min(1.0f, cfg.alpha));
        } else if(arg == "--depth-min" && i + 1 < argc) {
            cfg.depthMinM = std::stof(argv[++i]);
        } else if(arg == "--depth-max" && i + 1 < argc) {
            cfg.depthMaxM = std::stof(argv[++i]);
        } else if(arg == "--help") {
            printUsage();
            exit(0);
        } else if(arg.substr(0, 2) != "--") {
            cfg.deviceFilter.push_back(arg);
        }
    }
    return cfg;
}

static bool deviceMatches(const std::string &deviceName, const std::vector<std::string> &filter) {
    if(filter.empty()) return true;
    for(const auto &f : filter) {
        if(deviceName.find(f) != std::string::npos) return true;
    }
    return false;
}

static bool decodeColorToRGB(const uint8_t *data, uint32_t size, OBFormat format,
    int width, int height, uint8_t *rgbBuf,
    std::shared_ptr<MjpgDecoderResources> mjpgRes) {

    if(format == OB_FORMAT_RGB) {
        memcpy(rgbBuf, data, width * height * 3);
        return true;
    }

    if(format == OB_FORMAT_BGR) {
        for(int i = 0; i < width * height; i++) {
            rgbBuf[i * 3 + 0] = data[i * 3 + 2];
            rgbBuf[i * 3 + 1] = data[i * 3 + 1];
            rgbBuf[i * 3 + 2] = data[i * 3 + 0];
        }
        return true;
    }

    if(format == OB_FORMAT_RGBA) {
        for(int i = 0; i < width * height; i++) {
            rgbBuf[i * 3 + 0] = data[i * 4 + 0];
            rgbBuf[i * 3 + 1] = data[i * 4 + 1];
            rgbBuf[i * 3 + 2] = data[i * 4 + 2];
        }
        return true;
    }

    if(format == OB_FORMAT_BGRA) {
        for(int i = 0; i < width * height; i++) {
            rgbBuf[i * 3 + 0] = data[i * 4 + 2];
            rgbBuf[i * 3 + 1] = data[i * 4 + 1];
            rgbBuf[i * 3 + 2] = data[i * 4 + 0];
        }
        return true;
    }

    if(format == OB_FORMAT_MJPG || format == OB_FORMAT_MJPEG) {
        if(!mjpgRes->mjpgCtx || !mjpgRes->swsColorToRGB) return false;
        mjpgRes->mjpgPkt->data = const_cast<uint8_t *>(data);
        mjpgRes->mjpgPkt->size = size;
        int ret = avcodec_send_packet(mjpgRes->mjpgCtx, mjpgRes->mjpgPkt);
        if(ret < 0) return false;
        ret = avcodec_receive_frame(mjpgRes->mjpgCtx, mjpgRes->mjpgDecFrame);
        if(ret < 0) return false;

        AVFrame *tmpFrame = av_frame_alloc();
        if(!tmpFrame) return false;
        tmpFrame->format = AV_PIX_FMT_RGB24;
        tmpFrame->width = width;
        tmpFrame->height = height;
        if(av_frame_get_buffer(tmpFrame, 0) < 0) { av_frame_free(&tmpFrame); return false; }

        sws_scale(mjpgRes->swsColorToRGB, mjpgRes->mjpgDecFrame->data,
            mjpgRes->mjpgDecFrame->linesize,
            0, height, tmpFrame->data, tmpFrame->linesize);

        for(int row = 0; row < height; row++) {
            memcpy(rgbBuf + row * width * 3,
                tmpFrame->data[0] + row * tmpFrame->linesize[0],
                width * 3);
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

        SwsContext *tmpSws = sws_getContext(width, height, srcPixFmt,
            width, height, AV_PIX_FMT_RGB24,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!tmpSws) return false;

        AVFrame *tmpFrame = av_frame_alloc();
        if(!tmpFrame) { sws_freeContext(tmpSws); return false; }
        tmpFrame->format = AV_PIX_FMT_RGB24;
        tmpFrame->width = width;
        tmpFrame->height = height;
        if(av_frame_get_buffer(tmpFrame, 0) < 0) {
            av_frame_free(&tmpFrame);
            sws_freeContext(tmpSws);
            return false;
        }

        int srcStride[4] = {0, 0, 0, 0};
        const uint8_t *srcSlice[4] = { data, nullptr, nullptr, nullptr };

        if(format == OB_FORMAT_YUYV || format == OB_FORMAT_UYVY) {
            srcStride[0] = width * 2;
        } else if(format == OB_FORMAT_NV12 || format == OB_FORMAT_NV21) {
            srcStride[0] = width;
            srcStride[1] = width;
            srcSlice[1] = data + width * height;
        } else if(format == OB_FORMAT_I420) {
            srcStride[0] = width;
            srcStride[1] = width / 2;
            srcStride[2] = width / 2;
            srcSlice[0] = data;
            srcSlice[1] = data + width * height;
            srcSlice[2] = data + width * height * 5 / 4;
        }

        sws_scale(tmpSws, srcSlice, srcStride, 0, height,
            tmpFrame->data, tmpFrame->linesize);

        for(int row = 0; row < height; row++) {
            memcpy(rgbBuf + row * width * 3,
                tmpFrame->data[0] + row * tmpFrame->linesize[0],
                width * 3);
        }
        sws_freeContext(tmpSws);
        av_frame_free(&tmpFrame);
        return true;
    }

    return false;
}

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    FusionConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_d2c_fusion", "fusion_output");
    NIO_LOG_SET_LEVEL(nio::LogLevel::TRACE);
    NIO_LOG_INFO_S("Process started, alpha=" << cfg.alpha << " depthMin=" << cfg.depthMinM
        << " depthMax=" << cfg.depthMaxM << " device_filter_count=" << cfg.deviceFilter.size());

    ob::Context context;

    auto deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1) {
        std::cerr << "No Orbbec device found!" << std::endl;
        NIO_LOG_FATAL("No Orbbec device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputRootDir = "fusion_output/" + sessionTimestamp;
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
            << devInfo->getPid() << std::dec
            << ", " << devInfo->getConnectionType() << ")" << std::endl;
        NIO_LOG_INFO_S("Found device: " << name << " SN=" << devInfo->getSerialNumber()
            << " PID=0x" << std::hex << devInfo->getPid() << std::dec
            << " conn=" << devInfo->getConnectionType());

        auto safeName = name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);

        auto df = std::make_shared<DeviceFusion>();
        df->deviceName = safeName;
        df->alpha = cfg.alpha;
        df->depthMinM = cfg.depthMinM;
        df->depthMaxM = cfg.depthMaxM;
        df->fusedFrameCount = std::make_shared<std::atomic<uint64_t>>(0);

        try { device->timerSyncWithHost(); }
        catch(ob::Error &e) {
            std::cerr << " Timer sync warning: " << e.what() << std::endl;
            NIO_LOG_WARN_S("Timer sync failed for " << safeName << ": " << e.what());
        }

        if(device->isGlobalTimestampSupported()) {
            try { device->enableGlobalTimestamp(true); } catch(...) {}
        }

        df->pipeline = std::make_shared<ob::Pipeline>(device);
        auto config = std::make_shared<ob::Config>();

        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

        auto sensorList = device->getSensorList();
        bool hasColor = false, hasDepth = false;
        OBFormat colorFormat = OB_FORMAT_UNKNOWN;
        OBFormat depthFormat = OB_FORMAT_UNKNOWN;
        int colorW = 0, colorH = 0, colorFps = 30;
        int depthW = 0, depthH = 0, depthFps = 30;
        std::shared_ptr<ob::VideoStreamProfile> colorProfile, depthProfile;

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
                                    colorProfile = p;
                                    colorFormat = p->getFormat();
                                    break;
                                }
                            } catch(...) {}
                        }
                    }
                    if(colorFormat != OB_FORMAT_UNKNOWN) {
                        config->enableStream(colorProfile);
                        colorW = colorProfile->getWidth();
                        colorH = colorProfile->getHeight();
                        colorFps = colorProfile->getFps();
                    } else {
                        hasColor = false;
                        std::cout << "  Color: no usable format, skipping" << std::endl;
                    }
                } else {
                    hasColor = false;
                }
        if(hasColor) {
            std::cout << " Color: " << colorW << "x" << colorH
                << "@" << colorFps << " format=" << colorFormat << std::endl;
            NIO_LOG_INFO_S("Color stream: " << colorW << "x" << colorH << "@" << colorFps << " format=" << colorFormat);
        }
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
                                    depthProfile = p;
                                    depthFormat = p->getFormat();
                                    break;
                                }
                            } catch(...) {}
                        }
                    }
                    if(depthFormat != OB_FORMAT_UNKNOWN) {
                        config->enableStream(depthProfile);
                        depthW = depthProfile->getWidth();
                        depthH = depthProfile->getHeight();
                        depthFps = depthProfile->getFps();
                    } else {
                        hasDepth = false;
                        std::cout << "  Depth: no usable format, skipping" << std::endl;
                    }
                } else {
                    hasDepth = false;
                }
        if(hasDepth) {
            std::cout << " Depth: " << depthW << "x" << depthH
                << "@" << depthFps << " format=" << depthFormat << std::endl;
            NIO_LOG_INFO_S("Depth stream: " << depthW << "x" << depthH << "@" << depthFps
                << " format=" << depthFormat);
        }
                try {
                    int32_t precisionLevel = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch(precisionLevel) {
                    case 0: df->depthScale = 0.001f; break;
                    case 1: df->depthScale = 0.0005f; break;
                    case 2: df->depthScale = 0.00025f; break;
                    case 3: df->depthScale = 0.0001f; break;
                    default: df->depthScale = 0.001f; break;
                    }
            std::cout << " Depth scale: " << df->depthScale
                << " (precision " << precisionLevel << ")" << std::endl;
            NIO_LOG_INFO_S("Depth scale: " << df->depthScale << " precision=" << precisionLevel);
        } catch(...) {
            df->depthScale = 0.001f;
            std::cout << " Depth scale: 0.001 (default)" << std::endl;
            NIO_LOG_INFO("Depth scale: 0.001 (default)");
                }
                break;
            default: break;
            }
        }

    if(!hasColor || !hasDepth) {
        std::cerr << " Device " << safeName
            << " needs both color+depth for D2C fusion, skipping" << std::endl;
        NIO_LOG_WARN_S(safeName << " needs both color+depth for D2C fusion, hasColor="
            << hasColor << " hasDepth=" << hasDepth << ", skipping");
        continue;
        }

        df->colorW = colorW;
        df->colorH = colorH;
        df->fps = std::min(colorFps, depthFps);

        auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);

        std::string startTs = getTimestampMs();
        std::string fusedPath = deviceOutputDir + "/" + safeName + "_d2c_fused_" + startTs + ".h264";
        df->fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

        df->fusedEncoder = std::make_shared<H264Encoder>();
    if(!df->fusedEncoder->init(colorW, colorH, df->fps)) {
        std::cerr << " Failed to init fused H264 encoder for " << safeName << std::endl;
        NIO_LOG_ERROR_S("Failed to init fused H264 encoder for " << safeName
            << " " << colorW << "x" << colorH << "@" << df->fps);
        continue;
        }

        df->mjpgRes = std::make_shared<MjpgDecoderResources>();
        df->mjpgRes->init(colorW, colorH, colorFormat);

        int bufSize = colorW * colorH * 3;
        auto colorRGB = std::make_shared<std::vector<uint8_t>>(bufSize, 0);
        auto fusedRGB = std::make_shared<std::vector<uint8_t>>(bufSize, 0);

        auto pid = devInfo->getPid();
        auto vid = devInfo->getVid();
        if(ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
        }

        try {
            df->pipeline->enableFrameSync();
        } catch(...) {}

        try {
            df->pipeline->start(config,
                [df, alignFilter, colorRGB, fusedRGB, colorFormat]
                (std::shared_ptr<ob::FrameSet> frameSet) {
                if(!frameSet) return;

                auto alignedFrame = alignFilter->process(frameSet);
                if(!alignedFrame) return;

                auto alignedFS = std::dynamic_pointer_cast<ob::FrameSet>(alignedFrame);
                if(!alignedFS) {
                    alignedFS = frameSet;
                }

                auto colorFrame = alignedFS->getFrame(OB_FRAME_COLOR);
                auto depthFrame = alignedFS->getFrame(OB_FRAME_DEPTH);

                if(!colorFrame || !depthFrame) return;

                int w = df->colorW;
                int h = df->colorH;

                bool colorOk = decodeColorToRGB(
                    colorFrame->getData(), colorFrame->getDataSize(), colorFormat,
                    w, h, colorRGB->data(), df->mjpgRes);

                if(!colorOk) {
                    memset(colorRGB->data(), 128, w * h * 3);
                }

                auto depthData = depthFrame->getData();
                auto depthSize = depthFrame->getDataSize();
                auto depthFmt = depthFrame->getFormat();

                float scale = df->depthScale;
                try {
                    auto depthF = depthFrame->as<ob::DepthFrame>();
                    if(depthF) {
                        scale = depthF->getValueScale();
                    }
                } catch(...) {}

                int depthW = w;
                int depthH = h;
                try {
                    auto depthVF = depthFrame->as<ob::VideoFrame>();
                    if(depthVF) {
                        depthW = static_cast<int>(depthVF->getWidth());
                        depthH = static_cast<int>(depthVF->getHeight());
                    }
                } catch(...) {}

                float minDist = df->depthMinM;
                float maxDist = df->depthMaxM;
                float alpha = df->alpha;

                if(depthFmt == OB_FORMAT_Y16 && depthData && depthSize >= (uint32_t)(depthW * depthH * 2)) {
                    const uint16_t *depthPtr = reinterpret_cast<const uint16_t *>(depthData);
                    int blendW = std::min(w, depthW);
                    int blendH = std::min(h, depthH);

                    for(int y = 0; y < blendH; y++) {
                        for(int x = 0; x < blendW; x++) {
                            uint16_t rawVal = depthPtr[y * depthW + x];
                            float distM = rawVal * scale / 1000.0f;

                            if(rawVal == 0 || distM < minDist || distM > maxDist) {
                                int idx = (y * w + x) * 3;
                                (*fusedRGB)[idx + 0] = (*colorRGB)[idx + 0];
                                (*fusedRGB)[idx + 1] = (*colorRGB)[idx + 1];
                                (*fusedRGB)[idx + 2] = (*colorRGB)[idx + 2];
                            } else {
                                float norm = (distM - minDist) / (maxDist - minDist);
                                norm = std::max(0.0f, std::min(1.0f, norm));
                                uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                                uint8_t cr, cg, cb;
                                jetColormap(v, cr, cg, cb);

                                int idx = (y * w + x) * 3;
                                float inv = 1.0f - alpha;
                                (*fusedRGB)[idx + 0] = static_cast<uint8_t>(
                                    inv * (*colorRGB)[idx + 0] + alpha * cr + 0.5f);
                                (*fusedRGB)[idx + 1] = static_cast<uint8_t>(
                                    inv * (*colorRGB)[idx + 1] + alpha * cg + 0.5f);
                                (*fusedRGB)[idx + 2] = static_cast<uint8_t>(
                                    inv * (*colorRGB)[idx + 2] + alpha * cb + 0.5f);
                            }
                        }
                    }
                    for(int y = blendH; y < h; y++) {
                        memcpy(fusedRGB->data() + y * w * 3,
                               colorRGB->data() + y * w * 3, w * 3);
                    }
                } else {
                    memcpy(fusedRGB->data(), colorRGB->data(), w * h * 3);
                }

                df->fusedEncoder->encodeRGB(fusedRGB->data(), *df->fusedFile, df->fusedMtx, getTimestampMsInt());
                (*df->fusedFrameCount)++;
            });
    } catch(ob::Error &e) {
        std::cerr << " Pipeline start failed for " << safeName
            << ": " << e.what() << std::endl;
        NIO_LOG_ERROR_S("Pipeline start failed for " << safeName << ": " << e.what());
        df->pipeline.reset();
            continue;
        }

        fusions.push_back(df);
    }

    if(fusions.empty()) {
        std::cerr << "No suitable devices found for D2C fusion!" << std::endl;
        NIO_LOG_FATAL("No suitable devices found for D2C fusion!");
        if(!cfg.deviceFilter.empty()) {
            std::cerr << "Available devices:" << std::endl;
            for(uint32_t i = 0; i < deviceList->getCount(); i++) {
                auto dev = deviceList->getDevice(i);
                std::cerr << "  - " << dev->getDeviceInfo()->getName() << std::endl;
            }
        }
        return -1;
    }

    std::cout << "\n=== D2C Fusion recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << fusions.size() << " device(s)" << std::endl;
    std::cout << "Alpha: " << cfg.alpha << ", Depth range: "
        << cfg.depthMinM << "m - " << cfg.depthMaxM << "m" << std::endl;
    std::cout << "Press Ctrl+C or 'q' to stop.\n" << std::endl;
    NIO_LOG_INFO_S("=== D2C Fusion recording started === devices=" << fusions.size()
        << " alpha=" << cfg.alpha << " depthRange=" << cfg.depthMinM << "m-" << cfg.depthMaxM << "m");
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    auto lastReportTime = ob_smpl::getNowTimesMs();

    while(g_running) {
        auto key = ob_smpl::waitForKeyPressed(2000);
        if(key == ESC_KEY || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        auto currentTime = ob_smpl::getNowTimesMs();
        if(currentTime >= lastReportTime + 2000) {
            uint64_t reportDuration = currentTime - lastReportTime;
            lastReportTime = currentTime;

            for(auto &df : fusions) {
                uint64_t count = df->fusedFrameCount->exchange(0);
                float rate = (reportDuration > 0) ? (count / (reportDuration / 1000.0f)) : 0.0f;
                std::cout << "[" << df->deviceName << "] Fusion FPS: "
                    << std::fixed << std::setprecision(1) << rate << std::endl;
                NIO_LOG_TRACE_S("[" << df->deviceName << "] Fusion FPS: " << std::fixed << std::setprecision(1) << rate);
            }
        }
    }

    std::cout << "\n=== Stopping fusion recording ===" << std::endl;
    NIO_LOG_INFO("=== Stopping fusion recording ===");

    for(auto &df : fusions) {
        if(df->pipeline) df->pipeline->stop();
        if(df->fusedEncoder) df->fusedEncoder->close();
        if(df->fusedFile) df->fusedFile->close();
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
