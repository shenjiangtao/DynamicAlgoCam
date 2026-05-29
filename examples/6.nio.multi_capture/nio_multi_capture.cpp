#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"

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

static void mkdirp(const std::string &path) {
    size_t pos = 0;
    std::string tmp;
    while((pos = path.find('/', pos + 1)) != std::string::npos) {
        tmp = path.substr(0, pos);
        mkdir(tmp.c_str(), 0755);
    }
    mkdir(path.c_str(), 0755);
}

static bool isH264KeyFrame(const uint8_t *data, uint32_t size) {
    if(size < 5) return false;
    const uint8_t *ptr = data;
    const uint8_t *end = data + size;
    while(ptr + 4 < end) {
        if(ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1) {
            uint8_t nalType = ptr[4] & 0x1F;
            if(nalType == 5 || nalType == 7 || nalType == 8) return true;
        }
        ptr++;
    }
    return false;
}

static void writeH264StartCode(std::ofstream &f) {
    const uint8_t sc[] = {0x00, 0x00, 0x00, 0x01};
    f.write(reinterpret_cast<const char *>(sc), 4);
}

static void writeH264Frame(std::ofstream &file, const uint8_t *data, uint32_t size,
                            bool &keyFrameWritten, std::mutex &mtx) {
    if(!file.is_open()) return;
    bool hasStartCode = (size >= 4 && data[0] == 0 && data[1] == 0 &&
                         ((data[2] == 0 && data[3] == 1) || data[2] == 1));
    std::lock_guard<std::mutex> lock(mtx);
    if(!keyFrameWritten) {
        if(isH264KeyFrame(data, size)) keyFrameWritten = true;
        else return;
    }
    if(hasStartCode) {
        file.write(reinterpret_cast<const char *>(data), size);
    } else {
        writeH264StartCode(file);
        file.write(reinterpret_cast<const char *>(data), size);
    }
}

class H264Encoder {
public:
    H264Encoder() : codecCtx_(nullptr), frame_(nullptr), pkt_(nullptr), swsCtx_(nullptr),
                    pts_(0), width_(0), height_(0), initialized_(false) {}

    ~H264Encoder() { close(); }

    bool init(int width, int height, int fps, OBFormat srcFormat) {
        width_ = width;
        height_ = height;
        srcFormat_ = srcFormat;

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

        AVPixelFormat dstFmt = AV_PIX_FMT_YUV420P;
        AVPixelFormat srcFmt = AV_PIX_FMT_NONE;
        switch(srcFormat) {
        case OB_FORMAT_YUYV:  srcFmt = AV_PIX_FMT_YUYV422; break;
        case OB_FORMAT_UYVY:  srcFmt = AV_PIX_FMT_UYVY422; break;
        case OB_FORMAT_RGB:   srcFmt = AV_PIX_FMT_RGB24;   break;
        case OB_FORMAT_BGR:   srcFmt = AV_PIX_FMT_BGR24;   break;
        case OB_FORMAT_RGBA:  srcFmt = AV_PIX_FMT_RGBA;    break;
        case OB_FORMAT_BGRA:  srcFmt = AV_PIX_FMT_BGRA;   break;
        case OB_FORMAT_NV12: srcFmt = AV_PIX_FMT_NV12; break;
        case OB_FORMAT_NV21: srcFmt = AV_PIX_FMT_NV21; break;
        case OB_FORMAT_Y16: srcFmt = AV_PIX_FMT_GRAY16LE; break;
        case OB_FORMAT_Y8: srcFmt = AV_PIX_FMT_GRAY8; break;
        case OB_FORMAT_I420: srcFmt = AV_PIX_FMT_YUV420P; break;
        case OB_FORMAT_MJPG: srcFmt = AV_PIX_FMT_YUV420P; break;
        default:
            std::cerr << "Unsupported format for H264 encoding: " << srcFormat << std::endl;
            close();
            return false;
        }

    if(srcFmt != dstFmt || srcFormat == OB_FORMAT_MJPG || srcFormat == OB_FORMAT_MJPEG) {
        AVPixelFormat swsSrcFmt = srcFmt;
        if(srcFormat == OB_FORMAT_MJPG || srcFormat == OB_FORMAT_MJPEG) {
            swsSrcFmt = AV_PIX_FMT_YUV420P;
        }
        swsCtx_ = sws_getContext(width, height, swsSrcFmt,
            width, height, dstFmt,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
        if(!swsCtx_) {
            std::cerr << "Failed to create sws context" << std::endl;
            close();
            return false;
        }
    }

        mjpgCodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
        if(mjpgCodec_) {
            mjpgCtx_ = avcodec_alloc_context3(mjpgCodec_);
            if(mjpgCtx_) {
                mjpgCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
                mjpgCtx_->width = width;
                mjpgCtx_->height = height;
                if(avcodec_open2(mjpgCtx_, mjpgCodec_, nullptr) < 0) {
                    avcodec_free_context(&mjpgCtx_);
                    mjpgCtx_ = nullptr;
                }
            }
        }
        mjpgPkt_ = av_packet_alloc();
        mjpgDecFrame_ = av_frame_alloc();

        initialized_ = true;
        return true;
    }

    void close() {
        if(swsCtx_) { sws_freeContext(swsCtx_); swsCtx_ = nullptr; }
        if(frame_) { av_frame_free(&frame_); }
        if(pkt_) { av_packet_free(&pkt_); }
        if(codecCtx_) { avcodec_free_context(&codecCtx_); }
        if(mjpgDecFrame_) { av_frame_free(&mjpgDecFrame_); }
        if(mjpgPkt_) { av_packet_free(&mjpgPkt_); }
        if(mjpgCtx_) { avcodec_free_context(&mjpgCtx_); }
        initialized_ = false;
    }

    bool encode(const uint8_t *data, uint32_t size, std::ofstream &outFile, std::mutex &mtx) {
        if(!initialized_ || !codecCtx_) return false;

        AVFrame *srcFrame = nullptr;

        if(srcFormat_ == OB_FORMAT_MJPG || srcFormat_ == OB_FORMAT_MJPEG) {
            srcFrame = decodeMjpg(data, size);
            if(!srcFrame) return false;
        } else {
            if(swsCtx_) {
                int srcStride[4] = {0, 0, 0, 0};
                switch(srcFormat_) {
                case OB_FORMAT_YUYV:
                case OB_FORMAT_UYVY: srcStride[0] = width_ * 2; break;
                case OB_FORMAT_RGB:
                case OB_FORMAT_BGR: srcStride[0] = width_ * 3; break;
                case OB_FORMAT_RGBA:
                case OB_FORMAT_BGRA: srcStride[0] = width_ * 4; break;
                case OB_FORMAT_Y16:  srcStride[0] = width_ * 2; break;
                case OB_FORMAT_Y8:   srcStride[0] = width_; break;
                case OB_FORMAT_I420: srcStride[0] = width_;
                                     srcStride[1] = width_ / 2;
                                     srcStride[2] = width_ / 2; break;
                case OB_FORMAT_NV12:
                case OB_FORMAT_NV21: srcStride[0] = width_;
                                     srcStride[1] = width_; break;
                default: break;
                }

                const uint8_t *srcSlice[4] = { data, nullptr, nullptr, nullptr };
                if(srcFormat_ == OB_FORMAT_I420) {
                    srcSlice[0] = data;
                    srcSlice[1] = data + width_ * height_;
                    srcSlice[2] = data + width_ * height_ * 5 / 4;
                } else if(srcFormat_ == OB_FORMAT_NV12 || srcFormat_ == OB_FORMAT_NV21) {
                    srcSlice[0] = data;
                    srcSlice[1] = data + width_ * height_;
                }

                if(av_frame_make_writable(frame_) < 0) return false;
                sws_scale(swsCtx_, srcSlice, srcStride, 0, height_,
                          frame_->data, frame_->linesize);
            } else {
                if(av_frame_make_writable(frame_) < 0) return false;
                for(int i = 0; i < height_; i++)
                    memcpy(frame_->data[0] + i * frame_->linesize[0],
                           data + i * width_, width_);
                for(int i = 0; i < height_ / 2; i++) {
                    memcpy(frame_->data[1] + i * frame_->linesize[1],
                           data + width_ * height_ + i * width_ / 2, width_ / 2);
                    memcpy(frame_->data[2] + i * frame_->linesize[2],
                           data + width_ * height_ * 5 / 4 + i * width_ / 2, width_ / 2);
                }
            }
            srcFrame = frame_;
        }

        srcFrame->pts = pts_++;

        int ret = avcodec_send_frame(codecCtx_, srcFrame);
        if(ret < 0) {
            if(srcFrame != frame_) av_frame_free(&srcFrame);
            return false;
        }

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

        if(srcFrame != frame_) av_frame_free(&srcFrame);
        return wrote;
    }

private:
AVFrame *decodeMjpg(const uint8_t *data, uint32_t size) {
    if(!mjpgCtx_) return nullptr;

    mjpgPkt_->data = const_cast<uint8_t *>(data);
    mjpgPkt_->size = size;

    int ret = avcodec_send_packet(mjpgCtx_, mjpgPkt_);
    if(ret < 0) return nullptr;

    ret = avcodec_receive_frame(mjpgCtx_, mjpgDecFrame_);
    if(ret < 0) return nullptr;

    if(!swsCtx_) return nullptr;

    if(av_frame_make_writable(frame_) < 0) return nullptr;
    sws_scale(swsCtx_, mjpgDecFrame_->data, mjpgDecFrame_->linesize, 0,
        mjpgDecFrame_->height, frame_->data, frame_->linesize);
    return frame_;
}

    AVCodecContext *codecCtx_;
    AVFrame *frame_;
    AVPacket *pkt_;
    SwsContext *swsCtx_;
    int64_t pts_;
    int width_, height_;
    OBFormat srcFormat_;
    bool initialized_;

    const AVCodec *mjpgCodec_ = nullptr;
    AVCodecContext *mjpgCtx_ = nullptr;
    AVPacket *mjpgPkt_ = nullptr;
    AVFrame *mjpgDecFrame_ = nullptr;
};

struct StreamEncoder {
    std::shared_ptr<H264Encoder> encoder;
    std::shared_ptr<std::ofstream> file;
    std::mutex mtx;
    bool h264KeyFrameWritten = false;
    bool isNativeH264 = false;
    OBFormat srcFormat = OB_FORMAT_UNKNOWN;
    int width = 0;
    int height = 0;
    int fps = 30;
    std::string sensorTag;
};

struct SensorFiles {
    std::shared_ptr<StreamEncoder> color;
    std::shared_ptr<StreamEncoder> depth;
    std::shared_ptr<StreamEncoder> ir;
    std::shared_ptr<StreamEncoder> irLeft;
    std::shared_ptr<StreamEncoder> irRight;
    std::shared_ptr<std::ofstream> depthRawFile;
    std::shared_ptr<std::ofstream> imuFile;
    std::mutex depthRawMtx;
    std::mutex imuMtx;

    std::map<OBFrameType, uint64_t> frameCounts;
    std::mutex countMtx;
};

struct DeviceCapture {
    std::shared_ptr<ob::Pipeline> videoPipeline;
    std::shared_ptr<ob::Pipeline> imuPipeline;
    std::string deviceName;
    std::shared_ptr<SensorFiles> sensorFiles;
    bool hasIMU = false;
    float depthScale = 0.001f;
};

static std::shared_ptr<StreamEncoder> createStreamEncoder(const std::string &filePath,
                                                           OBFormat format, int w, int h, int fps) {
    auto se = std::make_shared<StreamEncoder>();
    se->srcFormat = format;
    se->width = w;
    se->height = h;
    se->fps = fps;
    se->sensorTag = filePath;

    if(format == OB_FORMAT_H264 || format == OB_FORMAT_H265 || format == OB_FORMAT_HEVC) {
        se->isNativeH264 = true;
        se->file = std::make_shared<std::ofstream>(filePath, std::ios::binary);
        return se;
    }

    se->encoder = std::make_shared<H264Encoder>();
    if(!se->encoder->init(w, h, fps, format)) {
        std::cerr << "  Failed to init H264 encoder for format=" << format
                  << " " << w << "x" << h << std::endl;
        se->encoder.reset();
        se->file = std::make_shared<std::ofstream>(filePath, std::ios::binary);
        return se;
    }

    se->file = std::make_shared<std::ofstream>(filePath, std::ios::binary);
    return se;
}

static void writeStreamFrame(StreamEncoder *se, const uint8_t *data, uint32_t size) {
    if(!se || !se->file || !se->file->is_open()) return;

    if(se->isNativeH264) {
        writeH264Frame(*se->file, data, size, se->h264KeyFrameWritten, se->mtx);
    } else if(se->encoder) {
        se->encoder->encode(data, size, *se->file, se->mtx);
    } else {
        std::lock_guard<std::mutex> lock(se->mtx);
        se->file->write(reinterpret_cast<const char *>(data), size);
        se->file->flush();
    }
}

static std::vector<std::string> parseDeviceNames(int argc, char **argv) {
    std::vector<std::string> names;
    for(int i = 1; i < argc; i++) names.push_back(argv[i]);
    return names;
}

static bool deviceMatches(const std::string &deviceName, const std::vector<std::string> &filter) {
    if(filter.empty()) return true;
    for(const auto &f : filter) {
        if(deviceName.find(f) != std::string::npos) return true;
    }
    return false;
}

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

static void writeDepthRawWithHeader(std::ofstream &file, const uint8_t *data, uint32_t size,
                                     int width, int height, float scale,
                                     uint64_t frameIndex, std::mutex &mtx) {
    std::lock_guard<std::mutex> lock(mtx);
    if(!file.is_open()) return;

    if(frameIndex == 0) {
        const char magic[] = "ORBBEC_DEPTH_RAW";
        file.write(magic, 16);

        uint32_t w32 = static_cast<uint32_t>(width);
        uint32_t h32 = static_cast<uint32_t>(height);
        file.write(reinterpret_cast<const char *>(&w32), 4);
        file.write(reinterpret_cast<const char *>(&h32), 4);

        uint32_t bpp = 2;
        file.write(reinterpret_cast<const char *>(&bpp), 4);

        file.write(reinterpret_cast<const char *>(&scale), 4);

        uint32_t frameSize = width * height * 2;
        file.write(reinterpret_cast<const char *>(&frameSize), 4);

        uint64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        file.write(reinterpret_cast<const char *>(&ts), 8);
    }

    file.write(reinterpret_cast<const char *>(data), size);
    file.flush();
}

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    auto deviceFilter = parseDeviceNames(argc, argv);

    ob::Context context;

    auto deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1) {
        std::cerr << "No Orbbec device found!" << std::endl;
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputRootDir = "capture_output/" + sessionTimestamp;
    mkdirp(outputRootDir);

    {
        std::ifstream usbfsFile("/sys/module/usbcore/parameters/usbfs_memory_mb");
        if(usbfsFile.is_open()) {
            int usbfsMb = 0;
            usbfsFile >> usbfsMb;
            if(usbfsMb < 128 && deviceList->getCount() > 1) {
                std::cerr << "WARNING: usbfs_memory_mb=" << usbfsMb << "MB is too low for "
                    << deviceList->getCount() << " devices. Recommend >= 128MB." << std::endl;
                std::cerr << "Fix: echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb" << std::endl;
                std::cerr << "Or:  sudo modprobe usbcore usbfs_memory_mb=256" << std::endl;
            }
        }
    }

    std::vector<std::shared_ptr<DeviceCapture>> captures;

for(uint32_t i = 0; i < deviceList->getCount(); i++) {
        auto device = deviceList->getDevice(i);
        auto devInfo = device->getDeviceInfo();
        std::string name = devInfo->getName();

        if(!deviceMatches(name, deviceFilter)) {
            std::cout << "Skipping device: " << name << std::endl;
            continue;
        }

        std::cout << "Found device: " << name
                  << " (SN: " << devInfo->getSerialNumber()
                  << ", PID: 0x" << std::hex << std::setw(4) << std::setfill('0')
                  << devInfo->getPid() << std::dec
                  << ", " << devInfo->getConnectionType() << ")" << std::endl;

        auto safeName = name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);

    auto cap = std::make_shared<DeviceCapture>();
    cap->deviceName = safeName;
    cap->sensorFiles = std::make_shared<SensorFiles>();

        auto startTs = getTimestampMs();
        std::string baseName = deviceOutputDir + "/" + safeName;

        try { device->timerSyncWithHost(); }
        catch(ob::Error &e) {
            std::cerr << "Timer sync warning: " << e.what() << std::endl;
        }

        if(device->isGlobalTimestampSupported()) {
            try { device->enableGlobalTimestamp(true); } catch(...) {}
        }

        auto pid = devInfo->getPid();
        auto vid = devInfo->getVid();

        cap->videoPipeline = std::make_shared<ob::Pipeline>(device);
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        auto sensorList = device->getSensorList();
        bool hasColor = false, hasDepth = false, hasIR = false;
        bool hasIRLeft = false, hasIRRight = false;
        bool hasAccel = false, hasGyro = false;

        OBFormat colorFormat = OB_FORMAT_UNKNOWN;
        OBFormat depthFormat = OB_FORMAT_UNKNOWN;
        OBFormat irFormat = OB_FORMAT_UNKNOWN;
        OBFormat irLeftFormat = OB_FORMAT_UNKNOWN;
        OBFormat irRightFormat = OB_FORMAT_UNKNOWN;
        int colorW = 0, colorH = 0, colorFps = 30;
        int depthW = 0, depthH = 0, depthFps = 30;
        int irW = 0, irH = 0, irFps = 30;
        int irLW = 0, irLH = 0, irLFps = 30;
        int irRW = 0, irRH = 0, irRFps = 30;

        std::shared_ptr<ob::VideoStreamProfile> colorProfile, depthProfile, irProfile, irLeftProfile, irRightProfile;

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
                    std::cout << " Color: no usable format found, skipping" << std::endl;
                }
            } else {
                hasColor = false;
            }
            if(hasColor) {
                std::cout << " Color: " << colorW << "x" << colorH
                    << "@" << colorFps << " format=" << colorFormat << std::endl;
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
                    std::cout << " Depth: no usable format found, skipping" << std::endl;
                }
            } else {
                hasDepth = false;
            }
            if(hasDepth) {
                std::cout << " Depth: " << depthW << "x" << depthH
                    << "@" << depthFps << " format=" << depthFormat << std::endl;
            }
                try {
                    auto depthSensorInfo = sensorList->getSensor(s);
                    (void)depthSensorInfo;
                } catch(...) {}
                try {
                    int32_t precisionLevel = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch(precisionLevel) {
                    case 0: cap->depthScale = 0.001f; break;
                    case 1: cap->depthScale = 0.0005f; break;
                    case 2: cap->depthScale = 0.00025f; break;
                    case 3: cap->depthScale = 0.0001f; break;
                    default: cap->depthScale = 0.001f; break;
                    }
                    std::cout << "  Depth scale: " << cap->depthScale << " (precision level " << precisionLevel << ")" << std::endl;
                } catch(...) {
                    cap->depthScale = 0.001f;
                    std::cout << "  Depth scale: 0.001 (default)" << std::endl;
                }
                break;
        case OB_SENSOR_IR:
            hasIR = true;
            irProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
            if(irProfile) {
                irFormat = irProfile->getFormat();
                if(irFormat == OB_FORMAT_UNKNOWN) irFormat = OB_FORMAT_Y8;
                config->enableStream(irProfile);
                irW = irProfile->getWidth();
                irH = irProfile->getHeight();
                irFps = irProfile->getFps();
            } else {
                hasIR = false;
            }
            if(hasIR) {
                std::cout << " IR: " << irW << "x" << irH
                    << "@" << irFps << " format=" << irFormat << std::endl;
            }
            break;
        case OB_SENSOR_IR_LEFT:
            hasIRLeft = true;
            irLeftProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
            if(irLeftProfile) {
                irLeftFormat = irLeftProfile->getFormat();
                if(irLeftFormat == OB_FORMAT_UNKNOWN) irLeftFormat = OB_FORMAT_Y8;
                config->enableStream(irLeftProfile);
                irLW = irLeftProfile->getWidth();
                irLH = irLeftProfile->getHeight();
                irLFps = irLeftProfile->getFps();
            } else {
                hasIRLeft = false;
            }
            if(hasIRLeft) {
                std::cout << " IR Left: " << irLW << "x" << irLH
                    << "@" << irLFps << " format=" << irLeftFormat << std::endl;
            }
            break;
        case OB_SENSOR_IR_RIGHT:
            hasIRRight = true;
            irRightProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
            if(irRightProfile) {
                irRightFormat = irRightProfile->getFormat();
                if(irRightFormat == OB_FORMAT_UNKNOWN) irRightFormat = OB_FORMAT_Y8;
                config->enableStream(irRightProfile);
                irRW = irRightProfile->getWidth();
                irRH = irRightProfile->getHeight();
                irRFps = irRightProfile->getFps();
            } else {
                hasIRRight = false;
            }
            if(hasIRRight) {
                std::cout << " IR Right: " << irRW << "x" << irRH
                    << "@" << irRFps << " format=" << irRightFormat << std::endl;
            }
            break;
        case OB_SENSOR_ACCEL: hasAccel = true; break;
            case OB_SENSOR_GYRO:  hasGyro = true; break;
            default: break;
            }
        }

        if(ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
            hasIRLeft = false;
            std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
        }

        auto sf = cap->sensorFiles;

        if(hasColor && colorFormat != OB_FORMAT_UNKNOWN) {
            sf->color = createStreamEncoder(baseName + "_color_" + startTs + ".h264",
                                            colorFormat, colorW, colorH, colorFps);
        }
        if(hasDepth && depthFormat != OB_FORMAT_UNKNOWN) {
            sf->depth = createStreamEncoder(baseName + "_depth_" + startTs + ".h264",
                                            depthFormat, depthW, depthH, depthFps);
            sf->depthRawFile = std::make_shared<std::ofstream>(
                baseName + "_depth_raw_" + startTs + ".raw", std::ios::binary);
        }
        if(hasIR && irFormat != OB_FORMAT_UNKNOWN) {
            sf->ir = createStreamEncoder(baseName + "_ir_" + startTs + ".h264",
                                         irFormat, irW, irH, irFps);
        }
        if(hasIRLeft && irLeftFormat != OB_FORMAT_UNKNOWN) {
            sf->irLeft = createStreamEncoder(baseName + "_ir_left_" + startTs + ".h264",
                                             irLeftFormat, irLW, irLH, irLFps);
        }
        if(hasIRRight && irRightFormat != OB_FORMAT_UNKNOWN) {
            sf->irRight = createStreamEncoder(baseName + "_ir_right_" + startTs + ".h264",
                                              irRightFormat, irRW, irRH, irRFps);
        }
        if(hasAccel || hasGyro) {
            sf->imuFile = std::make_shared<std::ofstream>(
                baseName + "_imu_" + startTs + ".txt");
            *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
            sf->imuFile->flush();
        }

auto depthFrameIdx = std::make_shared<std::atomic<uint64_t>>(0);

try {
    cap->videoPipeline->start(config,
        [sf, hasColor, hasDepth, hasIR, hasIRLeft, hasIRRight, cap, depthFrameIdx]
        (std::shared_ptr<ob::FrameSet> frameSet) {
                if(!frameSet) return;

                if(hasColor) {
                    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                    if(colorFrame) {
                        writeStreamFrame(sf->color.get(), colorFrame->getData(),
                                         colorFrame->getDataSize());
                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_COLOR]++;
                    }
                }

                if(hasDepth) {
                    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                    if(depthFrame) {
                        auto format = depthFrame->getFormat();
                        auto data = depthFrame->getData();
                        auto size = depthFrame->getDataSize();

                        if(format != OB_FORMAT_H264 && format != OB_FORMAT_H265 && format != OB_FORMAT_HEVC) {
                            if(sf->depthRawFile && sf->depthRawFile->is_open()) {
                                uint64_t idx = depthFrameIdx->fetch_add(1);
                                writeDepthRawWithHeader(*sf->depthRawFile, data, size,
                                                        cap->sensorFiles->depth ? cap->sensorFiles->depth->width : 0,
                                                        cap->sensorFiles->depth ? cap->sensorFiles->depth->height : 0,
                                                        cap->depthScale, idx, sf->depthRawMtx);
                            }
                        }

                        writeStreamFrame(sf->depth.get(), data, size);
                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_DEPTH]++;
                    }
                }

                if(hasIR) {
                    auto irFrame = frameSet->getFrame(OB_FRAME_IR);
                    if(irFrame) {
                        writeStreamFrame(sf->ir.get(), irFrame->getData(), irFrame->getDataSize());
                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_IR]++;
                    }
                }

                if(hasIRLeft) {
                    auto irLeftFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
                    if(irLeftFrame) {
                        writeStreamFrame(sf->irLeft.get(), irLeftFrame->getData(), irLeftFrame->getDataSize());
                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_IR_LEFT]++;
                    }
                }

                if(hasIRRight) {
                    auto irRightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
                    if(irRightFrame) {
                        writeStreamFrame(sf->irRight.get(), irRightFrame->getData(), irRightFrame->getDataSize());
                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_IR_RIGHT]++;
            }
        }
        });
    } catch(ob::Error &e) {
        std::cerr << " Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
        cap->videoPipeline.reset();
        continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    cap->hasIMU = (hasAccel && hasGyro);
        if(cap->hasIMU) {
            auto imuDev = cap->videoPipeline->getDevice();
            cap->imuPipeline = std::make_shared<ob::Pipeline>(imuDev);
            std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
            imuConfig->enableAccelStream();
            imuConfig->enableGyroStream();
            imuConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

            cap->imuPipeline->start(imuConfig, [sf](std::shared_ptr<ob::FrameSet> frameSet) {
                if(!frameSet) return;

                auto accelFrameRaw = frameSet->getFrame(OB_FRAME_ACCEL);
                auto gyroFrameRaw = frameSet->getFrame(OB_FRAME_GYRO);

                std::lock_guard<std::mutex> lock(sf->imuMtx);
                if(sf->imuFile && sf->imuFile->is_open()) {
                    auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();

                    if(accelFrameRaw) {
                        try {
                            auto accelFrame = accelFrameRaw->as<ob::AccelFrame>();
                            auto val = accelFrame->getValue();
                            auto ts = accelFrame->getTimeStampUs();
                            auto temp = accelFrame->getTemperature();
                            *sf->imuFile << nowMs << ",ACCEL,"
                                         << ts << ","
                                         << val.x << "," << val.y << "," << val.z << ","
                                         << temp << "\n";
                        } catch(...) {}
                    }

                    if(gyroFrameRaw) {
                        try {
                            auto gyroFrame = gyroFrameRaw->as<ob::GyroFrame>();
                            auto val = gyroFrame->getValue();
                            auto ts = gyroFrame->getTimeStampUs();
                            auto temp = gyroFrame->getTemperature();
                            *sf->imuFile << nowMs << ",GYRO,"
                                         << ts << ","
                                         << val.x << "," << val.y << "," << val.z << ","
                                         << temp << "\n";
                        } catch(...) {}
                    }
                    sf->imuFile->flush();
                }

                {
                    std::lock_guard<std::mutex> cLock(sf->countMtx);
                    if(accelFrameRaw) sf->frameCounts[OB_FRAME_ACCEL]++;
                    if(gyroFrameRaw) sf->frameCounts[OB_FRAME_GYRO]++;
                }
            });
        }

        captures.push_back(std::move(cap));
    }

    if(captures.empty()) {
        std::cerr << "No matching devices found!" << std::endl;
        if(!deviceFilter.empty()) {
            std::cerr << "Available devices:" << std::endl;
            for(uint32_t i = 0; i < deviceList->getCount(); i++) {
                auto dev = deviceList->getDevice(i);
                std::cerr << "  - " << dev->getDeviceInfo()->getName() << std::endl;
            }
        }
        return -1;
    }

    std::cout << "\n=== Recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << captures.size() << " device(s)" << std::endl;
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;

    auto startTime = ob_smpl::getNowTimesMs();
    uint32_t waitTime = 1000;

    while(g_running) {
        auto key = ob_smpl::waitForKeyPressed(waitTime);
        if(key == ESC_KEY || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        auto currentTime = ob_smpl::getNowTimesMs();
        if(currentTime >= startTime + waitTime) {
            for(auto &cap : captures) {
                std::map<OBFrameType, uint64_t> tempCounts;
                uint64_t duration;
                {
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    currentTime = ob_smpl::getNowTimesMs();
                    duration = currentTime - startTime;
                    if(!cap->sensorFiles->frameCounts.empty()) {
                        startTime = currentTime;
                        tempCounts = cap->sensorFiles->frameCounts;
                        for(auto &item : cap->sensorFiles->frameCounts) {
                            item.second = 0;
                        }
                    }
                }

                std::cout << "[" << cap->deviceName << "] ";
                if(tempCounts.empty()) {
                    std::cout << "Recording... waiting for frames";
                } else {
                    std::cout << "Recording... FPS: ";
                    std::string sep;
                    for(const auto &item : tempCounts) {
                        auto name = ob::TypeHelper::convertOBFrameTypeToString(item.first);
                        float rate = item.second / (duration / 1000.0f);
                        std::cout << std::fixed << std::setprecision(1)
                                  << sep << name << "=" << rate;
                        sep = ", ";
                    }
                }
                std::cout << std::endl;
            }
            waitTime = 2000;
        }
    }

    std::cout << "\n=== Stopping recording ===" << std::endl;

for(auto &cap : captures) {
    if(cap->videoPipeline) cap->videoPipeline->stop();
        if(cap->hasIMU && cap->imuPipeline) cap->imuPipeline->stop();

        auto &sf = cap->sensorFiles;
        if(sf->color && sf->color->encoder) sf->color->encoder->close();
        if(sf->depth && sf->depth->encoder) sf->depth->encoder->close();
        if(sf->ir && sf->ir->encoder) sf->ir->encoder->close();
        if(sf->irLeft && sf->irLeft->encoder) sf->irLeft->encoder->close();
        if(sf->irRight && sf->irRight->encoder) sf->irRight->encoder->close();

        if(sf->color && sf->color->file) sf->color->file->close();
        if(sf->depth && sf->depth->file) sf->depth->file->close();
        if(sf->ir && sf->ir->file) sf->ir->file->close();
        if(sf->irLeft && sf->irLeft->file) sf->irLeft->file->close();
        if(sf->irRight && sf->irRight->file) sf->irRight->file->close();
        if(sf->depthRawFile) sf->depthRawFile->close();
        if(sf->imuFile) sf->imuFile->close();

        std::cout << "Stopped: " << cap->deviceName << std::endl;
    }

    std::cout << "All recordings saved to: " << outputRootDir << "/" << std::endl;
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "OB Error: " << e.getFunction() << "\n  " << e.what()
              << "\n  status: " << e.getStatus() << std::endl;
    return -1;
}
catch(std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
}
