// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_multi_capture.cpp — Multi-device capture example. Records color,
// depth, and IR streams to H.264 / raw files with IMU CSV logging.
// Additionally performs D2C (depth-to-color) alignment and alpha-blends
// depth (jet colormap) with color, saving the fused result as H.264.
// Uses shared nio:: utilities from examples/utils/.
//
// Usage:
//   ./nio_multi_capture                                          # all devices
//   ./nio_multi_capture -c "305" "336L"                          # filter by camera type
//   ./nio_multi_capture -s /HDD/nio_capture                      # custom save directory
//   ./nio_multi_capture -c "305" -s /HDD/nio_capture --alpha 0.6 # combined

#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"
#include "nio_log.hpp"
#include "nio_common.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_stream_io.hpp"
#include "nio_color_convert.hpp"

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
#include <chrono>
#include <cmath>
#include <getopt.h>

using namespace nio;

struct DeviceCapture {
    std::shared_ptr<ob::Pipeline> videoPipeline;
    std::shared_ptr<ob::Pipeline> imuPipeline;
    std::string deviceName;
    std::shared_ptr<SensorFiles> sensorFiles;
    bool hasIMU = false;
    float depthScale = 0.001f;

    std::shared_ptr<ob::Align> alignFilter;
    std::shared_ptr<H264Encoder> fusedEncoder;
    std::shared_ptr<std::ofstream> fusedFile;
    std::mutex fusedMtx;
    std::shared_ptr<MjpgDecoderRes> mjpgRes;
    std::shared_ptr<std::vector<uint8_t>> colorRGBBuf;
    std::shared_ptr<std::vector<uint8_t>> fusedRGBBuf;
    int colorW = 0;
    int colorH = 0;
    int fusedFps = 30;
    OBFormat colorFormat = OB_FORMAT_UNKNOWN;
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    bool enableFusion = true;
    std::shared_ptr<std::atomic<uint64_t>> fusedFrameCount;
};

struct CaptureConfig {
    std::vector<std::string> cameraFilter;
    std::string saveDir;
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    bool enableFusion = true;
    bool noFusion = false;
};

static void printUsage(const char *prog) {
    std::cout << "Usage: " << prog << " [options] [camera_name_filter...]\n"
              << "\nOptions:\n"
              << "  -c <name...>   Camera type filter (can specify multiple, e.g. -c \"305\" \"336L\")\n"
              << "  -s <dir>       Save directory (default: capture_output/)\n"
              << "  --alpha VAL    Depth overlay opacity 0.0-1.0 (default: 0.5)\n"
              << "  --depth-min M  Min depth in meters for colormap (default: 0.3)\n"
              << "  --depth-max M  Max depth in meters for colormap (default: 5.0)\n"
              << "  --no-fusion    Disable D2C fusion output (only save individual streams)\n"
              << "  --help         Show this help\n"
              << "\nExamples:\n"
              << "  " << prog << "                                           # all devices, default settings\n"
              << "  " << prog << " -c \"305\" \"336L\"                            # filter cameras by type\n"
              << "  " << prog << " -s /HDD/nio_capture                         # custom save directory\n"
              << "  " << prog << " -c \"305\" -s /HDD/nio_capture --alpha 0.6    # combined options\n"
              << std::endl;
}

static CaptureConfig parseArgs(int argc, char **argv) {
    CaptureConfig cfg;

    static struct option longOpts[] = {
        {"alpha",     required_argument, nullptr, 'a'},
        {"depth-min", required_argument, nullptr, 'm'},
        {"depth-max", required_argument, nullptr, 'x'},
        {"no-fusion", no_argument,       nullptr, 'n'},
        {"help",      no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    opterr = 0;
    int ch;
    int optIdx = 0;

    while((ch = getopt_long(argc, argv, "c:s:h", longOpts, &optIdx)) != -1) {
        switch(ch) {
        case 'c':
            cfg.cameraFilter.push_back(optarg);
            while(optind < argc && argv[optind][0] != '-') {
                cfg.cameraFilter.push_back(argv[optind++]);
            }
            break;
        case 's':
            cfg.saveDir = optarg;
            while(!cfg.saveDir.empty() && cfg.saveDir.back() == '/')
                cfg.saveDir.pop_back();
            break;
        case 'a':
            cfg.alpha = std::stof(optarg);
            cfg.alpha = std::max(0.0f, std::min(1.0f, cfg.alpha));
            break;
        case 'm':
            cfg.depthMinM = std::stof(optarg);
            break;
        case 'x':
            cfg.depthMaxM = std::stof(optarg);
            break;
        case 'n':
            cfg.noFusion = true;
            break;
        case 'h':
            printUsage(argv[0]);
            exit(0);
        default:
            break;
        }
    }

    for(int i = optind; i < argc; i++) {
        cfg.cameraFilter.push_back(argv[i]);
    }

    if(cfg.noFusion) {
        cfg.enableFusion = false;
    }

    return cfg;
}

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    CaptureConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_multi_capture", cfg.saveDir.empty() ? "capture_output" : cfg.saveDir);
    NIO_LOG_SET_LEVEL(nio::LogLevel::TRACE);
    NIO_LOG_INFO_S("Process started, camera_filter_count=" << cfg.cameraFilter.size()
        << " saveDir=" << (cfg.saveDir.empty() ? "capture_output" : cfg.saveDir)
        << " alpha=" << cfg.alpha << " depthMin=" << cfg.depthMinM
        << " depthMax=" << cfg.depthMaxM << " fusion=" << (cfg.enableFusion ? "on" : "off"));
    for(size_t i = 0; i < cfg.cameraFilter.size(); i++) {
        NIO_LOG_DEBUG_S("Camera filter[" << i << "]=" << cfg.cameraFilter[i]);
    }

    ob::Context context;

    auto deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1) {
        std::cerr << "No Orbbec device found!" << std::endl;
        NIO_LOG_FATAL("No Orbbec device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputBaseDir = cfg.saveDir.empty() ? "capture_output" : cfg.saveDir;
    std::string outputRootDir = outputBaseDir + "/" + sessionTimestamp;
    mkdirp(outputRootDir);
    NIO_LOG_INFO_S("Session timestamp=" << sessionTimestamp << " outputDir=" << outputRootDir);

    {
        std::ifstream usbfsFile("/sys/module/usbcore/parameters/usbfs_memory_mb");
        if(usbfsFile.is_open()) {
            int usbfsMb = 0;
            usbfsFile >> usbfsMb;
            if(usbfsMb < 128 && deviceList->getCount() > 1) {
                std::cerr << "WARNING: usbfs_memory_mb=" << usbfsMb << "MB is too low for "
                          << deviceList->getCount() << " devices. Recommend >= 128MB." << std::endl;
                std::cerr << "Fix: echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb" << std::endl;
                std::cerr << "Or: sudo modprobe usbcore usbfs_memory_mb=256" << std::endl;
                NIO_LOG_WARN_S("usbfs_memory_mb=" << usbfsMb << "MB is too low for " << deviceList->getCount() << " devices, recommend >= 128MB");
            }
        }
    }

    std::vector<std::shared_ptr<DeviceCapture>> captures;

    for(uint32_t i = 0; i < deviceList->getCount(); i++) {
        auto device = deviceList->getDevice(i);
        auto devInfo = device->getDeviceInfo();
        std::string name = devInfo->getName();

        if(!deviceMatches(name, cfg.cameraFilter)) {
            std::cout << "Skipping device: " << name << std::endl;
            NIO_LOG_DEBUG_S("Skipping device: " << name << " (does not match filter)");
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
        NIO_LOG_DEBUG_S("Created output dir: " << deviceOutputDir);

        auto cap = std::make_shared<DeviceCapture>();
        cap->deviceName = safeName;
        cap->sensorFiles = std::make_shared<SensorFiles>();
        cap->alpha = cfg.alpha;
        cap->depthMinM = cfg.depthMinM;
        cap->depthMaxM = cfg.depthMaxM;
        cap->enableFusion = cfg.enableFusion;
        cap->fusedFrameCount = std::make_shared<std::atomic<uint64_t>>(0);

        auto startTs = getTimestampMs();
        std::string baseName = deviceOutputDir + "/" + safeName;

        try { device->timerSyncWithHost(); }
        catch(ob::Error &e) {
            std::cerr << "Timer sync warning: " << e.what() << std::endl;
            NIO_LOG_WARN_S("Timer sync failed for " << safeName << ": " << e.what());
        }

        if(device->isGlobalTimestampSupported()) {
            try { device->enableGlobalTimestamp(true); } catch(...) {}
        }

        auto pid = devInfo->getPid();
        auto vid = devInfo->getVid();

        cap->videoPipeline = std::make_shared<ob::Pipeline>(device);
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

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
                        std::cout << " Depth: no usable format found, skipping" << std::endl;
                    }
                } else {
                    hasDepth = false;
                }
                if(hasDepth) {
                    std::cout << " Depth: " << depthW << "x" << depthH
                              << "@" << depthFps << " format=" << depthFormat << std::endl;
                    NIO_LOG_INFO_S("Depth stream: " << depthW << "x" << depthH << "@" << depthFps << " format=" << depthFormat);
                }
                try {
                    int32_t precisionLevel = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch(precisionLevel) {
                    case 0: cap->depthScale = 0.001f; break;
                    case 1: cap->depthScale = 0.0005f; break;
                    case 2: cap->depthScale = 0.00025f; break;
                    case 3: cap->depthScale = 0.0001f; break;
                    default: cap->depthScale = 0.001f; break;
                    }
                    std::cout << " Depth scale: " << cap->depthScale << " (precision level " << precisionLevel << ")" << std::endl;
                    NIO_LOG_INFO_S("Depth scale: " << cap->depthScale << " precision_level=" << precisionLevel);
                } catch(...) {
                    cap->depthScale = 0.001f;
                    std::cout << " Depth scale: 0.001 (default)" << std::endl;
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
                    NIO_LOG_INFO_S("IR stream: " << irW << "x" << irH << "@" << irFps << " format=" << irFormat);
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
                    NIO_LOG_INFO_S("IR Left stream: " << irLW << "x" << irLH << "@" << irLFps << " format=" << irLeftFormat);
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
                    NIO_LOG_INFO_S("IR Right stream: " << irRW << "x" << irRH << "@" << irRFps << " format=" << irRightFormat);
                }
                break;
            case OB_SENSOR_ACCEL: hasAccel = true; break;
            case OB_SENSOR_GYRO: hasGyro = true; break;
            default: break;
            }
        }

        if(ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
            hasIRLeft = false;
            std::cout << " Gemini 305g: disabled IR_LEFT" << std::endl;
            NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
        }

        auto sf = cap->sensorFiles;

        if(hasColor && colorFormat != OB_FORMAT_UNKNOWN) {
            sf->color = createStreamEncoder(baseName + "_color_" + startTs + ".h264",
                colorFormat, colorW, colorH, colorFps, nullptr, false);
            NIO_LOG_INFO_S("Color output: " << baseName + "_color_" + startTs + ".h264" << " fmt=" << colorFormat);
        }
        if(hasDepth && depthFormat != OB_FORMAT_UNKNOWN) {
            sf->depth = createStreamEncoder(baseName + "_depth_" + startTs + ".h264",
                depthFormat, depthW, depthH, depthFps, nullptr, false);
            sf->depthRawFile = std::make_shared<std::ofstream>(
                baseName + "_depth_raw_" + startTs + ".raw", std::ios::binary);
            NIO_LOG_INFO_S("Depth output: " << baseName + "_depth_" + startTs + ".h264" << " + raw");
        }
        if(hasIR && irFormat != OB_FORMAT_UNKNOWN) {
            sf->ir = createStreamEncoder(baseName + "_ir_" + startTs + ".h264",
                irFormat, irW, irH, irFps, nullptr, false);
            NIO_LOG_INFO_S("IR output: " << baseName + "_ir_" + startTs + ".h264");
        }
        if(hasIRLeft && irLeftFormat != OB_FORMAT_UNKNOWN) {
            sf->irLeft = createStreamEncoder(baseName + "_ir_left_" + startTs + ".h264",
                irLeftFormat, irLW, irLH, irLFps, nullptr, false);
            NIO_LOG_INFO_S("IR Left output: " << baseName + "_ir_left_" + startTs + ".h264");
        }
        if(hasIRRight && irRightFormat != OB_FORMAT_UNKNOWN) {
            sf->irRight = createStreamEncoder(baseName + "_ir_right_" + startTs + ".h264",
                irRightFormat, irRW, irRH, irRFps, nullptr, false);
            NIO_LOG_INFO_S("IR Right output: " << baseName + "_ir_right_" + startTs + ".h264");
        }
        if(hasAccel || hasGyro) {
            sf->imuFile = std::make_shared<std::ofstream>(
                baseName + "_imu_" + startTs + ".txt");
            *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
            sf->imuFile->flush();
            NIO_LOG_INFO_S("IMU output: " << baseName + "_imu_" + startTs + ".txt");
        }

        bool canFuse = cfg.enableFusion && hasColor && hasDepth;
        if(canFuse) {
            cap->alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            cap->colorW = colorW;
            cap->colorH = colorH;
            cap->colorFormat = colorFormat;
            cap->fusedFps = std::min(colorFps, depthFps);

            std::string fusedPath = baseName + "_d2c_fused_" + startTs + ".h264";
            cap->fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

            cap->fusedEncoder = std::make_shared<H264Encoder>();
            if(!cap->fusedEncoder->initRGB(colorW, colorH, cap->fusedFps)) {
                std::cerr << " Failed to init fused H264 encoder for " << safeName << std::endl;
                NIO_LOG_ERROR_S("Failed to init fused H264 encoder for " << safeName
                                << " " << colorW << "x" << colorH << "@" << cap->fusedFps);
                canFuse = false;
            } else {
                cap->mjpgRes = std::make_shared<MjpgDecoderRes>();
                cap->mjpgRes->init(colorW, colorH, colorFormat);
                int rgbBufSize = colorW * colorH * 3;
                cap->colorRGBBuf = std::make_shared<std::vector<uint8_t>>(rgbBufSize, 0);
                cap->fusedRGBBuf = std::make_shared<std::vector<uint8_t>>(rgbBufSize, 0);
                std::cout << " D2C Fusion: " << colorW << "x" << colorH
                          << "@" << cap->fusedFps << " alpha=" << cfg.alpha
                          << " depth=[" << cfg.depthMinM << "m, " << cfg.depthMaxM << "m]" << std::endl;
                NIO_LOG_INFO_S("D2C Fusion enabled: " << colorW << "x" << colorH << "@" << cap->fusedFps
                               << " alpha=" << cfg.alpha << " depthRange=" << cfg.depthMinM << "m-" << cfg.depthMaxM << "m"
                               << " output=" << fusedPath);
            }
        } else if(cfg.enableFusion && !hasColor) {
            std::cout << " D2C Fusion: skipped (no color sensor)" << std::endl;
            NIO_LOG_DEBUG_S("D2C Fusion skipped for " << safeName << ": no color sensor");
        } else if(cfg.enableFusion && !hasDepth) {
            std::cout << " D2C Fusion: skipped (no depth sensor)" << std::endl;
            NIO_LOG_DEBUG_S("D2C Fusion skipped for " << safeName << ": no depth sensor");
        }

        auto depthFrameIdx = std::make_shared<std::atomic<uint64_t>>(0);

        try {
            cap->videoPipeline->enableFrameSync();
        } catch(...) {}

        try {
            cap->videoPipeline->start(config,
                [sf, hasColor, hasDepth, hasIR, hasIRLeft, hasIRRight, cap, depthFrameIdx, canFuse]
                (std::shared_ptr<ob::FrameSet> frameSet) {
                    if(!frameSet) return;

                    if(canFuse && cap->alignFilter) {
                        auto alignedFrame = cap->alignFilter->process(frameSet);
                        auto alignedFS = alignedFrame
                            ? std::dynamic_pointer_cast<ob::FrameSet>(alignedFrame)
                            : nullptr;
                        if(!alignedFS) alignedFS = frameSet;

                        auto colorFrame = alignedFS->getFrame(OB_FRAME_COLOR);
                        auto depthFrame = alignedFS->getFrame(OB_FRAME_DEPTH);

        if(colorFrame && depthFrame) {
            uint64_t colorTsUs = colorFrame->getTimeStampUs();
            (void)colorTsUs;

            int w = cap->colorW;
                            int h = cap->colorH;

                            bool colorOk = decodeColorToRGB(
                                colorFrame->getData(), colorFrame->getDataSize(),
                                cap->colorFormat, w, h,
                                cap->colorRGBBuf->data(), cap->mjpgRes);

                            if(!colorOk) {
                                memset(cap->colorRGBBuf->data(), 128, w * h * 3);
                            }

                            auto depthData = depthFrame->getData();
                            auto depthSize = depthFrame->getDataSize();
                            auto depthFmt = depthFrame->getFormat();

                            float scale = cap->depthScale;
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

                            float minDist = cap->depthMinM;
                            float maxDist = cap->depthMaxM;
                            float alpha = cap->alpha;

                            if(depthFmt == OB_FORMAT_Y16 && depthData &&
                               depthSize >= (uint32_t)(depthW * depthH * 2)) {
                                const uint16_t *depthPtr = reinterpret_cast<const uint16_t *>(depthData);
                                int blendW = std::min(w, depthW);
                                int blendH = std::min(h, depthH);

                                for(int y = 0; y < blendH; y++) {
                                    for(int x = 0; x < blendW; x++) {
                                        uint16_t rawVal = depthPtr[y * depthW + x];
                                        float distM = rawVal * scale / 1000.0f;

                                        if(rawVal == 0 || distM < minDist || distM > maxDist) {
                                            int idx = (y * w + x) * 3;
                                            (*cap->fusedRGBBuf)[idx + 0] = (*cap->colorRGBBuf)[idx + 0];
                                            (*cap->fusedRGBBuf)[idx + 1] = (*cap->colorRGBBuf)[idx + 1];
                                            (*cap->fusedRGBBuf)[idx + 2] = (*cap->colorRGBBuf)[idx + 2];
                                        } else {
                                            float norm = (distM - minDist) / (maxDist - minDist);
                                            norm = std::max(0.0f, std::min(1.0f, norm));
                                            uint8_t v = static_cast<uint8_t>(norm * 255.0f);
                                            uint8_t cr, cg, cb;
                                            jetColormap(v, cr, cg, cb);

                                            int idx = (y * w + x) * 3;
                                            float inv = 1.0f - alpha;
                                            (*cap->fusedRGBBuf)[idx + 0] = static_cast<uint8_t>(
                                                inv * (*cap->colorRGBBuf)[idx + 0] + alpha * cr + 0.5f);
                                            (*cap->fusedRGBBuf)[idx + 1] = static_cast<uint8_t>(
                                                inv * (*cap->colorRGBBuf)[idx + 1] + alpha * cg + 0.5f);
                                            (*cap->fusedRGBBuf)[idx + 2] = static_cast<uint8_t>(
                                                inv * (*cap->colorRGBBuf)[idx + 2] + alpha * cb + 0.5f);
                                        }
                                    }
                                }
                                for(int y = blendH; y < h; y++) {
                                    memcpy(cap->fusedRGBBuf->data() + y * w * 3,
                                           cap->colorRGBBuf->data() + y * w * 3, w * 3);
                                }
                            } else {
                                memcpy(cap->fusedRGBBuf->data(), cap->colorRGBBuf->data(), w * h * 3);
                            }

            cap->fusedEncoder->encodeRGB(
                cap->fusedRGBBuf->data(), *cap->fusedFile, cap->fusedMtx, depthFrame->getTimeStampUs());
                            (*cap->fusedFrameCount)++;
                        }
                    }

        if(hasColor) {
            auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
            if(colorFrame) {
                writeStreamFrame(sf->color.get(), colorFrame->getData(),
                    colorFrame->getDataSize(), colorFrame->getTimeStampUs());
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
                            cap->depthScale, idx, sf->depthRawMtx,
                            depthFrame->getTimeStampUs());
                    }
                }

                writeStreamFrame(sf->depth.get(), data, size, depthFrame->getTimeStampUs());
                std::lock_guard<std::mutex> lock(sf->countMtx);
                sf->frameCounts[OB_FRAME_DEPTH]++;
            }
        }

        if(hasIR) {
            auto irFrame = frameSet->getFrame(OB_FRAME_IR);
            if(irFrame) {
                writeStreamFrame(sf->ir.get(), irFrame->getData(), irFrame->getDataSize(),
                    irFrame->getTimeStampUs());
                std::lock_guard<std::mutex> lock(sf->countMtx);
                sf->frameCounts[OB_FRAME_IR]++;
            }
        }

        if(hasIRLeft) {
            auto irLeftFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
            if(irLeftFrame) {
                writeStreamFrame(sf->irLeft.get(), irLeftFrame->getData(), irLeftFrame->getDataSize(),
                    irLeftFrame->getTimeStampUs());
                std::lock_guard<std::mutex> lock(sf->countMtx);
                sf->frameCounts[OB_FRAME_IR_LEFT]++;
            }
        }

        if(hasIRRight) {
            auto irRightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
            if(irRightFrame) {
                writeStreamFrame(sf->irRight.get(), irRightFrame->getData(), irRightFrame->getDataSize(),
                    irRightFrame->getTimeStampUs());
                std::lock_guard<std::mutex> lock(sf->countMtx);
                sf->frameCounts[OB_FRAME_IR_RIGHT]++;
            }
        }
                });
        } catch(ob::Error &e) {
            std::cerr << " Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
            NIO_LOG_ERROR_S("Pipeline start failed for " << safeName << ": " << e.what());
            cap->videoPipeline.reset();
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        cap->hasIMU = (hasAccel && hasGyro);
        if(cap->hasIMU) {
            NIO_LOG_INFO_S("Starting IMU pipeline for " << safeName);
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
        NIO_LOG_FATAL("No matching devices found!");
        if(!cfg.cameraFilter.empty()) {
            std::cerr << "Available devices:" << std::endl;
            for(uint32_t i = 0; i < deviceList->getCount(); i++) {
                auto dev = deviceList->getDevice(i);
                std::cerr << " - " << dev->getDeviceInfo()->getName() << std::endl;
            }
        }
        return -1;
    }

    std::cout << "\n=== Recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << captures.size() << " device(s)" << std::endl;
    if(cfg.enableFusion) {
        std::cout << "D2C Fusion: alpha=" << cfg.alpha
                  << ", depth range: " << cfg.depthMinM << "m - " << cfg.depthMaxM << "m" << std::endl;
    } else {
        std::cout << "D2C Fusion: disabled" << std::endl;
    }
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;
    NIO_LOG_INFO_S("=== Recording started === devices=" << captures.size() << " outputDir=" << outputRootDir
                   << " fusion=" << (cfg.enableFusion ? "on" : "off"));
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    auto lastReportTime = ob_smpl::getNowTimesMs();
    uint32_t waitTime = 1000;

    while(g_running) {
        auto key = ob_smpl::waitForKeyPressed(waitTime);
        if(key == ESC_KEY || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        auto currentTime = ob_smpl::getNowTimesMs();
        if(currentTime >= lastReportTime + waitTime) {
            uint64_t reportDuration = currentTime - lastReportTime;
            lastReportTime = currentTime;

            for(auto &cap : captures) {
                std::map<OBFrameType, uint64_t> tempCounts;
                {
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    if(!cap->sensorFiles->frameCounts.empty()) {
                        tempCounts = cap->sensorFiles->frameCounts;
                        for(auto &item : cap->sensorFiles->frameCounts) {
                            item.second = 0;
                        }
                    }
                }

                std::cout << "[" << cap->deviceName << "] ";
                if(tempCounts.empty() && !cap->fusedEncoder) {
                    std::cout << "Recording... waiting for frames";
                } else {
                    std::cout << "Recording... FPS: ";
                    std::string sep;
                    for(const auto &item : tempCounts) {
                        auto name = ob::TypeHelper::convertOBFrameTypeToString(item.first);
                        float rate = (reportDuration > 0) ? (item.second / (reportDuration / 1000.0f)) : 0.0f;
                        std::cout << std::fixed << std::setprecision(1)
                                  << sep << name << "=" << rate;
                        sep = ", ";
                        NIO_LOG_TRACE_S("[" << cap->deviceName << "] " << name << "=" << std::fixed << std::setprecision(1) << rate);
                    }
                    if(cap->fusedEncoder) {
                        uint64_t fusedCount = cap->fusedFrameCount->exchange(0);
                        float fusedRate = (reportDuration > 0) ? (fusedCount / (reportDuration / 1000.0f)) : 0.0f;
                        std::cout << sep << "fused=" << std::fixed << std::setprecision(1) << fusedRate;
                        NIO_LOG_TRACE_S("[" << cap->deviceName << "] fused=" << std::fixed << std::setprecision(1) << fusedRate);
                    }
                }
                std::cout << std::endl;
            }
            waitTime = 2000;
        }
    }

    std::cout << "\n=== Stopping recording ===" << std::endl;
    NIO_LOG_INFO("=== Stopping recording ===");

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

        if(cap->fusedEncoder) cap->fusedEncoder->close();
        if(cap->fusedFile) cap->fusedFile->close();
        cap->mjpgRes.reset();

        std::cout << "Stopped: " << cap->deviceName << std::endl;
        NIO_LOG_INFO_S("Stopped device: " << cap->deviceName);
    }

    std::cout << "All recordings saved to: " << outputRootDir << "/" << std::endl;
    NIO_LOG_INFO_S("All recordings saved to: " << outputRootDir << "/");
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
