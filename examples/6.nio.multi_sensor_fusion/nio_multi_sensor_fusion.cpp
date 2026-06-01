// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_multi_sensor_fusion.cpp — Multi-sensor fusion with IMU integration.
// Renders a 2×2 composite of all device streams, encoded to H.264.
// Uses shared nio:: utilities from examples/utils/.

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

using namespace nio;

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
            std::cerr << "  Timer sync warning: " << e.what() << std::endl;
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
                if(hasColor) std::cout << "  Color: " << colorW << "x" << colorH << "@" << colorFps << " fmt=" << colorFormat << std::endl;
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
                if(hasDepth) std::cout << "  Depth: " << depthW << "x" << depthH << "@" << depthFps << " fmt=" << depthFormat << std::endl;
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
                    std::cout << "  Depth scale: " << df->depthScale << std::endl;
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
                if(hasIRLeft) std::cout << "  IR Left: " << irLW << "x" << irLH << "@" << irLFps << " fmt=" << irLeftFormat << std::endl;
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
                if(hasIRRight) std::cout << "  IR Right: " << irRW << "x" << irRH << "@" << irRFps << " fmt=" << irRightFormat << std::endl;
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
            std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
            NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
        }

        if(!hasDepth) {
            std::cerr << "  " << safeName << " has no depth sensor, skipping" << std::endl;
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
        if(!fusedEncoder->initRGB(outW, outH, outFps, 6000000)) {
            std::cerr << "  Failed to init H264 encoder for " << safeName << std::endl;
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
            std::cerr << "  Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
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
                std::cerr << "  IMU pipeline start failed: " << e.what() << std::endl;
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
    std::cerr << "OB Error: " << e.getFunction() << "\n  " << e.what()
              << "\n  status: " << e.getStatus() << std::endl;
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
