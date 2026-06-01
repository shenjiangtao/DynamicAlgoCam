// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_multi_capture_cv.cpp — Multi-device capture with OpenCV display.
// Records all sensor streams to H.264 / raw files and shows a live
// cv::imshow tile view. Uses shared nio:: utilities from examples/utils/.

#include <libobsensor/ObSensor.hpp>
#include "utils.hpp"
#include "nio_log.hpp"
#include "nio_common.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_stream_io.hpp"
#include "nio_color_convert_cv.hpp"
#include <opencv2/opencv.hpp>

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

using namespace nio;

struct DeviceCapture {
    std::shared_ptr<ob::Pipeline> videoPipeline;
    std::shared_ptr<ob::Pipeline> imuPipeline;
    std::string deviceName;
    std::shared_ptr<SensorFiles> sensorFiles;
    bool hasIMU = false;
    float depthScale = 0.001f;

    cv::Mat colorBGR;
    cv::Mat depthColorized;
    cv::Mat irBGR;
    cv::Mat irLeftBGR;
    cv::Mat irRightBGR;
    cv::Mat displayTile;
    std::mutex displayMtx;

    bool hasColor = false;
    bool hasDepth = false;
    bool hasIR = false;
    bool hasIRLeft = false;
    bool hasIRRight = false;

    int colorW = 0, colorH = 0;
    int depthW = 0, depthH = 0;
    int irW = 0, irH = 0;
    int irLW = 0, irLH = 0;
    int irRW = 0, irRH = 0;

    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
};

struct CaptureConfig {
    std::vector<std::string> deviceFilter;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
};

static void printUsage() {
    std::cout << "Usage: nio_multi_capture_cv [device_name_filter...] [options]\n"
              << "Options:\n"
              << "  --depth-min M   Min depth in meters for colormap (default: 0.3)\n"
              << "  --depth-max M   Max depth in meters for colormap (default: 5.0)\n"
              << "  --help          Show this help\n"
              << "\nExample:\n"
              << "  nio_multi_capture_cv                   # all devices\n"
              << "  nio_multi_capture_cv 336L --depth-max 3.0  # filter + custom range\n"
              << std::endl;
}

static CaptureConfig parseArgs(int argc, char **argv) {
    CaptureConfig cfg;
    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "--depth-min" && i + 1 < argc) {
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

static cv::Mat buildDeviceTile(DeviceCapture *cap) {
    std::lock_guard<std::mutex> lock(cap->displayMtx);

    std::vector<cv::Mat> row;
    if(cap->hasColor && !cap->colorBGR.empty()) {
        cv::Mat c = cap->colorBGR.clone();
        cv::putText(c, "Color", cv::Point(6, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        row.push_back(c);
    }
    if(cap->hasDepth && !cap->depthColorized.empty()) {
        cv::Mat d = cap->depthColorized.clone();
        cv::putText(d, "Depth", cv::Point(6, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        row.push_back(d);
    }
    if(cap->hasIR && !cap->irBGR.empty()) {
        cv::Mat ir = cap->irBGR.clone();
        cv::putText(ir, "IR", cv::Point(6, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        row.push_back(ir);
    }
    if(cap->hasIRLeft && !cap->irLeftBGR.empty()) {
        cv::Mat irl = cap->irLeftBGR.clone();
        cv::putText(irl, "IR-L", cv::Point(6, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        row.push_back(irl);
    }
    if(cap->hasIRRight && !cap->irRightBGR.empty()) {
        cv::Mat irr = cap->irRightBGR.clone();
        cv::putText(irr, "IR-R", cv::Point(6, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        row.push_back(irr);
    }

    if(row.empty()) return cv::Mat();

    int targetH = 240;
    for(auto &m : row) {
        if(!m.empty()) {
            double scale = static_cast<double>(targetH) / m.rows;
            cv::resize(m, m, cv::Size(), scale, scale);
        }
    }

    cv::Mat tile;
    cv::hconcat(row, tile);

    cv::putText(tile, cap->deviceName, cv::Point(6, tile.rows - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    return tile;
}

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    CaptureConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_multi_capture_cv", "capture_cv_output");
    NIO_LOG_SET_LEVEL(nio::LogLevel::TRACE);
    NIO_LOG_INFO_S("Process started, depthMin=" << cfg.depthMinM
                   << " depthMax=" << cfg.depthMaxM
                   << " device_filter_count=" << cfg.deviceFilter.size());
    for(size_t i = 0; i < cfg.deviceFilter.size(); i++) {
        NIO_LOG_DEBUG_S("Device filter[" << i << "]=" << cfg.deviceFilter[i]);
    }

    ob::Context context;

    auto deviceList = context.queryDeviceList();
    if(deviceList->getCount() < 1) {
        std::cerr << "No Orbbec device found!" << std::endl;
        NIO_LOG_FATAL("No Orbbec device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputRootDir = "capture_cv_output/" + sessionTimestamp;
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

        if(!deviceMatches(name, cfg.deviceFilter)) {
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
        cap->depthMinM = cfg.depthMinM;
        cap->depthMaxM = cfg.depthMaxM;

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

        auto sensorList = device->getSensorList();
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
                cap->hasColor = true;
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
                        cap->hasColor = false;
                        std::cout << "  Color: no usable format found, skipping" << std::endl;
                    }
                } else {
                    cap->hasColor = false;
                }
                if(cap->hasColor) {
                    cap->colorW = colorW;
                    cap->colorH = colorH;
                    std::cout << "  Color: " << colorW << "x" << colorH
                              << "@" << colorFps << " format=" << colorFormat << std::endl;
                    NIO_LOG_INFO_S("Color stream: " << colorW << "x" << colorH << "@" << colorFps << " format=" << colorFormat);
                }
                break;
            case OB_SENSOR_DEPTH:
                cap->hasDepth = true;
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
                        cap->hasDepth = false;
                        std::cout << "  Depth: no usable format found, skipping" << std::endl;
                    }
                } else {
                    cap->hasDepth = false;
                }
                if(cap->hasDepth) {
                    cap->depthW = depthW;
                    cap->depthH = depthH;
                    std::cout << "  Depth: " << depthW << "x" << depthH
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
                    std::cout << "  Depth scale: " << cap->depthScale << " (precision level " << precisionLevel << ")" << std::endl;
                    NIO_LOG_INFO_S("Depth scale: " << cap->depthScale << " precision_level=" << precisionLevel);
                } catch(...) {
                    cap->depthScale = 0.001f;
                    std::cout << "  Depth scale: 0.001 (default)" << std::endl;
                }
                break;
            case OB_SENSOR_IR:
                cap->hasIR = true;
                irProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irProfile) {
                    irFormat = irProfile->getFormat();
                    if(irFormat == OB_FORMAT_UNKNOWN) irFormat = OB_FORMAT_Y8;
                    config->enableStream(irProfile);
                    irW = irProfile->getWidth();
                    irH = irProfile->getHeight();
                    irFps = irProfile->getFps();
                } else {
                    cap->hasIR = false;
                }
                if(cap->hasIR) {
                    cap->irW = irW;
                    cap->irH = irH;
                    std::cout << "  IR: " << irW << "x" << irH
                              << "@" << irFps << " format=" << irFormat << std::endl;
                    NIO_LOG_INFO_S("IR stream: " << irW << "x" << irH << "@" << irFps << " format=" << irFormat);
                }
                break;
            case OB_SENSOR_IR_LEFT:
                cap->hasIRLeft = true;
                irLeftProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irLeftProfile) {
                    irLeftFormat = irLeftProfile->getFormat();
                    if(irLeftFormat == OB_FORMAT_UNKNOWN) irLeftFormat = OB_FORMAT_Y8;
                    config->enableStream(irLeftProfile);
                    irLW = irLeftProfile->getWidth();
                    irLH = irLeftProfile->getHeight();
                    irLFps = irLeftProfile->getFps();
                } else {
                    cap->hasIRLeft = false;
                }
                if(cap->hasIRLeft) {
                    cap->irLW = irLW;
                    cap->irLH = irLH;
                    std::cout << "  IR Left: " << irLW << "x" << irLH
                              << "@" << irLFps << " format=" << irLeftFormat << std::endl;
                    NIO_LOG_INFO_S("IR Left stream: " << irLW << "x" << irLH << "@" << irLFps << " format=" << irLeftFormat);
                }
                break;
            case OB_SENSOR_IR_RIGHT:
                cap->hasIRRight = true;
                irRightProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if(irRightProfile) {
                    irRightFormat = irRightProfile->getFormat();
                    if(irRightFormat == OB_FORMAT_UNKNOWN) irRightFormat = OB_FORMAT_Y8;
                    config->enableStream(irRightProfile);
                    irRW = irRightProfile->getWidth();
                    irRH = irRightProfile->getHeight();
                    irRFps = irRightProfile->getFps();
                } else {
                    cap->hasIRRight = false;
                }
                if(cap->hasIRRight) {
                    cap->irRW = irRW;
                    cap->irRH = irRH;
                    std::cout << "  IR Right: " << irRW << "x" << irRH
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
            cap->hasIRLeft = false;
            std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
            NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
        }

        auto sf = cap->sensorFiles;

        if(cap->hasColor && colorFormat != OB_FORMAT_UNKNOWN) {
            sf->color = createStreamEncoder(baseName + "_color_" + startTs + ".h264",
                                            colorFormat, colorW, colorH, colorFps, "nio@orbbec-captu");
            NIO_LOG_INFO_S("Color output: " << baseName + "_color_" + startTs + ".h264" << " fmt=" << colorFormat);
        }
        if(cap->hasDepth && depthFormat != OB_FORMAT_UNKNOWN) {
            sf->depth = createStreamEncoder(baseName + "_depth_" + startTs + ".h264",
                                            depthFormat, depthW, depthH, depthFps, "nio@orbbec-captu");
            sf->depthRawFile = std::make_shared<std::ofstream>(
                baseName + "_depth_raw_" + startTs + ".raw", std::ios::binary);
            NIO_LOG_INFO_S("Depth output: " << baseName + "_depth_" + startTs + ".h264" << " + raw");
        }
        if(cap->hasIR && irFormat != OB_FORMAT_UNKNOWN) {
            sf->ir = createStreamEncoder(baseName + "_ir_" + startTs + ".h264",
                                         irFormat, irW, irH, irFps, "nio@orbbec-captu");
            NIO_LOG_INFO_S("IR output: " << baseName + "_ir_" + startTs + ".h264");
        }
        if(cap->hasIRLeft && irLeftFormat != OB_FORMAT_UNKNOWN) {
            sf->irLeft = createStreamEncoder(baseName + "_ir_left_" + startTs + ".h264",
                                             irLeftFormat, irLW, irLH, irLFps, "nio@orbbec-captu");
            NIO_LOG_INFO_S("IR Left output: " << baseName + "_ir_left_" + startTs + ".h264");
        }
        if(cap->hasIRRight && irRightFormat != OB_FORMAT_UNKNOWN) {
            sf->irRight = createStreamEncoder(baseName + "_ir_right_" + startTs + ".h264",
                                              irRightFormat, irRW, irRH, irRFps, "nio@orbbec-captu");
            NIO_LOG_INFO_S("IR Right output: " << baseName + "_ir_right_" + startTs + ".h264");
        }
        if(hasAccel || hasGyro) {
            sf->imuFile = std::make_shared<std::ofstream>(
                baseName + "_imu_" + startTs + ".txt");
            *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
            sf->imuFile->flush();
            NIO_LOG_INFO_S("IMU output: " << baseName + "_imu_" + startTs + ".txt");
        }

        auto depthFrameIdx = std::make_shared<std::atomic<uint64_t>>(0);

        try {
            cap->videoPipeline->start(config,
                [cap, sf, depthFrameIdx](std::shared_ptr<ob::FrameSet> frameSet) {
                if(!frameSet) return;

                if(cap->hasColor) {
                    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                    if(colorFrame) {
                        writeStreamFrame(sf->color.get(), colorFrame->getData(),
                                         colorFrame->getDataSize());

                        cv::Mat bgr = frameToBGR(colorFrame);
                        if(!bgr.empty()) {
                            std::lock_guard<std::mutex> lock(cap->displayMtx);
                            bgr.copyTo(cap->colorBGR);
                        }

                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_COLOR]++;
                    }
                }

                if(cap->hasDepth) {
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

                        float scale = cap->depthScale;
                        try {
                            auto depthF = depthFrame->as<ob::DepthFrame>();
                            if(depthF) scale = depthF->getValueScale();
                        } catch(...) {}

                        cv::Mat dc = colorizeDepth(depthFrame, scale, cap->depthMinM, cap->depthMaxM);
                        if(!dc.empty()) {
                            std::lock_guard<std::mutex> lock(cap->displayMtx);
                            dc.copyTo(cap->depthColorized);
                        }

                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_DEPTH]++;
                    }
                }

                if(cap->hasIR) {
                    auto irFrame = frameSet->getFrame(OB_FRAME_IR);
                    if(irFrame) {
                        writeStreamFrame(sf->ir.get(), irFrame->getData(), irFrame->getDataSize());

                        cv::Mat irBgr = frameToBGR(irFrame);
                        if(!irBgr.empty()) {
                            std::lock_guard<std::mutex> lock(cap->displayMtx);
                            irBgr.copyTo(cap->irBGR);
                        }

                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_IR]++;
                    }
                }

                if(cap->hasIRLeft) {
                    auto irLeftFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
                    if(irLeftFrame) {
                        writeStreamFrame(sf->irLeft.get(), irLeftFrame->getData(), irLeftFrame->getDataSize());

                        cv::Mat irLBgr = frameToBGR(irLeftFrame);
                        if(!irLBgr.empty()) {
                            std::lock_guard<std::mutex> lock(cap->displayMtx);
                            irLBgr.copyTo(cap->irLeftBGR);
                        }

                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_IR_LEFT]++;
                    }
                }

                if(cap->hasIRRight) {
                    auto irRightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
                    if(irRightFrame) {
                        writeStreamFrame(sf->irRight.get(), irRightFrame->getData(), irRightFrame->getDataSize());

                        cv::Mat irRBgr = frameToBGR(irRightFrame);
                        if(!irRBgr.empty()) {
                            std::lock_guard<std::mutex> lock(cap->displayMtx);
                            irRBgr.copyTo(cap->irRightBGR);
                        }

                        std::lock_guard<std::mutex> lock(sf->countMtx);
                        sf->frameCounts[OB_FRAME_IR_RIGHT]++;
                    }
                }
            });
        } catch(ob::Error &e) {
            std::cerr << "  Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
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
        if(!cfg.deviceFilter.empty()) {
            std::cerr << "Available devices:" << std::endl;
            for(uint32_t i = 0; i < deviceList->getCount(); i++) {
                auto dev = deviceList->getDevice(i);
                std::cerr << "  - " << dev->getDeviceInfo()->getName() << std::endl;
            }
        }
        return -1;
    }

    std::cout << "\n=== Multi-Capture+CV recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << captures.size() << " device(s)" << std::endl;
    std::cout << "Depth colormap range: " << cfg.depthMinM << "m - " << cfg.depthMaxM << "m" << std::endl;
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;
    NIO_LOG_INFO_S("=== Multi-Capture+CV recording started === devices=" << captures.size() << " outputDir=" << outputRootDir);
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    cv::namedWindow("Multi Capture CV", cv::WINDOW_NORMAL);

    auto lastReportTime = ob_smpl::getNowTimesMs();

    while(g_running) {
        {
            std::vector<cv::Mat> tiles;
            for(auto &cap : captures) {
                cv::Mat tile = buildDeviceTile(cap.get());
                if(!tile.empty()) tiles.push_back(tile);
            }

            if(!tiles.empty()) {
                cv::Mat display;
                if(tiles.size() == 1) {
                    display = tiles[0];
                } else {
                    std::vector<cv::Mat> rows;
                    int maxW = 0;
                    for(const auto &t : tiles) maxW = std::max(maxW, t.cols);
                    for(auto &t : tiles) {
                        if(t.cols < maxW) {
                            cv::Mat padded;
                            cv::copyMakeBorder(t, padded, 0, 0, 0, maxW - t.cols,
                                               cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
                            t = padded;
                        }
                        rows.push_back(t);
                    }
                    cv::vconcat(rows, display);
                }

                if(!display.empty()) {
                    auto now = std::chrono::system_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
                    time_t secs = static_cast<time_t>(ms / 1000);
                    struct tm t;
                    localtime_r(&secs, &t);
                    char timeBuf[64];
                    snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02d %02d:%02d:%02d",
                             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                             t.tm_hour, t.tm_min, t.tm_sec);
                    cv::putText(display, timeBuf, cv::Point(8, 24),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                    cv::imshow("Multi Capture CV", display);
                }
            }
        }

        int key = cv::waitKey(1);
        if(key == 27 || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        auto currentTime = ob_smpl::getNowTimesMs();
        if(currentTime >= lastReportTime + 2000) {
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
                if(tempCounts.empty()) {
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
                        NIO_LOG_TRACE_S("[" << cap->deviceName << "] " << name << "="
                                        << std::fixed << std::setprecision(1) << rate);
                    }
                }
                std::cout << std::endl;
            }
        }
    }

    cv::destroyAllWindows();

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

        std::cout << "Stopped: " << cap->deviceName << std::endl;
        NIO_LOG_INFO_S("Stopped device: " << cap->deviceName);
    }

    std::cout << "All recordings saved to: " << outputRootDir << "/" << std::endl;
    NIO_LOG_INFO_S("All recordings saved to: " << outputRootDir << "/");
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
