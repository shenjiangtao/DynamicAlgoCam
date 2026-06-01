// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_d2c_fusion_cv.cpp — Depth-to-color fusion with OpenCV display.
// Composites depth (colorized) and color into a BGR frame, encoded to
// H.264 and shown via cv::imshow. Uses shared nio:: utilities.

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
#include <cmath>

using namespace nio;

struct DeviceFusion {
    std::shared_ptr<ob::Pipeline> pipeline;
    std::string deviceName;

    std::shared_ptr<H264Encoder> fusedEncoder;
    std::shared_ptr<std::ofstream> fusedFile;
    std::mutex fusedMtx;

    int colorW;
    int colorH;
    int fps;
    float depthScale;
    float alpha;
    float depthMinM;
    float depthMaxM;

    std::shared_ptr<std::atomic<uint64_t>> fusedFrameCount;

    cv::Mat fusedBGR;
    std::mutex displayMtx;
};

struct FusionConfig {
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    std::vector<std::string> deviceFilter;
};

static void printUsage() {
    std::cout << "Usage: nio_d2c_fusion_cv [device_name_filter...] [options]\n"
              << "Options:\n"
              << "  --alpha VALUE   Depth overlay opacity 0.0-1.0 (default: 0.5)\n"
              << "  --depth-min M   Min depth in meters for colormap (default: 0.3)\n"
              << "  --depth-max M   Max depth in meters for colormap (default: 5.0)\n"
              << "  --help          Show this help\n"
              << "\nExample:\n"
              << "  nio_d2c_fusion_cv                       # all devices, default alpha=0.5\n"
              << "  nio_d2c_fusion_cv 336L --alpha 0.7      # device with '336L' in name\n"
              << std::endl;
}

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

int main(int argc, char **argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    FusionConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_d2c_fusion_cv", "fusion_cv_output");
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
    std::string outputRootDir = "fusion_cv_output/" + sessionTimestamp;
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
            std::cerr << "  Timer sync warning: " << e.what() << std::endl;
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
                    std::cout << "  Color: " << colorW << "x" << colorH
                              << "@" << colorFps << " format=" << colorFormat << std::endl;
                    NIO_LOG_INFO_S("Color stream: " << colorW << "x" << colorH
                                   << "@" << colorFps << " format=" << colorFormat);
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
                    std::cout << "  Depth: " << depthW << "x" << depthH
                              << "@" << depthFps << " format=" << depthFormat << std::endl;
                    NIO_LOG_INFO_S("Depth stream: " << depthW << "x" << depthH
                                   << "@" << depthFps << " format=" << depthFormat);
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
                    std::cout << "  Depth scale: " << df->depthScale
                              << " (precision " << precisionLevel << ")" << std::endl;
                    NIO_LOG_INFO_S("Depth scale: " << df->depthScale << " precision=" << precisionLevel);
                } catch(...) {
                    df->depthScale = 0.001f;
                    std::cout << "  Depth scale: 0.001 (default)" << std::endl;
                    NIO_LOG_INFO("Depth scale: 0.001 (default)");
                }
                break;
            default: break;
            }
        }

        if(!hasColor || !hasDepth) {
            std::cerr << "  Device " << safeName
                      << " needs both color+depth for D2C fusion, skipping" << std::endl;
            NIO_LOG_WARN_S(safeName << " needs both color+depth for D2C fusion, hasColor="
                           << hasColor << " hasDepth=" << hasDepth << ", skipping");
            continue;
        }

        df->colorW = colorW;
        df->colorH = colorH;
        df->fps = std::min(colorFps, depthFps);
        df->fusedBGR = cv::Mat::zeros(colorH, colorW, CV_8UC3);

        auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);

        std::string startTs = getTimestampMs();
        std::string fusedPath = deviceOutputDir + "/" + safeName + "_d2c_fused_" + startTs + ".h264";
        df->fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

        df->fusedEncoder = std::make_shared<H264Encoder>();
        if(!df->fusedEncoder->initBGR(colorW, colorH, df->fps)) {
            std::cerr << "  Failed to init fused H264 encoder for " << safeName << std::endl;
            NIO_LOG_ERROR_S("Failed to init fused H264 encoder for " << safeName
                            << " " << colorW << "x" << colorH << "@" << df->fps);
            continue;
        }

        auto pid = devInfo->getPid();
        auto vid = devInfo->getVid();
        if(ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
        }

        try { df->pipeline->enableFrameSync(); } catch(...) {}

        try {
            df->pipeline->start(config,
                [df, alignFilter](std::shared_ptr<ob::FrameSet> frameSet) {
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

                    cv::Mat colorBGR = frameToBGR(colorFrame);
                    if(colorBGR.empty()) return;

                    float scale = df->depthScale;
                    try {
                        auto depthF = depthFrame->as<ob::DepthFrame>();
                        if(depthF) scale = depthF->getValueScale();
                    } catch(...) {}

                    cv::Mat depthColor = colorizeDepth(depthFrame, scale, df->depthMinM, df->depthMaxM);
                    if(depthColor.empty()) {
                        std::lock_guard<std::mutex> lock(df->displayMtx);
                        colorBGR.copyTo(df->fusedBGR);
                    } else {
                        if(depthColor.size() != colorBGR.size()) {
                            cv::resize(depthColor, depthColor, colorBGR.size());
                        }

                        cv::Mat blended;
                        cv::addWeighted(colorBGR, 1.0 - df->alpha, depthColor, df->alpha, 0.0, blended);

                        std::lock_guard<std::mutex> lock(df->displayMtx);
                        blended.copyTo(df->fusedBGR);
                    }

                    {
                        std::lock_guard<std::mutex> lock(df->displayMtx);
                        df->fusedEncoder->encodeBGR(df->fusedBGR.data, *df->fusedFile,
                                                    df->fusedMtx, getTimestampMsInt());
                    }
                    (*df->fusedFrameCount)++;
                });
        } catch(ob::Error &e) {
            std::cerr << "  Pipeline start failed for " << safeName
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

    std::cout << "\n=== D2C Fusion+CV recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << fusions.size() << " device(s)" << std::endl;
    std::cout << "Alpha: " << cfg.alpha << ", Depth range: "
              << cfg.depthMinM << "m - " << cfg.depthMaxM << "m" << std::endl;
    std::cout << "Press Ctrl+C or 'q' to stop.\n" << std::endl;
    NIO_LOG_INFO_S("=== D2C Fusion+CV recording started === devices=" << fusions.size()
                   << " alpha=" << cfg.alpha << " depthRange=" << cfg.depthMinM << "m-" << cfg.depthMaxM << "m");
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    cv::namedWindow("D2C Fusion", cv::WINDOW_NORMAL);

    auto lastReportTime = ob_smpl::getNowTimesMs();

    while(g_running) {
        {
            cv::Mat display;
            if(fusions.size() == 1) {
                std::lock_guard<std::mutex> lock(fusions[0]->displayMtx);
                fusions[0]->fusedBGR.copyTo(display);
            } else {
                std::vector<cv::Mat> mats;
                for(auto &df : fusions) {
                    std::lock_guard<std::mutex> lock(df->displayMtx);
                    mats.push_back(df->fusedBGR.clone());
                }
                if(!mats.empty()) cv::hconcat(mats, display);
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
                std::string overlay = std::string(timeBuf) + " alpha=" + std::to_string(cfg.alpha).substr(0, 4);
                cv::putText(display, overlay, cv::Point(8, 24),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::imshow("D2C Fusion", display);
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

            for(auto &df : fusions) {
                uint64_t count = df->fusedFrameCount->exchange(0);
                float rate = (reportDuration > 0) ? (count / (reportDuration / 1000.0f)) : 0.0f;
                std::cout << "[" << df->deviceName << "] Fusion FPS: "
                          << std::fixed << std::setprecision(1) << rate << std::endl;
                NIO_LOG_TRACE_S("[" << df->deviceName << "] Fusion FPS: "
                                << std::fixed << std::setprecision(1) << rate);
            }
        }
    }

    cv::destroyAllWindows();

    std::cout << "\n=== Stopping fusion recording ===" << std::endl;
    NIO_LOG_INFO("=== Stopping fusion recording ===");

    for(auto &df : fusions) {
        if(df->pipeline) df->pipeline->stop();
        if(df->fusedEncoder) df->fusedEncoder->close();
        if(df->fusedFile) df->fusedFile->close();
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
