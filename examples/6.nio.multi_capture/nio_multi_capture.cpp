// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_multi_capture.cpp — Multi-device capture example. Records color,
// depth, and IR streams to H.264 / raw files with IMU CSV logging.
// Additionally performs D2C (depth-to-color) alignment and alpha-blends
// depth (jet colormap) with color, saving the fused result as H.264.
// Uses shared nio:: utilities from examples/utils/.
//
// Threading model:
//   - SDK video callback: lightweight — enqueues frame data to per-stream
//     EncodeStreamTask / DepthRawTask / FusionStreamTask threads, and pushes
//     to SDLViewer (memcpy only). No encoding/fusion occurs in the callback.
//   - SDK IMU callback: formats CSV lines, enqueues to ImuStreamTask.
//   - Each stream has its own named worker thread for H.264 encoding, depth
//     raw writing, fusion blending, or IMU CSV logging.
//   - SDLViewer runs its own named decode + render threads.
//
// Usage:
//   ./nio_multi_capture                                          # all devices
//   ./nio_multi_capture -c "305" "336L"                          # filter by camera type
//   ./nio_multi_capture -s /HDD/nio_capture                      # custom save directory
//   ./nio_multi_capture -c "305" -s /HDD/nio_capture --alpha 0.6 # combined

#include "nio_color_convert.hpp"
#include "nio_common.hpp"
#include "nio_h264_encoder.hpp"
#include "nio_log.hpp"
#include "nio_sdl_viewer.hpp"
#include "nio_stream_io.hpp"
#include "nio_stream_tasks.hpp"
#include "nio_thread.hpp"
#include "utils.hpp"
#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "unknown"
#endif

using namespace nio;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

struct DeviceCapture
{
    std::shared_ptr<ob::Pipeline> videoPipeline;
    std::shared_ptr<ob::Pipeline> imuPipeline;
    std::string deviceName;
    std::shared_ptr<SensorFiles> sensorFiles;
    bool hasIMU = false;
    float depthScale = 0.001f;

    std::shared_ptr<ob::Align> alignFilter;
    std::mutex fusedMtx;
    std::shared_ptr<MjpgDecoderRes> mjpgRes;
    int colorW = 0;
    int colorH = 0;
    int fusedFps = 30;
    OBFormat colorFormat = OB_FORMAT_UNKNOWN;
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    bool enableFusion = true;
    bool hwD2CMode = false;
    int viewerIdx = -1;

    std::shared_ptr<EncodeStreamTask> colorEncodeTask;
    std::shared_ptr<EncodeStreamTask> depthEncodeTask;
    std::shared_ptr<EncodeStreamTask> irEncodeTask;
    std::shared_ptr<EncodeStreamTask> irLeftEncodeTask;
    std::shared_ptr<EncodeStreamTask> irRightEncodeTask;
    std::shared_ptr<DepthRawTask> depthRawTask;
    std::shared_ptr<FusionStreamTask> fusionTask;
    std::shared_ptr<ImuStreamTask> imuTask;
};

struct CaptureConfig
{
    std::vector<std::string> cameraFilter;
    std::string saveDir;
    float alpha = 0.5f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
    bool enableFusion = true;
    bool noFusion = false;
    bool noShow = false;
};

static void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options] [camera_name_filter...]\n"
              << "\nOptions:\n"
              << "  -c <name...>  Camera type filter (can specify multiple, e.g. -c \"305\" \"336L\")\n"
              << "  -s <dir>      Save directory (default: capture_output/)\n"
              << "  --alpha VAL   Depth overlay opacity 0.0-1.0 (default: 0.5)\n"
              << "  --depth-min M Min depth in meters for colormap (default: 0.3)\n"
              << "  --depth-max M Max depth in meters for colormap (default: 5.0)\n"
              << "  --no-fusion   Disable D2C fusion output (only save individual streams)\n"
              << "  --no-show     Disable SDL live preview window\n"
              << "  --help        Show this help\n"
              << "\nExamples:\n"
              << "  " << prog << "                                 # all devices, default settings\n"
              << "  " << prog << " -c \"305\" \"336L\"             # filter cameras by type\n"
              << "  " << prog << " -s /HDD/nio_capture             # custom save directory\n"
              << "  " << prog << " -c \"305\" --alpha 0.6          # combined options\n"
              << std::endl;
}

static CaptureConfig parseArgs(int argc, char** argv) {
    CaptureConfig cfg;
    static struct option longOpts[] = { { "alpha", required_argument, nullptr, 'a' },
                                        { "depth-min", required_argument, nullptr, 'm' },
                                        { "depth-max", required_argument, nullptr, 'x' },
                                        { "no-fusion", no_argument, nullptr, 'n' },
                                        { "no-show", no_argument, nullptr, 'S' },
                                        { "help", no_argument, nullptr, 'h' },
                                        { nullptr, 0, nullptr, 0 } };

    opterr = 0;
    int ch;
    int optIdx = 0;

    while ((ch = getopt_long(argc, argv, "c:s:h", longOpts, &optIdx)) != -1) {
        switch (ch) {
        case 'c':
            cfg.cameraFilter.push_back(optarg);
            while (optind < argc && argv[optind][0] != '-') {
                cfg.cameraFilter.push_back(argv[optind++]);
            }
            break;
        case 's':
            cfg.saveDir = optarg;
            while (!cfg.saveDir.empty() && cfg.saveDir.back() == '/')
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
        case 'S':
            cfg.noShow = true;
            break;
        case 'h':
            printUsage(argv[0]);
            exit(0);
        default:
            break;
        }
    }
    for (int i = optind; i < argc; i++) {
        cfg.cameraFilter.push_back(argv[i]);
    }

    if (cfg.noFusion) {
        cfg.enableFusion = false;
    }

    return cfg;
}

static bool checkIfSupportHWD2CAlign(std::shared_ptr<ob::Pipeline> pipe,
                                     std::shared_ptr<ob::StreamProfile> colorProfile,
                                     std::shared_ptr<ob::StreamProfile> depthProfile) {
    auto hwD2CDepthProfiles = pipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
    if (!hwD2CDepthProfiles || hwD2CDepthProfiles->getCount() == 0) {
        return false;
    }
    auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();
    auto count = hwD2CDepthProfiles->getCount();
    for (uint32_t i = 0; i < count; i++) {
        auto sp = hwD2CDepthProfiles->getProfile(i);
        auto vsp = sp->as<ob::VideoStreamProfile>();
        if (vsp->getWidth() == depthVsp->getWidth() && vsp->getHeight() == depthVsp->getHeight() &&
            vsp->getFormat() == depthVsp->getFormat() && vsp->getFps() == depthVsp->getFps()) {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    CaptureConfig cfg = parseArgs(argc, argv);

    NIO_LOG_INIT("nio_multi_capture", cfg.saveDir.empty() ? "capture_output" : cfg.saveDir);
    NIO_LOG_SET_LEVEL(nio::LogLevel::TRACE);
    NIO_LOG_INFO_S("Git commit: " << GIT_COMMIT_HASH);
    std::cout << "Git commit: " << GIT_COMMIT_HASH << std::endl;
    NIO_LOG_INFO_S("Process started, camera_filter_count="
                   << cfg.cameraFilter.size() << " saveDir=" << (cfg.saveDir.empty() ? "capture_output" : cfg.saveDir)
                   << " alpha=" << cfg.alpha << " depthMin=" << cfg.depthMinM << " depthMax=" << cfg.depthMaxM
                   << " fusion=" << (cfg.enableFusion ? "on" : "off"));
    for (size_t i = 0; i < cfg.cameraFilter.size(); i++) {
        NIO_LOG_DEBUG_S("Camera filter[" << i << "]=" << cfg.cameraFilter[i]);
    }

    ob::Context context;

    auto deviceList = context.queryDeviceList();
    if (deviceList->getCount() < 1) {
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
        if (usbfsFile.is_open()) {
            int usbfsMb = 0;
            usbfsFile >> usbfsMb;
            if (usbfsMb < 128 && deviceList->getCount() > 1) {
                std::cerr << "WARNING: usbfs_memory_mb=" << usbfsMb << "MB is too low for " << deviceList->getCount()
                          << " devices. Recommend >= 128MB." << std::endl;
                std::cerr << "Fix: echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb" << std::endl;
                std::cerr << "Or: sudo modprobe usbcore usbfs_memory_mb=256" << std::endl;
                NIO_LOG_WARN_S("usbfs_memory_mb=" << usbfsMb << "MB is too low for " << deviceList->getCount()
                                                  << " devices, recommend >= 128MB");
            }
        }
    }

    // -----------------------------------------------------------------------
    // Device enumeration & per-device setup
    // -----------------------------------------------------------------------
    std::vector<std::shared_ptr<DeviceCapture>> captures;

    SDLViewer viewer;
    if (!cfg.noShow) {
        if (!viewer.init()) {
            std::cerr << "SDL viewer init failed, continuing without preview" << std::endl;
            NIO_LOG_WARN_S("SDL viewer init failed, continuing without preview");
        }
    }

    for (uint32_t i = 0; i < deviceList->getCount(); i++) {
        auto device = deviceList->getDevice(i);
        auto devInfo = device->getDeviceInfo();
        std::string name = devInfo->getName();

        if (!deviceMatches(name, cfg.cameraFilter)) {
            std::cout << "Skipping device: " << name << std::endl;
            NIO_LOG_DEBUG_S("Skipping device: " << name << " (does not match filter)");
            continue;
        }

        std::cout << "Found device: " << name << " (SN: " << devInfo->getSerialNumber() << ", PID: 0x" << std::hex
                  << std::setw(4) << std::setfill('0') << devInfo->getPid() << std::dec << ", "
                  << devInfo->getConnectionType() << ")" << std::endl;
        NIO_LOG_INFO_S("Found device: " << name << " SN=" << devInfo->getSerialNumber() << " PID=0x" << std::hex
                                        << devInfo->getPid() << std::dec << " conn=" << devInfo->getConnectionType());

        auto safeName = name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');
        std::string serialNumber = devInfo->getSerialNumber();
        safeName = safeName + "_" + serialNumber;

        // Short device ID for thread names (first 8 chars, no spaces)
        std::string devId = safeName;
        if (devId.size() > 8)
            devId = devId.substr(0, 8);

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

        auto startTs = getTimestampMs();
        std::string baseName = deviceOutputDir + "/" + safeName;

        try {
            device->timerSyncWithHost();
        } catch (ob::Error& e) {
            std::cerr << "Timer sync warning: " << e.what() << std::endl;
            NIO_LOG_WARN_S("Timer sync failed for " << safeName << ": " << e.what());
        }

        if (device->isGlobalTimestampSupported()) {
            try {
                device->enableGlobalTimestamp(true);
            } catch (...) {
            }
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

        bool is305 = ob_smpl::isGemini305Device(vid, pid);
        OBFormat colorPreferredFmt = is305 ? OB_FORMAT_YUYV : OB_FORMAT_MJPG;

        for (uint32_t s = 0; s < sensorList->getCount(); s++) {
            auto sensorType = sensorList->getSensorType(s);
            auto sensor = sensorList->getSensor(s);
            auto profileList = sensor->getStreamProfileList();

            switch (sensorType) {
            case OB_SENSOR_COLOR:
                hasColor = true;
                colorProfile = selectBestProfile(profileList, colorPreferredFmt);
                if (colorProfile) {
                    colorFormat = colorProfile->getFormat();
                    if (colorFormat == OB_FORMAT_UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    colorProfile = p;
                                    colorFormat = p->getFormat();
                                    break;
                                }
                            } catch (...) {
                            }
                        }
                    }
                    if (colorFormat != OB_FORMAT_UNKNOWN) {
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
                if (hasColor) {
                    std::cout << " Color: " << colorW << "x" << colorH << "@" << colorFps << " format=" << colorFormat
                              << std::endl;
                    NIO_LOG_INFO_S("Color stream: " << colorW << "x" << colorH << "@" << colorFps
                                                    << " format=" << colorFormat);
                }
                break;
            case OB_SENSOR_DEPTH:
                hasDepth = true;
                depthProfile = selectBestProfile(profileList, OB_FORMAT_Y16);
                if (depthProfile) {
                    depthFormat = depthProfile->getFormat();
                    if (depthFormat == OB_FORMAT_UNKNOWN) {
                        for (uint32_t k = 0; k < profileList->getCount(); k++) {
                            try {
                                auto p = profileList->getProfile(k)->as<ob::VideoStreamProfile>();
                                if (p && p->getFormat() != OB_FORMAT_UNKNOWN) {
                                    depthProfile = p;
                                    depthFormat = p->getFormat();
                                    break;
                                }
                            } catch (...) {
                            }
                        }
                    }
                    if (depthFormat != OB_FORMAT_UNKNOWN) {
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
                if (hasDepth) {
                    std::cout << " Depth: " << depthW << "x" << depthH << "@" << depthFps << " format=" << depthFormat
                              << std::endl;
                    NIO_LOG_INFO_S("Depth stream: " << depthW << "x" << depthH << "@" << depthFps
                                                    << " format=" << depthFormat);
                }
                try {
                    int32_t precisionLevel = device->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
                    switch (precisionLevel) {
                    case 0:
                        cap->depthScale = 0.001f;
                        break;
                    case 1:
                        cap->depthScale = 0.0005f;
                        break;
                    case 2:
                        cap->depthScale = 0.00025f;
                        break;
                    case 3:
                        cap->depthScale = 0.0001f;
                        break;
                    default:
                        cap->depthScale = 0.001f;
                        break;
                    }
                    std::cout << " Depth scale: " << cap->depthScale << " (precision level " << precisionLevel << ")"
                              << std::endl;
                    NIO_LOG_INFO_S("Depth scale: " << cap->depthScale << " precision_level=" << precisionLevel);
                } catch (...) {
                    cap->depthScale = 0.001f;
                    std::cout << " Depth scale: 0.001 (default)" << std::endl;
                }
                break;
            case OB_SENSOR_IR:
                hasIR = true;
                irProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (irProfile) {
                    irFormat = irProfile->getFormat();
                    if (irFormat == OB_FORMAT_UNKNOWN)
                        irFormat = OB_FORMAT_Y8;
                    config->enableStream(irProfile);
                    irW = irProfile->getWidth();
                    irH = irProfile->getHeight();
                    irFps = irProfile->getFps();
                } else {
                    hasIR = false;
                }
                if (hasIR) {
                    std::cout << " IR: " << irW << "x" << irH << "@" << irFps << " format=" << irFormat << std::endl;
                    NIO_LOG_INFO_S("IR stream: " << irW << "x" << irH << "@" << irFps << " format=" << irFormat);
                }
                break;
            case OB_SENSOR_IR_LEFT:
                hasIRLeft = true;
                irLeftProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (irLeftProfile) {
                    irLeftFormat = irLeftProfile->getFormat();
                    if (irLeftFormat == OB_FORMAT_UNKNOWN)
                        irLeftFormat = OB_FORMAT_Y8;
                    config->enableStream(irLeftProfile);
                    irLW = irLeftProfile->getWidth();
                    irLH = irLeftProfile->getHeight();
                    irLFps = irLeftProfile->getFps();
                } else {
                    hasIRLeft = false;
                }
                if (hasIRLeft) {
                    std::cout << " IR Left: " << irLW << "x" << irLH << "@" << irLFps << " format=" << irLeftFormat
                              << std::endl;
                    NIO_LOG_INFO_S("IR Left stream: " << irLW << "x" << irLH << "@" << irLFps
                                                      << " format=" << irLeftFormat);
                }
                break;
            case OB_SENSOR_IR_RIGHT:
                hasIRRight = true;
                irRightProfile = selectBestProfile(profileList, OB_FORMAT_Y8);
                if (irRightProfile) {
                    irRightFormat = irRightProfile->getFormat();
                    if (irRightFormat == OB_FORMAT_UNKNOWN)
                        irRightFormat = OB_FORMAT_Y8;
                    config->enableStream(irRightProfile);
                    irRW = irRightProfile->getWidth();
                    irRH = irRightProfile->getHeight();
                    irRFps = irRightProfile->getFps();
                } else {
                    hasIRRight = false;
                }
                if (hasIRRight) {
                    std::cout << " IR Right: " << irRW << "x" << irRH << "@" << irRFps << " format=" << irRightFormat
                              << std::endl;
                    NIO_LOG_INFO_S("IR Right stream: " << irRW << "x" << irRH << "@" << irRFps
                                                       << " format=" << irRightFormat);
                }
                break;
            case OB_SENSOR_ACCEL:
                hasAccel = true;
                break;
            case OB_SENSOR_GYRO:
                hasGyro = true;
                break;
            default:
                break;
            }
        }

        if (ob_smpl::isGemini305gDevice(vid, pid, devInfo->getConnectionType())) {
            config->disableStream(OB_SENSOR_IR_LEFT);
            hasIRLeft = false;
            std::cout << "  Gemini 305g: disabled IR_LEFT" << std::endl;
            NIO_LOG_INFO("Gemini 305g detected, disabled IR_LEFT stream");
        }

        bool hwD2CSupported = false;
        if (hasColor && hasDepth && colorProfile && depthProfile) {
            hwD2CSupported = checkIfSupportHWD2CAlign(cap->videoPipeline, colorProfile, depthProfile);
            if (hwD2CSupported) {
                config->setAlignMode(ALIGN_D2C_HW_MODE);
                cap->hwD2CMode = true;
                std::cout << "  HW D2C: supported, using hardware depth-to-color alignment" << std::endl;
                NIO_LOG_INFO_S("HW D2C supported for " << safeName << ", using ALIGN_D2C_HW_MODE");
            } else {
                std::cout << "  HW D2C: not supported, using software alignment" << std::endl;
                NIO_LOG_INFO_S("HW D2C not supported for " << safeName << ", using SW alignment");
            }
        }

        // -----------------------------------------------------------------------
        // Create H264 encoders + output files + per-stream tasks
        // -----------------------------------------------------------------------
        auto sf = cap->sensorFiles;

        if (hasColor && colorFormat != OB_FORMAT_UNKNOWN) {
            sf->color = createStreamEncoder(baseName + "_color_" + startTs + ".h264", colorFormat, colorW, colorH,
                                            colorFps, nullptr, false);
            cap->colorEncodeTask = std::make_shared<EncodeStreamTask>(devId + "_color_enc", sf->color);
            cap->colorEncodeTask->start();
            NIO_LOG_INFO_S("Color output: " << baseName + "_color_" + startTs + ".h264" << " fmt=" << colorFormat);
        }
        if (hasDepth && depthFormat != OB_FORMAT_UNKNOWN) {
            sf->depth = createStreamEncoder(baseName + "_depth_" + startTs + ".h264", depthFormat, depthW, depthH,
                                            depthFps, nullptr, false);
            sf->depthRawFile =
                std::make_shared<std::ofstream>(baseName + "_depth_raw_" + startTs + ".raw", std::ios::binary);
            cap->depthEncodeTask = std::make_shared<EncodeStreamTask>(devId + "_depth_enc", sf->depth);
            cap->depthEncodeTask->start();
            cap->depthRawTask =
                std::make_shared<DepthRawTask>(devId + "_depth_raw", sf->depthRawFile, depthW, depthH, cap->depthScale);
            cap->depthRawTask->start();
            NIO_LOG_INFO_S("Depth output: " << baseName + "_depth_" + startTs + ".h264" << " + raw");
        }
        if (hasIR && irFormat != OB_FORMAT_UNKNOWN) {
            sf->ir =
                createStreamEncoder(baseName + "_ir_" + startTs + ".h264", irFormat, irW, irH, irFps, nullptr, false);
            cap->irEncodeTask = std::make_shared<EncodeStreamTask>(devId + "_ir_enc", sf->ir);
            cap->irEncodeTask->start();
            NIO_LOG_INFO_S("IR output: " << baseName + "_ir_" + startTs + ".h264");
        }
        if (hasIRLeft && irLeftFormat != OB_FORMAT_UNKNOWN) {
            sf->irLeft = createStreamEncoder(baseName + "_ir_left_" + startTs + ".h264", irLeftFormat, irLW, irLH,
                                             irLFps, nullptr, false);
            cap->irLeftEncodeTask = std::make_shared<EncodeStreamTask>(devId + "_irl_enc", sf->irLeft);
            cap->irLeftEncodeTask->start();
            NIO_LOG_INFO_S("IR Left output: " << baseName + "_ir_left_" + startTs + ".h264");
        }
        if (hasIRRight && irRightFormat != OB_FORMAT_UNKNOWN) {
            sf->irRight = createStreamEncoder(baseName + "_ir_right_" + startTs + ".h264", irRightFormat, irRW, irRH,
                                              irRFps, nullptr, false);
            cap->irRightEncodeTask = std::make_shared<EncodeStreamTask>(devId + "_irr_enc", sf->irRight);
            cap->irRightEncodeTask->start();
            NIO_LOG_INFO_S("IR Right output: " << baseName + "_ir_right_" + startTs + ".h264");
        }
        if (hasAccel || hasGyro) {
            sf->imuFile = std::make_shared<std::ofstream>(baseName + "_imu_" + startTs + ".txt");
            *sf->imuFile << "# host_ts_ms,type,device_ts_us,x,y,z,temperature\n";
            sf->imuFile->flush();
            cap->imuTask = std::make_shared<ImuStreamTask>(devId + "_imu", sf->imuFile);
            cap->imuTask->start();
            NIO_LOG_INFO_S("IMU output: " << baseName + "_imu_" + startTs + ".txt");
        }

        OBFormat depthSlotFmt = OB_FORMAT_Y16;
        int depthSlotW = depthW;
        int depthSlotH = depthH;
        if (hasDepth && cap->hwD2CMode && hasColor) {
            depthSlotW = colorW;
            depthSlotH = colorH;
        }
        if (!cfg.noShow) {
            std::string camType = name;
            std::replace(camType.begin(), camType.end(), ' ', '_');
            cap->viewerIdx = viewer.addDevice(safeName, camType, serialNumber, hasColor, colorFormat, colorW, colorH,
                                              hasDepth, depthSlotFmt, depthSlotW, depthSlotH, hasIR, irW, irH,
                                              hasIRLeft, irLW, irLH, hasIRRight, irRW, irRH);
        }

        // -----------------------------------------------------------------------
        // D2C fusion setup
        // -----------------------------------------------------------------------
        bool canFuse = cfg.enableFusion && hasColor && hasDepth;
        if (canFuse) {
            if (!cap->hwD2CMode) {
                cap->alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            }
            cap->colorW = colorW;
            cap->colorH = colorH;
            cap->colorFormat = colorFormat;
            cap->fusedFps = std::min(colorFps, depthFps);

            std::string fusedPath = baseName + "_d2c_fused_" + startTs + ".h264";
            auto fusedFile = std::make_shared<std::ofstream>(fusedPath, std::ios::binary);

            auto fusedEncoder = std::make_shared<H264Encoder>();
            if (!fusedEncoder->initRGB(colorW, colorH, cap->fusedFps)) {
                std::cerr << "  Failed to init fused H264 encoder for " << safeName << std::endl;
                NIO_LOG_ERROR_S("Failed to init fused H264 encoder for " << safeName << " " << colorW << "x" << colorH
                                                                         << "@" << cap->fusedFps);
                canFuse = false;
            } else {
                cap->mjpgRes = std::make_shared<MjpgDecoderRes>();
                cap->mjpgRes->init(colorW, colorH, colorFormat);
                cap->fusionTask = std::make_shared<FusionStreamTask>(
                    devId + "_fusion", colorW, colorH, colorFormat, cap->fusedFps, fusedEncoder, fusedFile,
                    cap->fusedMtx, cap->alignFilter, cap->hwD2CMode, cfg.alpha, cfg.depthMinM, cfg.depthMaxM,
                    cap->depthScale, cap->mjpgRes);
                cap->fusionTask->start();
                std::cout << "  D2C Fusion: " << colorW << "x" << colorH << "@" << cap->fusedFps
                          << " alpha=" << cfg.alpha << " depth=[" << cfg.depthMinM << "m, " << cfg.depthMaxM << "m]"
                          << " mode=" << (cap->hwD2CMode ? "HW" : "SW") << std::endl;
                NIO_LOG_INFO_S("D2C Fusion enabled: "
                               << colorW << "x" << colorH << "@" << cap->fusedFps << " alpha=" << cfg.alpha
                               << " depthRange=" << cfg.depthMinM << "m-" << cfg.depthMaxM << "m"
                               << " mode=" << (cap->hwD2CMode ? "HW" : "SW") << " output=" << fusedPath);
            }
        } else if (cfg.enableFusion && !hasColor) {
            std::cout << " D2C Fusion: skipped (no color sensor)" << std::endl;
            NIO_LOG_DEBUG_S("D2C Fusion skipped for " << safeName << ": no color sensor");
        } else if (cfg.enableFusion && !hasDepth) {
            std::cout << " D2C Fusion: skipped (no depth sensor)" << std::endl;
            NIO_LOG_DEBUG_S("D2C Fusion skipped for " << safeName << ": no depth sensor");
        }

        // -----------------------------------------------------------------------
        // Start video pipeline with lightweight frame callback
        // -----------------------------------------------------------------------
        try {
            cap->videoPipeline->enableFrameSync();
        } catch (...) {
        }

        try {
            cap->videoPipeline->start(config, [cap, canFuse, &viewer](std::shared_ptr<ob::FrameSet> frameSet) {
                if (!frameSet)
                    return;

                // ---- D2C fusion: enqueue to fusion task ----
                if (canFuse && cap->fusionTask) {
                    if (cap->hwD2CMode) {
                        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                        auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                        if (colorFrame && depthFrame) {
                            cap->fusionTask->enqueueColor(colorFrame->getData(), colorFrame->getDataSize(),
                                                          colorFrame->getTimeStampUs());
                            float depthScaleForFusion = cap->depthScale;
                            try {
                                auto depthF = depthFrame->as<ob::DepthFrame>();
                                if (depthF)
                                    depthScaleForFusion = depthF->getValueScale();
                            } catch (...) {
                            }
                            cap->fusionTask->enqueueDepth(depthFrame->getData(), depthFrame->getDataSize(),
                                                          depthFrame->getTimeStampUs(), depthScaleForFusion);
                        }
                    } else {
                        cap->fusionTask->enqueueFrameSet(frameSet);
                    }
                }

                // ---- Individual stream recording: enqueue to per-stream tasks ----
                if (auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR)) {
                    if (cap->colorEncodeTask)
                        cap->colorEncodeTask->enqueue(colorFrame->getData(), colorFrame->getDataSize(),
                                                      colorFrame->getTimeStampUs());
                    if (cap->viewerIdx >= 0)
                        viewer.pushFrame(cap->viewerIdx, ViewerChannel::COLOR, colorFrame->getData(),
                                         colorFrame->getDataSize());
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_COLOR]++;
                }

                if (auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH)) {
                    auto format = depthFrame->getFormat();
                    auto data = depthFrame->getData();
                    auto size = depthFrame->getDataSize();

                    if (format != OB_FORMAT_H264 && format != OB_FORMAT_H265 && format != OB_FORMAT_HEVC) {
                        if (cap->depthRawTask)
                            cap->depthRawTask->enqueue(data, size, depthFrame->getTimeStampUs());
                    }

                    if (cap->depthEncodeTask)
                        cap->depthEncodeTask->enqueue(data, size, depthFrame->getTimeStampUs());

                    float viewerDepthScale = cap->depthScale;
                    try {
                        auto depthF = depthFrame->as<ob::DepthFrame>();
                        if (depthF)
                            viewerDepthScale = depthF->getValueScale();
                    } catch (...) {
                    }
                    if (cap->viewerIdx >= 0)
                        viewer.pushFrame(cap->viewerIdx, ViewerChannel::DEPTH, data, size, viewerDepthScale,
                                         cap->depthMinM, cap->depthMaxM);
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_DEPTH]++;
                }

                if (auto irFrame = frameSet->getFrame(OB_FRAME_IR)) {
                    if (cap->irEncodeTask)
                        cap->irEncodeTask->enqueue(irFrame->getData(), irFrame->getDataSize(),
                                                   irFrame->getTimeStampUs());
                    if (cap->viewerIdx >= 0)
                        viewer.pushFrame(cap->viewerIdx, ViewerChannel::IR, irFrame->getData(), irFrame->getDataSize());
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_IR]++;
                }

                if (auto irLeftFrame = frameSet->getFrame(OB_FRAME_IR_LEFT)) {
                    if (cap->irLeftEncodeTask)
                        cap->irLeftEncodeTask->enqueue(irLeftFrame->getData(), irLeftFrame->getDataSize(),
                                                       irLeftFrame->getTimeStampUs());
                    if (cap->viewerIdx >= 0)
                        viewer.pushFrame(cap->viewerIdx, ViewerChannel::IR_LEFT, irLeftFrame->getData(),
                                         irLeftFrame->getDataSize());
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_IR_LEFT]++;
                }

                if (auto irRightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT)) {
                    if (cap->irRightEncodeTask)
                        cap->irRightEncodeTask->enqueue(irRightFrame->getData(), irRightFrame->getDataSize(),
                                                        irRightFrame->getTimeStampUs());
                    if (cap->viewerIdx >= 0)
                        viewer.pushFrame(cap->viewerIdx, ViewerChannel::IR_RIGHT, irRightFrame->getData(),
                                         irRightFrame->getDataSize());
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_IR_RIGHT]++;
                }
            });
        } catch (ob::Error& e) {
            std::cerr << " Pipeline start failed for " << safeName << ": " << e.what() << std::endl;
            NIO_LOG_ERROR_S("Pipeline start failed for " << safeName << ": " << e.what());
            cap->videoPipeline.reset();
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // -----------------------------------------------------------------------
        // Start IMU pipeline with lightweight callback (enqueue CSV lines)
        // -----------------------------------------------------------------------
        cap->hasIMU = (hasAccel && hasGyro);
        if (cap->hasIMU) {
            NIO_LOG_INFO_S("Starting IMU pipeline for " << safeName);
            auto imuDev = cap->videoPipeline->getDevice();
            cap->imuPipeline = std::make_shared<ob::Pipeline>(imuDev);
            std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
            imuConfig->enableAccelStream();
            imuConfig->enableGyroStream();
            imuConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

            cap->imuPipeline->start(imuConfig, [cap](std::shared_ptr<ob::FrameSet> frameSet) {
                if (!frameSet)
                    return;

                auto accelFrameRaw = frameSet->getFrame(OB_FRAME_ACCEL);
                auto gyroFrameRaw = frameSet->getFrame(OB_FRAME_GYRO);

                if (accelFrameRaw) {
                    try {
                        auto accelFrame = accelFrameRaw->as<ob::AccelFrame>();
                        auto val = accelFrame->getValue();
                        auto ts = accelFrame->getTimeStampUs();
                        auto temp = accelFrame->getTemperature();
                        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::system_clock::now().time_since_epoch())
                                         .count();
                        std::ostringstream oss;
                        oss << nowMs << ",ACCEL," << ts << "," << val.x << "," << val.y << "," << val.z << "," << temp
                            << "\n";
                        if (cap->imuTask)
                            cap->imuTask->enqueueLine(oss.str());
                    } catch (...) {
                    }
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_ACCEL]++;
                }

                if (gyroFrameRaw) {
                    try {
                        auto gyroFrame = gyroFrameRaw->as<ob::GyroFrame>();
                        auto val = gyroFrame->getValue();
                        auto ts = gyroFrame->getTimeStampUs();
                        auto temp = gyroFrame->getTemperature();
                        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::system_clock::now().time_since_epoch())
                                         .count();
                        std::ostringstream oss;
                        oss << nowMs << ",GYRO," << ts << "," << val.x << "," << val.y << "," << val.z << "," << temp
                            << "\n";
                        if (cap->imuTask)
                            cap->imuTask->enqueueLine(oss.str());
                    } catch (...) {
                    }
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    cap->sensorFiles->frameCounts[OB_FRAME_GYRO]++;
                }
            });
        }

        captures.push_back(std::move(cap));
    }

    if (!cfg.noShow) {
        if (!viewer.createWindow()) {
            std::cerr << "SDL viewer window creation failed, continuing without preview" << std::endl;
            NIO_LOG_WARN("SDL viewer window creation failed, continuing without preview");
        }
    }

    if (captures.empty()) {
        std::cerr << "No matching devices found!" << std::endl;
        NIO_LOG_FATAL("No matching devices found!");
        if (!cfg.cameraFilter.empty()) {
            std::cerr << "Available devices:" << std::endl;
            for (uint32_t i = 0; i < deviceList->getCount(); i++) {
                auto dev = deviceList->getDevice(i);
                std::cerr << " - " << dev->getDeviceInfo()->getName() << std::endl;
            }
        }
        return -1;
    }

    std::cout << "\n=== Recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << captures.size() << " device(s)" << std::endl;
    if (cfg.enableFusion) {
        std::cout << "D2C Fusion: alpha=" << cfg.alpha << ", depth range: " << cfg.depthMinM << "m - " << cfg.depthMaxM
                  << "m" << std::endl;
    } else {
        std::cout << "D2C Fusion: disabled" << std::endl;
    }
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;
    NIO_LOG_INFO_S("=== Recording started === devices=" << captures.size() << " outputDir=" << outputRootDir
                                                        << " fusion=" << (cfg.enableFusion ? "on" : "off"));
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

    // -----------------------------------------------------------------------
    // Main recording loop
    // -----------------------------------------------------------------------
    auto lastReportTime = ob_smpl::getNowTimesMs();
    uint32_t waitTime = 1000;

    while (g_running) {
        auto key = ob_smpl::waitForKeyPressed(waitTime);
        if (key == ESC_KEY || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        auto currentTime = ob_smpl::getNowTimesMs();
        if (currentTime >= lastReportTime + waitTime) {
            uint64_t reportDuration = currentTime - lastReportTime;
            lastReportTime = currentTime;

            for (auto& cap : captures) {
                std::map<OBFrameType, uint64_t> tempCounts;
                {
                    std::lock_guard<std::mutex> lock(cap->sensorFiles->countMtx);
                    if (!cap->sensorFiles->frameCounts.empty()) {
                        tempCounts = cap->sensorFiles->frameCounts;
                        for (auto& item : cap->sensorFiles->frameCounts) {
                            item.second = 0;
                        }
                    }
                }

                std::cout << "[" << cap->deviceName << "] ";
                if (tempCounts.empty() && !cap->fusionTask) {
                    std::cout << "Recording... waiting for frames";
                } else {
                    std::cout << "Recording... FPS: ";
                    std::string sep;
                    for (const auto& item : tempCounts) {
                        auto name = ob::TypeHelper::convertOBFrameTypeToString(item.first);
                        float rate = (reportDuration > 0) ? (item.second / (reportDuration / 1000.0f)) : 0.0f;
                        std::cout << std::fixed << std::setprecision(1) << sep << name << "=" << rate;
                        sep = ", ";
                        NIO_LOG_TRACE_S("[" << cap->deviceName << "] " << name << "=" << std::fixed
                                            << std::setprecision(1) << rate);
                    }
                    if (cap->fusionTask) {
                        uint64_t fusedCount = cap->fusionTask->frameCount.exchange(0);
                        float fusedRate = (reportDuration > 0) ? (fusedCount / (reportDuration / 1000.0f)) : 0.0f;
                        std::cout << sep << "fused=" << std::fixed << std::setprecision(1) << fusedRate;
                        NIO_LOG_TRACE_S("[" << cap->deviceName << "] fused=" << std::fixed << std::setprecision(1)
                                            << fusedRate);
                    }
                }
                std::cout << std::endl;
            }
            waitTime = 2000;
        }
    }

    std::cout << "\n=== Stopping recording ===" << std::endl;
    NIO_LOG_INFO("=== Stopping recording ===");

    for (auto& cap : captures) {
        if (cap->videoPipeline)
            cap->videoPipeline->stop();
        if (cap->hasIMU && cap->imuPipeline)
            cap->imuPipeline->stop();

        // Stop all stream tasks (joins threads, drains queues)
        if (cap->colorEncodeTask)
            cap->colorEncodeTask->stop();
        if (cap->depthEncodeTask)
            cap->depthEncodeTask->stop();
        if (cap->depthRawTask)
            cap->depthRawTask->stop();
        if (cap->irEncodeTask)
            cap->irEncodeTask->stop();
        if (cap->irLeftEncodeTask)
            cap->irLeftEncodeTask->stop();
        if (cap->irRightEncodeTask)
            cap->irRightEncodeTask->stop();
        if (cap->fusionTask)
            cap->fusionTask->stop();
        if (cap->imuTask)
            cap->imuTask->stop();

        // Close encoders and files
        auto& sf = cap->sensorFiles;
        if (sf->color && sf->color->encoder)
            sf->color->encoder->close();
        if (sf->depth && sf->depth->encoder)
            sf->depth->encoder->close();
        if (sf->ir && sf->ir->encoder)
            sf->ir->encoder->close();
        if (sf->irLeft && sf->irLeft->encoder)
            sf->irLeft->encoder->close();
        if (sf->irRight && sf->irRight->encoder)
            sf->irRight->encoder->close();

        if (sf->color && sf->color->file)
            sf->color->file->close();
        if (sf->depth && sf->depth->file)
            sf->depth->file->close();
        if (sf->ir && sf->ir->file)
            sf->ir->file->close();
        if (sf->irLeft && sf->irLeft->file)
            sf->irLeft->file->close();
        if (sf->irRight && sf->irRight->file)
            sf->irRight->file->close();
        if (sf->depthRawFile)
            sf->depthRawFile->close();
        if (sf->imuFile)
            sf->imuFile->close();

        cap->mjpgRes.reset();

        std::cout << "Stopped: " << cap->deviceName << std::endl;
        NIO_LOG_INFO_S("Stopped device: " << cap->deviceName);
    }

    viewer.close();

    std::cout << "All recordings saved to: " << outputRootDir << "/" << std::endl;
    NIO_LOG_INFO_S("All recordings saved to: " << outputRootDir << "/");
    NIO_LOG_SHUTDOWN();
    return 0;
} catch (ob::Error& e) {
    std::cerr << "OB Error: " << e.getFunction() << "\n  " << e.what() << "\n status: " << e.getStatus() << std::endl;
    NIO_LOG_FATAL_S("OB Error: " << e.getFunction() << " " << e.what() << " status=" << e.getStatus());
    NIO_LOG_SHUTDOWN();
    return -1;
} catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    NIO_LOG_FATAL_S("Exception: " << e.what());
    NIO_LOG_SHUTDOWN();
    return -1;
}
