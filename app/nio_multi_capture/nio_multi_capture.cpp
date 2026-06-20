// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_multi_capture.cpp — Multi-device capture application.
//
// Records color, depth, and IR streams to H.264 / raw files with IMU CSV
// logging. Performs D2C alignment + alpha-blend fusion as H.264.
//
// Usage:
//   ./nio_multi_capture                                          # all devices
//   ./nio_multi_capture -c "305" "336L"                          # filter by camera type
//   ./nio_multi_capture -s /HDD/nio_capture                      # custom save directory
//   ./nio_multi_capture -c "305" -s /HDD/nio_capture --alpha 0.6 # combined

#include "nio_capture_config.hpp"
#include "nio_capture_session.hpp"
#include "nio_common.hpp"
#include "nio_device.hpp"
#include "nio_log.hpp"
#include "nio_sdl_viewer.hpp"
#include "nio_ob_device.hpp"
#include "utils.hpp"

#ifdef ENABLE_RS_AC1
#include "nio_rs_device.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "unknown"
#endif

using namespace nio;

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

    // Discover OB devices
    ObContext obContext;
    auto obCount = obContext.getDeviceCount();

#ifdef ENABLE_RS_AC1
    // Discover RS-AC1 devices
    RsContext rsContext;
    auto rsCount = rsContext.getDeviceCount();
#else
    uint32_t rsCount = 0;
#endif

    if (obCount + rsCount < 1) {
        std::cerr << "No device found!" << std::endl;
        NIO_LOG_FATAL("No device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputBaseDir = cfg.saveDir.empty() ? "capture_output" : cfg.saveDir;
    std::string outputRootDir = outputBaseDir + "/" + sessionTimestamp;
    mkdirp(outputRootDir);
    NIO_LOG_INFO_S("Session timestamp=" << sessionTimestamp << " outputDir=" << outputRootDir);

    // Check USB memory for multi-device
    uint32_t totalDevices = obCount + rsCount;
    {
        std::ifstream usbfsFile("/sys/module/usbcore/parameters/usbfs_memory_mb");
        if (usbfsFile.is_open()) {
            int usbfsMb = 0;
            usbfsFile >> usbfsMb;
            if (usbfsMb < 128 && totalDevices > 1) {
                std::cerr << "WARNING: usbfs_memory_mb=" << usbfsMb << "MB is too low for " << totalDevices
                          << " devices. Recommend >= 128MB." << std::endl;
                std::cerr << "Fix: echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb" << std::endl;
            }
        }
    }

    SDLViewer viewer;
    if (!cfg.noShow) {
        if (!viewer.init()) {
            std::cerr << "SDL viewer init failed, continuing without preview" << std::endl;
        }
    }

    std::vector<std::shared_ptr<CaptureSession>> sessions;

    // Helper: create session from NioDevice + NioPipeline
    auto addSession = [&](std::shared_ptr<NioDevice> device, std::shared_ptr<NioPipeline> pipeline,
                         const std::string& safeName, const std::string& deviceOutputDir) {
        auto session = std::make_shared<CaptureSession>(device, pipeline, safeName, deviceOutputDir, cfg);
        if (!session->setup()) {
            NIO_LOG_ERROR_S("Setup failed for device: " << safeName);
            return;
        }

        session->startVideoPipeline(viewer, cfg.noShow);
        if (!session->hasVideoPipeline()) {
            NIO_LOG_WARN_S("Video pipeline not started for " << safeName << ", skipping IMU");
            sessions.push_back(session);
            return;
        }

        session->startImuPipeline();
        sessions.push_back(std::move(session));
    };

    // Enumerate OB devices
    for (uint32_t i = 0; i < obCount; i++) {
        auto nioDev = obContext.getDevice(i);
        auto devInfo = nioDev->getDeviceInfo();

        if (!deviceMatches(devInfo.name, cfg.cameraFilter)) {
            std::cout << "Skipping device: " << devInfo.name << std::endl;
            continue;
        }

        std::cout << "Found OB device: " << devInfo.name << " (SN: " << devInfo.serialNumber
                  << ", PID: 0x" << std::hex << std::setw(4) << std::setfill('0') << devInfo.pid << std::dec
                  << ", " << devInfo.connectionType << ")" << std::endl;

        auto safeName = devInfo.name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');
        safeName = safeName + "_" + devInfo.serialNumber;

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);

        auto obDev = std::dynamic_pointer_cast<ObDevice>(nioDev);
        auto pipeline = std::make_shared<ObPipeline>(obDev->obDevice());
        addSession(nioDev, pipeline, safeName, deviceOutputDir);
    }

#ifdef ENABLE_RS_AC1
    // Enumerate RS-AC1 devices
    for (uint32_t i = 0; i < rsCount; i++) {
        auto nioDev = rsContext.getDevice(i);
        auto devInfo = nioDev->getDeviceInfo();

        if (!deviceMatches(devInfo.name, cfg.cameraFilter)) {
            std::cout << "Skipping RS-AC1 device: " << devInfo.name << std::endl;
            continue;
        }

        std::cout << "Found RS-AC1 device: " << devInfo.name << std::endl;

        auto safeName = devInfo.name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');
        safeName = safeName + "_" + devInfo.serialNumber;

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);

        auto rsDev = std::dynamic_pointer_cast<RsDevice>(nioDev);
        auto pipeline = std::make_shared<RsPipeline>(rsDev);
        addSession(nioDev, pipeline, safeName, deviceOutputDir);
    }
#endif

    if (!cfg.noShow) {
        if (!viewer.createWindow()) {
            std::cerr << "SDL viewer window creation failed, continuing without preview" << std::endl;
        }
    }

    if (sessions.empty()) {
        std::cerr << "No matching devices found!" << std::endl;
        NIO_LOG_FATAL("No matching devices found!");
        return -1;
    }

    std::cout << "\n=== Recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << sessions.size() << " device(s)" << std::endl;
    if (cfg.enableFusion) {
        std::cout << "D2C Fusion: alpha=" << cfg.alpha << ", depth range: " << cfg.depthMinM << "m - " << cfg.depthMaxM
                  << "m" << std::endl;
    } else {
        std::cout << "D2C Fusion: disabled" << std::endl;
    }
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;

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

            for (auto& session : sessions)
                session->reportFps(reportDuration);

            waitTime = 2000;
        }
    }

    std::cout << "\n=== Stopping recording ===" << std::endl;

    for (auto& session : sessions)
        session->stop();

    viewer.close();

    std::cout << "All recordings saved to: " << outputRootDir << "/" << std::endl;
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
