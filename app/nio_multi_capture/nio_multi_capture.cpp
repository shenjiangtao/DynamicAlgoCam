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
#include "nio_driver_factory.hpp"
#include "nio_log.hpp"
#include "nio_sdl_viewer.hpp"
#include "utils.hpp"

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

    // Discover all devices via the driver factory
    auto discovered = discoverDevices();
    uint32_t totalDevices = static_cast<uint32_t>(discovered.size());

    if (totalDevices < 1) {
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
    {
        std::ifstream usbfsFile("/sys/module/usbcore/parameters/usbfs_memory_mb");
        if (usbfsFile.is_open()) {
            int usbfsMb = 0;
            usbfsFile >> usbfsMb;
            int minUsbfs = (totalDevices > 1) ? 128 : 32;
            if (usbfsMb < minUsbfs && totalDevices >= 1) {
                std::cerr << "WARNING: usbfs_memory_mb=" << usbfsMb << "MB is too low for " << totalDevices
                          << " device(s). Recommend >= " << minUsbfs << "MB." << std::endl;
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

    for (auto& dd : discovered) {
        auto devInfo = dd.device->getDeviceInfo();

        if (!deviceMatches(devInfo.name, cfg.cameraFilter)) {
            std::cout << "Skipping device: " << devInfo.name << std::endl;
            continue;
        }

        std::cout << "Found device: " << devInfo.name << " (SN: " << devInfo.serialNumber << ", PID: 0x" << std::hex
                  << std::setw(4) << std::setfill('0') << devInfo.pid << std::dec << ", " << devInfo.connectionType
                  << ")" << std::endl;

        auto safeName = devInfo.name;
        std::replace(safeName.begin(), safeName.end(), ' ', '_');
        safeName = safeName + "_" + devInfo.serialNumber;

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);

        auto session = std::make_shared<CaptureSession>(dd.device, dd.pipeline, safeName, deviceOutputDir, cfg);
        if (!session->setup()) {
            NIO_LOG_ERROR_S("Setup failed for device: " << safeName);
            continue;
        }

        session->startImuPipeline();
        session->startVideoPipeline(viewer, cfg.noShow);
        if (!session->hasVideoPipeline()) {
            NIO_LOG_WARN_S("Video pipeline not started for " << safeName << ", skipping");
            sessions.push_back(session);
            continue;
        }

        sessions.push_back(std::move(session));
    }

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

    auto lastReportTime = nio::getNowTimesMs();
    uint32_t waitTime = 1000;

    while (g_running) {
        auto key = nio::waitForKeyPressed(waitTime);
        if (key == ESC_KEY || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        auto currentTime = nio::getNowTimesMs();
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
} catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    NIO_LOG_FATAL_S("Exception: " << e.what());
    NIO_LOG_SHUTDOWN();
    return -1;
}
