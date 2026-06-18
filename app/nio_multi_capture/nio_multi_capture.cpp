// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_multi_capture.cpp — Multi-device capture application.
//
// Records color, depth, and IR streams to H.264 / raw files with IMU CSV
// logging. Performs D2C alignment + alpha-blend fusion as H.264.
//
// Threading model:
//   - SDK video callback: lightweight — enqueues to per-stream worker threads.
//   - SDK IMU callback: formats CSV lines, enqueues to ImuStreamTask.
//   - Each stream has its own named worker thread (encoding/raw/fusion/IMU).
//   - SDLViewer runs its own decode + render threads.
//
// Usage:
//   ./nio_multi_capture                                          # all devices
//   ./nio_multi_capture -c "305" "336L"                          # filter by camera type
//   ./nio_multi_capture -s /HDD/nio_capture                      # custom save directory
//   ./nio_multi_capture -c "305" -s /HDD/nio_capture --alpha 0.6 # combined

#include "nio_capture_config.hpp"
#include "nio_capture_session.hpp"
#include "nio_common.hpp"
#include "nio_log.hpp"
#include "nio_sdl_viewer.hpp"
#include "utils.hpp"
#include <libobsensor/ObSensor.hpp>

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

    SDLViewer viewer;
    if (!cfg.noShow) {
        if (!viewer.init()) {
            std::cerr << "SDL viewer init failed, continuing without preview" << std::endl;
            NIO_LOG_WARN_S("SDL viewer init failed, continuing without preview");
        }
    }

    std::vector<std::shared_ptr<CaptureSession>> sessions;

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

        std::string deviceOutputDir = outputRootDir + "/" + safeName;
        mkdirp(deviceOutputDir);
        NIO_LOG_DEBUG_S("Created output dir: " << deviceOutputDir);

        auto session = std::make_shared<CaptureSession>(device, safeName, deviceOutputDir, cfg);
        if (!session->setup()) {
            NIO_LOG_ERROR_S("Setup failed for device: " << safeName);
            continue;
        }

        session->startVideoPipeline(viewer, cfg.noShow);
        if (!session->hasVideoPipeline()) {
            NIO_LOG_WARN_S("Video pipeline not started for " << safeName << ", skipping IMU");
            sessions.push_back(session);
            continue;
        }

        session->startImuPipeline();
        sessions.push_back(std::move(session));
    }

    if (!cfg.noShow) {
        if (!viewer.createWindow()) {
            std::cerr << "SDL viewer window creation failed, continuing without preview" << std::endl;
            NIO_LOG_WARN("SDL viewer window creation failed, continuing without preview");
        }
    }

    if (sessions.empty()) {
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
    std::cout << "Recording " << sessions.size() << " device(s)" << std::endl;
    if (cfg.enableFusion) {
        std::cout << "D2C Fusion: alpha=" << cfg.alpha << ", depth range: " << cfg.depthMinM << "m - " << cfg.depthMaxM
                  << "m" << std::endl;
    } else {
        std::cout << "D2C Fusion: disabled" << std::endl;
    }
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;
    NIO_LOG_INFO_S("=== Recording started === devices=" << sessions.size() << " outputDir=" << outputRootDir
                                                        << " fusion=" << (cfg.enableFusion ? "on" : "off"));
    NIO_LOG_INFO_S("Log file: " << NIO_LOG_PATH());

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
    NIO_LOG_INFO("=== Stopping recording ===");

    for (auto& session : sessions)
        session->stop();

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
