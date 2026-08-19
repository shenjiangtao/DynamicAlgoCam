// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynamic_algo_cam.cpp — Multi-device capture application.
//
// Records color, depth, and IR streams to H.264 / raw files with IMU CSV
// logging. Performs D2C alignment + alpha-blend fusion as H.264.
// Optional: --engage-model / --engage-actuator enable perceive→locate→estimate→control loop.
//
// Usage:
//   ./dynamic_algo_cam                                          # all devices
//   ./dynamic_algo_cam -c "305" "336L"                          # filter by camera type
//   ./dynamic_algo_cam -s /HDD/dynalgo_capture                      # custom save directory
//   ./dynamic_algo_cam -c "305" -s /HDD/dynalgo_capture --alpha 0.6 # combined
//   ./dynamic_algo_cam --engage-model DUMMY --engage-actuator DUMMY # dry-run engagement

#include "dynalgo_capture_config.hpp"
#include "dynalgo_capture_session.hpp"
#include "dynalgo_common.hpp"
#include "dynalgo_device.hpp"
#include "dynalgo_driver_factory.hpp"
#include "dynalgo_log.hpp"
#include "dynalgo_model_factory.hpp"
#include "dynalgo_actuator_factory.hpp"
#include "dynalgo_sdl_viewer.hpp"
#include "../algo/dynalgo_engagement_loop.hpp"
#include "../algo/dynalgo_engagement_consumer.hpp"
#include "utils.hpp"
#include "event.hpp"
#ifdef ENABLE_EVENT_SIM
#include "event_simulator.hpp"
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
#include <optional>
#include <mutex>

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "unknown"
#endif

using namespace dynalgo;

// Helper: string -> DynalgoModelType
static DynalgoModelType modelTypeFromString(const std::string& s) {
    if (s == "NONE") return DynalgoModelType::NONE;
    if (s == "DUMMY") return DynalgoModelType::DUMMY;
    if (s == "YOLOV8_PY") return DynalgoModelType::YOLOV8_PY;
    if (s == "ONNXRUNTIME") return DynalgoModelType::ONNXRUNTIME;
    if (s == "TENSORRT") return DynalgoModelType::TENSORRT;
    return DynalgoModelType::NONE;
}

// Helper: string -> DynalgoActuatorType
static DynalgoActuatorType actuatorTypeFromString(const std::string& s) {
    if (s == "NONE") return DynalgoActuatorType::NONE;
    if (s == "DUMMY") return DynalgoActuatorType::DUMMY;
    if (s == "LASER_GENERIC") return DynalgoActuatorType::LASER_GENERIC;
    if (s == "GIMBAL_GENERIC") return DynalgoActuatorType::GIMBAL_GENERIC;
    return DynalgoActuatorType::NONE;
}

int main(int argc, char** argv) try {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    CaptureConfig cfg = parseArgs(argc, argv);

    // Initialize logger after determining the actual output directory (outputRootDir).
    // This ensures logs are stored alongside captured data.
    // (Logger will be initialized later, after outputRootDir is computed.)
    DYNALGO_LOG_SET_LEVEL(dynalgo::LogLevel::TRACE);
    // Note: DYNALGO_LOG_INIT will be called after outputRootDir is defined.
    std::cout << "Git commit: " << GIT_COMMIT_HASH << std::endl;
    // Log statements will be emitted after logger init.
    // DYNALGO_LOG_INFO_S will be called later when outputRootDir is ready.

    for (size_t i = 0; i < cfg.cameraFilter.size(); i++) {
        DYNALGO_LOG_DEBUG_S("Camera filter[" << i << "]=" << cfg.cameraFilter[i]);
    }

    // Engagement loop setup (Phase C) — created after sessions have depth intrinsics.
    // We'll store the config and instantiate per-session after setup().
    bool engageEnabled = !cfg.engageModel.empty() && !cfg.engageActuator.empty();
    DynalgoModelType engageModelType = modelTypeFromString(cfg.engageModel);
    DynalgoActuatorType engageActuatorType = actuatorTypeFromString(cfg.engageActuator);
    DynalgoModelConfig engageModelCfg;
    engageModelCfg.modelPath = cfg.engageModelPath.empty() ? "dummy" : cfg.engageModelPath;
    engageModelCfg.deviceHint = "cpu";
    engageModelCfg.confThreshold = 0.25f;
    engageModelCfg.iouThreshold = 0.45f;

    // Discover all devices via the driver factory
    auto discovered = discoverDevices();
    uint32_t totalDevices = static_cast<uint32_t>(discovered.size());

    if (totalDevices < 1) {
        std::cerr << "No device found!" << std::endl;
        DYNALGO_LOG_FATAL("No device found!");
        return -1;
    }

    std::string sessionTimestamp = getTimestampMs();
    std::string outputBaseDir = cfg.saveDir.empty() ? "capture_output" : cfg.saveDir;
    std::string outputRootDir = outputBaseDir + "/" + sessionTimestamp;
    mkdirp(outputRootDir);
    // Initialize logger now that we have the final output directory.
    DYNALGO_LOG_INIT("dynamic_algo_cam", outputRootDir);
    DYNALGO_LOG_INFO_S("Logger initialized to directory: " << outputRootDir);
    // Log build and run information now that logger is ready.
    DYNALGO_LOG_INFO_S("Git commit: " << GIT_COMMIT_HASH);
    DYNALGO_LOG_INFO_S("Process started, camera_filter_count=" << cfg.cameraFilter.size()
                       << " saveDir=" << outputRootDir << " alpha=" << cfg.alpha
                       << " depthMin=" << cfg.depthMinM << " depthMax=" << cfg.depthMaxM
                       << " fusion=" << (cfg.enableFusion ? "on" : "off"));
    if (engageEnabled) {
        DYNALGO_LOG_INFO_S("[engage] enabled: model=" << cfg.engageModel
                           << " actuator=" << cfg.engageActuator
                           << " modelPath=" << engageModelCfg.modelPath);
    }
    DYNALGO_LOG_INFO_S("Session timestamp=" << sessionTimestamp << " outputDir=" << outputRootDir);

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
            DYNALGO_LOG_ERROR_S("Setup failed for device: " << safeName);
            continue;
        }

        // Engagement loop per session (after setup() so we have depthIntrinsic/depthScale)
        DynalgoEngagementLoop* engageLoop = nullptr;
        DynalgoModelBackend* engageModelBackend = nullptr;
        DynalgoActuator* engageActuator = nullptr;
        if (engageEnabled) {
            auto modelBackendPtr = createModelBackend(engageModelType);
            auto actuatorPtr = createActuator(engageActuatorType);
            if (!modelBackendPtr) {
                DYNALGO_LOG_WARN_S("[engage] createModelBackend(" << cfg.engageModel << ") returned nullptr — engagement disabled for this session");
            } else if (!actuatorPtr) {
                DYNALGO_LOG_WARN_S("[engage] createActuator(" << cfg.engageActuator << ") returned nullptr — engagement disabled for this session");
            } else {
                // Load config into backends
                if (!modelBackendPtr->load(engageModelCfg)) {
                    DYNALGO_LOG_WARN_S("[engage] modelBackend load() failed");
                }
                DynalgoActuatorConfig actuatorCfg{};
                if (!actuatorPtr->load(actuatorCfg)) {
                    DYNALGO_LOG_WARN_S("[engage] actuator load() failed");
                }
                if (!actuatorPtr->open()) {
                    DYNALGO_LOG_WARN_S("[engage] actuator open() failed");
                }

                engageModelBackend = modelBackendPtr.release();
                engageActuator = actuatorPtr.release();

                const auto& depthIntr = session->depthIntrinsic();
                float depthScale = session->depthScale();
                engageLoop = new DynalgoEngagementLoop(
                    DynalgoEngagementLoop::Config{}, engageModelBackend, engageActuator, depthIntr, depthScale);
                auto engageConsumer = std::make_unique<DynalgoEngagementFrameConsumer>(engageLoop);
                session->addFrameConsumer(std::move(engageConsumer));
                DYNALGO_LOG_INFO_S("[engage] session " << safeName << " engagement loop armed");
            }
        }

        session->startImuPipeline();
        session->startVideoPipeline(viewer, cfg.noShow);
        if (!session->hasVideoPipeline()) {
            DYNALGO_LOG_WARN_S("Video pipeline not started for " << safeName << ", skipping");
            sessions.push_back(session);
            continue;
        }

        // Store engagement loop alongside session (shared lifetime)
        if (engageLoop)
            session->setEngagementLoop(engageLoop, engageModelBackend, engageActuator);

        sessions.push_back(std::move(session));
    }

    if (!cfg.noShow) {
        if (!viewer.createWindow()) {
            std::cerr << "SDL viewer window creation failed, continuing without preview" << std::endl;
        }
    }

    if (sessions.empty()) {
        std::cerr << "No matching devices found!" << std::endl;
        DYNALGO_LOG_FATAL("No matching devices found!");
        return -1;
    }

    std::cout << "\n=== Recording started ===" << std::endl;
    std::cout << "Output directory: " << outputRootDir << "/" << std::endl;
    std::cout << "Recording " << sessions.size() << " device(s)" << std::endl;
    if (cfg.enableFusion) {
        std::cout << "D2C Fusion: alpha=" << cfg.alpha << ", depth range: " << cfg.depthMinM << "m - " << cfg.depthMaxM << "m" << std::endl;
    } else {
        std::cout << "D2C Fusion: disabled" << std::endl;
    }
    std::cout << "Press Ctrl+C or 'q' to stop recording.\n" << std::endl;

    // ---------------------------------------------------------------------
    // Event‑window recorder – drives dynamic stop time and filename prefix
    // ---------------------------------------------------------------------
    struct EventWindow {
        uint64_t startTimeMs = 0;           // T0 of first event
        uint64_t endTimeMs = 0;             // Current deadline (T0+2s, extended)
        const uint64_t marginMs = 2000;    // ±2 s around each event
        const uint64_t maxWindowMs = 60000; // Upper bound 60 s
        std::mutex mtx;
        Event activeEvent; bool hasActive = false;
        void addEvent(const Event& ev) {
            std::lock_guard<std::mutex> lk(mtx);
            if (startTimeMs == 0) {
                startTimeMs = ev.tsMs;
                endTimeMs = startTimeMs + marginMs;
                activeEvent = ev; hasActive = true;
            } else {
                if (ev.tsMs > endTimeMs) {
                    // New event after current window – start fresh window
                    startTimeMs = ev.tsMs;
                    endTimeMs = startTimeMs + marginMs;
                    activeEvent = ev; hasActive = true;
                }
                // else: inside window – keep existing activeEvent
            }
            if (endTimeMs - startTimeMs > maxWindowMs) {
                endTimeMs = startTimeMs + maxWindowMs;
            }
        }
        bool shouldContinue(uint64_t nowMs) const {
            if (startTimeMs == 0) return true; // no events yet
            return nowMs <= endTimeMs;
        }
        std::string getBaseFileName() const {
            if (!hasActive) return "recording";
            return activeEvent.name + "_" + std::to_string(activeEvent.tsMs);
        }
    } eventWindow;

    // Lambda that registers an event and propagates the prefix to all sessions
    auto eventSink = [&](const Event& ev) {
        eventWindow.addEvent(ev);
        std::string prefix = eventWindow.getBaseFileName();
        for (auto& s : sessions) s->setFilePrefix(prefix);
    };

#ifdef ENABLE_EVENT_SIM
    // Simple synthetic event schedule for testing – can be removed/changed by user
    // E0 at t=0, E1 at +1200ms, E2 at +3200ms (relative to program start)
    EventSimulator simulator(eventSink);
    simulator.start({{0, "E0"}, {1200, "E1"}, {2000, "E2"}});
#endif

    auto lastReportTime = dynalgo::getNowTimesMs();
    uint32_t waitTime = 1000;

    while (g_running) {
        auto key = dynalgo::waitForKeyPressed(waitTime);
        if (key == ESC_KEY || key == 'q' || key == 'Q') {
            g_running = false;
            break;
        }

        // Insert real event detection here – call eventSink(Event(...))
        // Example placeholder (replace with actual trigger logic):
        // if (someCondition) eventSink(Event("E3", dynalgo::getNowTimesMs()));

        uint64_t now = dynalgo::getNowTimesMs();
        if (!eventWindow.shouldContinue(now)) {
            std::cout << "\n=== Event window expired, stopping recording ===" << std::endl;
            g_running = false;
            break;
        }

        if (now >= lastReportTime + waitTime) {
            uint64_t reportDuration = now - lastReportTime;
            lastReportTime = now;
            for (auto& session : sessions)
                session->reportFps(reportDuration);
            waitTime = 2000;
        }
    }

#ifdef ENABLE_EVENT_SIM
    simulator.stop();
#endif


    std::cout << "\n=== Stopping recording ===" << std::endl;

    for (auto& session : sessions)
        session->stop();

    viewer.close();

    std::cout << "All recordings saved to: " << outputRootDir << "/" << std::endl;
    DYNALGO_LOG_SHUTDOWN();
    return 0;
} catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    DYNALGO_LOG_FATAL_S("Exception: " << e.what());
    DYNALGO_LOG_SHUTDOWN();
    return -1;
}
