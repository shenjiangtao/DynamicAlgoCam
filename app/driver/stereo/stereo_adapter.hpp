// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// stereo_adapter.hpp — Hardware-sync stereo camera abstraction for Dynalgo.

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace dynalgo {

// Stereo camera configuration
struct StereoConfig {
    std::string devicePath;        // e.g. "/dev/video0" or serial number
    int width = 1280;              // per-eye width
    int height = 720;              // per-eye height
    int fps = 30;
    bool hardwareSync = true;      // require hardware GPIO sync
    bool computeDepth = true;      // enable stereo matching for depth
    int depthMinDisparity = 0;
    int depthMaxDisparity = 128;
    int depthBlockSize = 5;
    float depthScale = 0.001f;     // disparity to meters
};

// Stereo frame set (left + right + optional depth/disparity)
struct StereoFrameSet {
    uint64_t timestampUs = 0;      // hardware timestamp
    uint32_t frameId = 0;

    // Rectified images
    DynalgoFrame leftRect;         // BGR24 rectified left
    DynalgoFrame rightRect;        // BGR24 rectified right

    // Raw images (optional)
    DynalgoFrame leftRaw;
    DynalgoFrame rightRaw;

    // Depth/Disparity (if enabled)
    DynalgoFrame disparity;        // 16-bit or 32-bit disparity
    DynalgoFrame depth;            // 32-bit float depth in meters

    // Calibration
    DynalgoIntrinsic leftIntrinsic;
    DynalgoIntrinsic rightIntrinsic;
    DynalgoExtrinsic leftToRight;  // rotation + translation from left to right
};

// Stereo camera info
struct StereoCameraInfo {
    std::string serialNumber;
    std::string modelName;
    std::string firmwareVersion;
    DynalgoIntrinsic leftIntrinsic;
    DynalgoIntrinsic rightIntrinsic;
    DynalgoExtrinsic leftToRight;
    float baselineMeters = 0.12f;  // baseline in meters
};

// Abstract stereo camera interface
class IStereoCamera {
public:
    virtual ~IStereoCamera() = default;

    // Initialize and open device
    virtual bool open(const StereoConfig& cfg) = 0;

    // Close device
    virtual bool close() = 0;

    // Check if device is open
    virtual bool isOpen() const = 0;

    // Get camera info (calibration, etc.)
    virtual const StereoCameraInfo& getInfo() const = 0;

    // Grab next synchronized frame set (blocking)
    // Returns false on timeout/error
    virtual bool grab(StereoFrameSet& outFrame, int timeoutMs = 1000) = 0;

    // Start/stop streaming (for async callback mode)
    virtual bool startStreaming(std::function<void(const StereoFrameSet&)> callback) = 0;
    virtual bool stopStreaming() = 0;

    // Trigger single capture (for hardware trigger mode)
    virtual bool triggerCapture() = 0;

    // Set exposure/gain (if supported)
    virtual bool setExposure(float exposureMs) = 0;
    virtual bool setGain(float gain) = 0;

    // Get current exposure/gain
    virtual float getExposure() const = 0;
    virtual float getGain() const = 0;

    // Rectification maps (for undistort + rectify)
    virtual bool getRectificationMaps(
        std::vector<float>& leftMapX, std::vector<float>& leftMapY,
        std::vector<float>& rightMapX, std::vector<float>& rightMapY) const = 0;
};

// Factory for creating stereo camera instances
class StereoCameraFactory {
public:
    enum class Vendor {
        GENERIC_UVC,     // Generic UVC stereo (e.g. Dual UVC cameras)
        ZED,             // Stereolabs ZED
        MYNT_EYE,        // MYNT EYE (SLAMTEC)
        DEPTHAI,         // Luxonis DepthAI/OAK
        ORBBEC_STEREO,   // Orbbec stereo (if available)
        CUSTOM           // Custom implementation
    };

    static std::unique_ptr<IStereoCamera> create(Vendor vendor);
    static std::vector<std::string> discoverDevices(Vendor vendor);
};

} // namespace dynalgo