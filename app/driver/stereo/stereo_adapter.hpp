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

#ifdef DYNALOGO_HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

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

    // Calibration file (OpenCV YAML/XML)
    std::string calibrationFile;   // Path to stereo calibration YAML/XML
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
        MYNT_EYE,        // MYNT_EYE (SLAMTEC)
        DEPTHAI,         // Luxonis DepthAI/OAK
        ORBBEC_STEREO,   // Orbbec stereo (if available)
        CUSTOM           // Custom implementation
    };

    static std::unique_ptr<IStereoCamera> create(Vendor vendor);
    static std::vector<std::string> discoverDevices(Vendor vendor);
};

} // namespace dynalgo

#ifdef DYNALOGO_HAVE_OPENCV

namespace dynalgo {

// StereoRectifier — handles stereo rectification using OpenCV calibration data
// Loads calibration from OpenCV YAML/XML and provides rectification maps
// for undistort + rectify of stereo image pairs.
class StereoRectifier {
public:
    StereoRectifier() = default;
    ~StereoRectifier() = default;

    // Load calibration from OpenCV YAML/XML file
    // Expected format (OpenCV stereoRectify output):
    //   M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q
    bool loadCalibration(const std::string& calibrationFile);

    // Check if calibration is loaded
    bool isValid() const { return valid_; }

    // Rectify a pair of stereo images (BGR24 input -> BGR24 output)
    // Returns false if calibration not loaded or rectification failed
    bool rectify(const DynalgoFrame& leftRaw, const DynalgoFrame& rightRaw,
                 DynalgoFrame& leftRect, DynalgoFrame& rightRect) const;

    // Get rectification maps (for manual remap if needed)
    // Returns false if calibration not loaded
    bool getMaps(
        std::vector<float>& leftMapX, std::vector<float>& leftMapY,
        std::vector<float>& rightMapX, std::vector<float>& rightMapY) const;

    // Get rectified intrinsics (P1/P2 from calibration)
    DynalgoIntrinsic getLeftIntrinsic() const { return leftIntrinsic_; }
    DynalgoIntrinsic getRightIntrinsic() const { return rightIntrinsic_; }
    DynalgoExtrinsic getLeftToRight() const { return leftToRight_; }

    // Compute depth from rectified stereo pair using SGBM
    // Returns disparity map (Y16) and depth map (float32)
    bool computeDepth(const DynalgoFrame& leftRect, const DynalgoFrame& rightRect,
                      DynalgoFrame& disparity, DynalgoFrame& depth) const;

    // Get baseline in meters
    float getBaseline() const { return baseline_; }

private:
    bool valid_ = false;
    cv::Mat leftMap1_, leftMap2_, rightMap1_, rightMap2_;
    cv::Mat leftCameraMatrix_, rightCameraMatrix_;
    cv::Mat leftDistCoeffs_, rightDistCoeffs_;
    cv::Mat R_, T_, R1_, R2_, P1_, P2_, Q_;
    cv::Size imageSize_;

    DynalgoIntrinsic leftIntrinsic_, rightIntrinsic_;
    DynalgoExtrinsic leftToRight_;
    float baseline_ = 0.0f;
};

} // namespace dynalgo

#endif // DYNALOGO_HAVE_OPENCV
