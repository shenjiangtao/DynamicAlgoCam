// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_device.hpp — Abstract device and pipeline interfaces for multi-SDK support.
//
// NioDevice: abstract interface for a camera device (sensor enumeration,
// clock sync, property access).  Each SDK (Orbbec, RealSense, etc.) provides
// a concrete subclass.
//
// NioPipeline: abstract interface for a capture pipeline (configure streams,
// start/stop, enable sync, check HW D2C support).  Each SDK provides a
// concrete subclass.
//
// NioStreamConfig: SDK-neutral stream configuration (which streams to enable,
// resolution, format).  Replaces ob::Config.

#pragma once

#include "nio_frame.hpp"
#include "nio_types.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nio {

class NioPipeline;

// Forward declarations
class NioFrameSet;
struct NioD2CAlign;

// Device info (SDK-agnostic).
struct NioDeviceInfo
{
    std::string name;
    std::string serialNumber;
    uint16_t vid = 0;
    uint16_t pid = 0;
    std::string connectionType;
};

// Stream configuration for one sensor.
struct NioStreamConfig
{
    NioFrameType frameType = NioFrameType::COLOR;
    int width = 0;
    int height = 0;
    int fps = 30;
    NioFormat format = NioFormat::UNKNOWN;
    bool enabled = true;
};

// D2C alignment mode.
enum class NioAlignMode {
    NONE,
    HW,
    SW
};

// Callback type: called when a new FrameSet arrives from the pipeline.
using NioVideoCallback = std::function<void(std::shared_ptr<NioFrameSet>)>;

// Callback type: called when IMU samples arrive from the pipeline.
using NioImuCallback = std::function<void(const std::vector<NioImuSample>&)>;

// NioDevice: abstract camera device.
class NioDevice
{
public:
    virtual ~NioDevice() = default;

    virtual NioDeviceInfo getDeviceInfo() const = 0;
    virtual void timerSyncWithHost() = 0;
    virtual bool isGlobalTimestampSupported() const = 0;
    virtual void enableGlobalTimestamp(bool enable) = 0;
    virtual NioSensorInfo getSensorInfo() const = 0;

    // Get integer property (e.g. depth precision level).
    virtual int32_t getIntProperty(int propertyId) = 0;

    // Whether the device has any IR sensor (IR / IR-Left / IR-Right).
    // RS-AC1 has no IR sensors → returns false.
    virtual bool hasIRSensor() const = 0;

    // Enumerate sensors, select profiles, enable streams on the pipeline,
    // apply device quirks, and return the resolved sensor info.
    // This replaces CaptureSession's enumerateSensors/applyDeviceQuirks/detectDepthScale.
    virtual NioSensorInfo setupPipeline(NioPipeline& pipeline) = 0;
};

// NioPipeline: abstract capture pipeline.
class NioPipeline
{
public:
    virtual ~NioPipeline() = default;

    // Configure which streams to enable.
    virtual void enableStream(const NioStreamConfig& cfg) = 0;
    virtual void disableStream(NioFrameType type) = 0;

    // Set frame aggregation mode.
    virtual void setAggregateAllTypeFrameRequire(bool require) = 0;

    // Set D2C alignment mode.
    virtual void setAlignMode(NioAlignMode mode) = 0;

    // Check if HW D2C alignment is supported for the given profiles.
    virtual bool checkHWD2CSupport(int colorW, int colorH, NioFormat colorFmt, int depthW, int depthH,
                                   NioFormat depthFmt, int depthFps) = 0;

    // Enable frame sync.
    virtual void enableFrameSync() = 0;

    // Start pipeline with video callback (receives NioFrameSet).
    virtual bool start(NioVideoCallback callback) = 0;

    // Start IMU pipeline with callback.
    virtual bool startImu(NioImuCallback callback) = 0;

    // Stop pipeline.
    virtual void stop() = 0;

    // Stop IMU pipeline.
    virtual void stopImu() = 0;

    // Get the underlying device.
    virtual std::shared_ptr<NioDevice> getDevice() const = 0;

    // Whether this pipeline's depth sensor outputs 3D point cloud
    // (RS-AC1) instead of a 2D depth map (Orbbec).
    virtual bool isPointCloudDepth() const {
        return false;
    }

    // Current D2C alignment mode (HW / SW / NONE).
    // RS-AC1 always returns HW; OB returns the mode set via setAlignMode().
    virtual NioAlignMode getAlignMode() const = 0;

    // Get D2C alignment filter (may be null if HW D2C or not applicable).
    virtual std::shared_ptr<NioD2CAlign> getD2CAlignFilter() const {
        return nullptr;
    }
};

// NioD2CAlign: abstract D2C alignment filter.
// Wraps SDK-specific alignment (e.g. ob::Align for Orbbec).
// process() takes a type-erased native FrameSet, performs alignment,
// and returns aligned pixel data + metadata via NioAlignedFrameSet.
struct NioAlignedFrame
{
    const uint8_t* colorData = nullptr;
    uint32_t colorSize = 0;
    uint64_t colorTs = 0;
    const uint8_t* depthData = nullptr;
    uint32_t depthSize = 0;
    uint64_t depthTs = 0;
    float depthScale = 1.0f;
};

struct NioD2CAlign
{
    virtual ~NioD2CAlign() = default;
    virtual bool process(std::shared_ptr<void> nativeFrameSet, NioAlignedFrame& out) = 0;
};

// NioContext: abstract SDK context for device discovery.
class NioContext
{
public:
    virtual ~NioContext() = default;

    virtual uint32_t getDeviceCount() = 0;
    virtual std::shared_ptr<NioDevice> getDevice(uint32_t index) = 0;
};

} // namespace nio
