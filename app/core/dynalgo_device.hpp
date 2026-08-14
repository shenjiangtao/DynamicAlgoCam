// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_device.hpp — Abstract device and pipeline interfaces for multi-SDK support.
//
// DynalgoDevice: abstract interface for a camera device (sensor enumeration,
// clock sync, property access).  Each vendor SDK provides a concrete
// subclass.
//
// DynalgoPipeline: abstract interface for a capture pipeline (configure streams,
// start/stop, enable sync, check HW D2C support).  Each SDK provides a
// concrete subclass.
//
// DynalgoStreamConfig: SDK-neutral stream configuration (which streams to enable,
// resolution, format).  Replaces vendor-specific Config types.

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace dynalgo {

class DynalgoPipeline;

// Forward declarations
class DynalgoFrameSet;

// Device info (SDK-agnostic).
struct DynalgoDeviceInfo
{
    std::string name;
    std::string serialNumber;
    uint16_t vid = 0;
    uint16_t pid = 0;
    std::string connectionType;
};

// Stream configuration for one sensor.
struct DynalgoStreamConfig
{
    DynalgoFrameType frameType = DynalgoFrameType::COLOR;
    int width = 0;
    int height = 0;
    int fps = 30;
    DynalgoFormat format = DynalgoFormat::UNKNOWN;
    bool enabled = true;
};

namespace types {

// D2C alignment mode.
enum class DynalgoAlignMode {
    NONE,
    HW,
    SW
};

} // namespace types

using types::DynalgoAlignMode;

// Callback type: called when a new FrameSet arrives from the pipeline.
using DynalgoVideoCallback = std::function<void(std::shared_ptr<DynalgoFrameSet>)>;

// Callback type: called when IMU samples arrive from the pipeline.
using DynalgoImuCallback = std::function<void(const std::vector<DynalgoImuSample>&)>;

// DynalgoDevice: abstract camera device.
class DynalgoDevice
{
public:
    virtual ~DynalgoDevice() = default;

    virtual DynalgoDeviceInfo getDeviceInfo() const = 0;
    virtual void timerSyncWithHost() = 0;
    virtual bool isGlobalTimestampSupported() const = 0;
    virtual void enableGlobalTimestamp(bool enable) = 0;
    virtual DynalgoSensorInfo getSensorInfo() const = 0;

    // Get integer property (e.g. depth precision level).
    virtual int32_t getIntProperty(int propertyId) = 0;

    // Whether the device has any IR sensor (IR / IR-Left / IR-Right).
    virtual bool hasIRSensor() const = 0;

    // Enumerate sensors, select profiles, enable streams on the pipeline,
    // apply device quirks, and return the resolved sensor info.
    // This replaces CaptureSession's enumerateSensors/applyDeviceQuirks/detectDepthScale.
    virtual DynalgoSensorInfo setupPipeline(DynalgoPipeline& pipeline) = 0;
};

// DynalgoPipeline: abstract capture pipeline.
class DynalgoPipeline
{
public:
    virtual ~DynalgoPipeline() = default;

    // Configure which streams to enable.
    virtual void enableStream(const DynalgoStreamConfig& cfg) = 0;
    virtual void disableStream(DynalgoFrameType type) = 0;

    // Set frame aggregation mode.
    virtual void setAggregateAllTypeFrameRequire(bool require) = 0;

    // Set D2C alignment mode.
    virtual void setAlignMode(DynalgoAlignMode mode) = 0;

    // Check if HW D2C alignment is supported for the given profiles.
    virtual bool checkHWD2CSupport(int colorW, int colorH, DynalgoFormat colorFmt, int depthW, int depthH,
                                   DynalgoFormat depthFmt, int depthFps) = 0;

    // Enable frame sync.
    virtual void enableFrameSync() = 0;

    // Start pipeline with video callback (receives DynalgoFrameSet).
    virtual bool start(DynalgoVideoCallback callback) = 0;

    // Start IMU pipeline with callback.
    virtual bool startImu(DynalgoImuCallback callback) = 0;

    // Stop pipeline.
    virtual void stop() = 0;

    // Stop IMU pipeline.
    virtual void stopImu() = 0;

    // Get the underlying device.
    virtual std::shared_ptr<DynalgoDevice> getDevice() const = 0;

    // Whether point cloud output is enabled for this pipeline.
    virtual bool isPcdEnabled() const {
        return false;
    }

    // Enable depth-to-point-cloud conversion.
    // No-op for pipelines that always output points (e.g. LiDAR).
    virtual void setPointCloudEnabled(bool enable) {
        (void)enable;
    }

    // Current D2C alignment mode (HW / SW / NONE).
    virtual DynalgoAlignMode getAlignMode() const = 0;
};

// DynalgoContext: abstract SDK context for device discovery.
class DynalgoContext
{
public:
    virtual ~DynalgoContext() = default;

    virtual uint32_t getDeviceCount() = 0;
    virtual std::shared_ptr<DynalgoDevice> getDevice(uint32_t index) = 0;
};

} // namespace dynalgo
