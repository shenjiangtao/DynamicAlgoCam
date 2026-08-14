// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_device.hpp — Orbbec SDK implementation of DynalgoDevice/DynalgoPipeline/DynalgoContext.

#pragma once

#include "dynalgo_device.hpp"
#include "dynalgo_ob_adapter.hpp"
#include "dynalgo_ob_frame_adapter.hpp"
#include "dynalgo_ob_spec.hpp"

#include <libobsensor/ObSensor.hpp>

namespace dynalgo {

// ObDevice: wraps ob::Device as DynalgoDevice.
class ObDevice : public DynalgoDevice
{
public:
    explicit ObDevice(std::shared_ptr<ob::Device> device);

    DynalgoDeviceInfo getDeviceInfo() const override;
    void timerSyncWithHost() override;
    bool isGlobalTimestampSupported() const override;
    void enableGlobalTimestamp(bool enable) override;
    DynalgoSensorInfo getSensorInfo() const override;
    int32_t getIntProperty(int propertyId) override;
    bool hasIRSensor() const override;
    DynalgoSensorInfo setupPipeline(DynalgoPipeline& pipeline) override;

    std::shared_ptr<ob::Device> obDevice() const {
        return obDevice_;
    }

private:
    std::shared_ptr<ob::Device> obDevice_;
    mutable DynalgoSensorInfo cachedSensorInfo_;
    mutable bool sensorInfoCached_ = false;
};

// ObPipeline: wraps ob::Pipeline as DynalgoPipeline.
class ObPipeline : public DynalgoPipeline
{
public:
    explicit ObPipeline(std::shared_ptr<ob::Device> device);

    void enableStream(const DynalgoStreamConfig& cfg) override;
    void disableStream(DynalgoFrameType type) override;
    void setAggregateAllTypeFrameRequire(bool require) override;
    void setAlignMode(DynalgoAlignMode mode) override;
    void setPointCloudEnabled(bool enable) override;
    bool checkHWD2CSupport(int colorW, int colorH, DynalgoFormat colorFmt, int depthW, int depthH, DynalgoFormat depthFmt,
                           int depthFps) override;
    void enableFrameSync() override;
    bool start(DynalgoVideoCallback callback) override;
    bool startImu(DynalgoImuCallback callback) override;
    void stop() override;
    void stopImu() override;
    std::shared_ptr<DynalgoDevice> getDevice() const override;
    bool isPcdEnabled() const override {
        return pcdEnabled_;
    }
    DynalgoAlignMode getAlignMode() const override;

    bool isHwD2CMode() const {
        return hwD2CMode_;
    }

    std::shared_ptr<ob::Align> getAlignFilter() const {
        return alignFilter_;
    }

    // Expose raw ob pipeline for legacy code paths.
    std::shared_ptr<ob::Pipeline> obPipeline() const {
        return obPipeline_;
    }

    // Expose ob::Config for sensor enumeration in ObDevice::setupPipeline.
    std::shared_ptr<ob::Config> obConfig() const {
        return obConfig_;
    }

    // Set selected profiles (called from ObDevice::setupPipeline).
    void setColorProfile(std::shared_ptr<ob::VideoStreamProfile> p) {
        colorProfile_ = std::move(p);
    }
    void setDepthProfile(std::shared_ptr<ob::VideoStreamProfile> p) {
        depthProfile_ = std::move(p);
    }
    std::shared_ptr<ob::VideoStreamProfile> colorProfile() const {
        return colorProfile_;
    }
    std::shared_ptr<ob::VideoStreamProfile> depthProfile() const {
        return depthProfile_;
    }

private:
    std::shared_ptr<ob::Pipeline> obPipeline_;
    std::shared_ptr<ob::Config> obConfig_;
    std::shared_ptr<ob::Pipeline> imuPipeline_;
    std::shared_ptr<ob::Config> imuConfig_;
    std::shared_ptr<ob::Device> obDevice_;
    std::shared_ptr<ob::Align> alignFilter_;
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter_;
    bool pcdEnabled_ = false;
    bool hwD2CMode_ = false;
    bool imuStarted_ = false;
    DynalgoVideoCallback videoCallback_;
    DynalgoImuCallback imuCallback_;
    std::shared_ptr<ob::VideoStreamProfile> colorProfile_;
    std::shared_ptr<ob::VideoStreamProfile> depthProfile_;
};

// ObContext: wraps ob::Context as DynalgoContext.
class ObContext : public DynalgoContext
{
public:
    explicit ObContext(const std::string& configPath = "");

    uint32_t getDeviceCount() override;
    std::shared_ptr<DynalgoDevice> getDevice(uint32_t index) override;

    ob::Context& obCtx() {
        return ctx_;
    }

    static void initSDK(const std::string& extensionsDir);

private:
    ob::Context ctx_;
    static bool sdkInitialized_;
};

} // namespace dynalgo
