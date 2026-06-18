// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_device.hpp — Orbbec SDK implementation of NioDevice/NioPipeline/NioContext.

#pragma once

#include "nio_device.hpp"
#include "nio_ob_adapter.hpp"
#include "nio_ob_frame_adapter.hpp"

#include <libobsensor/ObSensor.hpp>

namespace nio {

// ObDevice: wraps ob::Device as NioDevice.
class ObDevice : public NioDevice {
public:
    explicit ObDevice(std::shared_ptr<ob::Device> device);

    NioDeviceInfo getDeviceInfo() const override;
    void timerSyncWithHost() override;
    bool isGlobalTimestampSupported() const override;
    void enableGlobalTimestamp(bool enable) override;
    NioSensorInfo getSensorInfo() const override;
    int32_t getIntProperty(int propertyId) override;

    std::shared_ptr<ob::Device> obDevice() const { return obDevice_; }

private:
    std::shared_ptr<ob::Device> obDevice_;
    mutable NioSensorInfo cachedSensorInfo_;
    mutable bool sensorInfoCached_ = false;
};

// ObPipeline: wraps ob::Pipeline as NioPipeline.
class ObPipeline : public NioPipeline {
public:
    explicit ObPipeline(std::shared_ptr<ob::Device> device);

    void enableStream(const NioStreamConfig &cfg) override;
    void disableStream(NioFrameType type) override;
    void setAggregateAllTypeFrameRequire(bool require) override;
    void setAlignMode(NioAlignMode mode) override;
    bool checkHWD2CSupport(int colorW, int colorH, NioFormat colorFmt,
                           int depthW, int depthH, NioFormat depthFmt, int depthFps) override;
    void enableFrameSync() override;
    bool start(NioVideoCallback callback) override;
    bool startImu(NioImuCallback callback) override;
    void stop() override;
    void stopImu() override;
    std::shared_ptr<NioDevice> getDevice() const override;

    // Expose for SW D2C alignment (transitional).
    std::shared_ptr<ob::Align> getAlignFilter() const { return alignFilter_; }
    bool isHwD2CMode() const { return hwD2CMode_; }

    // Expose raw ob pipeline for legacy code paths.
    std::shared_ptr<ob::Pipeline> obPipeline() const { return obPipeline_; }

private:
    std::shared_ptr<ob::Pipeline> obPipeline_;
    std::shared_ptr<ob::Config> obConfig_;
    std::shared_ptr<ob::Pipeline> imuPipeline_;
    std::shared_ptr<ob::Config> imuConfig_;
    std::shared_ptr<ob::Device> obDevice_;
    std::shared_ptr<ob::Align> alignFilter_;
    bool hwD2CMode_ = false;
    bool imuStarted_ = false;
    NioVideoCallback videoCallback_;
    NioImuCallback imuCallback_;
};

// ObContext: wraps ob::Context as NioContext.
class ObContext : public NioContext {
public:
    ObContext();

    uint32_t getDeviceCount() override;
    std::shared_ptr<NioDevice> getDevice(uint32_t index) override;

    // Expose for legacy code that needs ob::Context.
    ob::Context &obCtx() { return ctx_; }

private:
    ob::Context ctx_;
};

} // namespace nio
