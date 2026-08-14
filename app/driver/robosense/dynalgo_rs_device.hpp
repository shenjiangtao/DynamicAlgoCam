// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_rs_device.hpp — RoboSense RS-AC1 implementation of DynalgoDevice/DynalgoPipeline/DynalgoContext.
//
// RsDevice: fixed sensor info (no IR, depth=96×288@10 Y16, color=1920×1080@30 NV12).
// RsPipeline: wraps LidarDriver with dual get/put callbacks + FrameSet synthesis.
// RsContext: USB device discovery via libusb (VID=0x3840, PID=0x1010).

#pragma once

#include "dynalgo_device.hpp"
#include "dynalgo_rs_adapter.hpp"
#include "dynalgo_rs_frame_adapter.hpp"
#include "dynalgo_rs_spec.hpp"

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/msg/image_data_msg.hpp>
#include <rs_driver/msg/imu_data_msg.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/utility/sync_queue.hpp>

#include <libusb.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace dynalgo {

using RsPointCloudMsg = ::PointCloudT<::PointXYZIRT>;
using RsLidarDriver = robosense::lidar::LidarDriver<RsPointCloudMsg>;
template <typename T>
using RsSyncQueue = robosense::lidar::SyncQueue<T>;

// RsDevice: wraps RS-AC1 as DynalgoDevice.
class RsDevice : public DynalgoDevice
{
public:
    explicit RsDevice(uint32_t deviceIndex, const std::string& deviceUuid = "");

    DynalgoDeviceInfo getDeviceInfo() const override;
    void timerSyncWithHost() override;
    bool isGlobalTimestampSupported() const override;
    void enableGlobalTimestamp(bool enable) override;
    DynalgoSensorInfo getSensorInfo() const override;
    int32_t getIntProperty(int propertyId) override;
    bool hasIRSensor() const override {
        return false;
    }
    DynalgoSensorInfo setupPipeline(DynalgoPipeline& pipeline) override;

    uint32_t deviceIndex() const {
        return deviceIndex_;
    }
    const std::string& deviceUuid() const {
        return deviceUuid_;
    }

private:
    uint32_t deviceIndex_;
    std::string deviceUuid_;
    mutable bool deviceInfoQueried_ = false;
    mutable DynalgoDeviceInfo cachedDevInfo_;
};

// RsPipeline: wraps LidarDriver as DynalgoPipeline.
// Manages dual get/put callback pairs and synthesizes DynalgoFrameSet
// from asynchronous PointCloudMsg + ImageData arrivals.
class RsPipeline : public DynalgoPipeline
{
public:
    explicit RsPipeline(std::shared_ptr<RsDevice> device);

    void enableStream(const DynalgoStreamConfig& cfg) override;
    void disableStream(DynalgoFrameType type) override;
    void setAggregateAllTypeFrameRequire(bool require) override;
    void setAlignMode(DynalgoAlignMode mode) override;
    bool checkHWD2CSupport(int colorW, int colorH, DynalgoFormat colorFmt, int depthW, int depthH, DynalgoFormat depthFmt,
                           int depthFps) override;
    void enableFrameSync() override;
    bool start(DynalgoVideoCallback callback) override;
    bool startImu(DynalgoImuCallback callback) override;
    void stop() override;
    void stopImu() override;
    std::shared_ptr<DynalgoDevice> getDevice() const override;
    bool isPcdEnabled() const override {
        return true;
    }
    DynalgoAlignMode getAlignMode() const override {
        return DynalgoAlignMode::HW;
    }

private:
    // Processing threads for rs_driver stuffed queues.
    void processCloud();
    void processImageData();
    void processImu();

    // FrameSet synthesis: emit when color + depth both ready.
    void tryEmitFrameSet();

    // Format IMU data as CSV lines and push via imuCallback_.
    void emitImuData(const std::shared_ptr<robosense::lidar::ImuData>& imu);

    std::shared_ptr<RsDevice> rsDevice_;
    RsLidarDriver driver_;

    // Stream configuration (set via enableStream before start).
    DynalgoFormat imageFormat_ = rs::AC1::COLOR.format;
    int imageWidth_ = rs::AC1::COLOR.resolution.width;
    int imageHeight_ = rs::AC1::COLOR.resolution.height;
    int imageFps_ = rs::AC1::COLOR.fps;
    int imuFps_ = rs::AC1::IMU_FPS;
    bool enableImage_ = true;

    // Dual get/put callback queues.  The "stuffed" queues receive data
    // from rs_driver internal threads; the "free" queues are for recycling.
    RsSyncQueue<std::shared_ptr<RsPointCloudMsg>> freeCloudQueue_;
    RsSyncQueue<std::shared_ptr<RsPointCloudMsg>> stuffedCloudQueue_;
    RsSyncQueue<std::shared_ptr<robosense::lidar::ImuData>> freeImuQueue_;
    RsSyncQueue<std::shared_ptr<robosense::lidar::ImuData>> stuffedImuQueue_;
    RsSyncQueue<std::shared_ptr<robosense::lidar::ImageData>> freeImageQueue_;
    RsSyncQueue<std::shared_ptr<robosense::lidar::ImageData>> stuffedImageQueue_;

    // FrameSet synthesis state.
    std::mutex syncMtx_;
    std::shared_ptr<DynalgoFrame> colorFrame_;
    std::shared_ptr<DynalgoFrame> depthFrame_;
    std::shared_ptr<DynalgoFrame> pointFrame_;
    bool colorReady_ = false;
    bool depthReady_ = false;

    // Callbacks.
    DynalgoVideoCallback videoCallback_;
    DynalgoImuCallback imuCallback_;

    // Processing threads.
    std::thread cloudThread_;
    std::thread imageThread_;
    std::thread imuThread_;
    std::atomic<bool> running_{ false };
    bool started_ = false;
    bool imuStarted_ = false;
};

// RsContext: discovers RS-AC1 devices via libusb (VID=0x3840, PID=0x1010).
class RsContext : public DynalgoContext
{
public:
    RsContext();
    ~RsContext();

    uint32_t getDeviceCount() override;
    std::shared_ptr<DynalgoDevice> getDevice(uint32_t index) override;

private:
    // Scan USB bus for RS-AC1 devices; cache VID/PID matches.
    void scanDevices();

    libusb_context* usbCtx_ = nullptr;
    // Cached list of device UUIDs (busnum-devnum strings for multi-device).
    std::vector<std::string> deviceUuids_;
    bool scanned_ = false;
};

} // namespace dynalgo
