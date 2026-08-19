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

// [类说明 / Class Description]
// 中文: RsDevice 封装 RS-AC1 实现 DynalgoDevice 接口（固定传感器信息，无 IR，深度=96×288@10 Y16，颜色=1920×1080@30 NV12）
// English: RsDevice wraps RS-AC1 implementing DynalgoDevice interface (fixed sensor info, no IR, depth=96x288@10 Y16, color=1920x1080@30 NV12)
class RsDevice : public DynalgoDevice
{
public:
    explicit RsDevice(uint32_t deviceIndex, const std::string& deviceUuid = "");

    // [函数说明 / Function Description]
    // 中文: 获取设备信息（名称、VID、PID、连接类型、序列号）
    // English: Get device info (name, VID, PID, connection type, serial number)
    DynalgoDeviceInfo getDeviceInfo() const override;
    // [函数说明 / Function Description]
    // 中文: 与主机同步时间戳（空实现）
    // English: Sync timestamp with host (no-op)
    void timerSyncWithHost() override;
    // [函数说明 / Function Description]
    // 中文: 检查是否支持全局时间戳
    // English: Check if global timestamp is supported
    bool isGlobalTimestampSupported() const override;
    // [函数说明 / Function Description]
    // 中文: 启用/禁用全局时间戳（空实现）
    // English: Enable/disable global timestamp (no-op)
    void enableGlobalTimestamp(bool enable) override;
    // [函数说明 / Function Description]
    // 中文: 获取传感器信息（颜色、深度、IMU 等固定规格）
    // English: Get sensor info (fixed specs for color, depth, IMU, etc.)
    DynalgoSensorInfo getSensorInfo() const override;
    // [函数说明 / Function Description]
    // 中文: 获取整数属性（返回 0）
    // English: Get integer property (returns 0)
    int32_t getIntProperty(int propertyId) override;
    // [函数说明 / Function Description]
    // 中文: 检查是否有 IR 传感器（RS-AC1 无 IR）
    // English: Check if IR sensor exists (RS-AC1 has no IR)
    bool hasIRSensor() const override {
        return false;
    }
    // [函数说明 / Function Description]
    // 中文: 配置管道（返回固定传感器信息和深度比例）
    // English: Configure pipeline (returns fixed sensor info and depth scale)
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

// [类说明 / Class Description]
// 中文: RsPipeline 封装 LidarDriver 实现 DynalgoPipeline 接口，管理双 get/put 回调队列，从异步 PointCloudMsg + ImageData 合成 DynalgoFrameSet
// English: RsPipeline wraps LidarDriver implementing DynalgoPipeline, manages dual get/put callback queues, synthesizes DynalgoFrameSet from async PointCloudMsg + ImageData
class RsPipeline : public DynalgoPipeline
{
public:
    explicit RsPipeline(std::shared_ptr<RsDevice> device);

    // [函数说明 / Function Description]
    // 中文: 启用流（颜色流配置格式/分辨率/帧率，IMU 配置帧率）
    // English: Enable stream (color stream config format/resolution/fps, IMU config fps)
    void enableStream(const DynalgoStreamConfig& cfg) override;
    // [函数说明 / Function Description]
    // 中文: 禁用流（颜色流）
    // English: Disable stream (color stream)
    void disableStream(DynalgoFrameType type) override;
    // [函数说明 / Function Description]
    // 中文: 设置聚合模式（空实现）
    // English: Set aggregate mode (no-op)
    void setAggregateAllTypeFrameRequire(bool require) override;
    // [函数说明 / Function Description]
    // 中文: 设置对齐模式（空实现，RS-AC1 固定硬件对齐）
    // English: Set align mode (no-op, RS-AC1 fixed HW alignment)
    void setAlignMode(DynalgoAlignMode mode) override;
    // [函数说明 / Function Description]
    // 中文: 检查硬件 D2C 支持（总是返回 true）
    // English: Check HW D2C support (always returns true)
    bool checkHWD2CSupport(int colorW, int colorH, DynalgoFormat colorFmt, int depthW, int depthH, DynalgoFormat depthFmt,
                           int depthFps) override;
    // [函数说明 / Function Description]
    // 中文: 启用帧同步（空实现）
    // English: Enable frame sync (no-op)
    void enableFrameSync() override;
    // [函数说明 / Function Description]
    // 中文: 启动视频流（初始化驱动参数、注册回调、启动处理线程）
    // English: Start video stream (init driver params, register callbacks, start processing threads)
    bool start(DynalgoVideoCallback callback) override;
    // [函数说明 / Function Description]
    // 中文: 启动 IMU 流（延迟绑定 IMU 线程）
    // English: Start IMU stream (late-bind IMU thread)
    bool startImu(DynalgoImuCallback callback) override;
    // [函数说明 / Function Description]
    // 中文: 停止视频流（停止驱动、等待线程结束）
    // English: Stop video stream (stop driver, join threads)
    void stop() override;
    // [函数说明 / Function Description]
    // 中文: 停止 IMU 流（RS-AC1 无法独立停止 IMU）
    // English: Stop IMU stream (RS-AC1 cannot stop IMU independently)
    void stopImu() override;
    // [函数说明 / Function Description]
    // 中文: 获取关联的设备
    // English: Get associated device
    std::shared_ptr<DynalgoDevice> getDevice() const override;
    bool isPcdEnabled() const override {
        return true;
    }
    // [函数说明 / Function Description]
    // 中文: 获取对齐模式（固定硬件对齐）
    // English: Get align mode (fixed HW alignment)
    DynalgoAlignMode getAlignMode() const override {
        return DynalgoAlignMode::HW;
    }

private:
    // Processing threads for rs_driver stuffed queues.
    // [函数说明 / Function Description]
    // 中文: 处理点云队列（生成深度帧和点云帧，尝试发射 FrameSet）
    // English: Process cloud queue (generate depth frame and point frame, try emit FrameSet)
    void processCloud();
    // [函数说明 / Function Description]
    // 中文: 处理图像数据队列（生成颜色帧，尝试发射 FrameSet）
    // English: Process image data queue (generate color frame, try emit FrameSet)
    void processImageData();
    // [函数说明 / Function Description]
    // 中文: 处理 IMU 队列（格式化为 CSV 并推送回调）
    // English: Process IMU queue (format as CSV and push via callback)
    void processImu();

    // FrameSet synthesis: emit when color + depth both ready.
    // [函数说明 / Function Description]
    // 中文: 尝试发射 FrameSet（颜色+深度都就绪时）
    // English: Try emit FrameSet (when both color and depth ready)
    void tryEmitFrameSet();

    // Format IMU data as CSV lines and push via imuCallback_.
    // [函数说明 / Function Description]
    // 中文: 格式化 IMU 数据并推送回调
    // English: Format IMU data and push via callback
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

// [类说明 / Class Description]
// 中文: RsContext 通过 libusb 发现 RS-AC1 设备（VID=0x3840, PID=0x1010）
// English: RsContext discovers RS-AC1 devices via libusb (VID=0x3840, PID=0x1010)
class RsContext : public DynalgoContext
{
public:
    RsContext();
    ~RsContext();

    // [函数说明 / Function Description]
    // 中文: 获取设备数量
    // English: Get device count
    uint32_t getDeviceCount() override;
    // [函数说明 / Function Description]
    // 中文: 根据索引获取设备
    // English: Get device by index
    std::shared_ptr<DynalgoDevice> getDevice(uint32_t index) override;

private:
    // Scan USB bus for RS-AC1 devices; cache VID/PID matches.
    // [函数说明 / Function Description]
    // 中文: 扫描 USB 总线查找 RS-AC1 设备，缓存 VID/PID 匹配项
    // English: Scan USB bus for RS-AC1 devices; cache VID/PID matches
    void scanDevices();

    libusb_context* usbCtx_ = nullptr;
    // Cached list of device UUIDs (busnum-devnum strings for multi-device).
    std::vector<std::string> deviceUuids_;
    bool scanned_ = false;
};

} // namespace dynalgo
