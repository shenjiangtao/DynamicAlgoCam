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

// [类说明 / Class Description]
// 中文: ObDevice 封装 ob::Device 实现 DynalgoDevice 接口
// English: ObDevice wraps ob::Device implementing DynalgoDevice interface
class ObDevice : public DynalgoDevice
{
public:
    explicit ObDevice(std::shared_ptr<ob::Device> device);

    // [函数说明 / Function Description]
    // 中文: 获取设备信息（名称、序列号、VID、PID、连接类型）
    // English: Get device info (name, serial number, VID, PID, connection type)
    DynalgoDeviceInfo getDeviceInfo() const override;
    // [函数说明 / Function Description]
    // 中文: 与主机同步时间戳
    // English: Sync timestamp with host
    void timerSyncWithHost() override;
    // [函数说明 / Function Description]
    // 中文: 检查是否支持全局时间戳
    // English: Check if global timestamp is supported
    bool isGlobalTimestampSupported() const override;
    // [函数说明 / Function Description]
    // 中文: 启用/禁用全局时间戳
    // English: Enable/disable global timestamp
    void enableGlobalTimestamp(bool enable) override;
    // [函数说明 / Function Description]
    // 中文: 获取传感器信息（颜色、深度、IR、IMU 等）
    // English: Get sensor info (color, depth, IR, IMU, etc.)
    DynalgoSensorInfo getSensorInfo() const override;
    // [函数说明 / Function Description]
    // 中文: 获取整数属性
    // English: Get integer property
    int32_t getIntProperty(int propertyId) override;
    // [函数说明 / Function Description]
    // 中文: 检查是否有 IR 传感器
    // English: Check if IR sensor exists
    bool hasIRSensor() const override;
    // [函数说明 / Function Description]
    // 中文: 配置管道流（颜色、深度、IR 等），返回实际启用的传感器信息
    // English: Configure pipeline streams (color, depth, IR, etc.), return enabled sensor info
    DynalgoSensorInfo setupPipeline(DynalgoPipeline& pipeline) override;

    std::shared_ptr<ob::Device> obDevice() const {
        return obDevice_;
    }

private:
    std::shared_ptr<ob::Device> obDevice_;
    mutable DynalgoSensorInfo cachedSensorInfo_;
    mutable bool sensorInfoCached_ = false;
};

// [类说明 / Class Description]
// 中文: ObPipeline 封装 ob::Pipeline 实现 DynalgoPipeline 接口
// English: ObPipeline wraps ob::Pipeline implementing DynalgoPipeline interface
class ObPipeline : public DynalgoPipeline
{
public:
    explicit ObPipeline(std::shared_ptr<ob::Device> device);

    // [函数说明 / Function Description]
    // 中文: 启用流（需要具体的 ob::VideoStreamProfile，由 CaptureSession 实现）
    // English: Enable stream (requires ob::VideoStreamProfile, implemented in CaptureSession)
    void enableStream(const DynalgoStreamConfig& cfg) override;
    // [函数说明 / Function Description]
    // 中文: 禁用流
    // English: Disable stream
    void disableStream(DynalgoFrameType type) override;
    // [函数说明 / Function Description]
    // 中文: 设置聚合模式（任何情况都输出 FrameSet，保持融合 FPS 接近颜色传感器速率）
    // English: Set aggregate mode (emit FrameSet as soon as any stream arrives, keeps fused FPS near color rate)
    void setAggregateAllTypeFrameRequire(bool require) override;
    // [函数说明 / Function Description]
    // 中文: 设置对齐模式（硬件 D2C 或软件对齐）
    // English: Set align mode (HW D2C or SW alignment)
    void setAlignMode(DynalgoAlignMode mode) override;
    // [函数说明 / Function Description]
    // 中文: 启用/禁用点云
    // English: Enable/disable point cloud
    void setPointCloudEnabled(bool enable) override;
    // [函数说明 / Function Description]
    // 中文: 检查硬件 D2C 支持
    // English: Check HW D2C support
    bool checkHWD2CSupport(int colorW, int colorH, DynalgoFormat colorFmt, int depthW, int depthH, DynalgoFormat depthFmt,
                           int depthFps) override;
    // [函数说明 / Function Description]
    // 中文: 启用帧同步
    // English: Enable frame sync
    void enableFrameSync() override;
    // [函数说明 / Function Description]
    // 中文: 启动视频流（回调中处理点云、对齐、转换为 DynalgoFrameSet）
    // English: Start video stream (callback processes point cloud, alignment, converts to DynalgoFrameSet)
    bool start(DynalgoVideoCallback callback) override;
    // [函数说明 / Function Description]
    // 中文: 启动 IMU 流
    // English: Start IMU stream
    bool startImu(DynalgoImuCallback callback) override;
    // [函数说明 / Function Description]
    // 中文: 停止视频流
    // English: Stop video stream
    void stop() override;
    // [函数说明 / Function Description]
    // 中文: 停止 IMU 流
    // English: Stop IMU stream
    void stopImu() override;
    // [函数说明 / Function Description]
    // 中文: 获取关联的设备
    // English: Get associated device
    std::shared_ptr<DynalgoDevice> getDevice() const override;
    bool isPcdEnabled() const override {
        return pcdEnabled_;
    }
    // [函数说明 / Function Description]
    // 中文: 获取当前对齐模式
    // English: Get current align mode
    DynalgoAlignMode getAlignMode() const override;

    bool isHwD2CMode() const {
        return hwD2CMode_;
    }

    std::shared_ptr<ob::Align> getAlignFilter() const {
        return alignFilter_;
    }

    // Expose raw ob pipeline for legacy code paths.
    // [函数说明 / Function Description]
    // 中文: 暴露原始 ob::Pipeline 供旧代码路径使用
    // English: Expose raw ob::Pipeline for legacy code paths
    std::shared_ptr<ob::Pipeline> obPipeline() const {
        return obPipeline_;
    }

    // Expose ob::Config for sensor enumeration in ObDevice::setupPipeline.
    // [函数说明 / Function Description]
    // 中文: 暴露 ob::Config 供 ObDevice::setupPipeline 枚举传感器使用
    // English: Expose ob::Config for sensor enumeration in ObDevice::setupPipeline
    std::shared_ptr<ob::Config> obConfig() const {
        return obConfig_;
    }

    // Set selected profiles (called from ObDevice::setupPipeline).
    // [函数说明 / Function Description]
    // 中文: 设置选中的颜色流配置（由 ObDevice::setupPipeline 调用）
    // English: Set selected color profile (called from ObDevice::setupPipeline)
    void setColorProfile(std::shared_ptr<ob::VideoStreamProfile> p) {
        colorProfile_ = std::move(p);
    }
    // [函数说明 / Function Description]
    // 中文: 设置选中的深度流配置（由 ObDevice::setupPipeline 调用）
    // English: Set selected depth profile (called from ObDevice::setupPipeline)
    void setDepthProfile(std::shared_ptr<ob::VideoStreamProfile> p) {
        depthProfile_ = std::move(p);
    }
    // [函数说明 / Function Description]
    // 中文: 获取选中的颜色流配置
    // English: Get selected color profile
    std::shared_ptr<ob::VideoStreamProfile> colorProfile() const {
        return colorProfile_;
    }
    // [函数说明 / Function Description]
    // 中文: 获取选中的深度流配置
    // English: Get selected depth profile
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

// [类说明 / Class Description]
// 中文: ObContext 封装 ob::Context 实现 DynalgoContext 接口
// English: ObContext wraps ob::Context implementing DynalgoContext interface
class ObContext : public DynalgoContext
{
public:
    explicit ObContext(const std::string& configPath = "");

    // [函数说明 / Function Description]
    // 中文: 获取设备数量
    // English: Get device count
    uint32_t getDeviceCount() override;
    // [函数说明 / Function Description]
    // 中文: 根据索引获取设备
    // English: Get device by index
    std::shared_ptr<DynalgoDevice> getDevice(uint32_t index) override;

    ob::Context& obCtx() {
        return ctx_;
    }

    // [函数说明 / Function Description]
    // 中文: 初始化 Orbbec SDK（设置扩展目录）
    // English: Initialize Orbbec SDK (set extensions directory)
    static void initSDK(const std::string& extensionsDir);

private:
    ob::Context ctx_;
    static bool sdkInitialized_;
};

} // namespace dynalgo
