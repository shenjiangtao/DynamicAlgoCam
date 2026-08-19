// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_device.hpp — Abstract device and pipeline interfaces for multi-SDK support.
//
// [文件说明 / File Description]
// 中文：抽象设备和管道接口，支持多SDK的相机设备和捕获管道
// English: Abstract device and pipeline interfaces for multi-SDK camera device and capture pipeline support
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

// [结构体说明 / Struct Description]
// 中文：设备信息，SDK中立
// English: Device info (SDK-agnostic).
struct DynalgoDeviceInfo
{
    std::string name;
    std::string serialNumber;
    uint16_t vid = 0;
    uint16_t pid = 0;
    std::string connectionType;
};

// [结构体说明 / Struct Description]
// 中文：单传感器流配置，SDK中立
// English: Stream configuration for one sensor, SDK-neutral.
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

// [枚举说明 / Enum Description]
// 中文：D2C对齐模式
// English: D2C alignment mode.
enum class DynalgoAlignMode {
    NONE,
    HW,
    SW
};

} // namespace types

using types::DynalgoAlignMode;

// [类型别名 / Type Alias]
// 中文：视频回调类型，当新帧集到达管道时调用
// English: Callback type: called when a new FrameSet arrives from the pipeline.
using DynalgoVideoCallback = std::function<void(std::shared_ptr<DynalgoFrameSet>)>;

// [类型别名 / Type Alias]
// 中文：IMU回调类型，当IMU样本到达管道时调用
// English: Callback type: called when IMU samples arrive from the pipeline.
using DynalgoImuCallback = std::function<void(const std::vector<DynalgoImuSample>&)>;

// [类说明 / Class Description]
// 中文：抽象相机设备接口，提供传感器枚举、时钟同步和属性访问
// English: DynalgoDevice: abstract camera device interface, provides sensor enumeration, clock sync, and property access
class DynalgoDevice
{
public:
    virtual ~DynalgoDevice() = default;

    // [方法说明 / Method Description]
    // 中文：获取设备信息
    // English: Get device info
    virtual DynalgoDeviceInfo getDeviceInfo() const = 0;

    // [方法说明 / Method Description]
    // 中文：定时器与主机同步
    // English: Timer sync with host
    virtual void timerSyncWithHost() = 0;

    // [方法说明 / Method Description]
    // 中文：检查是否支持全局时间戳
    // English: Check if global timestamp is supported
    virtual bool isGlobalTimestampSupported() const = 0;

    // [方法说明 / Method Description]
    // 中文：启用/禁用全局时间戳
    // English: Enable/disable global timestamp
    virtual void enableGlobalTimestamp(bool enable) = 0;

    // [方法说明 / Method Description]
    // 中文：获取传感器信息
    // English: Get sensor info
    virtual DynalgoSensorInfo getSensorInfo() const = 0;

    // [方法说明 / Method Description]
    // 中文：获取整数属性（如深度精度级别）
    // English: Get integer property (e.g. depth precision level)
    virtual int32_t getIntProperty(int propertyId) = 0;

    // [方法说明 / Method Description]
    // 中文：检查设备是否有IR传感器
    // English: Check if device has any IR sensor
    virtual bool hasIRSensor() const = 0;

    // [方法说明 / Method Description]
    // 中文：枚举传感器、选择配置文件、启用流、应用设备特性并返回解析后的传感器信息
    // English: Enumerate sensors, select profiles, enable streams, apply device quirks and return resolved sensor info
    virtual DynalgoSensorInfo setupPipeline(DynalgoPipeline& pipeline) = 0;
};

// [类说明 / Class Description]
// 中文：抽象捕获管道接口，提供流配置、启停、同步和D2C对齐支持
// English: DynalgoPipeline: abstract capture pipeline interface, provides stream configuration, start/stop, sync, and D2C alignment support
class DynalgoPipeline
{
public:
    virtual ~DynalgoPipeline() = default;

    // [方法说明 / Method Description]
    // 中文：启用/禁用流
    // English: Enable/disable stream
    virtual void enableStream(const DynalgoStreamConfig& cfg) = 0;
    virtual void disableStream(DynalgoFrameType type) = 0;

    // [方法说明 / Method Description]
    // 中文：设置帧聚合模式
    // English: Set frame aggregation mode
    virtual void setAggregateAllTypeFrameRequire(bool require) = 0;

    // [方法说明 / Method Description]
    // 中文：设置D2C对齐模式
    // English: Set D2C alignment mode
    virtual void setAlignMode(DynalgoAlignMode mode) = 0;

    // [方法说明 / Method Description]
    // 中文：检查给定配置是否支持硬件D2C对齐
    // English: Check if HW D2C alignment is supported for the given profiles
    virtual bool checkHWD2CSupport(int colorW, int colorH, DynalgoFormat colorFmt, int depthW, int depthH,
                                   DynalgoFormat depthFmt, int depthFps) = 0;

    // [方法说明 / Method Description]
    // 中文：启用帧同步
    // English: Enable frame sync
    virtual void enableFrameSync() = 0;

    // [方法说明 / Method Description]
    // 中文：启动管道，接收DynalgoFrameSet回调
    // English: Start pipeline with video callback (receives DynalgoFrameSet)
    virtual bool start(DynalgoVideoCallback callback) = 0;

    // [方法说明 / Method Description]
    // 中文：启动IMU管道，接收IMU回调
    // English: Start IMU pipeline with callback
    virtual bool startImu(DynalgoImuCallback callback) = 0;

    // [方法说明 / Method Description]
    // 中文：停止管道
    // English: Stop pipeline
    virtual void stop() = 0;

    // [方法说明 / Method Description]
    // 中文：停止IMU管道
    // English: Stop IMU pipeline
    virtual void stopImu() = 0;

    // [方法说明 / Method Description]
    // 中文：获取底层设备
    // English: Get the underlying device
    virtual std::shared_ptr<DynalgoDevice> getDevice() const = 0;

    // [方法说明 / Method Description]
    // 中文：检查是否启用点云输出
    // English: Check if point cloud output is enabled
    virtual bool isPcdEnabled() const {
        return false;
    }

    // [方法说明 / Method Description]
    // 中文：启用深度到点云转换，对于始终输出点的管道（如LiDAR）为空操作
    // English: Enable depth-to-point-cloud conversion. No-op for pipelines that always output points (e.g. LiDAR)
    virtual void setPointCloudEnabled(bool enable) {
        (void)enable;
    }

    // [方法说明 / Method Description]
    // 中文：获取当前D2C对齐模式
    // English: Get current D2C alignment mode
    virtual DynalgoAlignMode getAlignMode() const = 0;
};

// [类说明 / Class Description]
// 中文：抽象SDK上下文，用于设备发现
// English: DynalgoContext: abstract SDK context for device discovery.
class DynalgoContext
{
public:
    virtual ~DynalgoContext() = default;

    // [方法说明 / Method Description]
    // 中文：获取设备数量
    // English: Get device count
    virtual uint32_t getDeviceCount() = 0;

    // [方法说明 / Method Description]
    // 中文：根据索引获取设备
    // English: Get device by index
    virtual std::shared_ptr<DynalgoDevice> getDevice(uint32_t index) = 0;
};

} // namespace dynalgo
