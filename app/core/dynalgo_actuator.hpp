// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_actuator.hpp — SDK-neutral actuator abstraction layer.
//
// [文件说明 / File Description]
// 中文：SDK中立的执行器抽象层，定义可插拔外部执行器（激光、云台等）的接口契约
// English: SDK-neutral actuator abstraction layer, defines the contract for pluggable external actuators (laser, gimbal, ...)
//
// Defines the contract for pluggable external actuators (laser, gimbal, ...).
// Concrete backends live outside dynalgo_core (e.g. under app/actuator); this
// header only states the interface. dynalgo_core stays SDK-neutral — no serial /
// CAN / vendor dependency is added by including this file.
//
// Usage:
//   auto actuator = createActuator(DynalgoActuatorType::DUMMY);
//   if (actuator && actuator->load(cfg)) {
//       actuator->open();
//       actuator->aimAt(1.0f, 2.0f, 3.0f);  // meters, camera-optical frame
//       actuator->fire(10.0);                // ms
//       actuator->close();
//   }
//
// Coordinate convention for aimAt(x,y,z):
//   - Origin is the camera optical center.
//   - Units are meters.
//   - Frame is the same as that produced by detectionCenterToCamera3D()
//     (see dynalgo_detection_to_3d.hpp, added later): +X right, +Y down, +Z forward.
//   - Backends are responsible for any per-device kinematics required to
//     translate this into yaw/pitch or other actuator-specific commands.
//
// Safety default: DynalgoActuatorConfig::dryRun is true by default. A concrete
// backend MUST refuse to drive real hardware when dryRun is true — only log.
// Only the caller can explicitly pass dryRun=false to enable real actuation.

#pragma once

#include "dynalgo_types.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace dynalgo {

// [枚举说明 / Enum Description]
// 中文：执行器后端类型，与DynalgoDriverVendor/DynalgoModelType模式保持一致
// English: Actuator backend type. Mirrors DynalgoDriverVendor / DynalgoModelType patterns.
enum class DynalgoActuatorType {
    NONE = 0,         // no backend — every call is a no-op
    DUMMY,            // in-process stub for dry runs / unit tests
    LASER_GENERIC,    // future: real laser (Modbus-RTU / serial / CAN)
    GIMBAL_GENERIC    // future: real gimbal (yaw/pitch platform)
};

// [结构体说明 / Struct Description]
// 中文：后端配置，保留用于后端特定参数（设备路径、协议提示、波特率等）
// English: Backend configuration. Reserved for backend-specific params (device path, protocol hint, baud rate, ...).
struct DynalgoActuatorConfig {
    std::string devicePath;      // e.g. "/dev/ttyUSB0", "can0", empty = stub
    std::string protocolHint;    // e.g. "modbus-rtu", "raw-serial", "can-id"
    bool dryRun = true;          // SAFETY DEFAULT: dry run; no real actuation
    int baudRate = 115200;       // serial baud (ignored for non-serial)
};

// [类说明 / Class Description]
// 中文：抽象执行器后端基类，具体实现由DynalgoActuatorFactory创建
// English: Abstract actuator backend. Concrete implementations are created by DynalgoActuatorFactory.
class DynalgoActuator
{
public:
    virtual ~DynalgoActuator() = default;

    // [方法说明 / Method Description]
    // 中文：加载配置/初始化后端，成功返回true，必须在open()之前调用
    // English: Load config / initialise backend. Returns true on success. Must be called once before open().
    virtual bool load(const DynalgoActuatorConfig& cfg) = 0;

    // [方法说明 / Method Description]
    // 中文：打开设备句柄，成功返回true，dryRun模式下仅设置状态标志并记录日志
    // English: Open the underlying device handle. Returns true on success. In dryRun mode this may simply set a state flag and log.
    virtual bool open() = 0;

    // [方法说明 / Method Description]
    // 中文：瞄准相机光学坐标系中的3D点（米），dryRun模式下仅记录请求的点
    // English: Aim at a 3D point in camera optical frame (meters). In dryRun mode this only logs the requested point.
    virtual bool aimAt(float x_m, float y_m, float z_m) = 0;

    // [方法说明 / Method Description]
    // 中文：触发执行器（如发射激光脉冲）指定毫秒数，dryRun模式下必须为空操作
    // English: Fire the actuator (e.g. emit laser pulse) for `durationMs` milliseconds. In dryRun mode this MUST be a no-op.
    virtual bool fire(double durationMs) = 0;

    // [方法说明 / Method Description]
    // 中文：关闭设备句柄，幂等操作
    // English: Close the underlying device handle. Idempotent.
    virtual bool close() = 0;

    // [方法说明 / Method Description]
    // 中文：返回人类可读的后端名称
    // English: Return human-readable backend name
    virtual const char* name() const = 0;

    // [方法说明 / Method Description]
    // 中文：获取当前配置（由load()设置），返回内部存储配置的引用
    // English: Current config (set by load()). Returns reference to internally stored config.
    virtual const DynalgoActuatorConfig& config() const = 0;
};

} // namespace dynalgo
