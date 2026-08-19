// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dummy_actuator.hpp — DynalgoActuatorType::DUMMY backend.
//
// A pure stub that implements every DynalgoActuator method as a log-only no-op.
// Used for engagement-loop dry runs and unit tests. Real-actuation safety
// comes from DynalgoActuatorConfig::dryRun (default true): DummyActuator never
// drives hardware regardless of the flag, but it DOES honor the flag in
// its log lines so integration tests can assert on it.

#pragma once

#include "dynalgo_actuator.hpp"

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 虚拟执行器实现，作为 DynalgoActuatorType::DUMMY 后端。所有方法均为无操作日志记录，用于干运行和单元测试。
// English: Dummy actuator implementation serving as DynalgoActuatorType::DUMMY backend. All methods are log-only no-ops for dry-runs and unit tests.
class DummyActuator : public DynalgoActuator
{
public:
    // [构造函数 / Constructor]
    // 中文: 默认构造函数。
    // English: Default constructor.
    DummyActuator() = default;
    // [析构函数 / Destructor]
    // 中文: 默认析构函数。
    // English: Default destructor.
    ~DummyActuator() override = default;

    // [方法: load / Method: load]
    // 中文: 加载执行器配置。
    // English: Load actuator configuration.
    bool load(const DynalgoActuatorConfig& cfg) override;
    // [方法: open / Method: open]
    // 中文: 打开执行器（无实际硬件操作）。
    // English: Open actuator (no real hardware operation).
    bool open() override;
    // [方法: aimAt / Method: aimAt]
    // 中文: 瞄准指定坐标（无实际运动）。
    // English: Aim at specified coordinates (no real motion).
    bool aimAt(float x_m, float y_m, float z_m) override;
    // [方法: fire / Method: fire]
    // 中文: 触发执行动作（无实际发射）。
    // English: Trigger actuation (no real emission).
    bool fire(double durationMs) override;
    // [方法: close / Method: close]
    // 中文: 关闭执行器。
    // English: Close actuator.
    bool close() override;
    // [方法: name / Method: name]
    // 中文: 返回执行器名称 "DUMMY"。
    // English: Return actuator name "DUMMY".
    const char* name() const override;
    // [方法: config / Method: config]
    // 中文: 返回当前配置。
    // English: Return current configuration.
    const DynalgoActuatorConfig& config() const override;

private:
    // [成员变量: cfg_ / Member: cfg_]
    // 中文: 存储的执行器配置。
    // English: Stored actuator configuration.
    DynalgoActuatorConfig cfg_{};
    // [成员变量: opened_ / Member: opened_]
    // 中文: 标记执行器是否已打开。
    // English: Flag indicating if actuator is opened.
    bool opened_ = false;
};

} // namespace dynalgo
