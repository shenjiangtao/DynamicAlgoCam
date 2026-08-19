// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dummy_actuator.cpp — DUMMY backend implementation + self-registration.
//
// Every method is a log-only no-op. Real actuation is impossible from this
// backend regardless of cfg.dryRun. The flag still appears in log lines so
// integration tests and operators can distinguish dry-run vs production paths
// end-to-end. Self-registration runs at static-init time; linking this
// translation unit into any binary/executable makes the DUMMY type available.

#include "dummy_actuator.hpp"
#include "dynalgo_actuator_factory.hpp"
#include "dynalgo_log.hpp"

namespace dynalgo {

// [方法: load / Method: load]
// 中文: 加载配置并记录日志，不进行实际硬件初始化。
// English: Load configuration and log, no real hardware initialization.
bool DummyActuator::load(const DynalgoActuatorConfig& cfg) {
    cfg_ = cfg;
    DYNALGO_LOG_INFO_S("DummyActuator::load devicePath='" << cfg_.devicePath
                   << "' protocolHint='" << cfg_.protocolHint
                   << "' baudRate=" << cfg_.baudRate
                   << " dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}

// [方法: open / Method: open]
// 中文: 标记为已打开并记录日志，不连接实际设备。
// English: Mark as opened and log, no real device connection.
bool DummyActuator::open() {
    opened_ = true;
    DYNALGO_LOG_INFO_S("DummyActuator::open (dryRun=" << (cfg_.dryRun ? "on" : "off")
                   << ") — no real device");
    return true;
}

// [方法: aimAt / Method: aimAt]
// 中文: 记录瞄准坐标日志，不执行实际运动。
// English: Log aim coordinates, no real motion executed.
bool DummyActuator::aimAt(float x_m, float y_m, float z_m) {
    DYNALGO_LOG_INFO_S("DummyActuator::aimAt x=" << x_m << " y=" << y_m << " z=" << z_m
                   << " m (dryRun=" << (cfg_.dryRun ? "on" : "off")
                   << ") — no real motion");
    return true;
}

// [方法: fire / Method: fire]
// 中文: 记录触发时长日志，不执行实际发射。
// English: Log fire duration, no real emission executed.
bool DummyActuator::fire(double durationMs) {
    DYNALGO_LOG_INFO_S("DummyActuator::fire durationMs=" << durationMs
                   << " (dryRun=" << (cfg_.dryRun ? "on" : "off")
                   << ") — NO-OP, no real emission");
    return true;
}

// [方法: close / Method: close]
// 中文: 关闭执行器并记录日志。
// English: Close actuator and log.
bool DummyActuator::close() {
    bool wasOpen = opened_;
    opened_ = false;
    DYNALGO_LOG_INFO_S("DummyActuator::close (wasOpen=" << (wasOpen ? "true" : "false")
                   << ") — no real device");
    return true;
}

// [方法: name / Method: name]
// 中文: 返回执行器名称 "DUMMY"。
// English: Return actuator name "DUMMY".
const char* DummyActuator::name() const { return "DUMMY"; }

// [方法: config / Method: config]
// 中文: 返回当前存储的配置。
// English: Return current stored configuration.
const DynalgoActuatorConfig& DummyActuator::config() const { return cfg_; }

} // namespace dynalgo

namespace {

// [结构体: DummyActuatorRegistrar / Struct: DummyActuatorRegistrar]
// 中文: 静态初始化时自动注册 DUMMY 执行器类型到工厂。
// English: Auto-registers DUMMY actuator type to factory at static initialization.
struct DummyActuatorRegistrar {
    DummyActuatorRegistrar() {
        dynalgo::registerActuator(dynalgo::DynalgoActuatorType::DUMMY,
                              []() -> std::unique_ptr<dynalgo::DynalgoActuator> {
                                  return std::unique_ptr<dynalgo::DynalgoActuator>(new dynalgo::DummyActuator);
                              });
    }
} g_dummyActuatorRegistrar __attribute__((used));

} // namespace
