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

bool DummyActuator::load(const DynalgoActuatorConfig& cfg) {
    cfg_ = cfg;
    DYNALGO_LOG_INFO_S("DummyActuator::load devicePath='" << cfg_.devicePath
                   << "' protocolHint='" << cfg_.protocolHint
                   << "' baudRate=" << cfg_.baudRate
                   << " dryRun=" << (cfg_.dryRun ? "on" : "off"));
    return true;
}

bool DummyActuator::open() {
    opened_ = true;
    DYNALGO_LOG_INFO_S("DummyActuator::open (dryRun=" << (cfg_.dryRun ? "on" : "off")
                   << ") — no real device");
    return true;
}

bool DummyActuator::aimAt(float x_m, float y_m, float z_m) {
    DYNALGO_LOG_INFO_S("DummyActuator::aimAt x=" << x_m << " y=" << y_m << " z=" << z_m
                   << " m (dryRun=" << (cfg_.dryRun ? "on" : "off")
                   << ") — no real motion");
    return true;
}

bool DummyActuator::fire(double durationMs) {
    DYNALGO_LOG_INFO_S("DummyActuator::fire durationMs=" << durationMs
                   << " (dryRun=" << (cfg_.dryRun ? "on" : "off")
                   << ") — NO-OP, no real emission");
    return true;
}

bool DummyActuator::close() {
    bool wasOpen = opened_;
    opened_ = false;
    DYNALGO_LOG_INFO_S("DummyActuator::close (wasOpen=" << (wasOpen ? "true" : "false")
                   << ") — no real device");
    return true;
}

const char* DummyActuator::name() const { return "DUMMY"; }

const DynalgoActuatorConfig& DummyActuator::config() const { return cfg_; }

} // namespace dynalgo

namespace {

struct DummyActuatorRegistrar {
    DummyActuatorRegistrar() {
        dynalgo::registerActuator(dynalgo::DynalgoActuatorType::DUMMY,
                              []() -> std::unique_ptr<dynalgo::DynalgoActuator> {
                                  return std::unique_ptr<dynalgo::DynalgoActuator>(new dynalgo::DummyActuator);
                              });
    }
} g_dummyActuatorRegistrar __attribute__((used));

} // namespace
