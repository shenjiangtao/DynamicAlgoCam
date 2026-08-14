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

class DummyActuator : public DynalgoActuator
{
public:
    DummyActuator() = default;
    ~DummyActuator() override = default;

    bool load(const DynalgoActuatorConfig& cfg) override;
    bool open() override;
    bool aimAt(float x_m, float y_m, float z_m) override;
    bool fire(double durationMs) override;
    bool close() override;
    const char* name() const override;

private:
    DynalgoActuatorConfig cfg_{};
    bool opened_ = false;
};

} // namespace dynalgo
