// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// dummy_actuator.hpp — NioActuatorType::DUMMY backend.
//
// A pure stub that implements every NioActuator method as a log-only no-op.
// Used for engagement-loop dry runs and unit tests. Real-actuation safety
// comes from NioActuatorConfig::dryRun (default true): DummyActuator never
// drives hardware regardless of the flag, but it DOES honor the flag in
// its log lines so integration tests can assert on it.

#pragma once

#include "nio_actuator.hpp"

namespace nio {

class DummyActuator : public NioActuator
{
public:
    DummyActuator() = default;
    ~DummyActuator() override = default;

    bool load(const NioActuatorConfig& cfg) override;
    bool open() override;
    bool aimAt(float x_m, float y_m, float z_m) override;
    bool fire(double durationMs) override;
    bool close() override;
    const char* name() const override;

private:
    NioActuatorConfig cfg_{};
    bool opened_ = false;
};

} // namespace nio
