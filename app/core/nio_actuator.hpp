// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_actuator.hpp — SDK-neutral actuator abstraction layer.
//
// Defines the contract for pluggable external actuators (laser, gimbal, ...).
// Concrete backends live outside nio_core (e.g. under app/actuator); this
// header only states the interface. nio_core stays SDK-neutral — no serial /
// CAN / vendor dependency is added by including this file.
//
// Usage:
//   auto actuator = createActuator(NioActuatorType::DUMMY);
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
//     (see nio_detection_to_3d.hpp, added later): +X right, +Y down, +Z forward.
//   - Backends are responsible for any per-device kinematics required to
//     translate this into yaw/pitch or other actuator-specific commands.
//
// Safety default: NioActuatorConfig::dryRun is true by default. A concrete
// backend MUST refuse to drive real hardware when dryRun is true — only log.
// Only the caller can explicitly pass dryRun=false to enable real actuation.

#pragma once

#include "nio_types.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace nio {

// Actuator backend type. Mirrors NioDriverVendor / NioModelType patterns.
enum class NioActuatorType {
    NONE = 0,         // no backend — every call is a no-op
    DUMMY,            // in-process stub for dry runs / unit tests
    LASER_GENERIC,    // future: real laser (Modbus-RTU / serial / CAN)
    GIMBAL_GENERIC    // future: real gimbal (yaw/pitch platform)
};

// Backend configuration. Reserved for backend-specific params (device path,
// protocol hint, baud rate, ...). Backends cast/handle keys they understand.
struct NioActuatorConfig {
    std::string devicePath;      // e.g. "/dev/ttyUSB0", "can0", empty = stub
    std::string protocolHint;    // e.g. "modbus-rtu", "raw-serial", "can-id"
    bool dryRun = true;          // SAFETY DEFAULT: dry run; no real actuation
    int baudRate = 115200;       // serial baud (ignored for non-serial)
};

// Abstract actuator backend. Concrete implementations are created by
// NioActuatorFactory (see nio_actuator_factory.hpp) and live in actuator code.
class NioActuator
{
public:
    virtual ~NioActuator() = default;

    // Load config / initialise backend. Returns true on success.
    // Must be called once before open().
    virtual bool load(const NioActuatorConfig& cfg) = 0;

    // Open the underlying device handle. Returns true on success.
    // In dryRun mode this may simply set a state flag and log.
    virtual bool open() = 0;

    // Aim at a 3D point in camera optical frame (meters). See header comment
    // for the coordinate convention. Returns true on success.
    // In dryRun mode this only logs the requested point.
    virtual bool aimAt(float x_m, float y_m, float z_m) = 0;

    // Fire the actuator (e.g. emit laser pulse) for `durationMs` milliseconds.
    // In dryRun mode this MUST be a no-op; only log.
    // Returns true on success.
    virtual bool fire(double durationMs) = 0;

    // Close the underlying device handle. Idempotent.
    virtual bool close() = 0;

    // Human-readable backend name, e.g. "DUMMY", "LASER_GENERIC".
    virtual const char* name() const = 0;
};

} // namespace nio
