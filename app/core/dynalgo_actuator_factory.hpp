// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_actuator_factory.hpp — Factory for creating DynalgoActuator instances.
//
// Mirrors the pattern of DynalgoActuatorFactory (see dynalgo_model_factory.hpp):
// a single createActuator() entry point hides concrete backend classes from
// the application layer. Backends register themselves via a hook at link /
// init time (see .cpp); this header has no dependency on any actuator SDK.

#pragma once

#include "dynalgo_actuator.hpp"

#include <functional>
#include <memory>

namespace dynalgo {

// Create an actuator instance. Returns nullptr if `type` is NONE or no
// backend is registered for that type under the current build configuration.
std::unique_ptr<DynalgoActuator> createActuator(DynalgoActuatorType type);

// ---- Backend registration hook (used by backend implementations) ----
//
// A Creator returns a fresh DynalgoActuator each call. Backends call
// registerActuator(DynalgoActuatorType, Creator) during their construction
// (typically from a static-init block guarded by their build option macro).
// The registry is a process-singleton.
//
// IMPORTANT (static-archive link caveat):
//   Self-registration relies on a static-init block in the backend's
//   translation unit. When the backend library is a static archive (.a),
//   the linker drops unreferenced object files by default, so the registrar
//   may never run and createActuator() would return nullptr. Concrete
//   backend archives (e.g. libnio_actuators.a) must be linked with
//   `--whole-archive`:
//
//     target_link_libraries(<consumer> PRIVATE
//         "-Wl,--whole-archive" dynalgo::actuators "-Wl,--no-whole-archive")
//
//   This mirrors the same caveat the model backend framework will face once
//   real backends ship; the DUMMY backend walks through the exact same path.
using ActuatorCreator = std::function<std::unique_ptr<DynalgoActuator>()>;
void registerActuator(DynalgoActuatorType type, ActuatorCreator creator);

} // namespace dynalgo
