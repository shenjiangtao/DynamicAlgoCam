// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_model_factory.hpp — Factory for creating NioModelBackend instances.
//
// Mirrors the pattern of NioDriverFactory (in nio_driver_factory.hpp): a
// single createModelBackend() entry point hides concrete backend classes
// from the application layer. Backends register themselves via a hook at
// link/init time (see .cpp); this header has no dependency on any inference
// SDK.
//
// Backends that are gated by a build option (e.g. YOLOV8_PY behind
// ENABLE_YOLOV8_PY) self-register inside their .cpp guarded by the macro.

#pragma once

#include "nio_model.hpp"

#include <functional>
#include <memory>

namespace nio {

// Create a backend instance. Returns nullptr if `type` is NONE or no
// backend is registered for that type under the current build configuration.
std::unique_ptr<NioModelBackend> createModelBackend(NioModelType type);

// ---- Backend registration hook (used by backend implementations) ----
//
// A Creator is a function that returns a fresh NioModelBackend each call.
// Backends call registerModelBackend(NioModelType, Creator) during their
// construction (typically from a static-init block guarded by their build
// option macro). The registry is process-singleton.
using ModelBackendCreator = std::function<std::unique_ptr<NioModelBackend>()>;
void registerModelBackend(NioModelType type, ModelBackendCreator creator);

} // namespace nio
