// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_model_factory.hpp — Factory for creating DynalgoModelBackend instances.
//
// [文件说明 / File Description]
// 中文：DynalgoModelBackend实例的工厂，隐藏具体后端类，后端通过链接/初始化时的钩子自注册
// English: Factory for creating DynalgoModelBackend instances, hides concrete backend classes, backends self-register via link/init hooks
//
// Mirrors the pattern of DynalgoDriverFactory (in dynalgo_driver_factory.hpp): a
// single createModelBackend() entry point hides concrete backend classes
// from the application layer. Backends register themselves via a hook at
// link/init time (see .cpp); this header has no dependency on any inference
// SDK.
//
// Backends that are gated by a build option (e.g. YOLOV8_PY behind
// ENABLE_YOLOV8_PY) self-register inside their .cpp guarded by the macro.

#pragma once

#include "dynalgo_model.hpp"

#include <functional>
#include <memory>

namespace dynalgo {

// [工厂函数 / Factory Function]
// 中文：创建后端实例，类型为NONE或未注册返回nullptr
// English: Create a backend instance, returns nullptr if type is NONE or no backend registered
std::unique_ptr<DynalgoModelBackend> createModelBackend(DynalgoModelType type);

// ---- Backend registration hook (used by backend implementations) ----
//
// [注册钩子 / Registration Hook]
// 中文：后端注册钩子，后端通过静态初始化块自注册，注册表是进程单例
// English: Backend registration hook, backends self-register via static-init blocks, registry is process-singleton
//
// A Creator is a function that returns a fresh DynalgoModelBackend each call.
// Backends call registerModelBackend(DynalgoModelType, Creator) during their
// construction (typically from a static-init block guarded by their build
// option macro). The registry is process-singleton.
using ModelBackendCreator = std::function<std::unique_ptr<DynalgoModelBackend>()>;
void registerModelBackend(DynalgoModelType type, ModelBackendCreator creator);

} // namespace dynalgo
