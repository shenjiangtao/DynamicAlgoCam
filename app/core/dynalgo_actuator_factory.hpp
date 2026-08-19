// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_actuator_factory.hpp — Factory for creating DynalgoActuator instances.
//
// [文件说明 / File Description]
// 中文：DynalgoActuator实例的工厂，隐藏具体后端类，后端通过链接/初始化时的钩子自注册
// English: Factory for creating DynalgoActuator instances, hides concrete backend classes, backends self-register via link/init hooks
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

// [工厂函数 / Factory Function]
// 中文：创建执行器实例，类型为NONE或未注册返回nullptr
// English: Create an actuator instance, returns nullptr if type is NONE or no backend registered
std::unique_ptr<DynalgoActuator> createActuator(DynalgoActuatorType type);

// ---- Backend registration hook (used by backend implementations) ----
//
// [注册钩子 / Registration Hook]
// 中文：后端注册钩子，后端通过静态初始化块自注册，注册表是进程单例
// English: Backend registration hook, backends self-register via static-init blocks, registry is process-singleton
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
