// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_actuator_factory.cpp — Backend registry + createActuator() impl.
//
// [文件说明 / File Description]
// 中文：后端注册表和createActuator()实现，将DynalgoActuatorType映射到Creator
// English: Backend registry and createActuator() implementation, maps DynalgoActuatorType to Creator
//
// The registry maps DynalgoActuatorType → Creator. Concrete backends register
// themselves via registerActuator() in a static-init block guarded by their
// build option macro. No concrete backend is registered here — dynalgo_core stays
// SDK-neutral. Concrete backends (DUMMY, real laser, real gimbal) live under
// app/actuator/ and self-register there.

#include "dynalgo_actuator_factory.hpp"
#include "dynalgo_log.hpp"

#include <mutex>
#include <unordered_map>

namespace dynalgo {

namespace {

// [注册表结构 / Registry Structure]
// 中文：后端注册表，映射执行器类型到创建函数
// English: Backend registry, maps actuator type to creator function
struct Registry {
    std::mutex mtx;
    std::unordered_map<DynalgoActuatorType, ActuatorCreator> entries;
};

// [获取注册表单例 / Get Registry Singleton]
// 中文：获取进程单例注册表
// English: Get process-singleton registry
Registry& registry() {
    static Registry r;
    return r;
}

} // namespace

// [注册函数 / Registration Function]
// 中文：注册执行器后端，支持重复注册（后者覆盖前者）
// English: Register actuator backend, supports duplicate registration (last one wins)
void registerActuator(DynalgoActuatorType type, ActuatorCreator creator) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    if (r.entries.find(type) != r.entries.end()) {
        DYNALGO_LOG_WARN_S("registerActuator: duplicate registration for type "
                       << static_cast<int>(type) << " — last one wins");
    }
    r.entries[type] = std::move(creator);
}

// [工厂函数 / Factory Function]
// 中文：创建执行器实例，类型为NONE或未注册返回nullptr
// English: Create actuator instance, returns nullptr if type is NONE or not registered
std::unique_ptr<DynalgoActuator> createActuator(DynalgoActuatorType type) {
    if (type == DynalgoActuatorType::NONE)
        return nullptr;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    auto it = r.entries.find(type);
    if (it == r.entries.end() || !it->second)
        return nullptr;
    return it->second();
}

} // namespace dynalgo
