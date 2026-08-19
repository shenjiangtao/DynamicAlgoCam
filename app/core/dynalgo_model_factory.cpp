// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_model_factory.cpp — Backend registry + createModelBackend() impl.
//
// [文件说明 / File Description]
// 中文：后端注册表和createModelBackend()实现，将DynalgoModelType映射到Creator
// English: Backend registry and createModelBackend() implementation, maps DynalgoModelType to Creator
//
// The registry maps DynalgoModelType → Creator. Concrete backends register
// themselves via registerModelBackend() in a static-init block guarded by
// their build option macro (no such backend exists yet — the infrastructure
// is added now per the "add a model inference abstraction layer" task, but
// no YOLOv8 / ONNX / TensorRT backend is wired up per the "only the
// abstraction layer" scope).

#include "dynalgo_model_factory.hpp"
#include "dynalgo_log.hpp"

#include <mutex>
#include <unordered_map>

namespace dynalgo {

namespace {

// [注册表结构 / Registry Structure]
// 中文：后端注册表，映射模型类型到创建函数
// English: Backend registry, maps model type to creator function
struct Registry {
    std::mutex mtx;
    std::unordered_map<DynalgoModelType, ModelBackendCreator> entries;
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
// 中文：注册模型后端，支持重复注册（后者覆盖前者）
// English: Register model backend, supports duplicate registration (last one wins)
void registerModelBackend(DynalgoModelType type, ModelBackendCreator creator) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    if (r.entries.find(type) != r.entries.end()) {
        DYNALGO_LOG_WARN_S("registerModelBackend: duplicate registration for type "
                       << static_cast<int>(type) << " — last one wins");
    }
    r.entries[type] = std::move(creator);
}

// [工厂函数 / Factory Function]
// 中文：创建模型后端实例，类型为NONE或未注册返回nullptr
// English: Create model backend instance, returns nullptr if type is NONE or not registered
std::unique_ptr<DynalgoModelBackend> createModelBackend(DynalgoModelType type) {
    if (type == DynalgoModelType::NONE)
        return nullptr;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    auto it = r.entries.find(type);
    if (it == r.entries.end() || !it->second)
        return nullptr;
    return it->second();
}

} // namespace dynalgo
