// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_model_factory.cpp — Backend registry + createModelBackend() impl.
//
// The registry maps NioModelType → Creator. Concrete backends register
// themselves via registerModelBackend() in a static-init block guarded by
// their build option macro (no such backend exists yet — the infrastructure
// is added now per the "add a model inference abstraction layer" task, but
// no YOLOv8 / ONNX / TensorRT backend is wired up per the "only the
// abstraction layer" scope).

#include "nio_model_factory.hpp"
#include "nio_log.hpp"

#include <mutex>
#include <unordered_map>

namespace nio {

namespace {

struct Registry {
    std::mutex mtx;
    std::unordered_map<NioModelType, ModelBackendCreator> entries;
};

Registry& registry() {
    static Registry r;
    return r;
}

} // namespace

void registerModelBackend(NioModelType type, ModelBackendCreator creator) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    if (r.entries.find(type) != r.entries.end()) {
        NIO_LOG_WARN_S("registerModelBackend: duplicate registration for type "
                       << static_cast<int>(type) << " — last one wins");
    }
    r.entries[type] = std::move(creator);
}

std::unique_ptr<NioModelBackend> createModelBackend(NioModelType type) {
    if (type == NioModelType::NONE)
        return nullptr;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    auto it = r.entries.find(type);
    if (it == r.entries.end() || !it->second)
        return nullptr;
    return it->second();
}

} // namespace nio
