// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_actuator_factory.cpp — Backend registry + createActuator() impl.
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

struct Registry {
    std::mutex mtx;
    std::unordered_map<DynalgoActuatorType, ActuatorCreator> entries;
};

Registry& registry() {
    static Registry r;
    return r;
}

} // namespace

void registerActuator(DynalgoActuatorType type, ActuatorCreator creator) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    if (r.entries.find(type) != r.entries.end()) {
        DYNALGO_LOG_WARN_S("registerActuator: duplicate registration for type "
                       << static_cast<int>(type) << " — last one wins");
    }
    r.entries[type] = std::move(creator);
}

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
