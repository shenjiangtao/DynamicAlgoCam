// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_actuator_factory.cpp — Backend registry + createActuator() impl.
//
// The registry maps NioActuatorType → Creator. Concrete backends register
// themselves via registerActuator() in a static-init block guarded by their
// build option macro. No concrete backend is registered here — nio_core stays
// SDK-neutral. Concrete backends (DUMMY, real laser, real gimbal) live under
// app/actuator/ and self-register there.

#include "nio_actuator_factory.hpp"
#include "nio_log.hpp"

#include <mutex>
#include <unordered_map>

namespace nio {

namespace {

struct Registry {
    std::mutex mtx;
    std::unordered_map<NioActuatorType, ActuatorCreator> entries;
};

Registry& registry() {
    static Registry r;
    return r;
}

} // namespace

void registerActuator(NioActuatorType type, ActuatorCreator creator) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    if (r.entries.find(type) != r.entries.end()) {
        NIO_LOG_WARN_S("registerActuator: duplicate registration for type "
                       << static_cast<int>(type) << " — last one wins");
    }
    r.entries[type] = std::move(creator);
}

std::unique_ptr<NioActuator> createActuator(NioActuatorType type) {
    if (type == NioActuatorType::NONE)
        return nullptr;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mtx);
    auto it = r.entries.find(type);
    if (it == r.entries.end() || !it->second)
        return nullptr;
    return it->second();
}

} // namespace nio
