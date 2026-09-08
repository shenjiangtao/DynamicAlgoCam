#include "hal_base.hpp"

namespace dynalgo::hal {

HALRegistry& HALRegistry::instance() {
    static HALRegistry instance;
    return instance;
}

ResultVoid HALRegistry::registerHAL(const std::string& type, const std::string& platform,
                                    const std::string& vendor, CreateFunc create, DestroyFunc destroy,
                                    bool builtin) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string key = type + ":" + platform + ":" + vendor;
    if (entries_.find(key) != entries_.end()) {
        return ResultVoid::err(ErrorCode::ALREADY_INITIALIZED);
    }

    HALEntry entry;
    entry.name = vendor;
    entry.vendor = vendor;
    entry.platform = platform;
    entry.type = type;
    entry.create = create;
    entry.destroy = destroy;
    entry.is_builtin = builtin;

    entries_[key] = std::move(entry);
    return ResultVoid::ok();
}

ResultVoid HALRegistry::unregisterHAL(const std::string& type, const std::string& platform,
                                      const std::string& vendor) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string key = type + ":" + platform + ":" + vendor;
    auto it = entries_.find(key);
    if (it == entries_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    entries_.erase(it);
    return ResultVoid::ok();
}

Result<HALRegistry::HALEntry> HALRegistry::findHAL(const std::string& type, const std::string& platform,
                                                   const std::string& vendor) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string key = type + ":" + platform + ":" + vendor;
    auto it = entries_.find(key);
    if (it == entries_.end()) {
        return Result<HALEntry>::err(ErrorCode::NOT_FOUND);
    }
    return Result<HALEntry>::ok(it->second);
}

std::vector<HALRegistry::HALEntry> HALRegistry::findHALs(const std::string& type, const std::string& platform) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<HALEntry> result;
    std::string prefix = type + ":" + platform + ":";

    for (const auto& [key, entry] : entries_) {
        if (key.rfind(prefix, 0) == 0) {
            result.push_back(entry);
        }
    }
    return result;
}

std::vector<std::string> HALRegistry::getTypes() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> types;
    for (const auto& [key, entry] : entries_) {
        if (std::find(types.begin(), types.end(), entry.type) == types.end()) {
            types.push_back(entry.type);
        }
    }
    return types;
}

std::vector<std::string> HALRegistry::getPlatforms() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> platforms;
    for (const auto& [key, entry] : entries_) {
        if (std::find(platforms.begin(), platforms.end(), entry.platform) == platforms.end()) {
            platforms.push_back(entry.platform);
        }
    }
    return platforms;
}

std::vector<std::string> HALRegistry::getVendors(const std::string& type, const std::string& platform) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> vendors;
    std::string prefix = type + ":" + platform + ":";

    for (const auto& [key, entry] : entries_) {
        if (key.rfind(prefix, 0) == 0) {
            if (std::find(vendors.begin(), vendors.end(), entry.vendor) == vendors.end()) {
                vendors.push_back(entry.vendor);
            }
        }
    }
    return vendors;
}

} // namespace dynalgo::hal