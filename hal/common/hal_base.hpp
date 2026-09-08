#pragma once

#include <dynalgo/hal/hal_types.hpp>
#include "buffer_manager.hpp"
#include "sync_manager.hpp"
#include "plugin_loader.hpp"
#include <string>
#include <memory>
#include <atomic>

namespace dynalgo::hal {

class HALBase {
public:
    HALBase() = default;
    virtual ~HALBase() = default;

    virtual ResultVoid initialize() = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;
    virtual Result<uint32_t> getHALVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    bool isInitialized() const { return initialized_.load(); }

protected:
    void setInitialized(bool v) { initialized_.store(v); }

    BufferManager& buffers() { return BufferManager::instance(); }
    SyncManager& sync() { return SyncManager::instance(); }
    PluginLoader& plugins() { return PluginLoader::instance(); }

private:
    std::atomic<bool> initialized_{false};
};

template<typename Derived, typename Config>
class ConfigurableHAL : public HALBase {
public:
    ResultVoid configure(const Config& config) {
        if (initialized_.load()) {
            return ResultVoid::err(ErrorCode::ALREADY_INITIALIZED);
        }
        config_ = config;
        return ResultVoid::ok();
    }

    const Config& getConfig() const { return config_; }
    Config& getConfig() { return config_; }

protected:
    Config config_;
};

class HALRegistry {
public:
    using CreateFunc = HALBase* (*)();
    using DestroyFunc = void (*)(HALBase*);

    struct HALEntry {
        std::string name;
        std::string vendor;
        std::string platform;
        std::string type;
        CreateFunc create = nullptr;
        DestroyFunc destroy = nullptr;
        bool is_builtin = false;
    };

    static HALRegistry& instance();

    ResultVoid registerHAL(const std::string& type, const std::string& platform,
                           const std::string& vendor, CreateFunc create, DestroyFunc destroy,
                           bool builtin = false);

    ResultVoid unregisterHAL(const std::string& type, const std::string& platform,
                             const std::string& vendor);

    Result<HALEntry> findHAL(const std::string& type, const std::string& platform,
                             const std::string& vendor) const;

    std::vector<HALEntry> findHALs(const std::string& type, const std::string& platform) const;

    std::vector<std::string> getTypes() const;
    std::vector<std::string> getPlatforms() const;
    std::vector<std::string> getVendors(const std::string& type, const std::string& platform) const;

private:
    HALRegistry() = default;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, HALEntry> entries_;
};

#define DYNALGO_REGISTER_HAL(Type, Platform, Vendor, CreateFunc, DestroyFunc) \
    namespace { \
        struct Register_##Type##_##Platform##_##Vendor { \
            Register_##Type##_##Platform##_##Vendor() { \
                dynalgo::hal::HALRegistry::instance().registerHAL( \
                    #Type, #Platform, #Vendor, CreateFunc, DestroyFunc, true); \
            } \
        } register_instance_##Type##_##Platform##_##Vendor; \
    }

} // namespace dynalgo::hal