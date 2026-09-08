#pragma once

#include <dynalgo/hal/hal_types.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <dlfcn.h>
#include <mutex>

namespace dynalgo::hal {

class PluginLoader {
public:
    struct PluginInfo {
        std::string name;
        std::string vendor;
        std::string version;
        std::string path;
        std::string platform;
        void* handle = nullptr;
        bool loaded = false;
        uint64_t load_time_ns = 0;
    };

    using SymbolResolver = std::function<void*(const std::string&)>;

    static PluginLoader& instance();

    ResultVoid loadPlugin(const std::string& path);
    ResultVoid unloadPlugin(const std::string& name);

    ResultVoid loadPluginDir(const std::string& directory, const std::string& pattern = "*.so");
    ResultVoid unloadAll();

    template<typename T>
    Result<T*> getSymbol(const std::string& plugin_name, const std::string& symbol_name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = plugins_.find(plugin_name);
        if (it == plugins_.end()) {
            return Result<T*>::err(ErrorCode::NOT_FOUND);
        }
        if (!it->second.loaded) {
            return Result<T*>::err(ErrorCode::INVALID_STATE);
        }

        void* sym = dlsym(it->second.handle, symbol_name.c_str());
        if (!sym) {
            return Result<T*>::err(ErrorCode::NOT_FOUND);
        }
        return Result<T*>::ok(reinterpret_cast<T*>(sym));
    }

    Result<PluginInfo> getPluginInfo(const std::string& name) const;
    std::vector<PluginInfo> getAllPlugins() const;
    std::vector<PluginInfo> getPluginsByVendor(const std::string& vendor) const;
    std::vector<PluginInfo> getPluginsByPlatform(const std::string& platform) const;

    bool isLoaded(const std::string& name) const;

    void setSearchPaths(const std::vector<std::string>& paths);
    const std::vector<std::string>& getSearchPaths() const;

    ResultVoid registerExternalSymbol(const std::string& name, void* symbol);
    void* getExternalSymbol(const std::string& name) const;

private:
    PluginLoader() = default;
    ~PluginLoader();

    mutable std::mutex mutex_;
    std::unordered_map<std::string, PluginInfo> plugins_;
    std::unordered_map<std::string, void*> external_symbols_;
    std::vector<std::string> search_paths_ = {
        "./lib/hal",
        "/usr/local/lib/dynalgo/hal",
        "/opt/dynalgo/lib/hal",
        "/usr/lib/dynalgo/hal"
    };
};

template<typename Interface>
class PluginFactory {
public:
    using CreateFunc = Interface* (*)();
    using DestroyFunc = void (*)(Interface*);

    PluginFactory(const std::string& platform, const std::string& vendor_prefix)
        : platform_(platform), vendor_prefix_(vendor_prefix) {}

    Result<Interface*> create(const std::string& vendor) {
        std::string plugin_name = vendor_prefix_ + "_" + vendor;
        auto create_func = PluginLoader::instance().getSymbol<CreateFunc>(plugin_name, "create");
        auto destroy_func = PluginLoader::instance().getSymbol<DestroyFunc>(plugin_name, "destroy");

        if (!create_func.is_ok() || !destroy_func.is_ok()) {
            return Result<Interface*>::err(ErrorCode::NOT_SUPPORTED);
        }

        Interface* instance = create_func.value()();
        if (!instance) {
            return Result<Interface*>::err(ErrorCode::OUT_OF_MEMORY);
        }

        instances_[instance] = {plugin_name, destroy_func.value()};
        return Result<Interface*>::ok(instance);
    }

    void destroy(Interface* instance) {
        auto it = instances_.find(instance);
        if (it != instances_.end()) {
            it->second.destroy(instance);
            instances_.erase(it);
        }
    }

    ~PluginFactory() {
        for (auto& [instance, info] : instances_) {
            info.destroy(instance);
        }
    }

private:
    struct InstanceInfo {
        std::string plugin_name;
        DestroyFunc destroy;
    };

    std::string platform_;
    std::string vendor_prefix_;
    std::unordered_map<Interface*, InstanceInfo> instances_;
};

#define DYNALGO_HAL_PLUGIN_EXPORT(Interface, Impl) \
    extern "C" { \
        Interface* create() { return new Impl(); } \
        void destroy(Interface* ptr) { delete ptr; } \
    }

} // namespace dynalgo::hal