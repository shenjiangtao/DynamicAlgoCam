#include "plugin_loader.hpp"
#include <filesystem>
#include <algorithm>

namespace dynalgo::hal {

PluginLoader& PluginLoader::instance() {
    static PluginLoader instance;
    return instance;
}

PluginLoader::~PluginLoader() {
    unloadAll();
}

ResultVoid PluginLoader::loadPlugin(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::filesystem::path p(path);
    if (!std::filesystem::exists(p)) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    std::string name = p.stem().string();
    if (name.rfind("lib", 0) == 0) {
        name = name.substr(3);
    }

    if (plugins_.find(name) != plugins_.end()) {
        return ResultVoid::err(ErrorCode::ALREADY_INITIALIZED);
    }

    void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        return ResultVoid::err(ErrorCode::DEVICE_ERROR);
    }

    PluginInfo info;
    info.name = name;
    info.path = path;
    info.handle = handle;
    info.loaded = true;
    info.load_time_ns = now_ns();

    plugins_[name] = std::move(info);
    return Result<void>::ok();
}

ResultVoid PluginLoader::unloadPlugin(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = plugins_.find(name);
    if (it == plugins_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (it->second.loaded && it->second.handle) {
        dlclose(it->second.handle);
    }

    plugins_.erase(it);
    return Result<void>::ok();
}

ResultVoid PluginLoader::loadPluginDir(const std::string& directory, const std::string& pattern) {
    std::filesystem::path dir(directory);
    if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    std::vector<std::string> plugins_to_load;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            if (ext == ".so" || ext == ".dylib" || ext == ".dll") {
                plugins_to_load.push_back(entry.path().string());
            }
        }
    }

    for (const auto& plugin_path : plugins_to_load) {
        loadPlugin(plugin_path);
    }

    return Result<void>::ok();
}

ResultVoid PluginLoader::unloadAll() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& [name, info] : plugins_) {
        if (info.loaded && info.handle) {
            dlclose(info.handle);
        }
    }
    plugins_.clear();
    return Result<void>::ok();
}

Result<PluginLoader::PluginInfo> PluginLoader::getPluginInfo(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = plugins_.find(name);
    if (it == plugins_.end()) {
        return Result<PluginInfo>::err(ErrorCode::NOT_FOUND);
    }
    return Result<PluginInfo>::ok(it->second);
}

std::vector<PluginLoader::PluginInfo> PluginLoader::getAllPlugins() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<PluginInfo> result;
    result.reserve(plugins_.size());
    for (const auto& [name, info] : plugins_) {
        result.push_back(info);
    }
    return result;
}

std::vector<PluginLoader::PluginInfo> PluginLoader::getPluginsByVendor(const std::string& vendor) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<PluginInfo> result;
    for (const auto& [name, info] : plugins_) {
        if (info.vendor == vendor) {
            result.push_back(info);
        }
    }
    return result;
}

std::vector<PluginLoader::PluginInfo> PluginLoader::getPluginsByPlatform(const std::string& platform) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<PluginInfo> result;
    for (const auto& [name, info] : plugins_) {
        if (info.platform == platform) {
            result.push_back(info);
        }
    }
    return result;
}

bool PluginLoader::isLoaded(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = plugins_.find(name);
    return it != plugins_.end() && it->second.loaded;
}

void PluginLoader::setSearchPaths(const std::vector<std::string>& paths) {
    std::lock_guard<std::mutex> lock(mutex_);
    search_paths_ = paths;
}

const std::vector<std::string>& PluginLoader::getSearchPaths() const {
    return search_paths_;
}

ResultVoid PluginLoader::registerExternalSymbol(const std::string& name, void* symbol) {
    std::lock_guard<std::mutex> lock(mutex_);
    external_symbols_[name] = symbol;
    return Result<void>::ok();
}

void* PluginLoader::getExternalSymbol(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = external_symbols_.find(name);
    if (it != external_symbols_.end()) {
        return it->second;
    }
    return nullptr;
}

} // namespace dynalgo::hal