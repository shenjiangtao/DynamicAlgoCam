#include "buffer_manager.hpp"
#include <algorithm>

namespace dynalgo::hal {

BufferManager& BufferManager::instance() {
    static BufferManager instance;
    return instance;
}

void BufferManager::setAllocators(AllocFunc alloc, FreeFunc free,
                                  MapFunc map, UnmapFunc unmap,
                                  ImportFunc import, ExportFunc export_fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    alloc_func_ = std::move(alloc);
    free_func_ = std::move(free);
    map_func_ = std::move(map);
    unmap_func_ = std::move(unmap);
    import_func_ = std::move(import);
    export_func_ = std::move(export_fn);
}

Result<BufferHandle> BufferManager::allocate(size_t size, PixelFormat format,
                                             uint32_t width, uint32_t height,
                                             const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!alloc_func_) {
        return Result<BufferHandle>::err(ErrorCode::NOT_INITIALIZED);
    }

    auto result = alloc_func_(size, format, width, height);
    if (!result.is_ok()) {
        return result;
    }

    BufferHandle handle = result.value();
    BufferInfo info;
    info.handle = handle;
    info.size = size;
    info.format = format;
    info.width = width;
    info.height = height;
    info.ref_count = 1;
    info.owner = owner.empty() ? default_owner_ : owner;
    info.alloc_time_ns = now_ns();

    buffers_[handle.handle] = std::move(info);
    return Result<BufferHandle>::ok(handle);
}

Result<BufferHandle> BufferManager::allocateFrame(const FrameMetadata& frame,
                                                   const std::string& owner) {
    size_t size = frame.get_frame_size();
    if (size == 0 && frame.stride && frame.height) {
        size = frame.stride * frame.height;
    }
    return allocate(size, frame.format, frame.width, frame.height, owner);
}

ResultVoid BufferManager::free(const BufferHandle& handle) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!free_func_) {
        return ResultVoid::err(ErrorCode::NOT_INITIALIZED);
    }

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (it->second.is_mapped) {
        unmap(handle);
    }

    auto result = free_func_(handle);
    if (result.is_ok()) {
        buffers_.erase(it);
    }
    return result;
}

ResultVoid BufferManager::freeAll(const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<uint64_t> to_free;
    for (const auto& [handle, info] : buffers_) {
        if (info.owner == owner) {
            to_free.push_back(handle);
        }
    }

    for (auto handle : to_free) {
        free(BufferHandle{handle, 0});
    }

    return ResultVoid::ok();
}

Result<void*> BufferManager::map(const BufferHandle& handle) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return Result<void*>::err(ErrorCode::NOT_FOUND);
    }

    if (it->second.is_mapped) {
        return Result<void*>::ok(it->second.mapped_ptr);
    }

    if (!map_func_) {
        return Result<void*>::err(ErrorCode::NOT_SUPPORTED);
    }

    auto result = map_func_(handle);
    if (result.is_ok()) {
        it->second.mapped_ptr = result.value();
        it->second.is_mapped = true;
    }
    return result;
}

ResultVoid BufferManager::unmap(const BufferHandle& handle) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (!it->second.is_mapped) {
        return ResultVoid::ok();
    }

    if (!unmap_func_) {
        return ResultVoid::err(ErrorCode::NOT_SUPPORTED);
    }

    auto result = unmap_func_(handle);
    if (result.is_ok()) {
        it->second.mapped_ptr = nullptr;
        it->second.is_mapped = false;
    }
    return result;
}

Result<BufferHandle> BufferManager::importBuffer(const BufferHandle& external) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!import_func_) {
        return Result<BufferHandle>::err(ErrorCode::NOT_SUPPORTED);
    }

    auto result = import_func_(external);
    if (!result.is_ok()) {
        return result;
    }

    BufferHandle handle = result.value();
    BufferInfo info;
    info.handle = handle;
    info.size = 0;
    info.format = PixelFormat::UNKNOWN;
    info.owner = default_owner_;
    info.alloc_time_ns = now_ns();

    buffers_[handle.handle] = std::move(info);
    return Result<BufferHandle>::ok(handle);
}

Result<BufferHandle> BufferManager::exportBuffer(const BufferHandle& internal) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(internal.handle);
    if (it == buffers_.end()) {
        return Result<BufferHandle>::err(ErrorCode::NOT_FOUND);
    }

    if (!export_func_) {
        return Result<BufferHandle>::err(ErrorCode::NOT_SUPPORTED);
    }

    return export_func_(internal);
}

Result<BufferManager::BufferInfo> BufferManager::getInfo(const BufferHandle& handle) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return Result<BufferInfo>::err(ErrorCode::NOT_FOUND);
    }

    return Result<BufferInfo>::ok(it->second);
}

std::vector<BufferManager::BufferInfo> BufferManager::getAllBuffers() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<BufferInfo> result;
    result.reserve(buffers_.size());
    for (const auto& [handle, info] : buffers_) {
        result.push_back(info);
    }
    return result;
}

std::vector<BufferManager::BufferInfo> BufferManager::getBuffersByOwner(const std::string& owner) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<BufferInfo> result;
    for (const auto& [handle, info] : buffers_) {
        if (info.owner == owner) {
            result.push_back(info);
        }
    }
    return result;
}

ResultVoid BufferManager::addRef(const BufferHandle& handle) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    it->second.ref_count++;
    return ResultVoid::ok();
}

ResultVoid BufferManager::releaseRef(const BufferHandle& handle) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (it->second.ref_count > 0) {
        it->second.ref_count--;
    }

    if (it->second.ref_count == 0) {
        free(handle);
    }
    return ResultVoid::ok();
}

uint32_t BufferManager::getRefCount(const BufferHandle& handle) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = buffers_.find(handle.handle);
    if (it == buffers_.end()) {
        return 0;
    }
    return it->second.ref_count;
}

size_t BufferManager::getTotalAllocated() const {
    std::lock_guard<std::mutex> lock(mutex_);

    size_t total = 0;
    for (const auto& [handle, info] : buffers_) {
        total += info.size;
    }
    return total;
}

size_t BufferManager::getBufferCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffers_.size();
}

// BufferPool implementation
BufferPool::~BufferPool() {
    deinitialize();
}

ResultVoid BufferPool::initialize(const PoolConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        return ResultVoid::err(ErrorCode::ALREADY_INITIALIZED);
    }

    config_ = config;

    if (config.preallocate) {
        for (uint32_t i = 0; i < config.count; ++i) {
            // Simple allocation using malloc
            void* ptr = malloc(config.buffer_size);
            if (!ptr) {
                deinitialize();
                return ResultVoid::err(ErrorCode::OUT_OF_MEMORY);
            }
            
            BufferHandle handle;
            handle.handle = reinterpret_cast<uint64_t>(ptr);
            handle.platform_id = 0;
            handle.reserved = nullptr;
            
            all_handles_.push_back(handle);
            free_handles_.push_back(handle);
        }
    }

    initialized_ = true;
    return ResultVoid::ok();
}

ResultVoid BufferPool::deinitialize() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& handle : all_handles_) {
        free(reinterpret_cast<void*>(handle.handle));
    }
    all_handles_.clear();
    free_handles_.clear();
    initialized_ = false;
    return ResultVoid::ok();
}

Result<BufferHandle> BufferPool::acquire(uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!initialized_) {
        return Result<BufferHandle>::err(ErrorCode::NOT_INITIALIZED);
    }

    auto wait_for_free = [this]() { return !free_handles_.empty(); };

    if (timeout_ms == 0) {
        if (!wait_for_free()) {
            return Result<BufferHandle>::err(ErrorCode::NO_DATA);
        }
    } else {
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
        if (!cv_.wait_until(lock, deadline, wait_for_free)) {
            return Result<BufferHandle>::err(ErrorCode::TIMEOUT);
        }
    }

    BufferHandle handle = free_handles_.back();
    free_handles_.pop_back();
    return Result<BufferHandle>::ok(handle);
}

ResultVoid BufferPool::release(const BufferHandle& handle) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::find(all_handles_.begin(), all_handles_.end(), handle);
    if (it == all_handles_.end()) {
        return ResultVoid::err(ErrorCode::INVALID_ARG);
    }

    free_handles_.push_back(handle);
    cv_.notify_one();
    return ResultVoid::ok();
}

Result<BufferHandle> BufferPool::tryAcquire() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_ || free_handles_.empty()) {
        return Result<BufferHandle>::err(ErrorCode::NO_DATA);
    }

    BufferHandle handle = free_handles_.back();
    free_handles_.pop_back();
    return Result<BufferHandle>::ok(handle);
}

uint32_t BufferPool::availableCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return free_handles_.size();
}

uint32_t BufferPool::totalCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return all_handles_.size();
}

} // namespace dynalgo::hal