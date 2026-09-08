#pragma once

#include <dynalgo/hal/hal_types.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <memory>
#include <functional>

namespace dynalgo::hal {

class BufferManager {
public:
    struct BufferInfo {
        BufferHandle handle;
        size_t size = 0;
        PixelFormat format = PixelFormat::UNKNOWN;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t stride = 0;
        uint32_t ref_count = 0;
        bool is_mapped = false;
        void* mapped_ptr = nullptr;
        std::string owner;
        uint64_t alloc_time_ns = 0;
    };

    using AllocFunc = std::function<Result<BufferHandle>(size_t, PixelFormat, uint32_t, uint32_t)>;
    using FreeFunc = std::function<ResultVoid(const BufferHandle&)>;
    using MapFunc = std::function<Result<void*>(const BufferHandle&)>;
    using UnmapFunc = std::function<ResultVoid(const BufferHandle&)>;
    using ImportFunc = std::function<Result<BufferHandle>(const BufferHandle&)>;
    using ExportFunc = std::function<Result<BufferHandle>(const BufferHandle&)>;

    static BufferManager& instance();

    void setAllocators(AllocFunc alloc, FreeFunc free,
                       MapFunc map = nullptr, UnmapFunc unmap = nullptr,
                       ImportFunc import = nullptr, ExportFunc export_fn = nullptr);

    Result<BufferHandle> allocate(size_t size, PixelFormat format,
                                  uint32_t width = 0, uint32_t height = 0,
                                  const std::string& owner = "");

    Result<BufferHandle> allocateFrame(const FrameMetadata& frame, const std::string& owner = "");

    ResultVoid free(const BufferHandle& handle);
    ResultVoid freeAll(const std::string& owner);

    Result<void*> map(const BufferHandle& handle);
    ResultVoid unmap(const BufferHandle& handle);

    Result<BufferHandle> importBuffer(const BufferHandle& external);
    Result<BufferHandle> exportBuffer(const BufferHandle& internal);

    Result<BufferInfo> getInfo(const BufferHandle& handle) const;
    std::vector<BufferInfo> getAllBuffers() const;
    std::vector<BufferInfo> getBuffersByOwner(const std::string& owner) const;

    ResultVoid addRef(const BufferHandle& handle);
    ResultVoid releaseRef(const BufferHandle& handle);
    uint32_t getRefCount(const BufferHandle& handle) const;

    void setDefaultOwner(const std::string& owner) { default_owner_ = owner; }
    const std::string& getDefaultOwner() const { return default_owner_; }

    size_t getTotalAllocated() const;
    size_t getBufferCount() const;

private:
    BufferManager() = default;
    ~BufferManager() = default;

    mutable std::mutex mutex_;
    std::unordered_map<uint64_t, BufferInfo> buffers_;
    AllocFunc alloc_func_;
    FreeFunc free_func_;
    MapFunc map_func_;
    UnmapFunc unmap_func_;
    ImportFunc import_func_;
    ExportFunc export_func_;
    std::string default_owner_ = "unknown";
    uint64_t next_handle_ = 1;
};

class BufferPool {
public:
    struct PoolConfig {
        size_t buffer_size = 0;
        PixelFormat format = PixelFormat::NV12;
        uint32_t width = 1920;
        uint32_t height = 1080;
        uint32_t count = 4;
        bool preallocate = true;
    };

    BufferPool() = default;
    ~BufferPool();

    ResultVoid initialize(const PoolConfig& config);
    ResultVoid deinitialize();

    Result<BufferHandle> acquire(uint32_t timeout_ms = 1000);
    ResultVoid release(const BufferHandle& handle);

    Result<BufferHandle> tryAcquire();
    uint32_t availableCount() const;
    uint32_t totalCount() const;

    const PoolConfig& getConfig() const { return config_; }

private:
    PoolConfig config_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<BufferHandle> free_handles_;
    std::vector<BufferHandle> all_handles_;
    bool initialized_ = false;
};

} // namespace dynalgo::hal