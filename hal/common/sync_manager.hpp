#pragma once

#include <dynalgo/hal/hal_types.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <atomic>

namespace dynalgo::hal {

class SyncManager {
public:
    struct FenceInfo {
        FenceHandle handle;
        bool signaled = false;
        int64_t signal_time_ns = 0;
        std::string owner;
        std::vector<std::function<void()>> waiters;
    };

    using CreateFenceFunc = std::function<Result<FenceHandle>()>;
    using SignalFunc = std::function<ResultVoid(const FenceHandle&, int64_t timestamp_ns)>;
    using WaitFunc = std::function<ResultVoid(const FenceHandle&, uint64_t timeout_ns)>;
    using DestroyFunc = std::function<ResultVoid(const FenceHandle&)>;
    using QueryFunc = std::function<Result<bool>(const FenceHandle&)>;

    static SyncManager& instance();

    void setCallbacks(CreateFenceFunc create, SignalFunc signal, WaitFunc wait,
                      DestroyFunc destroy, QueryFunc query);

    Result<FenceHandle> createFence();
    ResultVoid destroyFence(const FenceHandle& fence);

    ResultVoid signalFence(const FenceHandle& fence, int64_t timestamp_ns = -1);
    ResultVoid waitFence(const FenceHandle& fence, uint64_t timeout_ns = UINT64_MAX);
    Result<bool> queryFence(const FenceHandle& fence);

    Result<std::vector<FenceHandle>> waitMultiple(const std::vector<FenceHandle>& fences,
                                                  uint64_t timeout_ns = UINT64_MAX,
                                                  bool wait_all = true);

    void addWaiter(const FenceHandle& fence, std::function<void()> callback);

private:
    SyncManager() = default;
    ~SyncManager() = default;

    std::mutex mutex_;
    std::unordered_map<uint64_t, FenceInfo> fences_;
    CreateFenceFunc create_func_;
    SignalFunc signal_func_;
    WaitFunc wait_func_;
    DestroyFunc destroy_func_;
    QueryFunc query_func_;
    uint64_t next_fence_ = 1;
};

class TimelineSemaphore {
public:
    struct TimelinePoint {
        uint64_t value = 0;
        std::chrono::steady_clock::time_point time;
        bool signaled = false;
    };

    TimelineSemaphore() = default;
    ~TimelineSemaphore() = default;

    ResultVoid initialize(uint64_t initial_value = 0);
    ResultVoid deinitialize();

    ResultVoid signal(uint64_t value);
    ResultVoid wait(uint64_t value, uint64_t timeout_ns = UINT64_MAX);
    Result<uint64_t> getValue() const;

    ResultVoid addCallback(uint64_t value, std::function<void()> callback);

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    uint64_t current_value_ = 0;
    std::unordered_map<uint64_t, std::vector<std::function<void()>>> callbacks_;
    bool initialized_ = false;
};

class FrameSync {
public:
    struct SyncConfig {
        uint32_t num_streams = 2;
        uint64_t max_time_diff_ns = 33'333'333;
        bool drop_late_frames = true;
        bool block_on_sync = true;
    };

    using FrameReadyCallback = std::function<void(uint32_t stream_id, const FrameMetadata&)>;

    FrameSync() = default;
    ~FrameSync() = default;

    ResultVoid initialize(const SyncConfig& config, FrameReadyCallback callback);
    ResultVoid deinitialize();

    ResultVoid submitFrame(uint32_t stream_id, FrameMetadata frame);
    ResultVoid flush();

    uint32_t getSyncedFrameCount() const;
    uint32_t getDroppedFrameCount() const;
    void resetCounters();

private:
    struct PendingFrame {
        FrameMetadata frame;
        bool valid = false;
    };

    SyncConfig config_;
    FrameReadyCallback callback_;
    mutable std::mutex mutex_;
    std::vector<PendingFrame> pending_;
    uint64_t last_sync_time_ns_ = 0;
    uint32_t synced_count_ = 0;
    uint32_t dropped_count_ = 0;
    bool initialized_ = false;
};

} // namespace dynalgo::hal