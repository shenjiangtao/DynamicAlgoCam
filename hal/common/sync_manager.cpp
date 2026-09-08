#include "sync_manager.hpp"
#include <algorithm>
#include <thread>

namespace dynalgo::hal {

SyncManager& SyncManager::instance() {
    static SyncManager instance;
    return instance;
}

void SyncManager::setCallbacks(CreateFenceFunc create, SignalFunc signal, WaitFunc wait,
                               DestroyFunc destroy, QueryFunc query) {
    std::lock_guard<std::mutex> lock(mutex_);
    create_func_ = std::move(create);
    signal_func_ = std::move(signal);
    wait_func_ = std::move(wait);
    destroy_func_ = std::move(destroy);
    query_func_ = std::move(query);
}

Result<FenceHandle> SyncManager::createFence() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!create_func_) {
        return Result<FenceHandle>::err(ErrorCode::NOT_INITIALIZED);
    }

    auto result = create_func_();
    if (!result.is_ok()) {
        return result;
    }

    FenceHandle handle = result.value();
    FenceInfo info;
    info.handle = handle;
    info.signaled = false;
    info.signal_time_ns = 0;
    info.owner = "sync_manager";

    fences_[handle.handle] = std::move(info);
    return Result<FenceHandle>::ok(handle);
}

ResultVoid SyncManager::destroyFence(const FenceHandle& fence) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = fences_.find(fence.handle);
    if (it == fences_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (destroy_func_) {
        destroy_func_(fence);
    }

    fences_.erase(it);
    return ResultVoid::ok();
}

ResultVoid SyncManager::signalFence(const FenceHandle& fence, int64_t timestamp_ns) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = fences_.find(fence.handle);
    if (it == fences_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (!signal_func_) {
        return ResultVoid::err(ErrorCode::NOT_SUPPORTED);
    }

    int64_t ts = (timestamp_ns >= 0) ? timestamp_ns : now_ns();
    auto result = signal_func_(fence, ts);
    if (!result.is_ok()) {
        return result;
    }

    it->second.signaled = true;
    it->second.signal_time_ns = ts;

    for (auto& callback : it->second.waiters) {
        callback();
    }
    it->second.waiters.clear();

    return ResultVoid::ok();
}

ResultVoid SyncManager::waitFence(const FenceHandle& fence, uint64_t timeout_ns) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = fences_.find(fence.handle);
    if (it == fences_.end()) {
        return ResultVoid::err(ErrorCode::NOT_FOUND);
    }

    if (it->second.signaled) {
        return ResultVoid::ok();
    }

    if (!wait_func_) {
        return ResultVoid::err(ErrorCode::NOT_SUPPORTED);
    }

    auto result = wait_func_(fence, timeout_ns);
    if (result.is_ok()) {
        it->second.signaled = true;
        it->second.signal_time_ns = now_ns();
    }
    return result;
}

Result<bool> SyncManager::queryFence(const FenceHandle& fence) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = fences_.find(fence.handle);
    if (it == fences_.end()) {
        return Result<bool>::err(ErrorCode::NOT_FOUND);
    }

    if (it->second.signaled) {
        return Result<bool>::ok(true);
    }

    if (!query_func_) {
        return Result<bool>::ok(false);
    }

    return query_func_(fence);
}

Result<std::vector<FenceHandle>> SyncManager::waitMultiple(const std::vector<FenceHandle>& fences,
                                                            uint64_t timeout_ns,
                                                            bool wait_all) {
    if (fences.empty()) {
        return Result<std::vector<FenceHandle>>::ok({});
    }

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::nanoseconds(timeout_ns);

    std::vector<FenceHandle> signaled;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& fence : fences) {
                auto it = fences_.find(fence.handle);
                if (it != fences_.end() && it->second.signaled &&
                    std::find(signaled.begin(), signaled.end(), fence) == signaled.end()) {
                    signaled.push_back(fence);
                }
            }

            if (wait_all) {
                if (signaled.size() == fences.size()) {
                    return Result<std::vector<FenceHandle>>::ok(signaled);
                }
            } else {
                if (!signaled.empty()) {
                    return Result<std::vector<FenceHandle>>::ok(signaled);
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            if (wait_all || signaled.empty()) {
                return Result<std::vector<FenceHandle>>::err(ErrorCode::TIMEOUT);
            }
            return Result<std::vector<FenceHandle>>::ok(signaled);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void SyncManager::addWaiter(const FenceHandle& fence, std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = fences_.find(fence.handle);
    if (it != fences_.end()) {
        it->second.waiters.push_back(std::move(callback));
    }
}

// TimelineSemaphore implementation
ResultVoid TimelineSemaphore::initialize(uint64_t initial_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return ResultVoid::err(ErrorCode::ALREADY_INITIALIZED);
    }
    current_value_ = initial_value;
    initialized_ = true;
    return ResultVoid::ok();
}

ResultVoid TimelineSemaphore::deinitialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false;
    callbacks_.clear();
    return ResultVoid::ok();
}

ResultVoid TimelineSemaphore::signal(uint64_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return ResultVoid::err(ErrorCode::NOT_INITIALIZED);
    }
    if (value <= current_value_) {
        return ResultVoid::err(ErrorCode::INVALID_ARG);
    }
    current_value_ = value;

    auto it = callbacks_.find(value);
    if (it != callbacks_.end()) {
        for (auto& cb : it->second) {
            cb();
        }
        callbacks_.erase(it);
    }
    cv_.notify_all();
    return ResultVoid::ok();
}

ResultVoid TimelineSemaphore::wait(uint64_t value, uint64_t timeout_ns) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!initialized_) {
        return ResultVoid::err(ErrorCode::NOT_INITIALIZED);
    }

    auto predicate = [this, value]() { return current_value_ >= value; };

    if (timeout_ns == UINT64_MAX) {
        cv_.wait(lock, predicate);
    } else {
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::nanoseconds(timeout_ns);
        if (!cv_.wait_until(lock, deadline, predicate)) {
            return ResultVoid::err(ErrorCode::TIMEOUT);
        }
    }
    return ResultVoid::ok();
}

Result<uint64_t> TimelineSemaphore::getValue() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return Result<uint64_t>::err(ErrorCode::NOT_INITIALIZED);
    }
    return Result<uint64_t>::ok(current_value_);
}

ResultVoid TimelineSemaphore::addCallback(uint64_t value, std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return ResultVoid::err(ErrorCode::NOT_INITIALIZED);
    }
    if (current_value_ >= value) {
        callback();
    } else {
        callbacks_[value].push_back(std::move(callback));
    }
    return ResultVoid::ok();
}

// FrameSync implementation
ResultVoid FrameSync::initialize(const SyncConfig& config, FrameReadyCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        return ResultVoid::err(ErrorCode::ALREADY_INITIALIZED);
    }

    if (config.num_streams == 0 || config.num_streams > 16) {
        return ResultVoid::err(ErrorCode::INVALID_ARG);
    }

    config_ = config;
    callback_ = std::move(callback);
    pending_.resize(config_.num_streams);
    initialized_ = true;
    return ResultVoid::ok();
}

ResultVoid FrameSync::deinitialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false;
    pending_.clear();
    return ResultVoid::ok();
}

ResultVoid FrameSync::submitFrame(uint32_t stream_id, FrameMetadata frame) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        return ResultVoid::err(ErrorCode::NOT_INITIALIZED);
    }

    if (stream_id >= config_.num_streams) {
        return ResultVoid::err(ErrorCode::INVALID_ARG);
    }

    pending_[stream_id] = {std::move(frame), true};

    bool all_ready = true;
    for (const auto& pf : pending_) {
        if (!pf.valid) {
            all_ready = false;
            break;
        }
    }

    if (all_ready) {
        uint64_t min_ts = UINT64_MAX;
        uint64_t max_ts = 0;
        for (const auto& pf : pending_) {
            min_ts = std::min(min_ts, pf.frame.timestamp_ns);
            max_ts = std::max(max_ts, pf.frame.timestamp_ns);
        }

        if (max_ts - min_ts <= config_.max_time_diff_ns) {
            for (auto& pf : pending_) {
                if (callback_) {
                    callback_(0, pf.frame);
                }
                pf.valid = false;
            }
            synced_count_++;
            last_sync_time_ns_ = now_ns();
        } else if (config_.drop_late_frames) {
            dropped_count_ += config_.num_streams;
            for (auto& pf : pending_) {
                pf.valid = false;
            }
        }
    }

    return ResultVoid::ok();
}

ResultVoid FrameSync::flush() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& pf : pending_) {
        if (pf.valid) {
            if (callback_) {
                callback_(0, pf.frame);
            }
            pf.valid = false;
        }
    }
    return ResultVoid::ok();
}

uint32_t FrameSync::getSyncedFrameCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return synced_count_;
}

uint32_t FrameSync::getDroppedFrameCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_count_;
}

void FrameSync::resetCounters() {
    std::lock_guard<std::mutex> lock(mutex_);
    synced_count_ = 0;
    dropped_count_ = 0;
}

} // namespace dynalgo::hal