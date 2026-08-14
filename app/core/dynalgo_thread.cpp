// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_thread.cpp — Named thread utilities and StreamTask implementation.

#include "dynalgo_thread.hpp"
#include "dynalgo_log.hpp"

#include <algorithm>
#include <cstring>

#ifdef __linux__
#include <pthread.h>
#endif

namespace dynalgo {

void setThreadName(const std::string& name) {
#ifdef __linux__
    std::string truncated = name;
    if (truncated.size() > 15)
        truncated.resize(15);
    pthread_setname_np(pthread_self(), truncated.c_str());
#endif
    (void)name;
}

StreamTask::StreamTask(const std::string& name, size_t queueCapacity) : name_(name), queueCapacity_(queueCapacity) {
    queue_.resize(queueCapacity_ + 1);
}

StreamTask::~StreamTask() {
    stop();
}

void StreamTask::start() {
    if (running_.load())
        return;
    running_ = true;
    thread_ = std::thread(&StreamTask::run, this);
}

void StreamTask::stop() {
    running_ = false;
    { std::lock_guard<std::mutex> lock(mtx_); }
    cv_.notify_one();
    if (thread_.joinable())
        thread_.join();
}

bool StreamTask::enqueue(const uint8_t* data, uint32_t size, uint64_t timestampUs, float depthScale, float depthMinM,
                         float depthMaxM) {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (count_ >= queueCapacity_) {
            head_ = (head_ + 1) % queue_.size();
            count_--;
        }
        auto& slot = queue_[tail_];
        if (slot.data.size() < size)
            slot.data.resize(size);
        std::memcpy(slot.data.data(), data, size);
        slot.size = size;
        slot.timestampUs = timestampUs;
        slot.depthScale = depthScale;
        slot.depthMinM = depthMinM;
        slot.depthMaxM = depthMaxM;
        tail_ = (tail_ + 1) % queue_.size();
        count_++;
    }
    cv_.notify_one();
    return true;
}

void StreamTask::wakeup() {
    cv_.notify_one();
}

void StreamTask::run() {
    setThreadName(name_);
    DYNALGO_LOG_DEBUG_S("StreamTask started: " << name_);

    while (running_.load()) {
        FrameBlob blob;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() { return count_ > 0 || !running_.load(); });
            if (count_ > 0) {
                blob = std::move(queue_[head_]);
                head_ = (head_ + 1) % queue_.size();
                count_--;
            }
        }

        if (blob.size > 0) {
            processFrame(blob);
        } else {
            onIdle();
        }
    }

    while (true) {
        FrameBlob blob;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (count_ == 0)
                break;
            blob = std::move(queue_[head_]);
            head_ = (head_ + 1) % queue_.size();
            count_--;
        }
        if (blob.size > 0)
            processFrame(blob);
    }

    onStop();

    DYNALGO_LOG_DEBUG_S("StreamTask stopped: " << name_);
}

} // namespace dynalgo
