// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_thread.cpp — Named thread utilities and StreamTask implementation.
//
// [文件说明 / File Description]
// 中文：命名线程工具和StreamTask实现，提供工作线程管理和帧队列处理
// English: Named thread utilities and StreamTask implementation, provides worker thread management and frame queue processing

#include "dynalgo_thread.hpp"
#include "dynalgo_log.hpp"

#include <algorithm>
#include <cstring>

#ifdef __linux__
#include <pthread.h>
#endif

namespace dynalgo {

// [方法说明 / Method Description]
// 中文：设置线程名称，用于调试，在top -H和gdb info threads中可见
// English: Set thread name for debugging, visible in top -H and gdb info threads
void setThreadName(const std::string& name) {
#ifdef __linux__
    std::string truncated = name;
    if (truncated.size() > 15)
        truncated.resize(15);
    pthread_setname_np(pthread_self(), truncated.c_str());
#endif
    (void)name;
}

// [构造函数 / Constructor]
// 中文：初始化流任务，设置名称和队列容量
// English: Initialize stream task with name and queue capacity
StreamTask::StreamTask(const std::string& name, size_t queueCapacity) : name_(name), queueCapacity_(queueCapacity) {
    queue_.resize(queueCapacity_ + 1);
}

// [析构函数 / Destructor]
// 中文：停止工作线程
// English: Stop worker thread
StreamTask::~StreamTask() {
    stop();
}

// [方法说明 / Method Description]
// 中文：启动工作线程
// English: Start worker thread
void StreamTask::start() {
    if (running_.load())
        return;
    running_ = true;
    thread_ = std::thread(&StreamTask::run, this);
}

// [方法说明 / Method Description]
// 中文：停止工作线程并等待完成
// English: Stop worker thread and wait for completion
void StreamTask::stop() {
    running_ = false;
    { std::lock_guard<std::mutex> lock(mtx_); }
    cv_.notify_one();
    if (thread_.joinable())
        thread_.join();
}

// [方法说明 / Method Description]
// 中文：将帧数据入队，支持深度缩放和范围参数
// English: Enqueue frame data with depth scaling and range parameters
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

// [方法说明 / Method Description]
// 中文：唤醒工作线程
// English: Wake up worker thread
void StreamTask::wakeup() {
    cv_.notify_one();
}

// [方法说明 / Method Description]
// 中文：工作线程主循环，处理帧队列中的帧
// English: Worker thread main loop, processes frames from the frame queue
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
