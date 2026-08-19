// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_frame_queue.hpp — Bounded thread-safe queue for producer-consumer pattern.
//
// When the queue is full, the oldest item is dropped (back-pressure on the
// slow consumer). This ensures the consumer always gets the freshest data
// and the producer (SDK callback) is never blocked.
//
// Two specializations:
//   FrameQueue<DynalgoFrameSet>  — for video frames (shared_ptr)
//   FrameQueue<std::string>   — for IMU CSV lines

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dynalgo_frame.hpp"

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 有界线程安全队列，用于生产者-消费者模式。队列满时丢弃最旧数据，保证消费者总是获取最新数据
// English: Bounded thread-safe queue for producer-consumer pattern. Drops oldest when full, ensuring consumer always gets freshest data
template <typename T>
class FrameQueue
{
public:
    // [方法说明 / Method Description]
    // 中文: 构造函数，指定队列容量
    // English: Constructor, specify queue capacity
    explicit FrameQueue(size_t capacity);
    // [方法说明 / Method Description]
    // 中文: 推入数据，队列满时覆盖最旧数据
    // English: Push data, overwrites oldest when full
    void push(T item);
    // [方法说明 / Method Description]
    // 中文: 弹出数据，带超时等待
    // English: Pop data with timeout wait
    bool pop(T& item, uint32_t timeoutMs = 100);
    // [方法说明 / Method Description]
    // 中文: 关闭队列，唤醒所有等待线程
    // English: Shutdown queue, wake all waiting threads
    void shutdown();
    // [方法说明 / Method Description]
    // 中文: 唤醒所有等待的消费者线程
    // English: Wake all waiting consumer threads
    void wakeAll();

private:
    std::vector<T> buf_;
    size_t mask_;
    size_t capacity_;
    std::mutex mtx_;
    std::condition_variable cv_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    std::atomic<bool> shutdown_{ false };
};

// Video queue carries DynalgoFrameSet (SDK-agnostic).
using VideoFrameQueue = FrameQueue<std::shared_ptr<DynalgoFrameSet>>;
// IMU queue carries CSV lines (already SDK-agnostic).
using ImuFrameQueue = FrameQueue<std::string>;

} // namespace dynalgo
