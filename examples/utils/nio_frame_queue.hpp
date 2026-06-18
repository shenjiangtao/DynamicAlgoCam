// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame_queue.hpp — Bounded thread-safe queue for producer-consumer pattern.
//
// When the queue is full, the oldest item is dropped (back-pressure on the
// slow consumer). This ensures the consumer always gets the freshest data
// and the producer (SDK callback) is never blocked.
//
// Two specializations:
//   FrameQueue<ob::FrameSet>  — for video frames (shared_ptr)
//   FrameQueue<std::string>   — for IMU CSV lines

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>
#include <libobsensor/ObSensor.hpp>

namespace nio {

template <typename T>
class FrameQueue {
public:
    // Construct a ring buffer with the given capacity (rounded up to next power-of-2).
    explicit FrameQueue(size_t capacity);

    // Enqueue an item. If full, the oldest item is dropped (drop-oldest back-pressure).
    // Never blocks the caller — O(1) lock + notify.
    void push(T item);

    // Dequeue an item with timeout. Returns false on timeout or shutdown with empty queue.
    bool pop(T &item, uint32_t timeoutMs = 100);

    // Signal consumer threads to exit. Wakes all waiters.
    void shutdown();

    // Wake all consumers without setting shutdown flag (e.g. for early exit).
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
    std::atomic<bool> shutdown_{false};
};

using VideoFrameQueue = FrameQueue<std::shared_ptr<ob::FrameSet>>;
using ImuFrameQueue = FrameQueue<std::string>;

} // namespace nio
