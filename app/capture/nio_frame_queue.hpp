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
//   FrameQueue<NioFrameSet>  — for video frames (shared_ptr)
//   FrameQueue<std::string>   — for IMU CSV lines

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nio_frame.hpp"

namespace nio {

template <typename T>
class FrameQueue
{
public:
    explicit FrameQueue(size_t capacity);
    void push(T item);
    bool pop(T& item, uint32_t timeoutMs = 100);
    void shutdown();
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

// Video queue carries NioFrameSet (SDK-agnostic).
using VideoFrameQueue = FrameQueue<std::shared_ptr<NioFrameSet>>;
// IMU queue carries CSV lines (already SDK-agnostic).
using ImuFrameQueue = FrameQueue<std::string>;

} // namespace nio
