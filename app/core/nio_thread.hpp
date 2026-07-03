// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_thread.hpp — Named thread utilities and StreamTask base class.
//
// setThreadName(): wraps pthread_setname_np for debugging (visible in
// top -H, gdb info threads, /proc/<pid>/task/<tid>/comm).
//
// StreamTask: base class for per-stream worker threads. Each task owns:
//   - a named std::thread
//   - a thread-safe frame queue (bounded, spsc-like)
//   - start/stop lifecycle
//
// Subclasses override processFrame() to implement stream-specific logic
// (H264 encoding, depth raw writing, D2C fusion, IMU CSV logging, etc.).
// The SDK callback enqueues frames via enqueue() — O(1) memcpy + notify.
// The worker thread dequeues and processes frames independently.

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nio {

void setThreadName(const std::string& name);

struct FrameBlob
{
    std::vector<uint8_t> data;
    uint32_t size = 0;
    uint64_t timestampUs = 0;
    float depthScale = 1.0f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
};

class StreamTask
{
public:
    explicit StreamTask(const std::string& name, size_t queueCapacity = 2);
    virtual ~StreamTask() override;

    void start();
    void stop();

    bool enqueue(const uint8_t* data, uint32_t size, uint64_t timestampUs, float depthScale = 1.0f,
                 float depthMinM = 0.3f, float depthMaxM = 5.0f);

    const std::string& name() const {
        return name_;
    }

protected:
    virtual void processFrame(const FrameBlob& blob) = 0;
    virtual void onIdle() {}
    virtual void onStop() {}
    void wakeup();

private:
    void run();

    std::string name_;
    size_t queueCapacity_;

    std::vector<FrameBlob> queue_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    std::mutex mtx_;
    std::condition_variable cv_;

    std::thread thread_;
    std::atomic<bool> running_{ false };
};

} // namespace nio
