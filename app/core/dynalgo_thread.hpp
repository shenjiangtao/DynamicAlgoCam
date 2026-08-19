// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_thread.hpp — Named thread utilities and StreamTask base class.
//
// [文件说明 / File Description]
// 中文：命名线程工具和StreamTask基类，提供每流工作线程框架
// English: Named thread utilities and StreamTask base class, provides per-stream worker thread framework
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

namespace dynalgo {

// [方法说明 / Method Description]
// 中文：设置线程名称，用于调试（在top -H、gdb info threads中可见）
// English: Set thread name for debugging (visible in top -H, gdb info threads)
void setThreadName(const std::string& name);

// [结构体说明 / Struct Description]
// 中文：帧数据块，包含像素数据、时间戳和深度缩放信息
// English: Frame data blob containing pixel data, timestamp, and depth scaling info
struct FrameBlob
{
    std::vector<uint8_t> data;
    uint32_t size = 0;
    uint64_t timestampUs = 0;
    float depthScale = 1.0f;
    float depthMinM = 0.3f;
    float depthMaxM = 5.0f;
};

// [类说明 / Class Description]
// 中文：流任务基类，管理每流工作线程、线程安全帧队列和启动/停止生命周期
// English: StreamTask base class, manages per-stream worker thread, thread-safe frame queue, and start/stop lifecycle
class StreamTask
{
public:
    explicit StreamTask(const std::string& name, size_t queueCapacity = 2);
    virtual ~StreamTask();

    // [方法说明 / Method Description]
    // 中文：启动工作线程
    // English: Start worker thread
    void start();

    // [方法说明 / Method Description]
    // 中文：停止工作线程并等待完成
    // English: Stop worker thread and wait for completion
    void stop();

    // [方法说明 / Method Description]
    // 中文：将帧数据入队，支持深度缩放和范围参数
    // English: Enqueue frame data with depth scaling and range parameters
    bool enqueue(const uint8_t* data, uint32_t size, uint64_t timestampUs, float depthScale = 1.0f,
                 float depthMinM = 0.3f, float depthMaxM = 5.0f);

    // [方法说明 / Method Description]
    // 中文：获取任务名称
    // English: Get task name
    const std::string& name() const {
        return name_;
    }

protected:
    // [方法说明 / Method Description]
    // 中文：处理帧的纯虚函数，子类必须实现
    // English: Pure virtual function for frame processing, subclasses must implement
    virtual void processFrame(const FrameBlob& blob) = 0;

    // [方法说明 / Method Description]
    // 中文：空闲时回调，可选实现
    // English: Idle callback, optional implementation
    virtual void onIdle() {}

    // [方法说明 / Method Description]
    // 中文：停止时回调，可选实现
    // English: Stop callback, optional implementation
    virtual void onStop() {}

    // [方法说明 / Method Description]
    // 中文：唤醒工作线程
    // English: Wake up worker thread
    void wakeup();

private:
    // [方法说明 / Method Description]
    // 中文：工作线程主循环
    // English: Worker thread main loop
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

} // namespace dynalgo
