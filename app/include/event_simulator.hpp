#pragma once

#include "event.hpp"
#include <functional>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <utility>

/// Helper class used in tests to generate synthetic events.
/// It runs a background thread that fires events according to a schedule
/// of <delay_ms, name> pairs.
class EventSimulator {
public:
    using Callback = std::function<void(const Event&)>;

    explicit EventSimulator(Callback cb) : cb_(std::move(cb)), running_(false) {}

    /// Schedule is a vector of {delay_ms, event_name} where delay is relative
    /// to the moment `start()` is called.
    void start(const std::vector<std::pair<uint64_t, std::string>>& schedule) {
        if (running_) return;
        running_ = true;
        worker_ = std::thread([this, schedule] {
            auto base = std::chrono::steady_clock::now();
            for (const auto& item : schedule) {
                uint64_t delay = item.first;
                std::this_thread::sleep_until(base + std::chrono::milliseconds(delay));
                if (!running_) break;
                uint64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::system_clock::now().time_since_epoch()).count();
                cb_(Event(item.second, nowMs));
            }
        });
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

    ~EventSimulator() { stop(); }

private:
    Callback cb_;               // user‑provided sink
    std::atomic<bool> running_; // controls thread lifetime
    std::thread worker_;        // background event emitter
};
