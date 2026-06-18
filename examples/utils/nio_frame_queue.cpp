// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_frame_queue.cpp — Bounded thread-safe queue implementation (drop-oldest).

#include "nio_frame_queue.hpp"

namespace nio {

template <typename T>
FrameQueue<T>::FrameQueue(size_t capacity) : capacity_(capacity) {
    size_t size = 1;
    while (size < capacity_ + 1)
        size <<= 1;
    buf_.resize(size);
    mask_ = size - 1;
}

template <typename T>
void FrameQueue<T>::push(T item) {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (count_ >= capacity_) {
            head_ = (head_ + 1) & mask_;
            count_--;
        }
        buf_[tail_] = std::move(item);
        tail_ = (tail_ + 1) & mask_;
        count_++;
    }
    cv_.notify_one();
}

template <typename T>
bool FrameQueue<T>::pop(T& item, uint32_t timeoutMs) {
    std::unique_lock<std::mutex> lock(mtx_);
    if (!cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this]() { return count_ > 0 || shutdown_.load(); }))
        return false;
    if (shutdown_.load() && count_ == 0)
        return false;
    if (count_ == 0)
        return false;
    item = std::move(buf_[head_]);
    head_ = (head_ + 1) & mask_;
    count_--;
    return true;
}

template <typename T>
void FrameQueue<T>::shutdown() {
    shutdown_ = true;
    cv_.notify_all();
}

template <typename T>
void FrameQueue<T>::wakeAll() {
    cv_.notify_all();
}

template class FrameQueue<std::shared_ptr<ob::FrameSet>>;
template class FrameQueue<std::string>;

} // namespace nio
