// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_engagement_consumer.hpp — FrameConsumer that feeds frames into
// DynalgoEngagementLoop. Used only when --engage-* flags are provided.

#pragma once

#include "dynalgo_engagement_loop.hpp"
#include "../capture/dynalgo_frame_consumer.hpp"

#include <memory>

namespace dynalgo {

// [类说明 / Class Description]
// 中文: 交互帧消费者，将帧馈送到交互循环
// English: FrameConsumer that feeds frames into engagement loop
class DynalgoEngagementFrameConsumer : public FrameConsumer
{
public:
    // The loop pointer must remain valid for the lifetime of this consumer.
    explicit DynalgoEngagementFrameConsumer(DynalgoEngagementLoop* loop);

    // [方法说明 / Method Description]
    // 中文: 消费帧集并馈送到交互循环
    // English: Consume frame set and feed to engagement loop
    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    // [方法说明 / Method Description]
    // 中文: 停止消费任务
    // English: Stop the consumption task
    void stopTask() override;

private:
    DynalgoEngagementLoop* loop_;
};

} // namespace dynalgo