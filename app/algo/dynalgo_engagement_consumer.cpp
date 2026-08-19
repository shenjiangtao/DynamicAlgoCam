// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_engagement_consumer.cpp — Engagement frame consumer implementation.

#include "dynalgo_engagement_consumer.hpp"
#include "dynalgo_log.hpp"

namespace dynalgo {

// [构造函数说明 / Constructor Description]
// 中文: 初始化交互帧消费者
// English: Initialize engagement frame consumer
DynalgoEngagementFrameConsumer::DynalgoEngagementFrameConsumer(DynalgoEngagementLoop* loop)
    : loop_(loop)
{
}

// [方法说明 / Method Description]
// 中文: 消费帧集并馈送到交互循环
// English: Consume frame set and feed to engagement loop
void DynalgoEngagementFrameConsumer::consume(std::shared_ptr<DynalgoFrameSet> frameSet)
{
    if (loop_ && frameSet)
        loop_->onFrame(*frameSet);
}

// [方法说明 / Method Description]
// 中文: 停止交互循环
// English: Stop the engagement loop
void DynalgoEngagementFrameConsumer::stopTask()
{
    if (loop_)
        loop_->stop();
}

} // namespace dynalgo