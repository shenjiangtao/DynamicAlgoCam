// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_engagement_consumer.cpp — Engagement frame consumer implementation.

#include "dynalgo_engagement_consumer.hpp"
#include "dynalgo_log.hpp"

namespace dynalgo {

DynalgoEngagementFrameConsumer::DynalgoEngagementFrameConsumer(DynalgoEngagementLoop* loop)
    : loop_(loop)
{
}

void DynalgoEngagementFrameConsumer::consume(std::shared_ptr<DynalgoFrameSet> frameSet)
{
    if (loop_ && frameSet)
        loop_->onFrame(*frameSet);
}

void DynalgoEngagementFrameConsumer::stopTask()
{
    if (loop_)
        loop_->stop();
}

} // namespace dynalgo