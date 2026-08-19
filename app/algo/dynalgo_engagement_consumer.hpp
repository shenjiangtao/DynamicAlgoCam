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

class DynalgoEngagementFrameConsumer : public FrameConsumer
{
public:
    // The loop pointer must remain valid for the lifetime of this consumer.
    explicit DynalgoEngagementFrameConsumer(DynalgoEngagementLoop* loop);

    void consume(std::shared_ptr<DynalgoFrameSet> frameSet) override;
    void stopTask() override;

private:
    DynalgoEngagementLoop* loop_;
};

} // namespace dynalgo