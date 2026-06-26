// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_d2c_align.hpp — Orbbec-specific D2C alignment filter.
//
// Wraps ob::Align to implement NioD2CAlign interface.

#pragma once

#include "nio_device.hpp"

#include <libobsensor/ObSensor.hpp>

namespace nio {

struct ObD2CAlign : public NioD2CAlign
{
    std::shared_ptr<ob::Align> align_;

    explicit ObD2CAlign(std::shared_ptr<ob::Align> a) : align_(std::move(a)) {}

    bool process(std::shared_ptr<void> nativeFrameSet, NioAlignedFrame& out) override {
        if (!align_ || !nativeFrameSet)
            return false;
        auto obFs = std::static_pointer_cast<ob::FrameSet>(nativeFrameSet);
        if (!obFs)
            return false;
        auto alignedFrame = align_->process(obFs);
        auto alignedFS = alignedFrame ? std::dynamic_pointer_cast<ob::FrameSet>(alignedFrame) : nullptr;
        if (!alignedFS)
            alignedFS = obFs;

        auto colorFrame = alignedFS->getFrame(OB_FRAME_COLOR);
        auto depthFrame = alignedFS->getFrame(OB_FRAME_DEPTH);
        if (!colorFrame || !depthFrame)
            return false;

        out.colorData = colorFrame->getData();
        out.colorSize = colorFrame->getDataSize();
        out.colorTs = colorFrame->getTimeStampUs();
        out.depthData = depthFrame->getData();
        out.depthSize = depthFrame->getDataSize();
        out.depthTs = depthFrame->getTimeStampUs();

        try {
            auto depthF = depthFrame->as<ob::DepthFrame>();
            if (depthF)
                out.depthScale = depthF->getValueScale();
        } catch (...) {
        }
        return true;
    }
};

} // namespace nio
