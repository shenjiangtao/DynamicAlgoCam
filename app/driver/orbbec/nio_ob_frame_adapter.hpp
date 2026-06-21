// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_ob_frame_adapter.hpp — Convert Orbbec SDK FrameSet → NioFrameSet.
//
// Called in the SDK video callback (producer boundary).  Copies pixel
// data out of the SDK buffer so downstream consumers are fully
// SDK-agnostic.  For IMU frames, extracts accel/gyro values into
// NioImuSample.

#pragma once

#include "nio_frame.hpp"
#include "nio_ob_adapter.hpp"

#include <libobsensor/ObSensor.hpp>

namespace nio {

// Convert ob::FrameSet → NioFrameSet (deep copy pixel data).
// Extracts all video frames present in the FrameSet.
inline NioFrameSet obFrameSetToNio(std::shared_ptr<ob::FrameSet> obFs) {
    NioFrameSet nioFs;

    // Attach the original ob::FrameSet for pipeline operations (e.g. ob::Align)
    nioFs.nativeFrameSet = std::static_pointer_cast<void>(obFs);

    auto extract = [&](OBFrameType obType, NioFrameType nioType) {
        auto frame = obFs->getFrame(obType);
        if (!frame)
            return;
        NioFrame nf;
        nf.type = nioType;
        nf.format = obFormatToNio(frame->getFormat());
        nf.timestampUs = frame->getTimeStampUs();

        try {
            auto vf = frame->as<ob::VideoFrame>();
            if (vf) {
                nf.width = vf->getWidth();
                nf.height = vf->getHeight();
            }
        } catch (...) {
        }

        // Depth scale
        if (obType == OB_FRAME_DEPTH) {
            try {
                auto df = frame->as<ob::DepthFrame>();
                if (df)
                    nf.depthScale = df->getValueScale();
            } catch (...) {
            }
        }

        auto *data = frame->getData();
        auto size = frame->getDataSize();
        if (data && size > 0)
            nf.data.assign(data, data + size);

        nioFs.setFrame(nioType, std::move(nf));
    };

    extract(OB_FRAME_COLOR,    NioFrameType::COLOR);
    extract(OB_FRAME_DEPTH,    NioFrameType::DEPTH);
    extract(OB_FRAME_IR,       NioFrameType::IR);
    extract(OB_FRAME_IR_LEFT,  NioFrameType::IR_LEFT);
    extract(OB_FRAME_IR_RIGHT, NioFrameType::IR_RIGHT);

    return nioFs;
}

// Extract IMU samples from ob::FrameSet → vector<NioImuSample>.
inline std::vector<NioImuSample> obImuToNioSamples(std::shared_ptr<ob::FrameSet> obFs) {
    std::vector<NioImuSample> samples;

    auto accelFrameRaw = obFs->getFrame(OB_FRAME_ACCEL);
    if (accelFrameRaw) {
        try {
            auto accel = accelFrameRaw->as<ob::AccelFrame>();
            if (accel) {
                auto val = accel->getValue();
                NioImuSample s;
                s.type = NioFrameType::ACCEL;
                s.timestampUs = accel->getTimeStampUs();
                s.x = val.x;
                s.y = val.y;
                s.z = val.z;
                s.temperature = accel->getTemperature();
                samples.push_back(s);
            }
        } catch (...) {
        }
    }

    auto gyroFrameRaw = obFs->getFrame(OB_FRAME_GYRO);
    if (gyroFrameRaw) {
        try {
            auto gyro = gyroFrameRaw->as<ob::GyroFrame>();
            if (gyro) {
                auto val = gyro->getValue();
                NioImuSample s;
                s.type = NioFrameType::GYRO;
                s.timestampUs = gyro->getTimeStampUs();
                s.x = val.x;
                s.y = val.y;
                s.z = val.z;
                s.temperature = gyro->getTemperature();
                samples.push_back(s);
            }
        } catch (...) {
        }
    }

    return samples;
}

} // namespace nio
