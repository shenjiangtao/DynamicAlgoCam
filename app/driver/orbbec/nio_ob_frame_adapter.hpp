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

        auto* data = frame->getData();
        auto size = frame->getDataSize();
        if (data && size > 0)
            nf.data.assign(data, data + size);

        nioFs.setFrame(nioType, std::move(nf));
    };

    extract(OB_FRAME_COLOR, NioFrameType::COLOR);
    extract(OB_FRAME_DEPTH, NioFrameType::DEPTH);
    extract(OB_FRAME_IR, NioFrameType::IR);
    extract(OB_FRAME_IR_LEFT, NioFrameType::IR_LEFT);
    extract(OB_FRAME_IR_RIGHT, NioFrameType::IR_RIGHT);

    // Point cloud: wrap raw OB_FORMAT_POINT data in self-describing wire format
    auto pointFrameRaw = obFs->getFrame(OB_FRAME_POINTS);
    if (pointFrameRaw) {
        auto* srcData = pointFrameRaw->getData();
        auto srcSize = pointFrameRaw->getDataSize();
        if (srcData && srcSize > 0) {
            PcdLayout layout = PcdLayout::obXyz();
            uint32_t nPts = static_cast<uint32_t>(srcSize / layout.srcPointSize);
            uint32_t nFields = static_cast<uint32_t>(layout.fields.size());

            size_t hdrSize = 12 + nFields * sizeof(PcdFieldDesc);
            NioFrame nf;
            nf.type = NioFrameType::POINT;
            nf.format = NioFormat::POINT;
            nf.timestampUs = pointFrameRaw->getTimeStampUs();
            nf.data.resize(hdrSize + static_cast<size_t>(nPts) * layout.srcPointSize);

            uint8_t* dst = nf.data.data();
            std::memcpy(dst, &layout.srcPointSize, 4);
            dst += 4;
            std::memcpy(dst, &nFields, 4);
            dst += 4;
            std::memcpy(dst, &nPts, 4);
            dst += 4;
            std::memcpy(dst, layout.fields.data(), nFields * sizeof(PcdFieldDesc));
            dst += nFields * sizeof(PcdFieldDesc);
            std::memcpy(dst, srcData, static_cast<size_t>(nPts) * layout.srcPointSize);

            nioFs.setFrame(NioFrameType::POINT, std::move(nf));
        }
    }

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
