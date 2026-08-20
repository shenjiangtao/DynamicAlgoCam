// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_ob_frame_adapter.hpp — Convert Orbbec SDK FrameSet → DynalgoFrameSet.
//
// Called in the SDK video callback (producer boundary).  Copies pixel
// data out of the SDK buffer so downstream consumers are fully
// SDK-agnostic.  For IMU frames, extracts accel/gyro values into
// DynalgoImuSample.

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_ob_adapter.hpp"
#include "dynalgo_log.hpp"

#include <libobsensor/ObSensor.hpp>

namespace dynalgo {

// [函数说明 / Function Description]
// 中文: 将 ob::FrameSet 转换为 DynalgoFrameSet（深拷贝像素数据）
// English: Convert ob::FrameSet to DynalgoFrameSet (deep copy pixel data)
inline DynalgoFrameSet obFrameSetToDynalgo(std::shared_ptr<ob::FrameSet> obFs) {
    DynalgoFrameSet dynalgoFs;

    auto extract = [&](OBFrameType obType, DynalgoFrameType dynalgoType) {
        auto frame = obFs->getFrame(obType);
        if (!frame)
            return;
        DynalgoFrame nf;
        nf.type = dynalgoType;
        nf.format = obFormatToDynalgo(frame->getFormat());
        nf.timestampUs = frame->getTimeStampUs();

        try {
            auto vf = frame->as<ob::VideoFrame>();
            if (vf) {
                nf.width = vf->getWidth();
                nf.height = vf->getHeight();
            }
        } catch (...) {
            DYNALGO_LOG_WARN_S("obFrameSetToDynalgo: VideoFrame cast/size query threw for "
                           << static_cast<int>(dynalgoType));
        }

        // Depth scale / 深度比例
        if (obType == OB_FRAME_DEPTH) {
            try {
                auto df = frame->as<ob::DepthFrame>();
                if (df)
                    nf.depthScale = df->getValueScale();
            } catch (...) {
                DYNALGO_LOG_WARN_S("obFrameSetToDynalgo: DepthFrame getValueScale threw");
            }
        }

        auto* data = frame->getData();
        auto size = frame->getDataSize();
        if (data && size > 0)
            nf.data.assign(data, data + size);

        dynalgoFs.setFrame(dynalgoType, std::move(nf));
    };

    extract(OB_FRAME_COLOR, DynalgoFrameType::COLOR);
    extract(OB_FRAME_DEPTH, DynalgoFrameType::DEPTH);
    extract(OB_FRAME_IR, DynalgoFrameType::IR);
    extract(OB_FRAME_IR_LEFT, DynalgoFrameType::IR_LEFT);
    extract(OB_FRAME_IR_RIGHT, DynalgoFrameType::IR_RIGHT);

    // Point cloud: wrap raw OB_FORMAT_POINT data in self-describing wire format
    // [处理说明 / Processing Note]
    // 中文: 将原始点云数据包装为自描述线缆格式
    // English: Wrap raw point cloud data in self-describing wire format
    auto pointFrameRaw = obFs->getFrame(OB_FRAME_POINTS);
    if (pointFrameRaw) {
        auto* srcData = pointFrameRaw->getData();
        auto srcSize = pointFrameRaw->getDataSize();
        if (srcData && srcSize > 0) {
            PcdLayout layout = PcdLayout::obXyz();
            uint32_t nPts = static_cast<uint32_t>(srcSize / layout.srcPointSize);
            uint32_t nFields = static_cast<uint32_t>(layout.fields.size());

            size_t hdrSize = 12 + nFields * sizeof(PcdFieldDesc);
            DynalgoFrame nf;
            nf.type = DynalgoFrameType::POINT;
            nf.format = DynalgoFormat::POINT;
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

            dynalgoFs.setFrame(DynalgoFrameType::POINT, std::move(nf));
        }
    }

    return dynalgoFs;
}

// [函数说明 / Function Description]
// 中文: 从 ob::FrameSet 提取 IMU 样本为 vector<DynalgoImuSample>
// English: Extract IMU samples from ob::FrameSet to vector<DynalgoImuSample>
inline std::vector<DynalgoImuSample> obImuToDynalgoSamples(std::shared_ptr<ob::FrameSet> obFs) {
    std::vector<DynalgoImuSample> samples;

    auto accelFrameRaw = obFs->getFrame(OB_FRAME_ACCEL);
    if (accelFrameRaw) {
        try {
            auto accel = accelFrameRaw->as<ob::AccelFrame>();
            if (accel) {
                auto val = accel->getValue();
                DynalgoImuSample s;
                s.type = DynalgoFrameType::ACCEL;
                s.timestampUs = accel->getTimeStampUs();
                s.x = val.x;
                s.y = val.y;
                s.z = val.z;
                s.temperature = accel->getTemperature();
                samples.push_back(s);
            }
        } catch (...) {
            DYNALGO_LOG_WARN_S("obImuToDynalgoSamples: AccelFrame parse threw");
        }
    }

    auto gyroFrameRaw = obFs->getFrame(OB_FRAME_GYRO);
    if (gyroFrameRaw) {
        try {
            auto gyro = gyroFrameRaw->as<ob::GyroFrame>();
            if (gyro) {
                auto val = gyro->getValue();
                DynalgoImuSample s;
                s.type = DynalgoFrameType::GYRO;
                s.timestampUs = gyro->getTimeStampUs();
                s.x = val.x;
                s.y = val.y;
                s.z = val.z;
                s.temperature = gyro->getTemperature();
                samples.push_back(s);
            }
        } catch (...) {
            DYNALGO_LOG_WARN_S("obImuToDynalgoSamples: GyroFrame parse threw");
        }
    }

    return samples;
}

} // namespace dynalgo
