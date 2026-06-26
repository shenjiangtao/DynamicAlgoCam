// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_rs_frame_adapter.hpp — RS-AC1 PointCloudMsg + ImageData → NioFrame conversion.
//
// Provides rsDepthToNioFrame() (synthetic 2D depth map from point cloud),
// rsImageToNioFrame() (image data → NioFrame),
// and rsPointToNioFrame() (raw point cloud → NioFrame for PCD recording).

#pragma once

#include "nio_frame.hpp"
#include "nio_rs_adapter.hpp"

#include <rs_driver/msg/image_data_msg.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

#include <cmath>
#include <cstring>

namespace nio {
// RS-AC1 depth grid dimensions (from decoder_RSAC1.hpp constants)
static constexpr int RS_AC1_DEPTH_WIDTH = 96;
static constexpr int RS_AC1_DEPTH_HEIGHT = 288;
// 5mm per uint16 unit (distance / 0.005)
static constexpr float RS_AC1_DEPTH_SCALE = 5.0f;

// Convert RS-AC1 PointCloudMsg → NioFrame (synthetic Y16 depth map).
// Each point[i] maps to pixel(col=i%96, row=i/96).
// Invalid points (NaN or <0.2m) get depth=0.
inline NioFrame rsDepthToNioFrame(const std::shared_ptr<::PointCloudT<::PointXYZIRT>>& cloud) {
    NioFrame f;
    f.type = NioFrameType::DEPTH;
    f.format = NioFormat::Y16;
    f.width = RS_AC1_DEPTH_WIDTH;
    f.height = RS_AC1_DEPTH_HEIGHT;
    f.depthScale = RS_AC1_DEPTH_SCALE;
    f.timestampUs = static_cast<uint64_t>(cloud->timestamp * 1e6);

    size_t pixelCount = static_cast<size_t>(RS_AC1_DEPTH_WIDTH) * RS_AC1_DEPTH_HEIGHT;
    f.data.resize(pixelCount * 2);
    auto* dst = reinterpret_cast<uint16_t*>(f.data.data());

    size_t nPts = cloud->points.size();
    for (size_t i = 0; i < pixelCount; ++i) {
        if (i < nPts) {
            const auto& pt = cloud->points[i];
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                dst[i] = 0;
                continue;
            }
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (dist < 0.2f) {
                dst[i] = 0;
            } else {
                uint16_t raw = static_cast<uint16_t>(dist / 0.005f);
                dst[i] = (raw > 40000) ? 0 : raw;
            }
        } else {
            dst[i] = 0;
        }
    }

    return f;
}

// Convert RS-AC1 PointCloudMsg → NioFrame (raw POINT data for PCD recording).
// Binary layout: [4 bytes: pointCount (uint32)] [pointCount * elementSize bytes: packed XYZIRT]
// Each element: float x(4) + float y(4) + float z(4) + uint8 intensity(1) + uint16 ring(2) + double timestamp(8) = 23
// bytes
inline NioFrame rsPointToNioFrame(const std::shared_ptr<::PointCloudT<::PointXYZIRT>>& cloud) {
    NioFrame f;
    f.type = NioFrameType::POINT;
    f.format = NioFormat::POINT;
    f.timestampUs = static_cast<uint64_t>(cloud->timestamp * 1e6);

    uint32_t nPts = static_cast<uint32_t>(cloud->points.size());
    constexpr size_t elemSize = 4 + 4 + 4 + 1 + 2 + 8; // 23 bytes per point
    size_t dataSize = sizeof(uint32_t) + static_cast<size_t>(nPts) * elemSize;
    f.data.resize(dataSize);

    uint8_t* dst = f.data.data();
    std::memcpy(dst, &nPts, sizeof(uint32_t));
    dst += sizeof(uint32_t);

    for (uint32_t i = 0; i < nPts; ++i) {
        const auto& pt = cloud->points[i];
        std::memcpy(dst, &pt.x, 4);
        dst += 4;
        std::memcpy(dst, &pt.y, 4);
        dst += 4;
        std::memcpy(dst, &pt.z, 4);
        dst += 4;
        std::memcpy(dst, &pt.intensity, 1);
        dst += 1;
        std::memcpy(dst, &pt.ring, 2);
        dst += 2;
        std::memcpy(dst, &pt.timestamp, 8);
        dst += 8;
    }

    return f;
}

// Convert RS-AC1 ImageData → NioFrame (color frame).
inline NioFrame rsImageToNioFrame(const std::shared_ptr<robosense::lidar::ImageData>& img) {
    NioFrame f;
    f.type = NioFrameType::COLOR;
    f.format = rsFrameFormatToNio(img->frame_format);
    f.width = static_cast<int>(img->width);
    f.height = static_cast<int>(img->height);
    f.timestampUs = static_cast<uint64_t>(img->timestamp * 1e6);

    if (img->data && img->data_bytes > 0) {
        f.data.assign(img->data.get(), img->data.get() + img->data_bytes);
    }

    return f;
}

} // namespace nio
