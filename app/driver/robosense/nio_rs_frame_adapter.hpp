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

    size_t pixelCount = static_cast<size_t>(f.width) * f.height;
    f.data.resize(pixelCount * 2);
    auto* dst = reinterpret_cast<uint16_t*>(f.data.data());

    size_t nPts = cloud->points.size();
    // Upsample low‑resolution depth grid to the full frame size (nearest‑neighbor).
    const size_t scaleW = f.width / RS_AC1_DEPTH_WIDTH; // 1920/96 = 20
    const size_t scaleH = f.height / RS_AC1_DEPTH_HEIGHT; // 1080/288 = 3 (integer floor)
    for (size_t rowLow = 0; rowLow < RS_AC1_DEPTH_HEIGHT; ++rowLow) {
        for (size_t colLow = 0; colLow < RS_AC1_DEPTH_WIDTH; ++colLow) {
            size_t srcIdx = rowLow * RS_AC1_DEPTH_WIDTH + colLow;
            uint16_t depthVal = 0;
            if (srcIdx < nPts) {
                const auto& pt = cloud->points[srcIdx];
                if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                    if (dist >= 0.2f) {
                        uint16_t raw = static_cast<uint16_t>(dist / 0.005f);
                        depthVal = (raw > 40000) ? 0 : raw;
                    }
                }
            }
            // Write the value into the upsampled block (invert vertically).
            for (size_t dy = 0; dy < scaleH; ++dy) {
                size_t targetRow = (RS_AC1_DEPTH_HEIGHT - 1 - rowLow) * scaleH + dy; // vertical flip
                for (size_t dx = 0; dx < scaleW; ++dx) {
                    size_t targetCol = colLow * scaleW + dx;
                    size_t targetIdx = targetRow * f.width + targetCol;
                    dst[targetIdx] = depthVal;
                }
            }
        }
    }

    return f;
}

// Convert RS-AC1 PointCloudMsg → NioFrame (raw POINT data for PCD recording).
// Wire layout (see PcdLayout in nio_types.hpp):
//   [12B header: srcPointSize(4) + numFields(4) + pointCount(4)]
//   [numFields * 24B PcdFieldDesc entries]
//   [pointCount * 24B packed PointXYZIRT data]
// The PcdFieldDesc entries describe field offsets within the 24-byte
// PointXYZIRT struct (which has 1 byte padding after intensity).
inline NioFrame rsPointToNioFrame(const std::shared_ptr<::PointCloudT<::PointXYZIRT>>& cloud) {
    NioFrame f;
    f.type = NioFrameType::POINT;
    f.format = NioFormat::POINT;
    f.timestampUs = static_cast<uint64_t>(cloud->timestamp * 1e6);

    uint32_t nPts = static_cast<uint32_t>(cloud->points.size());
    PcdLayout layout = PcdLayout::rsAc1();
    constexpr size_t srcPtSize = 24; // sizeof(PointXYZIRT)

    // Wire header (serialized by PcdLayout), minus pointCount which we write ourselves
    size_t hdrSize = 12 + layout.fields.size() * sizeof(PcdFieldDesc);
    size_t dataSize = hdrSize + static_cast<size_t>(nPts) * srcPtSize;
    f.data.resize(dataSize);

    uint8_t* dst = f.data.data();
    // Write header: srcPointSize(4) + numFields(4) + pointCount(4)
    std::memcpy(dst, &layout.srcPointSize, 4);
    dst += 4;
    uint32_t nFields = static_cast<uint32_t>(layout.fields.size());
    std::memcpy(dst, &nFields, 4);
    dst += 4;
    std::memcpy(dst, &nPts, 4);
    dst += 4;
    // Write field descriptors
    if (!layout.fields.empty())
        std::memcpy(dst, layout.fields.data(), nFields * sizeof(PcdFieldDesc));
    dst += nFields * sizeof(PcdFieldDesc);

    // Bulk-memcpy the entire points array — PointXYZIRT is 24 bytes with 1 byte padding
    if (nPts > 0 && !cloud->points.empty()) {
        std::memcpy(dst, cloud->points.data(), static_cast<size_t>(nPts) * srcPtSize);
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
