// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_rs_frame_adapter.hpp — RS-AC1 PointCloudMsg + ImageData → NioFrame conversion.
//
// Provides rsDepthToNioFrame() (synthetic 2D depth map from point cloud)
// and rsImageToNioFrame() (image data → NioFrame).

#pragma once

#include "nio_frame.hpp"
#include "nio_rs_adapter.hpp"

#ifdef ENABLE_RS_AC1
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/msg/image_data_msg.hpp>
#endif

#include <cmath>
#include <cstring>

namespace nio {

#ifdef ENABLE_RS_AC1

// RS-AC1 depth grid dimensions (from decoder_RSAC1.hpp constants)
static constexpr int RS_AC1_DEPTH_WIDTH  = 96;
static constexpr int RS_AC1_DEPTH_HEIGHT = 288;
// 5mm per uint16 unit (distance / 0.005)
static constexpr float RS_AC1_DEPTH_SCALE = 5.0f;

// Convert RS-AC1 PointCloudMsg → NioFrame (synthetic Y16 depth map).
// Each point[i] maps to pixel(col=i%96, row=i/96).
// Invalid points (NaN or <0.2m) get depth=0.
NioFrame rsDepthToNioFrame(const std::shared_ptr<::PointCloudT<::PointXYZIRT>>& cloud) {
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

// Convert RS-AC1 ImageData → NioFrame (color frame).
NioFrame rsImageToNioFrame(const std::shared_ptr<robosense::lidar::ImageData>& img) {
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

#endif // ENABLE_RS_AC1

} // namespace nio
