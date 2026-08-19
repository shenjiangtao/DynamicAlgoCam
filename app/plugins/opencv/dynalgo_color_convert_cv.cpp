// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_color_convert_cv.cpp — OpenCV-based color conversion implementation.
//
// frameToBGR: dispatches on DynalgoFormat -> cv::cvtColor or cv::imdecode.
//   Supports: BGR, RGB, RGBA, BGRA, MJPEG, YUYV, UYVY, NV12, NV21, I420, Y8.
//
// colorizeDepth: Y16 -> float (meters) -> normalize -> JET colormap -> BGR.
//   Zero-valued pixels (invalid/no-signal) are masked to black.

#include "dynalgo_color_convert_cv.hpp"
#include "dynalgo_log.hpp"

#include <iostream>

namespace dynalgo {

// [函数: frameToBGR / Function: frameToBGR]
// 中文: 根据帧格式分派转换逻辑：直接拷贝、cvtColor 颜色空间转换、或 imdecode 解码 MJPEG。
// English: Dispatch conversion based on frame format: direct copy, cvtColor color space conversion, or imdecode for MJPEG.
cv::Mat frameToBGR(const DynalgoFrame& frame) {
    int w = frame.width;
    int h = frame.height;
    auto fmt = frame.format;
    const auto* data = frame.rawData();

    if (!data || w <= 0 || h <= 0)
        return cv::Mat();

    if (fmt == DynalgoFormat::BGR) {
        return cv::Mat(h, w, CV_8UC3, const_cast<uint8_t*>(data)).clone();
    }
    if (fmt == DynalgoFormat::RGB || fmt == DynalgoFormat::RGB888) {
        cv::Mat rgb(h, w, CV_8UC3, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    if (fmt == DynalgoFormat::RGBA) {
        cv::Mat rgba(h, w, CV_8UC4, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    if (fmt == DynalgoFormat::BGRA) {
        cv::Mat bgra(h, w, CV_8UC4, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    if (fmt == DynalgoFormat::MJPG || fmt == DynalgoFormat::MJPEG) {
        auto dataSize = frame.dataSize();
        std::vector<uint8_t> buf(data, data + dataSize);
        return cv::imdecode(buf, cv::IMREAD_COLOR);
    }
    if (fmt == DynalgoFormat::YUYV) {
        cv::Mat yuyv(h, w, CV_8UC2, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        return bgr;
    }
    if (fmt == DynalgoFormat::UYVY) {
        cv::Mat uyvy(h, w, CV_8UC2, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
        return bgr;
    }
    if (fmt == DynalgoFormat::NV12) {
        cv::Mat nv12(h + h / 2, w, CV_8UC1, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        return bgr;
    }
    if (fmt == DynalgoFormat::NV21) {
        cv::Mat nv21(h + h / 2, w, CV_8UC1, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);
        return bgr;
    }
    if (fmt == DynalgoFormat::I420) {
        cv::Mat i420(h + h / 2, w, CV_8UC1, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);
        return bgr;
    }
    if (fmt == DynalgoFormat::Y8) {
        cv::Mat y8(h, w, CV_8UC1, const_cast<uint8_t*>(data));
        cv::Mat bgr;
        cv::cvtColor(y8, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    DYNALGO_LOG_WARN_S("Unsupported display format: " << nioFormatToStr(fmt));
    return cv::Mat();
}

// [函数: colorizeDepth / Function: colorizeDepth]
// 中文: Y16 深度转浮点米 -> 归一化到 [0,1] -> 应用 JET 色图 -> 无效像素置黑。
// English: Y16 depth to float meters -> normalize to [0,1] -> apply JET colormap -> invalid pixels to black.
cv::Mat colorizeDepth(const DynalgoFrame& depthFrame, float depthMinM, float depthMaxM) {
    int w = depthFrame.width;
    int h = depthFrame.height;
    auto fmt = depthFrame.format;
    const auto* data = depthFrame.rawData();
    float depthScale = depthFrame.depthScale;

    if (fmt != DynalgoFormat::Y16 || !data) {
        return cv::Mat::zeros(h, w, CV_8UC3);
    }

    cv::Mat raw(h, w, CV_16UC1, const_cast<uint8_t*>(data));
    cv::Mat flt;
    raw.convertTo(flt, CV_32F, depthScale * 0.001f);

    float range = depthMaxM - depthMinM;
    if (range <= 0.0f)
        range = 1.0f;
    cv::Mat norm = (flt - depthMinM) / range;
    cv::Mat mask;
    cv::compare(raw, 0, mask, cv::CMP_EQ);
    norm.setTo(0, mask);
    cv::threshold(norm, norm, 1.0, 1.0, cv::THRESH_TRUNC);
    norm.setTo(0, norm < 0);

    cv::Mat normU8;
    norm.convertTo(normU8, CV_8UC1, 255.0);
    normU8.setTo(0, mask);

    cv::Mat colorized;
    cv::applyColorMap(normU8, colorized, cv::COLORMAP_JET);
    colorized.setTo(cv::Scalar(0, 0, 0), mask);

    return colorized;
}

} // namespace dynalgo
