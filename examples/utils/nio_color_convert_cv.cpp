// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert_cv.cpp — OpenCV-based color conversion implementation.

#include "nio_color_convert_cv.hpp"

#include <iostream>

namespace nio {

cv::Mat frameToBGR(std::shared_ptr<ob::Frame> frame) {
    auto vf = frame->as<ob::VideoFrame>();
    if(!vf) return cv::Mat();

    int w = static_cast<int>(vf->getWidth());
    int h = static_cast<int>(vf->getHeight());
    auto fmt = vf->getFormat();
    const auto *data = vf->getData();

    if(fmt == OB_FORMAT_BGR) {
        return cv::Mat(h, w, CV_8UC3, const_cast<uint8_t *>(data)).clone();
    }
    if(fmt == OB_FORMAT_RGB) {
        cv::Mat rgb(h, w, CV_8UC3, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    if(fmt == OB_FORMAT_RGBA) {
        cv::Mat rgba(h, w, CV_8UC4, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    if(fmt == OB_FORMAT_BGRA) {
        cv::Mat bgra(h, w, CV_8UC4, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    if(fmt == OB_FORMAT_MJPG || fmt == OB_FORMAT_MJPEG) {
        auto dataSize = vf->getDataSize();
        std::vector<uint8_t> buf(data, data + dataSize);
        return cv::imdecode(buf, cv::IMREAD_COLOR);
    }
    if(fmt == OB_FORMAT_YUYV) {
        cv::Mat yuyv(h, w, CV_8UC2, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        return bgr;
    }
    if(fmt == OB_FORMAT_UYVY) {
        cv::Mat uyvy(h, w, CV_8UC2, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
        return bgr;
    }
    if(fmt == OB_FORMAT_NV12) {
        cv::Mat nv12(h + h / 2, w, CV_8UC1, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        return bgr;
    }
    if(fmt == OB_FORMAT_NV21) {
        cv::Mat nv21(h + h / 2, w, CV_8UC1, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);
        return bgr;
    }
    if(fmt == OB_FORMAT_I420) {
        cv::Mat i420(h + h / 2, w, CV_8UC1, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);
        return bgr;
    }
    if(fmt == OB_FORMAT_Y8) {
        cv::Mat y8(h, w, CV_8UC1, const_cast<uint8_t *>(data));
        cv::Mat bgr;
        cv::cvtColor(y8, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    std::cerr << "Unsupported display format: " << fmt << std::endl;
    return cv::Mat();
}

cv::Mat colorizeDepth(std::shared_ptr<ob::Frame> depthFrame,
                      float depthScale, float depthMinM, float depthMaxM) {
    auto vf = depthFrame->as<ob::VideoFrame>();
    if(!vf) return cv::Mat();

    int w = static_cast<int>(vf->getWidth());
    int h = static_cast<int>(vf->getHeight());
    auto fmt = vf->getFormat();
    const auto *data = vf->getData();

    if(fmt != OB_FORMAT_Y16 || !data) {
        return cv::Mat::zeros(h, w, CV_8UC3);
    }

    cv::Mat raw(h, w, CV_16UC1, const_cast<uint8_t *>(data));
    cv::Mat flt;
    raw.convertTo(flt, CV_32F, depthScale * 0.001f);

    float range = depthMaxM - depthMinM;
    if(range <= 0.0f) range = 1.0f;
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

} // namespace nio
