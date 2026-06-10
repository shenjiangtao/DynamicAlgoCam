// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert_cv.hpp — OpenCV-based color conversion: frameToBGR,
// colorizeDepth. Only compiled when OpenCV is available.
//
// frameToBGR: converts an ob::Frame (any pixel format) to a cv::Mat BGR
// image using cv::cvtColor / cv::imdecode.  Used for debug visualization.
//
// colorizeDepth: applies JET colormap to Y16 depth data with configurable
// min/max clip range.  Invalid depth (raw==0) pixels are rendered black.

#pragma once

#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>
#include <memory>

namespace nio {

cv::Mat frameToBGR(std::shared_ptr<ob::Frame> frame);

cv::Mat colorizeDepth(std::shared_ptr<ob::Frame> depthFrame,
                      float depthScale, float depthMinM, float depthMaxM);

} // namespace nio
