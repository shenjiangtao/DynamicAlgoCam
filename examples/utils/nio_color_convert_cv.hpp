// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert_cv.hpp — OpenCV-based color conversion: frameToBGR,
// colorizeDepth. Only compiled when OpenCV is available.

#pragma once

#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>
#include <memory>

namespace nio {

cv::Mat frameToBGR(std::shared_ptr<ob::Frame> frame);

cv::Mat colorizeDepth(std::shared_ptr<ob::Frame> depthFrame,
                      float depthScale, float depthMinM, float depthMaxM);

} // namespace nio
