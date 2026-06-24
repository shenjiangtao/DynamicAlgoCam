// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_color_convert_cv.hpp — OpenCV-based color conversion: frameToBGR,
// colorizeDepth. Only compiled when OpenCV is available.

#pragma once

#include <opencv2/opencv.hpp>
#include "nio_frame.hpp"
#include "nio_types.hpp"

namespace nio {

cv::Mat frameToBGR(const NioFrame& frame);

cv::Mat colorizeDepth(const NioFrame& depthFrame,
                       float depthMinM, float depthMaxM);

} // namespace nio
