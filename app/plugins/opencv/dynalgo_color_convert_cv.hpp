// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_color_convert_cv.hpp — OpenCV-based color conversion: frameToBGR,
// colorizeDepth. Only compiled when OpenCV is available.

#pragma once

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"
#include <opencv2/opencv.hpp>

namespace dynalgo {

cv::Mat frameToBGR(const DynalgoFrame& frame);

cv::Mat colorizeDepth(const DynalgoFrame& depthFrame, float depthMinM, float depthMaxM);

} // namespace dynalgo
