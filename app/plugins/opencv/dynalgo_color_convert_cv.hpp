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

// [函数: frameToBGR / Function: frameToBGR]
// 中文: 将 DynalgoFrame 转换为 OpenCV BGR 格式 Mat。支持多种输入格式（BGR, RGB, RGBA, BGRA, MJPEG, YUYV, UYVY, NV12, NV21, I420, Y8）。
// English: Convert DynalgoFrame to OpenCV BGR Mat. Supports multiple input formats (BGR, RGB, RGBA, BGRA, MJPEG, YUYV, UYVY, NV12, NV21, I420, Y8).
cv::Mat frameToBGR(const DynalgoFrame& frame);

// [函数: colorizeDepth / Function: colorizeDepth]
// 中文: 将深度帧 (Y16) 伪彩色化为 BGR 图像。使用 JET 色图，无效像素 (值为0) 显示为黑色。
// English: Colorize depth frame (Y16) to pseudo-colored BGR image. Uses JET colormap, invalid pixels (value=0) shown as black.
cv::Mat colorizeDepth(const DynalgoFrame& depthFrame, float depthMinM, float depthMaxM);

} // namespace dynalgo
