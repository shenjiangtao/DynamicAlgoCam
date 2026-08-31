// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// stereo_rectifier.cpp — Stereo rectification and depth computation using OpenCV.

#include "stereo_adapter.hpp"

#include "dynalgo_log.hpp"
#include "utils.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>

namespace dynalgo {

bool StereoRectifier::loadCalibration(const std::string& calibrationFile) {
    cv::FileStorage fs(calibrationFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        DYNALGO_LOG_ERROR_S("StereoRectifier: failed to open calibration file: " << calibrationFile);
        return false;
    }

    // Read camera matrices
    fs["M1"] >> leftCameraMatrix_;
    fs["M2"] >> rightCameraMatrix_;
    fs["D1"] >> leftDistCoeffs_;
    fs["D2"] >> rightDistCoeffs_;
    fs["R"] >> R_;
    fs["T"] >> T_;
    fs["R1"] >> R1_;
    fs["R2"] >> R2_;
    fs["P1"] >> P1_;
    fs["P2"] >> P2_;
    fs["Q"] >> Q_;

    // Read image size
    cv::FileNode sizeNode = fs["imageSize"];
    if (!sizeNode.empty()) {
        sizeNode >> imageSize_;
    } else {
        // Fallback: try to get from camera matrix
        imageSize_ = cv::Size(
            static_cast<int>(leftCameraMatrix_.at<double>(0, 2) * 2),
            static_cast<int>(leftCameraMatrix_.at<double>(1, 2) * 2)
        );
    }

    if (leftCameraMatrix_.empty() || rightCameraMatrix_.empty() ||
        R_.empty() || T_.empty() || R1_.empty() || R2_.empty() ||
        P1_.empty() || P2_.empty() || Q_.empty()) {
        DYNALGO_LOG_ERROR_S("StereoRectifier: missing required calibration matrices");
        return false;
    }

    // Compute rectification maps
    cv::initUndistortRectifyMap(
        leftCameraMatrix_, leftDistCoeffs_, R1_, P1_, imageSize_, CV_32FC1,
        leftMap1_, leftMap2_);
    cv::initUndistortRectifyMap(
        rightCameraMatrix_, rightDistCoeffs_, R2_, P2_, imageSize_, CV_32FC1,
        rightMap1_, rightMap2_);

    // Extract intrinsics from P1, P2
    leftIntrinsic_.fx = static_cast<float>(P1_.at<double>(0, 0));
    leftIntrinsic_.fy = static_cast<float>(P1_.at<double>(1, 1));
    leftIntrinsic_.cx = static_cast<float>(P1_.at<double>(0, 2));
    leftIntrinsic_.cy = static_cast<float>(P1_.at<double>(1, 2));
    leftIntrinsic_.width = imageSize_.width;
    leftIntrinsic_.height = imageSize_.height;

    rightIntrinsic_.fx = static_cast<float>(P2_.at<double>(0, 0));
    rightIntrinsic_.fy = static_cast<float>(P2_.at<double>(1, 1));
    rightIntrinsic_.cx = static_cast<float>(P2_.at<double>(0, 2));
    rightIntrinsic_.cy = static_cast<float>(P2_.at<double>(1, 2));
    rightIntrinsic_.width = imageSize_.width;
    rightIntrinsic_.height = imageSize_.height;

    // Extract baseline from T (translation vector)
    baseline_ = static_cast<float>(cv::norm(T_));

    // Extract extrinsics (R, T)
    cv::Mat R_cv = R_.clone();
    cv::Mat T_cv = T_.clone();
    leftToRight_.r[0] = static_cast<float>(R_cv.at<double>(0, 0));
    leftToRight_.r[1] = static_cast<float>(R_cv.at<double>(0, 1));
    leftToRight_.r[2] = static_cast<float>(R_cv.at<double>(0, 2));
    leftToRight_.r[3] = static_cast<float>(R_cv.at<double>(1, 0));
    leftToRight_.r[4] = static_cast<float>(R_cv.at<double>(1, 1));
    leftToRight_.r[5] = static_cast<float>(R_cv.at<double>(1, 2));
    leftToRight_.r[6] = static_cast<float>(R_cv.at<double>(2, 0));
    leftToRight_.r[7] = static_cast<float>(R_cv.at<double>(2, 1));
    leftToRight_.r[8] = static_cast<float>(R_cv.at<double>(2, 2));
    leftToRight_.t[0] = static_cast<float>(T_cv.at<double>(0));
    leftToRight_.t[1] = static_cast<float>(T_cv.at<double>(1));
    leftToRight_.t[2] = static_cast<float>(T_cv.at<double>(2));

    valid_ = true;
    DYNALGO_LOG_INFO_S("StereoRectifier: loaded calibration from " << calibrationFile
                       << " imageSize=" << imageSize_.width << "x" << imageSize_.height
                       << " baseline=" << baseline_ << "m");
    return true;
}

bool StereoRectifier::rectify(const DynalgoFrame& leftRaw, const DynalgoFrame& rightRaw,
                              DynalgoFrame& leftRect, DynalgoFrame& rightRect) const {
    if (!valid_) {
        DYNALGO_LOG_WARN_S("StereoRectifier::rectify: not calibrated");
        return false;
    }

    // Convert DynalgoFrame to cv::Mat
    cv::Mat leftSrc(leftRaw.height, leftRaw.width, CV_8UC3, const_cast<uint8_t*>(leftRaw.data.data()));
    cv::Mat rightSrc(rightRaw.height, rightRaw.width, CV_8UC3, const_cast<uint8_t*>(rightRaw.data.data()));

    // Rectify
    cv::Mat leftRectified, rightRectified;
    cv::remap(leftSrc, leftRectified, leftMap1_, leftMap2_, cv::INTER_LINEAR);
    cv::remap(rightSrc, rightRectified, rightMap1_, rightMap2_, cv::INTER_LINEAR);

    // Convert back to DynalgoFrame
    leftRect.width = leftRectified.cols;
    leftRect.height = leftRectified.rows;
    leftRect.format = DynalgoFormat::BGR;
    leftRect.data.assign(leftRectified.datastart, leftRectified.dataend);

    rightRect.width = rightRectified.cols;
    rightRect.height = rightRectified.rows;
    rightRect.format = DynalgoFormat::BGR;
    rightRect.data.assign(rightRectified.datastart, rightRectified.dataend);

    return true;
}

bool StereoRectifier::getMaps(
    std::vector<float>& leftMapX, std::vector<float>& leftMapY,
    std::vector<float>& rightMapX, std::vector<float>& rightMapY) const {
    if (!valid_) return false;

    leftMapX.assign(leftMap1_.datastart, leftMap1_.dataend);
    leftMapY.assign(leftMap2_.datastart, leftMap2_.dataend);
    rightMapX.assign(rightMap1_.datastart, rightMap1_.dataend);
    rightMapY.assign(rightMap2_.datastart, rightMap2_.dataend);
    return true;
}

bool StereoRectifier::computeDepth(const DynalgoFrame& leftRect, const DynalgoFrame& rightRect,
                                   DynalgoFrame& disparity, DynalgoFrame& depth) const {
    if (!valid_) {
        DYNALGO_LOG_WARN_S("StereoRectifier::computeDepth: not calibrated");
        return false;
    }

    // Convert to grayscale
    cv::Mat leftGray, rightGray;
    cv::Mat leftSrc(leftRect.height, leftRect.width, CV_8UC3, const_cast<uint8_t*>(leftRect.data.data()));
    cv::Mat rightSrc(rightRect.height, rightRect.width, CV_8UC3, const_cast<uint8_t*>(rightRect.data.data()));

    cv::cvtColor(leftSrc, leftGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightSrc, rightGray, cv::COLOR_BGR2GRAY);

    // Configure SGBM
    int minDisparity = 0;
    int numDisparities = 128;  // Must be divisible by 16
    int blockSize = 5;
    int P1 = 8 * 3 * 5 * 5;   // 8 * channels * blockSize^2
    int P2 = 32 * 3 * 5 * 5;  // 32 * channels * blockSize^2
    int disp12MaxDiff = 1;
    int preFilterCap = 63;
    int uniquenessRatio = 10;
    int speckleWindowSize = 100;
    int speckleRange = 32;
    int mode = cv::StereoSGBM::MODE_SGBM_3WAY;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        minDisparity, numDisparities, blockSize,
        P1, P2, disp12MaxDiff, preFilterCap,
        uniquenessRatio, speckleWindowSize, speckleRange, mode);

    cv::Mat disparity16s;
    sgbm->compute(leftGray, rightGray, disparity16s);

    // Convert to float disparity (disparity16s is 16x actual disparity)
    cv::Mat disparityFloat;
    disparity16s.convertTo(disparityFloat, CV_32F, 1.0 / 16.0);

    // Compute depth using Q matrix: Z = f * B / disparity
    // Using reprojectImageTo3D with Q matrix
    cv::Mat depth32f;
    cv::reprojectImageTo3D(disparityFloat, depth32f, Q_, true, CV_32F);

    // Output disparity (Y16 - 16-bit scaled disparity)
    cv::Mat disparity16u;
    disparity16s.convertTo(disparity16u, CV_16U, 1.0 / 16.0);  // Scale to actual disparity

    // Output frames
    disparity.width = disparity16u.cols;
    disparity.height = disparity16u.rows;
    disparity.format = DynalgoFormat::Y16;
    disparity.data.assign(disparity16u.datastart, disparity16u.dataend);

    // Output depth (uint16 millimeters)
    // Extract Z channel from 3D points (depth32f is 3-channel: X, Y, Z)
    std::vector<cv::Mat> channels;
    cv::split(depth32f, channels);
    cv::Mat depthZ = channels[2];  // Z channel

    // Convert meters to millimeters (uint16)
    cv::Mat depthMM;
    depthZ.convertTo(depthMM, CV_16U, 1000.0);  // meters -> millimeters

    depth.width = depthMM.cols;
    depth.height = depthMM.rows;
    depth.format = DynalgoFormat::Y16;
    depth.data.assign(depthMM.datastart, depthMM.dataend);

    return true;
}

} // namespace dynalgo