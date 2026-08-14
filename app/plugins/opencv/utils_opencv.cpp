// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.

#include "utils_opencv.hpp"
#include "dynalgo_types.hpp"
#include "dynalgo_log.hpp"
#include "utils.hpp"

#if defined(__has_include)
#if __has_include(<opencv2/core/utils/logger.hpp>)
#include <opencv2/core/utils/logger.hpp>
#define TO_DISABLE_OPENCV_LOG
#endif
#endif

namespace dynalgo {

const std::string defaultKeyMapPrompt = "'Esc': Exit Window, '?': Show Key Map";
CVWindow::CVWindow(std::string name, uint32_t width, uint32_t height, ArrangeMode arrangeMode)
: name_(std::move(name))
, arrangeMode_(arrangeMode)
, width_(width)
, height_(height)
, closed_(false)
, showInfo_(true)
, showSyncTimeInfo_(false)
, isWindowDestroyed_(false)
, alpha_(0.6f)
, showPrompt_(false) {

#if defined(TO_DISABLE_OPENCV_LOG)
    cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
#endif

    prompt_ = defaultKeyMapPrompt;

    cv::namedWindow(name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(name_, width_, height_);

    renderMat_ = cv::Mat::zeros(height_, width_, CV_8UC3);
    cv::putText(renderMat_, "Waiting for streams...", cv::Point(8, 16), cv::FONT_HERSHEY_DUPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::imshow(name_, renderMat_);

    processThread_ = std::thread(&CVWindow::processFrames, this);

    winCreatedTime_ = dynalgo::getNowTimesMs();
}

CVWindow::~CVWindow() noexcept {
    close();
    destroyWindow();
}

void CVWindow::setKeyPressedCallback(std::function<void(int)> callback) {
    keyPressedCallback_ = callback;
}

bool CVWindow::run() {

    {
        std::lock_guard<std::mutex> lock(renderMatsMtx_);
        cv::imshow(name_, renderMat_);
    }

    int key = cv::waitKey(1);
    if (key != -1) {
        if (key == ESC_KEY) {
            closed_ = true;
            srcFrameGroupsCv_.notify_all();
        } else if (key == '1') {
            arrangeMode_ = ARRANGE_SINGLE;
            addLog("Switch to SINGLE arrange mode");
        } else if (key == '2') {
            arrangeMode_ = ARRANGE_ONE_ROW;
            addLog("Switch to ONE_ROW arrange mode");
        } else if (key == '3') {
            arrangeMode_ = ARRANGE_ONE_COLUMN;
            addLog("Switch to ONE_COLUMN arrange mode");
        } else if (key == '4') {
            arrangeMode_ = ARRANGE_GRID;
            addLog("Switch to GRID arrange mode");
        } else if (key == '5') {
            arrangeMode_ = ARRANGE_OVERLAY;
            addLog("Switch to OVERLAY arrange mode");
        } else if (key == '?' || key == '/') {
            showPrompt_ = !showPrompt_;
        } else if (key == '+' || key == '=') {
            alpha_ += 0.1f;
            if (alpha_ > 1) {
                alpha_ = 1;
            }
            addLog("Adjust alpha to " + dynalgo::toString(alpha_, 1) + " (Only valid in OVERLAY arrange mode)");
        } else if (key == '-' || key == '_') {
            alpha_ -= 0.1f;
            if (alpha_ < 0) {
                alpha_ = 0;
            }
            addLog("Adjust alpha to " + dynalgo::toString(alpha_, 1) + " (Only valid in OVERLAY arrange mode)");
        }
        if (keyPressedCallback_) {
            keyPressedCallback_(key);
        }
    }
    return !closed_;
}

void CVWindow::close() {
    {
        std::lock_guard<std::mutex> lock(renderMatsMtx_);
        closed_ = true;
        srcFrameGroupsCv_.notify_all();
    }

    if (processThread_.joinable()) {
        processThread_.join();
    }

    matGroups_.clear();
    srcFrameGroups_.clear();
}

void CVWindow::destroyWindow() {
    if (!isWindowDestroyed_) {
        cv::destroyWindow(name_);
        cv::waitKey(1);
        isWindowDestroyed_ = true;
    } else {
        DYNALGO_LOG_WARN_S("CVWindows has been destroyed!");
    }
}

void CVWindow::reset() {
    close();

    closed_ = false;
    processThread_ = std::thread(&CVWindow::processFrames, this);
}

void CVWindow::resize(int width, int height) {
    width_ = width;
    height_ = height;
    cv::resizeWindow(name_, width_, height_);
}

void CVWindow::setKeyPrompt(const std::string& prompt) {
    prompt_ = defaultKeyMapPrompt + ", " + prompt;
}

void CVWindow::addLog(const std::string& log) {
    log_ = log;
    logCreatedTime_ = dynalgo::getNowTimesMs();
}

void CVWindow::pushFramesToView(std::vector<const DynalgoFrame*> frames, int groupId) {
    if (frames.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lk(srcFrameGroupsMtx_);
    srcFrameGroups_[groupId] = frames;
    srcFrameGroupsCv_.notify_one();
}

void CVWindow::pushFramesToView(const DynalgoFrame* currentFrame, int groupId) {
    if (!currentFrame)
        return;
    pushFramesToView(std::vector<const DynalgoFrame*>{ currentFrame }, groupId);
}

void CVWindow::setShowInfo(bool show) {
    showInfo_ = show;
}

void CVWindow::setShowSyncTimeInfo(bool show) {
    showSyncTimeInfo_ = show;
}

void CVWindow::setAlpha(float alpha) {
    alpha_ = alpha;
    if (alpha_ < 0) {
        alpha_ = 0;
    } else if (alpha_ > 1) {
        alpha_ = 1;
    }
}

void CVWindow::processFrames() {
    std::map<int, std::vector<const DynalgoFrame*>> frameGroups;
    while (!closed_) {
        if (closed_) {
            break;
        }
        {
            std::unique_lock<std::mutex> lk(srcFrameGroupsMtx_);
            srcFrameGroupsCv_.wait(lk);
            frameGroups = srcFrameGroups_;
        }

        if (frameGroups.empty()) {
            continue;
        }

        for (const auto& framesItem : frameGroups) {
            int groupId = framesItem.first;
            const auto& frames = framesItem.second;
            for (const auto* frame : frames) {
                auto rstMat = visualize(frame);
                if (!rstMat.empty()) {
                    int uid = groupId * static_cast<int>(DynalgoFrameType::COUNT) + static_cast<int>(frame->type);
                    matGroups_[uid] = { frame, rstMat };
                }
            }
        }

        if (matGroups_.empty()) {
            continue;
        }

        arrangeFrames();
    }
}

void CVWindow::arrangeFrames() {
    cv::Mat renderMat;
    try {
        if (arrangeMode_ == ARRANGE_SINGLE || matGroups_.size() == 1) {
            auto& mat = matGroups_.begin()->second.second;
            renderMat = resizeMatKeepAspectRatio(mat, width_, height_);
        } else if (arrangeMode_ == ARRANGE_ONE_ROW) {
            for (auto& item : matGroups_) {
                auto& mat = item.second.second;
                cv::Mat resizeMat =
                    resizeMatKeepAspectRatio(mat, static_cast<int>(width_ / matGroups_.size()), height_);
                if (renderMat.dims > 0 && renderMat.cols > 0 && renderMat.rows > 0) {
                    cv::hconcat(renderMat, resizeMat, renderMat);
                } else {
                    renderMat = resizeMat;
                }
            }
        } else if (arrangeMode_ == ARRANGE_ONE_COLUMN) {
            for (auto& item : matGroups_) {
                auto& mat = item.second.second;
                cv::Mat resizeMat =
                    resizeMatKeepAspectRatio(mat, width_, static_cast<int>(height_ / matGroups_.size()));
                if (renderMat.dims > 0 && renderMat.cols > 0 && renderMat.rows > 0) {
                    cv::vconcat(renderMat, resizeMat, renderMat);
                } else {
                    renderMat = resizeMat;
                }
            }
        } else if (arrangeMode_ == ARRANGE_GRID) {
            int count = static_cast<int>(matGroups_.size());
            int idealSide = static_cast<int>(std::sqrt(count));
            int rows = idealSide;
            int cols = idealSide;
            while (rows * cols < count) {
                cols++;
                if (rows * cols < count) {
                    rows++;
                }
            }

            std::vector<cv::Mat> gridImages;
            auto it = matGroups_.begin();
            for (int i = 0; i < rows; i++) {
                std::vector<cv::Mat> rowImages;
                for (int j = 0; j < cols; j++) {
                    int index = i * cols + j;
                    cv::Mat resizeMat;
                    if (index < count) {
                        auto mat = it->second.second;
                        resizeMat = resizeMatKeepAspectRatio(mat, width_ / cols, height_ / rows);
                        it++;
                    } else {
                        resizeMat = cv::Mat::zeros(height_ / rows, width_ / cols, CV_8UC3);
                    }
                    rowImages.push_back(resizeMat);
                }
                cv::Mat lineMat;
                cv::hconcat(rowImages, lineMat);
                gridImages.push_back(lineMat);
            }

            cv::vconcat(gridImages, renderMat);
        } else if (arrangeMode_ == ARRANGE_OVERLAY && matGroups_.size() >= 2) {
            cv::Mat overlayMat;
            const auto& mat1 = matGroups_.begin()->second.second;
            const auto& mat2 = matGroups_.rbegin()->second.second;
            renderMat = resizeMatKeepAspectRatio(mat1, width_, height_);
            overlayMat = resizeMatKeepAspectRatio(mat2, width_, height_);

            float alpha = alpha_;
            for (int i = 0; i < renderMat.rows; i++) {
                for (int j = 0; j < renderMat.cols; j++) {
                    cv::Vec3b& outRgb = renderMat.at<cv::Vec3b>(i, j);
                    cv::Vec3b& overlayRgb = overlayMat.at<cv::Vec3b>(i, j);

                    outRgb[0] = (uint8_t)(outRgb[0] * (1.0f - alpha) + overlayRgb[0] * alpha);
                    outRgb[1] = (uint8_t)(outRgb[1] * (1.0f - alpha) + overlayRgb[1] * alpha);
                    outRgb[2] = (uint8_t)(outRgb[2] * (1.0f - alpha) + overlayRgb[2] * alpha);
                }
            }
        }
    } catch (std::exception& e) {
        DYNALGO_LOG_ERROR_S("CVWindow exception: " << e.what());
    }

    if (renderMat.empty()) {
        return;
    }

    if (showPrompt_ || dynalgo::getNowTimesMs() - winCreatedTime_ < 5000) {
        cv::putText(renderMat, prompt_, cv::Point(8, 16), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
                    cv::LINE_AA);
    }

    if (!log_.empty() && dynalgo::getNowTimesMs() - logCreatedTime_ < 3000) {
        cv::putText(renderMat, log_, cv::Point(8, height_ - 16), cv::FONT_HERSHEY_DUPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    std::lock_guard<std::mutex> lock(renderMatsMtx_);
    renderMat_ = renderMat;
}

cv::Mat CVWindow::visualize(const DynalgoFrame* frame) {
    if (frame == nullptr) {
        return cv::Mat();
    }

    cv::Mat rstMat;
    int w = frame->width;
    int h = frame->height;
    const auto* data = frame->rawData();

    switch (frame->type) {
    case DynalgoFrameType::COLOR:
    case DynalgoFrameType::COLOR_LEFT:
    case DynalgoFrameType::COLOR_RIGHT:
        {
            switch (frame->format) {
            case DynalgoFormat::MJPG:
            case DynalgoFormat::MJPEG:
                {
                    cv::Mat rawMat(1, frame->dataSize(), CV_8UC1, const_cast<uint8_t*>(data));
                    rstMat = cv::imdecode(rawMat, 1);
                }
                break;
            case DynalgoFormat::NV21:
                {
                    cv::Mat rawMat(h * 3 / 2, w, CV_8UC1, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_NV21);
                }
                break;
            case DynalgoFormat::YUYV:
            case DynalgoFormat::YUY2:
                {
                    cv::Mat rawMat(h, w, CV_8UC2, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_YUY2);
                }
                break;
            case DynalgoFormat::BGR:
                {
                    cv::Mat rawMat(h, w, CV_8UC3, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_BGR2RGB);
                }
                break;
            case DynalgoFormat::RGB:
            case DynalgoFormat::RGB888:
                {
                    cv::Mat rawMat(h, w, CV_8UC3, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
                }
                break;
            case DynalgoFormat::RGBA:
                {
                    cv::Mat rawMat(h, w, CV_8UC4, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_RGBA2BGR);
                }
                break;
            case DynalgoFormat::BGRA:
                {
                    cv::Mat rawMat(h, w, CV_8UC4, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_BGRA2RGB);
                }
                break;
            case DynalgoFormat::UYVY:
                {
                    cv::Mat rawMat(h, w, CV_8UC2, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_UYVY);
                }
                break;
            case DynalgoFormat::I420:
                {
                    cv::Mat rawMat(h * 3 / 2, w, CV_8UC1, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2BGR_I420);
                }
                break;
            case DynalgoFormat::Y8:
                {
                    cv::Mat rawMat(h, w, CV_8UC1, const_cast<uint8_t*>(data));
                    cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2BGR);
                }
                break;
            case DynalgoFormat::Y16:
                {
                    cv::Mat rawMat(h, w, CV_16UC1, const_cast<uint8_t*>(data));
                    cv::Mat gray8;
                    rawMat.convertTo(gray8, CV_8UC1, 255.0 / 65535.0);
                    cv::cvtColor(gray8, rstMat, cv::COLOR_GRAY2BGR);
                }
                break;
            default:
                break;
            }
            if (showSyncTimeInfo_ && !rstMat.empty()) {
                drawInfo(rstMat, frame);
            }
        }
        break;
    case DynalgoFrameType::DEPTH:
        {
            if (frame->format == DynalgoFormat::Y16) {
                cv::Mat rawMat = cv::Mat(h, w, CV_16UC1, const_cast<uint8_t*>(data));
                float scale = frame->depthScale;

                cv::Mat cvtMat;
                rawMat.convertTo(cvtMat, CV_32F, scale * 0.032f);
                cv::pow(cvtMat, 0.6f, cvtMat);
                cvtMat.convertTo(cvtMat, CV_8UC1, 10);
                cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);
            }
            if (showSyncTimeInfo_ && !rstMat.empty()) {
                drawInfo(rstMat, frame);
            }
        }
        break;
    case DynalgoFrameType::IR:
    case DynalgoFrameType::IR_LEFT:
    case DynalgoFrameType::IR_RIGHT:
        {
            if (frame->format == DynalgoFormat::Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(h, w, CV_16UC1, const_cast<uint8_t*>(data));
                rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
                cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
            } else if (frame->format == DynalgoFormat::Y8) {
                cv::Mat rawMat = cv::Mat(h, w, CV_8UC1, const_cast<uint8_t*>(data));
                cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2RGB);
            } else if (frame->format == DynalgoFormat::MJPG || frame->format == DynalgoFormat::MJPEG) {
                cv::Mat rawMat(1, frame->dataSize(), CV_8UC1, const_cast<uint8_t*>(data));
                rstMat = cv::imdecode(rawMat, 1);
                cv::cvtColor(rstMat, rstMat, cv::COLOR_GRAY2RGB);
            }
            if (showSyncTimeInfo_ && !rstMat.empty()) {
                drawInfo(rstMat, frame);
            }
        }
        break;
    case DynalgoFrameType::CONFIDENCE:
        {
            if (frame->format == DynalgoFormat::Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(h, w, CV_16UC1, const_cast<uint8_t*>(data));
                rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
                cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
            } else if (frame->format == DynalgoFormat::Y8) {
                cv::Mat rawMat = cv::Mat(h, w, CV_8UC1, const_cast<uint8_t*>(data));
                cv::cvtColor(rawMat, rstMat, cv::COLOR_GRAY2RGB);
            }
        }
        break;
    case DynalgoFrameType::ACCEL:
        {
            rstMat = cv::Mat::zeros(320, 300, CV_8UC3);
            std::string str = "Accel:";
            cv::putText(rstMat, str.c_str(), cv::Point(8, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, cv::LINE_AA);
            str = std::string(" timestamp=") + std::to_string(frame->timestampUs) + "us";
            cv::putText(rstMat, str.c_str(), cv::Point(8, 100), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, cv::LINE_AA);
            // IMU samples store x/y/z in data — for accel, interpret as float triples
            if (frame->data.size() >= 3 * sizeof(float)) {
                const float* vals = reinterpret_cast<const float*>(frame->rawData());
                str = std::string(" x=") + std::to_string(vals[0]) + "m/s^2";
                cv::putText(rstMat, str.c_str(), cv::Point(8, 140), cv::FONT_HERSHEY_DUPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                str = std::string(" y=") + std::to_string(vals[1]) + "m/s^2";
                cv::putText(rstMat, str.c_str(), cv::Point(8, 180), cv::FONT_HERSHEY_DUPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                str = std::string(" z=") + std::to_string(vals[2]) + "m/s^2";
                cv::putText(rstMat, str.c_str(), cv::Point(8, 220), cv::FONT_HERSHEY_DUPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
        break;
    case DynalgoFrameType::GYRO:
        {
            rstMat = cv::Mat::zeros(320, 300, CV_8UC3);
            std::string str = "Gyro:";
            cv::putText(rstMat, str.c_str(), cv::Point(8, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, cv::LINE_AA);
            str = std::string(" timestamp=") + std::to_string(frame->timestampUs) + "us";
            cv::putText(rstMat, str.c_str(), cv::Point(8, 100), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, cv::LINE_AA);
            if (frame->data.size() >= 3 * sizeof(float)) {
                const float* vals = reinterpret_cast<const float*>(frame->rawData());
                str = std::string(" x=") + std::to_string(vals[0]) + "rad/s";
                cv::putText(rstMat, str.c_str(), cv::Point(8, 140), cv::FONT_HERSHEY_DUPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                str = std::string(" y=") + std::to_string(vals[1]) + "rad/s";
                cv::putText(rstMat, str.c_str(), cv::Point(8, 180), cv::FONT_HERSHEY_DUPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                str = std::string(" z=") + std::to_string(vals[2]) + "rad/s";
                cv::putText(rstMat, str.c_str(), cv::Point(8, 220), cv::FONT_HERSHEY_DUPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
        break;
    default:
        break;
    }
    return rstMat;
}

void CVWindow::drawInfo(cv::Mat& imageMat, const DynalgoFrame* frame) {
    int baseline = 0;
    cv::Size textSize;
    int padding = 5;

    auto putTextWithBackground = [&](const std::string& text, cv::Point origin) {
        textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);

        cv::rectangle(imageMat, origin + cv::Point(0, baseline),
                      origin + cv::Point(textSize.width, -textSize.height) - cv::Point(0, padding),
                      cv::Scalar(255, 255, 255), cv::FILLED);

        cv::putText(imageMat, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    };

    auto frameType = frame->type;
    auto frameFormat = frame->format;
    switch (frameFormat) {
    case DynalgoFormat::NV21:
        {
            switch (frameType) {
            case DynalgoFrameType::COLOR:
                putTextWithBackground("Color-NV21", cv::Point(8, 16));
                break;
            case DynalgoFrameType::COLOR_LEFT:
                putTextWithBackground("LeftColor-NV21", cv::Point(8, 16));
                break;
            case DynalgoFrameType::COLOR_RIGHT:
                putTextWithBackground("RightColor-NV21", cv::Point(8, 16));
                break;
            default:
                break;
            }
        }
        break;
    case DynalgoFormat::MJPG:
    case DynalgoFormat::MJPEG:
        {
            switch (frameType) {
            case DynalgoFrameType::COLOR:
                putTextWithBackground("Color-MJPG", cv::Point(8, 16));
                break;
            case DynalgoFrameType::COLOR_LEFT:
                putTextWithBackground("LeftColor-MJPG", cv::Point(8, 16));
                break;
            case DynalgoFrameType::COLOR_RIGHT:
                putTextWithBackground("RightColor-MJPG", cv::Point(8, 16));
                break;
            default:
                break;
            }
        }
        break;
    case DynalgoFormat::YUYV:
    case DynalgoFormat::YUY2:
        {
            switch (frameType) {
            case DynalgoFrameType::COLOR:
                putTextWithBackground("Color-YUYV", cv::Point(8, 16));
                break;
            case DynalgoFrameType::COLOR_LEFT:
                putTextWithBackground("LeftColor-YUYV", cv::Point(8, 16));
                break;
            case DynalgoFrameType::COLOR_RIGHT:
                putTextWithBackground("RightColor-YUYV", cv::Point(8, 16));
                break;
            default:
                break;
            }
        }
        break;
    default:
        {
            switch (frameType) {
            case DynalgoFrameType::DEPTH:
                putTextWithBackground("Depth", cv::Point(8, 16));
                break;
            case DynalgoFrameType::IR:
                putTextWithBackground("IR", cv::Point(8, 16));
                break;
            case DynalgoFrameType::IR_LEFT:
                putTextWithBackground("LeftIR", cv::Point(8, 16));
                break;
            case DynalgoFrameType::IR_RIGHT:
                putTextWithBackground("RightIR", cv::Point(8, 16));
                break;
            default:
                break;
            }
        }
        break;
    }

    putTextWithBackground("frame timestamp(us):  " + std::to_string(frame->timestampUs), cv::Point(8, 40));
    putTextWithBackground("width: " + std::to_string(frame->width) + " height: " + std::to_string(frame->height),
                          cv::Point(8, 64));
}

cv::Mat CVWindow::resizeMatKeepAspectRatio(const cv::Mat& mat, int width, int height) {
    auto hScale = static_cast<double>(width) / mat.cols;
    auto vScale = static_cast<double>(height) / mat.rows;
    auto scale = std::min(hScale, vScale);
    auto newWidth = static_cast<int>(mat.cols * scale);
    auto newHeight = static_cast<int>(mat.rows * scale);
    cv::Mat resizeMat;
    cv::resize(mat, resizeMat, cv::Size(newWidth, newHeight));

    if (newWidth == width && newHeight == height) {
        return resizeMat;
    }

    cv::Mat paddedMat;
    if (newWidth < width) {
        auto paddingLeft = (width - newWidth) / 2;
        auto paddingRight = width - newWidth - paddingLeft;
        cv::copyMakeBorder(resizeMat, paddedMat, 0, 0, paddingLeft, paddingRight, cv::BORDER_CONSTANT,
                           cv::Scalar(0, 0, 0));
    }

    if (newHeight < height) {
        auto paddingTop = (height - newHeight) / 2;
        auto paddingBottom = height - newHeight - paddingTop;
        cv::copyMakeBorder(resizeMat, paddedMat, paddingTop, paddingBottom, 0, 0, cv::BORDER_CONSTANT,
                           cv::Scalar(0, 0, 0));
    }
    return paddedMat;
}

} // namespace dynalgo
