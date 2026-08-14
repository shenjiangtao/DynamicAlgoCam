// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.

#pragma once
#include <cmath>
#include <condition_variable>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "dynalgo_frame.hpp"
#include "dynalgo_types.hpp"
#include "utils.hpp"

namespace dynalgo {

typedef enum {
    ARRANGE_SINGLE,
    ARRANGE_ONE_ROW,
    ARRANGE_ONE_COLUMN,
    ARRANGE_GRID,
    ARRANGE_OVERLAY
} ArrangeMode;

class CVWindow
{
public:
    CVWindow(std::string name, uint32_t width = 1280, uint32_t height = 720, ArrangeMode arrangeMode = ARRANGE_SINGLE);
    ~CVWindow() noexcept;

    bool run();
    void close();
    void reset();

    void pushFramesToView(std::vector<const DynalgoFrame*> frames, int groupId = 0);
    void pushFramesToView(const DynalgoFrame* currentFrame, int groupId = 0);

    void setShowInfo(bool show);
    void setShowSyncTimeInfo(bool show);
    void setAlpha(float alpha);
    void resize(int width, int height);
    void setKeyPressedCallback(std::function<void(int)> callback);
    void setKeyPrompt(const std::string& prompt);
    void addLog(const std::string& log);
    void destroyWindow();

private:
    void processFrames();
    void arrangeFrames();
    cv::Mat visualize(const DynalgoFrame* frame);
    void drawInfo(cv::Mat& imageMat, const DynalgoFrame* frame);
    cv::Mat resizeMatKeepAspectRatio(const cv::Mat& mat, int width, int height);

private:
    std::string name_;
    ArrangeMode arrangeMode_;
    uint32_t width_;
    uint32_t height_;
    bool closed_;
    bool showInfo_;
    bool showSyncTimeInfo_;
    bool isWindowDestroyed_;
    float alpha_;

    std::thread processThread_;
    std::map<int, std::vector<const DynalgoFrame*>> srcFrameGroups_;
    std::mutex srcFrameGroupsMtx_;
    std::condition_variable srcFrameGroupsCv_;

    using StreamsMatMap = std::map<int, std::pair<const DynalgoFrame*, cv::Mat>>;
    StreamsMatMap matGroups_;
    std::mutex renderMatsMtx_;
    cv::Mat renderMat_;

    std::string prompt_;
    bool showPrompt_;
    uint64 winCreatedTime_;

    std::string log_;
    uint64 logCreatedTime_;

    std::function<void(int)> keyPressedCallback_;
};

} // namespace dynalgo
