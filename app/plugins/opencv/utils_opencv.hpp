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

// [枚举: ArrangeMode / Enum: ArrangeMode]
// 中文: 窗口布局模式：单帧、单行、单列、网格、叠加。
// English: Window layout modes: single, one row, one column, grid, overlay.
typedef enum {
    ARRANGE_SINGLE,
    ARRANGE_ONE_ROW,
    ARRANGE_ONE_COLUMN,
    ARRANGE_GRID,
    ARRANGE_OVERLAY
} ArrangeMode;

// [类: CVWindow / Class: CVWindow]
// 中文: 基于 OpenCV 的多流可视化窗口。支持多种布局模式、按键控制、信息叠加、日志显示。
// English: OpenCV-based multi-stream visualization window. Supports multiple layout modes, key controls, info overlay, log display.
class CVWindow
{
public:
    // [构造函数 / Constructor]
    // 中文: 创建窗口，启动后台处理线程。
    // English: Create window, start background processing thread.
    CVWindow(std::string name, uint32_t width = 1280, uint32_t height = 720, ArrangeMode arrangeMode = ARRANGE_SINGLE);
    // [析构函数 / Destructor]
    // 中文: 关闭窗口，停止处理线程，销毁 OpenCV 窗口。
    // English: Close window, stop processing thread, destroy OpenCV window.
    ~CVWindow() noexcept;

    // [方法: run / Method: run]
    // 中文: 运行一帧：显示渲染图、处理按键。返回 false 表示请求关闭。
    // English: Run one frame: show render mat, handle keys. Returns false if close requested.
    bool run();
    // [方法: close / Method: close]
    // 中文: 请求关闭窗口，等待处理线程结束。
    // English: Request window close, wait for processing thread to finish.
    void close();
    // [方法: reset / Method: reset]
    // 中文: 重置窗口状态，重启处理线程。
    // English: Reset window state, restart processing thread.
    void reset();

    // [方法: pushFramesToView / Method: pushFramesToView]
    // 中文: 推送一组帧到指定分组进行显示（线程安全）。
    // English: Push a group of frames for display (thread-safe).
    void pushFramesToView(std::vector<const DynalgoFrame*> frames, int groupId = 0);
    // [方法: pushFramesToView / Method: pushFramesToView]
    // 中文: 推送单帧到指定分组（重载）。
    // English: Push single frame to group (overload).
    void pushFramesToView(const DynalgoFrame* currentFrame, int groupId = 0);

    // [方法: setShowInfo / Method: setShowInfo]
    // 中文: 设置是否显示帧信息（时间戳、分辨率等）。
    // English: Set whether to show frame info (timestamp, resolution, etc.).
    void setShowInfo(bool show);
    // [方法: setShowSyncTimeInfo / Method: setShowSyncTimeInfo]
    // 中文: 设置是否显示同步时间信息。
    // English: Set whether to show sync time info.
    void setShowSyncTimeInfo(bool show);
    // [方法: setAlpha / Method: setAlpha]
    // 中文: 设置叠加模式下的透明度 (0.0-1.0)。
    // English: Set alpha for overlay mode (0.0-1.0).
    void setAlpha(float alpha);
    // [方法: resize / Method: resize]
    // 中文: 调整窗口大小。
    // English: Resize window.
    void resize(int width, int height);
    // [方法: setKeyPressedCallback / Method: setKeyPressedCallback]
    // 中文: 设置按键回调函数。
    // English: Set key pressed callback.
    void setKeyPressedCallback(std::function<void(int)> callback);
    // [方法: setKeyPrompt / Method: setKeyPrompt]
    // 中文: 设置额外的按键提示文本。
    // English: Set additional key prompt text.
    void setKeyPrompt(const std::string& prompt);
    // [方法: addLog / Method: addLog]
    // 中文: 添加临时日志文本（显示 3 秒）。
    // English: Add temporary log text (shown for 3 seconds).
    void addLog(const std::string& log);
    // [方法: destroyWindow / Method: destroyWindow]
    // 中文: 销毁 OpenCV 窗口。
    // English: Destroy OpenCV window.
    void destroyWindow();

private:
    // [私有方法: processFrames / Private Method: processFrames]
    // 中文: 后台线程循环：从队列取帧、可视化、按布局排列、更新渲染图。
    // English: Background thread loop: fetch frames from queue, visualize, arrange by layout, update render mat.
    void processFrames();
    // [私有方法: arrangeFrames / Private Method: arrangeFrames]
    // 中文: 根据当前布局模式排列多个图像到单一渲染图。
    // English: Arrange multiple images into single render mat per current layout mode.
    void arrangeFrames();
    // [私有方法: visualize / Private Method: visualize]
    // 中文: 将单帧转换为可显示的 BGR 图像（按类型/格式分派）。
    // English: Convert single frame to displayable BGR image (dispatch by type/format).
    cv::Mat visualize(const DynalgoFrame* frame);
    // [私有方法: drawInfo / Private Method: drawInfo]
    // 中文: 在图像上绘制帧信息文本（带背景）。
    // English: Draw frame info text on image (with background).
    void drawInfo(cv::Mat& imageMat, const DynalgoFrame* frame);
    // [私有方法: resizeMatKeepAspectRatio / Private Method: resizeMatKeepAspectRatio]
    // 中文: 等比缩放图像并填充黑边以适应目标尺寸。
    // English: Resize image keeping aspect ratio, pad with black bars to fit target size.
    cv::Mat resizeMatKeepAspectRatio(const cv::Mat& mat, int width, int height);

private:
    // [成员变量 / Member Variables]
    // 中文: 窗口名称、布局模式、尺寸、关闭标志、显示选项、透明度。
    // English: Window name, layout mode, dimensions, closed flag, display options, alpha.
    std::string name_;
    ArrangeMode arrangeMode_;
    uint32_t width_;
    uint32_t height_;
    bool closed_;
    bool showInfo_;
    bool showSyncTimeInfo_;
    bool isWindowDestroyed_;
    float alpha_;

    // [成员变量: 线程与队列 / Member: thread & queues]
    // 中文: 处理线程、帧分组队列、互斥锁、条件变量。
    // English: Processing thread, frame group queues, mutex, condition variable.
    std::thread processThread_;
    std::map<int, std::vector<const DynalgoFrame*>> srcFrameGroups_;
    std::mutex srcFrameGroupsMtx_;
    std::condition_variable srcFrameGroupsCv_;

    // [成员变量: 渲染缓存 / Member: render cache]
    // 中文: 已可视化的帧图映射、渲染互斥锁、当前渲染图。
    // English: Visualized frame mats map, render mutex, current render mat.
    using StreamsMatMap = std::map<int, std::pair<const DynalgoFrame*, cv::Mat>>;
    StreamsMatMap matGroups_;
    std::mutex renderMatsMtx_;
    cv::Mat renderMat_;

    // [成员变量: 提示与日志 / Member: prompt & log]
    // 中文: 按键提示文本、显示标志、窗口创建时间、日志文本、日志创建时间。
    // English: Key prompt text, show flag, window creation time, log text, log creation time.
    std::string prompt_;
    bool showPrompt_;
    uint64 winCreatedTime_;

    std::string log_;
    uint64 logCreatedTime_;

    // [成员变量: 回调 / Member: callback]
    // 中文: 按键按下回调函数。
    // English: Key pressed callback function.
    std::function<void(int)> keyPressedCallback_;
};

} // namespace dynalgo
