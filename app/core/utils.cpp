// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

// [文件说明 / File Description]
// 中文：C++工具函数实现，提供按键检测、时间戳获取、输入选项和ANSI转义序列支持
// English: C++ utility function implementations for key press detection, timestamp retrieval, input options, and ANSI escape sequence support

#include "utils.hpp"
#include "utils_c.h"
#include "dynalgo_common.hpp"

#include <chrono>
#include <linux/limits.h>
#include <unistd.h>

namespace dynalgo {

// [方法说明 / Method Description]
// 中文：等待按键输入，委托给C接口实现
// English: Wait for key press input, delegates to C interface implementation
char waitForKeyPressed(uint32_t timeout_ms) {
    return dynalgo_wait_for_key_press(timeout_ms);
}

// [方法说明 / Method Description]
// 中文：获取当前时间戳（毫秒），委托给dynalgo_common.cpp中的标准时间提供者
// English: Get current timestamp in milliseconds, delegates to canonical time provider in dynalgo_common.cpp
uint64_t getNowTimesMs() {
    return dynalgo::getTimestampMsInt();
}

// [方法说明 / Method Description]
// 中文：获取用户输入选项，ESC键返回-1，数字键返回对应数值
// English: Get user input option, returns -1 for ESC key, numeric value for digit keys
int getInputOption() {
    char inputOption = dynalgo::waitForKeyPressed();
    if (inputOption == ESC_KEY) {
        return -1;
    }
    return inputOption - '0';
}

// [方法说明 / Method Description]
// 中文：检测终端是否支持ANSI转义序列
// English: Detect whether terminal supports ANSI escape sequences
bool supportAnsiEscape() {
    if (dynalgo_support_ansi_escape() == 0) {
        return false;
    }
    return true;
}

// [方法说明 / Method Description]
// 中文：获取可执行文件所在目录路径
// English: Get directory path of the executable file
std::string getExeDir() {
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len <= 0)
        return ".";
    buf[len] = '\0';
    std::string path(buf);
    auto pos = path.rfind('/');
    if (pos == std::string::npos)
        return ".";
    return path.substr(0, pos);
}

} // namespace dynalgo
