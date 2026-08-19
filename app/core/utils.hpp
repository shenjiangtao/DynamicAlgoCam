// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

// [文件说明 / File Description]
// 中文：C++工具函数和类，提供按键检测、时间戳获取、字符串转换和流状态保护
// English: C++ utility functions and classes for key press detection, timestamp retrieval, string conversion, and stream state protection

#pragma once
#include "utils_types.h"
#include <stdint.h>

#include <sstream>
#include <string>

namespace dynalgo {

// [方法说明 / Method Description]
// 中文：等待按键输入，支持超时机制
// English: Wait for key press input with timeout support
char waitForKeyPressed(uint32_t timeout_ms = 0);

// [方法说明 / Method Description]
// 中文：获取当前时间戳（毫秒）
// English: Get current timestamp in milliseconds
uint64_t getNowTimesMs();

// [方法说明 / Method Description]
// 中文：获取用户输入选项，ESC键返回-1，数字键返回对应数值
// English: Get user input option, returns -1 for ESC key, numeric value for digit keys
int getInputOption();

// [方法说明 / Method Description]
// 中文：获取可执行文件所在目录路径
// English: Get directory path of the executable file
std::string getExeDir();

// [方法说明 / Method Description]
// 中文：将值转换为固定精度的字符串
// English: Convert value to string with fixed precision
template <typename T>
std::string toString(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}

// [方法说明 / Method Description]
// 中文：检测终端是否支持ANSI转义序列
// English: Detect whether terminal supports ANSI escape sequences
bool supportAnsiEscape();

// [类说明 / Class Description]
// 中文：流状态守卫类，用于在作用域内保存和恢复ios流状态
// English: Stream state guard class for saving and restoring ios stream state within scope
class StreamStateGuard
{
public:
    explicit StreamStateGuard(std::ios& s) : ios(s), flags(s.flags()), fill(s.fill()) {}
    ~StreamStateGuard() {
        ios.flags(flags);
        ios.fill(fill);
    }

private:
    std::ios& ios;
    std::ios::fmtflags flags;
    char fill{ 0 };
};

} // namespace dynalgo
