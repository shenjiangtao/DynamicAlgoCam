// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

// [文件说明 / File Description]
// 中文：C语言工具函数接口，提供时间戳获取、按键等待和ANSI转义序列支持
// English: C-language utility function interfaces for timestamp retrieval, key press waiting, and ANSI escape sequence support

#pragma once

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// [方法说明 / Method Description]
// 中文：获取当前时间戳（毫秒），用于日志和文件命名
// English: Get current timestamp in milliseconds for logging and file naming
uint64_t dynalgo_get_current_timestamp_ms(void);

// [方法说明 / Method Description]
// 中文：等待按键输入，支持超时机制
// English: Wait for key press input with timeout support
char dynalgo_wait_for_key_press(uint32_t timeout_ms);

// [方法说明 / Method Description]
// 中文：检测终端是否支持ANSI转义序列
// English: Detect whether terminal supports ANSI escape sequences
int dynalgo_support_ansi_escape(void);

#ifdef __cplusplus
}
#endif
