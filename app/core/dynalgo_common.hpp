// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_common.hpp — Shared Dynalgo utilities: signal handling, timestamps,
// directory creation, SEI NAL unit writing, device matching.
//
// [文件说明 / File Description]
// 中文：共享Dynalgo工具：信号处理、时间戳、目录创建、SEI NAL单元写入、设备匹配
// English: Shared Dynalgo utilities: signal handling, timestamps, directory creation, SEI NAL unit writing, device matching
//
// g_running / signalHandler: atomic flag set to false on SIGINT/SIGTERM,
// used by the main capture loop to exit cleanly.
//
// getTimestampMs / getTimestampMsInt / getTimestampIso: wall-clock time
// helpers for log messages and file naming.
//
// mkdirp: recursive mkdir -p equivalent (ignores EEXIST).
//
// SEI_COPYRIGHT / writeSEINalUnit: embed unregistered SEI NAL units
// with a 16-byte UUID prefix (used for copyright + per-frame timestamps).
//
// deviceMatches: case-insensitive substring match of device name against
// a filter list (used to select Orbbec camera models by name).

#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace dynalgo {

// [全局变量 / Global Variable]
// 中文：运行标志，SIGINT/SIGTERM时设置为false，用于主捕获循环干净退出
// English: Running flag, set to false on SIGINT/SIGTERM for clean main capture loop exit
extern std::atomic<bool> g_running;

// [信号处理函数 / Signal Handler]
// 中文：信号处理器，将g_running设置为false
// English: Signal handler, sets g_running to false
void signalHandler(int sig);

// [方法说明 / Method Description]
// 中文：获取当前时间戳字符串（毫秒）
// English: Get current timestamp string in milliseconds
std::string getTimestampMs();

// [方法说明 / Method Description]
// 中文：获取当前时间戳整数（毫秒）
// English: Get current timestamp integer in milliseconds
uint64_t getTimestampMsInt();

// [方法说明 / Method Description]
// 中文：获取ISO格式时间戳
// English: Get ISO format timestamp
std::string getTimestampIso();

// [方法说明 / Method Description]
// 中文：递归创建目录（忽略EEXIST错误）
// English: Recursively create directories (ignores EEXIST errors)
void mkdirp(const std::string& path);

// [全局常量 / Global Constant]
// 中文：SEI版权信息
// English: SEI copyright information
extern const char* SEI_COPYRIGHT;

// [方法说明 / Method Description]
// 中文：写入H.264未注册SEI NAL单元，包含16字节UUID前缀
// English: Write H.264 unregistered SEI NAL unit with 16-byte UUID prefix
void writeSEINalUnit(std::ofstream& outFile, const std::string& payload, std::mutex& mtx,
                     const char* uuid = "jiangtao.shen@ad");

// [方法说明 / Method Description]
// 中文：设备名称与过滤器列表的子字符串匹配（不区分大小写）
// English: Case-insensitive substring match of device name against filter list
bool deviceMatches(const std::string& deviceName, const std::vector<std::string>& filter);

} // namespace dynalgo
