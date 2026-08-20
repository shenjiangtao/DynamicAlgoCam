// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_common.cpp — Implementation of shared Dynalgo utilities.
//
// [文件说明 / File Description]
// 中文：共享Dynalgo工具实现，包括信号处理、时间戳、目录创建、SEI NAL单元写入和设备匹配
// English: Shared Dynalgo utilities implementation, including signal handling, timestamps, directory creation, SEI NAL unit writing, and device matching
//
// Sections:
//   1. Signal handling + g_running
//   2. Timestamp helpers
//   3. mkdirp — recursive directory creation
//   4. SEI NAL unit writer (H.264 unregistered SEI with UUID prefix)
//   5. deviceMatches — filter device name by substring list

#include "dynalgo_common.hpp"
#include "dynalgo_log.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstring>
#include <ctime>
#include <iostream>
#include <sstream>
#include <sys/stat.h>

namespace dynalgo {

// === Section 1: Signal handling ===

// [全局变量 / Global Variable]
// 中文：运行标志，初始为true，收到SIGINT/SIGTERM时设置为false
// English: Running flag, initially true, set to false on SIGINT/SIGTERM
std::atomic<bool> g_running{ true };

// [信号处理器 / Signal Handler]
// 中文：信号处理器，将运行标志设置为false
// English: Signal handler, sets running flag to false
void signalHandler(int) {
    g_running = false;
}

// === Section 2: Timestamp helpers ===

// [方法说明 / Method Description]
// 中文：获取当前时间戳字符串（毫秒）
// English: Get current timestamp string in milliseconds
std::string getTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return std::to_string(ms);
}

// [方法说明 / Method Description]
// 中文：获取当前时间戳整数（毫秒）
// English: Get current timestamp integer in milliseconds
uint64_t getTimestampMsInt() {
    auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
}

// [方法说明 / Method Description]
// 中文：获取ISO格式时间戳字符串
// English: Get ISO format timestamp string
std::string getTimestampIso() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    time_t secs = static_cast<time_t>(ms / 1000);
    int millis = static_cast<int>(ms % 1000);
    struct tm t;
    localtime_r(&secs, &t);
    char buf[64];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec, millis);
    return std::string(buf);
}

// === Section 3: mkdirp — recursive mkdir -p (ignores EEXIST) ===

// [方法说明 / Method Description]
// 中文：递归创建目录，忽略EEXIST错误
// English: Recursively create directories, ignoring EEXIST errors
void mkdirp(const std::string& path) {
    size_t pos = 0;
    std::string tmp;
    while ((pos = path.find('/', pos + 1)) != std::string::npos) {
        tmp = path.substr(0, pos);
        mkdir(tmp.c_str(), 0755);
    }
    mkdir(path.c_str(), 0755);
}

// === Section 4: SEI NAL unit writer ===

// [全局常量 / Global Constant]
// 中文：SEI版权信息字符串
// English: SEI copyright information string
const char* SEI_COPYRIGHT = "Copyright jiangtao.shen@nio.com";

// [方法说明 / Method Description]
// 中文：写入H.264未注册SEI NAL单元，包含16字节UUID前缀和载荷字符串
// English: Write H.264 unregistered SEI NAL unit with 16-byte UUID prefix and payload string
void writeSEINalUnit(std::ofstream& outFile, const std::string& payload, std::mutex& mtx, const char* uuid) {
    // [线程本地缓冲区 / Thread-Local Buffers]
    // 中文：跨帧重用线程本地缓冲区，每个编码器线程一个稳定缓冲区
    // English: Reuse thread-local buffers across frames, one stable buffer per encoder thread
    static thread_local std::vector<uint8_t> rbsp;
    static thread_local std::vector<uint8_t> nal;
    rbsp.clear();
    nal.clear();

    for (int i = 0; i < 16; i++)
        rbsp.push_back(static_cast<uint8_t>(uuid[i]));
    for (size_t i = 0; i < payload.size(); i++)
        rbsp.push_back(static_cast<uint8_t>(payload[i]));

    size_t payloadSize = rbsp.size();

    nal.push_back(0x00);
    nal.push_back(0x00);
    nal.push_back(0x00);
    nal.push_back(0x01);
    nal.push_back(0x06);
    nal.push_back(0x05);

    while (payloadSize >= 255) {
        nal.push_back(0xFF);
        payloadSize -= 255;
    }
    nal.push_back(static_cast<uint8_t>(payloadSize));

    // [仿真实现防止字节 / Emulation Prevention Byte Stuffing]
    // 中文：插入仿真实现防止字节，避免NAL单元中的起始码冲突
    // English: Insert emulation prevention byte stuffing to avoid start code conflicts in NAL unit
    int zeroCount = 0;
    for (size_t i = 0; i < rbsp.size(); i++) {
        uint8_t b = rbsp[i];
        if (zeroCount >= 2 && b <= 0x03) {
            nal.push_back(0x03);
            zeroCount = 0;
        }
        nal.push_back(b);
        if (b == 0x00)
            zeroCount++;
        else
            zeroCount = 0;
    }

    nal.push_back(0x80);

    {
        std::lock_guard<std::mutex> lock(mtx);
        outFile.write(reinterpret_cast<const char*>(nal.data()), nal.size());
    }
}

// === Section 5: deviceMatches — substring match against filter list ===

// [方法说明 / Method Description]
// 中文：设备名称与过滤器列表的子字符串匹配，空列表表示接受所有
// English: Case-insensitive substring match of device name against filter list, empty list accepts all
bool deviceMatches(const std::string& deviceName, const std::vector<std::string>& filter) {
    if (filter.empty())
        return true;
    for (const auto& f : filter) {
        if (deviceName.find(f) != std::string::npos)
            return true;
    }
    return false;
}

} // namespace dynalgo
