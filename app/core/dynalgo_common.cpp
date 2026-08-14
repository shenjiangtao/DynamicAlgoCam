// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_common.cpp — Implementation of shared NIO utilities.
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

std::atomic<bool> g_running{ true };

void signalHandler(int) {
    g_running = false;
}

// === Section 2: Timestamp helpers ===

std::string getTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return std::to_string(ms);
}

uint64_t getTimestampMsInt() {
    auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
}

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
// Writes H.264 unregistered SEI NAL (type 6, payload type 5) with:
//   - 16-byte UUID prefix (first 16 chars of uuid string)
//   - payload string (copyright notice or "dts=..." timestamp)
//   - RBSP trailing bits + emulation prevention byte stuffing
//
// NAL structure: [00 00 00 01] [06 05] [size bytes] [UUID+payload RBSP] [80]

const char* SEI_COPYRIGHT = "Copyright jiangtao.shen@nio.com";

void writeSEINalUnit(std::ofstream& outFile, const std::string& payload, std::mutex& mtx, const char* uuid) {
    // Reuse thread-local buffers across frames: each EncodeStreamTask runs
    // on its own StreamTask thread, so thread_local gives one stable buffer
    // per encoder thread. clear() preserves capacity — no per-frame malloc
    // after the first call. Hot path: 30fps x N streams.
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
// Returns true if filter is empty (no filter = accept all) or if any
// filter string appears as a substring of deviceName.

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
