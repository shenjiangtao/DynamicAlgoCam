// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_common.cpp — Implementation of shared NIO utilities.

#include "nio_common.hpp"
#include "nio_log.hpp"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstring>
#include <csignal>
#include <sys/stat.h>

namespace nio {

std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

std::string getTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return std::to_string(ms);
}

uint64_t getTimestampMsInt() {
    auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
}

std::string getTimestampIso() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    time_t secs = static_cast<time_t>(ms / 1000);
    int millis = static_cast<int>(ms % 1000);
    struct tm t;
    localtime_r(&secs, &t);
    char buf[64];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec, millis);
    return std::string(buf);
}

void mkdirp(const std::string &path) {
    size_t pos = 0;
    std::string tmp;
    while((pos = path.find('/', pos + 1)) != std::string::npos) {
        tmp = path.substr(0, pos);
        mkdir(tmp.c_str(), 0755);
    }
    mkdir(path.c_str(), 0755);
}

const char *SEI_COPYRIGHT = "Copyright jiangtao.shen@nio.com";

void writeSEINalUnit(std::ofstream &outFile, const std::string &payload,
                     std::mutex &mtx, const char *uuid) {
    std::vector<uint8_t> rbsp;
    for(int i = 0; i < 16; i++) rbsp.push_back(static_cast<uint8_t>(uuid[i]));
    for(size_t i = 0; i < payload.size(); i++) rbsp.push_back(static_cast<uint8_t>(payload[i]));

    size_t payloadSize = rbsp.size();

    std::vector<uint8_t> nal;
    nal.push_back(0x00); nal.push_back(0x00);
    nal.push_back(0x00); nal.push_back(0x01);
    nal.push_back(0x06);
    nal.push_back(0x05);

    while(payloadSize >= 255) { nal.push_back(0xFF); payloadSize -= 255; }
    nal.push_back(static_cast<uint8_t>(payloadSize));

    int zeroCount = 0;
    for(size_t i = 0; i < rbsp.size(); i++) {
        uint8_t b = rbsp[i];
        if(zeroCount >= 2 && b <= 0x03) {
            nal.push_back(0x03);
            zeroCount = 0;
        }
        nal.push_back(b);
        if(b == 0x00) zeroCount++; else zeroCount = 0;
    }

    nal.push_back(0x80);

    {
        std::lock_guard<std::mutex> lock(mtx);
        outFile.write(reinterpret_cast<const char *>(nal.data()), nal.size());
    }
}

bool deviceMatches(const std::string &deviceName, const std::vector<std::string> &filter) {
    if(filter.empty()) return true;
    for(const auto &f : filter) {
        if(deviceName.find(f) != std::string::npos) return true;
    }
    return false;
}

std::shared_ptr<ob::VideoStreamProfile> selectBestProfile(
    std::shared_ptr<ob::StreamProfileList> profiles, OBFormat preferredFormat) {
    std::shared_ptr<ob::VideoStreamProfile> best;
    int bestScore = -1;

    for(uint32_t i = 0; i < profiles->getCount(); i++) {
        try {
            auto sp = profiles->getProfile(i);
            if(!sp) continue;
            auto vsp = sp->as<ob::VideoStreamProfile>();
            if(!vsp) continue;

            int score = 0;
            if(vsp->getFormat() == preferredFormat) score += 1000;
            if(vsp->getWidth() == 640) score += 100;
            else if(vsp->getWidth() == 848) score += 90;
            else if(vsp->getWidth() == 1280) score += 80;
            if(vsp->getFps() == 30) score += 50;
            else if(vsp->getFps() == 25) score += 45;
            else if(vsp->getFps() == 15) score += 30;

            if(score > bestScore) {
                bestScore = score;
                best = vsp;
            }
        } catch(...) { continue; }
    }

    if(!best && profiles->getCount() > 0) {
        try {
            auto sp = profiles->getProfile(0);
            best = sp->as<ob::VideoStreamProfile>();
        } catch(...) {}
    }
    return best;
}

} // namespace nio
