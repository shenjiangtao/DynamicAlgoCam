// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_common.hpp — Shared NIO utilities: signal handling, timestamps,
// directory creation, SEI NAL unit writing, device matching, profile selection.

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <mutex>
#include <atomic>
#include <cstdint>

#include <libobsensor/ObSensor.hpp>

namespace nio {

extern std::atomic<bool> g_running;

void signalHandler(int sig);

std::string getTimestampMs();

uint64_t getTimestampMsInt();

std::string getTimestampIso();

void mkdirp(const std::string &path);

extern const char *SEI_COPYRIGHT;

void writeSEINalUnit(std::ofstream &outFile, const std::string &payload,
                     std::mutex &mtx, const char *uuid = "nio@orbbec-fusio");

bool deviceMatches(const std::string &deviceName, const std::vector<std::string> &filter);

std::shared_ptr<ob::VideoStreamProfile> selectBestProfile(
    std::shared_ptr<ob::StreamProfileList> profiles, OBFormat preferredFormat);

} // namespace nio
