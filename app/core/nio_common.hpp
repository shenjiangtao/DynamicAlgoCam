// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_common.hpp — Shared NIO utilities: signal handling, timestamps,
// directory creation, SEI NAL unit writing, device matching.
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

#include <string>
#include <vector>
#include <fstream>
#include <mutex>
#include <atomic>
#include <cstdint>

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

} // namespace nio
