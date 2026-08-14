// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "utils.hpp"
#include "utils_c.h"
#include "dynalgo_common.hpp"

#include <chrono>
#include <linux/limits.h>
#include <unistd.h>

namespace dynalgo {
char waitForKeyPressed(uint32_t timeout_ms) {
    return dynalgo_wait_for_key_press(timeout_ms);
}

// Forward to the single canonical time provider in dynalgo_common.cpp.
uint64_t getNowTimesMs() {
    return dynalgo::getTimestampMsInt();
}

int getInputOption() {
    char inputOption = dynalgo::waitForKeyPressed();
    if (inputOption == ESC_KEY) {
        return -1;
    }
    return inputOption - '0';
}

bool supportAnsiEscape() {
    if (dynalgo_support_ansi_escape() == 0) {
        return false;
    }
    return true;
}

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
