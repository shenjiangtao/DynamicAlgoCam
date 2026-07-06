// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "utils.hpp"
#include "utils_c.h"
#include "nio_common.hpp"

#include <chrono>
#include <linux/limits.h>
#include <unistd.h>

namespace nio {
char waitForKeyPressed(uint32_t timeout_ms) {
    return nio_wait_for_key_press(timeout_ms);
}

// Forward to the single canonical time provider in nio_common.cpp.
uint64_t getNowTimesMs() {
    return nio::getTimestampMsInt();
}

int getInputOption() {
    char inputOption = nio::waitForKeyPressed();
    if (inputOption == ESC_KEY) {
        return -1;
    }
    return inputOption - '0';
}

bool supportAnsiEscape() {
    if (nio_support_ansi_escape() == 0) {
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

} // namespace nio
