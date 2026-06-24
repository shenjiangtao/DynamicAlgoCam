// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "utils.hpp"
#include "utils_c.h"

#include <chrono>

namespace nio {
char waitForKeyPressed(uint32_t timeout_ms) {
    return nio_wait_for_key_press(timeout_ms);
}

uint64_t getNowTimesMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
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

bool isGemini305Device(int vid, int pid) {
    return nio_is_gemini305_device(vid, pid);
}

bool isGemini305gDevice(int vid, int pid, const char* connectionType) {
    return nio_is_gemini305g_device(vid, pid, connectionType);
}

bool isAstraMiniDevice(int vid, int pid) {
    return nio_is_astra_mini_device(vid, pid);
}

} // namespace nio
