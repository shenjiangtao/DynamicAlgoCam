// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once
#include <stdint.h>
#include "utils_types.h"

#include <sstream>

namespace ob_smpl {
char waitForKeyPressed(uint32_t timeout_ms = 0);

uint64_t getNowTimesMs();

int getInputOption();

template <typename T> std::string toString(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}

bool supportAnsiEscape();

#ifdef ENABLE_ORBBEC
bool isGemini305Device(int vid, int pid);

bool isGemini305gDevice(int vid, int pid, const char *connectionType);

bool isAstraMiniDevice(int vid, int pid);
#endif

class StreamStateGuard {
public:
    explicit StreamStateGuard(std::ios &s) : ios(s), flags(s.flags()), fill(s.fill()) {}
    ~StreamStateGuard() {
        ios.flags(flags);
        ios.fill(fill);
    }

private:
    std::ios          &ios;
    std::ios::fmtflags flags;
    char               fill{ 0 };
};

}  // namespace ob_smpl

// Legacy Orbbec SDK utilities (requires ob::Device from ObSensor).
// For modern app code, use nio::isLiDARDevice() from nio_ob_adapter.hpp.
#ifdef ENABLE_ORBBEC
#include <libobsensor/ObSensor.hpp>
namespace ob_smpl {
bool isLiDARDevice(std::shared_ptr<ob::Device> device);
}
#endif
