// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once
#include "utils_types.h"
#include <stdint.h>

#include <sstream>

namespace nio {
char waitForKeyPressed(uint32_t timeout_ms = 0);

uint64_t getNowTimesMs();

int getInputOption();

template <typename T>
std::string toString(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}

bool supportAnsiEscape();

class StreamStateGuard
{
public:
    explicit StreamStateGuard(std::ios& s) : ios(s), flags(s.flags()), fill(s.fill()) {}
    ~StreamStateGuard() {
        ios.flags(flags);
        ios.fill(fill);
    }

private:
    std::ios& ios;
    std::ios::fmtflags flags;
    char fill{ 0 };
};

} // namespace nio
