#pragma once

#include <string>
#include <cstdint>

/// Simple event descriptor used by the event‑driven recorder.
struct Event {
    std::string name;      // e.g. "E0", "E1"
    uint64_t tsMs;         // timestamp in milliseconds since epoch
    std::string payload;   // optional free‑form data (unused for now)
    Event() = default;
    Event(const std::string& n, uint64_t t, const std::string& p = "")
        : name(n), tsMs(t), payload(p) {}
};
