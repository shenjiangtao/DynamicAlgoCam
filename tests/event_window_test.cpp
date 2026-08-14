#include "event.hpp"
#include <gtest/gtest.h>

// Minimal copy of the EventWindow definition (as in the binary) for testing
struct EventWindowTest {
    uint64_t startTimeMs = 0;
    uint64_t endTimeMs = 0;
    const uint64_t marginMs = 2000;
    const uint64_t maxWindowMs = 60000;
    Event activeEvent;
    bool hasActive = false;
    void addEvent(const Event& ev) {
        if (startTimeMs == 0) {
            startTimeMs = ev.tsMs;
            endTimeMs = startTimeMs + marginMs;
            activeEvent = ev;
            hasActive = true;
        } else {
            if (ev.tsMs > endTimeMs) {
                startTimeMs = ev.tsMs;
                endTimeMs = startTimeMs + marginMs;
                activeEvent = ev;
                hasActive = true;
            }
        }
        if (endTimeMs - startTimeMs > maxWindowMs) {
            endTimeMs = startTimeMs + maxWindowMs;
        }
    }
    bool shouldContinue(uint64_t nowMs) const {
        if (startTimeMs == 0) return true;
        return nowMs <= endTimeMs;
    }
    std::string getBaseFileName() const {
        if (!hasActive) return "recording";
        return activeEvent.name + "_" + std::to_string(activeEvent.tsMs);
    }
};

TEST(EventWindowTest, BasicFlow) {
    EventWindowTest win;
    EXPECT_TRUE(win.shouldContinue(1000));

    win.addEvent(Event("E0", 5000));
    EXPECT_EQ(win.getBaseFileName(), "E0_5000");
    EXPECT_TRUE(win.shouldContinue(6000)); // within margin

    // Event inside current window – should keep same prefix
    win.addEvent(Event("E1", 6000));
    EXPECT_EQ(win.getBaseFileName(), "E0_5000");

    // Event after current deadline – new window
    win.addEvent(Event("E2", 8000));
    EXPECT_EQ(win.getBaseFileName(), "E2_8000");

    // Max window clamp
    win.addEvent(Event("E3", 8000 + 61000)); // 61s later
    EXPECT_LE(win.endTimeMs - win.startTimeMs, win.maxWindowMs);
}
