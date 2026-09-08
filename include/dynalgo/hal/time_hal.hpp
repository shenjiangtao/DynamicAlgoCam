#pragma once

#include "hal_types.hpp"
#include <string>
#include <chrono>
#include <functional>

namespace dynalgo::hal {

enum class TimeSource : uint32_t {
    MONOTONIC = 0,
    REALTIME = 1,
    PTP = 2,
    GPS = 3,
    NTP = 4,
    CUSTOM = 0xFFFF,
};

struct TimeConfig {
    TimeSource source = TimeSource::MONOTONIC;
    std::string ptp_interface;
    std::string gps_device;
    std::string ntp_server;
    uint32_t sync_interval_ms = 1000;
    int64_t max_offset_ns = 1000000;
    bool auto_sync = true;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct TimeInfo {
    TimeSource source = TimeSource::MONOTONIC;
    uint64_t timestamp_ns = 0;
    int64_t offset_ns = 0;
    int64_t frequency_ppb = 0;
    bool synchronized = false;
    uint64_t last_sync_ns = 0;
    std::string status;
};

struct PTPConfig {
    std::string interface;
    uint8_t domain = 0;
    bool master_only = false;
    bool slave_only = false;
    int priority1 = 128;
    int priority2 = 128;
    uint32_t announce_interval = 1;
    uint32_t sync_interval = 0;
    uint32_t delay_mechanism = 1;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

class ITimeHAL {
public:
    virtual ~ITimeHAL() = default;

    virtual ResultVoid initialize(const TimeConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual uint64_t now_ns() = 0;
    virtual TimePoint now_tp() = 0;
    virtual Result<uint64_t> now_ns(TimeSource source) = 0;

    virtual ResultVoid sync() = 0;
    virtual Result<TimeInfo> getTimeInfo() = 0;

    virtual ResultVoid setTime(uint64_t timestamp_ns) = 0;
    virtual ResultVoid adjustOffset(int64_t offset_ns) = 0;

    virtual ResultVoid startPTP(const PTPConfig& config) = 0;
    virtual ResultVoid stopPTP() = 0;
    virtual Result<TimeInfo> getPTPStatus() = 0;

    virtual ResultVoid setAlarm(uint64_t timestamp_ns, std::function<void()> callback) = 0;
    virtual ResultVoid cancelAlarm() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;

    virtual bool isSynchronized() const = 0;
};

class TimeHALFactory {
public:
    using CreateFunc = ITimeHAL* (*)();
    using DestroyFunc = void (*)(ITimeHAL*);

    static Result<ITimeHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<ITimeHAL*> create(const TimeConfig& config);
    static void destroy(ITimeHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);

    static ITimeHAL* getSystemTime();
    static void setSystemTime(ITimeHAL* time_hal);
};

inline uint64_t getMonotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline uint64_t getRealtimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

} // namespace dynalgo::hal