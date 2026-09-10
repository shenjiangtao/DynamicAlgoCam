#pragma once

#include "hal_types.hpp"
#include <string>
#include <functional>
#include <memory>
#include <chrono>

namespace dynalgo::hal {

enum class LogLevel : uint32_t {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5,
    OFF = 6,
};

enum class LogDestination : uint32_t {
    CONSOLE = 1 << 0,
    FILE = 1 << 1,
    SYSLOG = 1 << 2,
    NETWORK = 1 << 3,
    CALLBACK = 1 << 4,
    ALL = 0xFFFFFFFF,
};

inline LogDestination operator|(LogDestination a, LogDestination b) {
    return static_cast<LogDestination>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline LogDestination operator&(LogDestination a, LogDestination b) {
    return static_cast<LogDestination>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline LogDestination operator^(LogDestination a, LogDestination b) {
    return static_cast<LogDestination>(static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}

inline LogDestination operator~(LogDestination a) {
    return static_cast<LogDestination>(~static_cast<uint32_t>(a));
}

inline LogDestination& operator|=(LogDestination& a, LogDestination b) {
    a = a | b;
    return a;
}

inline LogDestination& operator&=(LogDestination& a, LogDestination b) {
    a = a & b;
    return a;
}

inline LogDestination& operator^=(LogDestination& a, LogDestination b) {
    a = a ^ b;
    return a;
}

struct LogConfig {
    LogLevel min_level = LogLevel::INFO;
    LogDestination destinations = LogDestination::CONSOLE | LogDestination::FILE;
    std::string log_file = "dynalgo.log";
    size_t max_file_size_mb = 100;
    uint32_t max_files = 10;
    bool async_logging = true;
    uint32_t flush_interval_ms = 1000;
    std::string pattern = "[%Y-%m-%d %H:%M:%S.%f] [%L] [%t] %v";
    bool color_output = true;
    void* vendor_params = nullptr;
    size_t vendor_params_size = 0;
};

struct LogEntry {
    LogLevel level = LogLevel::INFO;
    uint64_t timestamp_ns = 0;
    std::string logger_name;
    std::string message;
    std::string file;
    int line = 0;
    std::string function;
    uint32_t thread_id = 0;
    void* vendor_data = nullptr;
    size_t vendor_data_size = 0;
};

class ILoggerHAL {
public:
    virtual ~ILoggerHAL() = default;

    virtual ResultVoid initialize(const LogConfig& config) = 0;
    virtual ResultVoid deinitialize() = 0;

    virtual void log(const LogEntry& entry) = 0;
    virtual void log(LogLevel level, const std::string& logger_name,
                     const std::string& message, const std::string& file = "",
                     int line = 0, const std::string& function = "") = 0;

    virtual void trace(const std::string& logger_name, const std::string& msg,
                       const std::string& file = "", int line = 0, const std::string& func = "") {
        log(LogLevel::TRACE, logger_name, msg, file, line, func);
    }
    virtual void debug(const std::string& logger_name, const std::string& msg,
                       const std::string& file = "", int line = 0, const std::string& func = "") {
        log(LogLevel::DEBUG, logger_name, msg, file, line, func);
    }
    virtual void info(const std::string& logger_name, const std::string& msg,
                      const std::string& file = "", int line = 0, const std::string& func = "") {
        log(LogLevel::INFO, logger_name, msg, file, line, func);
    }
    virtual void warn(const std::string& logger_name, const std::string& msg,
                      const std::string& file = "", int line = 0, const std::string& func = "") {
        log(LogLevel::WARN, logger_name, msg, file, line, func);
    }
    virtual void error(const std::string& logger_name, const std::string& msg,
                       const std::string& file = "", int line = 0, const std::string& func = "") {
        log(LogLevel::ERROR, logger_name, msg, file, line, func);
    }
    virtual void fatal(const std::string& logger_name, const std::string& msg,
                       const std::string& file = "", int line = 0, const std::string& func = "") {
        log(LogLevel::FATAL, logger_name, msg, file, line, func);
    }

    virtual ResultVoid setLevel(LogLevel level) = 0;
    virtual Result<LogLevel> getLevel() = 0;
    virtual ResultVoid setLevel(const std::string& logger_name, LogLevel level) = 0;

    virtual ResultVoid flush() = 0;

    virtual Result<std::string> getName() const = 0;
    virtual Result<std::string> getVendor() const = 0;
    virtual Result<std::string> getVersion() const = 0;

    virtual ResultVoid vendorCommand(uint32_t cmd_id,
                                     const void* in_data, size_t in_size,
                                     void* out_data, size_t out_size) = 0;
};

class LoggerHALFactory {
public:
    using CreateFunc = ILoggerHAL* (*)();
    using DestroyFunc = void (*)(ILoggerHAL*);

    static Result<ILoggerHAL*> create(const std::string& platform, const std::string& vendor);
    static Result<ILoggerHAL*> create(const LogConfig& config);
    static void destroy(ILoggerHAL* hal);

    static std::vector<std::string> getSupportedVendors(const std::string& platform);
    static void registerVendor(const std::string& platform, const std::string& vendor,
                               CreateFunc create, DestroyFunc destroy);

    static ILoggerHAL* getDefaultLogger();
    static void setDefaultLogger(ILoggerHAL* logger);
};

#define DYNALGO_HAL_LOG_TRACE(logger, msg) \
    dynalgo::hal::LoggerHALFactory::getDefaultLogger()->trace(logger, msg, __FILE__, __LINE__, __FUNCTION__)

#define DYNALGO_HAL_LOG_DEBUG(logger, msg) \
    dynalgo::hal::LoggerHALFactory::getDefaultLogger()->debug(logger, msg, __FILE__, __LINE__, __FUNCTION__)

#define DYNALGO_HAL_LOG_INFO(logger, msg) \
    dynalgo::hal::LoggerHALFactory::getDefaultLogger()->info(logger, msg, __FILE__, __LINE__, __FUNCTION__)

#define DYNALGO_HAL_LOG_WARN(logger, msg) \
    dynalgo::hal::LoggerHALFactory::getDefaultLogger()->warn(logger, msg, __FILE__, __LINE__, __FUNCTION__)

#define DYNALGO_HAL_LOG_ERROR(logger, msg) \
    dynalgo::hal::LoggerHALFactory::getDefaultLogger()->error(logger, msg, __FILE__, __LINE__, __FUNCTION__)

#define DYNALGO_HAL_LOG_FATAL(logger, msg) \
    dynalgo::hal::LoggerHALFactory::getDefaultLogger()->fatal(logger, msg, __FILE__, __LINE__, __FUNCTION__)

} // namespace dynalgo::hal