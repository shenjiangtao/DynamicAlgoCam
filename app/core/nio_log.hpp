// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_log.hpp — Header-only Logger singleton with multi-level output,
// thread-safe logging, and auto-flush on destruction.
//
// Usage:
//   NIO_LOG_INIT("my_process", "/tmp/logs")     — must call once at startup
//   NIO_LOG_SET_LEVEL(nio::LogLevel::INFO)       — set minimum level
//   NIO_LOG_INFO_S("width=" << w << " height=" << h)  — stream-style logging
//   NIO_LOG_SHUTDOWN()                            — optional explicit close
//
// Log format: "YYYY-MM-DD HH:MM:SS.mmm LEVEL [thread_id] file:line func | msg"
//
// Macro variants:
//   NIO_LOG_INFO(msg)    — msg is a std::string
//   NIO_LOG_INFO_S(msg)  — msg is an ostringstream expression (preferred)

#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace nio {

// Log severity levels: TRACE < DEBUG < INFO < WARN < ERROR < FATAL
enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5
};

namespace detail {

inline const char* ansiColor(LogLevel level) {
    switch (level) {
    case LogLevel::TRACE:
        return "\033[90m";
    case LogLevel::DEBUG:
        return "\033[36m";
    case LogLevel::INFO:
        return "\033[32m";
    case LogLevel::WARN:
        return "\033[33m";
    case LogLevel::ERROR:
        return "\033[31m";
    case LogLevel::FATAL:
        return "\033[35;1m";
    default:
        return "";
    }
}

inline const char* ansiReset() {
    return "\033[0m";
}

} // namespace detail

// Logger: singleton with file + console output.
// WARN/ERROR/FATAL also go to stderr; INFO and below go to stdout.
// All file writes are mutex-protected and auto-flushed.
class Logger
{
    // --- Public API: init, setLevel, log, flush, shutdown ---
public:
    static Logger& instance() { // Meyer's singleton — thread-safe in C++11
        static Logger logger;
        return logger;
    }

    bool init(const std::string& processName, const std::string& outputDir) {
        // Ensure initialization runs only once, even if called from multiple threads.
        // The once_flag guarantees thread‑safe one‑time execution without a lock.
        static std::once_flag initFlag;
        std::call_once(initFlag, [this, &processName, &outputDir]() {
            processName_ = processName;
            outputDir_ = outputDir;
            mkdirp(outputDir_);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
            logFilePath_ = outputDir_ + "/" + processName_ + "_log_" + std::to_string(ms) + ".log";
            logFile_.open(logFilePath_, std::ios::out | std::ios::app);
            if (!logFile_.is_open()) {
                std::cerr << "[NIO_LOG] FATAL: Cannot open log file: " << logFilePath_ << std::endl;
                // If opening fails, we leave initialized_ false so future attempts can retry.
                return;
            }
            initialized_ = true;
            logFile_ << "# NIO Log Start | process=" << processName_ << " | log_file=" << logFilePath_ << "\n";
            logFile_.flush();
        });
        // If the earlier call failed to open the file, initialized_ will be false.
        return initialized_;
    }

    void setLevel(LogLevel level) {
        level_.store(level, std::memory_order_relaxed);
    }

    LogLevel getLevel() const {
        return level_.load(std::memory_order_relaxed);
    }

    void log(LogLevel level, const char* file, int line, const char* func, const std::string& msg) {
        if (level < level_.load(std::memory_order_relaxed))
            return;
        std::string lineStr = formatLine(level, file, line, func, msg);
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (logFile_.is_open()) {
                logFile_ << lineStr << "\n";
                logFile_.flush();
            }
        }
        if (level >= LogLevel::WARN) {
            if (colorTerm_)
                std::cerr << detail::ansiColor(level) << lineStr << detail::ansiReset() << std::endl;
            else
                std::cerr << lineStr << std::endl;
        } else if (level >= LogLevel::INFO) {
            if (colorTerm_)
                std::cout << detail::ansiColor(level) << lineStr << detail::ansiReset() << std::endl;
            else
                std::cout << lineStr << std::endl;
        }
    }

    void flush() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (logFile_.is_open())
            logFile_.flush();
    }

    void shutdown() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (logFile_.is_open()) {
            logFile_ << "# NIO Log End | process=" << processName_ << "\n";
            logFile_.flush();
            logFile_.close();
        }
        initialized_ = false;
    }

    const std::string& getLogFilePath() const {
        return logFilePath_;
    }
    bool isInitialized() const {
        return initialized_;
    }

private:
    Logger()
    : level_(LogLevel::TRACE), initialized_(false), colorTerm_(isatty(STDERR_FILENO) && isatty(STDOUT_FILENO)) {}
    ~Logger() {
        shutdown();
    }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static void mkdirp(const std::string& path) {
        size_t pos = 0;
        std::string tmp;
        while ((pos = path.find('/', pos + 1)) != std::string::npos) {
            tmp = path.substr(0, pos);
            mkdir(tmp.c_str(), 0755);
        }
        mkdir(path.c_str(), 0755);
    }

    static const char* levelStr(LogLevel level) {
        switch (level) {
        case LogLevel::TRACE:
            return "TRACE";
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARN:
            return "WARN";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        default:
            return "?????";
        }
    }

    std::string formatLine(LogLevel level, const char* file, int line, const char* func, const std::string& msg) {
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        time_t secs = static_cast<time_t>(ms / 1000);
        int millis = static_cast<int>(ms % 1000);
        struct tm t;
        localtime_r(&secs, &t);
        char timeBuf[64];
        snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02d %02d:%02d:%02d.%03d", t.tm_year + 1900, t.tm_mon + 1,
                 t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, millis);
        const char* basename = strrchr(file, '/');
        basename = basename ? basename + 1 : file;
        std::ostringstream oss;
        oss << timeBuf << " " << levelStr(level) << " "
            << "[" << std::this_thread::get_id() << "] " << basename << ":" << line << " " << func << " | " << msg;
        return oss.str();
    }

    std::mutex mtx_;
    std::ofstream logFile_;
    std::string processName_;
    std::string outputDir_;
    std::string logFilePath_;
    std::atomic<LogLevel> level_;
    bool initialized_;
    bool colorTerm_;
};

} // namespace nio

#define NIO_LOG_INIT(processName, outputDir) ::nio::Logger::instance().init(processName, outputDir)
#define NIO_LOG_SET_LEVEL(level) ::nio::Logger::instance().setLevel(level)
#define NIO_LOG_SHUTDOWN() ::nio::Logger::instance().shutdown()
#define NIO_LOG_FLUSH() ::nio::Logger::instance().flush()
#define NIO_LOG_PATH() ::nio::Logger::instance().getLogFilePath()

#define NIO_LOG_TRACE(msg) ::nio::Logger::instance().log(::nio::LogLevel::TRACE, __FILE__, __LINE__, __func__, msg)
#define NIO_LOG_DEBUG(msg) ::nio::Logger::instance().log(::nio::LogLevel::DEBUG, __FILE__, __LINE__, __func__, msg)
#define NIO_LOG_INFO(msg) ::nio::Logger::instance().log(::nio::LogLevel::INFO, __FILE__, __LINE__, __func__, msg)
#define NIO_LOG_WARN(msg) ::nio::Logger::instance().log(::nio::LogLevel::WARN, __FILE__, __LINE__, __func__, msg)
#define NIO_LOG_ERROR(msg) ::nio::Logger::instance().log(::nio::LogLevel::ERROR, __FILE__, __LINE__, __func__, msg)
#define NIO_LOG_FATAL(msg) ::nio::Logger::instance().log(::nio::LogLevel::FATAL, __FILE__, __LINE__, __func__, msg)

#define NIO_LOG_TRACE_S(msg)          \
    do {                              \
        std::ostringstream _nio_ss;   \
        _nio_ss << msg;               \
        NIO_LOG_TRACE(_nio_ss.str()); \
    } while (0)
#define NIO_LOG_DEBUG_S(msg)          \
    do {                              \
        std::ostringstream _nio_ss;   \
        _nio_ss << msg;               \
        NIO_LOG_DEBUG(_nio_ss.str()); \
    } while (0)
#define NIO_LOG_INFO_S(msg)          \
    do {                             \
        std::ostringstream _nio_ss;  \
        _nio_ss << msg;              \
        NIO_LOG_INFO(_nio_ss.str()); \
    } while (0)
#define NIO_LOG_WARN_S(msg)          \
    do {                             \
        std::ostringstream _nio_ss;  \
        _nio_ss << msg;              \
        NIO_LOG_WARN(_nio_ss.str()); \
    } while (0)
#define NIO_LOG_ERROR_S(msg)          \
    do {                              \
        std::ostringstream _nio_ss;   \
        _nio_ss << msg;               \
        NIO_LOG_ERROR(_nio_ss.str()); \
    } while (0)
#define NIO_LOG_FATAL_S(msg)          \
    do {                              \
        std::ostringstream _nio_ss;   \
        _nio_ss << msg;               \
        NIO_LOG_FATAL(_nio_ss.str()); \
    } while (0)
