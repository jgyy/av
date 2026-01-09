#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdarg>

namespace av {

// Simple logging without external dependencies
class Logger {
public:
    enum Level { TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL };

    static void init(const std::string& logFile = "av.log") {
        // Simple initialization - just enable logging
        initialized_ = true;
    }

    static void setLevel(Level level) {
        minLevel_ = level;
    }

    static void shutdown() {
        initialized_ = false;
    }

    static void log(Level level, const char* format, ...) {
        if (!initialized_ || level < minLevel_) {
            return;
        }

        const char* levelStr[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"};
        std::fprintf(stderr, "[%s] ", levelStr[level]);

        va_list args;
        va_start(args, format);
        std::vfprintf(stderr, format, args);
        va_end(args);

        std::fprintf(stderr, "\n");
        std::fflush(stderr);
    }

private:
    static bool initialized_;
    static Level minLevel_;
};

} // namespace av

// Convenient macros with variadic arguments
#define AV_TRACE(...) av::Logger::log(av::Logger::TRACE, __VA_ARGS__)
#define AV_DEBUG(...) av::Logger::log(av::Logger::DEBUG, __VA_ARGS__)
#define AV_INFO(...) av::Logger::log(av::Logger::INFO, __VA_ARGS__)
#define AV_WARN(...) av::Logger::log(av::Logger::WARN, __VA_ARGS__)
#define AV_ERROR(...) av::Logger::log(av::Logger::ERROR, __VA_ARGS__)
#define AV_CRITICAL(...) av::Logger::log(av::Logger::CRITICAL, __VA_ARGS__)
