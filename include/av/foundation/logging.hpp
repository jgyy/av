#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <memory>
#include <string>

namespace av {

// Logging singleton for the entire application
class Logger {
public:
    static void init(const std::string& logFile = "av.log") {
        if (logger_) {
            return; // Already initialized
        }

        try {
            // Create console sink
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_level(spdlog::level::debug);
            console_sink->set_pattern("[%H:%M:%S] [%^%l%$] %v");

            // Create file sink
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFile, true);
            file_sink->set_level(spdlog::level::debug);
            file_sink->set_pattern("[%H:%M:%S] [%l] %v");

            // Combine sinks
            spdlog::sinks_init_list sink_list = {console_sink, file_sink};
            logger_ = std::make_shared<spdlog::logger>("av", sink_list);

            logger_->set_level(spdlog::level::debug);
            logger_->flush_on(spdlog::level::err);

            // Register global logger
            spdlog::register_logger(logger_);
        } catch (const spdlog::spdlog_ex& ex) {
            // Fallback: create basic console logger
            logger_ = spdlog::stdout_color_mt("av");
            logger_->error("Log initialization failed: {}", ex.what());
        }
    }

    static void setLevel(spdlog::level::level_enum level) {
        if (logger_) {
            logger_->set_level(level);
        }
    }

    static std::shared_ptr<spdlog::logger>& get() {
        if (!logger_) {
            init();
        }
        return logger_;
    }

    static void shutdown() {
        if (logger_) {
            logger_->flush();
            spdlog::drop("av");
            logger_.reset();
        }
        spdlog::shutdown();
    }

private:
    static std::shared_ptr<spdlog::logger> logger_;
};

// Inline logging functions
inline void logTrace(const std::string& message) {
    Logger::get()->trace(message);
}

inline void logDebug(const std::string& message) {
    Logger::get()->debug(message);
}

inline void logInfo(const std::string& message) {
    Logger::get()->info(message);
}

inline void logWarn(const std::string& message) {
    Logger::get()->warn(message);
}

inline void logError(const std::string& message) {
    Logger::get()->error(message);
}

inline void logCritical(const std::string& message) {
    Logger::get()->critical(message);
}

} // namespace av

// Convenient macros
#define AV_TRACE(msg) av::Logger::get()->trace(msg)
#define AV_DEBUG(msg) av::Logger::get()->debug(msg)
#define AV_INFO(msg) av::Logger::get()->info(msg)
#define AV_WARN(msg) av::Logger::get()->warn(msg)
#define AV_ERROR(msg) av::Logger::get()->error(msg)
#define AV_CRITICAL(msg) av::Logger::get()->critical(msg)
