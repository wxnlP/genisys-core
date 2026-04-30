#pragma once

#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif

#include <fmt/core.h>
#include <fmt/format.h>

#include <atomic>
#include <iostream>
#include <string_view>

enum class LogLevel {
  DEBUG = 0,
  INFO,
  WARN,
  ERROR,
};

class ArmLogger {
 public:
  static void SetLogLevel(LogLevel level) { log_level_.store(level); }

  static LogLevel GetLogLevel() { return log_level_.load(); }

  static bool ShouldLog(LogLevel level) {
    return static_cast<int>(level) >= static_cast<int>(GetLogLevel());
  }

  static constexpr std::string_view LevelName(LogLevel level) {
    switch (level) {
      case LogLevel::DEBUG:
        return "DEBUG";
      case LogLevel::INFO:
        return "INFO";
      case LogLevel::WARN:
        return "WARN";
      case LogLevel::ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
  }

  template <typename... Args>
  static void Log(LogLevel level, fmt::format_string<Args...> fmt_str,
                  Args&&... args) {
    if (!ShouldLog(level)) {
      return;
    }

    const auto message = fmt::format(fmt_str, std::forward<Args>(args)...);
    std::ostream& output = (level == LogLevel::WARN || level == LogLevel::ERROR)
                               ? std::cerr
                               : std::cout;
    output << '[' << LevelName(level) << "] " << message << '\n';
  }

 private:
  inline static std::atomic<LogLevel> log_level_{LogLevel::INFO};
};

#define ARM_LOG_DEBUG(fmt, ...) \
  ArmLogger::Log(LogLevel::DEBUG, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
#define ARM_LOG_INFO(fmt, ...) \
  ArmLogger::Log(LogLevel::INFO, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
#define ARM_LOG_WARN(fmt, ...) \
  ArmLogger::Log(LogLevel::WARN, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
#define ARM_LOG_ERROR(fmt, ...) \
  ArmLogger::Log(LogLevel::ERROR, FMT_STRING(fmt) __VA_OPT__(, ) __VA_ARGS__)
