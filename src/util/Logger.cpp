#include "Logger.hpp"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "internal/ConfigManager.hpp"

#include <chrono>
#include <ctime>
#include <iostream>

// See https://github.com/gabime/spdlog/wiki/3.-Custom-formatting for formatting options
const char* logPatternTrace = "[%H:%M:%S.%e] [%^%L%$] [%s:%#] [%!()] %v";
const char* logPatternDebug = "[%H:%M:%S.%e] [%^%L%$] [%s:%#] %v";
const char* logPatternInfo = "[%H:%M:%S.%e] [%^%L%$] %v";

Logger::Logger(const std::string& logpath)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // Only edit if console and file should log different levels
    console_sink->set_level(spdlog::level::from_str(NAV::ConfigManager::Get<std::string>("console-log-level")));
    switch (spdlog::level::from_str(NAV::ConfigManager::Get<std::string>("console-log-level")))
    {
    case spdlog::level::trace:
        console_sink->set_pattern(logPatternTrace);
        break;
    case spdlog::level::debug:
        console_sink->set_pattern(logPatternDebug);
        break;
    case spdlog::level::info:
    case spdlog::level::warn:
    case spdlog::level::err:
    case spdlog::level::critical:
        console_sink->set_pattern(logPatternInfo);
        break;
    case spdlog::level::off:
    case spdlog::level::n_levels:
        break;
    }

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logpath, true);
    // Only edit if console and file should log different levels
    file_sink->set_level(spdlog::level::from_str(NAV::ConfigManager::Get<std::string>("file-log-level")));
    switch (spdlog::level::from_str(NAV::ConfigManager::Get<std::string>("file-log-level")))
    {
    case spdlog::level::trace:
        file_sink->set_pattern(logPatternTrace);
        break;
    case spdlog::level::debug:
        file_sink->set_pattern(logPatternDebug);
        break;
    case spdlog::level::info:
    case spdlog::level::warn:
    case spdlog::level::err:
    case spdlog::level::critical:
        file_sink->set_pattern(logPatternInfo);
        break;
    case spdlog::level::off:
    case spdlog::level::n_levels:
        break;
    }

    // Set the logger as default logger
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({ console_sink, file_sink })));

    // Level should be smaller or equal to the level of the sinks
    spdlog::set_level(spdlog::level::level_enum::trace);
    // Minimum level which automatically triggers a flush
    spdlog::flush_on(spdlog::level::trace);

    writeHeader();
}

Logger::Logger()
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);

    // Set the logger as default logger
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("console_sink", spdlog::sinks_init_list({ console_sink })));

    // Level should be smaller or equal to the level of the sinks
    spdlog::set_level(spdlog::level::level_enum::trace);
    // Minimum level which automatically triggers a flush
    spdlog::flush_on(spdlog::level::trace);

    // See https://github.com/gabime/spdlog/wiki/3.-Custom-formatting for formatting options
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
    spdlog::set_pattern(logPatternTrace);
#elif SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG
    spdlog::set_pattern(logPatternDebug);
#else
    spdlog::set_pattern(logPatternInfo);
#endif

    writeHeader();
}

Logger::~Logger()
{
    writeFooter();

    spdlog::default_logger()->flush();
}

void Logger::writeSeparator() noexcept
{
    LOG_INFO("===========================================================================================");
}

void Logger::writeHeader() noexcept
{
    writeSeparator();

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    tm* t = std::localtime(&now_c);

#ifdef NDEBUG
    LOG_INFO("Program started in Release on {:04d}-{:02d}-{:02d}", 1900 + t->tm_year, 1 + t->tm_mon, t->tm_mday);
#else
    LOG_INFO("Program started in Debug on {:04d}-{:02d}-{:02d}", 1900 + t->tm_year, 1 + t->tm_mon, t->tm_mday);
#endif

    writeSeparator();
}

void Logger::writeFooter() noexcept
{
    writeSeparator();

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    tm* t = std::localtime(&now_c);

    LOG_INFO("Program finished on {:04d}-{:02d}-{:02d}", 1900 + t->tm_year, 1 + t->tm_mon, t->tm_mday);

    writeSeparator();
}