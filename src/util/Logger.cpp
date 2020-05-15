#include "Logger.hpp"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <iostream>

Logger::Logger(const std::string& logpath)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // Only edit if console and file should log different levels
    console_sink->set_level(spdlog::level::trace);

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logpath, true);
    // Only edit if console and file should log different levels
    file_sink->set_level(spdlog::level::trace);

    // Set the logger as default logger
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({ console_sink, file_sink })));

    // Level should be smaller or equal to the level of the sinks
    spdlog::set_level(spdlog::level::level_enum::trace);
    // Minimum level which automatically triggers a flush
    spdlog::flush_on(spdlog::level::info);

    // See https://github.com/gabime/spdlog/wiki/3.-Custom-formatting for formatting options
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [%s:%3#] [%!()] %v");
#elif SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [%s:%3#] %v");
#else
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] %v");
#endif

    writeHeader();
}

Logger::~Logger()
{
    writeFooter();
}

void Logger::writeSeparator() noexcept
{
    std::cout << "===========================================================================================" << std::endl;
}

void Logger::writeHeader() noexcept
{
    writeSeparator();

#ifdef NDEBUG
    LOG_INFO("Program started in Release");
#else
    LOG_INFO("Program started in Debug");
#endif

    writeSeparator();
}

void Logger::writeFooter() noexcept
{
    writeSeparator();

    LOG_INFO("Program finished");

    writeSeparator();
}