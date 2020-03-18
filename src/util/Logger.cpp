#include "Logger.hpp"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <iostream>

NAV::NavStatus NAV::Logger::initialize(const std::string logpath)
{
    try
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
        spdlog::flush_on(spdlog::level::trace);

        // See https://github.com/gabime/spdlog/wiki/3.-Custom-formatting for formatting options
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [%s:%3#] [%!()] %v");
#elif SPDLOG_ACTIVE_LEVEL == SPDLOG_LEVEL_DEBUG
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [%s:%3#] %v");
#else
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] %v");
#endif

        return NavStatus::NAV_OK;
    }
    catch (const spdlog::spdlog_ex& ex)
    {
        std::cerr << "Log initialization failed: " << ex.what() << std::endl;
        return NavStatus::NAV_ERROR;
    }
}

void NAV::Logger::writeSeparator()
{
    std::cout << "===========================================================================================" << std::endl;
}

void NAV::Logger::writeHeader()
{
    writeSeparator();

    LOG_INFO("Program started");

    writeSeparator();
}

void NAV::Logger::writeFooter()
{
    writeSeparator();

#ifdef NDEBUG
    LOG_INFO("Program finished in Release");
#else
    LOG_INFO("Program finished in Debug");
#endif

    writeSeparator();
}