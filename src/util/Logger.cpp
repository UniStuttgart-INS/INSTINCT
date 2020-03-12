#include "Logger.hpp"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <iostream>

NAV::NavStatus NAV::Logger::initialize(const std::string logpath)
{
    try
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        // Level should be <= LOG_ACTIVE_LEVEL as we use Logging-Macros
        console_sink->set_level(spdlog::level::trace);
        // See https://github.com/gabime/spdlog/wiki/3.-Custom-formatting for formatting options
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [%s:%#] %v");

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logpath, true);
        // Level should be <= LOG_ACTIVE_LEVEL as we use Logging-Macros
        file_sink->set_level(spdlog::level::trace);
        // See https://github.com/gabime/spdlog/wiki/3.-Custom-formatting for formatting options
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] [%s:%#] %v");

        // Set the logger as default logger
        spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({ console_sink, file_sink })));
        // Level should be smaller or equal to the level of the sinks
        spdlog::set_level(spdlog::level::level_enum::trace);

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
    std::cout << "===========================================================================" << std::endl;
}

void NAV::Logger::writeHeader()
{
    writeSeparator();

    LOG_INFO("Software started");

    writeSeparator();
}

void NAV::Logger::writeFooter()
{
    writeSeparator();

#ifdef NDEBUG
    LOG_INFO("Programm finished in Release");
#else
    LOG_INFO("Programm finished in Debug");
#endif

    writeSeparator();
}