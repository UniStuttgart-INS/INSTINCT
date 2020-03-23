/**
 * @file Logger.hpp
 * @brief Utility class for logging with spdlog
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 */

#pragma once

/// Turn off irrelevant log levels during compilation
#define DATA_LOGGING_ENABLED 1
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define SPDLOG_FMT_EXTERNAL 1

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "Common.hpp"

#include <string>

// Macros are redefined in case SPDLOG is not available anymore and it needs to be switched to another Logger
#if DATA_LOGGING_ENABLED >= 1
    #define LOG_DATA SPDLOG_TRACE
#else
    #define LOG_DATA(...) (void)0
#endif

#define LOG_TRACE SPDLOG_TRACE
#define LOG_DEBUG SPDLOG_DEBUG
#define LOG_INFO SPDLOG_INFO
#define LOG_WARN SPDLOG_WARN
#define LOG_ERROR SPDLOG_ERROR
#define LOG_CRITICAL SPDLOG_CRITICAL

namespace NAV
{
/**
 * @brief Static utility class for logging
 * 
 * Use the Macros to do logging, as they can be turned off during compilation
 * - LOG_TRACE("Message {} {}", variable1, variable 2);
 * - LOG_DEBUG("Message {} {}", variable1, variable 2);
 * - LOG_INFO("Message {} {}", variable1, variable 2);
 * - LOG_WARN("Message {} {}", variable1, variable 2);
 * - LOG_ERROR("Message {} {}", variable1, variable 2);
 * - LOG_CRITICAL("Message {} {}", variable1, variable 2);
 */
class Logger
{
  public:
    /**
     * @brief Initialize the global console and file logger
     * 
     * @param[in] logpath Relative filepath to the logfile
     * @retval NavStatus Indicates whether initialization was successful
     */
    static NavStatus initialize(const std::string logpath);

    /// @brief Writes a separation line to the console only
    static void writeSeparator();

    /// @brief Writes the logging header
    static void writeHeader();

    /// @brief Writes the logging footer
    static void writeFooter();

  private:
    /// @brief Constructor is defined private, so creating an instance of this object is imposible
    Logger(){};
};

} // namespace NAV