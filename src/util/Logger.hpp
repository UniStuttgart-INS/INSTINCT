/**
 * @file Logger.hpp
 * @brief Utility class for logging to console and file
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-08
 */

#pragma once

// Available log levels
#define LOG_LEVEL_DATA 0
#define LOG_LEVEL_TRACE 1
#define LOG_LEVEL_DEBUG 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_WARN 4
#define LOG_LEVEL_ERROR 5
#define LOG_LEVEL_CRITICAL 6
#define LOG_LEVEL_OFF 7

// Active log level (passed as definition to cmake)
#if LOG_LEVEL == LOG_LEVEL_DATA
    /// Data Logging Macro
    #define LOG_DATA SPDLOG_TRACE
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#elif LOG_LEVEL == LOG_LEVEL_TRACE
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#elif LOG_LEVEL == LOG_LEVEL_DEBUG
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#elif LOG_LEVEL == LOG_LEVEL_INFO
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#elif LOG_LEVEL == LOG_LEVEL_WARN
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN
#elif LOG_LEVEL == LOG_LEVEL_ERROR
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_ERROR
#elif LOG_LEVEL == LOG_LEVEL_CRITICAL
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_CRITICAL
#elif LOG_LEVEL == LOG_LEVEL_OFF
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_OFF
#endif

/// External usage of FMT, as the internal one is missing some files
#define SPDLOG_FMT_EXTERNAL 1

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"

#include <string>
#include <stdexcept>

// Macros are redefined in case SPDLOG is not available anymore and it needs to be switched to another Logger

#ifndef LOG_DATA
    /// Data Logging Macro
    #define LOG_DATA(...) (void)0
#endif
/// Detailled info to trace the execution of the program
#define LOG_TRACE SPDLOG_TRACE
/// Debug information
#define LOG_DEBUG SPDLOG_DEBUG
/// Info to the user on the state of the program
#define LOG_INFO SPDLOG_INFO
/// Error occurred, but a fallback option exists and program continues to work normally
#define LOG_WARN SPDLOG_WARN
/// Error occurred, which stops part of the program to work, but not everything
#define LOG_ERROR SPDLOG_ERROR
/// Critical Event, which causes the program to work entirely and throws an exception
#define LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__), throw std::runtime_error(fmt::format(__VA_ARGS__))

/**
 * @brief Utility class for logging
 * 
 * Use the Macros to do logging, as they can be turned off during compilation
 * - LOG_DATA("Message {} {}", variable1, variable 2);
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
     * @brief Construct a new Logger object
     * 
     * @param[in] logpath Relative filepath to the logfile
     */
    explicit Logger(const std::string& logpath);

    Logger();                                  ///< Default constructor
    ~Logger();                                 ///< Destructor
    Logger(const Logger&) = delete;            ///< Copy constructor
    Logger(Logger&&) = delete;                 ///< Move constructor
    Logger& operator=(const Logger&) = delete; ///< Copy assignment operator
    Logger& operator=(Logger&&) = delete;      ///< Move assignment operator

  private:
    /// @brief Writes a separation line to the console only
    static void writeSeparator() noexcept;

    /// @brief Writes the logging header
    static void writeHeader() noexcept;

    /// @brief Writes the logging footer
    static void writeFooter() noexcept;
};