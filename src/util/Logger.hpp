/**
 * @file Logger.hpp
 * @brief Utility class for logging to console and file
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-08
 */

#pragma once

/// Enables the LOG_DATA() Macro (needs SPDLOG_LEVEL_TRACE)
#define DATA_LOGGING_ENABLED 0
/// Compile-time logging level
#ifndef SPDLOG_ACTIVE_LEVEL
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif
/// External usage of FMT, as the internal one is missing some files
#define SPDLOG_FMT_EXTERNAL 1

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"

#include <string>
#include <stdexcept>

// Macros are redefined in case SPDLOG is not available anymore and it needs to be switched to another Logger

#if DATA_LOGGING_ENABLED >= 1
    /// Data Logging Macro
    #define LOG_DATA SPDLOG_TRACE
#else
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