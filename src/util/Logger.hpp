// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Logger.hpp
/// @brief Utility class for logging to console and file
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-08

#pragma once

// Available log levels
#define LOG_LEVEL_DATA 0     ///< All output which occurs repeatedly every time observations are received
#define LOG_LEVEL_TRACE 1    ///< Detailled info to trace the execution of the program. Should not be called on functions which receive observations (spamming)
#define LOG_LEVEL_DEBUG 2    ///< All output needed to debug functions. Should not be called on functions which receive observations (spamming)
#define LOG_LEVEL_INFO 3     ///< All output informing the user about normal operation.
#define LOG_LEVEL_WARN 4     ///< All output informing the user about abnormal operation, but the code can still function or can switch to a fallback option.
#define LOG_LEVEL_ERROR 5    ///< All output informing the user about abnormal operation, which results in the code not working anymore
#define LOG_LEVEL_CRITICAL 6 ///< A critical event occurred which results in termination of the program
#define LOG_LEVEL_OFF 7      ///< Logging turned off

// Active log level (passed as definition to cmake)
#if LOG_LEVEL == LOG_LEVEL_DATA
    /// All output which occurs repeatedly every time observations are received
    #define LOG_DATA SPDLOG_TRACE
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_TRACE
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_DEBUG
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_INFO
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_WARN
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_ERROR
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_ERROR ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_CRITICAL
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_CRITICAL ///< Set the active SPDLOG level
#elif LOG_LEVEL == LOG_LEVEL_OFF
    #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_OFF ///< Set the active SPDLOG level
#endif

/// External usage of FMT, as the internal one is missing some files
#define SPDLOG_FMT_EXTERNAL 1

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/ringbuffer_sink.h"
#include <fmt/std.h>

#include <string>
#include <stdexcept>

// Macros are redefined in case SPDLOG is not available anymore and it needs to be switched to another Logger

#ifndef LOG_DATA
    /// All output which occurs repeatedly every time observations are received
    #define LOG_DATA(...) (void)0
#endif
/// Detailled info to trace the execution of the program. Should not be called on functions which receive observations (spamming)
#define LOG_TRACE SPDLOG_TRACE
/// Debug information. Should not be called on functions which receive observations (spamming)
#define LOG_DEBUG SPDLOG_DEBUG
/// Info to the user on the state of the program
#define LOG_INFO SPDLOG_INFO
/// Error occurred, but a fallback option exists and program continues to work normally
#define LOG_WARN SPDLOG_WARN
/// Error occurred, which stops part of the program to work, but not everything
#define LOG_ERROR SPDLOG_ERROR
/// Critical Event, which causes the program to work entirely and throws an exception
#define LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__), throw std::runtime_error(fmt::format(__VA_ARGS__))

/// @brief Utility class for logging
///
/// Use the Macros to do logging, as they can be turned off during compilation
/// - LOG_DATA("Message {} {}", variable1, variable 2);
/// - LOG_TRACE("Message {} {}", variable1, variable 2);
/// - LOG_DEBUG("Message {} {}", variable1, variable 2);
/// - LOG_INFO("Message {} {}", variable1, variable 2);
/// - LOG_WARN("Message {} {}", variable1, variable 2);
/// - LOG_ERROR("Message {} {}", variable1, variable 2);
/// - LOG_CRITICAL("Message {} {}", variable1, variable 2);
class Logger
{
  public:
    /// @brief Constructor
    /// @param[in] logpath Relative filepath to the logfile
    explicit Logger(const std::string& logpath);

    /// @brief Default constructor
    Logger();
    /// @brief Destructor
    ~Logger();
    /// @brief Copy constructor
    Logger(const Logger&) = delete;
    /// @brief Move constructor
    Logger(Logger&&) = default;
    /// @brief Copy assignment operator
    Logger& operator=(const Logger&) = delete;
    /// @brief Move assignment operator
    Logger& operator=(Logger&&) = default;

    /// @brief Returns the ring buffer sink
    static const std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt>& GetRingBufferSink();

  private:
    /// @brief Ring buffer sink
    static inline std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> _ringBufferSink = nullptr;

    /// @brief Writes a separation line to the console only
    static void writeSeparator() noexcept;

    /// @brief Writes the logging header
    static void writeHeader() noexcept;

    /// @brief Writes the logging footer
    static void writeFooter() noexcept;
};