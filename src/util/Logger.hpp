/**
 * @file Logger.hpp
 * @brief Utility class for logging with spdlog
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 */

#pragma once

/// Turn off irrelevant log levels during compilation
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "Common.hpp"

namespace NAV
{
/**
 * @brief Static utility class for logging with spdlog
 * 
 * Use the spdlog Macros to do logging, as they can be turned off during compilation with the SPDLOG_ACTIVE_LEVEL
 * - SPDLOG_TRACE("Message");
 * - SPDLOG_DEBUG("Message");
 * - SPDLOG_INFO("Message");
 * - SPDLOG_WARN("Message");
 * - SPDLOG_ERROR("Message");
 * - SPDLOG_CRITICAL("Message");
 */
class Logger
{
  public:
    /**
     * @brief Initialize the global console and file spdlog
     * 
     * @retval NavStatus Indicates whether initialization was successful
     */
    static NavStatus initialize();

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