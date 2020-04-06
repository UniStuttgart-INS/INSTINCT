/**
 * @file NodeCreator.hpp
 * @brief Defines possible nodes and creates them from program options
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-03
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <tuple>

#include "util/Common.hpp"
#include "util/Config.hpp"

#include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
#include "Nodes/DataLogger/GNSS/UbloxDataLogger.hpp"
#include "Nodes/UsbSync/GNSS/UbloxSyncSignal.hpp"

namespace NAV
{
/// Class representing an Input Port of a Node
class InputPort
{
  public:
    /// Type of the Input Node Data
    std::string type;
    /// Callback to be triggered when receiving the node data
    std::function<NAV::NavStatus(std::shared_ptr<void>, std::shared_ptr<void>)> callback;
};

/// Enum Providing possible types for program options
enum ConfigOptions
{
    CONFIG_UINT = 0,   ///< Unsigned Integer
    CONFIG_INT = 1,    ///< Integer
    CONFIG_STRING = 2, ///< String
};

/// Interface for a Node, specifying output data types and input types and callbacks
class NodeInterface
{
  public:
    /// Type of the Node
    const std::string type;
    /// Config information for node creation (Description, Type, Default)
    std::vector<std::tuple<std::string, ConfigOptions, std::string>> config;
    /// Input Ports of the Node
    std::vector<InputPort> in;
    /// Output Data Types of the Node
    std::vector<std::string> out;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

/// Global Definition of all possible Nodes
const std::vector<NodeInterface> nodeInterfaces = {
    { .type = "VectorNavFile",
      .config = { { "Path", CONFIG_STRING, "" } },
      .in = {},
      .out = { "VectorNavObs", "ImuObs" } },

    { .type = "VectorNavSensor",
      .config = { { "Frequency", CONFIG_UINT, "4" },
                  { "Port", CONFIG_STRING, "/dev/ttyUSB0" },
                  { "[Baudrate]", CONFIG_UINT, "" } },
      .in = {},
      .out = { "VectorNavObs" } },

    { .type = "VectorNavDataLogger",
      .config = { { "Path", CONFIG_STRING, "logs/vn-log.csv" },
                  { "ascii/binary", CONFIG_STRING, "ascii" } },
      .in = { { "VectorNavObs", NAV::VectorNavDataLogger::writeObservation } },
      .out = {} },

    { .type = "UbloxSensor",
      .config = { { "Frequency", CONFIG_UINT, "4" },
                  { "Port", CONFIG_STRING, "/dev/ttyUSB0" },
                  { "[Baudrate]", CONFIG_UINT, "" } },
      .in = {},
      .out = { { "UbloxObs" } } },

    { .type = "UbloxDataLogger",
      .config = { { "Path", CONFIG_STRING, "logs/ub-log.ubx" },
                  { "ascii/binary", CONFIG_STRING, "binary" } },
      .in = { { "UbloxObs", NAV::UbloxDataLogger::writeObservation } },
      .out = {} },

    { .type = "UbloxSyncSignal",
      .config = { { "Port", CONFIG_STRING, "/dev/ttyUSB0" },
                  { "UBX/NMEA", CONFIG_STRING, "UBX" },
                  { "UBX Class", CONFIG_STRING, "RXM" },
                  { "UBX Msg Id", CONFIG_STRING, "RAWX" } },
      .in = { { "UbloxObs", NAV::UbloxSyncSignal::triggerSync } },
      .out = {} }
};

#pragma GCC diagnostic pop

/// Static Class for creating Nodes from program options
class NodeCreator
{
  public:
    /**
     * @brief Creates Nodes from the program options
     * 
     * @param[in] pConfig The program options
     * @retval NavStatus Indicates if the creation was successfull
     */
    static NavStatus createNodes(NAV::Config* pConfig);

    /**
     * @brief Links Node callbacks depending on the program options
     * 
     * @param[in] pConfig The program options
     * @retval NavStatus Indicates if the linking was successfull
     */
    static NavStatus createLinks(NAV::Config* pConfig);

  private:
    NodeCreator() {}
};

} // namespace NAV
