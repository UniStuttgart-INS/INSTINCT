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
#include <map>

#include "util/Common.hpp"
#include "util/Config.hpp"

#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"

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
    CONFIG_UINT,   ///< Unsigned Integer
    CONFIG_INT,    ///< Integer
    CONFIG_FLOAT,  ///< Float
    CONFIG_STRING, ///< String
    CONFIG_LIST,   ///< List, use '|' to separate and [...] to specify the default option
};

/// Interface for a Node, specifying output data types and input types and callbacks
class NodeInterface
{
  public:
    /// Constructor function to be called for creating the node
    std::function<std::shared_ptr<NAV::Node>(std::string, std::vector<std::string>)> constructor;
    /// Config information for node creation (Description, Type, Default)
    std::vector<std::tuple<ConfigOptions, std::string, std::string>> config;
    /// Input Ports of the Node
    std::vector<InputPort> in;
    /// Output Data Types of the Node
    std::vector<std::string> out;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

/// This struct represents the inheritance structure for output data, as parent data can always be extracted from child data
const std::map<std::string, std::vector<std::string>> inheritance = {
    { "ImuObs", { "InsObs" } },
    { "VectorNavObs", { "ImuObs" } },
    { "GnssObs", { "InsObs" } },
    { "UbloxObs", { "GnssObs" } },
};

/// Global Definition of all possible Nodes
const std::map<std::string, NodeInterface> nodeInterfaces = {
    { "VectorNavFile",
      { .constructor = [](std::string name, std::vector<std::string> options) { return std::make_shared<NAV::VectorNavFile>(name, options); },
        .config = { { CONFIG_STRING, "Path", "" } },
        .in = {},
        .out = { "VectorNavObs" } } },

    { "VectorNavSensor",
      { .constructor = [](std::string name, std::vector<std::string> options) { return std::make_shared<NAV::VectorNavSensor>(name, options); },
        .config = { { CONFIG_UINT, "Frequency", "4" },
                    { CONFIG_STRING, "Port", "/dev/ttyUSB0" },
                    { CONFIG_LIST, "Baudrate", "[Fastest]|9600|19200|38400|57600|115200|128000|230400|460800|921600" } },
        .in = {},
        .out = { "VectorNavObs" } } },

    { "VectorNavDataLogger",
      { .constructor = [](std::string name, std::vector<std::string> options) { return std::make_shared<NAV::VectorNavDataLogger>(name, options); },
        .config = { { CONFIG_STRING, "Path", "logs/vn-log.csv" },
                    { CONFIG_STRING, "ascii/binary", "ascii" } },
        .in = { { "VectorNavObs", NAV::VectorNavDataLogger::writeObservation } },
        .out = {} } },

    { "UbloxSensor",
      { .constructor = [](std::string name, std::vector<std::string> options) { return std::make_shared<NAV::UbloxSensor>(name, options); },
        .config = { { CONFIG_UINT, "Frequency", "4" },
                    { CONFIG_STRING, "Port", "/dev/ttyACM0" },
                    { CONFIG_LIST, "Baudrate", "[Fastest]|9600|19200|38400|57600|115200|128000|230400|460800|921600" } },
        .in = {},
        .out = { { "UbloxObs" } } } },

    { "UbloxDataLogger",
      { .constructor = [](std::string name, std::vector<std::string> options) { return std::make_shared<NAV::UbloxDataLogger>(name, options); },
        .config = { { CONFIG_STRING, "Path", "logs/ub-log.ubx" },
                    { CONFIG_STRING, "ascii/binary", "binary" } },
        .in = { { "UbloxObs", NAV::UbloxDataLogger::writeObservation } },
        .out = {} } },

    { "UbloxSyncSignal",
      { .constructor = [](std::string name, std::vector<std::string> options) { return std::make_shared<NAV::UbloxSyncSignal>(name, options); },
        .config = { { CONFIG_STRING, "Port", "/dev/ttyUSB0" },
                    { CONFIG_STRING, "UBX/NMEA", "UBX" },
                    { CONFIG_STRING, "UBX Class", "RXM" },
                    { CONFIG_STRING, "UBX Msg Id", "RAWX" } },
        .in = { { "UbloxObs", NAV::UbloxSyncSignal::triggerSync } },
        .out = {} } }
};

#pragma GCC diagnostic pop

/// Static Class for creating Nodes from program options
class NodeCreator
{
  public:
    /**
     * @brief Get the Callback Port for the message type of the interface
     * 
     * @param[in] interfaceType Name of the Node Interface
     * @param[in] messageType Name of the Message Type
     * @retval size_t The callback port number
     */
    static size_t getCallbackPort(std::string interfaceType, std::string messageType, bool inPort = false);

    /**
     * @brief Checks if the target Type equals the message type of its base
     * 
     * @param[in] targetType String of target type
     * @param[in] messageType String of message type
     * @retval bool Returns if the types are compatible
     */
    static bool isTypeOrBase(std::string targetType, std::string messageType);

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
