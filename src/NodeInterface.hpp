/**
 * @file NodeInterface.hpp
 * @brief Interface Definitions for all Nodes
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-08
 */

#pragma once

#include "util/Common.hpp"

#include <string>
#include <memory>
#include <functional>
#include <tuple>
#include <map>

#ifndef GUI
    #include "Nodes/Node.hpp"
    #include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
    #include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
    #include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"

    #include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
    #include "Nodes/DataLogger/GNSS/UbloxDataLogger.hpp"
    #include "Nodes/UsbSync/GNSS/UbloxSyncSignal.hpp"
#endif

#ifndef GUI
    #define NGUI(...) __VA_ARGS__
    #define NGUI_V(...) __VA_ARGS__
#else
    #define NGUI(...) nullptr
    #define NGUI_V(...) void
#endif

namespace NAV
{
/// Interface for a Node, specifying output data types and input types and callbacks
class NodeInterface
{
  public:
    /// Class representing an Input Port of a Node
    class InputPort
    {
      public:
        /// Type of the Input Node Data
        std::string type;
        /// Callback to be triggered when receiving the node data
        std::function<NavStatus(std::shared_ptr<void>, std::shared_ptr<void>)> callback;
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

    /// Catergory of the Node
    std::string category;
    /// Constructor function to be called for creating the node
    std::function<std::shared_ptr<NGUI_V(Node)>(std::string, std::vector<std::string>)> constructor;
    /// Config information for node creation (Description, Type, Default)
    std::vector<std::tuple<ConfigOptions, std::string, std::string>> config;
    /// Input Ports of the Node
    std::vector<InputPort> in;
    /// Output Data Types of the Node
    std::vector<std::string> out;

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
      { .category = "DataProvider",
        .constructor = NGUI([](std::string name, std::vector<std::string> options) { return std::make_shared<VectorNavFile>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Path", "" } },
        .in = {},
        .out = { "VectorNavObs" } } },

    { "VectorNavSensor",
      { .category = "DataProvider",
        .constructor = NGUI([](std::string name, std::vector<std::string> options) { return std::make_shared<VectorNavSensor>(name, options); }),
        .config = { { NodeInterface::CONFIG_UINT, "Frequency", "4" },
                    { NodeInterface::CONFIG_STRING, "Port", "/dev/ttyUSB0" },
                    { NodeInterface::CONFIG_LIST, "Baudrate", "[Fastest]|9600|19200|38400|57600|115200|128000|230400|460800|921600" } },
        .in = {},
        .out = { "VectorNavObs" } } },

    { "VectorNavDataLogger",
      { .category = "DataLogger",
        .constructor = NGUI([](std::string name, std::vector<std::string> options) { return std::make_shared<VectorNavDataLogger>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Path", "logs/vn-log.csv" },
                    { NodeInterface::CONFIG_LIST, "Type", "[ascii]|binary" } },
        .in = { { "VectorNavObs", NGUI(VectorNavDataLogger::writeObservation) } },
        .out = {} } },

    { "UbloxSensor",
      { .category = "DataProvider",
        .constructor = NGUI([](std::string name, std::vector<std::string> options) { return std::make_shared<UbloxSensor>(name, options); }),
        .config = { { NodeInterface::CONFIG_UINT, "Frequency", "4" },
                    { NodeInterface::CONFIG_STRING, "Port", "/dev/ttyACM0" },
                    { NodeInterface::CONFIG_LIST, "Baudrate", "[Fastest]|9600|19200|38400|57600|115200|128000|230400|460800|921600" } },
        .in = {},
        .out = { { "UbloxObs" } } } },

    { "UbloxDataLogger",
      { .category = "DataLogger",
        .constructor = NGUI([](std::string name, std::vector<std::string> options) { return std::make_shared<UbloxDataLogger>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Path", "logs/ub-log.ubx" },
                    { NodeInterface::CONFIG_LIST, "Type", "ascii|[binary]" } },
        .in = { { "UbloxObs", NGUI(UbloxDataLogger::writeObservation) } },
        .out = {} } },

    { "UbloxSyncSignal",
      { .category = "Utility",
        .constructor = NGUI([](std::string name, std::vector<std::string> options) { return std::make_shared<UbloxSyncSignal>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Port", "/dev/ttyUSB0" },
                    { NodeInterface::CONFIG_LIST, "Protocol", "[UBX]|NMEA" },
                    { NodeInterface::CONFIG_LIST, "Class", "RXM" },
                    { NodeInterface::CONFIG_LIST, "Msg Id", "RAWX" } },
        .in = { { "UbloxObs", NGUI(UbloxSyncSignal::triggerSync) } },
        .out = {} } }
};

#pragma GCC diagnostic pop

} // namespace NAV