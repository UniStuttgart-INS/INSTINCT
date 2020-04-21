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
#include <deque>
#include <vector>

#ifndef GUI
    #include "Nodes/Node.hpp"
    #include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
    #include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
    #include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"

    #include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
    #include "Nodes/DataLogger/GNSS/UbloxDataLogger.hpp"
    #include "Nodes/UsbSync/GNSS/UbloxSyncSignal.hpp"

    #include "Nodes/GnuPlot/IMU/VectorNavGnuPlot.hpp"
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
        std::function<NavStatus(std::shared_ptr<NGUI_V(NodeData)>, std::shared_ptr<NGUI_V(Node)>)> callback;
    };

    /// Enum Providing possible types for program options
    enum ConfigOptions
    {
        CONFIG_BOOL,          ///< Boolean: Default
        CONFIG_INT,           ///< Integer: Min, Default, Max
        CONFIG_FLOAT,         ///< Float: Min, Default, Max
        CONFIG_STRING,        ///< String
        CONFIG_LIST,          ///< List: "option1|[default]|option3"
        CONFIG_LIST_LIST_INT, ///< 2 Lists and Integer: "[List1default]|List1option2||List2option1|[List2default]||min|default|max"
        CONFIG_MAP_INT,       ///< String Key and Integer Value: "key", "min", "default", "max"
    };

    enum NodeContext
    {
        REAL_TIME,
        POST_PROCESSING,
        ALL
    };

    /// Catergory of the Node
    std::string category;
    /// Node Compatibility
    NodeContext nodeCompat;
    /// Constructor function to be called for creating the node
    std::function<std::shared_ptr<NGUI_V(Node)>(std::string, std::deque<std::string>&)> constructor;
    /// Config information for node creation (Type, Description, ToolTip, Options)
    std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> config;
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

/// Specifies if the Application runs in real-time or post processing mode
extern NodeInterface::NodeContext appContext;

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
        .nodeCompat = NodeInterface::NodeContext::POST_PROCESSING,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<VectorNavFile>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Path", "Path to the File to read", { "" } } },
        .in = {},
        .out = { "VectorNavObs" } } },

    { "VectorNavSensor",
      { .category = "DataProvider",
        .nodeCompat = NodeInterface::NodeContext::REAL_TIME,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<VectorNavSensor>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Port", "COM port where the sensor is attached to\n- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n- \"/dev/ttyS1\" (Linux format for physical serial port)\n- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)", { "/dev/ttyUSB0" } },
                    { NodeInterface::CONFIG_LIST, "Baudrate", "Target Baudrate for the sensor", { "[Fastest]", "9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600" } },
                    { NodeInterface::CONFIG_INT, "Frequency", "Data Output Frequency", { "0", "100", "200" } } },
        .in = {},
        .out = { "VectorNavObs" } } },

    { "VectorNavDataLogger",
      { .category = "DataLogger",
        .nodeCompat = NodeInterface::NodeContext::ALL,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<VectorNavDataLogger>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Path", "Path where to save the data to", { "logs/vn-log.csv" } },
                    { NodeInterface::CONFIG_LIST, "Type", "Type of the output file", { "[ascii]", "binary" } } },
        .in = { { "VectorNavObs", NGUI(VectorNavDataLogger::writeObservation) } },
        .out = {} } },

    { "UbloxSensor",
      { .category = "DataProvider",
        .nodeCompat = NodeInterface::NodeContext::REAL_TIME,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<UbloxSensor>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Port", "COM port where the sensor is attached to", { "/dev/ttyACM0" } },
                    { NodeInterface::CONFIG_LIST, "Baudrate", "Target Baudrate for the sensor", { "[Fastest]", "9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600" } },
                    { NodeInterface::CONFIG_INT, "Frequency", "Data Output Frequency", { "0", "4", "200" } } },
        .in = {},
        .out = { { "UbloxObs" } } } },

    { "UbloxDataLogger",
      { .category = "DataLogger",
        .nodeCompat = NodeInterface::NodeContext::ALL,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<UbloxDataLogger>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Path", "Path where to save the data to", { "logs/ub-log.ubx" } },
                    { NodeInterface::CONFIG_LIST, "Type", "Type of the output file", { "ascii", "[binary]" } } },
        .in = { { "UbloxObs", NGUI(UbloxDataLogger::writeObservation) } },
        .out = {} } },

    { "UbloxSyncSignal",
      { .category = "Utility",
        .nodeCompat = NodeInterface::NodeContext::REAL_TIME,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<UbloxSyncSignal>(name, options); }),
        .config = { { NodeInterface::CONFIG_STRING, "Port", "COM port where the sensor, which should be synced, is attached to\n- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n- \"/dev/ttyS1\" (Linux format for physical serial port)\n- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)", { "/dev/ttyUSB0" } },
                    { NodeInterface::CONFIG_LIST, "Protocol", "Protocol type", { "[UBX]", "NMEA" } },
                    { NodeInterface::CONFIG_LIST, "Class", "Message Class", { "RXM" } },
                    { NodeInterface::CONFIG_LIST, "Msg Id", "Message Id", { "RAWX" } } },
        .in = { { "UbloxObs", NGUI(UbloxSyncSignal::triggerSync) } },
        .out = {} } },

    { "VectorNavGnuPlot",
      { .category = "Plot",
        .nodeCompat = NodeInterface::NodeContext::ALL,
        .constructor = NGUI([](std::string name, std::deque<std::string>& options) { return std::make_shared<VectorNavGnuPlot>(name, options); }),
        .config = {
            { NodeInterface::CONFIG_FLOAT, "X Display Scope", "Data older/smaller than the specified scope gets discarded.\ne.g. Shows only the last x seconds.\n\nOnly for Real-Time data", { "0", "10", "100" } },
            { NodeInterface::CONFIG_FLOAT, "Update Frequency", "Frequency to update the Plot Windows\n\nOnly for Real-Time data", { "0", "50", "200" } },
            { NodeInterface::CONFIG_LIST_LIST_INT, "Data to plot", "Specify what data should be plotted.\n\nData with the same Window Id gets plotted into the same GnuPlot window.\nWindow Id '-1' disables the plot.", { "[timeSinceStartup]", "timeSinceStartup|[quaternion]|magUncompXYZ|accelUncompXYZ|gyroUncompXYZ|magCompXYZ|accelCompXYZ|gyroCompXYZ|syncInCnt|dtime|dtheta|dvel|vpeStatus|temperature|pressure|magCompNED|accelCompNED|gyroCompNED|linearAccelXYZ|linearAccelNED|yawPitchRollUncertainty", "-1|-1|100" } } },
        .in = { { "VectorNavObs", NGUI(VectorNavGnuPlot::plotVectorNavObs) } },
        .out = {} } }
};

#pragma GCC diagnostic pop

} // namespace NAV