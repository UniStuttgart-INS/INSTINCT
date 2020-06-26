/**
 * @file EmlidSensor.hpp
 * @brief Emlid Sensor Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-06-23
 */

#pragma once

#ifndef DISABLE_SENSORS

    #include "NodeData/GNSS/EmlidObs.hpp"
    #include "../Gnss.hpp"
    #include "../../Protocol/UartSensor.hpp"
    #include "er/sensors/sensors.hpp"

namespace NAV
{
/// Emlid Sensor Class
class EmlidSensor final : public UartSensor, public Gnss
{
  public:
    /// Config Structure for the sensor
    typedef struct Config
    {
        /// OutputFrequency of async packages
        uint16_t outputFrequency = 1;
    } Config;

    /**
     * @brief Construct a new Emlid Sensor object
     * 
     * @param[in] name Name of the Sensor
     * @param[in, out] options Program options string list
     */
    EmlidSensor(const std::string& name, const std::map<std::string, std::string>& options);

    EmlidSensor() = default;                             ///< Default Constructor
    ~EmlidSensor() final;                                ///< Destructor
    EmlidSensor(const EmlidSensor&) = delete;            ///< Copy constructor
    EmlidSensor(EmlidSensor&&) = delete;                 ///< Move constructor
    EmlidSensor& operator=(const EmlidSensor&) = delete; ///< Copy assignment operator
    EmlidSensor& operator=(EmlidSensor&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("EmlidSensor");
    }

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataProvider");
    }

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<ConfigOptions> The gui configuration
     */
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        return { { CONFIG_STRING, "Port", "COM port where the sensor is attached to\n"
                                          "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                                          "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                                          "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                                          "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                                          "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)",
                   { "/dev/ttyUSB0" } },
                 { CONFIG_LIST, "Baudrate", "Target Baudrate for the sensor", { "[Fastest]", "9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600" } },
                 { CONFIG_INT, "Frequency", "Data Output Frequency", { "0", "4", "200" } } };
    }

    /**
     * @brief Returns the context of the class
     * 
     * @retval constexpr std::string_view The class context
     */
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::REAL_TIME;
    }

    /**
     * @brief Returns the number of Ports
     * 
     * @param[in] portType Specifies the port type
     * @retval constexpr uint8_t The number of ports
     */
    [[nodiscard]] constexpr uint8_t nPorts(PortType portType) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            return 1U;
        }

        return 0U;
    }

    /**
     * @brief Returns the data types provided by this class
     * 
     * @param[in] portType Specifies the port type
     * @param[in] portIndex Port index on which the data is sent
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return EmlidObs().type();
            }
        }

        return std::string_view("");
    }

    /**
     * @brief Handles the data sent on the input port
     * 
     * @param[in] portIndex The input port index
     * @param[in, out] data The data send on the input port
     */
    void handleInputData(uint8_t /* portIndex */, std::shared_ptr<NodeData> /* data */) final {}

  private:
    /**
     * @brief Callback handler for notifications of new asynchronous data packets received
     * 
     * @param[in, out] userData Pointer to the data we supplied when we called registerAsyncPacketReceivedHandler
     * @param[in] p Encapsulation of the data packet. At this state, it has already been validated and identified as an asynchronous data message
     * @param[in] index Advanced usage item and can be safely ignored for now
     */
    static void asciiOrBinaryAsyncMessageReceived(void* userData, er::protocol::uart::Packet& p, size_t index);

    /// erSensor Object
    er::sensors::ErSensor er;

    /// Config Object
    EmlidSensor::Config config;
};

} // namespace NAV

#endif
