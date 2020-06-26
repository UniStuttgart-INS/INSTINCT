/**
 * @file UbloxSyncSignal.hpp
 * @brief 
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-20
 */

#pragma once

#ifndef DISABLE_SENSORS

    #include "../UsbSyncSignal.hpp"
    #include "util/Ublox/UbloxTypes.hpp"
    #include "NodeData/GNSS/UbloxObs.hpp"

namespace NAV
{
class UbloxSyncSignal final : public UsbSyncSignal
{
  public:
    /**
     * @brief Construct a new Usb Sync Signal object
     * 
     * @param[in] name Name of the Object
     * @param[in] options Program options string map
     */
    UbloxSyncSignal(const std::string& name, const std::map<std::string, std::string>& options);

    UbloxSyncSignal() = default;                                 ///< Default Constructor
    ~UbloxSyncSignal() final = default;                          ///< Destructor
    UbloxSyncSignal(const UbloxSyncSignal&) = delete;            ///< Copy constructor
    UbloxSyncSignal(UbloxSyncSignal&&) = delete;                 ///< Move constructor
    UbloxSyncSignal& operator=(const UbloxSyncSignal&) = delete; ///< Copy assignment operator
    UbloxSyncSignal& operator=(UbloxSyncSignal&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("UbloxSyncSignal");
    }

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("TimeSync");
    }

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> The gui configuration
     */
    [[nodiscard]] std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> guiConfig() const final
    {
        return { { Node::CONFIG_STRING, "Port", "COM port where the sensor, which should be synced, is attached to\n"
                                                "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                                                "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                                                "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                                                "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                                                "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)",
                   { "/dev/ttyUSB0" } },
                 { Node::CONFIG_LIST, "Protocol", "Protocol type", { "[UBX]", "NMEA" } },
                 { Node::CONFIG_LIST, "Class", "Message Class", { "RXM" } },
                 { Node::CONFIG_LIST, "Msg Id", "Message Id", { "RAWX" } } };
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
            return 1U;
        case PortType::Out:
            break;
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
            if (portIndex == 0)
            {
                return UbloxObs().type();
            }
        case PortType::Out:
            break;
        }

        return std::string_view("");
    }

    /**
     * @brief Handles the data sent on the input port
     * 
     * @param[in] portIndex The input port index
     * @param[in, out] data The data send on the input port
     */
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) final
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<UbloxObs>(data);
            triggerSync(obs);
        }
    }
    /**
     * @brief Requests the node to send out its data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t /* portIndex */) final { return nullptr; }

    /**
     * @brief Requests the node to peek its output data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t /* portIndex */) final { return nullptr; }

  private:
    /**
     * @brief Triggers a Sync Signal to the USB port
     * 
     * @param[in] obs The received observation
     */
    void triggerSync(const std::shared_ptr<UbloxObs>& obs);

    /// Message Class to send the sync on
    ublox::UbxClass triggerClass = ublox::UbxClass::UBX_CLASS_NONE;

    /// Message Id to send the sync on
    uint8_t triggerId = 0;
};

} // namespace NAV

#endif