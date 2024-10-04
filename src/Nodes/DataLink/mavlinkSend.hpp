// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file mavlinkSend.hpp
/// @brief Sends a navigation solution via the MAVLink protocol
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-11-20

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/State/PosVelAtt.hpp"

#include <mavlink/common/mavlink.h>

#include "util/Vendor/MavLink/autopilot_interface.hpp"
#include "util/Vendor/MavLink/serial_port.hpp"
#include "util/Vendor/MavLink/udp_port.hpp"

namespace NAV
{
/// @brief MavLink Send node
class MavlinkSend : public Node
{
  public:
    /// @brief Default constructor
    MavlinkSend();
    /// @brief Destructor
    ~MavlinkSend() override;
    /// @brief Copy constructor
    MavlinkSend(const MavlinkSend&) = delete;
    /// @brief Move constructor
    MavlinkSend(MavlinkSend&&) = delete;
    /// @brief Copy assignment operator
    MavlinkSend& operator=(const MavlinkSend&) = delete;
    /// @brief Move assignment operator
    MavlinkSend& operator=(MavlinkSend&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t INPUT_PORT_INDEX_NODE_DATA = 0; ///< @brief Object (NodeData)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Callback when receiving data on a port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receivePosVelAtt(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// Available PortTypes
    enum class PortType
    {
        Serial_Port, /// Serial_Port
        UDP_Port,    /// UDP_Port
    };
    PortType _portType = PortType::UDP_Port;

    /// IPv4 address
    std::array<int, 4> _ip{ 127, 0, 0, 1 };

    /// @brief Helper function
    const char* convertArrayToIPAddress(const std::array<int, 4>& ipArray);

    /// UDP port number
    int _portNumber = 14540;

    /// Range an IPv4 address can be in [0, 2^8-1]
    static constexpr std::array<int, 2> IP_LIMITS = { 0, 255 };
    /// Range a port can be in [0, 2^16-1]
    static constexpr std::array<int, 2> PORT_LIMITS = { 0, 65535 };

    /// COM port where the MavLink device is attached to
    ///
    /// - "COM1" (Windows format for physical and virtual (USB) serial port)
    /// - "/dev/ttyS1" (Linux format for physical serial port)
    /// - "/dev/ttyUSB0" (Linux format for virtual (USB) serial port)
    /// - "/dev/tty.usbserial-FTXXXXXX" (Mac OS X format for virtual (USB) serial port)
    /// - "/dev/ttyS0" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)
    std::string _serialPort;

    /// Available Baudrates
    enum class Baudrate
    {
        BAUDRATE_1200,    ///< Baudrate with    1200 symbols per second [Baud]
        BAUDRATE_2400,    ///< Baudrate with    2400 symbols per second [Baud]
        BAUDRATE_4800,    ///< Baudrate with    4800 symbols per second [Baud]
        BAUDRATE_9600,    ///< Baudrate with    9600 symbols per second [Baud]
        BAUDRATE_19200,   ///< Baudrate with   19200 symbols per second [Baud]
        BAUDRATE_38400,   ///< Baudrate with   38400 symbols per second [Baud]
        BAUDRATE_57600,   ///< Baudrate with   57600 symbols per second [Baud]
        BAUDRATE_111100,  ///< Baudrate with  111100 symbols per second [Baud]
        BAUDRATE_115200,  ///< Baudrate with  115200 symbols per second [Baud]
        BAUDRATE_230400,  ///< Baudrate with  230400 symbols per second [Baud]
        BAUDRATE_256000,  ///< Baudrate with  256000 symbols per second [Baud]
        BAUDRATE_460800,  ///< Baudrate with  460800 symbols per second [Baud]
        BAUDRATE_500000,  ///< Baudrate with  500000 symbols per second [Baud]
        BAUDRATE_921600,  ///< Baudrate with  921600 symbols per second [Baud]
        BAUDRATE_1500000, ///< Baudrate with 1500000 symbols per second [Baud]
        COUNT             /// Count Variable
    };
    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(Baudrate value);

    /// @brief Converts the enum to int
    /// @param[in] value Enum value to convert into int
    /// @return int representation of the enum
    int getBaudrateValue(Baudrate value);

    /// Selected Baudrate for the Serialport in the GUI
    Baudrate _baudrate = Baudrate::BAUDRATE_57600;

    /// Determines whether the Mavlink message "GPS_INPUT (#232)" is sent
    bool _GPS_INPUT_Active = false;

    /// Determines how often the Mavlink message "GPS_INPUT (#232)" is sent
    double _GPS_INPUT_Frequency = 10;

    /// Determines whether the Mavlink message "ATT_POS_MOCAP (#138)" is sent
    bool _ATT_POS_MOCAP_Active = false;

    /// Determines how often the Mavlink message "ATT_POS_MOCAP (#138)" is sent
    double _ATT_POS_MOCAP_Frequency = 10;
};
} // namespace NAV