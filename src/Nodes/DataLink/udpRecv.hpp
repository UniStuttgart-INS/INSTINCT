// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file udpRecv.hpp
/// @brief Asynchronous data link - receiver node
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-07-26

#pragma once

#include <array>
#ifdef _WIN32
    // Set the proper SDK version before including boost/Asio
    #include <SDKDDKVer.h>
    // Note boost/ASIO includes Windows.h.
    #include <boost/asio.hpp>
#else
    #include <boost/asio.hpp>
#endif

#include "internal/Node/Node.hpp"
#include "UdpUtil.hpp"

#include <string>

namespace NAV
{
/// UDP Client
class UdpRecv : public Node
{
  public:
    /// @brief Default constructor
    UdpRecv();
    /// @brief Destructor
    ~UdpRecv() override;
    /// @brief Copy constructor
    UdpRecv(const UdpRecv&) = delete;
    /// @brief Move constructor
    UdpRecv(UdpRecv&&) = delete;
    /// @brief Copy assignment operator
    UdpRecv& operator=(const UdpRecv&) = delete;
    /// @brief Move assignment operator
    UdpRecv& operator=(UdpRecv&&) = delete;

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

    /// Enum specifying the type of the output message
    enum class OutputType : uint8_t
    {
        PosVelAtt, ///< Extract PosVelAtt data
        GnssObs,   ///< Extract GnssObs data
        COUNT,     ///< Number of items in the enum
    };

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_NODE_DATA = 0; ///< @brief Object (NodeData)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls the next data
    void asyncReceive();

    /// UDP port number
    int _port = 4567;

    /// The selected output type in the GUI
    OutputType _outputType = OutputType::PosVelAtt;

    /// Asynchronous receive fct
    boost::asio::io_context _io_context;
    /// Boost udp socket
    boost::asio::ip::udp::socket _socket;
    /// Boost udp endpoint
    boost::asio::ip::udp::endpoint _sender_endpoint;

    /// Receiver thread
    std::thread _recvThread;

    /// Flag that indicates the running data link
    bool _running = false;
    /// Startup handler: used in 'initialize()' to differentiate between startup and re-initialization
    bool _isStartup = true;

    /// Time point where the first package has been received
    std::chrono::steady_clock::time_point _startPoint;

    /// The array that contains the data from the UDP stream
    std::array<char, UdpUtil::MAXIMUM_BYTES> _charArray{};

    /// Message Type: 0 = posVelAtt, 1 = gnssObs
    int _msgType = 0;
};

/// @brief Converts the enum to a string
/// @param[in] value Enum value to convert into text
/// @return String representation of the enum
const char* to_string(NAV::UdpRecv::OutputType value);

} // namespace NAV