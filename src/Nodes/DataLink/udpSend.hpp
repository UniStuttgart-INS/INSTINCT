// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file udpSend.hpp
/// @brief Asynchronous data link - sender node
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-07-19

#pragma once

#ifdef _WIN32
    // Set the proper SDK version before including boost/Asio
    #include <SDKDDKVer.h>
    // Note boost/ASIO includes Windows.h.
    #include <boost/asio.hpp>
#else // _WIN32
    #include <boost/asio.hpp>
#endif //_WIN32

#include "internal/Node/Node.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include <string>

namespace NAV
{
/// UDP Client
class UdpSend : public Node
{
  public:
    /// @brief Default constructor
    UdpSend();
    /// @brief Destructor
    ~UdpSend() override;
    /// @brief Copy constructor
    UdpSend(const UdpSend&) = delete;
    /// @brief Move constructor
    UdpSend(UdpSend&&) = delete;
    /// @brief Copy assignment operator
    UdpSend& operator=(const UdpSend&) = delete;
    /// @brief Move assignment operator
    UdpSend& operator=(UdpSend&&) = delete;

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

    /// IPv4 address
    std::array<int, 4> _ip{};

    /// UDP port number
    int _port = 4567;

    /// Range an IPv4 address can be in [0, 2^8-1]
    static constexpr std::array<int, 2> IP_LIMITS = { 0, 255 };
    /// Range a port can be in [0, 2^16-1]
    static constexpr std::array<int, 2> PORT_LIMITS = { 0, 65535 };

    /// Asynchronous receive fct
    boost::asio::io_context _io_context;
    /// Boost udp socket
    boost::asio::ip::udp::socket _socket;
    /// Boost udp resolver
    boost::asio::ip::udp::resolver _resolver;
    /// Boost udp endpoint
    boost::asio::ip::udp::resolver::results_type _endpoints;
};
} // namespace NAV