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

#include <cstddef>
#include "NodeData/NodeData.hpp"
#ifdef _WIN32
    // Set the proper SDK version before including boost/Asio
    #include <SDKDDKVer.h>
    // Note boost/ASIO includes Windows.h.
    #include <boost/asio.hpp>
#else // _WIN32
    #include <boost/asio.hpp>
#endif //_WIN32

#include "internal/Node/Node.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

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
    void receiveData(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// IPv4 address
    std::array<int, 4> _ip{};

    /// UDP port number
    int _port = 4567;

    /// Message Type: 0 = posVelAtt, 1 = gnssObs
    int _msgType = 0;

    /// Range an IPv4 address can be in [0, 2^8-1]
    static constexpr std::array<int, 2> IP_LIMITS = { 0, 255 };
    /// Range a port can be in [0, 2^16-1]
    static constexpr std::array<int, 2> PORT_LIMITS = { 0, 65535 };

    /// Network data stream maximum buffer size in [bytes] (Maximum payload size of a UDP package)
    constexpr static unsigned int MAXIMUM_BYTES = 65507;

    /// Size of the message type
    constexpr static size_t SIZE_MSGTYPE = sizeof(_msgType);
    /// Size of a timestamp
    constexpr static size_t SIZE_TIMESTAMP = sizeof(NodeData::insTime);
    /// Size of a Pos
    constexpr static size_t SIZE_POS = 24;
    /// Size of a Vel
    constexpr static size_t SIZE_VEL = SIZE_POS;
    /// Size of a Quaternion element
    constexpr static size_t SIZE_QUAT = 8;

    /// Offset of the timestamp
    constexpr static size_t OFFSET_TIMESTAMP = SIZE_MSGTYPE;
    /// Offset of the position
    constexpr static size_t OFFSET_POS = OFFSET_TIMESTAMP + SIZE_TIMESTAMP;
    /// Offset of the velocity
    constexpr static size_t OFFSET_VEL = OFFSET_POS + SIZE_POS;
    /// Offset of the quaternion
    constexpr static size_t OFFSET_QUAT = OFFSET_VEL + SIZE_VEL;

    /// Total size of the data
    constexpr static size_t SIZE_TOTAL = OFFSET_QUAT + 4 * SIZE_QUAT;

    /// Size of the size of a GNSS observation
    constexpr static size_t SIZE_SIZE = 8;
    /// Size of a single GNSS observation
    constexpr static size_t SIZE_SINGLE_OBSERVATION_DATA = sizeof(GnssObs::ObservationData);

    /// Offset of the GNSS data size
    constexpr static size_t OFFSET_SIZE = OFFSET_POS;
    /// Offset of the GNSS data
    constexpr static size_t OFFSET_GNSSDATA = OFFSET_SIZE + SIZE_SIZE;

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