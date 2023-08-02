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

#include "internal/Node/Node.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include <boost/asio.hpp>
#include <string>
using boost::asio::ip::udp;

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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_NODE_DATA = 0; ///< @brief Object (NodeData)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls the next data
    // void pollPosVelAtt();

    /// @brief Polls the next data
    void pollData();

    /// UDP port number
    int _port = 4567;

    /// Asynchronous receive fct
    boost::asio::io_context _io_context;
    /// Boost udp socket
    udp::socket _socket;
    /// Boost udp endpoint
    udp::endpoint _sender_endpoint;

    /// Receiver thread
    std::thread _recvThread;

    /// Flag that indicates the running data link
    bool _running = false;
    /// Startup handler: used in 'initialize()' to differentiate between startup and re-initialization
    bool _isStartup = true;

    /// Flag that indicates whether seding is stopped
    double _flagsenderstopped = 1.0;

    /// Time point where the first package has been received
    std::chrono::steady_clock::time_point _startPoint;

    /// Stores the time of the last received message
    uint64_t _lastMessageTime{};

    /// Network data stream buffer size (boost::asio)
    constexpr static unsigned int _maxLength = 10;

    constexpr static unsigned int _maxBytes = 80;

    /// Network data stream array
    // std::vector<double> _data{};
    /// Network data stream array
    std::array<double, _maxLength> _data{};
};
} // namespace NAV