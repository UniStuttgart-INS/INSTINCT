// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SkydelNetworkStream.hpp
/// @brief Node receiving UDP packages from the Skydel GNSS simulator Instinct plugin
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-04-19

#pragma once

#ifdef _WIN32
    // Set the proper SDK version before including boost/Asio
    #include <SDKDDKVer.h>
    // Note boost/ASIO includes Windows.h.
    #include <boost/asio.hpp>
#else // _WIN32
    #include <boost/asio.hpp>
#endif //_WIN32

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include <cstdlib>
#include <thread>
#include <string>
#include <vector>

#include "Navigation/Time/InsTime.hpp"

namespace NAV::experimental
{
/// SkydelNetworkStream Sensor Class
class SkydelNetworkStream : public Imu
{
  public:
    /// @brief Default constructor
    SkydelNetworkStream();
    /// @brief Destructor
    ~SkydelNetworkStream() override;
    /// @brief Copy constructor
    SkydelNetworkStream(const SkydelNetworkStream&) = delete;
    /// @brief Move constructor
    SkydelNetworkStream(SkydelNetworkStream&&) = delete;
    /// @brief Copy assignment operator
    SkydelNetworkStream& operator=(const SkydelNetworkStream&) = delete;
    /// @brief Move assignment operator
    SkydelNetworkStream& operator=(SkydelNetworkStream&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_IMU_OBS = 0;  ///< @brief Port number of the Skydel-ImuObs output
    constexpr static size_t OUTPUT_PORT_INDEX_GNSS_OBS = 1; ///< @brief Port number of the Skydel-GnssObs output

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Asynchronous receive fct
    boost::asio::io_context _ioservice;

    /// @brief Receive Skydel network stream data
    void do_receive();

    /// Thread for receiver fct
    std::thread _testThread;

    /// Network data stream buffer size (boost::asio)
    constexpr static unsigned int _maxLength = 1024;

    /// Network data stream array
    std::array<char, _maxLength> _data{};

    /// Boost udp endpoint
    boost::asio::ip::udp::endpoint _senderEndpoint;
    /// Boost udp socket
    boost::asio::ip::udp::socket _socket;

    /// Stop handler: once true, the asynchronous receive function stops
    bool _stop = false;
    /// Startup handler: used in 'initialize()' to differentiate between startup and re-initialization
    bool _isStartup = true;

    /// Time point where the first package has been received
    std::chrono::steady_clock::time_point _startPoint;

    /// Stores the time of the last received message
    uint64_t _lastMessageTime{};

    /// Counter for received packages
    int _packageCount = 0;

    /// # of packages for averaging dataRate (minimum is '2', since two time points are required to calculate a data rate)
    int _packagesNumber = 2;

    /// Data rate of the received network stream [Hz]
    double _dataRate = 0.0;

    /// Counter for packages that are skipped until data rate is shown
    int _startCounter = 0;

    /// # of packages that are skipped until data rate is shown
    int _startNow = 20;
};

} // namespace NAV::experimental