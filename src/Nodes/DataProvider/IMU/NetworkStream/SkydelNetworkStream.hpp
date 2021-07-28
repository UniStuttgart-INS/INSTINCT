/// @file SkydelNetworkStream.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-04-19

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include <cstdlib>
#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <vector>

#include "util/InsTime.hpp"

namespace NAV
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
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

  private:
    /// @brief Port number of the Skydel-ImuObs output and GnssObs output
    constexpr static size_t OutputPortIndex_ImuObs = 1;
    constexpr static size_t OutputPortIndex_GnssObs = 2;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    // Asynchronous receive fct
    boost::asio::io_context ioservice;

    /// @brief Receive Skydel network stream data
    void do_receive();

    // Thread for receiver fct
    std::thread TestThread;

    // Network data stream buffer size (boost::asio)
    constexpr static unsigned int max_length = 1024;

    // Network data stream array
    std::array<char, max_length> m_data{};

    // boost setup
    boost::asio::ip::udp::endpoint m_senderEndpoint;
    boost::asio::ip::udp::socket m_socket;

    // Stop handler: once true, the asynchronous receive function stops
    bool stop = false;
    // Startup handler: used in 'initialize()' to differentiate between startup and re-initialization
    bool isStartup = true;

    // Time point where the first package has been received
    std::chrono::steady_clock::time_point startPoint;

    // Counter for received packages
    int packageCount;

    // # of packages for averaging dataRate (minimum is '2', since two time points are required to calculate a dataRate)
    constexpr static unsigned int packagesNumber = 1000;

    // DataRate of the incoming stream [Hz]
    double dataRate;
};

} // namespace NAV