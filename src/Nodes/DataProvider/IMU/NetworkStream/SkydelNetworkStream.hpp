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
    constexpr static size_t OutputPortIndex_ImuObs = 0;  ///< @brief Port number of the Skydel-ImuObs output
    constexpr static size_t OutputPortIndex_GnssObs = 1; ///< @brief Port number of the Skydel-GnssObs output

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
    int packageCount = 0;

    // # of packages for averaging dataRate (minimum is '2', since two time points are required to calculate a data rate)
    int packagesNumber = 2;

    // Data rate of the received network stream [Hz]
    double dataRate = 0.0;

    // Counter for packages that are skipped until data rate is shown
    int startCounter = 0;

    // # of packages that are skipped until data rate is shown
    int startNow = 20;

    Eigen::Vector3d firstPosLLA = Eigen::Vector3d::Zero();
};

} // namespace NAV