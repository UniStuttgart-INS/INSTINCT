/// @file SkydelImuStream.hpp
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
/// SkydelImuStream Sensor Class
class SkydelImuStream : public Imu
{
  public:
    /// @brief Default constructor
    SkydelImuStream();
    /// @brief Destructor
    ~SkydelImuStream() override;
    /// @brief Copy constructor
    SkydelImuStream(const SkydelImuStream&) = delete;
    /// @brief Move constructor
    SkydelImuStream(SkydelImuStream&&) = delete;
    /// @brief Copy assignment operator
    SkydelImuStream& operator=(const SkydelImuStream&) = delete;
    /// @brief Move assignment operator
    SkydelImuStream& operator=(SkydelImuStream&&) = delete;

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
    // Number of the output port of the SkydelImuStream node
    constexpr static size_t OutputPortIndex_ImuObs = 0; ///< @brief Flow (ImuObs)

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
};

} // namespace NAV