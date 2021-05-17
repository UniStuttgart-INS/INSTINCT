/// @file SkydelImuStream.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-04-19

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

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

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

    void do_receive();

    // Asynchronous receive fct
    //void async_receive_from();
    boost::asio::io_context ioservice;

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 1; ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    // Thread for receiver fct
    int ReceiverThread();

    std::thread TestThread;
    enum
    {
        max_length = 1024
    };

    uint16_t port;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    std::array<char, max_length> data_;
    boost::asio::ip::udp::socket socket_;
    // bool breakStream;

    // InsTime startTime;
    // uint64_t timeSinceStartupStart = 0;

    /*
    /// The Imu type
    ImuType imuType = ImuType::MPU;

    /// OutputFrequency to calculate rateDivisor field.
    int outputFrequency = 100;

    /// Timer object to handle async data requests
    CallbackTimer timer;
    /// Start Time to calculate the TimeSinceStartup
    std::chrono::time_point<std::chrono::steady_clock> startTime;

    /// Accelerometer X data, which are read into by the sensor
    float ax{};
    /// Accelerometer Y data, which are read into by the sensor
    float ay{};
    /// Accelerometer Z data, which are read into by the sensor
    float az{};
    /// Gyroscope X data, which are read into by the sensor
    float gx{};
    /// Gyroscope Y data, which are read into by the sensor
    float gy{};
    /// Gyroscope Z data, which are read into by the sensor
    float gz{};
    /// Magnetometer X data, which are read into by the sensor
    float mx{};
    /// Magnetometer Y data, which are read into by the sensor
    float my{};
    /// Magnetometer Z data, which are read into by the sensor
    float mz{};
*/
    /// @brief Function which performs the async data reading
    /// @param[in, out] userData Pointer to the SkydelImuStream object
    static void readImuThread(void* userData);
};

} // namespace NAV