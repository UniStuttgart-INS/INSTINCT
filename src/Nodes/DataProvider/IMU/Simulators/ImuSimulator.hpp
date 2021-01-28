/// @file ImuSimulator.hpp
/// @brief Imu Observation Simulator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "util/Eigen.hpp"
#include "util/InsTime.hpp"

namespace NAV
{
/// Imu Observation Simulator
class ImuSimulator : public Imu
{
  public:
    /// @brief Default constructor
    ImuSimulator();
    /// @brief Destructor
    ~ImuSimulator() override;
    /// @brief Copy constructor
    ImuSimulator(const ImuSimulator&) = delete;
    /// @brief Move constructor
    ImuSimulator(ImuSimulator&&) = delete;
    /// @brief Copy assignment operator
    ImuSimulator& operator=(const ImuSimulator&) = delete;
    /// @brief Move assignment operator
    ImuSimulator& operator=(ImuSimulator&&) = delete;

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

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 1;   ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_StateData = 0; ///< @brief Object (StateData)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls the next simulated data
    /// @param[in] peek Specifies if the data should be peeked or read
    /// @return The simulated observation
    [[nodiscard]] std::shared_ptr<NodeData> pollData(bool peek = false);

    /// Global starttime
    InsTime startTime;

    /// Duration of the data gerneation in [s]
    double duration = 1.0;
    /// Frequency of the data generation in [Hz]
    double frequency = 10.0;
    /// Current Simulation Time in [s]
    double currentSimTime = 0.0;

    /// Acceleration in navigation frame in [m/s^2]
    Eigen::Vector3f accel_n{ 0, 0, 0 };
    /// Acceleration in body frame in [m/s^2]
    Eigen::Vector3f accel_b{ 0, 0, 0 };
    /// Acceleration in platform frame in [m/s^2]
    Eigen::Vector3f accel_p{ 0, 0, 0 };
    /// Angular velocity in navigation frame in [rad/s]
    Eigen::Vector3f gyro_n{ 0, 0, 0 };
    /// Angular velocity in body frame in [rad/s]
    Eigen::Vector3f gyro_b{ 0, 0, 0 };
    /// Angular velocity in platform frame in [rad/s]
    Eigen::Vector3f gyro_p{ 0, 0, 0 };
    /// Magnetic field in navigation frame in [Gauss]
    Eigen::Vector3f mag_n{ 0, 0, 0 };
    /// Magnetic field in body frame in [Gauss]
    Eigen::Vector3f mag_b{ 0, 0, 0 };
    /// Magnetic field in platform frame in [Gauss]
    Eigen::Vector3f mag_p{ 0, 0, 0 };
    /// Temperature measured in units of [Celsius]
    double temperature = 20.0;
};

} // namespace NAV
