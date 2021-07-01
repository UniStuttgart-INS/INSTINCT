/// @file VectorNavImuObs.hpp
/// @brief Data storage class for one VectorNavImu observation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "ImuObs.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class VectorNavImuObs final : public ImuObs
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit VectorNavImuObs(const ImuPos& imuPos)
        : ImuObs(imuPos) {}

    /// @brief Default constructor
    VectorNavImuObs() = delete;
    /// @brief Destructor
    ~VectorNavImuObs() final = default;
    /// @brief Copy constructor
    VectorNavImuObs(const VectorNavImuObs&) = delete;
    /// @brief Move constructor
    VectorNavImuObs(VectorNavImuObs&&) = delete;
    /// @brief Copy assignment operator
    VectorNavImuObs& operator=(const VectorNavImuObs&) = delete;
    /// @brief Move assignment operator
    VectorNavImuObs& operator=(VectorNavImuObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("VectorNavImuObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { ImuObs::type() };
    }

    /// The time interval that the delta angle and velocities are integrated over in [seconds].
    std::optional<double> dtime;
    /// The delta rotation angles in [degree] incurred due to rotation, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    std::optional<Eigen::Vector3d> dtheta;
    /// The delta velocity in [m/s] incurred due to motion, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    std::optional<Eigen::Vector3d> dvel;
};

} // namespace NAV
