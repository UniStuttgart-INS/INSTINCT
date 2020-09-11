/// @file ImuPos.hpp
/// @brief Imu Position Data
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-11

#pragma once

#include "NodeData/NodeData.hpp"

#include "Eigen/Dense"
#include <Eigen/Geometry>

namespace NAV
{
/// IMU Position
class ImuPos : public NodeData
{
  public:
    /// @brief Default constructor
    ImuPos() = default;
    /// @brief Destructor
    ~ImuPos() override = default;
    /// @brief Copy constructor
    ImuPos(const ImuPos&) = delete;
    /// @brief Move constructor
    ImuPos(ImuPos&&) = delete;
    /// @brief Copy assignment operator
    ImuPos& operator=(const ImuPos&) = delete;
    /// @brief Move assignment operator
    ImuPos& operator=(ImuPos&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("ImuPos");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        return {};
    }

    /// Accelerometer position in body frame coordinates in [m]
    Eigen::Vector3d posAccel_b;
    /// Gyroscope position in body frame coordinates in [m]
    Eigen::Vector3d posGyro_b;
    /// Magnetometer position in body frame coordinates in [m]
    Eigen::Vector3d posMag_b;

    /// Quaternion from accelerometer platform frame to body frame
    Eigen::Quaterniond quatAccel_p2b;
    /// Quaternion from gyroscope platform frame to body frame
    Eigen::Quaterniond quatGyro_p2b;
    /// Quaternion from magnetometer platform frame to body frame
    Eigen::Quaterniond quatMag_p2b;
};

} // namespace NAV
