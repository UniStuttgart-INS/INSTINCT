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
// Forward declaration
class Imu;

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
    [[nodiscard]] const Eigen::Quaterniond& quatAccel_bp() const
    {
        return quaternionAccel_bp;
    }
    /// Quaternion from body frame to accelerometer platform frame
    [[nodiscard]] Eigen::Quaterniond quatAccel_pb() const
    {
        return quaternionAccel_bp.conjugate();
    }

    /// Quaternion from gyroscope platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatGyro_bp() const
    {
        return quaternionGyro_bp;
    }
    /// Quaternion from body frame to gyroscope platform frame
    [[nodiscard]] Eigen::Quaterniond quatGyro_pb() const
    {
        return quaternionGyro_bp.conjugate();
    }

    /// Quaternion from magnetometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatMag_bp() const
    {
        return quaternionMag_bp;
    }
    /// Quaternion from body frame to magnetometer platform frame
    [[nodiscard]] Eigen::Quaterniond quatMag_pb() const
    {
        return quaternionMag_bp.conjugate();
    }

  private:
    /// Quaternion from accelerometer platform frame to body frame
    Eigen::Quaterniond quaternionAccel_bp;
    /// Quaternion from gyroscope platform frame to body frame
    Eigen::Quaterniond quaternionGyro_bp;
    /// Quaternion from magnetometer platform frame to body frame
    Eigen::Quaterniond quaternionMag_bp;

    friend class Imu;
};

} // namespace NAV
