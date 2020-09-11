/// @file StateData.hpp
/// @brief State Data
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "NodeData/InsObs.hpp"

#include "util/InsTransformations.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace NAV
{
/// IMU Observation storage class
class StateData : public InsObs
{
  public:
    /// @brief Default constructor
    StateData() = default;
    /// @brief Destructor
    ~StateData() override = default;
    /// @brief Copy constructor
    StateData(const StateData&) = delete;
    /// @brief Move constructor
    StateData(StateData&&) = delete;
    /// @brief Copy assignment operator
    StateData& operator=(const StateData&) = delete;
    /// @brief Move assignment operator
    StateData& operator=(StateData&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("StateData");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "InsObs" };
        return parents;
    }

    /// @brief State Vector
    /// Entries are:
    /// [0-3] Quaternions body to navigation frame (roll, pitch, yaw)
    /// [4-6] Latitude, Longitude, Height
    Eigen::Matrix<double, 7, 1> X;

    /// Returns the Quaternion body to navigation frame (NED)
    Eigen::Ref<Eigen::Vector4d> quat_b2n_coeff() { return X.segment<4>(0); }
    /// Returns the Quaternion body to navigation frame (NED)
    [[nodiscard]] Eigen::Ref<Eigen::Vector4d const> quat_b2n_coeff() const { return X.segment<4>(0); }

    /// Returns the latitude, longitude and height in [rad, rad, m]
    Eigen::Ref<Eigen::Vector3d> latLonHeight() { return X.segment<3>(4); }
    /// Returns the latitude, longitude and height above ground in [rad, rad, m]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> latLonHeight() const { return X.segment<3>(4); }
    /// Returns the latitude in [rad]
    double& latitude() { return X(4); }
    /// Returns the latitude in [rad]
    [[nodiscard]] const double& latitude() const { return X(4); }
    /// Returns the longitude in [rad]
    double& longitude() { return X(5); }
    /// Returns the longitude in [rad]
    [[nodiscard]] const double& longitude() const { return X(5); }
    /// Returns the height above ground in [m]
    double& height() { return X(6); }
    /// Returns the height above ground in [m]
    [[nodiscard]] const double& height() const { return X(6); }

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_b2n() const
    {
        return Eigen::Quaterniond(quat_b2n_coeff());
    }

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_n2b() const
    {
        return quaternion_b2n().conjugate();
    }

    /// @brief Returns the Quaternion from platform to Earth-fixed frame
    /// @return The Quaternion for the rotation from platform to earth coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_n2e() const
    {
        return trafo::quat_n2e(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from body to Earth-fixed frame
    /// @return The Quaternion for the rotation from body to earth coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_b2e() const
    {
        return quaternion_n2e() * quaternion_b2n();
    }

    /// @brief Returns the Quaternion from Earth-fixed to body frame
    /// @return The Quaternion for the rotation from earth to body coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_e2b() const
    {
        return quaternion_b2e().conjugate();
    }

    // Eigen::Vector3d VelocityNED;
    // Eigen::Vector3d BodyRate;
    // Eigen::Vector3d BodyAccel;
    // Eigen::Vector3d PositionLatLonAlt;
    // Eigen::Vector3d PositionNED;
    // /// Attitude of the vehicle (body frame (b) relative to local level coordinate system (n))
    // /// R = Roll angle
    // /// P = Pitch angle
    // /// Y = Yaw angle
    // /// C^n_b = C(3, -Y) \cdot C(2, -P) \cdot C(1, -R)
    // Eigen::Vector3d EulerAngles;
    // // Quaternions calculated by navsos
    // Eigen::Quaterniond quat;
    // // DCM calculated by navsos
    // Eigen::Matrix3d DCMBodyToNED;
    // double_t NavTime = 0.0;
    // double_t MagneticHeading = 0.0;

    // /* Kalman Filter State Vector */
    // Eigen::VectorXd X; //(Current) State vector [N*1]
    // Eigen::VectorXd Z; //(Current) Measurement vector [M*1]

    // Eigen::Vector3d GyroBias = { 0.0, 0.0, 0.0 };
    // Eigen::Vector3d AccelBias = { 0.0, 0.0, 0.0 };

    // Eigen::Vector3d VelocityNoise = { 0.0, 0.0, 0.0 };
    // Eigen::Vector3d AngleNoise = { 0.0, 0.0, 0.0 };

    // Eigen::Vector3d GyroNoise = { 0.0, 0.0, 0.0 };
    // Eigen::Vector3d AccelNoise = { 0.0, 0.0, 0.0 };
};

} // namespace NAV
