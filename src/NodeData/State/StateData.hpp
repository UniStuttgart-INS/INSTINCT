/// @file StateData.hpp
/// @brief State Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "NodeData/InsObs.hpp"

#include "util/InsTransformations.hpp"

#include "util/Eigen.hpp"

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
    StateData(const StateData&) = default;
    /// @brief Move constructor
    StateData(StateData&&) = default;
    /// @brief Copy assignment operator
    StateData& operator=(const StateData&) = default;
    /// @brief Move assignment operator
    StateData& operator=(StateData&&) = default;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("StateData");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    Eigen::Quaterniond& quaternion_nb() { return q_nb; }

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    [[nodiscard]] const Eigen::Quaterniond& quaternion_nb() const { return q_nb; }

    /// @brief Returns the Quaternion from navigation to body frame (NED)
    /// @return The Quaternion for the rotation from navigation to body coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_bn() const
    {
        return quaternion_nb().conjugate();
    }

    /// @brief Returns the Quaternion from navigation to Earth-fixed frame
    /// @return The Quaternion for the rotation from navigation to earth coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_en() const
    {
        return trafo::quat_en(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from Earth-fixed frame to navigation
    /// @return The Quaternion for the rotation from earth navigation coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_ne() const
    {
        return quaternion_en().conjugate();
    }

    /// @brief Returns the Quaternion from body to Earth-fixed frame
    /// @return The Quaternion for the rotation from body to earth coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_eb() const
    {
        return quaternion_en() * quaternion_nb();
    }

    /// @brief Returns the Quaternion from Earth-fixed to body frame
    /// @return The Quaternion for the rotation from earth to body coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_be() const
    {
        return quaternion_eb().conjugate();
    }

    /// @brief Returns the Roll, Pitch and Yaw angles in [rad]
    /// @return [roll, pitch, yaw]^T
    [[nodiscard]] Eigen::Vector3d rollPitchYaw() const
    {
        // Eigen::Matrix3d DCMBodyToNED = quaternion_nb().toRotationMatrix();
        // Eigen::Vector3d EulerAngles = Eigen::Vector3d::Zero();

        // EulerAngles(1) = -asin(DCMBodyToNED(2, 0));
        // if (fabs(trafo::rad2deg(EulerAngles(1))) < 89.0)
        // {
        //     EulerAngles(0) = atan2((DCMBodyToNED(2, 1) / cos(EulerAngles(1))), (DCMBodyToNED(2, 2) / cos(EulerAngles(1))));

        //     EulerAngles(2) = atan2((DCMBodyToNED(1, 0) / cos(EulerAngles(1))), (DCMBodyToNED(0, 0) / cos(EulerAngles(1))));
        // }
        // else
        // {
        //     EulerAngles(0) = 0.0;
        //     EulerAngles(2) = 0.0;
        // }

        // return EulerAngles;
        return trafo::quat2eulerZYX(quaternion_nb());
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Position                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    [[nodiscard]] Eigen::Vector3d latLonAlt() const { return trafo::ecef2lla_WGS84(position_ecef()); }

    /// Returns the latitude 𝜙 in [rad]
    [[nodiscard]] double latitude() const { return latLonAlt()(0); }

    /// Returns the longitude λ in [rad]
    [[nodiscard]] double longitude() const { return latLonAlt()(1); }

    /// Returns the altitude (height above ground) in [m]
    [[nodiscard]] double altitude() const { return latLonAlt()(2); }

    /// Returns the ECEF coordinates in [m]
    Eigen::Vector3d& position_ecef() { return p_ecef; }

    /// Returns the ECEF coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& position_ecef() const { return p_ecef; }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Velocity                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the velocity in [m/s], in navigation coordinates
    Eigen::Vector3d& velocity_n() { return v_n; }

    /// Returns the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& velocity_n() const { return v_n; }

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

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Quaternion body to navigation frame (roll, pitch, yaw)
    Eigen::Quaterniond q_nb;
    /// Position in ECEF coordinates
    Eigen::Vector3d p_ecef;
    /// Velocity in navigation coordinates
    Eigen::Vector3d v_n;
};

} // namespace NAV
