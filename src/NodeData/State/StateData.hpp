/// @file StateData.hpp
/// @brief State Data
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "NodeData/InsObs.hpp"

#include "util/InsTransformations.hpp"

#include "util/LinearAlgebra.hpp"

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

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief State Vector
    /// Entries are:
    /// [0-3] Quaternions body to navigation frame (roll, pitch, yaw)
    /// [4-6] Latitude, Longitude, Altitude
    /// [7-9] Velocity in navigation coordinates
    Eigen::Matrix<double, 10, 1> X;

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the Quaternion body to navigation frame (NED)
    Eigen::Ref<Eigen::Vector4d> quat_nb_coeff() { return X.segment<4>(0); }

    /// Returns the Quaternion body to navigation frame (NED)
    [[nodiscard]] Eigen::Ref<Eigen::Vector4d const> quat_nb_coeff() const { return X.segment<4>(0); }

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    [[nodiscard]] Quaterniond<Navigation, Body> quaternion_nb() const
    {
        return Quaterniond<Navigation, Body>(Eigen::Quaterniond(quat_nb_coeff()));
    }

    /// @brief Returns the Quaternion from navigation to body frame (NED)
    /// @return The Quaternion for the rotation from navigation to body coordinates
    [[nodiscard]] Quaterniond<Body, Navigation> quaternion_bn() const
    {
        return quaternion_nb().conjugate();
    }

    /// @brief Returns the Quaternion from navigation to Earth-fixed frame
    /// @return The Quaternion for the rotation from navigation to earth coordinates
    [[nodiscard]] Quaterniond<Earth, Navigation> quaternion_en() const
    {
        return trafo::quat_en(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from Earth-fixed frame to navigation
    /// @return The Quaternion for the rotation from earth navigation coordinates
    [[nodiscard]] Quaterniond<Navigation, Earth> quaternion_ne() const
    {
        return quaternion_en().conjugate();
    }

    /// @brief Returns the Quaternion from body to Earth-fixed frame
    /// @return The Quaternion for the rotation from body to earth coordinates
    [[nodiscard]] Quaterniond<Earth, Body> quaternion_eb() const
    {
        return quaternion_en() * quaternion_nb();
    }

    /// @brief Returns the Quaternion from Earth-fixed to body frame
    /// @return The Quaternion for the rotation from earth to body coordinates
    [[nodiscard]] Quaterniond<Body, Earth> quaternion_be() const
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
        return trafo::quat2eulerZYX(quaternion_bn());
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Position                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Set the latitude 𝜙, longitude λ and altitude
    /// @param[in] latLonAlt [latitude 𝜙, longitude λ and altitude]^T in [rad, rad, m]
    void setLatLonAlt(const Vector3d<LLA>& latLonAlt)
    {
        X.segment<3>(4) = latLonAlt;
    }

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    Vector3d<LLA> latLonAlt() { return Vector3d<LLA>(Eigen::Vector3d(X.segment<3>(4))); }

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    [[nodiscard]] Vector3d<LLA> latLonAlt() const { return Vector3d<LLA>(Eigen::Vector3d(X.segment<3>(4))); }

    /// Returns the latitude 𝜙 in [rad]
    double& latitude() { return X(4); }

    /// Returns the latitude 𝜙 in [rad]
    [[nodiscard]] const double& latitude() const { return X(4); }

    /// Returns the longitude λ in [rad]
    double& longitude() { return X(5); }

    /// Returns the longitude λ in [rad]
    [[nodiscard]] const double& longitude() const { return X(5); }

    /// Returns the altitude (height above ground) in [m]
    double& altitude() { return X(6); }

    /// Returns the altitude (height above ground) in [m]
    [[nodiscard]] const double& altitude() const { return X(6); }

    /// Returns the ECEF coordinates in [m] using the WGS84 ellipsoid
    [[nodiscard]] Vector3d<Earth> positionECEF_WGS84() const { return trafo::lla2ecef_WGS84({ latitude(), longitude(), altitude() }); }

    /// Returns the ECEF coordinates in [m] using the GRS80 ellipsoid
    [[nodiscard]] Vector3d<Earth> positionECEF_GRS80() const { return trafo::lla2ecef_GRS80({ latitude(), longitude(), altitude() }); }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Velocity                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the velocity in [m/s], in navigation coordinates

    /// @brief Set the velocity
    /// @param[in] velocity_n Velocity in [m/s], in navigation coordinates
    void setVelocity_n(const Vector3d<Navigation>& velocity_n)
    {
        X.segment<3>(7) = velocity_n;
    }

    /// Returns the velocity in [m/s], in navigation coordinates
    Vector3d<Navigation> velocity_n() { return Vector3d<Navigation>(Eigen::Vector3d(X.segment<3>(7))); }

    /// Returns the velocity in [m/s], in navigation coordinates
    [[nodiscard]] Vector3d<Navigation> velocity_n() const { return Vector3d<Navigation>(Eigen::Vector3d(X.segment<3>(7))); }

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
