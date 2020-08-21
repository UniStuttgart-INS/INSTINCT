/// @file StateData.hpp
/// @brief State Data
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "NodeData/InsObs.hpp"

#include "Eigen/Dense"
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
    /// [0-2] asdaasd
    /// [3-5] asasas
    // Eigen::Matrix<double, 13, 1> X;

    // clang-format off
    //               Eigen::Ref<Eigen::Vector3d>       p()       { return X.segment<3>(0); }
    // [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> p() const { return X.segment<3>(0); }
    //               Eigen::Ref<Eigen::Vector3d      > v()       { return X.segment<3>(3); }
    // [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> v() const { return X.segment<3>(3); }
    //               Eigen::Ref<Eigen::Vector3d      > w()       { return X.segment<3>(6); }
    // [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> w() const { return X.segment<3>(6); }
    //               Eigen::Ref<Eigen::Vector4d      > q()       { return X.segment<4>(9); }
    // [[nodiscard]] Eigen::Ref<Eigen::Vector4d const> q() const { return X.segment<4>(9); }
    // clang-format on

    /// @brief Calcualtes the Direction Cosine Matrix from earth to platform coordinates
    /// @return The DCM from earth to platform coordinates
    [[nodiscard]] Eigen::Matrix3d DCM_e2p() const
    {
        return DCMBodyToNED;
    }

    Eigen::Matrix3d DCMBodyToNED;

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
