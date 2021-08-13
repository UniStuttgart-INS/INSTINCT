/// @file LCKFState.hpp
/// @brief State vector for the Loosely Coupled Kalman Filter GNSS/INS Integration
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-11

#pragma once

#include "util/Eigen.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class LCKFStateVector : public InsObs
{
  public:
    /// @brief Default constructor
    LCKFStateVector() = default;
    /// @brief Destructor
    ~LCKFStateVector() override = default;
    /// @brief Copy constructor
    LCKFStateVector(const LCKFStateVector&) = delete;
    /// @brief Move constructor
    LCKFStateVector(LCKFStateVector&&) = delete;
    /// @brief Copy assignment operator
    LCKFStateVector& operator=(const LCKFStateVector&) = delete;
    /// @brief Move assignment operator
    LCKFStateVector& operator=(LCKFStateVector&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("LCKFStateVector");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                            State vector x̂ = [δ𝛙_nb_n  δ𝐯_n  δ𝐩  𝐛_a  𝐛_g]^T                            */
    /* -------------------------------------------------------------------------------------------------------- */

    /// δx The state error
    Eigen::Ref<Eigen::Matrix<double, 15, 1>> x_stateVector() { return x; }
    /// δx The state error
    [[nodiscard]] Eigen::Ref<Eigen::Matrix<double, 15, 1> const> x_stateVector() const { return x; }

    /// δ𝛙_nb_n = [δ𝜑 δ𝜃 δ𝜓]_nb_n The attitude error (roll, pitch, yaw) in navigation coordinates in [rad]
    Eigen::Ref<Eigen::Vector3d> x_attitudeError_n() { return x.segment<3>(0); }
    /// δ𝛙_nb_n = [δ𝜑 δ𝜃 δ𝜓]_nb_n The attitude error (roll, pitch, yaw) in navigation coordinates in [rad]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> x_attitudeError_n() const { return x.segment<3>(0); }

    /// δ𝐯_n The velocity error in navigation coordinates in [m/s]
    Eigen::Ref<Eigen::Vector3d> x_velocityError_n() { return x.segment<3>(3); }
    /// δ𝐯_n The velocity error in navigation coordinates in [m/s]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> x_velocityError_n() const { return x.segment<3>(3); }

    /// δ𝐩 = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    Eigen::Ref<Eigen::Vector3d> x_positionError_lla() { return x.segment<3>(6); }
    /// δ𝐩 = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> x_positionError_lla() const { return x.segment<3>(6); }

    /// 𝐛_a The accelerometer bias in [m/s^2]
    Eigen::Ref<Eigen::Vector3d> x_biasAccel() { return x.segment<3>(9); }
    /// 𝐛_a The accelerometer bias in [m/s^2]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> x_biasAccel() const { return x.segment<3>(9); }

    /// 𝐛_g The gyroscope bias in [rad/s]
    Eigen::Ref<Eigen::Vector3d> x_biasGyro() { return x.segment<3>(12); }
    /// 𝐛_g The gyroscope bias in [rad/s]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> x_biasGyro() const { return x.segment<3>(12); }

  private:
    /// x̂ State vector
    Eigen::Matrix<double, 15, 1> x;
};

} // namespace NAV
