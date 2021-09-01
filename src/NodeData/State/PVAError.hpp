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
class PVAError : public InsObs
{
  public:
    /// @brief Default constructor
    PVAError() = default;
    /// @brief Destructor
    ~PVAError() override = default;
    /// @brief Copy constructor
    PVAError(const PVAError&) = delete;
    /// @brief Move constructor
    PVAError(PVAError&&) = delete;
    /// @brief Copy assignment operator
    PVAError& operator=(const PVAError&) = delete;
    /// @brief Move assignment operator
    PVAError& operator=(PVAError&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("PVAError");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                            State vector x̂ = [δ𝛙_nb_n  δ𝐯_n  δ𝐩]^T                            */
    /* -------------------------------------------------------------------------------------------------------- */

    /// δx The state error
    Eigen::Ref<Eigen::Matrix<double, 9, 1>> x_stateVector() { return x; }
    /// δx The state error
    [[nodiscard]] Eigen::Ref<Eigen::Matrix<double, 9, 1> const> x_stateVector() const { return x; }

    /// δ𝛙_nb_n = [δ𝜑 δ𝜃 δ𝜓]_nb_n The attitude error (roll, pitch, yaw) in navigation coordinates in [rad]
    Eigen::Ref<Eigen::Vector3d> attitudeError_n() { return x.segment<3>(0); }
    /// δ𝛙_nb_n = [δ𝜑 δ𝜃 δ𝜓]_nb_n The attitude error (roll, pitch, yaw) in navigation coordinates in [rad]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> attitudeError_n() const { return x.segment<3>(0); }

    /// δ𝐯_n The velocity error in navigation coordinates in [m/s]
    Eigen::Ref<Eigen::Vector3d> velocityError_n() { return x.segment<3>(3); }
    /// δ𝐯_n The velocity error in navigation coordinates in [m/s]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> velocityError_n() const { return x.segment<3>(3); }

    /// δ𝐩 = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    Eigen::Ref<Eigen::Vector3d> positionError_lla() { return x.segment<3>(6); }
    /// δ𝐩 = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    [[nodiscard]] Eigen::Ref<Eigen::Vector3d const> positionError_lla() const { return x.segment<3>(6); }

  private:
    /// x̂ State vector
    Eigen::Matrix<double, 9, 1> x;
};

} // namespace NAV
