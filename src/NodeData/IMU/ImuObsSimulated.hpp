// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuObsSimulated.hpp
/// @brief Data storage class for simulated IMU observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-11-20

#pragma once

#include "ImuObs.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class ImuObsSimulated final : public ImuObs
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit ImuObsSimulated(const ImuPos& imuPos)
        : ImuObs(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "ImuObsSimulated";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { ImuObs::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = ImuObs::GetStaticDataDescriptors();
        desc.emplace_back("AccelDynamicsN [m/s^2]");
        desc.emplace_back("AccelDynamicsE [m/s^2]");
        desc.emplace_back("AccelDynamicsD [m/s^2]");
        desc.emplace_back("AngularRateN (ω_nb_n) [rad/s]");
        desc.emplace_back("AngularRateE (ω_nb_n) [rad/s]");
        desc.emplace_back("AngularRateD (ω_nb_n) [rad/s]");
        desc.emplace_back("AccelDynamicsX ECEF [m/s^2]");
        desc.emplace_back("AccelDynamicsY ECEF [m/s^2]");
        desc.emplace_back("AccelDynamicsZ ECEF [m/s^2]");
        desc.emplace_back("AngularRateX ECEF (ω_nb_e) [rad/s]");
        desc.emplace_back("AngularRateY ECEF (ω_nb_e) [rad/s]");
        desc.emplace_back("AngularRateZ ECEF (ω_nb_e) [rad/s]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 32; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0:  // Time since startup [ns]
        case 1:  // Mag uncomp X [Gauss]
        case 2:  // Mag uncomp Y [Gauss]
        case 3:  // Mag uncomp Z [Gauss]
        case 4:  // Accel uncomp X [m/s^2]
        case 5:  // Accel uncomp Y [m/s^2]
        case 6:  // Accel uncomp Z [m/s^2]
        case 7:  // Gyro uncomp X [rad/s]
        case 8:  // Gyro uncomp Y [rad/s]
        case 9:  // Gyro uncomp Z [rad/s]
        case 10: // Mag Comp X [Gauss]
        case 11: // Mag Comp Y [Gauss]
        case 12: // Mag Comp Z [Gauss]
        case 13: // Accel Comp X [m/s^2]
        case 14: // Accel Comp Y [m/s^2]
        case 15: // Accel Comp Z [m/s^2]
        case 16: // Gyro Comp X [rad/s]
        case 17: // Gyro Comp Y [rad/s]
        case 18: // Gyro Comp Z [rad/s]
        case 19: // Temperature [°C]
            return ImuObs::getValueAt(idx);
        case 20: // AccelDynamicsN [m/s^2]
            return n_accelDynamics.x();
        case 21: // AccelDynamicsE [m/s^2]
            return n_accelDynamics.y();
        case 22: // AccelDynamicsD [m/s^2]
            return n_accelDynamics.z();
        case 23: // AngularRateN (ω_nb_n) [rad/s]
            return n_angularRateDynamics.x();
        case 24: // AngularRateE (ω_nb_n) [rad/s]
            return n_angularRateDynamics.y();
        case 25: // AngularRateD (ω_nb_n) [rad/s]
            return n_angularRateDynamics.z();
        case 26: // AccelDynamicsX ECEF [m/s^2]
            return e_accelDynamics.x();
        case 27: // AccelDynamicsY ECEF [m/s^2]
            return e_accelDynamics.y();
        case 28: // AccelDynamicsZ ECEF [m/s^2]
            return e_accelDynamics.z();
        case 29: // AngularRateX ECEF (ω_nb_e) [rad/s]
            return e_angularRateDynamics.x();
        case 30: // AngularRateY ECEF (ω_nb_e) [rad/s]
            return e_angularRateDynamics.y();
        case 31: // AngularRateZ ECEF (ω_nb_e) [rad/s]
            return e_angularRateDynamics.z();
        default:
            return std::nullopt;
        }
    }

    /// The acceleration derived from the trajectory in [m/s^2], given in the NED frame.
    Eigen::Vector3d n_accelDynamics;
    /// The angular rate ω_nb_n derived from the trajectory in [rad/s], given in the NED frame.
    Eigen::Vector3d n_angularRateDynamics;

    /// The acceleration derived from the trajectory in [m/s^2], given in the ECEF frame.
    Eigen::Vector3d e_accelDynamics;
    /// The angular rate ω_nb_e derived from the trajectory in [rad/s], given in the ECEF frame.
    Eigen::Vector3d e_angularRateDynamics;
};

} // namespace NAV
