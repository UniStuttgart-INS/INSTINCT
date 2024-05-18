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

#include "ImuObsWDelta.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class ImuObsSimulated final : public ImuObsWDelta
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit ImuObsSimulated(const ImuPos& imuPos)
        : ImuObsWDelta(imuPos) {}

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
        auto parent = ImuObsWDelta::parentTypes();
        parent.push_back(ImuObsWDelta::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = ImuObsWDelta::GetStaticDataDescriptors();
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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 30; }

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
        case 1:  // Accel X [m/s^2]
        case 2:  // Accel Y [m/s^2]
        case 3:  // Accel Z [m/s^2]
        case 4:  // Gyro X [rad/s]
        case 5:  // Gyro Y [rad/s]
        case 6:  // Gyro Z [rad/s]
        case 7:  // Mag X [Gauss]
        case 8:  // Mag Y [Gauss]
        case 9:  // Mag Z [Gauss]
        case 10: // Temperature [°C]
        case 11: // dTime [s]
        case 12: // dTheta X [deg]
        case 13: // dTheta Y [deg]
        case 14: // dTheta Z [deg]
        case 15: // dVelocity X [m/s]
        case 16: // dVelocity Y [m/s]
        case 17: // dVelocity Z [m/s]
            return ImuObsWDelta::getValueAt(idx);
        case 18: // AccelDynamicsN [m/s^2]
            return n_accelDynamics.x();
        case 19: // AccelDynamicsE [m/s^2]
            return n_accelDynamics.y();
        case 20: // AccelDynamicsD [m/s^2]
            return n_accelDynamics.z();
        case 21: // AngularRateN (ω_nb_n) [rad/s]
            return n_angularRateDynamics.x();
        case 22: // AngularRateE (ω_nb_n) [rad/s]
            return n_angularRateDynamics.y();
        case 23: // AngularRateD (ω_nb_n) [rad/s]
            return n_angularRateDynamics.z();
        case 24: // AccelDynamicsX ECEF [m/s^2]
            return e_accelDynamics.x();
        case 25: // AccelDynamicsY ECEF [m/s^2]
            return e_accelDynamics.y();
        case 26: // AccelDynamicsZ ECEF [m/s^2]
            return e_accelDynamics.z();
        case 27: // AngularRateX ECEF (ω_nb_e) [rad/s]
            return e_angularRateDynamics.x();
        case 28: // AngularRateY ECEF (ω_nb_e) [rad/s]
            return e_angularRateDynamics.y();
        case 29: // AngularRateZ ECEF (ω_nb_e) [rad/s]
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
