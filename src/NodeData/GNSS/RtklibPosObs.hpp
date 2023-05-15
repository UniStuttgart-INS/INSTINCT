// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtklibPosObs.hpp
/// @brief RTKLIB Pos Observation Class
/// @author
/// @date

#pragma once

#include "NodeData/State/PosVel.hpp"
#include "util/Eigen.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV
{
/// RTKLIB Observation Class
class RtklibPosObs : public PosVel
{
  public:
#ifdef TESTING
    /// Default constructor
    RtklibPosObs() = default;

    /// @brief Constructor
    /// @param[in] insTime Epoch time
    /// @param[in] e_position // Position in ECEF coordinates
    /// @param[in] lla_position // Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] e_velocity // Velocity in earth coordinates [m/s]
    /// @param[in] n_velocity // Velocity in navigation coordinates [m/s]
    /// @param[in] Q // 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    /// @param[in] ns // Number of satellites
    /// @param[in] sdXYZ // Standard Deviation XYZ [m]
    /// @param[in] sdNED // Standard Deviation North East Down [m]
    /// @param[in] sdxy // Standard Deviation xy [m]
    /// @param[in] sdyz // Standard Deviation yz [m]
    /// @param[in] sdzx // Standard Deviation zx [m]
    /// @param[in] sdne // Standard Deviation ne [m]
    /// @param[in] sded // Standard Deviation ed [m]
    /// @param[in] sddn // Standard Deviation dn [m]
    /// @param[in] age // Age [s]
    /// @param[in] ratio // Ratio
    /// @param[in] sdvNED // Standard Deviation velocity NED [m/s]
    /// @param[in] sdvne // Standard Deviation velocity north-east [m/s]
    /// @param[in] sdved // Standard Deviation velocity east-down [m/s]
    /// @param[in] sdvdn // Standard Deviation velocity down-north [m/s]
    /// @param[in] sdvXYZ // Standard Deviation velocity XYZ [m/s] TODO: check units
    /// @param[in] sdvxy // Standard Deviation velocity xy [m/s] TODO: check units
    /// @param[in] sdvyz // Standard Deviation velocity yz [m/s] TODO: check units
    /// @param[in] sdvzx // Standard Deviation velocity zx [m/s] TODO: check units
    RtklibPosObs(const InsTime& insTime,
                 const Eigen::Vector3d& e_position,
                 Eigen::Vector3d lla_position,
                 const Eigen::Vector3d& e_velocity,
                 const Eigen::Vector3d& n_velocity,
                 std::optional<uint8_t> Q,
                 std::optional<uint8_t> ns,
                 const Eigen::Vector3d& sdXYZ,
                 const Eigen::Vector3d& sdNED,
                 double sdxy,
                 double sdyz,
                 double sdzx,
                 double sdne,
                 double sded,
                 double sddn,
                 double age,
                 double ratio,
                 const Eigen::Vector3d& sdvNED,
                 double sdvne,
                 double sdved,
                 double sdvdn,
                 const Eigen::Vector3d& sdvXYZ,
                 double sdvxy,
                 double sdvyz,
                 double sdvzx)
        : Q(Q), ns(ns), sdXYZ(sdXYZ), sdNED(sdNED), sdxy(sdxy), sdyz(sdyz), sdzx(sdzx), sdne(sdne), sded(sded), sddn(sddn), age(age), ratio(ratio), sdvNED(sdvNED), sdvne(sdvne), sdved(sdved), sdvdn(sdvdn), sdvXYZ(sdvXYZ), sdvxy(sdvxy), sdvyz(sdvyz), sdvzx(sdvzx)
    {
        this->insTime = insTime;

        if (std::isnan(e_position(0)))
        {
            lla_position.head<2>() = deg2rad(lla_position.head<2>());
            this->setPosition_lla(lla_position);
        }
        else { this->setPosition_e(e_position); }

        if (std::isnan(e_velocity(0))) { this->setVelocity_n(n_velocity); }
        else { this->setVelocity_e(e_velocity); }
    }
#endif

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "RtklibPosObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    std::optional<uint8_t> Q;
    /// Number of satellites
    std::optional<uint8_t> ns;

    /// Standard Deviation XYZ [m]
    Eigen::Vector3d sdXYZ{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation North East Down [m]
    Eigen::Vector3d sdNED{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation xy [m]
    double sdxy = std::nan("");
    /// Standard Deviation yz [m]
    double sdyz = std::nan("");
    /// Standard Deviation zx [m]
    double sdzx = std::nan("");
    /// Standard Deviation ne [m]
    double sdne = std::nan("");
    /// Standard Deviation ed [m]
    double sded = std::nan("");
    /// Standard Deviation dn [m]
    double sddn = std::nan("");
    /// Age [s]
    double age = std::nan("");
    /// Ratio
    double ratio = std::nan("");
    /// Standard Deviation velocity NED [m/s]
    Eigen::Vector3d sdvNED{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation velocity north-east [m/s]
    double sdvne = std::nan("");
    /// Standard Deviation velocity east-down [m/s]
    double sdved = std::nan("");
    /// Standard Deviation velocity down-north [m/s]
    double sdvdn = std::nan("");

    /// Standard Deviation velocity XYZ [m/s] TODO: check units
    Eigen::Vector3d sdvXYZ{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation velocity xy [m/s] TODO: check units
    double sdvxy = std::nan("");
    /// Standard Deviation velocity yz [m/s] TODO: check units
    double sdvyz = std::nan("");
    /// Standard Deviation velocity zx [m/s] TODO: check units
    double sdvzx = std::nan("");
};

} // namespace NAV
