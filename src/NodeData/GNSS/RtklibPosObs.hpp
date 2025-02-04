// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtklibPosObs.hpp
/// @brief RTKLIB Pos Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-02

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
    /// @param[in] e_position Position in ECEF coordinates
    /// @param[in] lla_position Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] e_velocity Velocity in earth coordinates [m/s]
    /// @param[in] n_velocity Velocity in navigation coordinates [m/s]
    /// @param[in] Q 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    /// @param[in] ns Number of satellites
    /// @param[in] sdXYZ Standard Deviation XYZ [m]
    /// @param[in] sdNED Standard Deviation North East Down [m]
    /// @param[in] sdxy Standard Deviation xy [m]
    /// @param[in] sdyz Standard Deviation yz [m]
    /// @param[in] sdzx Standard Deviation zx [m]
    /// @param[in] sdne Standard Deviation ne [m]
    /// @param[in] sded Standard Deviation ed [m]
    /// @param[in] sddn Standard Deviation dn [m]
    /// @param[in] age Age [s]
    /// @param[in] ratio Ratio
    /// @param[in] sdvNED Standard Deviation velocity NED [m/s]
    /// @param[in] sdvne Standard Deviation velocity north-east [m/s]
    /// @param[in] sdved Standard Deviation velocity east-down [m/s]
    /// @param[in] sdvdn Standard Deviation velocity down-north [m/s]
    /// @param[in] sdvXYZ Standard Deviation velocity XYZ [m/s]
    /// @param[in] sdvxy Standard Deviation velocity xy [m/s]
    /// @param[in] sdvyz Standard Deviation velocity yz [m/s]
    /// @param[in] sdvzx Standard Deviation velocity zx [m/s]
    RtklibPosObs(const InsTime& insTime,
                 std::optional<Eigen::Vector3d> e_position,
                 std::optional<Eigen::Vector3d> lla_position,
                 std::optional<Eigen::Vector3d> e_velocity,
                 std::optional<Eigen::Vector3d> n_velocity,
                 uint8_t Q,
                 uint8_t ns,
                 std::optional<Eigen::Vector3d> sdXYZ,
                 std::optional<Eigen::Vector3d> sdNED,
                 std::optional<double> sdxy,
                 std::optional<double> sdyz,
                 std::optional<double> sdzx,
                 std::optional<double> sdne,
                 std::optional<double> sded,
                 std::optional<double> sddn,
                 double age,
                 double ratio,
                 std::optional<Eigen::Vector3d> sdvNED,
                 std::optional<double> sdvne,
                 std::optional<double> sdved,
                 std::optional<double> sdvdn,
                 std::optional<Eigen::Vector3d> sdvXYZ,
                 std::optional<double> sdvxy,
                 std::optional<double> sdvyz,
                 std::optional<double> sdvzx)
        : Q(Q), ns(ns), age(age), ratio(ratio)
    {
        this->insTime = insTime;

        if (lla_position) { lla_position->head<2>() = deg2rad(lla_position->head<2>()); }

        if (e_position && sdXYZ && sdxy && sdzx && sdyz)
        {
            Eigen::Matrix3d cov;
            cov << std::pow(sdXYZ->x(), 2), *sdxy, -(*sdzx),
                -(*sdxy), std::pow(sdXYZ->y(), 2), *sdyz,
                *sdzx, -(*sdyz), std::pow(sdXYZ->z(), 2);
            this->setPositionAndStdDev_e(*e_position, cov);
            this->setPosCovarianceMatrix_e(cov);
        }
        else if (lla_position && sdNED && sdne && sddn && sded)
        {
            Eigen::Matrix3d cov;
            cov << std::pow(sdNED->x(), 2), *sdne, -(*sddn),
                -(*sdne), std::pow(sdNED->y(), 2), *sded,
                *sddn, -(*sded), std::pow(sdNED->z(), 2);
            this->setPositionAndStdDev_lla(*lla_position, cov);
            this->setPosCovarianceMatrix_n(cov);
        }
        else if (e_position) { this->setPosition_e(*e_position); }
        else if (lla_position) { this->setPosition_lla(*lla_position); }

        if (e_velocity && sdvXYZ && sdvxy && sdvzx && sdvyz)
        {
            Eigen::Matrix3d cov;
            cov << std::pow(sdvXYZ->x(), 2), *sdvxy, -(*sdvzx),
                -(*sdvxy), std::pow(sdvXYZ->y(), 2), *sdvyz,
                *sdvzx, -(*sdvyz), std::pow(sdvXYZ->z(), 2);
            this->setVelocityAndStdDev_e(*e_velocity, cov);
        }
        else if (n_velocity && sdvNED && sdvne && sdvdn && sdved)
        {
            Eigen::Matrix3d cov;
            cov << std::pow(sdvNED->x(), 2), *sdvne, -(*sdvdn),
                -(*sdvne), std::pow(sdvNED->y(), 2), *sdved,
                *sdvdn, -(*sdved), std::pow(sdvNED->z(), 2);
            this->setVelocityAndStdDev_n(*n_velocity, cov);
        }
        else if (e_velocity) { this->setVelocity_e(*e_velocity); }
        else if (n_velocity) { this->setVelocity_n(*n_velocity); }
    }
#endif

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "RtklibPosObs";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = PosVel::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Q [-]");
        desc.emplace_back("ns [-]");
        desc.emplace_back("age [s]");
        desc.emplace_back("ratio [-]");

        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return PosVel::GetStaticDescriptorCount() + 4; }

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
        if (idx < PosVel::GetStaticDescriptorCount()) { return PosVel::getValueAt(idx); }
        switch (idx)
        {
        case PosVel::GetStaticDescriptorCount() + 0: // Q [-]
            return Q;
        case PosVel::GetStaticDescriptorCount() + 1: // ns [-]
            return ns;
        case PosVel::GetStaticDescriptorCount() + 2: // age [s]
            return age;
        case PosVel::GetStaticDescriptorCount() + 3: // ratio [-]
            return ratio;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    uint8_t Q = 0;
    /// Number of satellites
    uint8_t ns = 0;
    /// Age [s]
    double age = std::nan("");
    /// Ratio
    double ratio = std::nan("");
};

} // namespace NAV
