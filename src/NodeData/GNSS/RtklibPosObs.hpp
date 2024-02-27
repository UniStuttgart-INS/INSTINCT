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
        : Q(Q), ns(ns), sdxy(sdxy), sdyz(sdyz), sdzx(sdzx), sdne(sdne), sded(sded), sddn(sddn), age(age), ratio(ratio), sdvNED(std::move(sdvNED)), sdvXYZ(std::move(sdvXYZ)), sdvne(sdvne), sdved(sdved), sdvdn(sdvdn), sdvxy(sdvxy), sdvyz(sdvyz), sdvzx(sdvzx)
    {
        this->insTime = insTime;

        if (e_position.has_value()) { this->setPosition_e(e_position.value()); }
        else if (lla_position.has_value())
        {
            lla_position->head<2>() = deg2rad(lla_position->head<2>());
            this->setPosition_lla(lla_position.value());
        }

        if (e_velocity.has_value()) { this->setVelocity_e(e_velocity.value()); }
        else if (n_velocity.has_value()) { this->setVelocity_n(n_velocity.value()); }

        if (sdXYZ.has_value())
        {
            this->sdXYZ = sdXYZ.value();
            this->sdNED = trafo::n_Quat_e(latitude(), longitude()) * sdXYZ.value();
        }
        else if (sdNED.has_value())
        {
            this->sdNED = sdNED.value();
            this->sdXYZ = trafo::e_Quat_n(latitude(), longitude()) * sdNED.value();
        }
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
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetDataDescriptors()
    {
        auto desc = PosVel::GetDataDescriptors();
        desc.reserve(GetDescriptorCount());
        desc.emplace_back("Q [-]");
        desc.emplace_back("ns [-]");
        desc.emplace_back("sdx [m]");
        desc.emplace_back("sdy [m]");
        desc.emplace_back("sdz [m]");
        desc.emplace_back("sdn [m]");
        desc.emplace_back("sde [m]");
        desc.emplace_back("sdd [m]");
        desc.emplace_back("sdxy [m]");
        desc.emplace_back("sdyz [m]");
        desc.emplace_back("sdzx [m]");
        desc.emplace_back("sdne [m]");
        desc.emplace_back("sded [m]");
        desc.emplace_back("sddn [m]");
        desc.emplace_back("age [s]");
        desc.emplace_back("ratio [-]");
        desc.emplace_back("sdvn [m/s]");
        desc.emplace_back("sdve [m/s]");
        desc.emplace_back("sdvd [m/s]");
        desc.emplace_back("sdvne [m/s]");
        desc.emplace_back("sdved [m/s]");
        desc.emplace_back("sdvdn [m/s]");
        desc.emplace_back("sdvx [m/s]");
        desc.emplace_back("sdvy [m/s]");
        desc.emplace_back("sdvz [m/s]");
        desc.emplace_back("sdvxy [m/s]");
        desc.emplace_back("sdvyz [m/s]");
        desc.emplace_back("sdvzx [m/s]");

        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetDescriptorCount() { return 43; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> dataDescriptors() const override { return GetDataDescriptors(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetDescriptorCount());
        switch (idx)
        {
        case 0:  // Latitude [deg]
        case 1:  // Longitude [deg]
        case 2:  // Altitude [m]
        case 3:  // North/South [m]
        case 4:  // East/West [m]
        case 5:  // X-ECEF [m]
        case 6:  // Y-ECEF [m]
        case 7:  // Z-ECEF [m]
        case 8:  // Velocity norm [m/s]
        case 9:  // X velocity ECEF [m/s]
        case 10: // Y velocity ECEF [m/s]
        case 11: // Z velocity ECEF [m/s]
        case 12: // North velocity [m/s]
        case 13: // East velocity [m/s]
        case 14: // Down velocity [m/s]
            return PosVel::getValueAt(idx);
        case 15: // Q [-]
            return Q;
        case 16: // ns [-]
            return ns;
        case 17: // sdx [m]
            return sdXYZ.x();
        case 18: // sdy [m]
            return sdXYZ.y();
        case 19: // sdz [m]
            return sdXYZ.z();
        case 20: // sdn [m]
            return sdNED.x();
        case 21: // sde [m]
            return sdNED.y();
        case 22: // sdd [m]
            return sdNED.z();
        case 23: // sdxy [m]
            if (sdxy.has_value()) { return sdxy.value(); }
            break;
        case 24: // sdyz [m]
            if (sdyz.has_value()) { return sdyz.value(); }
            break;
        case 25: // sdzx [m]
            if (sdzx.has_value()) { return sdzx.value(); }
            break;
        case 26: // sdne [m]
            if (sdne.has_value()) { return sdne.value(); }
            break;
        case 27: // sded [m]
            if (sded.has_value()) { return sded.value(); }
            break;
        case 28: // sddn [m]
            if (sddn.has_value()) { return sddn.value(); }
            break;
        case 29: // age [s]
            return age;
        case 30: // ratio [-]
            return ratio;
        case 31: // sdvn [m/s]
            if (sdvNED.has_value()) { return sdvNED->x(); }
            break;
        case 32: // sdve [m/s]
            if (sdvNED.has_value()) { return sdvNED->y(); }
            break;
        case 33: // sdvd [m/s]
            if (sdvNED.has_value()) { return sdvNED->z(); }
            break;
        case 34: // sdvne [m/s]
            if (sdvne.has_value()) { return sdvne.value(); }
            break;
        case 35: // sdved [m/s]
            if (sdved.has_value()) { return sdved.value(); }
            break;
        case 36: // sdvdn [m/s]
            if (sdvdn.has_value()) { return sdvdn.value(); }
            break;
        case 37: // sdvx [m/s]
            if (sdvXYZ.has_value()) { return sdvXYZ->x(); }
            break;
        case 38: // sdvy [m/s]
            if (sdvXYZ.has_value()) { return sdvXYZ->y(); }
            break;
        case 39: // sdvz [m/s]
            if (sdvXYZ.has_value()) { return sdvXYZ->z(); }
            break;
        case 40: // sdvxy [m/s]
            if (sdvxy.has_value()) { return sdvxy.value(); }
            break;
        case 41: // sdvyz [m/s]
            if (sdvyz.has_value()) { return sdvyz.value(); }
            break;
        case 42: // sdvzx [m/s]
            if (sdvzx.has_value()) { return sdvzx.value(); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// 1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
    uint8_t Q = 0;
    /// Number of satellites
    uint8_t ns = 0;

    /// Standard Deviation XYZ [m]
    Eigen::Vector3d sdXYZ{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation North East Down [m]
    Eigen::Vector3d sdNED{ std::nan(""), std::nan(""), std::nan("") };
    /// Standard Deviation xy [m]
    std::optional<double> sdxy;
    /// Standard Deviation yz [m]
    std::optional<double> sdyz;
    /// Standard Deviation zx [m]
    std::optional<double> sdzx;
    /// Standard Deviation ne [m]
    std::optional<double> sdne;
    /// Standard Deviation ed [m]
    std::optional<double> sded;
    /// Standard Deviation dn [m]
    std::optional<double> sddn;

    /// Age [s]
    double age = std::nan("");
    /// Ratio
    double ratio = std::nan("");

    /// Standard Deviation velocity NED [m/s]
    std::optional<Eigen::Vector3d> sdvNED;
    /// Standard Deviation velocity XYZ [m/s]
    std::optional<Eigen::Vector3d> sdvXYZ;
    /// Standard Deviation velocity north-east [m/s]
    std::optional<double> sdvne;
    /// Standard Deviation velocity east-down [m/s]
    std::optional<double> sdved;
    /// Standard Deviation velocity down-north [m/s]
    std::optional<double> sdvdn;
    /// Standard Deviation velocity xy [m/s]
    std::optional<double> sdvxy;
    /// Standard Deviation velocity yz [m/s]
    std::optional<double> sdvyz;
    /// Standard Deviation velocity zx [m/s]
    std::optional<double> sdvzx;
};

} // namespace NAV
