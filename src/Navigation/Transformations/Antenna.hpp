// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Antenna.hpp
/// @brief GNSS Antenna related transformations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-01

#pragma once

#include <Eigen/Dense>
#include <memory>

#include "CoordinateFrames.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "Navigation/GNSS/Positioning/AntexReader.hpp"

namespace NAV::trafo
{

/// @brief Converts a marker position to the antenna reference point position
/// @param[in] e_posMarker Marker position in ECEF coordinates [m]
/// @param[in] gnssObs GNSS observation with antenna info
/// @param[in] hen_delta Additional height, east, north in [m]
/// @return Antenna Reference Point position in ECEF frame [m] (Marker + antennaDeltaNEU)
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar>
    e_posMarker2ARP(const Eigen::MatrixBase<Derived>& e_posMarker,
                    const std::shared_ptr<const GnssObs>& gnssObs,
                    const Eigen::Vector3d& hen_delta = Eigen::Vector3d::Zero())
{
    Eigen::Vector3<typename Derived::Scalar> e_posARP = e_posMarker;

    auto lla_posMarker = trafo::ecef2lla_WGS84(e_posMarker);
    auto e_Quat_n = trafo::e_Quat_n(lla_posMarker(0), lla_posMarker(1));

    if (gnssObs && gnssObs->receiverInfo)
    {
        Eigen::Vector3<typename Derived::Scalar> n_antennaDelta(typename Derived::Scalar(0.0),
                                                                typename Derived::Scalar(0.0),
                                                                typename Derived::Scalar(-gnssObs->receiverInfo->get().antennaDeltaNEU.z()));
        e_posARP += e_Quat_n * n_antennaDelta;
    }
    return e_posARP
           + e_Quat_n
                 * Eigen::Vector3<typename Derived::Scalar>(typename Derived::Scalar(0.0),
                                                            typename Derived::Scalar(0.0),
                                                            typename Derived::Scalar(hen_delta(0)));
}

/// @brief Converts a marker position to the antenna reference point position
/// @param[in] lla_posMarker Marker position in LLA coordinates [rad, rad, m]
/// @param[in] gnssObs GNSS observation with antenna info
/// @param[in] hen_delta Additional height, east, north in [m]
/// @return Antenna Reference Point position in LLA coordinates [rad, rad, m] (Marker + antennaDeltaNEU)
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar>
    lla_posMarker2ARP(const Eigen::MatrixBase<Derived>& lla_posMarker,
                      const std::shared_ptr<const GnssObs>& gnssObs,
                      const Eigen::Vector3d& hen_delta = Eigen::Vector3d::Zero())
{
    Eigen::Vector3<typename Derived::Scalar> lla_posARP = lla_posMarker;
    auto e_Quat_n = trafo::e_Quat_n(lla_posMarker(0), lla_posMarker(1));
    if (gnssObs && gnssObs->receiverInfo)
    {
        lla_posARP.z() += gnssObs->receiverInfo->get().antennaDeltaNEU.z();
    }
    return lla_posARP
           + e_Quat_n
                 * Eigen::Vector3<typename Derived::Scalar>(typename Derived::Scalar(0.0),
                                                            typename Derived::Scalar(0.0),
                                                            typename Derived::Scalar(hen_delta(0)));
}

/// @brief Converts a antenna reference point position position to the antenna phase center position
/// @param[in] e_posARP Antenna reference point position in ECEF coordinates [m]
/// @param[in] gnssObs GNSS observation with antenna info
/// @param[in] freq Frequency of the observation
/// @param[in] antennaType Antenna type
/// @param[in] nameId NameId of the calling node for Log output
/// @return Antenna phase center position in ECEF frame [m] (ARP + antenna phase center)
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar>
    e_posARP2APC(const Eigen::MatrixBase<Derived>& e_posARP,
                 const std::shared_ptr<const GnssObs>& gnssObs,
                 Frequency freq,
                 const std::string& antennaType,
                 const std::string& nameId)
{
    Eigen::Vector3<typename Derived::Scalar> e_posAPC = e_posARP;
    if (gnssObs)
    {
        if (auto neu_antennaPhaseCenterOffset = AntexReader::Get().getAntennaPhaseCenterOffsetToARP(antennaType,
                                                                                                    Frequency_(freq),
                                                                                                    gnssObs->insTime,
                                                                                                    nameId))
        {
            Eigen::Vector3<typename Derived::Scalar> n_antennaPhaseCenterOffset(typename Derived::Scalar(0.0),
                                                                                typename Derived::Scalar(0.0),
                                                                                typename Derived::Scalar(-neu_antennaPhaseCenterOffset->z()));
            auto lla_posARP = trafo::ecef2lla_WGS84(e_posARP);
            if (!std::isnan(lla_posARP(0)) && !std::isnan(lla_posARP(1)))
            {
                e_posAPC += trafo::e_Quat_n(lla_posARP(0), lla_posARP(1)) * n_antennaPhaseCenterOffset;
            }
        }
    }
    return e_posAPC;
}

/// @brief Converts a antenna reference point position to the antenna phase center position
/// @param[in] lla_posARP Antenna reference point position in LLA coordinates [rad, rad, m]
/// @param[in] gnssObs GNSS observation with antenna info
/// @param[in] freq Frequency of the observation
/// @param[in] antennaType Antenna type
/// @param[in] nameId NameId of the calling node for Log output
/// @return Antenna phase center position in LLA coordinates [rad, rad, m] (ARP + antenna phase center)
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar>
    lla_posARP2APC(const Eigen::MatrixBase<Derived>& lla_posARP,
                   const std::shared_ptr<const GnssObs>& gnssObs,
                   Frequency freq,
                   const std::string& antennaType,
                   const std::string& nameId)
{
    Eigen::Vector3<typename Derived::Scalar> lla_posAPC = lla_posARP;
    if (gnssObs)
    {
        if (auto neu_antennaPhaseCenterOffset =
                AntexReader::Get().getAntennaPhaseCenterOffsetToARP(antennaType,
                                                                    Frequency_(freq),
                                                                    gnssObs->insTime,
                                                                    nameId))
        {
            lla_posAPC.z() += neu_antennaPhaseCenterOffset->z();
        }
    }
    return lla_posAPC;
}

} // namespace NAV::trafo