// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Receiver.hpp
/// @brief Receiver information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#pragma once

#include <functional>
#include <memory>
#include <optional>

#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "Navigation/Transformations/Antenna.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

#include "util/Container/UncertainValue.hpp"

namespace NAV
{

/// @brief Receiver information
template<typename ReceiverType>
struct Receiver
{
    /// @brief Constructor
    /// @param type Receiver enum type
    /// @param satelliteSystems Satellite systems to use
    explicit Receiver(ReceiverType type, const std::vector<SatelliteSystem>& satelliteSystems)
        : type(type), recvClk(satelliteSystems) {}

    /// Receiver Type
    ReceiverType type;
    /// Marker Position in ECEF frame [m]
    Eigen::Vector3d e_posMarker = Eigen::Vector3d::Zero();
    /// Marker Position in LLA frame [rad, rad, m]
    Eigen::Vector3d lla_posMarker = Eigen::Vector3d::Zero();
    /// Velocity in ECEF frame [m/s]
    Eigen::Vector3d e_vel = Eigen::Vector3d::Zero();
    /// Estimated receiver clock parameters
    ReceiverClock recvClk;
    /// Inter frequency biases [s]
    std::unordered_map<Frequency, UncertainValue<double>> interFrequencyBias = std::unordered_map<Frequency, UncertainValue<double>>{};
    /// Latest GNSS observation
    std::shared_ptr<const GnssObs> gnssObs = nullptr;

    /// @brief Antenna Reference Point position in ECEF frame [m] (Marker + antennaDeltaNEU)
    /// @param[in] hen_delta Additional height, east, north in [m]
    [[nodiscard]] Eigen::Vector3d e_calcPosARP(const Eigen::Vector3d& hen_delta = Eigen::Vector3d::Zero()) const
    {
        return trafo::e_posMarker2ARP(e_posMarker, gnssObs, hen_delta);
    }

    /// @brief Antenna Reference Point position in LLA frame [rad, rad, m] (Marker + antennaDeltaNEU)
    /// @param[in] hen_delta Additional height, east, north in [m]
    [[nodiscard]] Eigen::Vector3d lla_calcPosARP(const Eigen::Vector3d& hen_delta = Eigen::Vector3d::Zero()) const
    {
        return trafo::lla_posMarker2ARP(lla_posMarker, gnssObs, hen_delta);
    }

    /// @brief Marker position in ECEF frame [m] (ARP + antenna phase center)
    /// @param[in] freq Frequency of the observation
    /// @param[in] antennaType Antenna type
    /// @param[in] nameId NameId of the calling node for Log output
    /// @param[in] hen_delta Additional height, east, north in [m]
    [[nodiscard]] Eigen::Vector3d e_calcPosAPC(Frequency freq,
                                               const std::string& antennaType,
                                               const std::string& nameId,
                                               const Eigen::Vector3d& hen_delta = Eigen::Vector3d::Zero()) const
    {
        return trafo::e_posARP2APC(e_calcPosARP(hen_delta), gnssObs, freq, antennaType, nameId);
    }

    /// @brief Marker position in LLA frame [rad, rad, m] (ARP + antenna phase center)
    /// @param[in] freq Frequency of the observation
    /// @param[in] antennaType Antenna type
    /// @param[in] nameId NameId of the calling node for Log output
    /// @param[in] hen_delta Additional height, east, north in [m]
    [[nodiscard]] Eigen::Vector3d lla_calcPosAPC(Frequency freq,
                                                 const std::string& antennaType,
                                                 const std::string& nameId,
                                                 const Eigen::Vector3d& hen_delta = Eigen::Vector3d::Zero()) const
    {
        return trafo::lla_posARP2APC(lla_calcPosARP(hen_delta), gnssObs, freq, antennaType, nameId);
    }
};

} // namespace NAV