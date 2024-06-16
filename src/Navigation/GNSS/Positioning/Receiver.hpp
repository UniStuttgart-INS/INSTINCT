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
#include "Navigation/GNSS/Positioning/AntexReader.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Receiver information
template<typename ReceiverType>
struct Receiver
{
    /// @brief Constructor
    /// @param type Receiver enum type
    explicit Receiver(ReceiverType type) : type(type) {}

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
    /// Inter frequency biases
    std::unordered_map<Frequency, UncertainValue<double>> interFrequencyBias = std::unordered_map<Frequency, UncertainValue<double>>{};
    /// Latest GNSS observation
    std::shared_ptr<const GnssObs> gnssObs = nullptr;

    /// @brief Antenna Reference Point position in ECEF frame [m] (Marker + antennaDeltaNEU)
    [[nodiscard]] Eigen::Vector3d e_posARP() const
    {
        Eigen::Vector3d e_posARP = e_posMarker;
        if (gnssObs && gnssObs->receiverInfo)
        {
            Eigen::Vector3d n_antennaDelta = Eigen::Vector3d(0.0, 0.0, -gnssObs->receiverInfo->get().antennaDeltaNEU.z());
            // Eigen::Vector3d n_antennaDelta = gnssObs->receiverInfo->get().antennaDeltaNEU;
            // n_antennaDelta.z() *= -1.0;
            e_posARP += trafo::e_Quat_n(lla_posMarker(0), lla_posMarker(1)) * n_antennaDelta;
        }
        return e_posARP;
    }

    /// @brief Antenna Reference Point position in LLA frame [rad, rad, m] (Marker + antennaDeltaNEU)
    [[nodiscard]] Eigen::Vector3d lla_posARP() const
    {
        Eigen::Vector3d lla_posARP = lla_posMarker;
        if (gnssObs && gnssObs->receiverInfo)
        {
            lla_posARP.z() += gnssObs->receiverInfo->get().antennaDeltaNEU.z();
        }
        return lla_posARP;
    }

    /// @brief Marker position in ECEF frame [m] (ARP + antenna phase center)
    /// @param[in] freq Frequency of the observation
    /// @param[in] antennaType Antenna type
    /// @param[in] nameId NameId of the calling node for Log output
    [[nodiscard]] Eigen::Vector3d e_posAntennaPhaseCenter(Frequency freq, const std::string& antennaType, const std::string& nameId) const
    {
        Eigen::Vector3d e_posAPC = e_posARP();
        if (gnssObs)
        {
            if (auto neu_antennaPhaseCenterOffset = AntexReader::Get().getAntennaPhaseCenterOffsetToARP(antennaType,
                                                                                                        Frequency_(freq),
                                                                                                        gnssObs->insTime,
                                                                                                        type,
                                                                                                        nameId))
            {
                Eigen::Vector3d n_antennaPhaseCenterOffset = Eigen::Vector3d(0.0, 0.0, -neu_antennaPhaseCenterOffset->z());
                // Eigen::Vector3d n_antennaPhaseCenterOffset = *neu_antennaPhaseCenterOffset;
                // n_antennaPhaseCenterOffset.z() *= -1.0;
                e_posAPC += trafo::e_Quat_n(lla_posMarker(0), lla_posMarker(1)) * n_antennaPhaseCenterOffset;
            }
        }
        return e_posAPC;
    }

    /// @brief Marker position in LLA frame [rad, rad, m] (ARP + antenna phase center)
    /// @param[in] freq Frequency of the observation
    /// @param[in] antennaType Antenna type
    /// @param[in] nameId NameId of the calling node for Log output
    [[nodiscard]] Eigen::Vector3d lla_posAntennaPhaseCenter(Frequency freq, const std::string& antennaType, const std::string& nameId) const
    {
        Eigen::Vector3d lla_posAPC = lla_posARP();
        if (gnssObs)
        {
            if (auto neu_antennaPhaseCenterOffset = AntexReader::Get().getAntennaPhaseCenterOffsetToARP(antennaType,
                                                                                                        Frequency_(freq),
                                                                                                        gnssObs->insTime,
                                                                                                        type,
                                                                                                        nameId))
            {
                lla_posAPC.z() += neu_antennaPhaseCenterOffset->z();
            }
        }
        return lla_posAPC;
    }
};

} // namespace NAV