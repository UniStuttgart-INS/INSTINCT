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

#include <memory>
#include <optional>

#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"

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
    /// Position in ECEF frame [m]
    Eigen::Vector3d e_pos = Eigen::Vector3d::Zero();
    /// Position in LLA frame [rad, rad, m]
    Eigen::Vector3d lla_pos = Eigen::Vector3d::Zero();
    /// Velocity in ECEF frame [m/s]
    Eigen::Vector3d e_vel = Eigen::Vector3d::Zero();
    /// Estimated receiver clock parameters
    ReceiverClock recvClk;
    /// Inter frequency biases
    std::unordered_map<Frequency, UncertainValue<double>> interFrequencyBias = std::unordered_map<Frequency, UncertainValue<double>>{};
    /// Latest GNSS observation
    std::shared_ptr<const GnssObs> gnssObs = nullptr;
};

} // namespace NAV