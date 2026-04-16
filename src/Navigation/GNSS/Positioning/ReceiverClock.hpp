// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ReceiverClock.hpp
/// @brief Receiver Clock information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-02-07

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "util/Container/UncertainValue.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "util/Logger.hpp"

namespace NAV
{

/// Receiver Clock information
struct ReceiverClock
{
    /// @brief Constructor
    ReceiverClock() = default;

    /// @brief Add a new system
    /// @param[in] satSys Satellite System to add
    void addSystem(SatelliteSystem satSys)
    {
        if (bias.first == SatSys_None)
        {
            bias.first = satSys;
        }
        else if (bias.first == satSys)
        {
            return;
        }
        else if (!interSystemBiases.contains(satSys))
        {
            interSystemBiases.emplace(satSys, UncertainValue<double>{});
        }
    }

    /// Receiver clock bias [s]
    std::pair<SatelliteSystem, UncertainValue<double>> bias = { SatSys_None, UncertainValue<double>{} };

    /// Receiver clock drift [s/s]
    /// @note The satellite reference times do not drift with respect to each other (or so small that receiver clock drift is way bigger)
    UncertainValue<double> drift;

    /// Inter system biases [s]
    std::unordered_map<SatelliteSystem, UncertainValue<double>> interSystemBiases;
};

} // namespace NAV