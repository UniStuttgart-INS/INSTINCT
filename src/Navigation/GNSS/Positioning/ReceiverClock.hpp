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
    /// @param[in] satelliteSystems Satellite systems to add
    explicit ReceiverClock(std::vector<SatelliteSystem> satelliteSystems)
        : satelliteSystems(std::move(satelliteSystems))
    {
        bias.resize(this->satelliteSystems.size());
        biasStdDev.resize(this->satelliteSystems.size());
    }

    /// @brief Add a new system
    /// @param[in] satSys Satellite System to add
    void addSystem(SatelliteSystem satSys)
    {
        if (std::ranges::find(satelliteSystems, satSys) != satelliteSystems.end())
        {
            return;
        }
        satelliteSystems.push_back(satSys);
        bias.emplace_back();
        biasStdDev.emplace_back();
    }

    /// @brief Clear all the structures
    void clear()
    {
        satelliteSystems.clear();
        bias.clear();
        biasStdDev.clear();
        drift = 0.;
        driftStdDev = 0.;
    }

    /// @brief Resets all structures to 0 (not removing them)
    void reset()
    {
        std::ranges::for_each(bias, [](double& v) { v = 0; });
        std::ranges::for_each(biasStdDev, [](double& v) { v = 0; });
        drift = 0.;
        driftStdDev = 0.;
    }

    /// @brief Get the bias for the given satellite system
    /// @param[in] satSys Satellite system
    /// @return The bias in [s] for the given satellite system. Or null if it is not found
    [[nodiscard]] const double* biasFor(SatelliteSystem satSys) const
    {
        if (auto i = getIdx(satSys)) { return &bias.at(*i); }
        return nullptr;
    }
    /// @brief Get the bias for the given satellite system
    /// @param[in] satSys Satellite system
    /// @return The bias in [s] for the given satellite system. Or null if it is not found
    [[nodiscard]] double* biasFor(SatelliteSystem satSys)
    {
        if (auto i = getIdx(satSys)) { return &bias.at(*i); }
        return nullptr;
    }

    /// @brief Get the bias StdDev for the given satellite system
    /// @param[in] satSys Satellite system
    /// @return The bias StdDev in [s] for the given satellite system. Or null if it is not found
    [[nodiscard]] const double* biasStdDevFor(SatelliteSystem satSys) const
    {
        if (auto i = getIdx(satSys)) { return &biasStdDev.at(*i); }
        return nullptr;
    }
    /// @brief Get the bias StdDev for the given satellite system
    /// @param[in] satSys Satellite system
    /// @return The bias StdDev in [s] for the given satellite system. Or null if it is not found
    [[nodiscard]] double* biasStdDevFor(SatelliteSystem satSys)
    {
        if (auto i = getIdx(satSys)) { return &biasStdDev.at(*i); }
        return nullptr;
    }

    /// @brief Get the index of the sat system
    /// @param[in] satSys Satellite system
    /// @return The index if it was in the list
    [[nodiscard]] std::optional<size_t> getIdx(SatelliteSystem satSys) const
    {
        for (size_t i = 0; i < satelliteSystems.size(); i++)
        {
            if (satelliteSystems.at(i) == satSys) { return i; }
        }
        return std::nullopt;
    }

    /// Order of satellite systems
    std::vector<SatelliteSystem> satelliteSystems;

    /// Receiver clock biases for each satellite system [s]
    std::vector<double> bias;
    /// StdDev of the receiver clock biases for each satellite system [s]
    std::vector<double> biasStdDev;
    /// Receiver clock drift [s/s]
    /// @note The satellite reference times do not drift with respect to each other (or so small that receiver clock drift is way bigger)
    double drift = 0.;
    /// StdDev of the receiver clock drift[s]
    /// @note The satellite reference times do not drift with respect to each other (or so small that receiver clock drift is way bigger)
    double driftStdDev = 0.;
};

} // namespace NAV