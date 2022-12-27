// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Clock.hpp
/// @brief Abstract satellite clock information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-02

#pragma once

#include "Navigation/Time/InsTime.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{

/// @brief Abstract satellite clock information
class Clock
{
  public:
    /// Satellite clock corrections
    struct Corrections
    {
        InsTime transmitTime{}; ///< Transmit time of the signal
        double bias{};          ///< Satellite clock bias [s]
        double drift{};         ///< Satellite clock drift [s/s]
    };

    /// @brief Default Constructor
    Clock() = default;
    /// @brief Destructor
    virtual ~Clock() = default;
    /// @brief Copy constructor
    Clock(const Clock&) = default;
    /// @brief Move constructor
    Clock(Clock&&) = default;
    /// @brief Copy assignment operator
    Clock& operator=(const Clock&) = delete;
    /// @brief Move assignment operator
    Clock& operator=(Clock&&) = delete;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] virtual Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const = 0;
};

} // namespace NAV
