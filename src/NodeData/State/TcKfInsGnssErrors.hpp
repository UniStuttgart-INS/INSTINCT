// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TcKfInsGnssErrors.hpp
/// @brief Tightly-coupled Kalman Filter INS/GNSS errors
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-02-24

#pragma once

#include "LcKfInsGnssErrors.hpp"

namespace NAV
{
/// Tightly-coupled Kalman Filter INS/GNSS errors
class TcKfInsGnssErrors : public LcKfInsGnssErrors
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "TcKfInsGnssErrors";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = LcKfInsGnssErrors::parentTypes();
        parent.push_back(LcKfInsGnssErrors::type());
        return parent;
    }

    /// δϱ The receiver clock offset in [m]
    double recvClkOffset{};

    /// δϱ_dot The receiver clock drift in [m/s]
    double recvClkDrift{};
};

} // namespace NAV