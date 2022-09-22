// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InertialNavSol.hpp
/// @brief PosVelAtt Observation with ImuObs included
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-31

#pragma once

#include "PosVelAtt.hpp"
#include "NodeData/IMU/ImuObs.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class InertialNavSol : public PosVelAtt
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "InertialNavSol";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { PosVelAtt::type() };
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Imu observation used to calculate the integrated navigation solution
    std::shared_ptr<const ImuObs> imuObs = nullptr;
};

} // namespace NAV
