/// @file InertialNavSol.hpp
/// @brief
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
        return std::string("InertialNavSol");
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
