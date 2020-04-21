/**
 * @file Imu.hpp
 * @brief Abstract IMU Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "Nodes/Node.hpp"

namespace NAV
{
/// Abstract IMU Class
class Imu : public Node
{
  protected:
    /**
     * @brief Construct a new Imu object
     * 
     * @param[in] name Name of the Imu
     * @param[in, out] options Program options string list
     */
    Imu(std::string name, std::deque<std::string>& options);

    /// Destroy the Imu object
    ~Imu();
};

} // namespace NAV
