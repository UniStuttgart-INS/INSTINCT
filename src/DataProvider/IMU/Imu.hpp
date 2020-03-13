/**
 * @file Imu.hpp
 * @brief Abstract IMU Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "../DataProvider.hpp"

namespace NAV
{
/// Abstract IMU Class
class Imu : public DataProvider
{
  protected:
    /**
     * @brief Construct a new Imu object
     * 
     * @param[in] name Name of the Imu
     */
    Imu(std::string name);

    /// Destroy the Imu object
    ~Imu();
};

} // namespace NAV
