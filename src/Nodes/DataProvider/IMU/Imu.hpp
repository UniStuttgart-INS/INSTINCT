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
  public:
    Imu(const Imu&) = delete;            ///< Copy constructor
    Imu(Imu&&) = delete;                 ///< Move constructor
    Imu& operator=(const Imu&) = delete; ///< Copy assignment operator
    Imu& operator=(Imu&&) = delete;      ///< Move assignment operator

  protected:
    /**
     * @brief Construct a new Imu object
     * 
     * @param[in] name Name of the Imu
     * @param[in] options Program options string map
     */
    Imu(const std::string& name, const std::map<std::string, std::string>& options);

    /// Default constructor
    Imu() = default;

    /// Destructor
    ~Imu() override = default;
};

} // namespace NAV
