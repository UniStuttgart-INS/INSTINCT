/// @file Imu.hpp
/// @brief Abstract IMU Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Nodes/Node.hpp"

namespace NAV
{
/// Abstract IMU Class
class Imu : public Node
{
  public:
    /// @brief Copy constructor
    Imu(const Imu&) = delete;
    /// @brief Move constructor
    Imu(Imu&&) = delete;
    /// @brief Copy assignment operator
    Imu& operator=(const Imu&) = delete;
    /// @brief Move assignment operator
    Imu& operator=(Imu&&) = delete;

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Imu
    /// @param[in] options Program options string map
    Imu(const std::string& name, const std::map<std::string, std::string>& options);

    /// Default constructor
    Imu() = default;

    /// Destructor
    ~Imu() override = default;
};

} // namespace NAV
