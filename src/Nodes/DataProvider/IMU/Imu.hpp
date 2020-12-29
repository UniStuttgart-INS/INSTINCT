/// @file Imu.hpp
/// @brief Abstract IMU Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Node.hpp"

#include "NodeData/IMU/ImuPos.hpp"

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

    /// Position and rotation information for conversion from platform to body frame
    [[nodiscard]] const ImuPos& imuPosition() const { return imuPos; }

  protected:
    /// @brief Default constructor
    Imu() = default;

    /// @brief Destructor
    ~Imu() override = default;

    /// Position and rotation information for conversion from platform to body frame
    ImuPos imuPos;
};

} // namespace NAV
