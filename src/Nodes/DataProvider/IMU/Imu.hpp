/// @file Imu.hpp
/// @brief Abstract IMU Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "Nodes/Node.hpp"

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

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Imu
    /// @param[in] options Program options string map
    Imu(const std::string& name, const std::map<std::string, std::string>& options);

    /// Default constructor
    Imu() = default;

    /// Destructor
    ~Imu() override = default;

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const override
    {
        return { { CONFIG_FLOAT3, "Accel pos", "Accelerometer mounting position in body frame coordinates in [m]", { "-1000", "0", "1000", "3", "-1000", "0", "1000", "3", "-1000", "0", "1000", "3" } },
                 { CONFIG_FLOAT3, "Accel rot", "Angles between accelerometer frame and body frame axis [Degree]", { "0", "0", "360", "2", "0", "0", "360", "2", "0", "0", "360", "2" } },
                 { CONFIG_FLOAT3, "Gyro pos", "Gyroscope mounting position in body frame coordinates in [m]", { "-1000", "0", "1000", "3", "-1000", "0", "1000", "3", "-1000", "0", "1000", "3" } },
                 { CONFIG_FLOAT3, "Gyro rot", "Angles between gyroscope frame and body frame axis [Degree]", { "0", "0", "360", "2", "0", "0", "360", "2", "0", "0", "360", "2" } },
                 { CONFIG_FLOAT3, "Mag pos", "Magnetometer mounting position in body frame coordinates in [m]", { "-1000", "0", "1000", "3", "-1000", "0", "1000", "3", "-1000", "0", "1000", "3" } },
                 { CONFIG_FLOAT3, "Mag rot", "Angles between magnetometer frame and body frame axis [Degree]", { "0", "0", "360", "2", "0", "0", "360", "2", "0", "0", "360", "2" } } };
    }

    /// Position and rotation information for conversion from platform to body frame
    std::shared_ptr<ImuPos> imuPos;
};

} // namespace NAV
