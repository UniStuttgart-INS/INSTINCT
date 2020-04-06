/**
 * @file ImuObs.hpp
 * @brief Parent Class for all IMU Observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "NodeData/InsObs.hpp"

#include "Eigen/Dense"
#include <Eigen/Geometry>

namespace NAV
{
/// IMU Observation storage class
class ImuObs : public InsObs
{
  public:
    /** The estimated attitude quaternion. The first term is the scalar value.
     *  The attitude is given as the body frame with respect to the local North East Down (NED) frame. */
    std::optional<Eigen::Quaterniond> quaternion;

    /// The IMU magnetic field measured in units of [Gauss], given in the body frame.
    std::optional<Eigen::Vector3d> magUncompXYZ;
    /// The IMU acceleration measured in units of [m/s^2], given in the body frame.
    std::optional<Eigen::Vector3d> accelUncompXYZ;
    /// The IMU angular rate measured in units of [rad/s], given in the body frame.
    std::optional<Eigen::Vector3d> gyroUncompXYZ;
    /// The compensated magnetic field measured in units of [Gauss], and given in the body frame.
    std::optional<Eigen::Vector3d> magCompXYZ;
    /// The compensated acceleration measured in units of [m/s^2], and given in the body frame.
    std::optional<Eigen::Vector3d> accelCompXYZ;
    /// The compensated angular rate measured in units of [rad/s], and given in the body frame.
    std::optional<Eigen::Vector3d> gyroCompXYZ;
};

} // namespace NAV
