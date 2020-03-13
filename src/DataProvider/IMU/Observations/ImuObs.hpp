/**
 * @file ImuObs.hpp
 * @brief Parent Class for all IMU Observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "DataProvider/InsObs.hpp"

// TODO: Replace with Eigen library
#include "util/Math/Vector.hpp"
#include "util/Math/Quaternion.hpp"

namespace NAV
{
/// IMU Observation storage class
class ImuObs : public InsObs
{
  public:
    /** The estimated attitude quaternion. The first term is the scalar value.
     *  The attitude is given as the body frame with respect to the local North East Down (NED) frame. */
    std::optional<Quaternion<double>> quaternion;

    /// The IMU magnetic field measured in units of [Gauss], given in the body frame.
    std::optional<Vector<3, double>> magUncompXYZ;
    /// The IMU acceleration measured in units of [m/s^2], given in the body frame.
    std::optional<Vector<3, double>> accelUncompXYZ;
    /// The IMU angular rate measured in units of [rad/s], given in the body frame.
    std::optional<Vector<3, double>> gyroUncompXYZ;
    /// The compensated magnetic field measured in units of [Gauss], and given in the body frame.
    std::optional<Vector<3, double>> magCompXYZ;
    /// The compensated acceleration measured in units of [m/s^2], and given in the body frame.
    std::optional<Vector<3, double>> accelCompXYZ;
    /// The compensated angular rate measured in units of [rad/s], and given in the body frame.
    std::optional<Vector<3, double>> gyroCompXYZ;
};

} // namespace NAV
