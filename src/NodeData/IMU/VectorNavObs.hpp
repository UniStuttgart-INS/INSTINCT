/**
 * @file VectorNavObs.hpp
 * @brief Data storage class for one VectorNav observation
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#pragma once

#include "ImuObs.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class VectorNavObs : public ImuObs
{
  public:
    /** The system time since startup measured in [nano seconds]. */
    std::optional<uint64_t> timeSinceStartup;

    /** The time since the last SyncIn event trigger expressed in [nano seconds] */
    std::optional<uint64_t> timeSinceSyncIn;
    /** The number of SyncIn trigger events that have occurred. */
    std::optional<uint32_t> syncInCnt;

    /** The time interval that the delta angle and velocities are integrated over in [seconds]. */
    std::optional<float> dtime;
    /** The delta rotation angles in [degree] incurred due to rotation, by the local body reference frame, since the last time the values were outputted by the device. */
    std::optional<Eigen::Array3d> dtheta;
    /** The delta velocity in [m/s] incurred due to motion, by the local body reference frame, since the last time the values were outputted by the device. */
    std::optional<Eigen::Vector3d> dvel;

    /** @brief The VPE status bitfield.
     * 
     *  Bit | Name                    | Description
     *  0+1 | AttitudeQuality         | Provides an indication of the quality of the attitude solution. 0 - Excellent, 1 - Good, 2 - Bad, 3 - Not tracking
     *   2  | GyroSaturation          | At least one gyro axis is currently saturated.
     *   3  | GyroSaturationRecovery  | Filter is in the process of recovering from a gyro saturation event.
     *  4+5 | MagDisturbance          | A magnetic DC disturbance has been detected. 0 – No magnetic disturbance. 1 to 3 – Magnetic disturbance is present.
     *   6  | MagSaturation           | At least one magnetometer axis is currently saturated.
     *  7+8 | AccDisturbance          | A strong acceleration disturbance has been detected. 0 – No acceleration disturbance. 1 to 3 – Acceleration disturbance has been detected.
     *   9  | AccSaturation           | At least one accelerometer axis is currently saturated.
     *  11  | KnownMagDisturbance     | A known magnetic disturbance has been reported by the user and the magnetometer is currently tuned out.
     *  12  | KnownAccelDisturbance   | A known acceleration disturbance has been reported by the user and the accelerometer is currently tuned out.
     */
    std::optional<uint16_t> vpeStatus;

    /** The IMU temperature measured in units of [Celsius]. */
    std::optional<double> temperature;
    /** The absolute IMU pressure measured in [kPa]. */
    std::optional<double> pressure;

    /** The compensated magnetic field measured in units of [Gauss], and given in the North East Down (NED) frame. */
    std::optional<Eigen::Vector3d> magCompNED;
    /** The compensated acceleration (with gravity) measured in units of [m/s^2], and given in the North East Down (NED) frame. */
    std::optional<Eigen::Vector3d> accelCompNED;
    /** The compensated angular rate measured in units of [rad/s], and given in the North East Down (NED) frame. */
    std::optional<Eigen::Vector3d> gyroCompNED;

    /** The estimated linear acceleration (without gravity) reported in [m/s^2], and given in the body frame. */
    std::optional<Eigen::Vector3d> linearAccelXYZ;
    /** The estimated linear acceleration (without gravity) reported in [m/s^2], and given in the North East Down (NED) frame. */
    std::optional<Eigen::Vector3d> linearAccelNED;

    /** The estimated attitude (Yaw, Pitch, Roll) uncertainty (1 Sigma), reported in degrees. */
    std::optional<Eigen::Array3d> yawPitchRollUncertainty;

  public: // Inline functions
    /// Extract the attitude quality from the vpe status
    inline constexpr uint8_t attitudeQuality()
    {
        return ((vpeStatus.value() & (1 << 0 | 1 << 1)) >> 0);
    }
    /// Extract the gyro saturation from the vpe status
    inline constexpr uint8_t gyroSaturation()
    {
        return ((vpeStatus.value() & (1 << 2)) >> 2);
    }
    /// Extract the gyro saturation recovery from the vpe status
    inline constexpr uint8_t gyroSaturationRecovery()
    {
        return ((vpeStatus.value() & (1 << 3)) >> 3);
    }
    /// Extract the magnetic disturbance from the vpe status
    inline constexpr uint8_t magDisturbance()
    {
        return ((vpeStatus.value() & (1 << 4 | 1 << 5)) >> 4);
    }
    /// Extract the magnetic saturation from the vpe status
    inline constexpr uint8_t magSaturation()
    {
        return ((vpeStatus.value() & (1 << 6)) >> 6);
    }
    /// Extract the acceleration disturbance from the vpe status
    inline constexpr uint8_t accDisturbance()
    {
        return ((vpeStatus.value() & (1 << 7 | 1 << 8)) >> 7);
    }
    /// Extract the acceleration saturation from the vpe status
    inline constexpr uint8_t accSaturation()
    {
        return ((vpeStatus.value() & (1 << 9)) >> 9);
    }
    /// Extract the known magnetic disturbance from the vpe status
    inline constexpr uint8_t knownMagDisturbance()
    {
        return ((vpeStatus.value() & (1 << 11)) >> 11);
    }
    /// Extract the known acceleration disturbance from the vpe status
    inline constexpr uint8_t knownAccelDisturbance()
    {
        return ((vpeStatus.value() & (1 << 12)) >> 12);
    }
};

} // namespace NAV
