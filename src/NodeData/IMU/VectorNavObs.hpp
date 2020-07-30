/// @file VectorNavObs.hpp
/// @brief Data storage class for one VectorNav observation
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "ImuObs.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class VectorNavObs final : public ImuObs
{
  public:
    /// @brief Default constructor
    VectorNavObs() = default;
    /// @brief Destructor
    ~VectorNavObs() final = default;
    /// @brief Copy constructor
    VectorNavObs(const VectorNavObs&) = delete;
    /// @brief Move constructor
    VectorNavObs(VectorNavObs&&) = delete;
    /// @brief Copy assignment operator
    VectorNavObs& operator=(const VectorNavObs&) = delete;
    /// @brief Move assignment operator
    VectorNavObs& operator=(VectorNavObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("VectorNavObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "ImuObs" };
        return parents;
    }

    /// The estimated attitude quaternion. The first term is the scalar value.
    /// The attitude is given as the body frame with respect to the local North East Down (NED) frame.
    std::optional<Eigen::Quaterniond> quaternion;

    /// The estimated attitude Yaw, Pitch, and Roll angles measured in [degrees].
    /// The attitude is given as a 3,2,1 Euler angle sequence describing the body frame
    /// with respect to the local North East Down (NED) frame.
    /// Yaw   +/- 180°
    /// Pitch +/- 90°
    /// Roll  +/- 180°
    std::optional<Eigen::Array3d> yawPitchRoll;

    /// The time since the last SyncIn event trigger expressed in [nano seconds]
    std::optional<uint64_t> timeSinceSyncIn;
    /// The number of SyncIn trigger events that have occurred.
    std::optional<uint32_t> syncInCnt;

    /// The compensated magnetic field measured in units of [Gauss], and given in the body frame.
    std::optional<Eigen::Vector3d> magCompXYZ;
    /// The compensated acceleration measured in units of [m/s^2], and given in the body frame.
    std::optional<Eigen::Vector3d> accelCompXYZ;
    /// The compensated angular rate measured in units of [rad/s], and given in the body frame.
    std::optional<Eigen::Vector3d> gyroCompXYZ;

    /// The time interval that the delta angle and velocities are integrated over in [seconds].
    std::optional<double> dtime;
    /// The delta rotation angles in [degree] incurred due to rotation, by the local body reference frame,
    /// since the last time the values were outputted by the device.
    std::optional<Eigen::Array3d> dtheta;
    /// The delta velocity in [m/s] incurred due to motion, by the local body reference frame,
    /// since the last time the values were outputted by the device.
    std::optional<Eigen::Vector3d> dvel;

    /// @brief The VPE status bitfield
    ///
    /// Bit | Name                    | Description
    /// 0+1 | AttitudeQuality         | Provides an indication of the quality of the attitude solution. 0 - Excellent, 1 - Good, 2 - Bad, 3 - Not tracking
    ///  2  | GyroSaturation          | At least one gyro axis is currently saturated.
    ///  3  | GyroSaturationRecovery  | Filter is in the process of recovering from a gyro saturation event.
    /// 4+5 | MagDisturbance          | A magnetic DC disturbance has been detected. 0 – No magnetic disturbance. 1 to 3 – Magnetic disturbance is present.
    ///  6  | MagSaturation           | At least one magnetometer axis is currently saturated.
    /// 7+8 | AccDisturbance          | A strong acceleration disturbance has been detected. 0 – No acceleration disturbance. 1 to 3 – Acceleration disturbance has been detected.
    ///  9  | AccSaturation           | At least one accelerometer axis is currently saturated.
    /// 11  | KnownMagDisturbance     | A known magnetic disturbance has been reported by the user and the magnetometer is currently tuned out.
    /// 12  | KnownAccelDisturbance   | A known acceleration disturbance has been reported by the user and the accelerometer is currently tuned out.
    class VpeStatus
    {
      public:
        /// Constructor
        explicit VpeStatus(uint16_t status) : status(status) {}

        /// @brief Assignment operator
        /// @param[in] status Status to set
        VpeStatus& operator=(const uint16_t& status)
        {
            this->status = status;
            return *this;
        }

        /// @brief Default constructor
        VpeStatus() = default;
        /// @brief Destructor
        ~VpeStatus() = default;
        /// @brief Copy constructor
        VpeStatus(const VpeStatus&) = delete;
        /// @brief Move constructor
        VpeStatus(VpeStatus&&) = delete;
        /// @brief Copy assignment operator
        VpeStatus& operator=(const VpeStatus&) = delete;
        /// @brief Move assignment operator
        VpeStatus& operator=(VpeStatus&&) = delete;

        /// The storage field
        uint16_t status;

        /// Extract the attitude quality from the vpe status
        [[nodiscard]] constexpr uint8_t attitudeQuality() const
        {
            return ((status & (1U << 0U | 1U << 1U)) >> 0U);
        }
        /// Extract the gyro saturation from the vpe status
        [[nodiscard]] constexpr uint8_t gyroSaturation() const
        {
            return ((status & (1U << 2U)) >> 2U); // NOLINT
        }
        /// Extract the gyro saturation recovery from the vpe status
        [[nodiscard]] constexpr uint8_t gyroSaturationRecovery() const
        {
            return ((status & (1U << 3U)) >> 3U); // NOLINT
        }
        /// Extract the magnetic disturbance from the vpe status
        [[nodiscard]] constexpr uint8_t magDisturbance() const
        {
            return ((status & (1U << 4U | 1U << 5U)) >> 4U); // NOLINT
        }
        /// Extract the magnetic saturation from the vpe status
        [[nodiscard]] constexpr uint8_t magSaturation() const
        {
            return ((status & (1U << 6U)) >> 6U); // NOLINT
        }
        /// Extract the acceleration disturbance from the vpe status
        [[nodiscard]] constexpr uint8_t accDisturbance() const
        {
            return ((status & (1U << 7U | 1U << 8U)) >> 7U); // NOLINT
        }
        /// Extract the acceleration saturation from the vpe status
        [[nodiscard]] constexpr uint8_t accSaturation() const
        {
            return ((status & (1U << 9U)) >> 9U); // NOLINT
        }
        /// Extract the known magnetic disturbance from the vpe status
        [[nodiscard]] constexpr uint8_t knownMagDisturbance() const
        {
            return ((status & (1U << 11U)) >> 11U); // NOLINT
        }
        /// Extract the known acceleration disturbance from the vpe status
        [[nodiscard]] constexpr uint8_t knownAccelDisturbance() const
        {
            return ((status & (1U << 12U)) >> 12U); // NOLINT
        }
    };

    /// @brief The VPE status bitfield
    std::optional<VpeStatus> vpeStatus;

    /// The absolute IMU pressure measured in [kPa].
    std::optional<double> pressure;

    /// The compensated magnetic field measured in units of [Gauss], and given in the North East Down (NED) frame.
    std::optional<Eigen::Vector3d> magCompNED;
    /// The compensated acceleration (with gravity) measured in units of [m/s^2], and given in the North East Down (NED) frame.
    std::optional<Eigen::Vector3d> accelCompNED;
    /// The compensated angular rate measured in units of [rad/s], and given in the North East Down (NED) frame.
    std::optional<Eigen::Vector3d> gyroCompNED;

    /// The estimated linear acceleration (without gravity) reported in [m/s^2], and given in the body frame.
    std::optional<Eigen::Vector3d> linearAccelXYZ;
    /// The estimated linear acceleration (without gravity) reported in [m/s^2], and given in the North East Down (NED) frame.
    std::optional<Eigen::Vector3d> linearAccelNED;

    /// The estimated attitude (Yaw, Pitch, Roll) uncertainty (1 Sigma), reported in degrees.
    std::optional<Eigen::Array3d> yawPitchRollUncertainty;
};

} // namespace NAV
