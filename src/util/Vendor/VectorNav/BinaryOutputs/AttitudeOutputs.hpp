/// @file TimeOutputs.hpp
/// @brief Binary Group 5 – Attitude Outputs
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <cstdint>
#include <util/Eigen.hpp>

#include <vn/types.h>
#include "util/Vendor/VectorNav/VectorNavTypes.hpp"

namespace NAV::vendor::vectornav
{
/// @brief Binary Group 5 – Attitude Outputs
struct AttitudeOutputs
{
    /// @brief Available data in this struct
    vn::protocol::uart::AttitudeGroup attitudeField = vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE;

    /// @brief VPE Status
    ///
    /// The VPE status bitfield.
    VpeStatus vpeStatus{};

    /// @brief Yaw Pitch Roll
    ///
    /// The estimated attitude Yaw, Pitch, and Roll angles measured in degrees. The attitude is given as a 3,2,1 Euler
    /// angle sequence describing the body frame with respect to the local North East Down (NED) frame.
    /// Yaw   +/- 180°
    /// Pitch +/- 90°
    /// Roll  +/- 180°
    Eigen::Vector3f ypr{};

    /// @brief Quaternion
    ///
    /// The estimated attitude quaternion. The last term is the scalar value. The attitude is given as the body frame
    /// with respect to the local North East Down (NED) frame.
    Eigen::Quaternionf qtn{};

    /// @brief Directional Cosine Matrix
    ///
    /// The estimated attitude directional cosine matrix given in column major order. The DCM maps vectors from
    /// the North East Down (NED) frame into the body frame.
    Eigen::Matrix3f dcm{};

    /// @brief Compensated magnetic (NED)
    ///
    /// The current estimated magnetic field (Gauss), given in the North East Down (NED) frame. The current
    /// attitude solution is used to map the measurement from the measured body frame to the inertial (NED) frame.
    /// This measurement is compensated by both the static calibration (individual factory calibration stored in
    /// flash), and the dynamic calibration such as the user or onboard Hard/Soft Iron compensation registers.
    Eigen::Vector3f magNed{};

    /// @brief Compensated acceleration (NED)
    ///
    /// The estimated acceleration (with gravity) reported in m/s^2, given in the North East Down (NED) frame. The
    /// acceleration measurement has been bias compensated by the onboard INS filter. This measurement is
    /// attitude dependent, since the attitude is used to map the measurement from the body frame into the inertial
    /// (NED) frame. If the device is stationary and the INS filter is tracking, the measurement should be nominally
    /// equivalent to the gravity reference vector in the inertial frame (NED).
    Eigen::Vector3f accelNed{};

    /// @brief Compensated linear acceleration (no gravity)
    ///
    /// The estimated linear acceleration (without gravity) reported in m/s^2, and given in the body frame. The
    /// acceleration measurement has been bias compensated by the onboard INS filter, and the gravity component
    /// has been removed using the current gravity reference vector model. This measurement is attitude
    /// dependent, since the attitude solution is required to map the gravity reference vector (known in the inertial
    /// NED frame), into the body frame so that it can be removed from the measurement. If the device is stationary
    /// and the onboard INS filter is tracking, the measurement nominally will read 0 in all three axes.
    Eigen::Vector3f linearAccelBody{};

    /// @brief Compensated linear acceleration (no gravity) (NED)
    ///
    /// The estimated linear acceleration (without gravity) reported in m/s^2, and given in the North East Down (NED)
    /// frame. This measurement is attitude dependent as the attitude solution is used to map the measurement
    /// from the body frame into the inertial (NED) frame. This acceleration measurement has been bias
    /// compensated by the onboard INS filter, and the gravity component has been removed using the current
    /// gravity reference vector estimate. If the device is stationary and the onboard INS filter is tracking, the
    /// measurement nominally will read 0 in all three axes.
    Eigen::Vector3f linearAccelNed{};

    /// @brief Yaw Pitch Roll uncertainty
    ///
    /// The estimated attitude (Yaw, Pitch, Roll) uncertainty (1 Sigma), reported in degrees.
    ///
    /// The estimated attitude (YprU) field is not valid when the INS Scenario mode in the INS Basic
    /// Configuration register is set to AHRS mode. See the INS Basic Configuration Register in the INS
    /// section for more details.
    Eigen::Vector3f yprU{};
};

} // namespace NAV::vendor::vectornav