// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeOutputs.hpp
/// @brief Binary Group 3 – IMU Outputs
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <cstdint>
#include <util/Eigen.hpp>

#include <vn/types.h>

namespace NAV::vendor::vectornav
{
/// @brief Binary Group 3 – IMU Outputs
struct ImuOutputs
{
    /// @brief Available data in this struct
    vn::protocol::uart::ImuGroup imuField = vn::protocol::uart::ImuGroup::IMUGROUP_NONE;

    /// @brief Reserved for future use.
    ///
    /// Status is reserved for future use. Not currently used in the current code, as such will always report 0.
    uint16_t imuStatus{};

    /// @brief Uncompensated magnetic measurement.
    ///
    /// The IMU magnetic field measured in units of Gauss, given in the body frame. This measurement is
    /// compensated by the static calibration (individual factory calibration stored in flash), and the user
    /// compensation, however it is not compensated by the onboard Hard/Soft Iron estimator.
    Eigen::Vector3f uncompMag{};

    /// @brief Uncompensated acceleration measurement.
    ///
    /// The IMU acceleration measured in units of m/s^2, given in the body frame. This measurement is
    /// compensated by the static calibration (individual factory calibration stored in flash), however it is not
    /// compensated by any dynamic calibration such as bias compensation from the onboard INS Kalman filter.
    Eigen::Vector3f uncompAccel{};

    /// @brief Uncompensated angular rate measurement.
    ///
    /// The IMU angular rate measured in units of rad/s, given in the body frame. This measurement is compensated
    /// by the static calibration (individual factory calibration stored in flash), however it is not compensated by any
    /// dynamic calibration such as the bias compensation from the onboard AHRS/INS Kalman filters.
    Eigen::Vector3f uncompGyro{};

    /// @brief Temperature measurement.
    ///
    /// The IMU temperature measured in units of Celsius.
    float temp{};

    /// @brief Pressure measurement.
    ///
    /// The IMU pressure measured in kilopascals. This is an absolute pressure measurement. Typical pressure at
    /// sea level would be around 100 kPa.
    float pres{};

    /// @brief Delta time.
    ///
    /// The delta time (dtime) is the time interval that the delta angle and velocities are integrated over.
    /// Delta time is given in seconds.
    float deltaTime{};

    /// @brief Delta theta angles.
    ///
    /// The delta time (dtime) is the time interval that the delta angle and velocities are integrated over.
    /// The delta theta (dtheta) is the delta rotation angles incurred due to rotation, by the local body reference frame, since
    /// the last time the values were outputted by the device. The delta velocity (dvel) is the delta velocity incurred
    /// due to motion, by the local body reference frame, since the last time the values were outputted by the device.
    /// The frame of reference of these delta measurements are determined by the IntegrationFrame field in the
    /// Delta Theta and Delta Velocity Configuration Register (Register 82). These delta angles and delta velocities
    /// are calculated based upon the onboard coning and sculling integration performed onboard the sensor at the
    /// full IMU rate (default 800Hz). The integration for both the delta angles and velocities are reset each time
    /// either of the values are either polled or sent out due to a scheduled asynchronous ASCII or binary output.
    /// Delta Theta and Delta Velocity values correctly capture the nonlinearities involved in measuring motion from
    /// a rotating strapdown platform (as opposed to the older mechanically inertial navigation systems), thus
    /// providing you with the ability to integrate velocity and angular rate at much lower speeds (say for example
    /// 10 Hz, reducing bandwidth and computational complexity), while still maintaining the same numeric
    /// precision as if you had performed the integration at the full IMU measurement rate of 800Hz. Time is given
    /// in seconds. Delta angles are given in degrees.
    Eigen::Vector3f deltaTheta{};

    /// @brief Delta velocity.
    ///
    /// The delta velocity (dvel) is the delta velocity incurred due to motion, since the last time the values were output
    /// by the device. The delta velocities are calculated based upon the onboard conning and sculling integration
    /// performed onboard the sensor at the IMU sampling rate (nominally 800Hz). The integration for the delta
    /// velocities are reset each time the values are either polled or sent out due to a scheduled asynchronous ASCII
    /// or binary output. Delta velocity is given in meters per second.
    Eigen::Vector3f deltaV{};

    /// @brief Compensated magnetic measurement.
    ///
    /// The IMU compensated magnetic field measured units of Gauss, and given in the body frame. This
    /// measurement is compensated by the static calibration (individual factory calibration stored in flash), the user
    /// compensation, and the dynamic calibration from the onboard Hard/Soft Iron estimator.
    Eigen::Vector3f mag{};

    /// @brief Compensated acceleration measurement.
    ///
    /// The compensated acceleration measured in units of m/s^2, and given in the body frame. This measurement
    /// is compensated by the static calibration (individual factory calibration stored in flash), the user
    /// compensation, and the dynamic bias compensation from the onboard INS Kalman filter.
    Eigen::Vector3f accel{};

    /// @brief Compensated angular rate measurement.
    ///
    /// The compensated angular rate measured in units of rad/s, and given in the body frame. This measurement
    /// is compensated by the static calibration (individual factor calibration stored in flash), the user compensation,
    /// and the dynamic bias compensation from the onboard INS Kalman filter.
    Eigen::Vector3f angularRate{};
};

} // namespace NAV::vendor::vectornav