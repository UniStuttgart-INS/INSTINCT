// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PinData.hpp
/// @brief Information about a sensor which is connected to a certain pin
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2024-04-04

#pragma once

#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Information about a sensor which is connected to a certain pin (i.e. sensor characteristics defined in GUI)
struct PinData
{
    // ------------------------------------- State and variance units ----------------------------------------
    /// Possible Units for the angular rate
    enum class AngRateUnit : uint8_t
    {
        deg_s, ///< in [deg/s, deg/s, deg/s]
        rad_s, ///< in [rad/s, rad/s, rad/s]
    };

    /// Possible Units for the acceleration
    enum class AccelerationUnit : uint8_t
    {
        m_s2, ///< in [m/s², m/s², m/s²]
    };

    /// Possible Units for the variance for the process noise of the angular rate (standard deviation σ or Variance σ²)
    enum class AngRateVarianceUnit : uint8_t
    {
        rad2_s2, ///< Variance [rad²/s², rad²/s², rad²/s²]
        rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
        deg2_s2, ///< Variance [deg²/s², deg²/s², deg²/s²]
        deg_s,   ///< Standard deviation [deg/s, deg/s, deg/s]
    };

    /// Possible Units for the variance for the process noise of the acceleration (standard deviation σ or Variance σ²)
    enum class AccelerationVarianceUnit : uint8_t
    {
        m2_s4, ///< Variance [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        m_s2,  ///< Standard deviation [m/s², m/s², m/s²]
    };

    // ---------------------------------------- Unit initialization ------------------------------------------

    /// GUI selection for the unit of the initial angular rate bias
    AngRateUnit initAngularRateBiasUnit = AngRateUnit::deg_s;
    /// GUI selection for the unit of the initial angular acceleration bias
    AccelerationUnit initAccelerationBiasUnit = AccelerationUnit::m_s2;

    /// Gui selection for the Unit of the initial covariance for the angular rate
    AngRateVarianceUnit initCovarianceAngularRateUnit = AngRateVarianceUnit::deg_s;
    /// Gui selection for the Unit of the initial covariance for the acceleration
    AccelerationVarianceUnit initCovarianceAccelerationUnit = AccelerationVarianceUnit::m_s2;
    /// Gui selection for the Unit of the initial covariance for the angular rate biases
    AngRateVarianceUnit initCovarianceBiasAngRateUnit = AngRateVarianceUnit::deg_s;
    /// Gui selection for the Unit of the initial covariance for the acceleration biases
    AccelerationVarianceUnit initCovarianceBiasAccUnit = AccelerationVarianceUnit::m_s2;
    /// Gui selection for the Unit of the process noise of the angular rate
    AngRateVarianceUnit varBiasAngRateNoiseUnit = AngRateVarianceUnit::deg_s;
    /// Gui selection for the Unit of the process noise of the acceleration
    AccelerationVarianceUnit varBiasAccelerationNoiseUnit = AccelerationVarianceUnit::m_s2;
    /// Gui selection for the unit of the angular rate's measurement uncertainty
    AngRateVarianceUnit measurementUncertaintyAngularRateUnit = AngRateVarianceUnit::deg_s;
    /// Gui selection for the unit of the acceleration's measurement uncertainty
    AccelerationVarianceUnit measurementUncertaintyAccelerationUnit = AccelerationVarianceUnit::m_s2;

    // --------------------------------------- State initialization ------------------------------------------

    /// GUI selection for the initial angular rate bias
    Eigen::Vector3d initAngularRateBias{ 0.0, 0.0, 0.0 };
    /// GUI selection for the initial acceleration bias
    Eigen::Vector3d initAccelerationBias{ 0.0, 0.0, 0.0 };

    /// GUI selection for the initial covariance diagonal values for angular rate (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceAngularRate{ 1.0, 1.0, 1.0 };
    /// GUI selection for the initial covariance diagonal values for acceleration (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceAcceleration{ 0.1, 0.1, 0.1 };
    /// GUI selection for the initial covariance diagonal values for angular rate biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceBiasAngRate{ 1.0, 1.0, 1.0 };
    /// GUI selection for the initial covariance diagonal values for acceleration biases (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceBiasAcc{ 0.1, 0.1, 0.1 };
    /// GUI selection for the process noise of the angular rate diagonal values (standard deviation σ or Variance σ²)
    Eigen::Vector3d varBiasAngRateNoise = { 1.0, 1.0, 1.0 };
    /// GUI selection for the process noise of the acceleration diagonal values (standard deviation σ or Variance σ²)
    Eigen::Vector3d varBiasAccelerationNoise{ 0.1, 0.1, 0.1 };
    /// Gui selection for the angular rate measurement uncertainty diagonal values
    Eigen::Vector3d measurementUncertaintyAngularRate{ 1.0, 1.0, 1.0 };
    /// Gui selection for the acceleration measurement uncertainty diagonal values
    Eigen::Vector3d measurementUncertaintyAcceleration{ 0.1, 0.1, 0.1 };
};

/// @brief Sensor information specific to the IRW-KF
struct PinDataIRWKF
{
    // ----------------------------------- State and variance units --------------------------------------
    /// Possible Units for the angular acceleration
    enum class AngularAccUnit : uint8_t
    {
        deg_s2, ///< in [deg/s², deg/s², deg/s²]
        rad_s2, ///< in [rad/s², rad/s², rad/s²]
    };
    /// Possible Units for the jerk
    enum class JerkUnit : uint8_t
    {
        m_s3, ///< in [m/s³, m/s³, m/s³]
    };

    /// Possible Units for the variance for the process noise of the angular acceleration (standard deviation σ or Variance σ²)
    enum class AngularAccVarianceUnit : uint8_t
    {
        rad2_s4, ///< Variance [(rad^2)/(s^4), (rad^2)/(s^4), (rad^2)/(s^4)]
        rad_s2,  ///< Standard deviation [rad/s², rad/s², rad/s²]
        deg2_s4, ///< Variance [(deg^2)/(s^4), (deg^2)/(s^4), (deg^2)/(s^4)]
        deg_s2,  ///< Standard deviation [deg/s², deg/s², deg/s²]
    };
    /// Possible Units for the variance for the process noise of the jerk (standard deviation σ or Variance σ²)
    enum class JerkVarianceUnit : uint8_t
    {
        m2_s6, ///< Variance [(m^2)/(s^6), (m^2)/(s^6), (m^2)/(s^6)]
        m_s3,  ///< Standard deviation [m/s³, m/s³, m/s³]
    };

    // -------------------------------------- Unit initialization ----------------------------------------

    /// Gui selection for the unit of the initial angular rate
    PinData::AngRateUnit initAngularRateUnit = PinData::AngRateUnit::deg_s;
    /// Gui selection for the unit of the initial angular acceleration
    AngularAccUnit initAngularAccUnit = AngularAccUnit::deg_s2;
    /// Gui selection for the unit of the initial acceleration
    PinData::AccelerationUnit initAccelerationUnit = PinData::AccelerationUnit::m_s2;
    /// Gui selection for the Unit of the initial jerk
    JerkUnit initJerkUnit = JerkUnit::m_s3;

    /// Gui selection for the unit of the initial covariance for the angular acceleration
    AngularAccVarianceUnit initCovarianceAngularAccUnit = AngularAccVarianceUnit::deg_s2;
    /// Gui selection for the unit of the initial covariance for the jerk
    JerkVarianceUnit initCovarianceJerkUnit = JerkVarianceUnit::m_s3;

    /// Gui selection for the unit of the angular acceleration process noise
    AngularAccVarianceUnit varAngularAccNoiseUnit = AngularAccVarianceUnit::deg_s2;
    /// Gui selection for the unit of the jerk process noise
    JerkVarianceUnit varJerkNoiseUnit = JerkVarianceUnit::m_s3;

    // ------------------------------------- State initialization ----------------------------------------

    /// GUI selection for the initial angular rate
    Eigen::Vector3d initAngularRate{ 0.0, 0.0, 0.0 };
    /// GUI selection for the initial angular acceleration
    Eigen::Vector3d initAngularAcc{ 0.0, 0.0, 0.0 };
    /// GUI selection for the initial acceleration
    Eigen::Vector3d initAcceleration{ 0.0, 0.0, 0.0 };
    /// GUI selection for the initial jerk
    Eigen::Vector3d initJerk{ 0.0, 0.0, 0.0 };

    /// GUI selection for the initial covariance diagonal values for angular acceleration (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceAngularAcc{ 0.1, 0.1, 0.1 };
    /// GUI selection for the initial covariance diagonal values for jerk (standard deviation σ or Variance σ²)
    Eigen::Vector3d initCovarianceJerk{ 0.1, 0.1, 0.1 };

    /// GUI selection for the angular acceleration process noise diagonal values
    Eigen::Vector3d varAngularAccNoise{ 0.1, 0.1, 0.1 };
    /// GUI selection for the jerk process noise diagonal values
    Eigen::Vector3d varJerkNoise{ 0.1, 0.1, 0.1 };
};

/// @brief Sensor information specific to the Bspline-KF
struct PinDataBsplineKF
{
    // -------------------------------------- Unit initialization ----------------------------------------

    /// Gui selection for the unit of the initial coefficients of the angular rate B-splines
    PinData::AngRateUnit initCoeffsAngularRateUnit = PinData::AngRateUnit::deg_s;
    /// Gui selection for the unit of the initial coefficients of the acceleration B-splines
    PinData::AccelerationUnit initCoeffsAccelUnit = PinData::AccelerationUnit::m_s2;

    /// Gui selection for the unit of the initial covariance for the coefficients of angular rate
    PinData::AngRateVarianceUnit initCovarianceCoeffsAngRateUnit = PinData::AngRateVarianceUnit::deg_s;
    /// Gui selection for the unit of the initial covariance for the coefficients of acceleration
    PinData::AccelerationVarianceUnit initCovarianceCoeffsAccelUnit = PinData::AccelerationVarianceUnit::m_s2;

    /// GUI selection for the B-spline coeffs of the angular rate process noise (diagonal values)
    PinData::AngRateVarianceUnit varCoeffsAngRateUnit = PinData::AngRateVarianceUnit::deg_s;
    /// GUI selection for the B-spline coeffs of the acceleration process noise (diagonal values)
    PinData::AccelerationVarianceUnit varCoeffsAccelUnit = PinData::AccelerationVarianceUnit::m_s2;

    // ------------------------------------- State initialization ----------------------------------------

    /// GUI selection for the initial B-spline coefficients of the angular rate
    Eigen::VectorXd initCoeffsAngRate = Eigen::VectorXd::Zero(9);
    /// GUI selection for the initial B-spline coefficients of the acceleration
    Eigen::VectorXd initCoeffsAccel = Eigen::VectorXd::Zero(9);

    /// GUI selection for the initial covariance of the B-spline coefficients of the angular rate
    Eigen::VectorXd initCovarianceCoeffsAngRate = Eigen::VectorXd::Ones(9);
    /// GUI selection for the initial covariance of the B-spline coefficients of the acceleration
    Eigen::VectorXd initCovarianceCoeffsAccel = Eigen::VectorXd::Ones(9);

    /// GUI selection for the coeffs of the angular rate process noise (diagonal values)
    Eigen::VectorXd varCoeffsAngRateNoise = Eigen::VectorXd::Ones(9) * 0.1;
    /// GUI selection for the coeffs of the acceleration process noise (diagonal values)
    Eigen::VectorXd varCoeffsAccelNoise = Eigen::VectorXd::Ones(9) * 0.1;
};
} // namespace NAV