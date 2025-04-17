// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ProcessNoise.hpp
/// @brief General process Noise definitions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-31

#pragma once

#include <Eigen/Core>

#include "Navigation/GNSS/SystemModel/MotionModel.hpp"
#include "Navigation/INS/Keys.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Container/KeyedMatrix.hpp"

namespace NAV
{

/// @brief Random walk noise input matrix 𝐆
/// @param[in] sigma2 Variance of the noise on the measurements
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
Eigen::Matrix3d G_RandomWalk(const Eigen::Vector3d& sigma2);

/// @brief Submatrix 𝐆_a of the noise input matrix 𝐆
/// @param[in] sigma2 Variance of the noise on the measurements
/// @param[in] beta Gauss-Markov constant 𝛽 = 1 / 𝜏 (𝜏 correlation length)
/// @note See T. Hobiger (2021) Inertialnavigation V06 - equation (6.3)
Eigen::Matrix3d G_GaussMarkov1(const Eigen::Vector3d& sigma2, const Eigen::Vector3d& beta);

/// @brief u_bias^2 Power Spectral Density of the bias for a Gauss-Markov random process
/// @param[in] sigma2_bd 𝜎²_bd standard deviation of the bias noise
/// @param[in] tau_bd 𝜏 Correlation length in [s]
/// @note See Brown & Hwang (2011) - Introduction to Random Signals and Applied Kalman Filtering (example 9.6)
[[nodiscard]] Eigen::Vector3d psd2BiasGaussMarkov(const Eigen::Vector3d& sigma2_bd, const Eigen::Vector3d& tau_bd);

/// @brief Submatrix 𝐐_11 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_11
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d Q_psi_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const double& tau_s);

/// @brief Submatrix 𝐐_21 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_21
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_22 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_22
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_25 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ien_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] ien_Dcm_b Direction Cosine Matrix from body to {i,e,n} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_25
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d ien_Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const Eigen::Matrix3d& ien_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_31 of the system noise covariance matrix 𝐐
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_31
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_32 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_32
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_32 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_32
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s);

/// @brief Submatrix 𝐐_33 of the system noise covariance matrix 𝐐
/// @param[in] S_ra Power Spectral Density of the accelerometer random noise
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] S_rg Power Spectral Density of the gyroscope random noise
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅 in {i,e} frame
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_33
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s);

/// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_34 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] ie_Dcm_b Direction Cosine Matrix from body to {i,e} coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_34
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] n_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] T_rn_p Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] n_Dcm_b Direction Cosine Matrix from body to navigation coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d n_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_35 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] ie_F_21 Submatrix 𝐅_21 of the system matrix 𝐅
/// @param[in] ie_Dcm_b Direction Cosine Matrix from body to {i,e} coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_35
/// @note See Groves (2013) equation (14.81)
[[nodiscard]] Eigen::Matrix3d ie_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s);

/// @brief Submatrix 𝐐_42 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] b_Dcm_ien Direction Cosine Matrix from {i,e,n} to body coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_42
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s);

/// @brief Submatrix 𝐐_44 of the system noise covariance matrix 𝐐
/// @param[in] S_bad Power Spectral Density of the accelerometer bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_44
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s);

/// @brief Submatrix 𝐐_51 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] b_Dcm_ien Direction Cosine Matrix from {i,e,n} to body coordinates
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_51
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s);

/// @brief Submatrix 𝐐_55 of the system noise covariance matrix 𝐐
/// @param[in] S_bgd Power Spectral Density of the gyroscope bias variation
/// @param[in] tau_s Time interval in [s]
/// @return The 3x3 matrix 𝐐_55
/// @note See Groves (2013) equation (14.80)
[[nodiscard]] Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s);

/// @brief Submatrix 𝐐_66 of the system noise covariance matrix 𝐐
/// @param[in] S_cPhi Power Spectral Density of the receiver clock phase drift in [m^2 s^-1]
/// @param[in] S_cf Power Spectral Density of the receiver clock frequency-drift [m^2 s^-3]
/// @param[in] tau_s Time interval in [s]
/// @return See Groves (2013) equation (9.152)
[[nodiscard]] Eigen::Matrix2d Q_gnss(const double& S_cPhi, const double& S_cf, const double& tau_s);

/// @brief Calculate the noise input matrix in Earth frame coordinates
/// @param b_quat_p Quaternion from IMU platform frame to body frame
/// @param e_quat_b Quaternion from body to Earth frame
/// @param accelBiases Flag wether to calculate the rows and columns for the acceleration bias
/// @param gyroBiases Flag wether to calculate the rows and columns for the angular rate bias
/// @param accelScaleFactor Flag wether to calculate the rows and columns for the acceleration scale factor
/// @param gyroScaleFactor Flag wether to calculate the rows and columns for the angular rate scale factor
/// @param accelMisalignment Flag wether to calculate the rows and columns for the accelerometer misalignment
/// @param gyroMisalignment Flag wether to calculate the rows and columns for the gyroscope misalignment
template<typename KeyType, typename T>
[[nodiscard]] KeyedMatrixX<T, KeyType, KeyType> e_noiseInput_G(const Eigen::Quaterniond& b_quat_p,
                                                               const Eigen::Quaternion<T>& e_quat_b,
                                                               bool accelBiases, bool gyroBiases,
                                                               bool accelScaleFactor, bool gyroScaleFactor,
                                                               bool accelMisalignment, bool gyroMisalignment)

{
    std::vector<KeyType> rowKeys;
    rowKeys.reserve(3 + 3 + 4
                    + 3 * (static_cast<size_t>(accelBiases) + static_cast<size_t>(gyroBiases))
                    + 3 * (static_cast<size_t>(accelScaleFactor) + static_cast<size_t>(gyroScaleFactor))
                    + 4 * (static_cast<size_t>(accelMisalignment) + static_cast<size_t>(gyroMisalignment)));
    std::vector<KeyType> colKeys;
    colKeys.reserve(3 + 3 + 3
                    + 3 * (static_cast<size_t>(accelBiases) + static_cast<size_t>(gyroBiases))
                    + 3 * (static_cast<size_t>(accelScaleFactor) + static_cast<size_t>(gyroScaleFactor)));

    constexpr std::array<KeyType, 3> AngleErrorKeys = { Keys::AttQ1, Keys::AttQ2, Keys::AttQ3 }; // 3 quaternion keys are used for the angle error

    rowKeys.insert(rowKeys.end(), Keys::PosVelAtt<KeyType>.begin(), Keys::PosVelAtt<KeyType>.end());
    colKeys.insert(colKeys.end(), Keys::PosVel<KeyType>.begin(), Keys::PosVel<KeyType>.end());
    colKeys.insert(colKeys.end(), AngleErrorKeys.begin(), AngleErrorKeys.end());

    if (accelBiases)
    {
        rowKeys.insert(rowKeys.end(), Keys::AccelBias<KeyType>.begin(), Keys::AccelBias<KeyType>.end());
        colKeys.insert(colKeys.end(), Keys::AccelBias<KeyType>.begin(), Keys::AccelBias<KeyType>.end());
    }
    if (gyroBiases)
    {
        rowKeys.insert(rowKeys.end(), Keys::GyroBias<KeyType>.begin(), Keys::GyroBias<KeyType>.end());
        colKeys.insert(colKeys.end(), Keys::GyroBias<KeyType>.begin(), Keys::GyroBias<KeyType>.end());
    }
    if (accelScaleFactor)
    {
        rowKeys.insert(rowKeys.end(), Keys::AccelScaleFactor<KeyType>.begin(), Keys::AccelScaleFactor<KeyType>.end());
        colKeys.insert(colKeys.end(), Keys::AccelScaleFactor<KeyType>.begin(), Keys::AccelScaleFactor<KeyType>.end());
    }
    if (gyroScaleFactor)
    {
        rowKeys.insert(rowKeys.end(), Keys::GyroScaleFactor<KeyType>.begin(), Keys::GyroScaleFactor<KeyType>.end());
        colKeys.insert(colKeys.end(), Keys::GyroScaleFactor<KeyType>.begin(), Keys::GyroScaleFactor<KeyType>.end());
    }
    if (accelMisalignment) { rowKeys.insert(rowKeys.end(), Keys::AccelMisalignment<KeyType>.begin(), Keys::AccelMisalignment<KeyType>.end()); }
    if (gyroMisalignment) { rowKeys.insert(rowKeys.end(), Keys::GyroMisalignment<KeyType>.begin(), Keys::GyroMisalignment<KeyType>.end()); }

    KeyedMatrixX<T, KeyType, KeyType> G(Eigen::MatrixX<T>::Zero(static_cast<int>(rowKeys.size()), static_cast<int>(colKeys.size())), rowKeys, colKeys);

    Eigen::Quaternion<T> e_quat_p = e_quat_b * b_quat_p.cast<T>();
    G(Keys::Vel<KeyType>, Keys::Vel<KeyType>) = e_quat_p.toRotationMatrix();
    G(Keys::Att<KeyType>, AngleErrorKeys) = trafo::covRPY2quatJacobian(e_quat_p) * e_quat_p.toRotationMatrix();
    if (accelBiases) { G(Keys::AccelBias<KeyType>, Keys::AccelBias<KeyType>).setIdentity(); }
    if (gyroBiases) { G(Keys::GyroBias<KeyType>, Keys::GyroBias<KeyType>).setIdentity(); }
    if (accelScaleFactor) { G(Keys::AccelScaleFactor<KeyType>, Keys::AccelScaleFactor<KeyType>).setIdentity(); }
    if (gyroScaleFactor) { G(Keys::GyroScaleFactor<KeyType>, Keys::GyroScaleFactor<KeyType>).setIdentity(); }

    return G;
}

/// @brief Calculate the noise scale matrix in Earth frame coordinates
/// @param sigma2_ra 𝜎²_ra Variance of the noise on the accelerometer specific-force measurements [m^2 / s^4 / Hz]
/// @param sigma2_rg 𝜎²_rg Variance of the noise on the gyro angular-rate measurements [rad^2 / s^2 /Hz]
/// @param sigma2_bad 𝜎²_bad Variance of the accelerometer dynamic bias [m^2 / s^4]
/// @param sigma2_bgd 𝜎²_bgd Variance of the gyro dynamic bias [rad^2/s^2]
/// @param sigma2_sad 𝜎²_sad Variance of the accelerometer dynamic scale factor [-]
/// @param sigma2_sgd 𝜎²_sgd Variance of the gyro dynamic scale factor [-]
/// @param accelBiases Flag wether to calculate the rows and columns for the acceleration bias
/// @param gyroBiases Flag wether to calculate the rows and columns for the angular rate bias
/// @param accelScaleFactor Flag wether to calculate the rows and columns for the acceleration scale factor
/// @param gyroScaleFactor Flag wether to calculate the rows and columns for the angular rate scale factor
template<typename KeyType, typename T>
[[nodiscard]] KeyedMatrixX<T, KeyType, KeyType> e_noiseScale_W(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                               const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                               const Eigen::Vector3d& sigma2_sad, const Eigen::Vector3d& sigma2_sgd,
                                                               bool accelBiases, bool gyroBiases,
                                                               bool accelScaleFactor, bool gyroScaleFactor)

{
    std::vector<KeyType> keys;
    keys.reserve(3 + 3 + 3
                 + 3 * (static_cast<size_t>(accelBiases) + static_cast<size_t>(gyroBiases))
                 + 3 * (static_cast<size_t>(accelScaleFactor) + static_cast<size_t>(gyroScaleFactor)));

    constexpr std::array<KeyType, 3> AngleErrorKeys = { Keys::AttQ1, Keys::AttQ2, Keys::AttQ3 }; // 3 quaternion keys are used for the angle error
    keys.insert(keys.end(), Keys::PosVel<KeyType>.begin(), Keys::PosVel<KeyType>.end());
    keys.insert(keys.end(), AngleErrorKeys.begin(), AngleErrorKeys.end());

    if (accelBiases) { keys.insert(keys.end(), Keys::AccelBias<KeyType>.begin(), Keys::AccelBias<KeyType>.end()); }
    if (gyroBiases) { keys.insert(keys.end(), Keys::GyroBias<KeyType>.begin(), Keys::GyroBias<KeyType>.end()); }
    if (accelScaleFactor) { keys.insert(keys.end(), Keys::AccelScaleFactor<KeyType>.begin(), Keys::AccelScaleFactor<KeyType>.end()); }
    if (gyroScaleFactor) { keys.insert(keys.end(), Keys::GyroScaleFactor<KeyType>.begin(), Keys::GyroScaleFactor<KeyType>.end()); }

    KeyedMatrixX<T, KeyType, KeyType> W(Eigen::MatrixX<T>::Zero(static_cast<int>(keys.size()), static_cast<int>(keys.size())), keys, keys);

    W(Keys::Vel<KeyType>, Keys::Vel<KeyType>).diagonal() = sigma2_ra.cast<T>();
    W(AngleErrorKeys, AngleErrorKeys).diagonal() = sigma2_rg.cast<T>();
    if (accelBiases) { W(Keys::AccelBias<KeyType>, Keys::AccelBias<KeyType>).diagonal() = sigma2_bad.cast<T>(); }
    if (gyroBiases) { W(Keys::GyroBias<KeyType>, Keys::GyroBias<KeyType>).diagonal() = sigma2_bgd.cast<T>(); }
    if (accelScaleFactor) { W(Keys::AccelScaleFactor<KeyType>, Keys::AccelScaleFactor<KeyType>).diagonal() = sigma2_sad.cast<T>(); }
    if (gyroScaleFactor) { W(Keys::GyroScaleFactor<KeyType>, Keys::GyroScaleFactor<KeyType>).diagonal() = sigma2_sgd.cast<T>(); }

    return W;
}

} // namespace NAV