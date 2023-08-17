// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Functions.hpp
/// @brief GNSS helper functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-19

#pragma once

#include <Eigen/Core>
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/Time/InsTime.hpp"

namespace NAV
{

/// @brief Calculates the line-of-sight unit vector from the user antenna to the satellite
/// @param[in] e_posAnt Position of the user antenna in ECEF frame coordinates
/// @param[in] e_posSat Position of the satellite in ECEF frame coordinates
/// @return The line-of-sight unit vector in ECEF frame coordinates
///
/// @note See \cite Groves2013 Groves, ch. 8.5.3, eq. 8.41, p. 341
[[nodiscard]] Eigen::Vector3d e_calcLineOfSightUnitVector(const Eigen::Vector3d& e_posAnt, const Eigen::Vector3d& e_posSat);

/// @brief Calculates the elevation of the satellite from the antenna
/// @param[in] n_lineOfSightUnitVector Line-of-sight unit vector from the antenna to the satellite in NED frame coordinates
/// @return Elevation [rad]
///
/// @note See \cite Groves2013 Groves, ch. 8.5.4, eq. 8.57, p. 344
[[nodiscard]] double calcSatElevation(const Eigen::Vector3d& n_lineOfSightUnitVector);

/// @brief Calculates the azimuth of the satellite from the antenna
/// @param[in] n_lineOfSightUnitVector Line-of-sight unit vector from the antenna to the satellite in NED frame coordinates
/// @return Azimuth [rad]
///
/// @note See \cite Groves2013 Groves, ch. 8.5.4, eq. 8.57, p. 344
[[nodiscard]] double calcSatAzimuth(const Eigen::Vector3d& n_lineOfSightUnitVector);

/// @brief Calculates the Earth rotation/Sagnac correction
/// @param[in] e_posAnt Position of the user antenna in ECEF frame coordinates
/// @param[in] e_satPos Position of the satellite in ECEF frame coordinates
/// @return Earth rotation/Sagnac correction [m]
///
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
[[nodiscard]] double calcSagnacCorrection(const Eigen::Vector3d& e_posAnt, const Eigen::Vector3d& e_satPos);

/// @brief Calculates the Range-rate Earth rotation/Sagnac correction
/// @param[in] e_posAnt Position of the user antenna in ECEF frame coordinates
/// @param[in] e_satPos Position of the satellite in ECEF frame coordinates
/// @param[in] e_velAnt Velocity of the user antenna in ECEF frame coordinates
/// @param[in] e_satVel Velocity of the satellite in ECEF frame coordinates
/// @return Range-rate Earth rotation/Sagnac correction [m/s]
///
/// @note See \cite Groves2013 Groves ch. 8.5.3, eq. 8.46, p. 342
[[nodiscard]] double calcSagnacRateCorrection(const Eigen::Vector3d& e_posAnt, const Eigen::Vector3d& e_satPos, const Eigen::Vector3d& e_velAnt, const Eigen::Vector3d& e_satVel);

/// @brief Transforms a doppler-shift measurement into a pseudorange-rate measurement
/// @param[in] doppler The doppler-shift measurement to transform [Hz]
/// @param[in] freq Frequency the measurement originates from
/// @param[in] num  Frequency number. Only used for GLONASS G1 and G2
/// @return The corresponding pseudorange-rate measurement [s/s]
///
/// @note See \cite Groves2013 Groves, ch. 9.2.7, eq. 9.70, p. 388
[[nodiscard]] double doppler2psrRate(double doppler, Frequency freq, int8_t num = -128);

/// @brief Calculates the ration of the frequencies squared γ
///
/// \anchor eq-GNSS-freqRatio \f{equation}{ \label{eq:eq-GNSS-freqRatio}
///   \gamma = (f_1 / f_2)^2
/// \f}
///
/// @param[in] f1 First frequency (usually L1)
/// @param[in] f2 Second frequency (usually L2)
/// @param[in] num1 First frequency number. Only used for GLONASS G1 and G2
/// @param[in] num2 Second frequency number. Only used for GLONASS G1 and G2
/// @return The ratio (f1/f2)^2 [-]
/// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.3.3.2 p.99
[[nodiscard]] double ratioFreqSquared(Frequency f1, Frequency f2, int8_t num1 = -128, int8_t num2 = -128);

/// @brief Converts a GALILEO SISA (Signal in space accuracy) value to it's index
/// @param[in] val SISA value in [m]
/// @return The SISA index
///
/// @note See \cite GAL-ICD-2.0 GAL ICD ch. 5.1.12, p.58
[[nodiscard]] uint8_t galSisaVal2Idx(double val);

/// @brief Converts a GALILEO SISA (Signal in space accuracy) index to it's value
/// @param[in] idx The SISA index
/// @return SISA value in [m]
///
/// @note See \cite GAL-ICD-2.0 GAL ICD ch. 5.1.12, p.58
[[nodiscard]] double galSisaIdx2Val(uint8_t idx);

/// @brief Converts a GPS URA (user range accuracy) value to it's index
/// @param[in] val URA value in [m]
/// @return The URA index
///
/// @note See \cite IS-GPS-200M GPS ICD ch. 20.3.3.3.1.3, p.92ff
[[nodiscard]] uint8_t gpsUraVal2Idx(double val);

/// @brief Converts a GPS URA (user range accuracy) index to it's value
/// @param[in] idx The URA index
/// @return URA value in [m]
///
/// @note See \cite IS-GPS-200M GPS ICD ch. 20.3.3.3.1.3, p.92ff
[[nodiscard]] double gpsUraIdx2Val(uint8_t idx);

} // namespace NAV
