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

#include <cmath>
#include <Eigen/Core>
#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{

/// @brief Calculates the line-of-sight unit vector from the user antenna to the satellite
/// @param[in] e_posAnt Position of the user antenna in ECEF frame coordinates
/// @param[in] e_posSat Position of the satellite in ECEF frame coordinates
/// @return The line-of-sight unit vector in ECEF frame coordinates
///
/// @note See \cite Groves2013 Groves, ch. 8.5.3, eq. 8.41, p. 341
template<typename DerivedA, typename DerivedB>
[[nodiscard]] Eigen::Vector3<typename DerivedA::Scalar> e_calcLineOfSightUnitVector(const Eigen::MatrixBase<DerivedA>& e_posAnt,
                                                                                    const Eigen::MatrixBase<DerivedB>& e_posSat)
{
    return (e_posSat - e_posAnt) / (e_posSat - e_posAnt).norm();
}

/// @brief Calculates the elevation of the satellite from the antenna
/// @param[in] n_lineOfSightUnitVector Line-of-sight unit vector from the antenna to the satellite in NED frame coordinates
/// @return Elevation [rad]
///
/// @note See \cite Groves2013 Groves, ch. 8.5.4, eq. 8.57, p. 344
template<typename Derived>
[[nodiscard]] typename Derived::Scalar calcSatElevation(const Eigen::MatrixBase<Derived>& n_lineOfSightUnitVector)
{
    return -asin(n_lineOfSightUnitVector(2));
}

/// @brief Calculates the azimuth of the satellite from the antenna
/// @param[in] n_lineOfSightUnitVector Line-of-sight unit vector from the antenna to the satellite in NED frame coordinates
/// @return Azimuth [rad]
///
/// @note See \cite Groves2013 Groves, ch. 8.5.4, eq. 8.57, p. 344
template<typename Derived>
[[nodiscard]] typename Derived::Scalar calcSatAzimuth(const Eigen::MatrixBase<Derived>& n_lineOfSightUnitVector)
{
    return atan2(n_lineOfSightUnitVector(1), n_lineOfSightUnitVector(0));
}

/// @brief Calculates the Earth rotation/Sagnac correction
/// @param[in] e_posAnt Position of the user antenna in ECEF frame coordinates
/// @param[in] e_satPos Position of the satellite in ECEF frame coordinates
/// @return Earth rotation/Sagnac correction [m]
///
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
template<typename DerivedA, typename DerivedB>
[[nodiscard]] typename DerivedA::Scalar calcSagnacCorrection(const Eigen::MatrixBase<DerivedA>& e_posAnt,
                                                             const Eigen::MatrixBase<DerivedB>& e_satPos)
{
    return 1.0 / InsConst::C * (e_posAnt - e_satPos).dot(InsConst::e_omega_ie.cross(e_posAnt));
}

/// @brief Calculates the Range-rate Earth rotation/Sagnac correction
/// @param[in] e_recvPos Position of the user antenna in ECEF frame coordinates
/// @param[in] e_satPos Position of the satellite in ECEF frame coordinates
/// @param[in] e_recvVel Velocity of the user antenna in ECEF frame coordinates
/// @param[in] e_satVel Velocity of the satellite in ECEF frame coordinates
/// @return Range-rate Earth rotation/Sagnac correction [m/s]
///
/// @note See \cite Groves2013 Groves ch. 8.5.3, eq. 8.46, p. 342
template<typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
[[nodiscard]] typename DerivedA::Scalar calcSagnacRateCorrection(const Eigen::MatrixBase<DerivedA>& e_recvPos,
                                                                 const Eigen::MatrixBase<DerivedB>& e_satPos,
                                                                 const Eigen::MatrixBase<DerivedC>& e_recvVel,
                                                                 const Eigen::MatrixBase<DerivedD>& e_satVel)
{
    return InsConst::omega_ie / InsConst::C
           * (e_satVel.y() * e_recvPos.x() + e_satPos.y() * e_recvVel.x()
              - e_satVel.x() * e_recvPos.y() - e_satPos.x() * e_recvVel.y());
}

/// @brief Transforms a doppler-shift into a range-rate
/// @param[in] doppler The doppler-shift to transform [Hz]
/// @param[in] freq Frequency
/// @param[in] num Frequency number. Only used for GLONASS G1 and G2
/// @return The corresponding range-rate [m/s]
///
/// @note See \cite Groves2013 Groves, ch. 9.2.7, eq. 9.70, p. 388
[[nodiscard]] double doppler2rangeRate(double doppler, Frequency freq, int8_t num);

/// @brief Transforms a range-rate into a doppler-shift
/// @param[in] rangeRate The range-rate to transform [m/s]
/// @param[in] freq Frequency
/// @param[in] num Frequency number. Only used for GLONASS G1 and G2
/// @return The corresponding doppler-shift [Hz]
///
/// @note See \cite Groves2013 Groves, ch. 9.2.7, eq. 9.70, p. 388
[[nodiscard]] double rangeRate2doppler(double rangeRate, Frequency freq, int8_t num);

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
[[nodiscard]] double ratioFreqSquared(Frequency f1, Frequency f2, int8_t num1, int8_t num2);

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
