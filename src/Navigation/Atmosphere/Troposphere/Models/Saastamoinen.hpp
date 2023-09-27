// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Saastamoinen.hpp
/// @brief Saastamoinen troposphere correction model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26
/// @anchor Troposphere-Model-Saastamoinen
///
/// #### Algorithm description
/// Given the approximate position \f$ \phi \f$, \f$ \lambda \f$, \f$ h \f$, the elevation \f$ \varepsilon \f$ and the relative humidity \f$ h_{\mathrm{rel}} \f$ the tropospheric delay can be calculated using Saastamoinen's formulas.
///
/// A standard atmosphere is applied as a reference for the necessary parameters of the atmosphere.
///
/// - total pressure in [hPa]
///     \anchor eq-Saastamoinen-pressure \f{equation}{ \label{eq:eq-Saastamoinen-pressure}
///         p = 1013.25 \cdot \left(1-2.2557 \cdot 10^{-5} \cdot h \right)^{5.2568}
///     \f}
///
/// - absolute temperature in [K]
///     \anchor eq-Saastamoinen-temperature \f{equation}{ \label{eq:eq-Saastamoinen-temperature}
///         T = 15.0 - 6.5 \cdot 10^{-3} \cdot h + 273.16
///     \f}
///
///     where
///     \f$ T_{0} = 15^{\circ} \f$ temperature at sea level
///
/// - partial pressure of water vapor in [hPa]
///     \anchor eq-Saastamoinen-partial_pressure_water_vapor \f{equation}{ \label{eq:eq-Saastamoinen-partial_pressure_water_vapor}
///         e = 6.108 \cdot \exp \left\{ \frac{17.15 T-4684.0}{T-38.45} \right\} \cdot \frac{h_{\mathrm{rel}}}{100}
///     \f}
///
///     where
///     \f$ h_{\mathrm{rel}} = 0.7 \f$ is the relative humidity
///
/// - zenith hydrostatic delay in [m] (see \cite Davis1985 Davis, Appendix A, eq. A11, p. 1604 or \cite Böhm2006 Böhm ch. 1, eq. 3, p. 1)
///     \anchor eq-Saastamoinen-zhd \f{equation}{ \label{eq:eq-Saastamoinen-zhd}
///         \Delta L_{\mathrm{h}}^{\mathrm{z}} = \dfrac{0.0022768 \cdot p}{1-0.00266 \cdot \cos 2 \varphi-0.00028 \cdot \dfrac{ h }{1000}}
///     \f}
///
/// - zenith wet delay (see \cite SpringerHandbookGNSS2017 Springer Handbook GNSS ch. 6, eq. 6.54, p. 173) in [m]
///     \anchor eq-Saastamoinen-zwd \f{equation}{ \label{eq:eq-Saastamoinen-zwd}
///         \Delta L_{\mathrm{w}}^{\mathrm{z}} = 0.002277 \cdot \left(\frac{1255.0}{T} + 0.05 \right) \cdot e
///     \f}
///
/// - zenith total delay in [m]
///     \anchor eq-Saastamoinen-ztd \f{equation}{ \label{eq:eq-Saastamoinen-ztd}
///         \Delta L^{\mathrm{z}} = \Delta L_{\mathrm{h}}^{\mathrm{z}} + \Delta L_{\mathrm{w}}^{\mathrm{z}}
///     \f}
///
/// - slant total delay along elevation angle in [m]
///     \anchor eq-Saastamoinen-std \f{equation}{ \label{eq:eq-Saastamoinen-std}
///         \Delta L_{\mathrm{troposphere}} = \frac{\Delta L^{\mathrm{z}}}{\cos{ \left( z d \right) }}
///     \f}
///
///     where \f$ zd \f$ is the zenith angle as \f$ \varepsilon=\frac{\pi}{2} - z d \f$ in [rad]

#pragma once

#include <Eigen/Core>

namespace NAV
{

/// @brief Calculates the tropospheric zenith hydrostatic delay with the Saastamoinen model
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] p Total barometric pressure in [millibar]
/// @return Range correction for troposphere and stratosphere for radio ranging in [m]
/// @note See \cite Saastamoinen1973 Saastamoinen, p. 32
/// @note See \cite Davis1985 Davis, Appendix A, p. 1604
/// @note See \cite Böhm2006 Böhm ch. 1, p. 1
/// @note See \cite RTKLIB RTKLIB ch. E.5 Troposphere and Ionosphere Models, sec. (1), p. 149
double calcZHD_Saastamoinen(const Eigen::Vector3d& lla_pos, double p);

/// @brief Calculates the tropospheric zenith wet delay with the Saastamoinen model
/// @param[in] T Absolute temperature in [K]
/// @param[in] e Partial pressure of water vapour in [millibar]
/// @return Range correction for troposphere and stratosphere for radio ranging in [m]
/// @note See \cite Saastamoinen1973 Saastamoinen, p. 32
/// @note See \cite Davis1985 Davis, Appendix A, p. 1604
/// @note See \cite Böhm2006 Böhm ch. 1, p. 1
/// @note See \cite RTKLIB RTKLIB ch. E.5 Troposphere and Ionosphere Models, sec. (1), p. 149
double calcZWD_Saastamoinen(double T, double e);

} // namespace NAV
