// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Klobuchar.hpp
/// @brief Klobuchar Ionospheric correction model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

#include <array>
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{

/// @brief Calculates the ionospheric time delay with the Klobuchar model
/// @param[in] tow GPS time of week in [s]
/// @param[in] freq Frequency of the signal
/// @param[in] freqNum Frequency number. Only used for GLONASS G1 and G2
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @param[in] elevation Angle between the user and satellite [rad]
/// @param[in] azimuth Angle between the user and satellite, measured clockwise positive from the true North [rad]
/// @param[in] alpha The coefficients of a cubic equation representing the amplitude of the vertical delay
/// @param[in] beta The coefficients of a cubic equation representing the period of the model
/// @return Ionospheric time delay in [s]
/// @note See \cite Klobuchar1987 Klobuchar p. 329
/// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.5.2.5 Figure 20-4 p.131-134
/// @anchor Ionosphere-Model-Klobuchar
///
/// #### Algorithm description
/// Given the approximate position \f$ \phi, \lambda \f$, the elevation angle \f$ \varepsilon \f$, the azimuth \f$ \alpha \f$ as well as the 8 Klobuchar coefficients \f$ \alpha_i, \beta_i, i = 0,...,3 \f$ one can compute the following:
///
/// - earth-centred angle (elevation in semicircles)
///     \anchor eq-klobuchar-earthCentredAngle \f{equation}{ \label{eq:eq-klobuchar-earthCentredAngle}
///     \psi = \frac{0.0137}{\varepsilon + 0.11} - 0.022
///     \f}
///
/// - latitude of the Ionospheric Pierce Point IPP (semicircles)
///     \anchor eq-klobuchar-latIPP \f{equation}{ \label{eq:eq-klobuchar-latIPP}
///     \phi_\mathrm{IPP} = \phi + \psi \cos(\alpha)
///     \f}
///     If \f$ \phi_\mathrm{IPP} > 0.416 \f$, then \f$ \phi_\mathrm{IPP} = 0.416 \f$ and if \f$ \phi_\mathrm{IPP} < -0.416 \f$, then \f$ \phi_\mathrm{IPP} = -0.416 \f$ .
///
/// - longitude of the Ionospheric Pierce Point IPP (semicircles)
///     \anchor eq-klobuchar-longIPP \f{equation}{ \label{eq:eq-klobuchar-longIPP}
///     \lambda_\mathrm{IPP} = \lambda + \frac{\psi \sin(\alpha)}{\cos(\phi_\mathrm{IPP})}
///     \f}
///
/// - geomagnetic latitude of IPP
///     \anchor eq-klobuchar-latIPP-geomag \f{equation}{ \label{eq:eq-klobuchar-latIPP-geomag}
///     \phi_\mathrm{mag} = \phi_\mathrm{IPP} + 0.064 \cos(\lambda_\mathrm{IPP}-1.617)
///     \f}
///
/// - local time at IPP in [s]
///     \anchor eq-klobuchar-localTimeIPP \f{equation}{ \label{eq:eq-klobuchar-localTimeIPP}
///     t = 43200 \lambda_\mathrm{IPP} + t_\mathrm{GPS}
///     \f}
///     where the local time at IPP is \f$ 0 \leq t < 86400 \f$, so if \f$ t \geq 86400 \f$, \f$ t = t - 86400 \f$ and if \f$ t < 0 \f$,  \f$ t = t + 86400 \f$.
///
/// - slant factor (elevation in semicircles)
///     \anchor eq-klobuchar-slantFactor \f{equation}{ \label{eq:eq-klobuchar-slantFactor}
///     F = 1.0 + 16.0 (0.53-\varepsilon)^{3}
///     \f}
///
/// - amplitude of ionospheric delay in [s]
///     \anchor eq-klobuchar-amp \f{equation}{ \label{eq:eq-klobuchar-amp}
///     A_\mathrm{I}=\sum_{n=0}^{3} \alpha_{n} \phi_{mag}^{n}
///     \f}
///
/// - period of ionospheric delay in [s]
///     \anchor eq-klobuchar-period \f{equation}{ \label{eq:eq-klobuchar-period}
///     P_\mathrm{I}=\sum_{n=0}^{3} \beta_{n} \phi_{mag}^{n}
///     \f}
///     If \f$ P_\mathrm{I} < 72000 \f$, set it to \f$ P_\mathrm{I} = 72000 \f$.
///
/// - phase of ionospheric delay in [rad]
///     \anchor eq-klobuchar-phase \f{equation}{ \label{eq:eq-klobuchar-phase}
///     X_\mathrm{I} = \frac{2\pi (t-50400)}{P_\mathrm{I}}
///     \f}
///
/// - ionospheric time delay in [s]
///     \anchor eq-klobuchar-ionoTimeDelay \f{equation}{ \label{eq:eq-klobuchar-ionoTimeDelay}
///     I_{L1_{GPS}}= \begin{cases}{
///     \left[5 \cdot 10^{-9}+ A_\mathrm{I} \cdot\left(1-\frac{X_{I}^{2}}{2}+\frac{X_{I}^{4}}{24}\right)\right] \cdot F} & , \text{if} \left|X_{I}\right| < 1.57 \\
///     5 \cdot 10^{-9} \cdot F & , \text{if} \left|X_{I}\right| \geq 1.57
///     \end{cases}
///     \f}
///
/// - The ionospheric time delay \f$ I_{L1_{GPS}} \f$ refers to the GPS L1 frequency and gets adapted to the given frequency automatically.
double calcIonosphericTimeDelay_Klobuchar(double tow, Frequency freq, int8_t freqNum,
                                          double latitude, double longitude,
                                          double elevation, double azimuth,
                                          const std::array<double, 4>& alpha, const std::array<double, 4>& beta);

} // namespace NAV
