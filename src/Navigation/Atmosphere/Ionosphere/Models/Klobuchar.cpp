// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Klobuchar.hpp"

#include <algorithm>
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Assert.h"
#include "util/Logger.hpp"

namespace NAV
{

double calcIonosphericTimeDelay_Klobuchar(double tow, Frequency freq, int8_t freqNum,
                                          double latitude, double longitude,
                                          double elevation, double azimuth,
                                          const std::array<double, 4>& alpha, const std::array<double, 4>& beta)
{
    double phi_u = rad2semicircles(latitude);     // User geodetic latitude [semi-circles] WGS 84
    double lambda_u = rad2semicircles(longitude); // User geodetic longitude [semi-circles] WGS 84
    double E = rad2semicircles(elevation);        // Angle between the user and satellite [semi-circles]

    // Earth's central angle between the user position and the earth projection of ionospheric intersection point [semi-circles]
    double psi = 0.0137 / (E + 0.11) - 0.022;
    LOG_DATA("psi {} [semi-circles] (Earth's central angle between the user position and the earth projection of ionospheric intersection point)", psi);

    // Sub-ionospheric latitude (geodetic latitude of the earth projection of the ionospheric intersection point) [semi-circles]
    // Also known as Ionospheric Pierce Point IPP
    double phi_I = std::clamp(phi_u + psi * std::cos(azimuth), -0.416, 0.416);
    LOG_DATA("phi_I {} [semi-circles] (Sub-ionospheric latitude)", phi_I);

    // Sub-ionospheric longitude (geodetic longitude of the earth projection of the ionospheric intersection point) [semi-circles]
    // Also known as Ionospheric Pierce Point IPP
    double lambda_I = lambda_u + (psi * std::sin(azimuth)) / std::cos(semicircles2rad(phi_I));
    LOG_DATA("lambda_I {} [semi-circles] (Sub-ionospheric longitude)", lambda_I);

    // Geomagnetic latitude of the earth projection of the ionospheric intersection point (mean ionospheric height assumed 350 km) [semi-circles]
    double phi_m = phi_I + 0.064 * std::cos(semicircles2rad(lambda_I - 1.617));
    LOG_DATA("phi_m {} [semi-circles] (Geomagnetic latitude)", phi_m);

    // Local time [s]
    double t = std::fmod(4.32e4 * lambda_I + tow, InsTimeUtil::SECONDS_PER_DAY);
    if (t < 0)
    {
        t += InsTimeUtil::SECONDS_PER_DAY;
    }
    LOG_DATA("t {} [s] (Local time)", t);

    // Slant factor / obliquity factor [-]
    double F = 1.0 + 16.0 * std::pow(0.53 - E, 3);
    LOG_DATA("F {} [-] (Slant factor / obliquity factor)", F);

    // Period of the model in [s]
    double PER = 0.0;
    for (size_t n = 0; n < beta.size(); ++n)
    {
        PER += beta.at(n) * std::pow(phi_m, n);
    }
    PER = std::max(PER, 72000.0);
    LOG_DATA("PER {} [s] (Period of the model)", PER);

    // Amplitude of the vertical delay in [s]
    double AMP = 0.0;
    for (size_t n = 0; n < beta.size(); ++n)
    {
        AMP += alpha.at(n) * std::pow(phi_m, n);
    }
    AMP = std::max(AMP, 0.0);
    LOG_DATA("AMP {} [s] (Amplitude of the vertical delay)", AMP);

    // Phase [rad]
    double x = (2 * M_PI * (t - 50400.0)) / PER;
    LOG_DATA("x {} [rad] (Phase)", x);

    // Ionospheric delay [s]
    double T_iono = std::abs(x) < 1.57 ? F * (5e-9 + AMP * (1.0 - std::pow(x, 2) / 2.0 + std::pow(x, 4) / 24.0))
                                       : F * 5e-9;
    LOG_DATA("T_iono_L1 {} [s] (Ionospheric delay)", T_iono);

    // T_iono is referred to the L1 frequency; if the user is operating on the L2 frequency, the correction term must be multiplied by γ
    T_iono *= ratioFreqSquared(G01, freq, 0, freqNum);
    LOG_DATA("T_iono    {} [s] (Ionospheric delay for requested frequency)", T_iono);

    return T_iono;
}

} // namespace NAV