// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ViennaMappingFunction.hpp"
#include <cmath>
#include <tuple>

namespace NAV
{
double vmf1h(const double& ah, const double& dmjd,
             const double& dlat, const double& ht, const double& zd)
{
    double doy = dmjd - 44239.0 + 1 - 28;

    double bh = 0.0029;
    double c0h = 0.062;

    auto [phh, c11h, c10h] = dlat < 0.0
                                 ? std::make_tuple(M_PI, 0.007, 0.002) // southern hemisphere
                                 : std::make_tuple(0.0, 0.005, 0.001); // northern hemisphere

    double ch = c0h + ((std::cos(doy / 365.25 * 2.0 * M_PI + phh) + 1.0) * c11h / 2.0 + c10h) * (1.0 - std::cos(dlat));

    double sine = std::sin(M_PI / 2.0 - zd);
    double beta = bh / (sine + ch);
    double gamma = ah / (sine + beta);
    double topcon = 1.0 + ah / (1.0 + bh / (1.0 + ch));
    double vmf1h = topcon / (sine + gamma);

    //  height correction [Niell, 1996]
    double a_ht = 2.53e-5;
    double b_ht = 5.49e-3;
    double c_ht = 1.14e-3;
    double hs_km = ht / 1000.0;
    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = 1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht));
    double ht_corr_coef = 1.0 / sine - topcon / (sine + gamma);
    double ht_corr = ht_corr_coef * hs_km;
    vmf1h += ht_corr;

    return vmf1h;
}
double vmf1w(const double& aw, const double& zd)
{
    double sine = std::sin(M_PI / 2.0 - zd);
    double bw = 0.00146;
    double cw = 0.04391;
    double beta = bw / (sine + cw);
    double gamma = aw / (sine + beta);
    double topcon = 1.0 + aw / (1.0 + bw / (1.0 + cw));

    return topcon / (sine + gamma);
}

} // namespace NAV