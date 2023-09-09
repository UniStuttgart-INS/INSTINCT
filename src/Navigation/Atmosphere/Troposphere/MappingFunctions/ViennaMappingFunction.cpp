// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ViennaMappingFunction.hpp"
#include <cmath>

namespace NAV
{
double vmf1h(const double& ah, const double& dmjd,
             const double& dlat, const double& ht, const double& zd)
{
    double a_ht = 0.0;
    double b_ht = 0.0;
    double beta = 0.0;
    double bh = 0.0;
    double c0h = 0.0;
    double c10h = 0.0;
    double c11h = 0.0;
    double c_ht = 0.0;
    double ch = 0.0;
    double doy = 0.0;
    double gamma = 0.0;
    double hs_km = 0.0;
    double ht_corr = 0.0;
    double ht_corr_coef = 0.0;
    double phh = 0.0;
    double sine = 0.0;
    double topcon = 0.0;

    double vmf1h = 0.0;

    doy = dmjd - 44239.0 + 1 - 28;

    bh = 0.0029;
    c0h = 0.062;
    if (dlat < 0.0) /* southern hemisphere*/
    {
        phh = M_PI;
        c11h = 0.007;
        c10h = 0.002;
    }
    else /* northern hemisphere*/
    {
        phh = 0.0;
        c11h = 0.005;
        c10h = 0.001;
    }
    ch = c0h + ((cos(doy / 365.25 * 2.0 * M_PI + phh) + 1.0) * c11h / 2.0 + c10h) * (1.0 - cos(dlat));

    sine = sin(M_PI / 2.0 - zd);
    beta = bh / (sine + ch);
    gamma = ah / (sine + beta);
    topcon = 1.0 + ah / (1.0 + bh / (1.0 + ch));
    vmf1h = topcon / (sine + gamma);

    //  height correction [Niell, 1996]
    a_ht = 2.53e-5;
    b_ht = 5.49e-3;
    c_ht = 1.14e-3;
    hs_km = ht / 1000.0;
    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = 1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht));
    ht_corr_coef = 1.0 / sine - topcon / (sine + gamma);
    ht_corr = ht_corr_coef * hs_km;
    vmf1h += ht_corr;

    return vmf1h;
}
double vmf1w(const double& aw, const double& zd)
{
    double vmf1w = 0.0;

    double sine = sin(M_PI / 2.0 - zd);
    double bw = 0.00146;
    double cw = 0.04391;
    double beta = bw / (sine + cw);
    double gamma = aw / (sine + beta);
    double topcon = 1.0 + aw / (1.0 + bw / (1.0 + cw));

    vmf1w = topcon / (sine + gamma);

    return vmf1w;
}

} // namespace NAV