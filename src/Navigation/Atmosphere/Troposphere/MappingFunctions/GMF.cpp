// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GMF.cpp
/// @brief Global Mapping Function (GMF)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-04-21
/// @note See \cite Böhm2006a Böhm2006: Global Mapping Function (GMF): A new empirical mapping function based on numerical weather model data
/// @note See https://vmf.geo.tuwien.ac.at/codes/ for code sources in matlab.

#include "GMF.hpp"
#include "internal/GMFCoeffs.hpp"

namespace NAV::internal::GMF
{

namespace
{

// degree n and order m
constexpr int nmax = 9;

std::array<Eigen::Matrix<double, nmax + 1, nmax + 1>, 2> calcLegendrePolynomials(const Eigen::Vector3d& lla_pos)
{
    // unit vector
    double x = std::cos(lla_pos(0)) * std::cos(lla_pos(1));
    double y = std::cos(lla_pos(0)) * std::sin(lla_pos(1));
    double z = std::sin(lla_pos(0));

    // Legendre polynomials
    Eigen::Matrix<double, nmax + 1, nmax + 1> V;
    Eigen::Matrix<double, nmax + 1, nmax + 1> W;

    V(0, 0) = 1;
    W(0, 0) = 0;
    V(1, 0) = z * V(0, 0);
    W(1, 0) = 0;

    for (int n = 2; n <= nmax; n++)
    {
        auto dn = static_cast<double>(n);
        V(n, 0) = ((2 * dn - 1) * z * V(n - 1, 0) - (dn - 1) * V(n - 2, 0)) / dn;
        W(n, 0) = 0;
    }
    for (int m = 1; m <= nmax; m++)
    {
        auto dm = static_cast<double>(m);
        V(m, m) = (2 * dm - 1) * (x * V(m - 1, m - 1) - y * W(m - 1, m - 1));
        W(m, m) = (2 * dm - 1) * (x * W(m - 1, m - 1) + y * V(m - 1, m - 1));
        if (m < nmax)
        {
            V(m + 1, m) = (2 * dm + 1) * z * V(m, m);
            W(m + 1, m) = (2 * dm + 1) * z * W(m, m);
        }
        for (int n = m + 2; n <= nmax; n++)
        {
            auto dn = static_cast<double>(n);
            V(n, m) = ((2 * dn - 1) * z * V(n - 1, m) - (dn + dm - 1) * V(n - 2, m)) / (dn - dm);
            W(n, m) = ((2 * dn - 1) * z * W(n - 1, m) - (dn + dm - 1) * W(n - 2, m)) / (dn - dm);
        }
    }

    return { V, W };
}

} // namespace

} // namespace NAV::internal::GMF

double NAV::calcTropoMapFunc_GMFH(double mjd, const Eigen::Vector3d& lla_pos, double elevation)
{
    using namespace internal::GMF; // NOLINT(google-build-using-namespace)

    // reference day is 28 January
    // this is taken from Niell (1996) to be consistent
    double doy = mjd - 44239.0 + 1 - 28;

    auto [V, W] = calcLegendrePolynomials(lla_pos);

    double bh = 0.0029;
    double c0h = 0.062;
    double phh = 0.0;
    double c11h = 0.0;
    double c10h = 0.0;
    if (lla_pos(0) < 0) // southern hemisphere
    {
        phh = M_PI;
        c11h = 0.007;
        c10h = 0.002;
    }
    else // northern hemisphere
    {
        phh = 0;
        c11h = 0.005;
        c10h = 0.001;
    }
    double ch = c0h + ((std::cos(doy / 365.25 * 2 * M_PI + phh) + 1) * c11h / 2 + c10h) * (1 - std::cos(lla_pos(0)));

    double ahm = 0;
    double aha = 0;
    size_t i = 0;
    for (int n = 0; n <= nmax; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            ahm = ahm + (ah_mean.at(i) * V(n, m) + bh_mean.at(i) * W(n, m));
            aha = aha + (ah_amp.at(i) * V(n, m) + bh_amp.at(i) * W(n, m));
            i = i + 1;
        }
    }
    double ah = (ahm + aha * std::cos(doy / 365.25 * 2 * M_PI)) * 1e-5;

    double sine = std::sin(elevation);
    double beta = bh / (sine + ch);
    double gamma = ah / (sine + beta);
    double topcon = (1 + ah / (1 + bh / (1 + ch)));
    double gmfh = topcon / (sine + gamma);

    // height correction for hydrostatic mapping function from Niell (1996) in order to reduce the coefficients to sea level
    double a_ht = 2.53e-5;
    double b_ht = 5.49e-3;
    double c_ht = 1.14e-3;
    double hs_km = lla_pos(2) / 1000;

    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = (1 + a_ht / (1 + b_ht / (1 + c_ht)));
    double ht_corr_coef = 1 / sine - topcon / (sine + gamma);
    double ht_corr = ht_corr_coef * hs_km;
    gmfh += ht_corr;

    return gmfh;
}

double NAV::calcTropoMapFunc_GMFW(double mjd, const Eigen::Vector3d& lla_pos, double elevation)
{
    using namespace internal::GMF; // NOLINT(google-build-using-namespace)

    // reference day is 28 January
    // this is taken from Niell (1996) to be consistent
    double doy = mjd - 44239.0 + 1 - 28;

    auto [V, W] = calcLegendrePolynomials(lla_pos);

    double bw = 0.00146;
    double cw = 0.04391;

    double awm = 0.0;
    double awa = 0.0;
    size_t i = 0;
    for (int n = 0; n <= nmax; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            awm = awm + (aw_mean.at(i) * V(n, m) + bw_mean.at(i) * W(n, m));
            awa = awa + (aw_amp.at(i) * V(n, m) + bw_amp.at(i) * W(n, m));
            i = i + 1;
        }
    }
    double aw = (awm + awa * std::cos(doy / 365.25 * 2 * M_PI)) * 1e-5;

    double sine = std::sin(elevation);
    double beta = bw / (sine + cw);
    double gamma = aw / (sine + beta);
    double topcon = (1 + aw / (1 + bw / (1 + cw)));
    double gmfw = topcon / (sine + gamma);

    return gmfw;
}