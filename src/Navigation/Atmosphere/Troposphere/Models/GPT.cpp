// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GPT.hpp"

#include <array>
#include <cmath>

#include "internal/GPT2Coeffs.hpp"
#include "internal/GPT3Coeffs.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV
{
GPT2output GPT2_param(const double& mjd, const Eigen::Vector3d& lla_pos)
{
    using internal::GPT2_grid;

    GPT2output gpt2outputs;

    // change the reference epoch to January 1 2000
    double mjd1 = mjd - 51544.5;
    // GPT2 with time variation
    double cosfy = cos(mjd1 / 365.25 * 2.0 * M_PI);
    double coshy = cos(mjd1 / 365.25 * 4.0 * M_PI);
    double sinfy = sin(mjd1 / 365.25 * 2.0 * M_PI);
    double sinhy = sin(mjd1 / 365.25 * 4.0 * M_PI);

    double dlat = lla_pos(0);
    double dlon = lla_pos(1);
    double hell = lla_pos(2);

    // only positive longitude in degrees
    double plon = dlon < 0.0 ? (dlon + 2.0 * M_PI) * 180.0 / M_PI : dlon * 180.0 / M_PI;

    // transform to polar distance in degrees
    double ppod = (-dlat + M_PI / 2.0) * 180.0 / M_PI;

    // find the index (line in the grid file) of the nearest point
    auto ipod = static_cast<int>(floor(ppod + 1.0));
    auto ilon = static_cast<int>(floor(plon + 1.0));

    // normalized (to one) differences, can be positive or negative
    double diffpod = ppod - (ipod - 0.5);
    double difflon = plon - (ilon - 0.5);

    // added by HCY
    if (ipod == 181) { ipod = 180; }
    // added by GP
    if (ilon == 361) { ilon = 1; }
    if (ilon == 0) { ilon = 360; }

    constexpr size_t N = 4;
    std::array<size_t, N> indx{};
    // get the number of the corresponding line
    indx[0] = static_cast<size_t>((ipod - 1) * 360 + ilon) - 1;

    // near the poles: nearest neighbour interpolation
    // otherwise: bilinear, with the 1 degree grid the limits are lower and upper (GP)
    if (ppod <= 0.5 || ppod >= 179.5) // case of nearest neighborhood
    {
        auto ix = indx[0];
        // transforming ellipsoidal height to orthometric height
        gpt2outputs.undu = GPT2_grid.at(ix).undulation; // undu: geoid undulation in m
        double hgt = hell - gpt2outputs.undu;

        // pressure, temperature at the height of the grid
        double T0 = GPT2_grid.at(ix).T_A0 + GPT2_grid.at(ix).T_A1 * cosfy + GPT2_grid.at(ix).T_B1 * sinfy + GPT2_grid.at(ix).T_A2 * coshy + GPT2_grid.at(ix).T_B2 * sinhy;
        double p0 = GPT2_grid.at(ix).p_A0 + GPT2_grid.at(ix).p_A1 * cosfy + GPT2_grid.at(ix).p_B1 * sinfy + GPT2_grid.at(ix).p_A2 * coshy + GPT2_grid.at(ix).p_B2 * sinhy;

        // specific humidity
        double Q = (GPT2_grid.at(ix).Q_A0 + GPT2_grid.at(ix).Q_A1 * cosfy + GPT2_grid.at(ix).Q_B1 * sinfy + GPT2_grid.at(ix).Q_A2 * coshy + GPT2_grid.at(ix).Q_B2 * sinhy)
                   / 1e3;

        // lapse rate of the temperature
        gpt2outputs.dT = (GPT2_grid.at(ix).dT_A0 + GPT2_grid.at(ix).dT_A1 * cosfy + GPT2_grid.at(ix).dT_B1 * sinfy + GPT2_grid.at(ix).dT_A2 * coshy + GPT2_grid.at(ix).dT_B2 * sinhy)
                         / 1e3;

        // station height - grid height
        double redh = hgt - GPT2_grid.at(ix).Hs; // Hs: orthometric grid height in m

        // temperature at station height in Celsius
        gpt2outputs.T = T0 + gpt2outputs.dT * redh - 273.150;

        // temperature lapse rate in degrees / km
        gpt2outputs.dT = gpt2outputs.dT * 1e3;

        // virtual temperature in Kelvin
        double Tv = T0 * (1 + 0.6077 * Q);

        double c = InsConst<>::G_NORM * InsConst<>::dMtr / (InsConst<>::Rg * Tv);

        // pressure in hPa
        gpt2outputs.p = (p0 * exp(-c * redh)) / 100.0;

        // hydrostatic coefficient ah
        gpt2outputs.ah = (GPT2_grid.at(ix).h_A0 + GPT2_grid.at(ix).h_A1 * cosfy + GPT2_grid.at(ix).h_B1 * sinfy + GPT2_grid.at(ix).h_A2 * coshy + GPT2_grid.at(ix).h_B2 * sinhy)
                         / 1e3;

        // wet coefficient aw
        gpt2outputs.aw = (GPT2_grid.at(ix).w_A0 + GPT2_grid.at(ix).w_A1 * cosfy + GPT2_grid.at(ix).w_B1 * sinfy + GPT2_grid.at(ix).w_A2 * coshy + GPT2_grid.at(ix).w_B2 * sinhy)
                         / 1e3;

        // water vapour decrease factor la - added by GP
        gpt2outputs.la = GPT2_grid.at(ix).la_A0 + GPT2_grid.at(ix).la_A1 * cosfy + GPT2_grid.at(ix).la_B1 * sinfy + GPT2_grid.at(ix).la_A2 * coshy + GPT2_grid.at(ix).la_B2 * sinhy;

        // mean temperature of the water vapor Tm - added by GP
        gpt2outputs.Tm = GPT2_grid.at(ix).Tm_A0 + GPT2_grid.at(ix).Tm_A1 * cosfy + GPT2_grid.at(ix).Tm_B1 * sinfy + GPT2_grid.at(ix).Tm_A2 * coshy + GPT2_grid.at(ix).Tm_B2 * sinhy;

        // water vapor pressure in hPa - changed by GP
        double e0 = Q * p0 / (0.622 + 0.378 * Q) / 100.0; // on the grid

        gpt2outputs.e = e0 * pow((100.0 * gpt2outputs.p / p0), (gpt2outputs.la + 1)); // on the station height - (14) Askne and Nordius, 1987
    }
    else // bilinear interpolation
    {
        double ipod1 = ipod + int(math::sign(1.0, diffpod));
        double ilon1 = ilon + int(math::sign(1.0, difflon));

        // changed for the 1 degree grid (GP)
        if (ilon1 == 361) { ilon1 = 1; }
        if (ilon1 == 0) { ilon1 = 360; }
        // get the number of the line
        indx[1] = static_cast<size_t>((ipod1 - 1) * 360 + ilon) - 1;  // along same longitude
        indx[2] = static_cast<size_t>((ipod - 1) * 360 + ilon1) - 1;  // along same polar distance
        indx[3] = static_cast<size_t>((ipod1 - 1) * 360 + ilon1) - 1; // diagonal

        std::array<double, N> undul{};
        std::array<double, N> Ql{};
        std::array<double, N> dTl{};
        std::array<double, N> Tl{};
        std::array<double, N> pl{};
        std::array<double, N> ahl{};
        std::array<double, N> awl{};
        std::array<double, N> lal{};
        std::array<double, N> Tml{};
        std::array<double, N> el{};

        for (size_t l = 0; l < N; l++)
        {
            // transforming ellipsoidal height to orthometric height:
            // Hortho = -N + Hell
            auto ix = indx.at(l);

            undul.at(l) = GPT2_grid.at(ix).undulation; // undu: geoid undulation in m
            double hgt = hell - undul.at(l);

            // pressure, temperature at the height of the grid
            double T0 = GPT2_grid.at(ix).T_A0 + GPT2_grid.at(ix).T_A1 * cosfy + GPT2_grid.at(ix).T_B1 * sinfy + GPT2_grid.at(ix).T_A2 * coshy + GPT2_grid.at(ix).T_B2 * sinhy;
            double p0 = GPT2_grid.at(ix).p_A0 + GPT2_grid.at(ix).p_A1 * cosfy + GPT2_grid.at(ix).p_B1 * sinfy + GPT2_grid.at(ix).p_A2 * coshy + GPT2_grid.at(ix).p_B2 * sinhy;

            // humidity
            Ql.at(l) = (GPT2_grid.at(ix).Q_A0 + GPT2_grid.at(ix).Q_A1 * cosfy + GPT2_grid.at(ix).Q_B1 * sinfy + GPT2_grid.at(ix).Q_A2 * coshy + GPT2_grid.at(ix).Q_B2 * sinhy)
                       / 1e3;

            // reduction = stationheight - gridheight
            double redh = hgt - GPT2_grid.at(ix).Hs; // Hs: orthometric grid height in m

            // lapse rate of the temperature in degree / m
            dTl.at(l) = (GPT2_grid.at(ix).dT_A0 + GPT2_grid.at(ix).dT_A1 * cosfy + GPT2_grid.at(ix).dT_B1 * sinfy + GPT2_grid.at(ix).dT_A2 * coshy + GPT2_grid.at(ix).dT_B2 * sinhy)
                        / 1e3;

            // temperature reduction to station height
            Tl.at(l) = T0 + dTl.at(l) * redh - 273.150;

            // virtual temperature
            double Tv = T0 * (1 + 0.6077 * Ql.at(l));
            double c = InsConst<>::G_NORM * InsConst<>::dMtr / (InsConst<>::Rg * Tv);

            // pressure in hPa
            pl.at(l) = (p0 * exp(-c * redh)) / 100.0;

            // hydrostatic coefficient ah
            ahl.at(l) = (GPT2_grid.at(ix).h_A0 + GPT2_grid.at(ix).h_A1 * cosfy + GPT2_grid.at(ix).h_B1 * sinfy + GPT2_grid.at(ix).h_A2 * coshy + GPT2_grid.at(ix).h_B2 * sinhy)
                        / 1e3;

            // wet coefficient aw
            awl.at(l) = (GPT2_grid.at(ix).w_A0 + GPT2_grid.at(ix).w_A1 * cosfy + GPT2_grid.at(ix).w_B1 * sinfy + GPT2_grid.at(ix).w_A2 * coshy + GPT2_grid.at(ix).w_B2 * sinhy)
                        / 1e3;

            // water vapor decrease factor la - added by GP
            lal.at(l) = GPT2_grid.at(ix).la_A0 + GPT2_grid.at(ix).la_A1 * cosfy + GPT2_grid.at(ix).la_B1 * sinfy + GPT2_grid.at(ix).la_A2 * coshy + GPT2_grid.at(ix).la_B2 * sinhy;

            // mean temperature of the water vapor Tm - added by GP
            Tml.at(l) = GPT2_grid.at(ix).Tm_A0 + GPT2_grid.at(ix).Tm_A1 * cosfy + GPT2_grid.at(ix).Tm_B1 * sinfy + GPT2_grid.at(ix).Tm_A2 * coshy + GPT2_grid.at(ix).Tm_B2 * sinhy;

            // water vapor pressure in hPa - changed by GP
            double e0 = Ql.at(l) * p0 / (0.622 + 0.378 * Ql.at(l)) / 100.0; // on the grid

            el.at(l) = e0 * pow((100.0 * pl.at(l) / p0), (lal.at(l) + 1.0)); // on the station height  (14) Askne and Nordius, 1987
        }

        double dnpod1 = fabs(diffpod); // distance nearer point
        double dnpod2 = 1.0 - dnpod1;  // distance to distant point
        double dnlon1 = fabs(difflon);
        double dnlon2 = 1.0 - dnlon1;

        // pressure
        double R1 = dnpod2 * pl[0] + dnpod1 * pl[1];
        double R2 = dnpod2 * pl[2] + dnpod1 * pl[3];
        gpt2outputs.p = dnlon2 * R1 + dnlon1 * R2;

        // temperature
        R1 = dnpod2 * Tl[0] + dnpod1 * Tl[1];
        R2 = dnpod2 * Tl[2] + dnpod1 * Tl[3];
        gpt2outputs.T = dnlon2 * R1 + dnlon1 * R2;

        // temperature in degree per km
        R1 = dnpod2 * dTl[0] + dnpod1 * dTl[1];
        R2 = dnpod2 * dTl[2] + dnpod1 * dTl[3];
        gpt2outputs.dT = (dnlon2 * R1 + dnlon1 * R2) * 1e3;

        // water vapor pressure in hPa - changed by GP
        R1 = dnpod2 * el[0] + dnpod1 * el[1];
        R2 = dnpod2 * el[2] + dnpod1 * el[3];
        gpt2outputs.e = dnlon2 * R1 + dnlon1 * R2;

        // hydrostatic
        R1 = dnpod2 * ahl[0] + dnpod1 * ahl[1];
        R2 = dnpod2 * ahl[2] + dnpod1 * ahl[3];
        gpt2outputs.ah = dnlon2 * R1 + dnlon1 * R2;

        // wet
        R1 = dnpod2 * awl[0] + dnpod1 * awl[1];
        R2 = dnpod2 * awl[2] + dnpod1 * awl[3];
        gpt2outputs.aw = dnlon2 * R1 + dnlon1 * R2;

        // undulation
        R1 = dnpod2 * undul[0] + dnpod1 * undul[1];
        R2 = dnpod2 * undul[2] + dnpod1 * undul[3];
        gpt2outputs.undu = dnlon2 * R1 + dnlon1 * R2;

        // water vapor decrease factor la - added by GP
        R1 = dnpod2 * lal[0] + dnpod1 * lal[1];
        R2 = dnpod2 * lal[2] + dnpod1 * lal[3];
        gpt2outputs.la = dnlon2 * R1 + dnlon1 * R2;

        // mean temperature of the water vapor - added by GP
        R1 = dnpod2 * Tml[0] + dnpod1 * Tml[1];
        R2 = dnpod2 * Tml[2] + dnpod1 * Tml[3];
        gpt2outputs.Tm = dnlon2 * R1 + dnlon1 * R2;
    }

    return gpt2outputs;
}

GPT3output GPT3_param(const double& mjd, const Eigen::Vector3d& lla_pos)
{
    using internal::GPT3_grid;

    GPT3output gpt3outputs;

    //   convert mjd to doy
    double doy = mjd2doy(mjd);
    //   factors for amplitudes
    double cosfy = cos(doy / 365.25 * 2.0 * M_PI);
    double coshy = cos(doy / 365.25 * 4.0 * M_PI);
    double sinfy = sin(doy / 365.25 * 2.0 * M_PI);
    double sinhy = sin(doy / 365.25 * 4.0 * M_PI);

    double dlat = lla_pos(0);
    double dlon = lla_pos(1);
    double hell = lla_pos(2);

    // only positive longitude in degrees
    double plon = dlon < 0.0 ? (dlon + 2.0 * M_PI) * 180.0 / M_PI : dlon * 180.0 / M_PI;

    // transform to polar distance in degrees
    double ppod = (-dlat + M_PI / 2.0) * 180.0 / M_PI;

    // find the index (line in the grid file) of the nearest point
    auto ipod = static_cast<int>(floor(ppod + 1.0));
    auto ilon = static_cast<int>(floor(plon + 1.0));

    // normalized (to one) differences, can be positive or negative
    double diffpod = ppod - (ipod - 0.5);
    double difflon = plon - (ilon - 0.5);

    // added by HCY
    if (ipod == 181) { ipod = 180; }
    // added by GP
    if (ilon == 361) { ilon = 1; }
    if (ilon == 0) { ilon = 360; }

    constexpr size_t N = 4;
    std::array<size_t, N> indx{};
    // get the number of the corresponding line
    indx[0] = static_cast<size_t>((ipod - 1) * 360 + ilon) - 1;

    // near the poles: nearest neighbour interpolation
    // otherwise: bilinear, with the 1 degree grid the limits are lower and upper (GP)
    if (ppod <= 0.5 || ppod >= 179.5) // case of nearest neighborhood
    {
        auto ix = indx[0];
        // transforming ellipsoidal height to orthometric height
        gpt3outputs.undu = GPT3_grid.at(ix).undulation; // undu: geoid undulation in m
        double hgt = hell - gpt3outputs.undu;

        // pressure, temperature at the height of the grid
        double T0 = GPT3_grid.at(ix).T_A0 + GPT3_grid.at(ix).T_A1 * cosfy + GPT3_grid.at(ix).T_B1 * sinfy + GPT3_grid.at(ix).T_A2 * coshy + GPT3_grid.at(ix).T_B2 * sinhy;
        double p0 = GPT3_grid.at(ix).p_A0 + GPT3_grid.at(ix).p_A1 * cosfy + GPT3_grid.at(ix).p_B1 * sinfy + GPT3_grid.at(ix).p_A2 * coshy + GPT3_grid.at(ix).p_B2 * sinhy;

        // specific humidity
        double Q = (GPT3_grid.at(ix).Q_A0 + GPT3_grid.at(ix).Q_A1 * cosfy + GPT3_grid.at(ix).Q_B1 * sinfy + GPT3_grid.at(ix).Q_A2 * coshy + GPT3_grid.at(ix).Q_B2 * sinhy)
                   / 1e3;

        // lapse rate of the temperature
        gpt3outputs.dT = (GPT3_grid.at(ix).dT_A0 + GPT3_grid.at(ix).dT_A1 * cosfy + GPT3_grid.at(ix).dT_B1 * sinfy + GPT3_grid.at(ix).dT_A2 * coshy + GPT3_grid.at(ix).dT_B2 * sinhy)
                         / 1e3;

        // station height - grid height
        double redh = hgt - GPT3_grid.at(ix).Hs; // Hs: orthometric grid height in m

        // temperature at station height in Celsius
        gpt3outputs.T = T0 + gpt3outputs.dT * redh - 273.150;

        // temperature lapse rate in degrees / km
        gpt3outputs.dT = gpt3outputs.dT * 1e3;

        // virtual temperature in Kelvin
        double Tv = T0 * (1 + 0.6077 * Q);

        double c = InsConst<>::G_NORM * InsConst<>::dMtr / (InsConst<>::Rg * Tv);

        // pressure in hPa
        gpt3outputs.p = (p0 * exp(-c * redh)) / 100.0;

        // hydrostatic coefficient ah
        gpt3outputs.ah = (GPT3_grid.at(ix).h_A0 + GPT3_grid.at(ix).h_A1 * cosfy + GPT3_grid.at(ix).h_B1 * sinfy + GPT3_grid.at(ix).h_A2 * coshy + GPT3_grid.at(ix).h_B2 * sinhy)
                         / 1e3;

        // wet coefficient aw
        gpt3outputs.aw = (GPT3_grid.at(ix).w_A0 + GPT3_grid.at(ix).w_A1 * cosfy + GPT3_grid.at(ix).w_B1 * sinfy + GPT3_grid.at(ix).w_A2 * coshy + GPT3_grid.at(ix).w_B2 * sinhy)
                         / 1e3;

        // water vapour decrease factor la - added by GP
        gpt3outputs.la = GPT3_grid.at(ix).la_A0 + GPT3_grid.at(ix).la_A1 * cosfy + GPT3_grid.at(ix).la_B1 * sinfy + GPT3_grid.at(ix).la_A2 * coshy + GPT3_grid.at(ix).la_B2 * sinhy;

        // mean temperature of the water vapor Tm - added by GP
        gpt3outputs.Tm = GPT3_grid.at(ix).Tm_A0 + GPT3_grid.at(ix).Tm_A1 * cosfy + GPT3_grid.at(ix).Tm_B1 * sinfy + GPT3_grid.at(ix).Tm_A2 * coshy + GPT3_grid.at(ix).Tm_B2 * sinhy;

        // water vapor pressure in hPa - changed by GP
        double e0 = Q * p0 / (0.622 + 0.378 * Q) / 100.0; // on the grid

        gpt3outputs.e = e0 * pow((100.0 * gpt3outputs.p / p0), (gpt3outputs.la + 1)); // on the station height - (14) Askne and Nordius, 1987

        //   north and east gradients (total, hydrostatic and wet)
        gpt3outputs.Gn_h = (GPT3_grid.at(ix).Gnh_A0 + GPT3_grid.at(ix).Gnh_A1 * cosfy + GPT3_grid.at(ix).Gnh_B1 * sinfy + GPT3_grid.at(ix).Gnh_A2 * coshy + GPT3_grid.at(ix).Gnh_B2 * sinhy)
                           / 1e5;
        gpt3outputs.Ge_h = (GPT3_grid.at(ix).Geh_A0 + GPT3_grid.at(ix).Geh_A1 * cosfy + GPT3_grid.at(ix).Geh_B1 * sinfy + GPT3_grid.at(ix).Geh_A2 * coshy + GPT3_grid.at(ix).Geh_B2 * sinhy)
                           / 1e5;
        gpt3outputs.Gn_w = (GPT3_grid.at(ix).Gnw_A0 + GPT3_grid.at(ix).Gnw_A1 * cosfy + GPT3_grid.at(ix).Gnw_B1 * sinfy + GPT3_grid.at(ix).Gnw_A2 * coshy + GPT3_grid.at(ix).Gnw_B2 * sinhy)
                           / 1e5;
        gpt3outputs.Ge_w = (GPT3_grid.at(ix).Gew_A0 + GPT3_grid.at(ix).Gew_A1 * cosfy + GPT3_grid.at(ix).Gew_B1 * sinfy + GPT3_grid.at(ix).Gew_A2 * coshy + GPT3_grid.at(ix).Gew_B2 * sinhy)
                           / 1e5;
    }
    else // bilinear interpolation
    {
        double ipod1 = ipod + int(math::sign(1.0, diffpod));
        double ilon1 = ilon + int(math::sign(1.0, difflon));

        // changed for the 1 degree grid (GP)
        if (ilon1 == 361) { ilon1 = 1; }
        if (ilon1 == 0) { ilon1 = 360; }
        // get the number of the line
        indx[1] = static_cast<size_t>((ipod1 - 1) * 360 + ilon) - 1;  // along same longitude
        indx[2] = static_cast<size_t>((ipod - 1) * 360 + ilon1) - 1;  // along same polar distance
        indx[3] = static_cast<size_t>((ipod1 - 1) * 360 + ilon1) - 1; // diagonal

        std::array<double, N> undul{};
        std::array<double, N> Ql{};
        std::array<double, N> dTl{};
        std::array<double, N> Tl{};
        std::array<double, N> pl{};
        std::array<double, N> ahl{};
        std::array<double, N> awl{};
        std::array<double, N> lal{};
        std::array<double, N> Tml{};
        std::array<double, N> el{};
        std::array<double, N> Gn_hl{};
        std::array<double, N> Ge_hl{};
        std::array<double, N> Gn_wl{};
        std::array<double, N> Ge_wl{};

        for (size_t l = 0; l < N; l++)
        {
            // transforming ellipsoidal height to orthometric height:
            // Hortho = -N + Hell
            auto ix = indx.at(l);

            undul.at(l) = GPT3_grid.at(ix).undulation; // undu: geoid undulation in m
            double hgt = hell - undul.at(l);

            // pressure, temperature at the height of the grid
            double T0 = GPT3_grid.at(ix).T_A0 + GPT3_grid.at(ix).T_A1 * cosfy + GPT3_grid.at(ix).T_B1 * sinfy + GPT3_grid.at(ix).T_A2 * coshy + GPT3_grid.at(ix).T_B2 * sinhy;
            double p0 = GPT3_grid.at(ix).p_A0 + GPT3_grid.at(ix).p_A1 * cosfy + GPT3_grid.at(ix).p_B1 * sinfy + GPT3_grid.at(ix).p_A2 * coshy + GPT3_grid.at(ix).p_B2 * sinhy;

            // humidity
            Ql.at(l) = (GPT3_grid.at(ix).Q_A0 + GPT3_grid.at(ix).Q_A1 * cosfy + GPT3_grid.at(ix).Q_B1 * sinfy + GPT3_grid.at(ix).Q_A2 * coshy + GPT3_grid.at(ix).Q_B2 * sinhy)
                       / 1e3;

            // reduction = stationheight - gridheight
            double redh = hgt - GPT3_grid.at(ix).Hs; // Hs: orthometric grid height in m

            // lapse rate of the temperature in degree / m
            dTl.at(l) = (GPT3_grid.at(ix).dT_A0 + GPT3_grid.at(ix).dT_A1 * cosfy + GPT3_grid.at(ix).dT_B1 * sinfy + GPT3_grid.at(ix).dT_A2 * coshy + GPT3_grid.at(ix).dT_B2 * sinhy)
                        / 1e3;

            // temperature reduction to station height
            Tl.at(l) = T0 + dTl.at(l) * redh - 273.150;

            // virtual temperature
            double Tv = T0 * (1 + 0.6077 * Ql.at(l));
            double c = InsConst<>::G_NORM * InsConst<>::dMtr / (InsConst<>::Rg * Tv);

            // pressure in hPa
            pl.at(l) = (p0 * exp(-c * redh)) / 100.0;

            // hydrostatic coefficient ah
            ahl.at(l) = (GPT3_grid.at(ix).h_A0 + GPT3_grid.at(ix).h_A1 * cosfy + GPT3_grid.at(ix).h_B1 * sinfy + GPT3_grid.at(ix).h_A2 * coshy + GPT3_grid.at(ix).h_B2 * sinhy)
                        / 1e3;

            // wet coefficient aw
            awl.at(l) = (GPT3_grid.at(ix).w_A0 + GPT3_grid.at(ix).w_A1 * cosfy + GPT3_grid.at(ix).w_B1 * sinfy + GPT3_grid.at(ix).w_A2 * coshy + GPT3_grid.at(ix).w_B2 * sinhy)
                        / 1e3;

            // water vapor decrease factor la - added by GP
            lal.at(l) = GPT3_grid.at(ix).la_A0 + GPT3_grid.at(ix).la_A1 * cosfy + GPT3_grid.at(ix).la_B1 * sinfy + GPT3_grid.at(ix).la_A2 * coshy + GPT3_grid.at(ix).la_B2 * sinhy;

            // mean temperature of the water vapor Tm - added by GP
            Tml.at(l) = GPT3_grid.at(ix).Tm_A0 + GPT3_grid.at(ix).Tm_A1 * cosfy + GPT3_grid.at(ix).Tm_B1 * sinfy + GPT3_grid.at(ix).Tm_A2 * coshy + GPT3_grid.at(ix).Tm_B2 * sinhy;

            // water vapor pressure in hPa - changed by GP
            double e0 = Ql.at(l) * p0 / (0.622 + 0.378 * Ql.at(l)) / 100.0; // on the grid

            el.at(l) = e0 * pow((100.0 * pl.at(l) / p0), (lal.at(l) + 1.0)); // on the station height  (14) Askne and Nordius, 1987

            //   north and east gradients (total, hydrostatic and wet)
            Gn_hl.at(l) = (GPT3_grid.at(ix).Gnh_A0 + GPT3_grid.at(ix).Gnh_A1 * cosfy + GPT3_grid.at(ix).Gnh_B1 * sinfy + GPT3_grid.at(ix).Gnh_A2 * coshy + GPT3_grid.at(ix).Gnh_B2 * sinhy)
                          / 1e5;
            Ge_hl.at(l) = (GPT3_grid.at(ix).Geh_A0 + GPT3_grid.at(ix).Geh_A1 * cosfy + GPT3_grid.at(ix).Geh_B1 * sinfy + GPT3_grid.at(ix).Geh_A2 * coshy + GPT3_grid.at(ix).Geh_B2 * sinhy)
                          / 1e5;
            Gn_wl.at(l) = (GPT3_grid.at(ix).Gnw_A0 + GPT3_grid.at(ix).Gnw_A1 * cosfy + GPT3_grid.at(ix).Gnw_B1 * sinfy + GPT3_grid.at(ix).Gnw_A2 * coshy + GPT3_grid.at(ix).Gnw_B2 * sinhy)
                          / 1e5;
            Ge_wl.at(l) = (GPT3_grid.at(ix).Gew_A0 + GPT3_grid.at(ix).Gew_A1 * cosfy + GPT3_grid.at(ix).Gew_B1 * sinfy + GPT3_grid.at(ix).Gew_A2 * coshy + GPT3_grid.at(ix).Gew_B2 * sinhy)
                          / 1e5;
        }

        double dnpod1 = fabs(diffpod); // distance nearer point
        double dnpod2 = 1.0 - dnpod1;  // distance to distant point
        double dnlon1 = fabs(difflon);
        double dnlon2 = 1.0 - dnlon1;

        // pressure
        double R1 = dnpod2 * pl[0] + dnpod1 * pl[1];
        double R2 = dnpod2 * pl[2] + dnpod1 * pl[3];
        gpt3outputs.p = dnlon2 * R1 + dnlon1 * R2;

        // temperature
        R1 = dnpod2 * Tl[0] + dnpod1 * Tl[1];
        R2 = dnpod2 * Tl[2] + dnpod1 * Tl[3];
        gpt3outputs.T = dnlon2 * R1 + dnlon1 * R2;

        // temperature in degree per km
        R1 = dnpod2 * dTl[0] + dnpod1 * dTl[1];
        R2 = dnpod2 * dTl[2] + dnpod1 * dTl[3];
        gpt3outputs.dT = (dnlon2 * R1 + dnlon1 * R2) * 1e3;

        // water vapor pressure in hPa - changed by GP
        R1 = dnpod2 * el[0] + dnpod1 * el[1];
        R2 = dnpod2 * el[2] + dnpod1 * el[3];
        gpt3outputs.e = dnlon2 * R1 + dnlon1 * R2;

        // hydrostatic
        R1 = dnpod2 * ahl[0] + dnpod1 * ahl[1];
        R2 = dnpod2 * ahl[2] + dnpod1 * ahl[3];
        gpt3outputs.ah = dnlon2 * R1 + dnlon1 * R2;

        // wet
        R1 = dnpod2 * awl[0] + dnpod1 * awl[1];
        R2 = dnpod2 * awl[2] + dnpod1 * awl[3];
        gpt3outputs.aw = dnlon2 * R1 + dnlon1 * R2;

        // undulation
        R1 = dnpod2 * undul[0] + dnpod1 * undul[1];
        R2 = dnpod2 * undul[2] + dnpod1 * undul[3];
        gpt3outputs.undu = dnlon2 * R1 + dnlon1 * R2;

        // water vapor decrease factor la - added by GP
        R1 = dnpod2 * lal[0] + dnpod1 * lal[1];
        R2 = dnpod2 * lal[2] + dnpod1 * lal[3];
        gpt3outputs.la = dnlon2 * R1 + dnlon1 * R2;

        // mean temperature of the water vapor - added by GP
        R1 = dnpod2 * Tml[0] + dnpod1 * Tml[1];
        R2 = dnpod2 * Tml[2] + dnpod1 * Tml[3];
        gpt3outputs.Tm = dnlon2 * R1 + dnlon1 * R2;

        //   gradients
        R1 = dnpod2 * Gn_hl[0] + dnpod1 * Gn_hl[1];
        R2 = dnpod2 * Gn_hl[2] + dnpod1 * Gn_hl[3];
        gpt3outputs.Gn_h = dnlon2 * R1 + dnlon1 * R2;

        R1 = dnpod2 * Ge_hl[0] + dnpod1 * Ge_hl[1];
        R2 = dnpod2 * Ge_hl[2] + dnpod1 * Ge_hl[3];
        gpt3outputs.Ge_h = dnlon2 * R1 + dnlon1 * R2;

        R1 = dnpod2 * Gn_wl[0] + dnpod1 * Gn_wl[1];
        R2 = dnpod2 * Gn_wl[2] + dnpod1 * Gn_wl[3];
        gpt3outputs.Gn_w = dnlon2 * R1 + dnlon1 * R2;

        R1 = dnpod2 * Ge_wl[0] + dnpod1 * Ge_wl[1];
        R2 = dnpod2 * Ge_wl[2] + dnpod1 * Ge_wl[3];
        gpt3outputs.Ge_w = dnlon2 * R1 + dnlon1 * R2;
    }

    return gpt3outputs;
}

double mjd2doy(const double& mjd)
{
    // convert mjd to doy
    double hour = floor((mjd - floor(mjd)) * 24);                         // get hours
    double minu = floor((((mjd - floor(mjd)) * 24) - hour) * 60);         // get minutes
    double sec = (((((mjd - floor(mjd)) * 24) - hour) * 60) - minu) * 60; // get seconds

    // change secs, min hour whose sec==60 and days, whose hour==24
    if (sec == 60.0) { minu = minu + 1; }
    if (minu == 60.0) { hour = hour + 1; }

    // calc jd (yet wrong for hour==24)
    double jd = mjd + 2400000.5;

    // if hour==24, correct jd and set hour==0
    if (hour == 24.0) { jd = jd + 1; }

    // integer Julian date
    double jd_int = floor(jd + 0.5);

    double aa = jd_int + 32044;
    double bb = floor((4 * aa + 3) / 146097);
    double cc = aa - floor((bb * 146097) / 4);
    double dd = floor((4 * cc + 3) / 1461);
    double ee = cc - floor((1461 * dd) / 4);
    double mm = floor((5 * ee + 2) / 153);

    auto day = static_cast<int>(ee - floor((153 * mm + 2) / 5) + 1);
    auto month = static_cast<int>(mm + 3 - 12 * floor(mm / 10));
    auto year = static_cast<int>(bb * 100 + dd - 4800 + floor(mm / 10));

    // first check if the specified year is leap year or not (logical output)
    double leapYear = 0.0;
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
    {
        leapYear = 1.0;
    }
    else
    {
        leapYear = 0.0;
    }
    auto days = [inits = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }] { return std::vector<int>{ std::begin(inits), std::end(inits) }; }();
    int sum = 0;
    for (size_t i = 0; i < static_cast<size_t>(month) - 1; i++)
    {
        sum = sum + days[i];
    }
    double doy = sum + day;

    if (leapYear == 1 && month > 2)
    {
        doy = doy + 1;
    }

    // Add decimal places
    doy = doy + mjd - floor(mjd);

    return doy;
}

double asknewet(const double& e, const double& Tm, const double& la)
{
    // coefficients
    double k1 = 77.604;                       // K/hPa
    double k2 = 64.79;                        // K/hPa
    double k2p = k2 - k1 * 18.0152 / 28.9644; // K/hPa
    double k3 = 377600.0;                     // KK/hPa

    // specific gas constant for dry consituents
    constexpr double Rd = InsConst<>::Rg / InsConst<>::dMtr;

    return 1.0e-6 * (k2p + k3 / Tm) * Rd / (la + 1.0) / InsConst<>::G_NORM * e;
}

} // namespace NAV