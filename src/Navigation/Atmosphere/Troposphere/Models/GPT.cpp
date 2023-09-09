// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GPT.hpp"
#include <vector>

namespace NAV
{
void GPT2_param(const double& mjd, const Eigen::Vector3d& lla_pos, const std::array<internal::GPT2Data, 64800>& GPT2_grid,
                double& p, double& T, double& dT, double& Tm, double& e, double& ah, double& aw, double& la, double& undu)
{
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
    double plon = 0.0;
    double ppod = 0.0;

    // only positive longitude in degrees
    if (dlon < 0.0)
    {
        plon = (dlon + 2.0 * M_PI) * 180.0 / M_PI;
    }
    else
    {
        plon = dlon * 180.0 / M_PI;
    }
    // transform to polar distance in degrees
    ppod = (-dlat + M_PI / 2.0) * 180.0 / M_PI;

    // find the index (line in the grid file) of the nearest point
    auto ipod = static_cast<int>(floor(ppod + 1.0));
    auto ilon = static_cast<int>(floor(plon + 1.0));

    // normalized (to one) differences, can be positive or negative
    double diffpod = ppod - (ipod - 0.5);
    double difflon = plon - (ilon - 0.5);

    // added by HCY
    if (ipod == 181)
    {
        ipod = 180;
    }
    // added by GP
    if (ilon == 361)
    {
        ilon = 1;
    }
    if (ilon == 0)
    {
        ilon = 360;
    }
    std::vector<unsigned int> indx(4);
    // get the number of the corresponding line
    indx[0] = static_cast<unsigned int>((ipod - 1) * 360 + ilon);
    indx[0] -= 1; // -1 to use in c

    // near the poles: nearest neighbour interpolation, otherwise: bilinear
    // with the 1 degree grid the limits are lower and upper (GP)
    int ibilinear = 0;
    if ((ppod > 0.5) && (ppod < 179.5))
    {
        ibilinear = 1;
    }
    unsigned int ix = 0;
    double hgt = 0.0;
    double T0 = 0.0;
    double p0 = 0.0;
    double Q = 0.0;
    double e0 = 0.0;
    double redh = 0.0;
    double Tv = 0.0;
    double c = 0.0;

    // case of nearest neighborhood
    if (ibilinear == 0) // then
    {
        ix = indx[0] - 1;                   // -1 to use in c
                                            // transforming ellipsoidal height to orthometric height
        undu = GPT2_grid.at(ix).undulation; // undu: geoid undulation in m
        hgt = hell - undu;

        // pressure, temperature at the height of the grid
        T0 = GPT2_grid.at(ix).T_A0 + GPT2_grid.at(ix).T_A1 * cosfy + GPT2_grid.at(ix).T_B1 * sinfy + GPT2_grid.at(ix).T_A2 * coshy + GPT2_grid.at(ix).T_B2 * sinhy;
        p0 = GPT2_grid.at(ix).p_A0 + GPT2_grid.at(ix).p_A1 * cosfy + GPT2_grid.at(ix).p_B1 * sinfy + GPT2_grid.at(ix).p_A2 * coshy + GPT2_grid.at(ix).p_B2 * sinhy;

        // specific humidity
        Q = GPT2_grid.at(ix).Q_A0 + GPT2_grid.at(ix).Q_A1 * cosfy + GPT2_grid.at(ix).Q_B1 * sinfy + GPT2_grid.at(ix).Q_A2 * coshy + GPT2_grid.at(ix).Q_B2 * sinhy;
        Q /= 1000.0;

        // lapse rate of the temperature
        dT = GPT2_grid.at(ix).dT_A0 + GPT2_grid.at(ix).dT_A1 * cosfy + GPT2_grid.at(ix).dT_B1 * sinfy + GPT2_grid.at(ix).dT_A2 * coshy + GPT2_grid.at(ix).dT_B2 * sinhy;
        dT /= 1000.0;

        // station height - grid height
        redh = hgt - GPT2_grid.at(ix).Hs; // Hs: orthometric grid height in m

        // temperature at station height in Celsius
        T = T0 + dT * redh - 273.150;

        // temperature lapse rate in degrees / km
        dT = dT * 1000.0;

        // virtual temperature in Kelvin
        Tv = T0 * (1 + 0.6077 * Q);

        c = gm * dMtr / (Rg * Tv);

        // pressure in hPa
        p = (p0 * exp(-c * redh)) / 100.0;

        // hydrostatic coefficient ah
        ah = GPT2_grid.at(ix).h_A0 + GPT2_grid.at(ix).h_A1 * cosfy + GPT2_grid.at(ix).h_B1 * sinfy + GPT2_grid.at(ix).h_A2 * coshy + GPT2_grid.at(ix).h_B2 * sinhy;
        ah /= 1000.0;

        // wet coefficient aw
        aw = GPT2_grid.at(ix).w_A0 + GPT2_grid.at(ix).w_A1 * cosfy + GPT2_grid.at(ix).w_B1 * sinfy + GPT2_grid.at(ix).w_A2 * coshy + GPT2_grid.at(ix).w_B2 * sinhy;
        aw /= 1000.0;

        // water vapour decrease factor la - added by GP
        la = GPT2_grid.at(ix).la_A0 + GPT2_grid.at(ix).la_A1 * cosfy + GPT2_grid.at(ix).la_B1 * sinfy + GPT2_grid.at(ix).la_A2 * coshy + GPT2_grid.at(ix).la_B2 * sinhy;

        // mean temperature of the water vapor Tm - added by GP
        Tm = GPT2_grid.at(ix).Tm_A0 + GPT2_grid.at(ix).Tm_A1 * cosfy + GPT2_grid.at(ix).Tm_B1 * sinfy + GPT2_grid.at(ix).Tm_A2 * coshy + GPT2_grid.at(ix).Tm_B2 * sinhy;

        // water vapor pressure in hPa - changed by GP
        e0 = Q * p0 / (0.622 + 0.378 * Q) / 100.0; // on the grid

        e = e0 * pow((100.0 * p / p0), (la + 1)); // on the station height - (14) Askne and Nordius, 1987
    }
    else // bilinear interpolation
    {
        double ipod1 = ipod + int(sign(1.0, diffpod));
        double ilon1 = ilon + int(sign(1.0, difflon));

        // changed for the 1 degree grid (GP)
        if (ilon1 == 361)
        {
            ilon1 = 1;
        }
        if (ilon1 == 0)
        {
            ilon1 = 360;
        }
        // get the number of the line
        indx[1] = static_cast<unsigned int>((ipod1 - 1) * 360 + ilon);  // along same longitude
        indx[2] = static_cast<unsigned int>((ipod - 1) * 360 + ilon1);  // along same polar distance
        indx[3] = static_cast<unsigned int>((ipod1 - 1) * 360 + ilon1); // diagonal

        indx[1] -= 1;
        indx[2] -= 1;
        indx[3] -= 1;

        unsigned int l = 0;
        std::vector<double> undul(4);
        std::vector<double> Ql(4);
        std::vector<double> dTl(4);
        std::vector<double> Tl(4);
        std::vector<double> pl(4);
        std::vector<double> ahl(4);
        std::vector<double> awl(4);
        std::vector<double> lal(4);
        std::vector<double> Tml(4);
        std::vector<double> el(4);

        do // do l = 1:4
        {
            // transforming ellipsoidal height to orthometric height:
            // Hortho = -N + Hell
            auto ix = indx[l];

            undul[l] = GPT2_grid.at(ix).undulation; // undu: geoid undulation in m
            hgt = hell - undul[l];

            // pressure, temperature at the height of the grid
            T0 = GPT2_grid.at(ix).T_A0 + GPT2_grid.at(ix).T_A1 * cosfy + GPT2_grid.at(ix).T_B1 * sinfy + GPT2_grid.at(ix).T_A2 * coshy + GPT2_grid.at(ix).T_B2 * sinhy;
            p0 = GPT2_grid.at(ix).p_A0 + GPT2_grid.at(ix).p_A1 * cosfy + GPT2_grid.at(ix).p_B1 * sinfy + GPT2_grid.at(ix).p_A2 * coshy + GPT2_grid.at(ix).p_B2 * sinhy;

            // humidity
            Ql[l] = GPT2_grid.at(ix).Q_A0 + GPT2_grid.at(ix).Q_A1 * cosfy + GPT2_grid.at(ix).Q_B1 * sinfy + GPT2_grid.at(ix).Q_A2 * coshy + GPT2_grid.at(ix).Q_B2 * sinhy;
            Ql[l] /= 1000.0;

            // reduction = stationheight - gridheight
            redh = hgt - GPT2_grid.at(ix).Hs; // Hs: orthometric grid height in m

            // lapse rate of the temperature in degree / m
            dTl[l] = GPT2_grid.at(ix).dT_A0 + GPT2_grid.at(ix).dT_A1 * cosfy + GPT2_grid.at(ix).dT_B1 * sinfy + GPT2_grid.at(ix).dT_A2 * coshy + GPT2_grid.at(ix).dT_B2 * sinhy;
            dTl[l] /= 1000.0;

            // temperature reduction to station height
            Tl[l] = T0 + dTl[l] * redh - 273.150;

            // virtual temperature
            Tv = T0 * (1 + 0.6077 * Ql[l]);
            c = gm * dMtr / (Rg * Tv);

            // pressure in hPa
            pl[l] = (p0 * exp(-c * redh)) / 100.0;

            // hydrostatic coefficient ah
            ahl[l] = GPT2_grid.at(ix).h_A0 + GPT2_grid.at(ix).h_A1 * cosfy + GPT2_grid.at(ix).h_B1 * sinfy + GPT2_grid.at(ix).h_A2 * coshy + GPT2_grid.at(ix).h_B2 * sinhy;
            ahl[l] /= 1000.0;

            // wet coefficient aw
            awl[l] = GPT2_grid.at(ix).w_A0 + GPT2_grid.at(ix).w_A1 * cosfy + GPT2_grid.at(ix).w_B1 * sinfy + GPT2_grid.at(ix).w_A2 * coshy + GPT2_grid.at(ix).w_B2 * sinhy;
            awl[l] /= 1000.0;

            // water vapor decrease factor la - added by GP
            lal[l] = GPT2_grid.at(ix).la_A0 + GPT2_grid.at(ix).la_A1 * cosfy + GPT2_grid.at(ix).la_B1 * sinfy + GPT2_grid.at(ix).la_A2 * coshy + GPT2_grid.at(ix).la_B2 * sinhy;

            // mean temperature of the water vapor Tm - added by GP
            Tml[l] = GPT2_grid.at(ix).Tm_A0 + GPT2_grid.at(ix).Tm_A1 * cosfy + GPT2_grid.at(ix).Tm_B1 * sinfy + GPT2_grid.at(ix).Tm_A2 * coshy + GPT2_grid.at(ix).Tm_B2 * sinhy;

            // water vapor pressure in hPa - changed by GP
            e0 = Ql[l] * p0 / (0.622 + 0.378 * Ql[l]) / 100.0; // on the grid

            el[l] = e0 * pow((100.0 * pl[l] / p0), (lal[l] + 1.0)); // on the station height  (14) Askne and Nordius, 1987

            l++;

        } while (l < 4); // end do

        double dnpod1 = fabs(diffpod); // distance nearer point
        double dnpod2 = 1.0 - dnpod1;  // distance to distant point
        double dnlon1 = fabs(difflon);
        double dnlon2 = 1.0 - dnlon1;

        double R1 = 0.0;
        double R2 = 0.0;

        // pressure
        R1 = dnpod2 * pl[0] + dnpod1 * pl[1];
        R2 = dnpod2 * pl[2] + dnpod1 * pl[3];
        p = dnlon2 * R1 + dnlon1 * R2;

        // temperature
        R1 = dnpod2 * Tl[0] + dnpod1 * Tl[1];
        R2 = dnpod2 * Tl[2] + dnpod1 * Tl[3];
        T = dnlon2 * R1 + dnlon1 * R2;

        // temperature in degree per km
        R1 = dnpod2 * dTl[0] + dnpod1 * dTl[1];
        R2 = dnpod2 * dTl[2] + dnpod1 * dTl[3];
        dT = (dnlon2 * R1 + dnlon1 * R2) * 1000.0;

        // water vapor pressure in hPa - changed by GP
        R1 = dnpod2 * el[0] + dnpod1 * el[1];
        R2 = dnpod2 * el[2] + dnpod1 * el[3];
        e = dnlon2 * R1 + dnlon1 * R2;

        // hydrostatic
        R1 = dnpod2 * ahl[0] + dnpod1 * ahl[1];
        R2 = dnpod2 * ahl[2] + dnpod1 * ahl[3];
        ah = dnlon2 * R1 + dnlon1 * R2;

        // wet
        R1 = dnpod2 * awl[0] + dnpod1 * awl[1];
        R2 = dnpod2 * awl[2] + dnpod1 * awl[3];
        aw = dnlon2 * R1 + dnlon1 * R2;

        // undulation
        R1 = dnpod2 * undul[0] + dnpod1 * undul[1];
        R2 = dnpod2 * undul[2] + dnpod1 * undul[3];
        undu = dnlon2 * R1 + dnlon1 * R2;

        // water vapor decrease factor la - added by GP
        R1 = dnpod2 * lal[0] + dnpod1 * lal[1];
        R2 = dnpod2 * lal[2] + dnpod1 * lal[3];
        la = dnlon2 * R1 + dnlon1 * R2;

        // mean temperature of the water vapor - added by GP
        R1 = dnpod2 * Tml[0] + dnpod1 * Tml[1];
        R2 = dnpod2 * Tml[2] + dnpod1 * Tml[3];
        Tm = dnlon2 * R1 + dnlon1 * R2;
    } // else
}

void GPT3_param(const double& mjd, const Eigen::Vector3d& lla_pos, const std::array<internal::GPT3Data, 64800>& GPT3_grid,
                double& p, double& T, double& dT, double& Tm, double& e, double& ah, double& aw, double& la, double& undu,
                double& Gn_h, double& Ge_h, double& Gn_w, double& Ge_w)
{
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
    double plon = 0.0;
    double ppod = 0.0;

    // only positive longitude in degrees
    if (dlon < 0.0)
    {
        plon = (dlon + 2.0 * M_PI) * 180.0 / M_PI;
    }
    else
    {
        plon = dlon * 180.0 / M_PI;
    }
    // transform to polar distance in degrees
    ppod = (-dlat + M_PI / 2.0) * 180.0 / M_PI;

    // find the index (line in the grid file) of the nearest point
    auto ipod = static_cast<int>(floor(ppod + 1.0));
    auto ilon = static_cast<int>(floor(plon + 1.0));

    // normalized (to one) differences, can be positive or negative
    double diffpod = ppod - (ipod - 0.5);
    double difflon = plon - (ilon - 0.5);

    // added by HCY
    if (ipod == 181)
    {
        ipod = 180;
    }
    // added by GP
    if (ilon == 361)
    {
        ilon = 1;
    }
    if (ilon == 0)
    {
        ilon = 360;
    }

    std::vector<unsigned int> indx(4);
    // get the number of the corresponding line
    indx[0] = static_cast<unsigned int>((ipod - 1) * 360 + ilon);
    indx[0] -= 1; // -1 to use in c

    // near the poles: nearest neighbour interpolation, otherwise: bilinear
    // with the 1 degree grid the limits are lower and upper (GP)
    int ibilinear = 0;
    if ((ppod > 0.5) && (ppod < 179.5))
    {
        ibilinear = 1;
    }

    unsigned int ix = 0;
    double hgt = 0.0;
    double T0 = 0.0;
    double p0 = 0.0;
    double Q = 0.0;
    double e0 = 0.0;
    double redh = 0.0;
    double Tv = 0.0;
    double c = 0.0;

    // case of nearest neighborhood
    if (ibilinear == 0) // then
    {
        ix = indx[0] - 1;                   // -1 to use in c
                                            // transforming ellipsoidal height to orthometric height
        undu = GPT3_grid.at(ix).undulation; // undu: geoid undulation in m
        hgt = hell - undu;

        // pressure, temperature at the height of the grid
        T0 = GPT3_grid.at(ix).T_A0 + GPT3_grid.at(ix).T_A1 * cosfy + GPT3_grid.at(ix).T_B1 * sinfy + GPT3_grid.at(ix).T_A2 * coshy + GPT3_grid.at(ix).T_B2 * sinhy;
        p0 = GPT3_grid.at(ix).p_A0 + GPT3_grid.at(ix).p_A1 * cosfy + GPT3_grid.at(ix).p_B1 * sinfy + GPT3_grid.at(ix).p_A2 * coshy + GPT3_grid.at(ix).p_B2 * sinhy;

        // specific humidity
        Q = GPT3_grid.at(ix).Q_A0 + GPT3_grid.at(ix).Q_A1 * cosfy + GPT3_grid.at(ix).Q_B1 * sinfy + GPT3_grid.at(ix).Q_A2 * coshy + GPT3_grid.at(ix).Q_B2 * sinhy;
        Q /= 1000.0;

        // lapse rate of the temperature
        dT = GPT3_grid.at(ix).dT_A0 + GPT3_grid.at(ix).dT_A1 * cosfy + GPT3_grid.at(ix).dT_B1 * sinfy + GPT3_grid.at(ix).dT_A2 * coshy + GPT3_grid.at(ix).dT_B2 * sinhy;
        dT /= 1000.0;

        // station height - grid height
        redh = hgt - GPT3_grid.at(ix).Hs; // Hs: orthometric grid height in m

        // temperature at station height in Celsius
        T = T0 + dT * redh - 273.150;

        // temperature lapse rate in degrees / km
        dT = dT * 1000.0;

        // virtual temperature in Kelvin
        Tv = T0 * (1 + 0.6077 * Q);

        c = gm * dMtr / (Rg * Tv);

        // pressure in hPa
        p = (p0 * exp(-c * redh)) / 100.0;

        // hydrostatic coefficient ah
        ah = GPT3_grid.at(ix).h_A0 + GPT3_grid.at(ix).h_A1 * cosfy + GPT3_grid.at(ix).h_B1 * sinfy + GPT3_grid.at(ix).h_A2 * coshy + GPT3_grid.at(ix).h_B2 * sinhy;
        ah /= 1000.0;

        // wet coefficient aw
        aw = GPT3_grid.at(ix).w_A0 + GPT3_grid.at(ix).w_A1 * cosfy + GPT3_grid.at(ix).w_B1 * sinfy + GPT3_grid.at(ix).w_A2 * coshy + GPT3_grid.at(ix).w_B2 * sinhy;
        aw /= 1000.0;

        // water vapour decrease factor la - added by GP
        la = GPT3_grid.at(ix).la_A0 + GPT3_grid.at(ix).la_A1 * cosfy + GPT3_grid.at(ix).la_B1 * sinfy + GPT3_grid.at(ix).la_A2 * coshy + GPT3_grid.at(ix).la_B2 * sinhy;

        // mean temperature of the water vapor Tm - added by GP
        Tm = GPT3_grid.at(ix).Tm_A0 + GPT3_grid.at(ix).Tm_A1 * cosfy + GPT3_grid.at(ix).Tm_B1 * sinfy + GPT3_grid.at(ix).Tm_A2 * coshy + GPT3_grid.at(ix).Tm_B2 * sinhy;

        // water vapor pressure in hPa - changed by GP
        e0 = Q * p0 / (0.622 + 0.378 * Q) / 100.0; // on the grid

        e = e0 * pow((100.0 * p / p0), (la + 1)); // on the station height - (14) Askne and Nordius, 1987

        //   north and east gradients (total, hydrostatic and wet)
        Gn_h = GPT3_grid.at(ix).Gnh_A0 + GPT3_grid.at(ix).Gnh_A1 * cosfy + GPT3_grid.at(ix).Gnh_B1 * sinfy + GPT3_grid.at(ix).Gnh_A2 * coshy + GPT3_grid.at(ix).Gnh_B2 * sinhy;
        Gn_h /= 100000.0;
        Ge_h = GPT3_grid.at(ix).Geh_A0 + GPT3_grid.at(ix).Geh_A1 * cosfy + GPT3_grid.at(ix).Geh_B1 * sinfy + GPT3_grid.at(ix).Geh_A2 * coshy + GPT3_grid.at(ix).Geh_B2 * sinhy;
        Ge_h /= 100000.0;
        Gn_w = GPT3_grid.at(ix).Gnw_A0 + GPT3_grid.at(ix).Gnw_A1 * cosfy + GPT3_grid.at(ix).Gnw_B1 * sinfy + GPT3_grid.at(ix).Gnw_A2 * coshy + GPT3_grid.at(ix).Gnw_B2 * sinhy;
        Gn_w /= 100000.0;
        Ge_w = GPT3_grid.at(ix).Gew_A0 + GPT3_grid.at(ix).Gew_A1 * cosfy + GPT3_grid.at(ix).Gew_B1 * sinfy + GPT3_grid.at(ix).Gew_A2 * coshy + GPT3_grid.at(ix).Gew_B2 * sinhy;
        Ge_w /= 100000.0;
    }
    else // bilinear interpolation
    {
        double ipod1 = ipod + int(sign(1.0, diffpod));
        double ilon1 = ilon + int(sign(1.0, difflon));

        // changed for the 1 degree grid (GP)
        if (ilon1 == 361)
        {
            ilon1 = 1;
        }
        if (ilon1 == 0)
        {
            ilon1 = 360;
        }
        // get the number of the line
        indx[1] = static_cast<unsigned int>((ipod1 - 1) * 360 + ilon);  // along same longitude
        indx[2] = static_cast<unsigned int>((ipod - 1) * 360 + ilon1);  // along same polar distance
        indx[3] = static_cast<unsigned int>((ipod1 - 1) * 360 + ilon1); // diagonal

        indx[1] -= 1;
        indx[2] -= 1;
        indx[3] -= 1;

        unsigned int l = 0;
        std::vector<double> undul(4);
        std::vector<double> Ql(4);
        std::vector<double> dTl(4);
        std::vector<double> Tl(4);
        std::vector<double> pl(4);
        std::vector<double> ahl(4);
        std::vector<double> awl(4);
        std::vector<double> lal(4);
        std::vector<double> Tml(4);
        std::vector<double> el(4);
        std::vector<double> Gn_hl(4);
        std::vector<double> Ge_hl(4);
        std::vector<double> Gn_wl(4);
        std::vector<double> Ge_wl(4);
        do // do l = 1:4
        {
            // transforming ellipsoidal height to orthometric height:
            // Hortho = -N + Hell
            auto ix = indx[l];

            undul[l] = GPT3_grid.at(ix).undulation; // undu: geoid undulation in m
            hgt = hell - undul[l];

            // pressure, temperature at the height of the grid
            T0 = GPT3_grid.at(ix).T_A0 + GPT3_grid.at(ix).T_A1 * cosfy + GPT3_grid.at(ix).T_B1 * sinfy + GPT3_grid.at(ix).T_A2 * coshy + GPT3_grid.at(ix).T_B2 * sinhy;
            p0 = GPT3_grid.at(ix).p_A0 + GPT3_grid.at(ix).p_A1 * cosfy + GPT3_grid.at(ix).p_B1 * sinfy + GPT3_grid.at(ix).p_A2 * coshy + GPT3_grid.at(ix).p_B2 * sinhy;

            // humidity
            Ql[l] = GPT3_grid.at(ix).Q_A0 + GPT3_grid.at(ix).Q_A1 * cosfy + GPT3_grid.at(ix).Q_B1 * sinfy + GPT3_grid.at(ix).Q_A2 * coshy + GPT3_grid.at(ix).Q_B2 * sinhy;
            Ql[l] /= 1000.0;

            // reduction = stationheight - gridheight
            redh = hgt - GPT3_grid.at(ix).Hs; // Hs: orthometric grid height in m

            // lapse rate of the temperature in degree / m
            dTl[l] = GPT3_grid.at(ix).dT_A0 + GPT3_grid.at(ix).dT_A1 * cosfy + GPT3_grid.at(ix).dT_B1 * sinfy + GPT3_grid.at(ix).dT_A2 * coshy + GPT3_grid.at(ix).dT_B2 * sinhy;
            dTl[l] /= 1000.0;

            // temperature reduction to station height
            Tl[l] = T0 + dTl[l] * redh - 273.150;

            // virtual temperature
            Tv = T0 * (1 + 0.6077 * Ql[l]);
            c = gm * dMtr / (Rg * Tv);

            // pressure in hPa
            pl[l] = (p0 * exp(-c * redh)) / 100.0;

            // hydrostatic coefficient ah
            ahl[l] = GPT3_grid.at(ix).h_A0 + GPT3_grid.at(ix).h_A1 * cosfy + GPT3_grid.at(ix).h_B1 * sinfy + GPT3_grid.at(ix).h_A2 * coshy + GPT3_grid.at(ix).h_B2 * sinhy;
            ahl[l] /= 1000.0;

            // wet coefficient aw
            awl[l] = GPT3_grid.at(ix).w_A0 + GPT3_grid.at(ix).w_A1 * cosfy + GPT3_grid.at(ix).w_B1 * sinfy + GPT3_grid.at(ix).w_A2 * coshy + GPT3_grid.at(ix).w_B2 * sinhy;
            awl[l] /= 1000.0;

            // water vapor decrease factor la - added by GP
            lal[l] = GPT3_grid.at(ix).la_A0 + GPT3_grid.at(ix).la_A1 * cosfy + GPT3_grid.at(ix).la_B1 * sinfy + GPT3_grid.at(ix).la_A2 * coshy + GPT3_grid.at(ix).la_B2 * sinhy;

            // mean temperature of the water vapor Tm - added by GP
            Tml[l] = GPT3_grid.at(ix).Tm_A0 + GPT3_grid.at(ix).Tm_A1 * cosfy + GPT3_grid.at(ix).Tm_B1 * sinfy + GPT3_grid.at(ix).Tm_A2 * coshy + GPT3_grid.at(ix).Tm_B2 * sinhy;

            // water vapor pressure in hPa - changed by GP
            e0 = Ql[l] * p0 / (0.622 + 0.378 * Ql[l]) / 100.0; // on the grid

            el[l] = e0 * pow((100.0 * pl[l] / p0), (lal[l] + 1.0)); // on the station height  (14) Askne and Nordius, 1987

            //   north and east gradients (total, hydrostatic and wet)
            Gn_hl[l] = GPT3_grid.at(ix).Gnh_A0 + GPT3_grid.at(ix).Gnh_A1 * cosfy + GPT3_grid.at(ix).Gnh_B1 * sinfy + GPT3_grid.at(ix).Gnh_A2 * coshy + GPT3_grid.at(ix).Gnh_B2 * sinhy;
            Gn_hl[l] /= 100000.0;
            Ge_hl[l] = GPT3_grid.at(ix).Geh_A0 + GPT3_grid.at(ix).Geh_A1 * cosfy + GPT3_grid.at(ix).Geh_B1 * sinfy + GPT3_grid.at(ix).Geh_A2 * coshy + GPT3_grid.at(ix).Geh_B2 * sinhy;
            Ge_hl[l] /= 100000.0;
            Gn_wl[l] = GPT3_grid.at(ix).Gnw_A0 + GPT3_grid.at(ix).Gnw_A1 * cosfy + GPT3_grid.at(ix).Gnw_B1 * sinfy + GPT3_grid.at(ix).Gnw_A2 * coshy + GPT3_grid.at(ix).Gnw_B2 * sinhy;
            Gn_wl[l] /= 100000.0;
            Ge_wl[l] = GPT3_grid.at(ix).Gew_A0 + GPT3_grid.at(ix).Gew_A1 * cosfy + GPT3_grid.at(ix).Gew_B1 * sinfy + GPT3_grid.at(ix).Gew_A2 * coshy + GPT3_grid.at(ix).Gew_B2 * sinhy;
            Ge_wl[l] /= 100000.0;

            l++;

        } while (l < 4);               // end do
        double dnpod1 = fabs(diffpod); // distance nearer point
        double dnpod2 = 1.0 - dnpod1;  // distance to distant point
        double dnlon1 = fabs(difflon);
        double dnlon2 = 1.0 - dnlon1;

        double R1 = 0.0;
        double R2 = 0.0;

        // pressure
        R1 = dnpod2 * pl[0] + dnpod1 * pl[1];
        R2 = dnpod2 * pl[2] + dnpod1 * pl[3];
        p = dnlon2 * R1 + dnlon1 * R2;

        // temperature
        R1 = dnpod2 * Tl[0] + dnpod1 * Tl[1];
        R2 = dnpod2 * Tl[2] + dnpod1 * Tl[3];
        T = dnlon2 * R1 + dnlon1 * R2;

        // temperature in degree per km
        R1 = dnpod2 * dTl[0] + dnpod1 * dTl[1];
        R2 = dnpod2 * dTl[2] + dnpod1 * dTl[3];
        dT = (dnlon2 * R1 + dnlon1 * R2) * 1000.0;

        // water vapor pressure in hPa - changed by GP
        R1 = dnpod2 * el[0] + dnpod1 * el[1];
        R2 = dnpod2 * el[2] + dnpod1 * el[3];
        e = dnlon2 * R1 + dnlon1 * R2;

        // hydrostatic
        R1 = dnpod2 * ahl[0] + dnpod1 * ahl[1];
        R2 = dnpod2 * ahl[2] + dnpod1 * ahl[3];
        ah = dnlon2 * R1 + dnlon1 * R2;

        // wet
        R1 = dnpod2 * awl[0] + dnpod1 * awl[1];
        R2 = dnpod2 * awl[2] + dnpod1 * awl[3];
        aw = dnlon2 * R1 + dnlon1 * R2;

        // undulation
        R1 = dnpod2 * undul[0] + dnpod1 * undul[1];
        R2 = dnpod2 * undul[2] + dnpod1 * undul[3];
        undu = dnlon2 * R1 + dnlon1 * R2;

        // water vapor decrease factor la - added by GP
        R1 = dnpod2 * lal[0] + dnpod1 * lal[1];
        R2 = dnpod2 * lal[2] + dnpod1 * lal[3];
        la = dnlon2 * R1 + dnlon1 * R2;

        // mean temperature of the water vapor - added by GP
        R1 = dnpod2 * Tml[0] + dnpod1 * Tml[1];
        R2 = dnpod2 * Tml[2] + dnpod1 * Tml[3];
        Tm = dnlon2 * R1 + dnlon1 * R2;

        //   gradients
        R1 = dnpod2 * Gn_hl[0] + dnpod1 * Gn_hl[1];
        R2 = dnpod2 * Gn_hl[2] + dnpod1 * Gn_hl[3];
        Gn_h = dnlon2 * R1 + dnlon1 * R2;

        R1 = dnpod2 * Ge_hl[0] + dnpod1 * Ge_hl[1];
        R2 = dnpod2 * Ge_hl[2] + dnpod1 * Ge_hl[3];
        Ge_h = dnlon2 * R1 + dnlon1 * R2;

        R1 = dnpod2 * Gn_wl[0] + dnpod1 * Gn_wl[1];
        R2 = dnpod2 * Gn_wl[2] + dnpod1 * Gn_wl[3];
        Gn_w = dnlon2 * R1 + dnlon1 * R2;

        R1 = dnpod2 * Ge_wl[0] + dnpod1 * Ge_wl[1];
        R2 = dnpod2 * Ge_wl[2] + dnpod1 * Ge_wl[3];
        Ge_w = dnlon2 * R1 + dnlon1 * R2;
    } // else
}
double mjd2doy(const double& mjd)
{
    double hour = 0.0;
    double minu = 0.0;
    double sec = 0.0;
    double jd = 0.0;
    double jd_int = 0.0;
    double aa = 0.0;
    double bb = 0.0;
    double cc = 0.0;
    double dd = 0.0;
    double ee = 0.0;
    double mm = 0.0;
    double leapYear = 0.0;
    double doy = 0.0;

    int sum = 0;
    int day = 0;
    int month = 0;
    int year = 0;

    //   convert mjd to doy
    hour = floor((mjd - floor(mjd)) * 24);                         //   get hours
    minu = floor((((mjd - floor(mjd)) * 24) - hour) * 60);         //   get minutes
    sec = (((((mjd - floor(mjd)) * 24) - hour) * 60) - minu) * 60; //   get seconds

    //   change secs, min hour whose sec==60 and days, whose hour==24
    if (sec == 60.0)
    {
        minu = minu + 1;
    }
    if (minu == 60.0)
    {
        hour = hour + 1;
    }

    //   calc jd (yet wrong for hour==24)
    jd = mjd + 2400000.5;

    //   if hour==24, correct jd and set hour==0
    if (hour == 24.0)
    {
        jd = jd + 1;
    }

    //   integer Julian date
    jd_int = floor(jd + 0.5);

    aa = jd_int + 32044;
    bb = floor((4 * aa + 3) / 146097);
    cc = aa - floor((bb * 146097) / 4);
    dd = floor((4 * cc + 3) / 1461);
    ee = cc - floor((1461 * dd) / 4);
    mm = floor((5 * ee + 2) / 153);

    day = static_cast<int>(ee - floor((153 * mm + 2) / 5) + 1);
    month = static_cast<int>(mm + 3 - 12 * floor(mm / 10));
    year = static_cast<int>(bb * 100 + dd - 4800 + floor(mm / 10));

    //   first check if the specified year is leap year or not (logical output)
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
    {
        leapYear = 1.0;
    }
    else
    {
        leapYear = 0.0;
    }
    auto days = [inits = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }] { return std::vector<int>{ std::begin(inits), std::end(inits) }; }();
    for (unsigned int i = 0; i < static_cast<unsigned int>(month) - 1; i++)
    {
        sum = sum + days[i];
    }
    doy = sum + day;

    if (leapYear == 1 && month > 2)
    {
        doy = doy + 1;
    }

    //   add decimal places
    doy = doy + mjd - floor(mjd);

    return doy;
}

double sign(const double& x, const double& y)
{
    double value = 0.0;
    // similar function 'sign' in fortran
    if (y >= 0.0)
    {
        value = fabs(x);
    }
    else if (y < 0.0)
    {
        value = -1.0 * fabs(x);
    }
    else
    {
        value = 0.0;
    }
    return value;
}

double asknewet(const double& e, const double& Tm, const double& la)
{
    double k1 = 0.0;
    double k2 = 0.0;
    double k2p = 0.0;
    double k3 = 0.0;
    double zwd = 0.0;

    // coefficients
    k1 = 77.604;                       // K/hPa
    k2 = 64.79;                        // K/hPa
    k2p = k2 - k1 * 18.0152 / 28.9644; // K/hPa
    k3 = 377600.0;                     // KK/hPa

    // specific gas constant for dry consituents
    double Rd = Rg / dMtr;

    zwd = 1.0e-6 * (k2p + k3 / Tm) * Rd / (la + 1.0) / gm * e;
    return zwd;
}

} // namespace NAV