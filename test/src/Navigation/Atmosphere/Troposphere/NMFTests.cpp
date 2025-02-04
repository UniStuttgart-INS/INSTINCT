// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NMFTests.cpp
/// @brief Neill Mapping Function Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-17

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include "Navigation/Atmosphere/Troposphere/MappingFunctions/NiellMappingFunction.hpp"
#include "Navigation/Transformations/Units.hpp"
#include <catch2/matchers/catch_matchers.hpp>

namespace NAV::TESTS::NMF
{

// NOLINTBEGIN

using gtime_t = struct
{                /* time struct */
    time_t time; /* time (s) expressed by standard time_t */
    double sec;  /* fraction of second under 1 s */
};

extern double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}

extern void time2epoch(gtime_t t, double* ep)
{
    const int mday[] = { /* # of days in a month */
                         31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                         31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    int days, sec, mon, day;

    /* leap year if year%4==0 in 1901-2099 */
    days = static_cast<int>(t.time / 86400);
    sec = static_cast<int>(t.time - static_cast<time_t>(days) * 86400);
    for (day = days % 1461, mon = 0; mon < 48; mon++)
    {
        if (day >= mday[mon])
            day -= mday[mon];
        else
            break;
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
}

extern gtime_t epoch2time(const double* ep)
{
    const int doy[] = { 1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 };
    gtime_t time = { 0, 0 };
    int days, sec, year = static_cast<int>(ep[0]), mon = static_cast<int>(ep[1]), day = static_cast<int>(ep[2]);

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

    /* leap year if year%4==0 in 1901-2099 */
    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
    sec = static_cast<int>(floor(ep[5]));
    time.time = static_cast<time_t>(days) * 86400 + static_cast<int>(ep[3]) * 3600 + static_cast<int>(ep[4]) * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}

extern double time2doy(gtime_t t)
{
    double ep[6];

    time2epoch(t, ep);
    ep[1] = ep[2] = 1.0;
    ep[3] = ep[4] = ep[5] = 0.0;
    return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}

double interpc(const double coef[], double lat)
{
    auto i = static_cast<int>(lat / 15.0);
    if (i < 1)
    {
        return coef[0];
    }
    if (i > 4)
    {
        return coef[4];
    }
    return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
}
double mapf(double el, double a, double b, double c)
{
    double sinel = sin(el);
    return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
}

double nmf(gtime_t time, const double pos[], const double azel[], double* mapfw)
{
    /* ref [5] table 3 */
    /* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
    const double coef[][5] = {
        { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3 },
        { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3 },
        { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3 },

        { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5 },
        { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5 },
        { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5 },

        { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4 },
        { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3 },
        { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2 }
    };
    const double aht[] = { 2.53E-5, 5.49E-3, 1.14E-3 }; /* height correction */

    double y, cosy, ah[3], aw[3], dm, el = azel[1], lat = rad2deg(pos[0]), hgt = pos[2];
    int i;

    if (el <= 0.0)
    {
        if (mapfw) { *mapfw = 0.0; }
        return 0.0;
    }
    /* year from doy 28, added half a year for southern latitudes */
    y = (time2doy(time) - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);
    LOG_DATA("doy = {}, y = {}", time2doy(time), y);

    cosy = cos(2.0 * M_PI * y);
    lat = fabs(lat);

    for (i = 0; i < 3; i++)
    {
        ah[i] = interpc(coef[i], lat) - interpc(coef[i + 3], lat) * cosy;
        aw[i] = interpc(coef[i + 6], lat);
    }
    LOG_DATA("a_d = {}, b_d = {}, c_d = {}", ah[0], ah[1], ah[2]);

    /* ellipsoidal height is used instead of height above sea level */
    dm = (1.0 / sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;

    LOG_DATA("dm = {}", dm);

    if (mapfw) { *mapfw = mapf(el, aw[0], aw[1], aw[2]); }

    return mapf(el, ah[0], ah[1], ah[2]) + dm;
}

// NOLINTEND

TEST_CASE("[NMF] Compare with Rtklib code", "[NMF]")
{
    auto logger = initializeTestLogger();

    Eigen::Vector3d lla_pos(deg2rad(30.0), 0.0, 100.0);
    double elevation = deg2rad(30.0);
    InsTime epoch(2024, 10, 25, 10, 30, 0.0);

    double mf_h = calcTropoMapFunc_NMFH(epoch, lla_pos, elevation);
    double mf_w = calcTropoMapFunc_NMFW(lla_pos, elevation);

    auto unix = epoch.toUnixTime();
    auto unixDays = static_cast<time_t>(unix);
    gtime_t time{ .time = unixDays, .sec = static_cast<double>(unix - unixDays) };
    std::array<double, 2> azel{ 0.0, elevation };
    double mf_w_rtklib{};
    double mf_h_rtklib = nmf(time, lla_pos.data(), azel.data(), &mf_w_rtklib);

    REQUIRE_THAT(mf_h, Catch::Matchers::WithinAbs(mf_h_rtklib, 1e-6));
    REQUIRE_THAT(mf_w, Catch::Matchers::WithinAbs(mf_w_rtklib, 1e-6));
}

} // namespace NAV::TESTS::NMF