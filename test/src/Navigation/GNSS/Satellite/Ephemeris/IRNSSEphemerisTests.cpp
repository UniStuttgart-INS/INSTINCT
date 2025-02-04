// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Common.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/IRNSSEphemeris.hpp"

namespace NAV::TESTS::EphemerisTests
{

// TODO Test real data. Therefore get orbits for real data, since there is no data for IRNSS in the sp3 files

TEST_CASE("[Ephemeris] IRNSS Ephemeris calc orbit (Skydel data) IGSO Satellite", "[Ephemeris]")
{
    // I02 - Generated from Skydel
    // file: test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20238959_4800S_IN.rnx
    // NOLINTBEGIN
    IRNSSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.506004166207e-04, -2.626165951369e-11, 0.000000000000e+00,
                       1.000000000000e+00, -4.023750000000e+02, 4.141601085738e-09, -1.732777692392e+00,
                       -1.328811049461e-05, 1.831341884099e-03, 1.068785786629e-05, 6.493304498672e+03,
                       4.320000000000e+04, 1.527369022369e-07, 2.697144041504e+00, 5.587935447693e-08,
                       5.091938487106e-01, -2.446250000000e+02, -3.135464476589e+00, -3.623008055685e-09,
                       -1.027542801282e-09, 2.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                       2.000000000000e+00, 0.000000000000e+00, -1.862645149231e-09, 0.000000000000e+00,
                       9.999000000000e+08, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 5.1e-09; // file has only 16 digits after comma
    margin.pos = 1.9e-05;

    testEphemerisData({ IRNSS, 2 }, eph,
                      "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/NAVICL5 02.csv",
                      I05, Skydel, margin);
}

TEST_CASE("[Ephemeris] IRNSS Ephemeris calc orbit (Skydel data) GEO Satellite", "[Ephemeris]")
{
    // I03 - Generated from Skydel
    // file: test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20238959_4800S_IN.rnx
    // NOLINTBEGIN
    IRNSSEphemeris eph(2023, 1, 8, 12, 0, 0, -4.790757011506e-04, -3.058175934711e-11, 0.000000000000e+00,
                       1.000000000000e+00, -1.418750000000e+02, 1.258623855307e-09, -2.085582523402e+00,
                       -4.727393388748e-06, 1.656766631640e-03, 2.516806125641e-05, 6.493450317383e+03,
                       4.320000000000e+04, -1.341104507446e-07, 2.264411573488e-01, 2.197921276093e-07,
                       5.962630550353e-02, -7.704375000000e+02, 1.759525656106e-01, -2.114373786441e-10,
                       8.996803324399e-10, 2.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                       2.000000000000e+00, 0.000000000000e+00, -1.396983861923e-09, 0.000000000000e+00,
                       9.999000000000e+08, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 3.8e-09; // file has only 16 digits after comma
    margin.pos = 5.4e-06;

    testEphemerisData({ IRNSS, 3 }, eph,
                      "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/NAVICL5 03.csv",
                      I05, Skydel, margin);
}

} // namespace NAV::TESTS::EphemerisTests