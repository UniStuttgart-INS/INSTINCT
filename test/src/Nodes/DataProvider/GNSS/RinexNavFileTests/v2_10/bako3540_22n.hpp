// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file bako3540_22n.hpp
/// @brief Test Data for the file bako3540.22n
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-23

#pragma once

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/GnssNavInfo.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v2_10
{
/// @brief Test Data for the file bako3540.22n
const GnssNavInfo gnssNavInfo_bako3540_22n = {
    .satelliteSystems = { GPS },
    .ionosphericCorrections = { {
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Alpha, .data = { 2.1420e-08, 0.0000e+00, -5.9605e-08, 1.1921e-07 } },
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Beta, .data = { 1.4336e+05, -2.1299e+05, 0.0000e+00, 2.6214e+05 } },
    } },
    .timeSysCorr = {
        { { GPST, UTC }, { 2.793967723846e-09, 2.664535259100e-15 } },
    },
    .m_satellites = {
        { { GPS, 1 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2022, 12, 19, 14, 0, 0.0, 2.356213517487e-04, -5.115907697473e-12, 0.000000000000e+00, //
                                                             3.200000000000e+01, -9.478125000000e+01, 3.792300695693e-09, 4.271765015474e-01,       //
                                                             -4.975125193596e-06, 1.212759001646e-02, 8.169561624527e-06, 5.153656442642e+03,       //
                                                             1.368000000000e+05, -1.341104507446e-07, -1.134383541274e+00, 1.471489667892e-07,      //
                                                             9.890701622166e-01, 2.403125000000e+02, 9.383766377821e-01, -7.908186994143e-09,       //
                                                             -1.564350871064e-10, 1.000000000000e+00, 2.241000000000e+03, 0.000000000000e+00,       //
                                                             2.000000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 3.200000000000e+01,        //
                                                             1.295400000000e+05, 0.000000000000e+00),                                               //
                          },
                      } },
        { { GPS, 2 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2022, 12, 20, 0, 0, 0.0, -6.321193650365e-04, 2.046363078989e-12, 0.000000000000e+00, //
                                                             6.600000000000e+01, -1.198437500000e+02, 4.350181104229e-09, -1.560460888100e+00,     //
                                                             -5.939975380898e-06, 2.007885510102e-02, 8.083879947662e-06, 5.153694664001e+03,      //
                                                             1.728000000000e+05, -2.160668373108e-07, -1.230261401577e+00, 4.712492227554e-07,     //
                                                             9.667863298684e-01, 2.262500000000e+02, -1.351785634840e+00, -8.134624529532e-09,     //
                                                             -3.257278613500e-10, 1.000000000000e+00, 2.241000000000e+03, 0.000000000000e+00,      //
                                                             2.000000000000e+00, 0.000000000000e+00, -1.769512891769e-08, 6.600000000000e+01,      //
                                                             1.655400000000e+05, 0.000000000000e+00),                                              //
                              std::make_shared<GPSEphemeris>(2022, 12, 20, 2, 0, 0.0, -6.321044638753e-04, 2.046363078989e-12, 0.000000000000e+00, //
                                                             6.700000000000e+01, -1.075937500000e+02, 4.408398091016e-09, -5.102199460116e-01,     //
                                                             -5.666166543961e-06, 2.007965464145e-02, 8.136034011841e-06, 5.153697889328e+03,      //
                                                             1.800000000000e+05, -2.402812242508e-07, -1.230322997743e+00, -3.725290298462e-09,    //
                                                             9.667834347535e-01, 2.282187500000e+02, -1.351859643866e+00, -8.489639213849e-09,     //
                                                             -1.517920372729e-10, 1.000000000000e+00, 2.241000000000e+03, 0.000000000000e+00,      //
                                                             2.000000000000e+00, 0.000000000000e+00, -1.769512891769e-08, 6.700000000000e+01,      //
                                                             1.727400000000e+05, 0.000000000000e+00),                                              //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v2_10