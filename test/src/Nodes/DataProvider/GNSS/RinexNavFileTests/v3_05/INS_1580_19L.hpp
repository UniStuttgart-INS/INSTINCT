// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INS_1580_19L.hpp
/// @brief Test Data for the file INS_1580.19L
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

#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_05
{
/// @brief Test Data for the file INS_1580.19L
const GnssNavInfo gnssNavInfo_INS_1580_19L = {
    .satelliteSystems = { GAL },
    .ionosphericCorrections = { {
        { .satSys = GAL, .alphaBeta = IonosphericCorrections::Alpha, .data = { 2.8500e+01, 7.8125e-03, 1.1169e-02, 0.0000e+00 } },
    } },
    .timeSysCorr = {
        { { GST, GPST }, { 3.6961864680e-09, -1.332267630e-15 } },
        { { GST, UTC }, { -9.3132257462e-10, 0.000000000e+00 } },
    },
    .m_satellites = {
        { { GAL, 2 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GalileoEphemeris>(2019, 6, 6, 22, 40, 0, 6.366008892655e-05, 1.733724275255e-12, 0.000000000000e+00, //
                                                                 7.200000000000e+01, 1.047812500000e+02, 2.312596328920e-09, -7.626860954287e-01,   //
                                                                 4.980713129044e-06, 1.261847792193e-04, 1.181103289127e-05, 5.440612405777e+03,    //
                                                                 4.272000000000e+05, -5.401670932770e-08, 7.354717244466e-01, 1.247972249985e-07,   //
                                                                 9.880244040051e-01, 1.005625000000e+02, 1.196084944939e+00, -5.081997399876e-09,   //
                                                                 -1.957224383395e-10, 5.170000000000e+02, 2.056000000000e+03, 0.000000000000e+00,   //
                                                                 3.120000000000e+00, 0.000000000000e+00, -3.259629011154e-09, -4.190951585770e-09,  //
                                                                 4.278650000000e+05),                                                               //
                              std::make_shared<GalileoEphemeris>(2019, 6, 6, 22, 50, 0, 6.366113666445e-05, 1.733724275255e-12, 0.000000000000e+00, //
                                                                 7.300000000000e+01, 1.049375000000e+02, 2.306167489704e-09, -6.882583138871e-01,   //
                                                                 4.984438419342e-06, 1.262001460418e-04, 1.180917024612e-05, 5.440612436295e+03,    //
                                                                 4.278000000000e+05, -8.009374141693e-08, 7.354687283904e-01, 1.247972249985e-07,   //
                                                                 9.880243791355e-01, 1.005625000000e+02, 1.196042062421e+00, -5.072354141053e-09,   //
                                                                 -2.096515899731e-10, 5.170000000000e+02, 2.056000000000e+03, 0.000000000000e+00,   //
                                                                 3.120000000000e+00, 0.000000000000e+00, -3.259629011154e-09, -4.190951585770e-09,  //
                                                                 4.284650000000e+05),                                                               //
                          },
                      } },
        { { GAL, 3 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GalileoEphemeris>(2019, 6, 6, 22, 50, 0, -1.744942856021e-04, -4.149569576839e-12, 0.000000000000e+00, //
                                                                 7.300000000000e+01, 1.693750000000e+01, 3.498717164185e-09, -2.962133938551e+00,     //
                                                                 5.923211574554e-07, 3.684812691063e-04, 7.325783371925e-06, 5.440613746643e+03,      //
                                                                 4.278000000000e+05, -2.421438694000e-08, -1.357894583246e+00, -5.960464477539e-08,   //
                                                                 9.523798739645e-01, 1.783437500000e+02, -7.183772355696e-01, -5.607733584613e-09,    //
                                                                 3.593006806013e-10, 5.170000000000e+02, 2.056000000000e+03, 0.000000000000e+00,      //
                                                                 3.120000000000e+00, 0.000000000000e+00, 6.984919309616e-10, 9.313225746155e-10,      //
                                                                 4.284650000000e+05),                                                                 //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_05