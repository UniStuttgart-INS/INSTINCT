// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INS_1580_19N.hpp
/// @brief Test Data for the file INS_1580.19N
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

namespace NAV::TESTS::RinexNavFileTests::v3_05
{
/// @brief Test Data for the file INS_1580.19N
const GnssNavInfo gnssNavInfo_INS_1580_19N = {
    .satelliteSystems = { GPS },
    .ionosphericCorrections = { {
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Alpha, .data = { 6.5193e-09, 2.2352e-08, -5.9605e-08, -1.1921e-07 } },
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Beta, .data = { 8.6016e+04, 9.8304e+04, -6.5536e+04, -5.2429e+05 } },
    } },
    .timeSysCorr = {
        { { GPST, UTC }, { -3.7252902985e-09, -7.105427358e-15 } },
    },
    .m_satellites = {
        { { GPS, 1 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2019, 6, 6, 22, 0, 0, -3.462424501777e-05, -9.436007530894e-12, 0.000000000000e+00, //
                                                             4.100000000000e+01, -1.273437500000e+02, 4.150530029093e-09, 2.600635796634e+00,    //
                                                             -6.720423698425e-06, 8.921553497203e-03, 6.552785634995e-06, 5.153658721924e+03,    //
                                                             4.248000000000e+05, -1.285225152969e-07, -3.105805414428e+00, -1.434236764908e-07,  //
                                                             9.761313861903e-01, 2.615312500000e+02, 7.112214493196e-01, -7.808182384995e-09,    //
                                                             -1.592923494515e-10, 1.000000000000e+00, 2.056000000000e+03, 0.000000000000e+00,    //
                                                             2.000000000000e+00, 0.000000000000e+00, 5.587935447693e-09, 4.100000000000e+01,     //
                                                             4.176180000000e+05, 4.000000000000e+00),                                            //
                              std::make_shared<GPSEphemeris>(2019, 6, 7, 0, 0, 0, -3.469269722700e-05, -9.436007530894e-12, 0.000000000000e+00,  //
                                                             5.100000000000e+01, -1.248437500000e+02, 4.183031382905e-09, -2.632401611510e+00,   //
                                                             -6.632879376411e-06, 8.921077591367e-03, 6.230548024178e-06, 5.153659620285e+03,    //
                                                             4.320000000000e+05, 9.313225746155e-08, -3.105862471159e+00, -2.160668373108e-07,   //
                                                             9.761301953750e-01, 2.672187500000e+02, 7.112606730791e-01, -7.903900657759e-09,    //
                                                             -4.571618997710e-11, 1.000000000000e+00, 2.056000000000e+03, 0.000000000000e+00,    //
                                                             2.000000000000e+00, 0.000000000000e+00, 5.587935447693e-09, 5.100000000000e+01,     //
                                                             4.248180000000e+05, 4.000000000000e+00),                                            //
                          },
                      } },
        { { GPS, 2 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2019, 6, 6, 22, 0, 0, -2.312315627933e-04, -8.981260180008e-12, 0.000000000000e+00, //
                                                             6.600000000000e+01, -1.488750000000e+02, 4.714482091388e-09, 2.879261055706e+00,    //
                                                             -7.480382919312e-06, 1.889210427180e-02, 4.511326551437e-06, 5.153727025986e+03,    //
                                                             4.248000000000e+05, 2.533197402954e-07, 3.108457728793e+00, -1.937150955200e-07,    //
                                                             9.552165847649e-01, 2.853437500000e+02, -1.737103907156e+00, -8.016048186297e-09,   //
                                                             -2.964409193827e-10, 1.000000000000e+00, 2.056000000000e+03, 0.000000000000e+00,    //
                                                             2.000000000000e+00, 0.000000000000e+00, -2.048909664154e-08, 6.600000000000e+01,    //
                                                             4.240260000000e+05, 4.000000000000e+00),                                            //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_05