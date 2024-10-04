// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INS_1581_19N.hpp
/// @brief Test Data for the file INS_1581.19N
/// @author Mikael Senger
/// @date 2024-06-28

#pragma once

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/GnssNavInfo.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v4_00
{
/// @brief Test Data for the file INS_1581.19N
const GnssNavInfo gnssNavInfo_INS_1581_19N = {
    .satelliteSystems = { GPS },
    .ionosphericCorrections = { {
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Alpha, .data = { 6.519258022308E-09, 2.235174179077E-08, -5.960464477539E-08, -1.192092895508E-07 } },
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Beta, .data = { 8.601600000000E+04, 9.830400000000E+04, -6.553600000000E+04, -5.242880000000E+05 } },
    } },
    .timeSysCorr = {
        { { GPST, UTC }, { -3.725290298462E-09, -7.105427357601E-15 } },
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
        { { GPS, 4 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2019, 6, 7, 2, 0, 0, 1.658042892814E-04, 2.160049916711E-12, 0.000000000000E+00,  //
                                                             1.850000000000E+02, 1.321875000000E+01, 4.916633368943E-09, -2.475223719598E+00,  //
                                                             6.407499313354E-07, 4.445955855772E-04, 6.224960088730E-06, 5.153733726501E+03,   //
                                                             4.392000000000E+05, -5.587935447693E-08, -9.863013308151E-01, 6.332993507385E-08, //
                                                             9.595218663600E-01, 2.577500000000E+02, -1.464285047475E+00, -8.134624554050E-09, //
                                                             1.075044779930E-10, 1.000000000000E+00, 2.056000000000E+03, 0.000000000000E+00,   //
                                                             2.048000000000E+03, 6.300000000000E+01, -8.847564458847E-09, 9.530000000000E+02,  //
                                                             4.320180000000E+05, 4.000000000000E+00),                                          //
                          },
                      } },
        { { GPS, 9 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2019, 6, 7, 2, 0, 0, 3.966777585447E-04, -7.503331289627E-12, 0.000000000000E+00,  //
                                                             8.000000000000E+01, 1.409375000000E+01, 5.058424989419E-09, 9.150000348870E-01,    //
                                                             6.146728992462E-07, 1.200041850097E-03, 6.297603249550E-06, 5.153656385422E+03,    //
                                                             4.392000000000E+05, -8.754432201385E-08, -1.031589827914E+00, -1.490116119385E-08, //
                                                             9.521370573588E-01, 2.529375000000E+02, 1.750483303867E+00, -8.212127782371E-09,   //
                                                             1.350056235261E-10, 1.000000000000E+00, 2.056000000000E+03, 0.000000000000E+00,    //
                                                             2.000000000000E+00, 0.000000000000E+00, 1.396983861923E-09, 8.000000000000E+01,    //
                                                             4.320180000000E+05, 4.000000000000E+00),                                           //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v4_00