// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Allo223mA_22n.hpp
/// @brief Test Data for the file Allo223mA.22n
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-16

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

namespace NAV::TESTS::RinexNavFileTests::v3_04
{
/// @brief Test Data for the file Allo223mA.22n
const GnssNavInfo gnssNavInfo_Allo223mA_22n = {
    .satelliteSystems = { GPS },
    .ionosphericCorrections = { {
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Alpha, .data = { 0.8382e-08, 0.2235e-07, -0.5960e-07, -0.1192e-06 } },
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Beta, .data = { 0.9216e+05, 0.1147e+06, -0.6554e+05, -0.5898e+06 } },
    } },
    .timeSysCorr = {
        { { GPST, UTC }, { -0.9313225746e-09, 0.000000000e+00 } },
    },
    .m_satellites = {
        { { GPS, 14 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GPSEphemeris>(2022, 8, 11, 12, 0, 0, -0.109632965177e-03, 0.136424205266e-11, 0.000000000000e+00, //
                                                              0.310000000000e+02, -0.260312500000e+02, 0.486877423256e-08, -0.379655971818e+00,   //
                                                              -0.130198895931e-05, 0.219930591993e-02, 0.840239226818e-05, 0.515365441513e+04,    //
                                                              0.388800000000e+06, 0.447034835815e-07, -0.829752086985e+00, 0.502914190292e-07,    //
                                                              0.951562004743e+00, 0.211468750000e+03, -0.305661533675e+01, -0.809640867649e-08,   //
                                                              -0.548594279725e-09, 0.100000000000e+01, 0.222200000000e+04, 0.000000000000e+00,    //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.791624188423e-08, 0.543000000000e+03,    //
                                                              0.381618000000e+06, 0.400000000000e+01, 0.000000000000e+00, 0.000000000000e+00),    //
                           },
                       } },
        { { GPS, 22 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GPSEphemeris>(2022, 8, 11, 12, 0, 0, 0.307971611619e-03, 0.659383658785e-11, 0.000000000000e+00, //
                                                              0.780000000000e+02, -0.598750000000e+02, 0.393516391537e-08, 0.306181691022e+01,   //
                                                              -0.294297933578e-05, 0.134400288807e-01, 0.122245401144e-04, 0.515373084068e+04,   //
                                                              0.388800000000e+06, 0.143423676491e-06, -0.287856453846e+01, -0.745058059692e-07,  //
                                                              0.962138004224e+00, 0.140906250000e+03, -0.184511752288e+01, -0.735102048522e-08,  //
                                                              0.460733477113e-09, 0.100000000000e+01, 0.222200000000e+04, 0.000000000000e+00,    //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.791624188423e-08, 0.780000000000e+02,   //
                                                              0.381618000000e+06, 0.400000000000e+01, 0.000000000000e+00, 0.000000000000e+00),   //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_04