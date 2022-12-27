// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Allo060xA_22n.hpp
/// @brief Test Data for the file Allo060xA.22n
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

namespace NAV::TESTS::RinexNavFileTests::v2_11
{
/// @brief Test Data for the file Allo060xA.22n
const GnssNavInfo gnssNavInfo_Allo060xA_22n = {
    .satelliteSystems = { GPS },
    .ionosphericCorrections = { {
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Alpha, .data = { 0.1397e-07, 0.0000e+00, -0.5960e-07, 0.5960e-07 } },
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Beta, .data = { 0.1106e+06, -0.3277e+05, -0.2621e+06, 0.1966e+06 } },
    } },
    .timeSysCorr = {
        { { GPST, UTC }, { 0.465661287308e-08, 0.710542735760e-14 } },
    },
    .m_satellites = {
        { { GPS, 1 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2022, 3, 2, 0, 0, 0.0, 0.419155228883e-03, -0.932232069317e-11, 0.000000000000e+00, //
                                                             0.230000000000e+02, 0.110718750000e+03, 0.418696011798e-08, 0.152290926733e+01,     //
                                                             0.573694705963e-05, 0.114660460968e-01, 0.510737299919e-05, 0.515367102432e+04,     //
                                                             0.259200000000e+06, -0.303611159325e-06, -0.216191106951e+01, 0.106170773506e-06,   //
                                                             0.986927421717e+00, 0.297062500000e+03, 0.883506194036e+00, -0.819176979152e-08,    //
                                                             0.328942273195e-09, 0.100000000000e+01, 0.219900000000e+04, 0.000000000000e+00,     //
                                                             0.200000000000e+01, 0.000000000000e+00, 0.512227416039e-08, 0.230000000000e+02,     //
                                                             0.252018000000e+06, 0.400000000000e+01, 0.000000000000e+00, 0.000000000000e+00),    //
                          },
                      } },
        { { GPS, 31 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GPSEphemeris>(2022, 3, 1, 23, 59, 44.0, -0.167020596564e-03, -0.170530256582e-11, 0.000000000000e+00, //
                                                              0.200000000000e+01, 0.460937500000e+02, 0.485163066132e-08, 0.395455163769e+00,         //
                                                              0.257603824139e-05, 0.104426422622e-01, 0.735186040401e-05, 0.515369047928e+04,         //
                                                              0.259184000000e+06, 0.106170773506e-06, 0.991345722511e+00, 0.152736902237e-06,         //
                                                              0.955083176877e+00, 0.234312500000e+03, 0.379175433950e+00, -0.830963184381e-08,        //
                                                              0.655384442250e-09, 0.100000000000e+01, 0.219900000000e+04, 0.000000000000e+00,         //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.135041773319e-07, 0.200000000000e+01,        //
                                                              0.254286000000e+06, 0.400000000000e+01, 0.000000000000e+00, 0.000000000000e+00),        //
                           },
                       } },
        { { GPS, 14 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GPSEphemeris>(2022, 3, 2, 0, 0, 0.0, -0.895131379366e-04, -0.409272615798e-11, 0.000000000000e+00, //
                                                              0.820000000000e+02, -0.481875000000e+02, 0.508699760815e-08, 0.268765011279e+00,     //
                                                              -0.249408185482e-05, 0.148966396227e-02, 0.592693686485e-05, 0.515364431763e+04,     //
                                                              0.259200000000e+06, -0.260770320892e-07, 0.205491203428e+01, 0.465661287308e-07,     //
                                                              0.953238573230e+00, 0.261500000000e+03, 0.302066767985e+01, -0.835177645644e-08,     //
                                                              -0.447161483213e-09, 0.100000000000e+01, 0.219900000000e+04, 0.000000000000e+00,     //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.745058059692e-08, 0.338000000000e+03,     //
                                                              0.252018000000e+06, 0.400000000000e+01, 0.000000000000e+00, 0.000000000000e+00),     //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v2_11