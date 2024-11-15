// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Allo223mA_22c.hpp
/// @brief Test Data for the file Allo223mA.22c
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-20

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

#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_02
{
/// @brief Test Data for the file Allo223mA.22n
const GnssNavInfo gnssNavInfo_Allo223mA_22c = {
    .satelliteSystems = { BDS },
    .ionosphericCorrections = {},
    .timeSysCorr = {
        { { BDT, UTC }, { .a0 = 0.000000000000e+00, .a1 = 0.000000000000e+00 } },
    },
    .m_satellites = {
        { { BDS, 16 }, Satellite{
                           // NOLINTBEGIN
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<BDSEphemeris>(16, 2022, 8, 11, 11, 0, 0, 0.142745906487e-03, -0.669686528454e-12, 0.000000000000e+00, //
                                                              0.100000000000e+01, 0.648437500000e+01, 0.100147028669e-08, -0.288849817208e+01,        //
                                                              0.499654561281e-06, 0.514506001491e-02, 0.210315920413e-04, 0.649293703079e+04,         //
                                                              0.385200000000e+06, 0.115018337965e-06, -0.244791475194e+01, 0.265426933765e-07,        //
                                                              0.961727879529e+00, -0.399843750000e+03, -0.231287269773e+01, -0.154685014680e-08,      //
                                                              0.142041630890e-08, 0.100000000000e+01, 0.866000000000e+03, 0.000000000000e+00,         //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.249999998481e-08, 0.460000000000e-08,        //
                                                              0.385200000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),        //
                               std::make_shared<BDSEphemeris>(16, 2022, 8, 11, 12, 0, 0, 0.142742996104e-03, -0.722089055216e-12, 0.000000000000e+00, //
                                                              0.100000000000e+01, 0.142500000000e+02, 0.960397147253e-09, -0.262555230514e+01,        //
                                                              0.752042979002e-06, 0.514246569946e-02, 0.221608206630e-04, 0.649294475555e+04,         //
                                                              0.388800000000e+06, 0.125262886286e-06, -0.244792046464e+01, 0.591389834881e-07,        //
                                                              0.961732958781e+00, -0.433843750000e+03, -0.231324274140e+01, -0.154827877774e-08,      //
                                                              0.143398830280e-08, 0.100000000000e+01, 0.866000000000e+03, 0.000000000000e+00,         //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.249999998481e-08, 0.460000000000e-08,        //
                                                              0.388800000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),        //
                           },
                           // NOLINTEND
                       } },
        { { BDS, 29 }, Satellite{
                           // NOLINTBEGIN
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<BDSEphemeris>(29, 2022, 8, 11, 12, 0, 0, 0.593908131123e-03, 0.501643171447e-11, 0.000000000000e+00, //
                                                              0.100000000000e+01, 0.176812500000e+03, 0.374265589664e-08, 0.151918447309e+01,        //
                                                              0.879168510437e-05, 0.293030170724e-03, 0.880751758814e-05, 0.528261849403e+04,        //
                                                              0.388800000000e+06, -0.405125319958e-07, 0.576864887801e+00, -0.270083546638e-07,      //
                                                              0.963738044541e+00, 0.184718750000e+03, 0.185136469510e+01, -0.675706717326e-08,       //
                                                              0.296798077117e-09, 0.100000000000e+01, 0.866000000000e+03, 0.000000000000e+00,        //
                                                              0.200000000000e+01, 0.000000000000e+00, -0.800000010681e-09, -0.800000000000e-09,      //
                                                              0.388800000000e+06, 0.100000000000e+01, 0.000000000000e+00, 0.000000000000e+00),       //
                           },
                           // NOLINTEND
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_02