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

namespace NAV::TESTS::RinexNavFileTests::v3_04
{
/// @brief Test Data for the file Allo223mA.22n
const GnssNavInfo gnssNavInfo_Allo223mA_22c = {
    .satelliteSystems = { BDS },
    .ionosphericCorrections = { {
        { .satSys = BDS, .alphaBeta = IonosphericCorrections::Alpha, .data = { 0.1956e-07, 0.1043e-06, -0.1073e-05, 0.1669e-05 } },
        { .satSys = BDS, .alphaBeta = IonosphericCorrections::Beta, .data = { 0.1475e+06, -0.6881e+06, 0.4588e+07, -0.3408e+07 } },
    } },
    .timeSysCorr = {
        { { BDT, UTC }, { 0.000000000000e+00, 0.000000000000e+00 } },
    },
    .m_satellites = {
        { { BDS, 16 }, Satellite{
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
                       } },
        { { BDS, 39 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<BDSEphemeris>(39, 2022, 8, 11, 11, 0, 0, -0.201049260795e-06, -0.116351372981e-12, 0.000000000000e+00, //
                                                              0.100000000000e+01, -0.385000000000e+02, 0.877536552920e-09, -0.205606119447e+01,        //
                                                              -0.101840123534e-05, 0.256746052764e-02, 0.231186859310e-04, 0.649307484245e+04,         //
                                                              0.385200000000e+06, 0.968575477600e-07, -0.251789464664e+01, 0.633299350739e-07,         //
                                                              0.962462211740e+00, -0.459281250000e+03, -0.296288418585e+01, -0.148934775160e-08,       //
                                                              0.140362989539e-08, 0.100000000000e+01, 0.866000000000e+03, 0.000000000000e+00,          //
                                                              0.200000000000e+01, 0.000000000000e+00, 0.849999981511e-08, 0.850000000000e-08,          //
                                                              0.385200000000e+06, 0.100000000000e+01, 0.000000000000e+00, 0.000000000000e+00),         //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_04