// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INS_1580_19I.hpp
/// @brief Test Data for the file INS_1580.19I
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-23

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

namespace NAV::TESTS::RinexNavFileTests::v3_05
{
/// @brief Test Data for the file INS_1580.19I
const GnssNavInfo gnssNavInfo_INS_1580_19I = {
    .satelliteSystems = { BDS },
    .ionosphericCorrections = {},
    .timeSysCorr = {},
    .m_satellites = {
        { { BDS, 5 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<BDSEphemeris>(5, 2019, 6, 6, 22, 0, 0, -2.213711850345e-04, -4.911271389574e-11, 0.000000000000e+00, //
                                                             1.000000000000e+00, -4.562968750000e+02, -3.752656313198e-09, -1.273125246548e+00,     //
                                                             -1.499615609646e-05, 9.644716046751e-04, -1.047831028700e-05, 6.493460638046e+03,      //
                                                             4.248000000000e+05, -5.075708031654e-08, 2.769452268335e+00, 9.639188647270e-08,       //
                                                             1.108218186421e-01, 3.154375000000e+02, -9.116033205088e-01, 4.728768400756e-09,       //
                                                             3.957307694893e-10, 0.000000000000e+00, 7.000000000000e+02, 0.000000000000e+00,        //
                                                             2.000000000000e+00, 0.000000000000e+00, -4.000000000000e-10, -8.900000000000e-09,      //
                                                             4.248276000000e+05, 0.000000000000e+00),                                               //
                              std::make_shared<BDSEphemeris>(5, 2019, 6, 6, 23, 0, 0, -2.215489512309e-04, -4.924416430185e-11, 0.000000000000e+00, //
                                                             1.000000000000e+00, -5.378125000000e+02, -3.038340844806e-09, -1.010331733744e+00,     //
                                                             -1.783622428775e-05, 9.642329532653e-04, -3.765802830458e-06, 6.493460729599e+03,      //
                                                             4.284000000000e+05, -6.798654794693e-08, 2.974878815026e+00, 8.614733815193e-08,       //
                                                             1.117026372289e-01, 1.113593750000e+02, -1.117587550590e+00, 3.999452307528e-09,       //
                                                             4.628764235181e-10, 0.000000000000e+00, 7.000000000000e+02, 0.000000000000e+00,        //
                                                             2.000000000000e+00, 0.000000000000e+00, -4.000000000000e-10, -8.900000000000e-09,      //
                                                             4.284276000000e+05, 0.000000000000e+00),                                               //
                          },
                      } },
        { { BDS, 6 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<BDSEphemeris>(6, 2019, 6, 6, 20, 0, 0, 9.215852478519e-04, -3.255529179569e-11, 0.000000000000e+00, //
                                                             1.000000000000e+00, 7.567187500000e+01, 1.792931825664e-09, -1.855979805191e+00,      //
                                                             2.284999936819e-06, 8.097116136923e-03, 6.556510925293e-06, 6.493510314941e+03,       //
                                                             4.176000000000e+05, -1.396983861923e-07, -1.110316436656e+00, -9.778887033463e-09,    //
                                                             9.441736086663e-01, 3.709375000000e+01, -2.198623993492e+00, -2.235450258333e-09,     //
                                                             4.243033882249e-10, 0.000000000000e+00, 7.000000000000e+02, 0.000000000000e+00,       //
                                                             2.000000000000e+00, 0.000000000000e+00, 8.100000000000e-09, -1.800000000000e-09,      //
                                                             4.176180000000e+05, 0.000000000000e+00),                                              //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_05