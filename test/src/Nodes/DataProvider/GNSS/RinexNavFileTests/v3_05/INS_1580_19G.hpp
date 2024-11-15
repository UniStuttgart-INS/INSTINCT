// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INS_1580_19G.hpp
/// @brief Test Data for the file INS_1580.19G
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

#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_05
{
/// @brief Test Data for the file INS_1580.19G
const GnssNavInfo gnssNavInfo_INS_1580_19G = {
    .satelliteSystems = { GLO },
    .ionosphericCorrections = {},
    .timeSysCorr = {},
    .m_satellites = {
        { { GLO, 4 }, Satellite{
                          // NOLINTBEGIN
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2019, 6, 6, 21, 15, 0, 3.091366961598e-04, 9.094947017729e-13, 4.212000000000E+05, //
                                                                 -4.091441894531e+03, -1.242310523987e+00, 9.313225746155e-10, 0.000000000000E+00,  //
                                                                 1.900038037109e+04, 1.890688896179e+00, 9.313225746155e-10, 6.000000000000E+00,    //
                                                                 1.653417626953e+04, -2.484695434570e+00, -2.793967723846e-09, 0.000000000000E+00,  //
                                                                 1.830000000000e+02, -2.793967723846e-09, 3.000000000000e+00, 0.000000000000E+00),  //
                              std::make_shared<GLONASSEphemeris>(2019, 6, 6, 21, 45, 0, 3.091394901276e-04, 9.094947017729e-13, 4.230000000000E+05, //
                                                                 -5.767441894531e+03, -6.301259994507e-01, 9.313225746155e-10, 0.000000000000E+00,  //
                                                                 2.204026855469e+04, 1.442908287048e+00, 9.313225746155e-10, 6.000000000000E+00,    //
                                                                 1.148081054688e+04, -3.093800544739e+00, -2.793967723846e-09, 0.000000000000E+00,  //
                                                                 2.470000000000e+02, -2.793967723846e-09, 3.000000000000e+00, 0.000000000000E+00),  //
                          },
                          // NOLINTEND
                      } },
        { { GLO, 5 }, Satellite{
                          // NOLINTBEGIN
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2019, 6, 6, 23, 15, 0, 2.956017851830e-05, 9.094947017729e-13, 4.284000000000E+05, //
                                                                 2.064522460938e+03, 2.200222015381e-02, 1.862645149231e-09, 0.000000000000E+00,    //
                                                                 2.353465820312e+04, 1.324039459229e+00, 9.313225746155e-10, 1.000000000000E+00,    //
                                                                 9.612238769531e+03, -3.253516197205e+00, -2.793967723846e-09, 0.000000000000E+00,  //
                                                                 1.830000000000e+02, 2.793967723846e-09, 1.000000000000e+00, 0.000000000000E+00),   //
                          },
                          // NOLINTEND
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_05