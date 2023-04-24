// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexNavFileTests.cpp
/// @brief Tests for the RinexNavFile node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-18

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "FlowTester.hpp"
#include "Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "GnssNavInfoComparisons.hpp"
#include "v2_01/brdc0990_22g.hpp"
#include "v2_10/bako3540_22n.hpp"
#include "v2_11/Allo060xA_22n.hpp"
#include "v2_11/Allo223mA_22g.hpp"
#include "v3_02/Allo223mA_22n.hpp"
#include "v3_02/Allo223mA_22l.hpp"
#include "v3_02/Allo223mA_22g.hpp"
#include "v3_02/Allo223mA_22c.hpp"
#include "v3_02/INSA11DEU_R_20223182100_01H_01S_EN_RNX.hpp"
#include "v3_04/Allo223mA_22n.hpp"
#include "v3_04/Allo223mA_22l.hpp"
#include "v3_04/Allo223mA_22g.hpp"
#include "v3_04/Allo223mA_22c.hpp"
#include "v3_04/INSA11DEU_R_20223181900_01H_01S_MN_RNX.hpp"
#include "v3_05/INS_1580_19N.hpp"
#include "v3_05/INS_1580_19L.hpp"
#include "v3_05/INS_1580_19G.hpp"
#include "v3_05/INS_1580_19I.hpp"
#include "v3_05/INS_1580_19P.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexNavFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::RinexNavFileTests
{

void testRinexNavFileFlow(const std::string& path, const GnssNavInfo& gnssNavInfoRef)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexNavFile*>(nm::FindNode(2))->_path = path;
    });

    nm::RegisterCleanupCallback([&]() {
        auto* pin = nm::FindOutputPin(1);
        REQUIRE(pin != nullptr);
        const auto* gnssNavInfo = static_cast<const GnssNavInfo*>(std::get<const void*>(pin->data));
        REQUIRE(*gnssNavInfo == gnssNavInfoRef);
    });

    // ###########################################################################################################
    //                                             RinexNavFile.flow
    // ###########################################################################################################
    //
    //  2 RinexNavFile
    //      1 GnssNavInfo <>
    //
    // ###########################################################################################################

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/RinexNavFile.flow"));
}

// ###########################################################################################################
//                                                   v2.01
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read v2_01/brdc0990.22g (GLONASS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v2_01/brdc0990.22g", v2_01::gnssNavInfo_brdc0990_22g);
}

// ###########################################################################################################
//                                                   v2.10
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read v2_10/bako3540.22n (GPS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v2_10/bako3540.22n", v2_10::gnssNavInfo_bako3540_22n);
}

// ###########################################################################################################
//                                                   v2.11
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read v2_11/Allo060xA.22n (GPS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v2_11/Allo060xA.22n", v2_11::gnssNavInfo_Allo060xA_22n);
}
TEST_CASE("[RinexNavFile] Read v2_11/Allo223mA.22g (GLONASS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v2_11/Allo223mA.22g", v2_11::gnssNavInfo_Allo223mA_22g);
}

// ###########################################################################################################
//                                                   v3.02
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read v3_02/Allo223mA.22n (GPS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_02/Allo223mA.22n", v3_02::gnssNavInfo_Allo223mA_22n);
}
TEST_CASE("[RinexNavFile] Read v3_02/Allo223mA.22l (Galileo)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_02/Allo223mA.22l", v3_02::gnssNavInfo_Allo223mA_22l);
}
TEST_CASE("[RinexNavFile] Read v3_02/Allo223mA.22g (GLONASS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_02/Allo223mA.22g", v3_02::gnssNavInfo_Allo223mA_22g);
}
TEST_CASE("[RinexNavFile] Read v3_02/Allo223mA.22c (BeiDou)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_02/Allo223mA.22c", v3_02::gnssNavInfo_Allo223mA_22c);
}
// TEST_CASE("[RinexNavFile] Read v3_02/Allo223mA.22h (SBAS)", "[RinexNavFile][flow]") // TODO: Add RinexNavFile SBAS v3.02 tests
// {
//     testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_02/Allo223mA.22h", v3_02::gnssNavInfo_Allo223mA_22h);
// }
TEST_CASE("[RinexNavFile] Read v3_02/INSA11DEU_R_20223182100_01H_01S_EN.rnx (Galileo)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_02/INSA11DEU_R_20223182100_01H_01S_EN.rnx", v3_02::gnssNavInfo_INSA11DEU_R_20223182100_01H_01S_EN_RNX);
}

// ###########################################################################################################
//                                                   v3.03
// ###########################################################################################################

// TODO: Add RinexNavFile v3.03 tests

// ###########################################################################################################
//                                                   v3.04
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read v3_04/Allo223mA.22n (GPS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/Allo223mA.22n", v3_04::gnssNavInfo_Allo223mA_22n);
}
TEST_CASE("[RinexNavFile] Read v3_04/Allo223mA.22l (Galileo)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/Allo223mA.22l", v3_04::gnssNavInfo_Allo223mA_22l);
}
TEST_CASE("[RinexNavFile] Read v3_04/Allo223mA.22g (GLONASS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/Allo223mA.22g", v3_04::gnssNavInfo_Allo223mA_22g);
}
TEST_CASE("[RinexNavFile] Read v3_04/Allo223mA.22c (BeiDou)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/Allo223mA.22c", v3_04::gnssNavInfo_Allo223mA_22c);
}
// TEST_CASE("[RinexNavFile] Read v3_04/ANK200TUR_S_20223570000_01D_JN.rnx (QZSS)", "[RinexNavFile][flow]")
// {
//     testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/ANK200TUR_S_20223570000_01D_JN.rnx", v3_04::gnssNavInfo_ANK200TUR_S_20223570000_01D_JN_RNX); // TODO: Add RinexNavFile QZSS v3.04 tests
// }
// TEST_CASE("[RinexNavFile] Read v3_04/Allo223mA.22h (SBAS)", "[RinexNavFile][flow]") // TODO: Add RinexNavFile SBAS v3.04 tests
// {
//     testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/Allo223mA.22h", v3_04::gnssNavInfo_Allo223mA_22h);
// }
TEST_CASE("[RinexNavFile] Read v3_04/INSA11DEU_R_20223181900_01H_01S_MN.rnx (Mixed)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_04/INSA11DEU_R_20223181900_01H_01S_MN.rnx", v3_04::gnssNavInfo_INSA11DEU_R_20223181900_01H_01S_MN_RNX);
}

// ###########################################################################################################
//                                                   v3.05
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read v3_05/INS_1580.19N (GPS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_05/INS_1580.19N", v3_05::gnssNavInfo_INS_1580_19N);
}
TEST_CASE("[RinexNavFile] Read v3_05/INS_1580.19L (Galileo)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_05/INS_1580.19L", v3_05::gnssNavInfo_INS_1580_19L);
}
TEST_CASE("[RinexNavFile] Read v3_05/INS_1580.19G (GLONASS)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_05/INS_1580.19G", v3_05::gnssNavInfo_INS_1580_19G);
}
TEST_CASE("[RinexNavFile] Read v3_05/INS_1580.19I (BeiDou)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_05/INS_1580.19I", v3_05::gnssNavInfo_INS_1580_19I);
}
TEST_CASE("[RinexNavFile] Read v3_05/INS_1580.19P (Mixed)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/v3_05/INS_1580.19P", v3_05::gnssNavInfo_INS_1580_19P); // TODO: Enable SBAS + QZSS satellites
}

// ###########################################################################################################
//                                                   v4.00
// ###########################################################################################################

// TODO: After supporting RINEX 4.0, Add RinexNavFile tests

// ###########################################################################################################
//                                               Corrupt files
// ###########################################################################################################

TEST_CASE("[RinexNavFile] Read corrupt/LAMP00ITA_R_20223570000_01D_MN_1.rnx (Mixed, damaged file)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/corrupt/LAMP00ITA_R_20223570000_01D_MN_1.rnx", {});
}
TEST_CASE("[RinexNavFile] Read corrupt/LAMP00ITA_R_20223570000_01D_MN_2.rnx (Mixed, damaged file)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/corrupt/LAMP00ITA_R_20223570000_01D_MN_2.rnx", {});
}
TEST_CASE("[RinexNavFile] Read corrupt/LAMP00ITA_R_20223570000_01D_MN_3.rnx (Mixed, damaged file)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/corrupt/LAMP00ITA_R_20223570000_01D_MN_3.rnx", {});
}
TEST_CASE("[RinexNavFile] Read corrupt/LAMP00ITA_R_20223570000_01D_MN_4.rnx (Mixed, damaged file)", "[RinexNavFile][flow]")
{
    testRinexNavFileFlow("DataProvider/GNSS/RinexNavFile/corrupt/LAMP00ITA_R_20223570000_01D_MN_4.rnx", {});
}

} // namespace NAV::TESTS::RinexNavFileTests
