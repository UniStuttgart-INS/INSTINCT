// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UbloxGnssOrbitCollectorTests.cpp
/// @brief Tests for the UbloxGnssOrbitCollector node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-12

#include <catch2/catch_test_macros.hpp>

#include "FlowTester.hpp"
#include "Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "NodeData/GNSS/GnssNavInfoComparisons.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/UbloxFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RinexNavFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::UbloxGnssOrbitCollectorTests
{

TEST_CASE("[UbloxGnssOrbitCollectorTests][flow] Spirent_ublox-F9P_static_duration-15min_sys-GPS-GAL_iono-Klobuchar_tropo-Saastamoinen.ubx", "[UbloxGnssOrbitCollectorTests][flow]")
{
    auto logger = initializeTestLogger();

    // ###########################################################################################################
    //                                       UbloxGnssOrbitCollector.flow
    // ###########################################################################################################
    //
    // UbloxFile (2)                    UbloxGnssOrbitCollector (5)
    //  (1) UbloxObs |>  --(6)-->  |> UbloxObs (3)  (4) GnssNavInfo <>
    constexpr size_t NODE_ID_UBLOX_FILE = 2;
    constexpr size_t PIN_ID_UBLOX_RINEX_NAV_INFO = 4;
    //
    // RinexNavFile (8)
    //   (7) GnssNavInfo |>
    constexpr size_t NODE_ID_RINEX_NAV_FILE = 8;
    constexpr size_t PIN_ID_RINEX_NAV_INFO = 7;
    //
    // ###########################################################################################################

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<UbloxFile*>(nm::FindNode(NODE_ID_UBLOX_FILE))->_path = "Converter/GNSS/Ublox/Spirent_ublox-F9P_static_duration-15min_sys-GPS-GAL_iono-Klobuchar_tropo-Saastamoinen.ubx";
        dynamic_cast<RinexNavFile*>(nm::FindNode(NODE_ID_RINEX_NAV_FILE))->_path = "Converter/GNSS/Ublox/Spirent_ublox-F9P_static_duration-15min_sys-GPS-GAL_iono-Klobuchar_tropo-Saastamoinen.nav";
    });

    nm::RegisterCleanupCallback([&]() {
        auto* ubloxCollectorPin = nm::FindOutputPin(PIN_ID_UBLOX_RINEX_NAV_INFO);
        REQUIRE(ubloxCollectorPin != nullptr);
        auto* rinexNavFilePin = nm::FindOutputPin(PIN_ID_RINEX_NAV_INFO);
        REQUIRE(rinexNavFilePin != nullptr);
        const auto* gnssNavInfoUblox = static_cast<const GnssNavInfo*>(std::get<const void*>(ubloxCollectorPin->data));
        const auto* gnssNavInfoRinex = static_cast<const GnssNavInfo*>(std::get<const void*>(rinexNavFilePin->data));

        REQUIRE(*gnssNavInfoUblox == *gnssNavInfoRinex);
    });

    REQUIRE(testFlow("test/flow/Nodes/Converter/GNSS/UbloxGnssOrbitCollector.flow"));
}

} // namespace NAV::TESTS::UbloxGnssOrbitCollectorTests