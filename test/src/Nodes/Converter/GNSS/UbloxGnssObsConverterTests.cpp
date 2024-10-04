// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UbloxGnssObsConverterTests.cpp
/// @brief Tests for the UbloxGnssObsConverter node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-08

#include <catch2/catch_test_macros.hpp>

#include "FlowTester.hpp"
#include "Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssObsComparisons.hpp"

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/UbloxFile.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::UbloxGnssObsConverterTests
{

void compareObservations(std::deque<std::shared_ptr<const NAV::GnssObs>>& data1, std::deque<std::shared_ptr<const NAV::GnssObs>>& data2)
{
    if (data1.empty() || data2.empty()) { return; }

    LOG_DEBUG("UbloxConverter [{}]", data1.front()->insTime.toYMDHMS(GPST));
    for ([[maybe_unused]] const auto& obsData : data1.front()->data)
    {
        LOG_DEBUG("  [{}]", obsData.satSigId);
    }
    LOG_DEBUG("RinexObsFile [{}]", data2.front()->insTime.toYMDHMS(GPST));
    for ([[maybe_unused]] const auto& obsData : data2.front()->data)
    {
        LOG_DEBUG("  [{}]", obsData.satSigId);
    }
    REQUIRE(*data1.front() == *data2.front());

    data1.pop_front();
    data2.pop_front();
}

TEST_CASE("[UbloxGnssObsConverterTests][flow] Spirent_ublox-F9P_static_duration-15min_sys-GPS-GAL_iono-Klobuchar_tropo-Saastamoinen.ubx", "[UbloxGnssObsConverterTests][flow]")
{
    auto logger = initializeTestLogger();

    // ###########################################################################################################
    //                                         UbloxGnssObsConverter.flow
    // ###########################################################################################################
    //
    // UbloxFile (2)                  UbloxGnssObsConverter (5)
    //  (1) UbloxObs |>  --(6)-->  |> UbloxObs (3)  (4) GnssObs |>  --(6)-->  |> (7) Terminator (8)
    constexpr size_t NODE_ID_UBLOX_FILE = 2;
    constexpr size_t PIN_ID_UBLOX_TERMINATOR = 7;
    //
    // RinexObsFile (19)
    //   (18) GnssObs |>  --(22)-->  |> (20) Terminator (21)
    constexpr size_t NODE_ID_RINEX_OBS_FILE = 19;
    constexpr size_t PIN_ID_RINEX_TERMINATOR = 20;
    //
    // ###########################################################################################################

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<UbloxFile*>(nm::FindNode(NODE_ID_UBLOX_FILE))->_path = "Converter/GNSS/Ublox/Spirent_ublox-F9P_static_duration-15min_sys-GPS-GAL_iono-Klobuchar_tropo-Saastamoinen.ubx";
        dynamic_cast<RinexObsFile*>(nm::FindNode(NODE_ID_RINEX_OBS_FILE))->_path = "Converter/GNSS/Ublox/Spirent_ublox-F9P_static_duration-15min_sys-GPS-GAL_iono-Klobuchar_tropo-Saastamoinen.obs";
    });

    std::atomic<size_t> messageCounterTerminator1 = 0;
    std::atomic<size_t> messageCounterTerminator2 = 0;
    std::deque<std::shared_ptr<const NAV::GnssObs>> data1;
    std::deque<std::shared_ptr<const NAV::GnssObs>> data2;
    std::mutex comparisonMutex;

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_UBLOX_TERMINATOR, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterTerminator1++;

        data1.push_back(std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()));

        std::scoped_lock lk(comparisonMutex);
        compareObservations(data1, data2);
    });
    std::shared_ptr<const NAV::GnssObs> lastObs = nullptr;
    nm::RegisterWatcherCallbackToInputPin(PIN_ID_RINEX_TERMINATOR, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterTerminator2++;

        // Make a copy, because we need to modify the data
        auto gnssObs = std::make_shared<NAV::GnssObs>(*std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()));

        for (auto& obsData : gnssObs->data)
        {
            if (lastObs == nullptr) { break; }
            auto last = std::find_if(lastObs->data.begin(), lastObs->data.end(),
                                     [&obsData = obsData](const auto& data) { return obsData.satSigId == data.satSigId; });
            if (last == lastObs->data.end()) { continue; }

            // Signal [G1C-13] at epoch [2023-1-8 9:55:33.004] has the LLI flag set to [0] by Rtklib
            // The ublox data do not give reason to set this flag there.
            //                                 LLI
            //                                  |
            // > 2023 01 08 09 55 30.004        |
            // G13  26244973.569   137918262.0872       2377.292          43.000
            //      prValid true, cpValid true, halfCycValid false, subHalfSubtractedFromPhase false, trkStat 00000011, observedLastEpoch true
            // > 2023 01 08 09 55 31.004        |
            // G13  26244521.628   137915884.9942       2376.833          43.000
            //      prValid true, cpValid true, halfCycValid false, subHalfSubtractedFromPhase false, trkStat 00000011, observedLastEpoch true
            // > 2023 01 08 09 55 32.004        |
            // G13  26244069.081   137913507.7781       2376.553          43.000
            //      prValid true, cpValid true, halfCycValid true, subHalfSubtractedFromPhase true, trkStat 00001111, observedLastEpoch true
            // > 2023 01 08 09 55 33.004        |
            // G13  26243616.667   137911131.472        2376.072          43.000
            //      prValid true, cpValid true, halfCycValid true, subHalfSubtractedFromPhase true, trkStat 00001111, observedLastEpoch true
            if (last->carrierPhase && last->carrierPhase->LLI == 2
                && obsData.carrierPhase && obsData.carrierPhase->LLI == 1)
            {
                obsData.carrierPhase->LLI = 0;
            }

            // Signal [G1C-09] at epoch [2023-1-8 9:57:58.004] has the LLI flag set to [3] by Rtklib
            // In the epoch before, the signal was observed with LLI flag [2]. So it does not make sense to set it to [3] now.
            //                                 LLI
            //                                  |
            // > 2023 01 08 09 57 57.004        |
            // G09  24630808.015   129435769.1322      -3284.171          43.000    24630789.560    96656507.708       -2452.655          47.000
            // > 2023 01 08 09 57 58.004        |
            // G09  24631433.195   129439054.6733      -3284.665          43.000    24631414.268    96658960.445       -2452.765          47.000
            // > 2023 01 08 09 57 59.004        |
            // G09  24632058.989   129442343.3513      -3284.669          43.000    24632039.739    96661413.288       -2452.832          47.000
            // > 2023 01 08 09 58 00.004        |
            // G09  24632683.857   129445627.1822      -3285.114          43.000    24632664.586    96663866.228       -2453.071          47.000
            if (last->carrierPhase && last->carrierPhase->LLI == 2
                && obsData.carrierPhase && obsData.carrierPhase->LLI == 3)
            {
                obsData.carrierPhase->LLI = 2;
            }
        }

        data2.push_back(gnssObs);
        lastObs = gnssObs;

        std::scoped_lock lk(comparisonMutex);
        compareObservations(data1, data2);
    });

    REQUIRE(testFlow("test/flow/Nodes/Converter/GNSS/UbloxGnssObsConverter.flow"));

    REQUIRE(messageCounterTerminator1 == messageCounterTerminator2);
    REQUIRE(data1.empty());
    REQUIRE(data2.empty());
    REQUIRE(messageCounterTerminator1 == 931);
}

} // namespace NAV::TESTS::UbloxGnssObsConverterTests