#include <catch2/catch.hpp>

#include "FlowTester.hpp"
#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "NodeData/GNSS/GnssNavInfo.hpp"

namespace NAV::TEST::RinexNavFileTests
{

TEST_CASE("[RinexNavFile] Read v3.03 Files and check correctness", "[RinexNavFile][flow]")
{
    Logger logger;

    // ###########################################################################################################
    //                                             RinexNavFile.flow
    // ###########################################################################################################
    //
    //  2 RinexNavFile("Skydel-static_4h_1min-rate/SkydelRINEX_S_2022152120_7200S_GN.rnx")
    //     |>  1 GnssNavInfo
    //  4 RinexNavFile("Skydel-static_4h_1min-rate/SkydelRINEX_S_2022152120_600S_EN")
    //     |>  5 GnssNavInfo
    //  7 RinexNavFile("Skydel-static_4h_1min-rate/SkydelRINEX_S_2022152120_1800S_RN.rnx")
    //     |>  6 GnssNavInfo
    // 13 RinexNavFile("Skydel-static_4h_1min-rate/SkydelRINEX_S_2022152120_120S_SN")
    //     |> 12 GnssNavInfo
    //
    // ###########################################################################################################

    nm::RegisterCleanupCallback([]() {
        auto* gpsPin = nm::FindOutputPin(1);
        REQUIRE(gpsPin != nullptr);

        auto* gnssNavInfo_GPS = static_cast<GnssNavInfo*>(std::get<void*>(gpsPin->dataOld));

        REQUIRE(gnssNavInfo_GPS->broadcastEphemeris.size() == 32);
        for (const auto& ephOfSat : gnssNavInfo_GPS->broadcastEphemeris)
        {
            REQUIRE(ephOfSat.second.size() == 3);
        }

        // TODO: Write the actual test here
    });

    testFlow("test/flow/Nodes/DataProvider/GNSS/RinexNavFile.flow");
}

} // namespace NAV::TEST::RinexNavFileTests