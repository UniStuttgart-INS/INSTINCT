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
    //  2 RinexNavFile("Skydel-NavMsg/SkydelRINEX_S_20221521359_7200S_GN.rnx")
    //     |>  1 GnssNavInfo
    //  4 RinexNavFile("Skydel-NavMsg/SkydelRINEX_S_20221521359_600S_EN.rnx")
    //     |>  5 GnssNavInfo
    //  7 RinexNavFile("Skydel-NavMsg/SkydelRINEX_S_20221521359_1800S_RN.rnx")
    //     |>  6 GnssNavInfo
    // 13 RinexNavFile("Skydel-NavMsg/SkydelRINEX_S_20221521359_120S_SN.rnx")
    //     |> 12 GnssNavInfo
    //
    // ###########################################################################################################

    nm::RegisterCleanupCallback([]() {
        auto* gpsPin = nm::FindPin(1);
        REQUIRE(gpsPin != nullptr);

        auto* gnssNavInfo_GPS = static_cast<GnssNavInfo*>(std::get<void*>(gpsPin->data));

        REQUIRE(gnssNavInfo_GPS->broadcastEphemeris.size() == 32);
        for (const auto& ephOfSat : gnssNavInfo_GPS->broadcastEphemeris)
        {
            REQUIRE(ephOfSat.second.size() == 2);
        }

        // TODO: Write the actual test here
    });

    testFlow("test/flow/Nodes/DataProvider/GNSS/RinexNavFile.flow");
}

} // namespace NAV::TEST::RinexNavFileTests