// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexObsFileTests.cpp
/// @brief RinexObsFile unit test
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-11-11

#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <array>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

#include "NodeData/State/PosVel.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RINEX/RinexObsFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TEST::RinexObsFileTests
{

constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

constexpr double Gps_LeapSec = 18;

enum RinexTimeRef : size_t
{
    RINEX_Year,
    RINEX_Month,
    RINEX_Day,
    RINEX_Hour,
    RINEX_Minute,
    RINEX_Second,
    RINEX_EpochFlag,
    RINEX_NumSats,
    RINEX_reserved,     // a one-digit flag that is not specified in the standard
    RINEX_RcvClkOffset, // optional
};

constexpr std::array<double, 8> RINEX_REFERENCE_EPOCH = { 2022, 11, 2, 16, 39, 59.6920000, 0, 39 };

enum RinexRef : size_t
{
    RINEX_SatNum,
    RINEX_Obs_Pseudorange,
    RINEX_SSI_Pseudorange,
    RINEX_Obs_CarrierPhase,
    RINEX_SSI_CarrierPhase,
    RINEX_Obs_Doppler,
    RINEX_Obs_SigStrength,
};

constexpr std::array<std::array<long double, 13>, 39> RINEX_REFERENCE_DATA = { {
    { 1, 22876259.395, 1, 120215551.306, 3, 562.461, 36.000, 22876256.297, 2, 93674506.094, 4, 438.697, 34.000 }, // blanks exchanged with 0.0, as defined in standard
    { 10, 21560439.759, 1, 113300853.531, 2, 482.031, 43.000, 21560437.057, 1, 88286394.188, 3, 375.763, 38.000 },
    { 12, 20935544.932, 1, 110017023.901, 1, 3045.839, 44.000, 20935541.573, 1, 85727543.876, 2, 2373.437, 39.000 },
    { 13, 19627552.460, 1, 103143484.484, 1, -3581.727, 46.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 14, 21414679.092, 1, 112534904.039, 2, -3820.361, 41.000, 21414675.846, 1, 87689515.821, 2, -2976.947, 39.000 },
    { 15, 18585989.059, 1, 97670033.209, 1, -2623.834, 47.000, 18585984.831, 1, 76106497.729, 1, -2044.598, 45.000 },
    { 17, 20145382.374, 1, 105864697.412, 1, -676.568, 45.000, 20145377.045, 1, 82491949.882, 2, -527.329, 39.000 },
    { 19, 20135419.183, 1, 105812336.287, 1, 1845.147, 45.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 23, 20763668.044, 1, 109113809.771, 2, -1579.095, 43.000, 20763664.670, 1, 85023733.967, 2, -1230.798, 41.000 },
    { 24, 18287519.029, 1, 96101556.829, 1, 1113.959, 48.000, 18287516.433, 1, 74884317.186, 1, 868.158, 47.000 },
    { 1, 18660381.325, 1, 99750420.719, 1, -4129.152, 46.000, 18660383.569, 1, 77583668.925, 2, -3211.629, 42.000 },
    { 2, 17691557.882, 1, 94405545.700, 1, 374.853, 52.000, 17691559.706, 1, 73426538.395, 1, 291.621, 48.000 },
    { 3, 20977870.930, 1, 112296185.907, 1, 3442.789, 45.000, 20977870.229, 1, 87341454.854, 2, 2677.694, 40.000 },
    { 10, 21657654.213, 1, 115447492.344, 2, -4203.686, 41.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 11, 17621718.894, 1, 94165120.864, 1, -2405.734, 51.000, 17621716.932, 1, 73239525.745, 1, -1871.005, 48.000 },
    { 12, 17891274.856, 1, 95571975.459, 1, 1328.011, 51.000, 17891275.390, 1, 74333758.388, 1, 1032.895, 48.000 },
    { 13, 21689411.389, 4, 115820280.545, 5, 3020.874, 32.000, 21689421.083, 4, 0.0, 0, 2349.882, 26.000 },
    { 20, 20311761.281, 1, 108616099.739, 2, -22.965, 40.000, 20311764.973, 2, 84479219.197, 3, -17.851, 37.000 },
    { 21, 21236134.219, 1, 113638826.837, 1, 3153.654, 46.000, 21236132.706, 1, 88385789.311, 2, 2452.924, 39.000 },
    { 3, 22094318.881, 1, 116106429.676, 1, -2446.293, 46.000, 22094320.071, 1, 88964669.099, 1, -1874.433, 50.000 },
    { 5, 25830241.475, 1, 135738837.274, 3, -3677.055, 37.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 8, 21929558.058, 1, 115240605.683, 1, 370.383, 48.000, 21929556.971, 1, 88301238.656, 1, 283.871, 50.000 },
    { 10, 26384057.373, 2, 138649147.469, 5, 1722.633, 33.000, 26384057.709, 1, 106237664.704, 2, 1320.141, 39.000 },
    { 12, 26045867.853, 2, 136871942.618, 4, 878.009, 33.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 24, 25704141.401, 1, 135076180.515, 2, -3321.829, 41.000, 25704139.224, 8, 103499921.274, 4, -2544.875, 37.000 },
    { 25, 21605768.273, 1, 113539089.544, 1, -1664.976, 47.000, 21605768.863, 1, 86997478.315, 1, -1275.841, 51.000 },
    { 30, 25956325.677, 1, 136401418.770, 3, 2315.611, 36.000, 25956331.488, 1, 104515389.371, 2, 1774.421, 41.000 },
    { 7, 0.0, 0, 0.0, 0, 0.0, 0.0, 37737385.511, 1, 151952809.467, 2, -1170.329, 40.000 },
    { 10, 36759786.025, 1, 191417848.535, 2, -1192.425, 42.000, 36759779.463, 1, 148016392.042, 2, -921.977, 40.000 },
    { 11, 23032817.499, 1, 119937928.789, 2, 2005.866, 39.000, 23032813.013, 1, 92743592.229, 1, 1551.213, 42.000 },
    { 12, 22738622.569, 1, 118405973.343, 2, -298.299, 40.000, 22738615.872, 1, 91558979.039, 1, -230.443, 44.000 },
    { 23, 19672914.865, 1, 102442028.933, 1, 96.813, 50.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 25, 20516444.452, 1, 106834510.909, 1, -2470.015, 50.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 32, 20853372.881, 1, 108588989.370, 1, -1624.284, 48.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 34, 22438311.271, 1, 116842176.709, 1, 1026.243, 45.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 37, 22643373.862, 1, 117909990.682, 1, 2110.202, 44.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 23, 36325088.073, 1, 190889609.539, 1, -535.350, 44.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 26, 38114876.816, 1, 200295012.790, 3, -444.674, 37.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
    { 36, 35983781.242, 1, 189096034.482, 1, -532.228, 44.000, 0.0, 0, 0.0, 0, 0.0, 0.0 },
} };

void compareObservation(const std::shared_ptr<const NAV::GnssObs>& obs, size_t messageCounter)
{
    // ---------------------------------------------- InsTime ------------------------------------------------
    if (messageCounter == 0) // check Epoch timestamp just once at the beginning
    {
        REQUIRE(!obs->insTime.empty());

        REQUIRE(obs->insTime.toYMDHMS().year == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Year)));
        REQUIRE(obs->insTime.toYMDHMS().month == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Month)));
        REQUIRE(obs->insTime.toYMDHMS().day == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Day)));
        REQUIRE(obs->insTime.toYMDHMS().hour == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Hour)));
        REQUIRE(obs->insTime.toYMDHMS().min == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Minute)));
        REQUIRE(obs->insTime.toYMDHMS().sec == Approx(RINEX_REFERENCE_EPOCH.at(RINEX_Second) - Gps_LeapSec).margin(EPSILON));
    }

    // -------------------------------------------- Observation ----------------------------------------------
    REQUIRE(obs->data.at(messageCounter).pseudorange == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_Pseudorange)).margin(EPSILON));
    REQUIRE(obs->data.at(messageCounter).carrierPhase == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_CarrierPhase)).margin(EPSILON));
    REQUIRE(obs->data.at(messageCounter).doppler == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_Doppler)).margin(EPSILON));
    REQUIRE(obs->data.at(messageCounter).CN0 == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_SigStrength)).margin(EPSILON));
}

TEST_CASE("[RinexObsFile][flow] Read RINEX file (v3.03) and compare content with hardcoded values", "[RinexObsFile][flow]")
{
    Logger logger;

    // ###########################################################################################################
    //                                             RinexObsFile.flow
    // ###########################################################################################################
    //
    // RinexObsFile (2)                Combiner (28)
    //       (1) GnssObs |> --(29)--> |> GnssObs (25)
    //                                |> <not linked> (26)
    //
    // ###########################################################################################################

    // TODO: Add tests for more Rinex versions
    // nm::RegisterPreInitCallback([&]() { dynamic_cast<RinexObsFile*>(nm::FindNode(2))->_path = "Rinex/FixedSize/vn310-imu.csv"; });

    size_t messageCounter = 0;

    nm::RegisterWatcherCallbackToInputPin(25, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareObservation(std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/RinexObsFile.flow"));
}

} // namespace NAV::TEST::RinexObsFileTests