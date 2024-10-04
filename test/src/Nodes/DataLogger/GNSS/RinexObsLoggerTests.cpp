// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexObsLoggerTests.hpp
/// @brief Tests for the RinexObsLogger Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-18

#include <catch2/catch_test_macros.hpp>

#include "FlowTester.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssObsComparisons.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/Version.hpp"
#include "util/StringUtil.hpp"

#include "Logger.hpp"

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"
#include "Nodes/DataLogger/GNSS/RinexObsLogger.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::RinexObsLoggerTests
{

void compareObservations(std::deque<std::shared_ptr<const NAV::GnssObs>>& data1, std::deque<std::shared_ptr<const NAV::GnssObs>>& data2)
{
    if (data1.empty() || data2.empty()) { return; }

    REQUIRE(*data1.front() == *data2.front());

    data1.pop_front();
    data2.pop_front();
}

void testFile(const std::string& path, const std::vector<std::string>& expectedHeader)
{
    // ###########################################################################################################
    //                                         RinexObsLogger.flow
    // ###########################################################################################################
    //
    // RinexObsFile (2)            RinexObsLogger (4)
    //   (1) GnssObs |>  --(5)-->  |> Binary Output (3)
    constexpr size_t NODE_ID_RINEX_OBS_FILE_1 = 2;
    constexpr size_t NODE_ID_RINEX_OBS_LOGGER = 4;
    //
    // ###########################################################################################################

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(NODE_ID_RINEX_OBS_FILE_1))->_path = path;
        dynamic_cast<RinexObsLogger*>(nm::FindNode(NODE_ID_RINEX_OBS_LOGGER))->_path = "RinexObsLoggerTests/" + path;
    });
    REQUIRE(testFlow("test/flow/Nodes/DataLogger/GNSS/RinexObsLogger.flow"));

    // ###########################################################################################################
    //                                         RinexObsLoggerCheck.flow
    // ###########################################################################################################
    //
    // RinexObsFile (2)             Terminator (4)
    //   (1) GnssObs |>  ---(9)---> |> (3)
    constexpr size_t PIN_ID_TERM_1 = 3;
    //
    // RinexObsFile (6)             Terminator (8)
    //   (5) GnssObs |>  ---(10)--> |> (7)
    constexpr size_t NODE_ID_RINEX_OBS_FILE_2 = 6;
    constexpr size_t PIN_ID_TERM_2 = 7;
    //
    // ###########################################################################################################

    std::atomic<size_t> messageCounterTerminator1 = 0;
    std::atomic<size_t> messageCounterTerminator2 = 0;
    std::deque<std::shared_ptr<const NAV::GnssObs>> data1;
    std::deque<std::shared_ptr<const NAV::GnssObs>> data2;
    std::mutex comparisonMutex;

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(NODE_ID_RINEX_OBS_FILE_1))->_path = path;
        dynamic_cast<RinexObsFile*>(nm::FindNode(NODE_ID_RINEX_OBS_FILE_2))->_path = "../logs/RinexObsLoggerTests/" + path;
    });

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_TERM_1, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterTerminator1++;
        LOG_TRACE("messageCounterTerminator1 = {}", fmt::streamed(messageCounterTerminator1));

        data1.push_back(std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()));

        std::scoped_lock lk(comparisonMutex);
        compareObservations(data1, data2);
    });
    nm::RegisterWatcherCallbackToInputPin(PIN_ID_TERM_2, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterTerminator2++;
        LOG_TRACE("messageCounterTerminator2 = {}", fmt::streamed(messageCounterTerminator2));

        data2.push_back(std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()));

        std::scoped_lock lk(comparisonMutex);
        compareObservations(data1, data2);
    });

    REQUIRE(testFlow("test/flow/Nodes/DataLogger/GNSS/RinexObsLoggerCheck.flow"));

    REQUIRE(messageCounterTerminator1 == messageCounterTerminator2);
    REQUIRE(data1.empty());
    REQUIRE(data2.empty());

    std::ifstream fs{ "test/logs/RinexObsLoggerTests/" + path };
    REQUIRE(fs.good());

    size_t headerCount = 0;
    while (!fs.eof())
    {
        std::string line;
        std::getline(fs, line);
        if (line[0] == '>') { break; }

        CAPTURE(line);
        CAPTURE(headerCount);
        if (line.find("PGM / RUN BY / DATE", 60) != std::string::npos)
        {
            REQUIRE(line.substr(20, 20) == expectedHeader.at(headerCount).substr(20, 20)); // RUN BY
            REQUIRE(line.substr(60) == expectedHeader.at(headerCount).substr(60));         // Label
        }
        else
        {
            REQUIRE(line == expectedHeader.at(headerCount));
        }
        headerCount++;
    }
    REQUIRE(headerCount == expectedHeader.size());
}

TEST_CASE("[RinexObsLoggerTests][flow] DataLogger/GNSS/RinexObsLogger-1.obs", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataLogger/GNSS/RinexObsLogger-1.obs",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 135830 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G    8 C1C L1C D1C S1C C2X L2X D2X S2X                      SYS / # / OBS TYPES ",
                 "E    8 C1X L1X D1X S1X C7X L7X D7X S7X                      SYS / # / OBS TYPES ",
                 "R    8 C2C L2C D2C S2C C1C L1C D1C S1C                      SYS / # / OBS TYPES ",
                 "C    8 C2I L2I D2I S2I C7I L7I D7I S7I                      SYS / # / OBS TYPES ",
                 "J   20 C1C L1C D1C S1C C1X L1X D1X S1X C1Z L1Z D1Z S1Z C5X  SYS / # / OBS TYPES ",
                 "       L5X D5X S5X C5Z L5Z D5Z S5Z                          SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "     0.100                                                  INTERVAL            ",
                 "  2023     5    24     7     4   30.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2023     5    24     7     4   30.3000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    43                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataLogger/GNSS/RinexObsLogger-2.obs", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataLogger/GNSS/RinexObsLogger-2.obs",
             {
                 "     3.04           OBSERVATION DATA    R: GLONASS          RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140058 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "R    8 C2C L2C D2C S2C C1C L1C D1C S1C                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "     0.100                                                  INTERVAL            ",
                 "  2023     5    24     7     4   30.0000000     GLO         TIME OF FIRST OBS   ",
                 "  2023     5    24     7     4   30.1000000     GLO         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "     9                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataProcessor/tckf/reach-m2-01_raw_202306291111_noDoppler.23O", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataProcessor/tckf/reach-m2-01_raw_202306291111_noDoppler.23O",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140416 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G    6 C1C L1C S1C C2X L2X S2X                              SYS / # / OBS TYPES ",
                 "E    6 C1X L1X S1X C7X L7X S7X                              SYS / # / OBS TYPES ",
                 "R    6 C1C L1C S1C C2C S2C L2C                              SYS / # / OBS TYPES ",
                 "C    3 C2I L2I S2I                                          SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "     0.100                                                  INTERVAL            ",
                 "  2023     6    29    11    12   42.0940000     GPS         TIME OF FIRST OBS   ",
                 "  2023     6    29    11    13   42.9940000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    35                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataProcessor/tckf/reach-m2-01_raw_202306291111_psrGaps.23O", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataProcessor/tckf/reach-m2-01_raw_202306291111_psrGaps.23O",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140417 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G    8 C1C L1C D1C S1C C2X L2X D2X S2X                      SYS / # / OBS TYPES ",
                 "E    8 C1X L1X D1X S1X C7X L7X D7X S7X                      SYS / # / OBS TYPES ",
                 "R    8 C1C L1C D1C S1C C2C D2C S2C L2C                      SYS / # / OBS TYPES ",
                 "C    4 C2I L2I D2I S2I                                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "     0.100                                                  INTERVAL            ",
                 "  2023     6    29    11    12   42.0940000     GPS         TIME OF FIRST OBS   ",
                 "  2023     6    29    11    13   42.9940000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    35                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataProcessor/tckf/reach-m2-01_raw_202306291111.23O", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataProcessor/tckf/reach-m2-01_raw_202306291111.23O",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140419 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G    8 C1C L1C D1C S1C C2X L2X D2X S2X                      SYS / # / OBS TYPES ",
                 "E    8 C1X L1X D1X S1X C7X L7X D7X S7X                      SYS / # / OBS TYPES ",
                 "R    8 C1C L1C D1C S1C C2C D2C S2C L2C                      SYS / # / OBS TYPES ",
                 "C    4 C2I L2I D2I S2I                                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "     0.100                                                  INTERVAL            ",
                 "  2023     6    29    11    12   42.0940000     GPS         TIME OF FIRST OBS   ",
                 "  2023     6    29    11    13   42.9940000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    35                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataProvider/GNSS/RinexObsFile/v3_02/INSA11DEU_R_MO.rnx", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataProvider/GNSS/RinexObsFile/v3_02/INSA11DEU_R_MO.rnx",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140953 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G   15 C1C L1C S1C C2X L2X S2X C5X L5X S5X C1X L1X S1X C2W  SYS / # / OBS TYPES ",
                 "       L2W S2W                                              SYS / # / OBS TYPES ",
                 "E   15 C1X L1X S1X C5X L5X S5X C7X L7X S7X C8X L8X S8X C6X  SYS / # / OBS TYPES ",
                 "       L6X S6X                                              SYS / # / OBS TYPES ",
                 "R    9 C1C L1C S1C C2C L2C S2C C3X L3X S3X                  SYS / # / OBS TYPES ",
                 "C    9 C2I L2I S2I C6I L6I S6I C7I L7I S7I                  SYS / # / OBS TYPES ",
                 "S    3 C1C L1C S1C                                          SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "                                                            INTERVAL            ",
                 "  2022     3     1    23     0    0.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2022     3     1    23     0    0.0000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    12                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataProvider/GNSS/RinexObsFile/v3_03/reach-m2-01_raw.22O", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataProvider/GNSS/RinexObsFile/v3_03/reach-m2-01_raw.22O",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140953 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G    8 C1C L1C D1C S1C C2X L2X D2X S2X                      SYS / # / OBS TYPES ",
                 "E    8 C1X L1X D1X S1X C7X L7X D7X S7X                      SYS / # / OBS TYPES ",
                 "R    8 C1C L1C D1C S1C C2C L2C D2C S2C                      SYS / # / OBS TYPES ",
                 "C    8 C7I L7I D7I S7I C2I L2I D2I S2I                      SYS / # / OBS TYPES ",
                 "S    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "                                                            INTERVAL            ",
                 "  2022    11     2    16    39   59.6920000     GPS         TIME OF FIRST OBS   ",
                 "  2022    11     2    16    39   59.6920000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    11                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] DataProvider/GNSS/RinexObsFile/v3_04/INS_1581.19O", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("DataProvider/GNSS/RinexObsFile/v3_04/INS_1581.19O",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140953 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G    9 C1C L1C C1W C2W L2W C2L L2L C5Q L5Q                  SYS / # / OBS TYPES ",
                 "E   10 C1C L1C C6C L6C C5Q L5Q C7Q L7Q C8Q L8Q              SYS / # / OBS TYPES ",
                 "R   10 C1C L1C C1P L1P C2P L2P C2C L2C C3Q L3Q              SYS / # / OBS TYPES ",
                 "C    6 C2I L2I C7I L7I C6I L6I                              SYS / # / OBS TYPES ",
                 "J    6 C1C L1C C2L L2L C5Q L5Q                              SYS / # / OBS TYPES ",
                 "I    2 C5A L5A                                              SYS / # / OBS TYPES ",
                 "S    4 C1C L1C C5I L5I                                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "                                                            INTERVAL            ",
                 "  2019     6     7     0     0    0.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2019     6     7     0     0    0.0000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    15                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/Septentrio-PolaRx5TR.obs", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/Septentrio-PolaRx5TR.obs",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140953 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G   18 C1C L1C D1C S1C C1W S1W C2W L2W D2W S2W C2L L2L D2L  SYS / # / OBS TYPES ",
                 "       S2L C1L L1L D1L S1L                                  SYS / # / OBS TYPES ",
                 "E    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ",
                 "R    8 C1C L1C D1C S1C C2C L2C D2C S2C                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "   300.000                                                  INTERVAL            ",
                 "  2023     1     8    10     0    0.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2023     1     8    14     0    0.0000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    40                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20230080000_04H_MO.rnx", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20230080000_04H_MO.rnx",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140954 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G   24 C1C L1C D1C S1C C1P L1P D1P S1P C1X L1X D1X S1X C2P  SYS / # / OBS TYPES ",
                 "       L2P D2P S2P C2C L2C D2C S2C C5X L5X D5X S5X          SYS / # / OBS TYPES ",
                 "E   12 C1X L1X D1X S1X C5X L5X D5X S5X C7X L7X D7X S7X      SYS / # / OBS TYPES ",
                 "R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES ",
                 "C   16 C1X L1X D1X S1X C2X L2X D2X S2X C5X L5X D5X S5X C7X  SYS / # / OBS TYPES ",
                 "       L7X D7X S7X                                          SYS / # / OBS TYPES ",
                 "J   20 C1C L1C D1C S1C C1X L1X D1X S1X C1Z L1Z D1Z S1Z C5X  SYS / # / OBS TYPES ",
                 "       L5X D5X S5X C5Z L5Z D5Z S5Z                          SYS / # / OBS TYPES ",
                 "I    4 C5X L5X D5X S5X                                      SYS / # / OBS TYPES ",
                 "S    8 C1C L1C D1C S1C C5X L5X D5X S5X                      SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "   300.000                                                  INTERVAL            ",
                 "  2023     1     8    10     0    0.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2023     1     8    14     0    0.0000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    88                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Septentrio-PolaRx5T.obs", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Septentrio-PolaRx5T.obs",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140954 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G   18 C1C L1C D1C S1C C1W S1W C2W L2W D2W S2W C2L L2L D2L  SYS / # / OBS TYPES ",
                 "       S2L C5Q L5Q D5Q S5Q                                  SYS / # / OBS TYPES ",
                 "E   16 C1C L1C D1C S1C C5Q L5Q D5Q S5Q C7Q L7Q D7Q S7Q C8Q  SYS / # / OBS TYPES ",
                 "       L8Q D8Q S8Q                                          SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "   300.000                                                  INTERVAL            ",
                 "  2023     1     8    10     0    0.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2023     1     8    14     0    0.0000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    29                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

TEST_CASE("[RinexObsLoggerTests][flow] GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Spirent_RINEX_MO.obs", "[RinexObsLoggerTests][flow]")
{
    auto logger = initializeTestLogger();

    testFile("GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Spirent_RINEX_MO.obs",
             {
                 "     3.04           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE",
                 "INSTINCT 1.1.0      qwertyuiopasdfghjklz20240123 140954 UTC PGM / RUN BY / DATE ",
                 "hnwerctiwot8c4th8thcwo8thc3nwothhhhhhho384co834hcot8n834othtCOMMENT             ",
                 "                                                            COMMENT             ",
                 "345c34w                                                     COMMENT             ",
                 "34c9834ctu8340mu340cu34034u934ucm405uc3495cu3405cu345c93425uMARKER NAME         ",
                 "cny34895y0c34n5cy304                                        MARKER NUMBER       ",
                 "oy834cno934u09tcm034                                        MARKER TYPE         ",
                 "fndmncxvd,sjgfs;kfsdo34g43utbtjbtt3jwbkjbt43hjtbw3ljb34tljbtOBSERVER / AGENCY   ",
                 "comi34co58345yunnn924c594387ncy93485cyn c42534mmmmmmmmmm2xy3REC # / TYPE / VERS ",
                 "m34xy34y93487yx9y394x73ym398xyyyy93457y3                    ANT # / TYPE        ",
                 "-99999999.9999 99999999.9999        0.0000                  APPROX POSITION XYZ ",
                 "-99999999.9999       -1.2346-99999999.9999                  ANTENNA: DELTA H/E/N",
                 "G   20 C1C L1C D1C S1C C1P L1P D1P S1P C2X L2X D2X S2X C2P  SYS / # / OBS TYPES ",
                 "       L2P D2P S2P C5I L5I D5I S5I                          SYS / # / OBS TYPES ",
                 "E   16 C1C L1C D1C S1C C5Q L5Q D5Q S5Q C7Q L7Q D7Q S7Q C8Q  SYS / # / OBS TYPES ",
                 "       L8Q D8Q S8Q                                          SYS / # / OBS TYPES ",
                 "R   16 C1C L1C D1C S1C C1P L1P D1P S1P C2C L2C D2C S2C C2P  SYS / # / OBS TYPES ",
                 "       L2P D2P S2P                                          SYS / # / OBS TYPES ",
                 "C   24 C2I L2I D2I S2I C7I L7I D7I S7I C6I L6I D6I S6I C7D  SYS / # / OBS TYPES ",
                 "       L7D D7D S7D C5X L5X D5X S5X C1X L1X D1X S1X          SYS / # / OBS TYPES ",
                 "J   20 C1C L1C D1C S1C C1X L1X D1X S1X C1Z L1Z D1Z S1Z C2X  SYS / # / OBS TYPES ",
                 "       L2X D2X S2X C5I L5I D5I S5I                          SYS / # / OBS TYPES ",
                 "DBHZ                                                        SIGNAL STRENGTH UNIT",
                 "   300.000                                                  INTERVAL            ",
                 "  2023     1     8    10     0    0.0000000     GPS         TIME OF FIRST OBS   ",
                 "  2023     1     8    14     0    0.0000000     GPS         TIME OF LAST OBS    ",
                 "                                                            SYS / PHASE SHIFT   ",
                 "                                                            GLONASS COD/PHS/BIS ",
                 "    18                                                      LEAP SECONDS        ",
                 "    65                                                      # OF SATELLITES     ",
                 "                                                            END OF HEADER       ",
             });
}

} // namespace NAV::TESTS::RinexObsLoggerTests