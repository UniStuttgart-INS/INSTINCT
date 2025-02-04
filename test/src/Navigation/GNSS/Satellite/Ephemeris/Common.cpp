// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Common.cpp
/// @brief Common functions for Ephemeris tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-03-16

#include "Common.hpp"

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "data/SpirentAsciiSatelliteData.hpp"

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexNavFile.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::EphemerisTests
{

void testNavFile(DataSource dataSource, const std::string& navDataPath, const std::vector<std::tuple<SatSigId, std::string, Margin>>& files)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexNavFile*>(nm::FindNode(2))->_path = navDataPath;
    });

    nm::RegisterCleanupCallback([&]() {
        auto* pin = nm::FindOutputPin(1);
        REQUIRE(pin != nullptr);
        const auto* gnssNavInfo = static_cast<const GnssNavInfo*>(std::get<const void*>(pin->data));
        REQUIRE(gnssNavInfo != nullptr);

        for (const auto& [satSigId, referenceDataFilepath, margin] : files)
        {
            bool somethingChecked = false;

            const auto satId = satSigId.toSatId();
            CAPTURE(referenceDataFilepath);
            CAPTURE(satSigId);
            LOG_TRACE("[{}] - {}", satSigId, referenceDataFilepath);
            std::ifstream fs{ referenceDataFilepath, std::ios_base::binary };
            REQUIRE(fs.good());

            std::string line;
            long double gpsTime = 0;
            for (size_t i = 0; i < (dataSource == Spirent ? 5 : 1); i++) // Read header lines
            {
                std::getline(fs, line);
                if (i == 1 && dataSource == Spirent)
                {
                    std::vector<std::string> v = str::split(line, ",");
                    REQUIRE(v.size() == 7);
                    gpsTime = std::stold(v[1]);
                }
            }
            if (dataSource == Spirent)
            {
                REQUIRE(gpsTime != 0.0);
            }

            while (!fs.eof() && std::getline(fs, line) && !line.empty())
            {
                std::vector<std::string> v = str::split(line, ",");
                REQUIRE(v.size() == (dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_COUNT) : size_t(SkydelSatData_COUNT)));

                InsTime recvTime;
                if (dataSource == Spirent)
                {
                    std::string satSys;
                    switch (SatelliteSystem_(satId.satSys))
                    {
                    case GPS:
                        satSys = "GPS";
                        break;
                    case GAL:
                        satSys = "GALILEO";
                        break;
                    case GLO:
                        satSys = "GLONASS";
                        break;
                    case BDS:
                        satSys = "BeiDou";
                        break;
                    case QZSS:
                        satSys = "Quasi-Zenith";
                        break;
                    case IRNSS:
                        satSys = "IRNSS";
                        break;
                    case SBAS:
                    case SatSys_None:
                        satSys = "None";
                        break;
                    }

                    if (v[SpirentAsciiSatelliteData_Sat_type] != satSys || v[SpirentAsciiSatelliteData_Sat_ID] != std::to_string(satId.satNum)) { continue; }

                    recvTime = InsTime{ 1980, 1, 6, 0, 0, gpsTime + std::stod(v[SpirentAsciiSatelliteData_Time_ms]) * 1e-3, GPST };
                }
                else // Skydel
                {
                    recvTime = InsTime{ 0, std::stoi(v[SkydelSatData_GPS_Week_Number]), std::stold(v[SkydelSatData_GPS_TOW]) };
                }
                LOG_TRACE("  recvTime = {} ({})", recvTime.toYMDHMS(GPST), recvTime.toGPSweekTow(GPST));
                CAPTURE(recvTime);

                auto satNav = gnssNavInfo->searchNavigationData(satSigId.toSatId(), recvTime);
                REQUIRE(satNav != nullptr);
                if (!satNav->isHealthy())
                {
                    LOG_TRACE("  Not healthy. Skipping.", recvTime.toYMDHMS(GPST), recvTime.toGPSweekTow(GPST));
                    somethingChecked = true;
                    continue;
                }

                size_t grp = dataSource == Spirent ? SpirentAsciiSatelliteData::GetGroupIncr(satSigId.freq()) : 0;

                LOG_TRACE("    reference: {} ", line);

                double psr = std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_P_Range_Group_A) + grp : size_t(SkydelSatData_PSR)]);
                LOG_TRACE("    psr{} {} [m]",
                          dataSource == Skydel
                              ? ""
                              : (grp == 0
                                     ? "_A"
                                     : (grp == 1
                                            ? "_B"
                                            : (grp == 2
                                                   ? "_C"
                                                   : (grp == 3
                                                          ? "_D"
                                                          : (grp == 4
                                                                 ? "_E"
                                                                 : (grp == 5
                                                                        ? "_F"
                                                                        : "_A")))))),
                          psr);

                auto satClk = satNav->calcClockCorrections(recvTime, psr, satSigId.freq());
                if (dataSource == Skydel)
                {
                    double clkCorrection_ref = std::stod(v[SkydelSatData_Clock_Correction]);
                    if (std::abs(satClk.bias - clkCorrection_ref) > margin.clock)
                    {
                        LOG_TRACE("    clkBias           {}", satClk.bias);
                        LOG_TRACE("    clkCorrection_ref {}", clkCorrection_ref);
                        LOG_TRACE("    clkBias - ref {:e}", satClk.bias - clkCorrection_ref);
                    }
                    CHECK_THAT(satClk.bias - clkCorrection_ref, Catch::Matchers::WithinAbs(0.0, margin.clock));
                }

                auto pos = satNav->calcSatellitePos(satClk.transmitTime);

                // The Spirent reference frame is rotated by the signal transmit time
                auto rotateDataFrame = [&recvTime, &satClk](Eigen::Vector3d& data) {
                    auto dt = static_cast<double>((recvTime - satClk.transmitTime).count());

                    // see \cite SpringerHandbookGNSS2017 Springer Handbook GNSS ch. 21.2, eq. 21.18, p. 610
                    data = Eigen::AngleAxisd(InsConst::omega_ie * dt, Eigen::Vector3d::UnitZ()) * data;
                };

                Eigen::Vector3d e_refPos(std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_Sat_Pos_X) : size_t(SkydelSatData_ECEF_X)]),
                                         std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_Sat_Pos_Y) : size_t(SkydelSatData_ECEF_Y)]),
                                         std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_Sat_Pos_Z) : size_t(SkydelSatData_ECEF_Z)]));
                if (dataSource == Spirent) { rotateDataFrame(e_refPos); }
                if ((pos.e_pos - e_refPos).norm() > margin.pos)
                {
                    LOG_TRACE("    e_refPos {}", e_refPos.transpose());
                    LOG_TRACE("    pos      {}", pos.e_pos.transpose());
                    LOG_TRACE("      pos - e_refPos   = {}", (pos.e_pos - e_refPos).transpose());
                    LOG_TRACE("    | pos - e_refPos | = {:e}", (pos.e_pos - e_refPos).norm());
                }
                CHECK_THAT((pos.e_pos - e_refPos).norm(), Catch::Matchers::WithinAbs(0.0, margin.pos));

                if (dataSource == Spirent)
                {
                    auto posVel = satNav->calcSatellitePosVel(satClk.transmitTime);
                    REQUIRE(pos.e_pos == posVel.e_pos);

                    Eigen::Vector3d e_refVel(std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_X]),
                                             std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Y]),
                                             std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Z]));
                    rotateDataFrame(e_refVel);
                    if ((posVel.e_vel - e_refVel).norm() > margin.vel)
                    {
                        LOG_TRACE("    vel      {}", posVel.e_vel.transpose());
                        LOG_TRACE("    e_refVel {}", e_refVel.transpose());
                        LOG_TRACE("      vel - e_refVel   = {}", (posVel.e_vel - e_refVel).transpose());
                        LOG_TRACE("    | vel - e_refVel | = {:e}", (posVel.e_vel - e_refVel).norm());
                    }
                    CHECK_THAT((posVel.e_vel - e_refVel).norm(), Catch::Matchers::WithinAbs(0.0, margin.vel));

                    auto posVelAccel = satNav->calcSatellitePosVelAccel(satClk.transmitTime);
                    REQUIRE(posVel.e_pos == posVelAccel.e_pos);
                    REQUIRE(posVel.e_vel == posVelAccel.e_vel);

                    Eigen::Vector3d e_refAcc(std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_X]),
                                             std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Y]),
                                             std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Z]));
                    rotateDataFrame(e_refAcc);
                    if ((posVelAccel.e_accel - e_refAcc).norm() > margin.accel)
                    {
                        LOG_TRACE("    acc      {}", posVelAccel.e_accel.transpose());
                        LOG_TRACE("    e_refAcc {}", e_refAcc.transpose());
                        LOG_TRACE("      acc - e_refAcc   = {}", (posVelAccel.e_accel - e_refAcc).transpose());
                        LOG_TRACE("    | acc - e_refAcc | = {:e}", (posVelAccel.e_accel - e_refAcc).norm());
                    }
                    CHECK_THAT((posVelAccel.e_accel - e_refAcc).norm(), Catch::Matchers::WithinAbs(0.0, margin.accel));
                }
                somethingChecked = true;
            }
            REQUIRE(somethingChecked);
        }
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

} // namespace NAV::TESTS::EphemerisTests