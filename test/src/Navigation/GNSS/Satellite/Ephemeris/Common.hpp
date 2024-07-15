// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Common.hpp
/// @brief Common functions for Ephemeris tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-22

#pragma once

#include "Logger.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "util/StringUtil.hpp"

#include "data/SkydelSatData.hpp"
#include "data/SpirentAsciiSatelliteData.hpp"

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

namespace NAV::TESTS::EphemerisTests
{

enum DataSource
{
    Spirent,
    Skydel,
};

struct Margin
{
    double clock = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    double accel = 0.0;
};

template<class Ephemeris>
void testBrdcEphemerisData(const SatId& satId, const Ephemeris& eph, const std::string& referenceDataFilepath, const Margin& margin)
{
    auto logger = initializeTestLogger();

    LOG_TRACE("  eph.toc = {} ({})", eph.toc.toYMDHMS(), eph.toc.toGPSweekTow());
    std::ifstream fs{ referenceDataFilepath, std::ios_base::binary };
    REQUIRE(fs.good());

    size_t nCalc = 0;

    std::string line;
    while (!fs.eof() && std::getline(fs, line) && !line.empty())
    {
        if (line[0] == '*') // *  2023  1  8  0  0  0.00000000
        {
            // Columns 10-12 in the first %c record define what time system is used for the date/times in the ephemeris.
            // %c M  cc GPS ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc
            InsTime sp3Time = InsTime{ std::stoi(line.substr(3, 4)), std::stoi(line.substr(8, 2)), std::stoi(line.substr(11, 2)),
                                       std::stoi(line.substr(14, 2)), std::stoi(line.substr(17, 2)), std::stold(line.substr(20)), GPST };

            LOG_TRACE("  sp3Time = {} ({})", sp3Time.toYMDHMS(), sp3Time.toGPSweekTow());

            auto timeDiff = static_cast<double>((sp3Time - eph.toc).count());
            LOG_TRACE("    timeDiff {} [s]", timeDiff);

            // Only calculate for times which are within ±1 hour of ephemeris time (or 15 minutes in case of GLONASS)
            if (std::abs(timeDiff) > ((satId.satSys == GLO ? 15 : 60) + 1) * InsTimeUtil::SECONDS_PER_MINUTE) { continue; }

            auto pos = fs.tellg();
            while (!fs.eof() && std::getline(fs, line) && !line.empty())
            {
                if (line[0] == '*')
                {
                    fs.seekg(pos);
                    break;
                }
                pos = fs.tellg();

                SatId sp3SatId{ SatelliteSystem::fromChar(line[1]), static_cast<uint16_t>(std::stoul(line.substr(2, 2))) };

                if (satId != sp3SatId) { continue; }

                nCalc++;

                LOG_TRACE("    reference: {} ", line);

                Frequency freq = Freq_None;
                switch (SatelliteSystem_(satId.satSys))
                {
                case GPS:
                    freq = G01;
                    break;
                case GAL:
                    freq = E01;
                    break;
                case GLO:
                    freq = R01;
                    break;
                case BDS:
                    freq = B01;
                    break;
                case QZSS:
                    freq = J01;
                    break;
                case IRNSS:
                    freq = I05;
                    break;
                case SBAS:
                    freq = S01;
                    break;
                case SatSys_None:
                    freq = Freq_None;
                    break;
                }

                auto satClk = eph.calcClockCorrections(sp3Time, 0, freq);

                LOG_TRACE("    clkBias           {} [s]", satClk.bias);
                double clkCorrection_ref = std::stod(line.substr(47, 13)) * 1e-6;
                LOG_TRACE("    clkCorrection_ref {} [s]", clkCorrection_ref);
                LOG_TRACE("    clkBias - ref {} [s]", satClk.bias - clkCorrection_ref);
                REQUIRE_THAT(satClk.bias - clkCorrection_ref, Catch::Matchers::WithinAbs(0.0, margin.clock));

                auto pos = eph.calcSatellitePos(sp3Time);

                auto e_refPos = Eigen::Vector3d(std::stod(line.substr(5, 13)),
                                                std::stod(line.substr(19, 13)),
                                                std::stod(line.substr(33, 13)));
                e_refPos *= 1e3;
                LOG_TRACE("    e_refPos {}", e_refPos.transpose());
                LOG_TRACE("    pos      {}", pos.e_pos.transpose());
                LOG_TRACE("      pos - e_refPos   = {}", (pos.e_pos - e_refPos).transpose());
                LOG_TRACE("    | pos - e_refPos | = {}", (pos.e_pos - e_refPos).norm());
                REQUIRE_THAT((pos.e_pos - e_refPos).norm(), Catch::Matchers::WithinAbs(0.0, margin.pos));
            }
        }
    }

    // 5 minute rate measurements
    // ±  1 hour    = 25
    // ± 15 minutes =  7
    REQUIRE(nCalc == (satId.satSys == GLO ? 7 : 25));
}

template<class Ephemeris>
void testEphemerisData(const SatId& satId, const Ephemeris& eph, const std::string& referenceDataFilepath, Frequency freq, DataSource dataSource,
                       const Margin& margin)
{
    auto logger = initializeTestLogger();

    if (dataSource == Spirent) { LOG_TRACE("{}", satId); }
    else { LOG_TRACE("{} - {}", satId, referenceDataFilepath.substr(referenceDataFilepath.find_last_of('/') + 1)); }

    LOG_TRACE("  eph.toc = {} ({})", eph.toc.toYMDHMS(), eph.toc.toGPSweekTow());

    std::ifstream fs{ referenceDataFilepath, std::ios_base::binary };
    REQUIRE(fs.good());

    std::string line;
    for (size_t i = 0; i < (dataSource == Spirent ? 5 : 1); i++) { std::getline(fs, line); } // Read header lines

    size_t nCalc = 0;

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

            recvTime = InsTime{ 2023, 1, 8, 9, 59, std::stod(v[SpirentAsciiSatelliteData_Time_ms]) * 1e-3, GPST };
        }
        else // Skydel
        {
            recvTime = InsTime{ 0, std::stoi(v[SkydelSatData_GPS_Week_Number]), std::stold(v[SkydelSatData_GPS_TOW]) };
        }
        LOG_TRACE("  recvTime = {} ({})", recvTime.toYMDHMS(), recvTime.toGPSweekTow());

        auto timeDiff = static_cast<double>((recvTime - eph.toc).count());
        LOG_TRACE("    timeDiff {} [s]", timeDiff);

        // Only calculate for times which are within ±1 hour of ephemeris time (or 15 minutes in case of GLONASS)
        if (std::abs(timeDiff) > ((satId.satSys == GLO ? 15 : 60) + 1) * InsTimeUtil::SECONDS_PER_MINUTE) { continue; }

        nCalc++;

        size_t grp = 0;
        if (dataSource == Spirent)
        {
            switch (Frequency_(freq)) // TODO: Check sorting of GLONASS
            {
            case Freq_None:
            case G01:
            case E01:
            case R04:
            case J01:
            case I09:
            case S01:
                grp = 0; // Group A (L1/E1/S/B1I)
                break;
            case G02:
            case E06:
            case R06:
            case J02:
                grp = 1; // Group B (L2/E6/B2I)
                break;
            case G05:
            case E08:
            case R01:
            case B05:
            case J05:
            case I05:
            case S05:
                grp = 2; // Group C (L5/E5/B2A/C1)
                break;
            case R02:
            case J06:
                grp = 3; // Group D (L6/B1C/C2)
                break;
            case R03:
            case B06:
                grp = 4; // Group E (B3I/C3)
                break;
            case B07:
                grp = 5; // Group F (B2b)
                break;
            case E07: // TODO: sort these
            case E05:
            case B01:
            case B02:
            case B08:
                REQUIRE(false);
            }
        }

        LOG_TRACE("    reference: {} ", line);

        double psr = std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_P_Range_Group_A) + grp : size_t(SkydelSatData_PSR)]);
        LOG_TRACE("    psr {} [m]", psr);

        auto satClk = eph.calcClockCorrections(recvTime, psr, freq);

        LOG_TRACE("    clkBias           {}", satClk.bias);
        if (dataSource == Skydel)
        {
            double clkCorrection_ref = std::stod(v[SkydelSatData_Clock_Correction]);
            LOG_TRACE("    clkCorrection_ref {}", clkCorrection_ref);
            LOG_TRACE("    clkBias - ref {}", satClk.bias - clkCorrection_ref);
            REQUIRE_THAT(satClk.bias - clkCorrection_ref, Catch::Matchers::WithinAbs(0.0, margin.clock));
        }

        LOG_TRACE("    transmitTime = {} ({})", satClk.transmitTime.toYMDHMS(), satClk.transmitTime.toGPSweekTow());

        auto pos = eph.calcSatellitePos(satClk.transmitTime);

        // The Spirent reference frame is rotated by the signal transmit time
        auto rotateDataFrame = [&recvTime, &satClk](Eigen::Vector3d& data) {
            auto dt = static_cast<double>((recvTime - satClk.transmitTime).count());

            // see \cite SpringerHandbookGNSS2017 Springer Handbook GNSS ch. 21.2, eq. 21.18, p. 610
            data = Eigen::AngleAxisd(InsConst<>::omega_ie * dt, Eigen::Vector3d::UnitZ()) * data;
        };

        Eigen::Vector3d e_refPos(std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_Sat_Pos_X) : size_t(SkydelSatData_ECEF_X)]),
                                 std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_Sat_Pos_Y) : size_t(SkydelSatData_ECEF_Y)]),
                                 std::stod(v[dataSource == Spirent ? size_t(SpirentAsciiSatelliteData_Sat_Pos_Z) : size_t(SkydelSatData_ECEF_Z)]));
        if (dataSource == Spirent) { rotateDataFrame(e_refPos); }
        LOG_TRACE("    e_refPos {}", e_refPos.transpose());
        LOG_TRACE("    pos      {}", pos.e_pos.transpose());
        LOG_TRACE("      pos - e_refPos   = {}", (pos.e_pos - e_refPos).transpose());
        LOG_TRACE("    | pos - e_refPos | = {}", (pos.e_pos - e_refPos).norm());
        REQUIRE_THAT((pos.e_pos - e_refPos).norm(), Catch::Matchers::WithinAbs(0.0, margin.pos));

        if (dataSource == Spirent)
        {
            auto posVel = eph.calcSatellitePosVel(satClk.transmitTime);
            REQUIRE(pos.e_pos == posVel.e_pos);

            Eigen::Vector3d e_refVel(std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_X]),
                                     std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Y]),
                                     std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Z]));
            rotateDataFrame(e_refVel);
            LOG_TRACE("    vel      {}", posVel.e_vel.transpose());
            LOG_TRACE("    e_refVel {}", e_refVel.transpose());
            LOG_TRACE("      vel - e_refVel   = {}", (posVel.e_vel - e_refVel).transpose());
            LOG_TRACE("    | vel - e_refVel | = {}", (posVel.e_vel - e_refVel).norm());
            REQUIRE_THAT((posVel.e_vel - e_refVel).norm(), Catch::Matchers::WithinAbs(0.0, margin.vel));

            auto posVelAccel = eph.calcSatellitePosVelAccel(satClk.transmitTime);
            REQUIRE(posVel.e_pos == posVelAccel.e_pos);
            REQUIRE(posVel.e_vel == posVelAccel.e_vel);

            Eigen::Vector3d e_refAcc(std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_X]),
                                     std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Y]),
                                     std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Z]));
            rotateDataFrame(e_refAcc);
            LOG_TRACE("    acc      {}", posVelAccel.e_accel.transpose());
            LOG_TRACE("    e_refAcc {}", e_refAcc.transpose());
            LOG_TRACE("      acc - e_refAcc   = {}", (posVelAccel.e_accel - e_refAcc).transpose());
            LOG_TRACE("    | acc - e_refAcc | = {}", (posVelAccel.e_accel - e_refAcc).norm());
            REQUIRE_THAT((posVelAccel.e_accel - e_refAcc).norm(), Catch::Matchers::WithinAbs(0.0, margin.accel));
        }
    }

    // 5 minute rate measurements
    // ±  1 hour    = 25
    // ± 15 minutes =  7
    REQUIRE(nCalc == (satId.satSys == GLO ? 7 : 25));
}

void testNavFile(DataSource dataSource, const std::string& navDataPath, const std::vector<std::tuple<SatSigId, std::string, Margin>>& files);

} // namespace NAV::TESTS::EphemerisTests