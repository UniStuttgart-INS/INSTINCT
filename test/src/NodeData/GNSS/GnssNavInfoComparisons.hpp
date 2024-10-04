// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssNavInfoComparisons.hpp
/// @brief Comparison operators to compare the GnssNavInfo type
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-19

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

#include "Logger.hpp"
#include "CatchMatchers.hpp"
#include "util/Container/STL.hpp"

#include "Navigation/Math/Math.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/QZSSEphemeris.hpp"

namespace NAV
{

inline bool operator==(const IonosphericCorrections::Corrections& lhs, const IonosphericCorrections::Corrections& rhs)
{
    LOG_DEBUG("    [{}] == [{}] | satSys", lhs.satSys, rhs.satSys);
    REQUIRE(lhs.satSys == rhs.satSys);
    LOG_DEBUG("    [{}] == [{}] | alphaBeta",
              lhs.alphaBeta == IonosphericCorrections::Alpha ? "Alpha" : "Beta",
              rhs.alphaBeta == IonosphericCorrections::Alpha ? "Alpha" : "Beta");
    REQUIRE(lhs.alphaBeta == rhs.alphaBeta);
    LOG_DEBUG("    [{}] == [{}] | data", joinToString(lhs.data), joinToString(rhs.data));
    REQUIRE_THAT(lhs.data, Catch::Matchers::EqualsSigDigitsContainer(rhs.data, 4)); // RINEX Format: D12.4
    return true;
}

inline bool operator==(const IonosphericCorrections& lhs, const IonosphericCorrections& rhs)
{
    LOG_DEBUG("  [{}] == [{}] | data().size()", lhs.data().size(), rhs.data().size());
    REQUIRE(lhs.data().size() == rhs.data().size());
    LOG_DEBUG("IonosphericCorrections::Corrections");
    for (const auto& lcor : lhs.data())
    {
        auto iter = std::find_if(rhs.data().begin(), rhs.data().end(), [&](const auto& rcor) {
            return lcor.satSys == rcor.satSys && lcor.alphaBeta == rcor.alphaBeta;
        });
        REQUIRE(iter != rhs.data().end());
        REQUIRE(lcor == *iter);
    }
    return true;
}
inline bool operator==(const GnssNavInfo::TimeSystemCorrections& lhs, const GnssNavInfo::TimeSystemCorrections& rhs)
{
    LOG_DEBUG("  [{}] == [{}] | a0", lhs.a0, rhs.a0);
    REQUIRE_THAT(lhs.a0, Catch::Matchers::EqualsSigDigits(rhs.a0, 10)); // RINEX Format: D17.10
    LOG_DEBUG("  [{}] == [{}] | a1", lhs.a1, rhs.a1);
    REQUIRE_THAT(lhs.a1, Catch::Matchers::EqualsSigDigits(rhs.a1, 9)); // RINEX Format: D16.9
    return true;
}

inline bool operator==(const GPSEphemeris& lhs, const GPSEphemeris& rhs)
{
    LOG_DEBUG("   [{}] == [{}] | toc", lhs.toc.toYMDHMS(GPST), rhs.toc.toYMDHMS(GPST));
    REQUIRE(lhs.toc == rhs.toc);
    LOG_DEBUG("   [{}] == [{}] | toe", lhs.toe.toYMDHMS(GPST), rhs.toe.toYMDHMS(GPST));
    REQUIRE(lhs.toe == rhs.toe);
    LOG_DEBUG("    [{}] == [{}] | IODE", lhs.IODE, rhs.IODE);
    REQUIRE(lhs.IODE == rhs.IODE);
    LOG_DEBUG("    [{}] == [{}] | IODC", lhs.IODC, rhs.IODC);
    REQUIRE(lhs.IODC == rhs.IODC);
    LOG_DEBUG("    [{}] == [{}] | a", joinToString(lhs.a), joinToString(rhs.a));
    REQUIRE_THAT(lhs.a, Catch::Matchers::EqualsSigDigitsContainer(rhs.a, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | sqrt_A", lhs.sqrt_A, rhs.sqrt_A);
    REQUIRE_THAT(lhs.sqrt_A, Catch::Matchers::EqualsSigDigits(rhs.sqrt_A, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | e", lhs.e, rhs.e);
    REQUIRE_THAT(lhs.e, Catch::Matchers::EqualsSigDigits(rhs.e, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_0", lhs.i_0, rhs.i_0);
    REQUIRE_THAT(lhs.i_0, Catch::Matchers::EqualsSigDigits(rhs.i_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_0", lhs.Omega_0, rhs.Omega_0);
    REQUIRE_THAT(lhs.Omega_0, Catch::Matchers::EqualsSigDigits(rhs.Omega_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | omega", lhs.omega, rhs.omega);
    REQUIRE_THAT(lhs.omega, Catch::Matchers::EqualsSigDigits(rhs.omega, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | M_0", lhs.M_0, rhs.M_0);
    REQUIRE_THAT(lhs.M_0, Catch::Matchers::EqualsSigDigits(rhs.M_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | delta_n", lhs.delta_n, rhs.delta_n);
    REQUIRE_THAT(lhs.delta_n, Catch::Matchers::EqualsSigDigits(rhs.delta_n, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_dot", lhs.Omega_dot, rhs.Omega_dot);
    REQUIRE_THAT(lhs.Omega_dot, Catch::Matchers::EqualsSigDigits(rhs.Omega_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_dot", lhs.i_dot, rhs.i_dot);
    REQUIRE_THAT(lhs.i_dot, Catch::Matchers::EqualsSigDigits(rhs.i_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cus", lhs.Cus, rhs.Cus);
    REQUIRE_THAT(lhs.Cus, Catch::Matchers::EqualsSigDigits(rhs.Cus, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cuc", lhs.Cuc, rhs.Cuc);
    REQUIRE_THAT(lhs.Cuc, Catch::Matchers::EqualsSigDigits(rhs.Cuc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cis", lhs.Cis, rhs.Cis);
    REQUIRE_THAT(lhs.Cis, Catch::Matchers::EqualsSigDigits(rhs.Cis, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cic", lhs.Cic, rhs.Cic);
    REQUIRE_THAT(lhs.Cic, Catch::Matchers::EqualsSigDigits(rhs.Cic, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crs", lhs.Crs, rhs.Crs);
    REQUIRE_THAT(lhs.Crs, Catch::Matchers::EqualsSigDigits(rhs.Crs, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crc", lhs.Crc, rhs.Crc);
    REQUIRE_THAT(lhs.Crc, Catch::Matchers::EqualsSigDigits(rhs.Crc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | svAccuracy", lhs.svAccuracy, rhs.svAccuracy);
    REQUIRE(lhs.svAccuracy == rhs.svAccuracy);
    LOG_DEBUG("    [{}] == [{}] | svHealth", lhs.svHealth, rhs.svHealth);
    REQUIRE(lhs.svHealth == rhs.svHealth);
    LOG_DEBUG("    [{}] == [{}] | L2ChannelCodes", lhs.L2ChannelCodes, rhs.L2ChannelCodes);
    REQUIRE(lhs.L2ChannelCodes == rhs.L2ChannelCodes);
    LOG_DEBUG("    [{}] == [{}] | L2DataFlagPCode", lhs.L2DataFlagPCode, rhs.L2DataFlagPCode);
    REQUIRE(lhs.L2DataFlagPCode == rhs.L2DataFlagPCode);
    LOG_DEBUG("    [{}] == [{}] | T_GD", lhs.T_GD, rhs.T_GD);
    REQUIRE_THAT(lhs.T_GD, Catch::Matchers::EqualsSigDigits(rhs.T_GD, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | fitInterval", lhs.fitInterval, rhs.fitInterval);
    REQUIRE(lhs.fitInterval == rhs.fitInterval);
    return true;
}

inline bool operator==(const GalileoEphemeris& lhs, const GalileoEphemeris& rhs)
{
    LOG_DEBUG("   [{}] == [{}] | toc", lhs.toc.toYMDHMS(GPST), rhs.toc.toYMDHMS(GPST));
    REQUIRE(lhs.toc == rhs.toc);
    LOG_DEBUG("   [{}] == [{}] | toe", lhs.toe.toYMDHMS(GPST), rhs.toe.toYMDHMS(GPST));
    REQUIRE(lhs.toe == rhs.toe);
    LOG_DEBUG("    [{}] == [{}] | IODnav", lhs.IODnav, rhs.IODnav);
    REQUIRE(lhs.IODnav == rhs.IODnav);
    LOG_DEBUG("    [{}] == [{}] | a", joinToString(lhs.a), joinToString(rhs.a));
    REQUIRE_THAT(lhs.a, Catch::Matchers::EqualsSigDigitsContainer(rhs.a, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | sqrt_A", lhs.sqrt_A, rhs.sqrt_A);
    REQUIRE_THAT(lhs.sqrt_A, Catch::Matchers::EqualsSigDigits(rhs.sqrt_A, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | e", lhs.e, rhs.e);
    REQUIRE_THAT(lhs.e, Catch::Matchers::EqualsSigDigits(rhs.e, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_0", lhs.i_0, rhs.i_0);
    REQUIRE_THAT(lhs.i_0, Catch::Matchers::EqualsSigDigits(rhs.i_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_0", lhs.Omega_0, rhs.Omega_0);
    REQUIRE_THAT(lhs.Omega_0, Catch::Matchers::EqualsSigDigits(rhs.Omega_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | omega", lhs.omega, rhs.omega);
    REQUIRE_THAT(lhs.omega, Catch::Matchers::EqualsSigDigits(rhs.omega, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | M_0", lhs.M_0, rhs.M_0);
    REQUIRE_THAT(lhs.M_0, Catch::Matchers::EqualsSigDigits(rhs.M_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | delta_n", lhs.delta_n, rhs.delta_n);
    REQUIRE_THAT(lhs.delta_n, Catch::Matchers::EqualsSigDigits(rhs.delta_n, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_dot", lhs.Omega_dot, rhs.Omega_dot);
    REQUIRE_THAT(lhs.Omega_dot, Catch::Matchers::EqualsSigDigits(rhs.Omega_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_dot", lhs.i_dot, rhs.i_dot);
    REQUIRE_THAT(lhs.i_dot, Catch::Matchers::EqualsSigDigits(rhs.i_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cus", lhs.Cus, rhs.Cus);
    REQUIRE_THAT(lhs.Cus, Catch::Matchers::EqualsSigDigits(rhs.Cus, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cuc", lhs.Cuc, rhs.Cuc);
    REQUIRE_THAT(lhs.Cuc, Catch::Matchers::EqualsSigDigits(rhs.Cuc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cis", lhs.Cis, rhs.Cis);
    REQUIRE_THAT(lhs.Cis, Catch::Matchers::EqualsSigDigits(rhs.Cis, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cic", lhs.Cic, rhs.Cic);
    REQUIRE_THAT(lhs.Cic, Catch::Matchers::EqualsSigDigits(rhs.Cic, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crs", lhs.Crs, rhs.Crs);
    REQUIRE_THAT(lhs.Crs, Catch::Matchers::EqualsSigDigits(rhs.Crs, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crc", lhs.Crc, rhs.Crc);
    REQUIRE_THAT(lhs.Crc, Catch::Matchers::EqualsSigDigits(rhs.Crc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | signalAccuracy", lhs.signalAccuracy, rhs.signalAccuracy);
    REQUIRE(lhs.signalAccuracy == rhs.signalAccuracy);
    LOG_DEBUG("    [{}] == [{}] | E5a_DataValidityStatus", fmt::underlying(lhs.svHealth.E5a_DataValidityStatus), fmt::underlying(rhs.svHealth.E5a_DataValidityStatus));
    REQUIRE(lhs.svHealth.E5a_DataValidityStatus == rhs.svHealth.E5a_DataValidityStatus);
    LOG_DEBUG("    [{}] == [{}] | E5b_DataValidityStatus", fmt::underlying(lhs.svHealth.E5b_DataValidityStatus), fmt::underlying(rhs.svHealth.E5b_DataValidityStatus));
    REQUIRE(lhs.svHealth.E5b_DataValidityStatus == rhs.svHealth.E5b_DataValidityStatus);
    LOG_DEBUG("    [{}] == [{}] | E1B_DataValidityStatus", fmt::underlying(lhs.svHealth.E1B_DataValidityStatus), fmt::underlying(rhs.svHealth.E1B_DataValidityStatus));
    REQUIRE(lhs.svHealth.E1B_DataValidityStatus == rhs.svHealth.E1B_DataValidityStatus);
    LOG_DEBUG("    [{}] == [{}] | E5a_SignalHealthStatus", fmt::underlying(lhs.svHealth.E5a_SignalHealthStatus), fmt::underlying(rhs.svHealth.E5a_SignalHealthStatus));
    REQUIRE(lhs.svHealth.E5a_SignalHealthStatus == rhs.svHealth.E5a_SignalHealthStatus);
    LOG_DEBUG("    [{}] == [{}] | E5b_SignalHealthStatus", fmt::underlying(lhs.svHealth.E5b_SignalHealthStatus), fmt::underlying(rhs.svHealth.E5b_SignalHealthStatus));
    REQUIRE(lhs.svHealth.E5b_SignalHealthStatus == rhs.svHealth.E5b_SignalHealthStatus);
    LOG_DEBUG("    [{}] == [{}] | E1B_SignalHealthStatus", fmt::underlying(lhs.svHealth.E1B_SignalHealthStatus), fmt::underlying(rhs.svHealth.E1B_SignalHealthStatus));
    REQUIRE(lhs.svHealth.E1B_SignalHealthStatus == rhs.svHealth.E1B_SignalHealthStatus);
    LOG_DEBUG("    [{}] == [{}] | dataSource", lhs.dataSource, rhs.dataSource);
    REQUIRE(lhs.dataSource == rhs.dataSource);
    LOG_DEBUG("    [{}] == [{}] | BGD_E1_E5a", lhs.BGD_E1_E5a, rhs.BGD_E1_E5a);
    REQUIRE_THAT(lhs.BGD_E1_E5a, Catch::Matchers::EqualsSigDigits(rhs.BGD_E1_E5a, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | BGD_E1_E5b", lhs.BGD_E1_E5b, rhs.BGD_E1_E5b);
    REQUIRE_THAT(lhs.BGD_E1_E5b, Catch::Matchers::EqualsSigDigits(rhs.BGD_E1_E5b, 12)); // RINEX Format: D19.12
    return true;
}

inline bool operator==(const GLONASSEphemeris& lhs, const GLONASSEphemeris& rhs)
{
    // REQUIRE(lhs.tau_c == rhs.tau_c); // tau_c is not set in the reference data

    LOG_DEBUG("   [{}] == [{}] | toc", lhs.toc.toYMDHMS(GPST), rhs.toc.toYMDHMS(GPST));
    REQUIRE(lhs.toc == rhs.toc);
    LOG_DEBUG("    [{}] == [{}] | tau_n", lhs.tau_n, rhs.tau_n);
    REQUIRE_THAT(lhs.tau_n, Catch::Matchers::EqualsSigDigits(rhs.tau_n, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | gamma_n", lhs.gamma_n, rhs.gamma_n);
    REQUIRE_THAT(lhs.gamma_n, Catch::Matchers::EqualsSigDigits(rhs.gamma_n, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | health", lhs.health, rhs.health);
    REQUIRE(lhs.health == rhs.health);
    LOG_DEBUG("    [{}] == [{}] | PZ90_pos", lhs.PZ90_pos, rhs.PZ90_pos);
    REQUIRE_THAT(lhs.PZ90_pos, Catch::Matchers::EqualsSigDigitsContainer(rhs.PZ90_pos, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | PZ90_vel", lhs.PZ90_vel, rhs.PZ90_vel);
    REQUIRE_THAT(lhs.PZ90_vel, Catch::Matchers::EqualsSigDigitsContainer(rhs.PZ90_vel, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | PZ90_accelLuniSolar", lhs.PZ90_accelLuniSolar, rhs.PZ90_accelLuniSolar);
    REQUIRE_THAT(lhs.PZ90_accelLuniSolar, Catch::Matchers::EqualsSigDigitsContainer(rhs.PZ90_accelLuniSolar, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | frequencyNumber", lhs.frequencyNumber, rhs.frequencyNumber);
    REQUIRE(lhs.frequencyNumber == rhs.frequencyNumber);
    return true;
}

inline bool operator==(const BDSEphemeris& lhs, const BDSEphemeris& rhs)
{
    LOG_DEBUG("   [{}] == [{}] | toc", lhs.toc.toYMDHMS(GPST), rhs.toc.toYMDHMS(GPST));
    REQUIRE(lhs.toc == rhs.toc);
    LOG_DEBUG("   [{}] == [{}] | toe", lhs.toe.toYMDHMS(GPST), rhs.toe.toYMDHMS(GPST));
    REQUIRE(lhs.toe == rhs.toe);
    LOG_DEBUG("    [{}] == [{}] | AODE", lhs.AODE, rhs.AODE);
    REQUIRE(lhs.AODE == rhs.AODE);
    LOG_DEBUG("    [{}] == [{}] | AODC", lhs.AODC, rhs.AODC);
    REQUIRE(lhs.AODC == rhs.AODC);
    LOG_DEBUG("    [{}] == [{}] | a", joinToString(lhs.a), joinToString(rhs.a));
    REQUIRE_THAT(lhs.a, Catch::Matchers::EqualsSigDigitsContainer(rhs.a, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | sqrt_A", lhs.sqrt_A, rhs.sqrt_A);
    REQUIRE_THAT(lhs.sqrt_A, Catch::Matchers::EqualsSigDigits(rhs.sqrt_A, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | e", lhs.e, rhs.e);
    REQUIRE_THAT(lhs.e, Catch::Matchers::EqualsSigDigits(rhs.e, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_0", lhs.i_0, rhs.i_0);
    REQUIRE_THAT(lhs.i_0, Catch::Matchers::EqualsSigDigits(rhs.i_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_0", lhs.Omega_0, rhs.Omega_0);
    REQUIRE_THAT(lhs.Omega_0, Catch::Matchers::EqualsSigDigits(rhs.Omega_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | omega", lhs.omega, rhs.omega);
    REQUIRE_THAT(lhs.omega, Catch::Matchers::EqualsSigDigits(rhs.omega, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | M_0", lhs.M_0, rhs.M_0);
    REQUIRE_THAT(lhs.M_0, Catch::Matchers::EqualsSigDigits(rhs.M_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | delta_n", lhs.delta_n, rhs.delta_n);
    REQUIRE_THAT(lhs.delta_n, Catch::Matchers::EqualsSigDigits(rhs.delta_n, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_dot", lhs.Omega_dot, rhs.Omega_dot);
    REQUIRE_THAT(lhs.Omega_dot, Catch::Matchers::EqualsSigDigits(rhs.Omega_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_dot", lhs.i_dot, rhs.i_dot);
    REQUIRE_THAT(lhs.i_dot, Catch::Matchers::EqualsSigDigits(rhs.i_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cus", lhs.Cus, rhs.Cus);
    REQUIRE_THAT(lhs.Cus, Catch::Matchers::EqualsSigDigits(rhs.Cus, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cuc", lhs.Cuc, rhs.Cuc);
    REQUIRE_THAT(lhs.Cuc, Catch::Matchers::EqualsSigDigits(rhs.Cuc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cis", lhs.Cis, rhs.Cis);
    REQUIRE_THAT(lhs.Cis, Catch::Matchers::EqualsSigDigits(rhs.Cis, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cic", lhs.Cic, rhs.Cic);
    REQUIRE_THAT(lhs.Cic, Catch::Matchers::EqualsSigDigits(rhs.Cic, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crs", lhs.Crs, rhs.Crs);
    REQUIRE_THAT(lhs.Crs, Catch::Matchers::EqualsSigDigits(rhs.Crs, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crc", lhs.Crc, rhs.Crc);
    REQUIRE_THAT(lhs.Crc, Catch::Matchers::EqualsSigDigits(rhs.Crc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | svAccuracy", lhs.svAccuracy, rhs.svAccuracy);
    REQUIRE(lhs.svAccuracy == rhs.svAccuracy);
    LOG_DEBUG("    [{}] == [{}] | satH1", lhs.satH1, rhs.satH1);
    REQUIRE(lhs.satH1 == rhs.satH1);
    LOG_DEBUG("    [{}] == [{}] | T_GD1", lhs.T_GD1, rhs.T_GD1);
    REQUIRE_THAT(lhs.T_GD1, Catch::Matchers::EqualsSigDigits(rhs.T_GD1, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | T_GD2", lhs.T_GD2, rhs.T_GD2);
    REQUIRE_THAT(lhs.T_GD2, Catch::Matchers::EqualsSigDigits(rhs.T_GD2, 12)); // RINEX Format: D19.12
    return true;
}

inline bool operator==(const QZSSEphemeris& lhs, const QZSSEphemeris& rhs)
{
    LOG_DEBUG("   [{}] == [{}] | toc", lhs.toc.toYMDHMS(GPST), rhs.toc.toYMDHMS(GPST));
    REQUIRE(lhs.toc == rhs.toc);
    LOG_DEBUG("   [{}] == [{}] | toe", lhs.toe.toYMDHMS(GPST), rhs.toe.toYMDHMS(GPST));
    REQUIRE(lhs.toe == rhs.toe);
    LOG_DEBUG("    [{}] == [{}] | IODE", lhs.IODE, rhs.IODE);
    REQUIRE(lhs.IODE == rhs.IODE);
    LOG_DEBUG("    [{}] == [{}] | IODC", lhs.IODC, rhs.IODC);
    REQUIRE(lhs.IODC == rhs.IODC);
    LOG_DEBUG("    [{}] == [{}] | a", joinToString(lhs.a), joinToString(rhs.a));
    REQUIRE_THAT(lhs.a, Catch::Matchers::EqualsSigDigitsContainer(rhs.a, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | sqrt_A", lhs.sqrt_A, rhs.sqrt_A);
    REQUIRE_THAT(lhs.sqrt_A, Catch::Matchers::EqualsSigDigits(rhs.sqrt_A, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | e", lhs.e, rhs.e);
    REQUIRE_THAT(lhs.e, Catch::Matchers::EqualsSigDigits(rhs.e, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_0", lhs.i_0, rhs.i_0);
    REQUIRE_THAT(lhs.i_0, Catch::Matchers::EqualsSigDigits(rhs.i_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_0", lhs.Omega_0, rhs.Omega_0);
    REQUIRE_THAT(lhs.Omega_0, Catch::Matchers::EqualsSigDigits(rhs.Omega_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | omega", lhs.omega, rhs.omega);
    REQUIRE_THAT(lhs.omega, Catch::Matchers::EqualsSigDigits(rhs.omega, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | M_0", lhs.M_0, rhs.M_0);
    REQUIRE_THAT(lhs.M_0, Catch::Matchers::EqualsSigDigits(rhs.M_0, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | delta_n", lhs.delta_n, rhs.delta_n);
    REQUIRE_THAT(lhs.delta_n, Catch::Matchers::EqualsSigDigits(rhs.delta_n, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Omega_dot", lhs.Omega_dot, rhs.Omega_dot);
    REQUIRE_THAT(lhs.Omega_dot, Catch::Matchers::EqualsSigDigits(rhs.Omega_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | i_dot", lhs.i_dot, rhs.i_dot);
    REQUIRE_THAT(lhs.i_dot, Catch::Matchers::EqualsSigDigits(rhs.i_dot, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cus", lhs.Cus, rhs.Cus);
    REQUIRE_THAT(lhs.Cus, Catch::Matchers::EqualsSigDigits(rhs.Cus, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cuc", lhs.Cuc, rhs.Cuc);
    REQUIRE_THAT(lhs.Cuc, Catch::Matchers::EqualsSigDigits(rhs.Cuc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cis", lhs.Cis, rhs.Cis);
    REQUIRE_THAT(lhs.Cis, Catch::Matchers::EqualsSigDigits(rhs.Cis, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Cic", lhs.Cic, rhs.Cic);
    REQUIRE_THAT(lhs.Cic, Catch::Matchers::EqualsSigDigits(rhs.Cic, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crs", lhs.Crs, rhs.Crs);
    REQUIRE_THAT(lhs.Crs, Catch::Matchers::EqualsSigDigits(rhs.Crs, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | Crc", lhs.Crc, rhs.Crc);
    REQUIRE_THAT(lhs.Crc, Catch::Matchers::EqualsSigDigits(rhs.Crc, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | svAccuracy", lhs.svAccuracy, rhs.svAccuracy);
    REQUIRE(lhs.svAccuracy == rhs.svAccuracy);
    LOG_DEBUG("    [{}] == [{}] | svHealth", lhs.svHealth, rhs.svHealth);
    REQUIRE(lhs.svHealth == rhs.svHealth);
    LOG_DEBUG("    [{}] == [{}] | L2ChannelCodes", lhs.L2ChannelCodes, rhs.L2ChannelCodes);
    REQUIRE(lhs.L2ChannelCodes == rhs.L2ChannelCodes);
    LOG_DEBUG("    [{}] == [{}] | L2DataFlagPCode", lhs.L2DataFlagPCode, rhs.L2DataFlagPCode);
    REQUIRE(lhs.L2DataFlagPCode == rhs.L2DataFlagPCode);
    LOG_DEBUG("    [{}] == [{}] | T_GD", lhs.T_GD, rhs.T_GD);
    REQUIRE_THAT(lhs.T_GD, Catch::Matchers::EqualsSigDigits(rhs.T_GD, 12)); // RINEX Format: D19.12
    LOG_DEBUG("    [{}] == [{}] | fitInterval", lhs.fitIntervalFlag, rhs.fitIntervalFlag);
    REQUIRE(lhs.fitIntervalFlag == rhs.fitIntervalFlag);
    return true;
}

inline bool operator==(const std::shared_ptr<NAV::SatNavData>& lhs, const std::shared_ptr<NAV::SatNavData>& rhs)
{
    REQUIRE(lhs->type == rhs->type);
    REQUIRE(lhs->refTime == rhs->refTime);

    switch (lhs->type)
    {
    case SatNavData::Type::GPSEphemeris:
        REQUIRE(*std::dynamic_pointer_cast<GPSEphemeris>(lhs) == *std::dynamic_pointer_cast<GPSEphemeris>(rhs));
        break;
    case SatNavData::Type::GalileoEphemeris:
        REQUIRE(*std::dynamic_pointer_cast<GalileoEphemeris>(lhs) == *std::dynamic_pointer_cast<GalileoEphemeris>(rhs));
        break;
    case SatNavData::Type::GLONASSEphemeris:
        REQUIRE(*std::dynamic_pointer_cast<GLONASSEphemeris>(lhs) == *std::dynamic_pointer_cast<GLONASSEphemeris>(rhs));
        break;
    case SatNavData::Type::BeiDouEphemeris:
        REQUIRE(*std::dynamic_pointer_cast<BDSEphemeris>(lhs) == *std::dynamic_pointer_cast<BDSEphemeris>(rhs));
        break;
    case SatNavData::Type::QZSSEphemeris:
        REQUIRE(*std::dynamic_pointer_cast<QZSSEphemeris>(lhs) == *std::dynamic_pointer_cast<QZSSEphemeris>(rhs));
        break;
    default:
        FAIL(fmt::format("SatNavData::Type '{}' is not supported in the test yet. Please implement the comparison operator overload.", lhs->type));
    }
    return true;
}

inline bool operator==(const Satellite& lhs, const Satellite& rhs)
{
    LOG_DEBUG("  [{}] == [{}] | m_navigationData.size()", lhs.m_navigationData.size(), rhs.m_navigationData.size());
    LOG_DEBUG("  [{}] == [{}] | m_navigationData.refTime",
              joinToStringCustom(lhs.m_navigationData, [](const std::shared_ptr<SatNavData>& satNavData) -> std::string { return fmt::format("{}", satNavData->refTime.toYMDHMS(GPST)); }),
              joinToStringCustom(rhs.m_navigationData, [](const std::shared_ptr<SatNavData>& satNavData) -> std::string { return fmt::format("{}", satNavData->refTime.toYMDHMS(GPST)); }));
    REQUIRE(lhs.m_navigationData.size() == rhs.m_navigationData.size());
    LOG_DEBUG("  m_navigationData");
    REQUIRE(lhs.m_navigationData == rhs.m_navigationData);
    return true;
}

inline bool operator==(const GnssNavInfo& lhs, const GnssNavInfo& rhs)
{
    LOG_DEBUG("[{}] == [{}] | satelliteSystems", lhs.satelliteSystems, rhs.satelliteSystems);
    REQUIRE(lhs.satelliteSystems == rhs.satelliteSystems);
    LOG_DEBUG("ionosphericCorrections");
    REQUIRE(lhs.ionosphericCorrections == rhs.ionosphericCorrections);
    LOG_DEBUG("[{}] == [{}] | timeSysCorr.size()", lhs.timeSysCorr.size(), rhs.timeSysCorr.size());
    REQUIRE(lhs.timeSysCorr.size() == rhs.timeSysCorr.size());
    LOG_DEBUG("timeSysCorr");
    REQUIRE(lhs.timeSysCorr == rhs.timeSysCorr);
    LOG_DEBUG("[{}] == [{}] | m_satellites.size()", lhs.m_satellites.size(), rhs.m_satellites.size());
    REQUIRE(lhs.m_satellites.size() == rhs.m_satellites.size());
    LOG_DEBUG("m_satellites");
    for (const auto& [satId, sat] : lhs.m_satellites)
    {
        LOG_DEBUG(" [{}]", satId);
        REQUIRE(rhs.m_satellites.contains(satId));
        REQUIRE(sat == rhs.m_satellites.at(satId));
    }
    return true;
}

} // namespace NAV