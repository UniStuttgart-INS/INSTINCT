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
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/GnssNavInfo.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"

namespace NAV
{

inline bool operator==(const IonosphericCorrections::Corrections& lhs, const IonosphericCorrections::Corrections& rhs)
{
    REQUIRE(lhs.satSys == rhs.satSys);
    REQUIRE(lhs.alphaBeta == rhs.alphaBeta);
    REQUIRE(lhs.data == rhs.data);
    return true;
}

inline bool operator==(const IonosphericCorrections& lhs, const IonosphericCorrections& rhs)
{
    REQUIRE(lhs.data().size() == rhs.data().size());
    REQUIRE(lhs.data() == rhs.data());
    return true;
}
inline bool operator==(const GnssNavInfo::TimeSystemCorrections& lhs, const GnssNavInfo::TimeSystemCorrections& rhs)
{
    REQUIRE(lhs.a0 == rhs.a0);
    REQUIRE(lhs.a1 == rhs.a1);
    return true;
}

inline bool operator==(const GPSEphemeris& lhs, const GPSEphemeris& rhs)
{
    REQUIRE(lhs.toc == rhs.toc);
    REQUIRE(lhs.toe == rhs.toe);
    REQUIRE(lhs.IODE == rhs.IODE);
    REQUIRE(lhs.IODC == rhs.IODC);
    REQUIRE(lhs.a == rhs.a);
    REQUIRE(lhs.sqrt_A == rhs.sqrt_A);
    REQUIRE(lhs.e == rhs.e);
    REQUIRE(lhs.i_0 == rhs.i_0);
    REQUIRE(lhs.Omega_0 == rhs.Omega_0);
    REQUIRE(lhs.omega == rhs.omega);
    REQUIRE(lhs.M_0 == rhs.M_0);
    REQUIRE(lhs.delta_n == rhs.delta_n);
    REQUIRE(lhs.Omega_dot == rhs.Omega_dot);
    REQUIRE(lhs.i_dot == rhs.i_dot);
    REQUIRE(lhs.Cus == rhs.Cus);
    REQUIRE(lhs.Cuc == rhs.Cuc);
    REQUIRE(lhs.Cis == rhs.Cis);
    REQUIRE(lhs.Cic == rhs.Cic);
    REQUIRE(lhs.Crs == rhs.Crs);
    REQUIRE(lhs.Crc == rhs.Crc);
    REQUIRE(lhs.svAccuracy == rhs.svAccuracy);
    REQUIRE(lhs.svHealth == rhs.svHealth);
    REQUIRE(lhs.L2ChannelCodes == rhs.L2ChannelCodes);
    REQUIRE(lhs.L2DataFlagPCode == rhs.L2DataFlagPCode);
    REQUIRE(lhs.T_GD == rhs.T_GD);
    REQUIRE(lhs.fitInterval == rhs.fitInterval);
    return true;
}

inline bool operator==(const GalileoEphemeris& lhs, const GalileoEphemeris& rhs)
{
    REQUIRE(lhs.toc == rhs.toc);
    REQUIRE(lhs.toe == rhs.toe);
    REQUIRE(lhs.IODnav == rhs.IODnav);
    REQUIRE(lhs.a == rhs.a);
    REQUIRE(lhs.sqrt_A == rhs.sqrt_A);
    REQUIRE(lhs.e == rhs.e);
    REQUIRE(lhs.i_0 == rhs.i_0);
    REQUIRE(lhs.Omega_0 == rhs.Omega_0);
    REQUIRE(lhs.omega == rhs.omega);
    REQUIRE(lhs.M_0 == rhs.M_0);
    REQUIRE(lhs.delta_n == rhs.delta_n);
    REQUIRE(lhs.Omega_dot == rhs.Omega_dot);
    REQUIRE(lhs.i_dot == rhs.i_dot);
    REQUIRE(lhs.Cus == rhs.Cus);
    REQUIRE(lhs.Cuc == rhs.Cuc);
    REQUIRE(lhs.Cis == rhs.Cis);
    REQUIRE(lhs.Cic == rhs.Cic);
    REQUIRE(lhs.Crs == rhs.Crs);
    REQUIRE(lhs.Crc == rhs.Crc);
    REQUIRE(lhs.signalAccuracy == rhs.signalAccuracy);
    REQUIRE(lhs.svHealth.E5a_DataValidityStatus == rhs.svHealth.E5a_DataValidityStatus);
    REQUIRE(lhs.svHealth.E5b_DataValidityStatus == rhs.svHealth.E5b_DataValidityStatus);
    REQUIRE(lhs.svHealth.E1B_DataValidityStatus == rhs.svHealth.E1B_DataValidityStatus);
    REQUIRE(lhs.svHealth.E5a_SignalHealthStatus == rhs.svHealth.E5a_SignalHealthStatus);
    REQUIRE(lhs.svHealth.E5b_SignalHealthStatus == rhs.svHealth.E5b_SignalHealthStatus);
    REQUIRE(lhs.svHealth.E1BC_SignalHealthStatus == rhs.svHealth.E1BC_SignalHealthStatus);
    REQUIRE(lhs.dataSource == rhs.dataSource);
    REQUIRE(lhs.BGD_E1_E5a == rhs.BGD_E1_E5a);
    REQUIRE(lhs.BGD_E1_E5b == rhs.BGD_E1_E5b);
    return true;
}

inline bool operator==(const GLONASSEphemeris& lhs, const GLONASSEphemeris& rhs)
{
    // REQUIRE(lhs.tau_c == rhs.tau_c); // tau_c is not set in the reference data
    REQUIRE(lhs.toc == rhs.toc);
    REQUIRE(lhs.tau_n == rhs.tau_n);
    REQUIRE(lhs.gamma_n == rhs.gamma_n);
    REQUIRE(lhs.health == rhs.health);
    REQUIRE(lhs.PZ90_pos == rhs.PZ90_pos);
    REQUIRE(lhs.PZ90_vel == rhs.PZ90_vel);
    REQUIRE(lhs.PZ90_accelLuniSolar == rhs.PZ90_accelLuniSolar);
    REQUIRE(lhs.frequencyNumber == rhs.frequencyNumber);
    return true;
}

inline bool operator==(const BDSEphemeris& lhs, const BDSEphemeris& rhs)
{
    REQUIRE(lhs.toc == rhs.toc);
    REQUIRE(lhs.toe == rhs.toe);
    REQUIRE(lhs.AODE == rhs.AODE);
    REQUIRE(lhs.AODC == rhs.AODC);
    REQUIRE(lhs.a == rhs.a);
    REQUIRE(lhs.sqrt_A == rhs.sqrt_A);
    REQUIRE(lhs.e == rhs.e);
    REQUIRE(lhs.i_0 == rhs.i_0);
    REQUIRE(lhs.Omega_0 == rhs.Omega_0);
    REQUIRE(lhs.omega == rhs.omega);
    REQUIRE(lhs.M_0 == rhs.M_0);
    REQUIRE(lhs.delta_n == rhs.delta_n);
    REQUIRE(lhs.Omega_dot == rhs.Omega_dot);
    REQUIRE(lhs.i_dot == rhs.i_dot);
    REQUIRE(lhs.Cus == rhs.Cus);
    REQUIRE(lhs.Cuc == rhs.Cuc);
    REQUIRE(lhs.Cis == rhs.Cis);
    REQUIRE(lhs.Cic == rhs.Cic);
    REQUIRE(lhs.Crs == rhs.Crs);
    REQUIRE(lhs.Crc == rhs.Crc);
    REQUIRE(lhs.svAccuracy == rhs.svAccuracy);
    REQUIRE(lhs.satH1 == rhs.satH1);
    REQUIRE(lhs.T_GD1 == rhs.T_GD1);
    REQUIRE(lhs.T_GD2 == rhs.T_GD2);
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
    default:
        FAIL(fmt::format("SatNavData::Type '{}' is not supported in the test yet. Please implement the comparison operator overload.", lhs->type));
    }
    return true;
}

inline bool operator==(const Satellite& lhs, const Satellite& rhs)
{
    REQUIRE(lhs.m_navigationData.size() == rhs.m_navigationData.size());
    REQUIRE(lhs.m_navigationData == rhs.m_navigationData);
    return true;
}

inline bool operator==(const GnssNavInfo& lhs, const GnssNavInfo& rhs)
{
    REQUIRE(lhs.satelliteSystems == rhs.satelliteSystems);
    REQUIRE(lhs.ionosphericCorrections == rhs.ionosphericCorrections);
    REQUIRE(lhs.timeSysCorr.size() == rhs.timeSysCorr.size());
    REQUIRE(lhs.timeSysCorr == rhs.timeSysCorr);
    REQUIRE(lhs.m_satellites.size() == rhs.m_satellites.size());
    REQUIRE(lhs.m_satellites == rhs.m_satellites);
    return true;
}

} // namespace NAV