// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "BDSEphemeris.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Functions.hpp"

#include "util/Logger.hpp"

namespace NAV
{

BDSEphemeris::BDSEphemeris(const InsTime& toc, const InsTime& toe,
                           const size_t& AODE, const size_t& AODC,
                           const std::array<double, 3>& a,
                           const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                           const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                           const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                           const double& svAccuracy, uint8_t satH1, double T_GD1, double T_GD2)
    : SatNavData(SatNavData::BeiDouEphemeris, toc),
      toc(toc),
      toe(toe),
      AODE(AODE),
      AODC(AODC),
      a(a),
      sqrt_A(sqrt_A),
      e(e),
      i_0(i_0),
      Omega_0(Omega_0),
      omega(omega),
      M_0(M_0),
      delta_n(delta_n),
      Omega_dot(Omega_dot),
      i_dot(i_dot),
      Cus(Cus),
      Cuc(Cuc),
      Cis(Cis),
      Cic(Cic),
      Crs(Crs),
      Crc(Crc),
      svAccuracy(svAccuracy),
      satH1(satH1),
      T_GD1(T_GD1),
      T_GD2(T_GD2) {}

#ifdef TESTING

BDSEphemeris::BDSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                           double AODE, double Crs, double delta_n, double M_0,
                           double Cuc, double e, double Cus, double sqrt_A,
                           double Toe, double Cic, double Omega_0, double Cis,
                           double i_0, double Crc, double omega, double Omega_dot,
                           double i_dot, double /* spare1 */, double BDTWeek, double /* spare2 */,
                           double svAccuracy, double satH1, double T_GD1, double T_GD2,
                           double /* TransmissionTimeOfMessage */, double AODC, double /* spare3 */, double /* spare4 */)
    : SatNavData(SatNavData::BeiDouEphemeris, InsTime(year, month, day, hour, minute, second, BDT)),
      toc(refTime),
      toe(InsTime(0, static_cast<int32_t>(BDTWeek) + InsTimeUtil::DIFF_BDT_WEEK_TO_GPST_WEEK, Toe, BDT)),
      AODE(static_cast<size_t>(AODE)),
      AODC(static_cast<size_t>(AODC)),
      a({ svClockBias, svClockDrift, svClockDriftRate }),
      sqrt_A(sqrt_A),
      e(e),
      i_0(i_0),
      Omega_0(Omega_0),
      omega(omega),
      M_0(M_0),
      delta_n(delta_n),
      Omega_dot(Omega_dot),
      i_dot(i_dot),
      Cus(Cus),
      Cuc(Cuc),
      Cis(Cis),
      Cic(Cic),
      Crs(Crs),
      Crc(Crc),
      svAccuracy(svAccuracy),
      satH1(static_cast<uint8_t>(satH1)),
      T_GD1(T_GD1),
      T_GD2(T_GD2)
{}

#endif

Orbit::Pos BDSEphemeris::calcSatellitePos(const InsTime& /* transTime */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

Orbit::PosVel BDSEphemeris::calcSatellitePosVel(const InsTime& /* transTime */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

Orbit::PosVelAccel BDSEphemeris::calcSatellitePosVelAccel(const InsTime& /* transTime */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

Clock::Corrections BDSEphemeris::calcClockCorrections(const InsTime& /* recvTime */, double /* dist */, const Frequency& /* freq */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

bool BDSEphemeris::isHealthy() const
{
    return false;
    // TODO: satH1 == 0;
}

double BDSEphemeris::calcSatellitePositionVariance() const
{
    // Getting the index and value again will discretize the URA values
    return std::pow(gpsUraIdx2Val(gpsUraVal2Idx(svAccuracy)), 2);
}

} // namespace NAV