// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GLONASSEphemeris.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "util/Logger.hpp"

namespace NAV
{

GLONASSEphemeris::GLONASSEphemeris(const InsTime& toc, double tau_c,
                                   double tau_n, double gamma_n, bool health,
                                   Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d accelLuniSolar,
                                   int8_t frequencyNumber)
    : SatNavData(SatNavData::GLONASSEphemeris, toc),
      tau_c(tau_c),
      toc(toc),
      tau_n(tau_n),
      gamma_n(gamma_n),
      health(health),
      pos(std::move(pos)),
      vel(std::move(vel)),
      accelLuniSolar(std::move(accelLuniSolar)),
      frequencyNumber(frequencyNumber) {}

#ifdef TESTING

GLONASSEphemeris::GLONASSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second,
                                   double tau_n, double gamma_n, double /* messageFrameTime */,
                                   double satPos_x, double satVel_x, double satAccel_x, double health,
                                   double satPos_y, double satVel_y, double satAccel_y, double frequencyNumber,
                                   double satPos_z, double satVel_z, double satAccel_z, double /* ageOfOperationInfo */,
                                   double /* statusFlags */, double /* L1L2groupDelayDifference */, double /* URAI */, double /* healthFlags */)
    : SatNavData(SatNavData::GLONASSEphemeris, InsTime(year, month, day, hour, minute, second, GLNT)),
      tau_c(0.0),
      toc(refTime),
      tau_n(tau_n),
      gamma_n(gamma_n),
      health(static_cast<bool>(health)),
      pos(satPos_x * 1e3, satPos_y * 1e3, satPos_z * 1e3),
      vel(satVel_x * 1e3, satVel_y * 1e3, satVel_z * 1e3),
      accelLuniSolar(satAccel_x * 1e3, satAccel_y * 1e3, satAccel_z * 1e3),
      frequencyNumber(static_cast<int8_t>(frequencyNumber))
{}

#endif

Orbit::Pos GLONASSEphemeris::calcSatellitePos(const InsTime& /* transTime */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

Orbit::PosVel GLONASSEphemeris::calcSatellitePosVel(const InsTime& /* transTime */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

Orbit::PosVelAccel GLONASSEphemeris::calcSatellitePosVelAccel(const InsTime& /* transTime */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

Clock::Corrections GLONASSEphemeris::calcClockCorrections(const InsTime& /* recvTime */, double /* dist */, const Frequency& /* freq */) const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

bool GLONASSEphemeris::isHealthy() const
{
    throw std::runtime_error("Not implemented yet!");
    return false;
}

double GLONASSEphemeris::calcSatellitePositionVariance() const
{
    throw std::runtime_error("Not implemented yet!");
    return {};
}

void GLONASSEphemeris::calcSatellitePosVelAccelClk(const InsTime& recvTime, double dist, double* clkBias, double* clkDrift, Eigen::Vector3d* e_pos, Eigen::Vector3d* e_vel, Eigen::Vector3d* e_accel) const
{
    LOG_DATA("Calc Sat Position at receiver time {} - {}", recvTime.toGPSweekTow(), recvTime.toYMDHMS());
    LOG_DATA("    Using ephemeris data at time {} - {}", toc.toGPSweekTow(), toc.toYMDHMS());

    // ----------------------------- Coordinate transformation PZ90 to absolute geocentric coordinate system -------------------------------

    // Skipped

    // ----------------------------------- Numerical integration with 4th order Runge-Kutta technique --------------------------------------

    // Calculates the position and velocity derivative for the satellite position
    //      y State [x, y, z, v_x, v_y, v_z]^T
    //      c Constant values needed to calculate the derivatives
    //      Returns the derivative ∂/∂t [x, y, z, v_x, v_y, v_z]^T
    auto calcPosVelDerivative = [](const Eigen::Matrix<double, 6, 1>& y, const Eigen::Vector3d& accelLuniSolar) {
        //       0  1  2   3    4    5
        // ∂/∂t [x, y, z, v_x, v_y, v_z]^T
        Eigen::Matrix<double, 6, 1> y_dot = Eigen::Matrix<double, 6, 1>::Zero();

        double r0 = std::sqrt(std::pow(y(0), 2) + std::pow(y(1), 2) + std::pow(y(2), 2));
        double x0 = y(0) / r0;
        double y0 = y(1) / r0;
        double z0 = y(2) / r0;
        double mu = InsConst::GLO::MU / std::pow(r0, 2);
        double rho = InsConst::GLO::a / r0;

        y_dot(0) = y(3);
        y_dot(1) = y(4);
        y_dot(2) = y(5);
        y_dot(3) = -mu * x0 + 3.0 / 2.0 * InsConst::GLO::C20 * mu * x0 * std::pow(rho, 2) * (1 - 5 * std::pow(z0, 2)) + accelLuniSolar.x();
        y_dot(4) = -mu * y0 + 3.0 / 2.0 * InsConst::GLO::C20 * mu * y0 * std::pow(rho, 2) * (1 - 5 * std::pow(z0, 2)) + accelLuniSolar.y();
        y_dot(5) = -mu * z0 + 3.0 / 2.0 * InsConst::GLO::C20 * mu * z0 * std::pow(rho, 2) * (1 - 5 * std::pow(z0, 2)) + accelLuniSolar.z();

        return y_dot;
    };

    // State [x, y, z, v_x, v_y, v_z]^T
    Eigen::Matrix<double, 6, 1> y;
    y << pos, vel;
    LOG_DATA("    pos {} (start state)", y.topRows<3>().transpose());
    LOG_DATA("    vel {} (start state)", y.bottomRows<3>().transpose());

    // Time at transmission
    InsTime transTime = recvTime - std::chrono::duration<double>(dist / InsConst::C);
    LOG_DATA("    transTime {} (Time at transmission)", transTime.toGPSweekTow());

    // SV clock time offset [s]
    double dt_sv = tau_c + tau_n - gamma_n * static_cast<double>((transTime - toc).count());
    LOG_DATA("    dt_sv {} [s] (SV clock time offset)", dt_sv);
    if (clkBias)
    {
        *clkBias = dt_sv;
    }
    if (clkDrift)
    {
        *clkDrift = -gamma_n;
    }

    // Time at transmission corrected
    InsTime transTime_c = transTime + std::chrono::duration<double>(dt_sv);

    // Time to integrate in [s]
    double dt = static_cast<double>((transTime_c - toc).count());
    LOG_DATA("      dt {} [s] (Time to integrate)", dt);

    while (std::abs(dt) > 1e-9)
    {
        double step = dt > 0 ? _h : -_h;
        if (std::abs(dt) < _h)
        {
            step = dt;
        }
        y = RungeKutta4(calcPosVelDerivative, step, y, accelLuniSolar);
        dt -= step;
    }

    LOG_DATA("    pos {} (end state)", y.topRows<3>().transpose());
    LOG_DATA("    vel {} (end state)", y.bottomRows<3>().transpose());

    // ---------------------------------- Coordinates transformation back to the PZ-90 reference system ------------------------------------

    // Skipped

    // --------------------------------- Coordinates transformation PZ-90 to ECEF WGS84 reference system -----------------------------------

    if (e_pos)
    {
        *e_pos = trafo::pz90toWGS84_pos(y.topRows<3>());
        LOG_DATA("    pos (WGS84) {}", e_pos->transpose());
    }

    if (e_vel)
    {
        *e_vel = trafo::pz90toWGS84(y.bottomRows<3>(), y.topRows<3>());
        LOG_DATA("    vel (WGS84) {}", e_vel->transpose());
    }

    if (e_accel)
    {
        Eigen::Matrix<double, 6, 1> y_dot = calcPosVelDerivative(y, accelLuniSolar);
        *e_accel = trafo::pz90toWGS84(y_dot.bottomRows<3>(), y.topRows<3>());
        LOG_DATA("    accel (WGS84) {}", e_accel->transpose());
    }
}

} // namespace NAV