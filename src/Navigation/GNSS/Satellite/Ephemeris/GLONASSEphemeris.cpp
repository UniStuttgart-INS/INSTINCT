// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GLONASSEphemeris.hpp"
#include <cstdint>

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
      PZ90_pos(std::move(pos)),
      PZ90_vel(std::move(vel)),
      PZ90_accelLuniSolar(std::move(accelLuniSolar)),
      frequencyNumber(frequencyNumber) {}

#ifdef TESTING

GLONASSEphemeris::GLONASSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second,
                                   double m_tau_n, double gamma_n, double /* messageFrameTime */,
                                   double satPos_x, double satVel_x, double satAccel_x, double health,
                                   double satPos_y, double satVel_y, double satAccel_y, double frequencyNumber,
                                   double satPos_z, double satVel_z, double satAccel_z, double /* ageOfOperationInfo */,
                                   double /* statusFlags */, double /* L1L2groupDelayDifference */, double /* URAI */, double /* healthFlags */,
                                   double tau_c)
    : SatNavData(SatNavData::GLONASSEphemeris, InsTime(year, month, day, hour, minute, second, UTC)),
      tau_c(tau_c),
      toc(refTime),
      tau_n(-m_tau_n),
      gamma_n(gamma_n),
      health(static_cast<bool>(health)),
      PZ90_pos(satPos_x * 1e3, satPos_y * 1e3, satPos_z * 1e3),
      PZ90_vel(satVel_x * 1e3, satVel_y * 1e3, satVel_z * 1e3),
      PZ90_accelLuniSolar(satAccel_x * 1e3, satAccel_y * 1e3, satAccel_z * 1e3),
      frequencyNumber(static_cast<int8_t>(frequencyNumber))
{}

#endif

Clock::Corrections GLONASSEphemeris::calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& /* freq */) const
{
    LOG_DATA("Calc Sat Clock corrections at receiver time {}", recvTime.toGPSweekTow());
    LOG_DATA("    toc {} (Time of clock)", toc.toGPSweekTow());

    // Time at transmission
    InsTime transTime = recvTime - std::chrono::duration<double>(dist / InsConst::C);

    // SV clock time offset [s]
    double dt_sv = -tau_n + gamma_n * static_cast<double>((transTime - toc).count());
    LOG_DATA("    dt_sv {} [s] (SV clock time offset)", dt_sv);

    transTime += std::chrono::duration<double>(dt_sv);
    LOG_DATA("    transTime {} (Time at transmission)", transTime.toGPSweekTow());

    return { .transmitTime = transTime, .bias = dt_sv, .drift = -gamma_n };
}

Orbit::PosVelAccel GLONASSEphemeris::calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const
{
    LOG_DATA("Calc Sat Position at transmit time {}", transTime.toGPSweekTow());
    LOG_DATA("    toc {} (Time of clock)", toc.toGPSweekTow());

    // Calculates the position and velocity derivative for the satellite position
    //      y State [x, y, z, v_x, v_y, v_z]^T
    //      c Constant values needed to calculate the derivatives
    //      Returns the derivative ∂/∂t [x, y, z, v_x, v_y, v_z]^T
    auto calcPosVelDerivative = [](const Eigen::Matrix<double, 6, 1>& y, int /* z */, const Eigen::Vector3d& accelLuniSolar, double /* t */ = 0.0) {
        //       0  1  2   3    4    5
        // ∂/∂t [x, y, z, v_x, v_y, v_z]^T
        Eigen::Matrix<double, 6, 1> y_dot = Eigen::Matrix<double, 6, 1>::Zero();

        enum State : uint8_t
        {
            X,
            Y,
            Z,
            VX,
            VY,
            VZ
        };

        double r = y.topRows<3>().norm();

        double omega_ie2 = std::pow(InsConst::GLO::omega_ie, 2);

        double a = 1.5 * InsConst::GLO::J2 * InsConst::GLO::MU * std::pow(InsConst::GLO::a, 2) / std::pow(r, 5);
        double c = -InsConst::GLO::MU / std::pow(r, 3) - a * (1. - 5. * std::pow(y(Z), 2) / std::pow(r, 2));

        y_dot.topRows<3>() = y.bottomRows<3>();
        y_dot(3) = (c + omega_ie2) * y(X) + 2 * InsConst::GLO::omega_ie * y(VY) + accelLuniSolar.x();
        y_dot(4) = (c + omega_ie2) * y(Y) - 2 * InsConst::GLO::omega_ie * y(VX) + accelLuniSolar.y();
        y_dot(5) = (c - 2. * a) * y(Z) + accelLuniSolar.z();

        return y_dot;
    };

    // State [x, y, z, v_x, v_y, v_z]^T
    Eigen::Matrix<double, 6, 1> y;
    y << PZ90_pos, PZ90_vel;

    // Time to integrate in [s]
    double dt = static_cast<double>((transTime - toc).count());
    LOG_DATA("    dt {} [s] (Time to integrate)", dt);

    while (std::abs(dt) > 1e-9)
    {
        double step = dt > 0 ? _h : -_h;
        if (std::abs(dt) < _h)
        {
            step = dt;
        }
        LOG_DATA("    step {:0.2f}, pos {}, vel {}", step, y.topRows<3>().transpose(), y.bottomRows<3>().transpose());
        ;
        y = RungeKutta4(y, std::array<int, 4>{}, step, calcPosVelDerivative, PZ90_accelLuniSolar);
        dt -= step;
    }
    LOG_DATA("    pos {}, vel {} (end state)", y.topRows<3>().transpose(), y.bottomRows<3>().transpose());

    Eigen::Vector3d e_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_accel = Eigen::Vector3d::Zero();

    if (calc & Calc_Position)
    {
        e_pos = y.topRows<3>(); // trafo::pz90toWGS84_pos(y.topRows<3>());
        // LOG_DATA("    pos (WGS84) {}", e_pos.transpose());
    }
    if (calc & Calc_Velocity)
    {
        e_vel = y.bottomRows<3>(); // trafo::pz90toWGS84(y.bottomRows<3>(), y.topRows<3>());
        // LOG_DATA("    vel (WGS84) {}", e_vel.transpose());
    }
    if (calc & Calc_Acceleration)
    {
        Eigen::Matrix<double, 6, 1> y_dot = calcPosVelDerivative(y, 0, PZ90_accelLuniSolar, 0.0);
        e_accel = y_dot.bottomRows<3>(); // trafo::pz90toWGS84(y_dot.bottomRows<3>(), y.topRows<3>());
        // LOG_DATA("    accel (WGS84) {}", e_accel.transpose());
    }

    return { .e_pos = e_pos,
             .e_vel = e_vel,
             .e_accel = e_accel };
}

bool GLONASSEphemeris::isHealthy() const
{
    return !health;
}

double GLONASSEphemeris::calcSatellitePositionVariance() const
{
    return 5 * 5;
}

} // namespace NAV