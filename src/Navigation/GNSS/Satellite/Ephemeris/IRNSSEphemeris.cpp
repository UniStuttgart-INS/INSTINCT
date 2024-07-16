// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IRNSSEphemeris.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Functions.hpp"

#include "util/Logger.hpp"

namespace NAV
{

IRNSSEphemeris::IRNSSEphemeris(const InsTime& toc, const InsTime& toe,
                               const size_t& IODEC,
                               const std::array<double, 3>& a,
                               const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                               const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                               const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                               const double& svAccuracy, uint8_t svHealth,
                               const double& T_GD)
    : SatNavData(SatNavData::IRNSSEphemeris, toc),
      toc(toc),
      toe(toe),
      IODEC(IODEC),
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
      svHealth(svHealth),
      T_GD(T_GD) {}

#ifdef TESTING

IRNSSEphemeris::IRNSSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                               double IODEC, double Crs, double delta_n, double M_0,
                               double Cuc, double e, double Cus, double sqrt_A,
                               double Toe, double Cic, double Omega_0, double Cis,
                               double i_0, double Crc, double omega, double Omega_dot,
                               double i_dot, double /*spare1*/, double IRNWeek, double /*spare2*/,
                               double svAccuracy, double svHealth, double T_GD, double /*spare3*/,
                               double /*TransmissionTimeOfMessage*/, double /*spare4*/, double /*spare5*/, double /*spare6*/)
    : SatNavData(SatNavData::IRNSSEphemeris, InsTime(year, month, day, hour, minute, second, SatelliteSystem(IRNSS).getTimeSystem())),
      toc(refTime),
      toe(InsTime(0, static_cast<int32_t>(IRNWeek), Toe, SatelliteSystem(IRNSS).getTimeSystem())),
      IODEC(static_cast<size_t>(IODEC)),
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
      svHealth(static_cast<uint8_t>(svHealth)),
      T_GD(T_GD)
{}

#endif

Clock::Corrections IRNSSEphemeris::calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const
{
    LOG_DATA("Calc Sat Clock corrections at receiver time {}", recvTime.toGPSweekTow());
    // Earth gravitational constant [m³/s²]
    const auto mu = InsConst<>::IRNSS::MU;
    // Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
    const auto F = InsConst<>::IRNSS::F;

    LOG_DATA("    toe {} (Time of ephemeris)", toe.toGPSweekTow());

    const auto A = sqrt_A * sqrt_A; // Semi-major axis [m]
    LOG_DATA("    A {} [m] (Semi-major axis)", A);
    auto n_0 = std::sqrt(mu / std::pow(A, 3)); // Computed mean motion [rad/s]
    LOG_DATA("    n_0 {} [rad/s] (Computed mean motion)", n_0);
    auto n = n_0 + delta_n; // Corrected mean motion [rad/s]
    LOG_DATA("    n {} [rad/s] (Corrected mean motion)", n);

    // Time at transmission
    InsTime transTime0 = recvTime - std::chrono::duration<double>(dist / InsConst<>::C);

    InsTime transTime = transTime0;
    LOG_DATA("    Iterating Time at transmission");
    double dt_sv = 0.0;
    double clkDrift = 0.0;

    for (size_t i = 0; i < 2; i++)
    {
        LOG_DATA("      transTime {} (Time at transmission)", transTime.toGPSweekTow());

        // [s]
        auto t_minus_toc = static_cast<double>((transTime - toc).count());
        LOG_DATA("      transTime - toc {} [s]", t_minus_toc);

        // Time difference from ephemeris reference epoch [s]
        double t_k = static_cast<double>((transTime - toe).count());
        LOG_DATA("      transTime - toe {} [s] (t_k = Time difference from ephemeris reference epoch)", t_k);

        // Mean anomaly [rad]
        auto M_k = M_0 + n * t_k;
        LOG_DATA("      M_k {} [s] (Mean anomaly)", M_k);

        // Eccentric anomaly [rad]
        double E_k = M_k;
        double E_k_old = 0.0;

        for (size_t i = 0; std::abs(E_k - E_k_old) > 1e-13 && i < 10; i++)
        {
            E_k_old = E_k; // Kepler’s equation ( Mk = E_k − e sin E_k ) may be solved for Eccentric anomaly (E_k) by iteration:
            E_k = M_k + e * sin(E_k);
        }

        // Relativistic correction term [s]
        double dt_r = F * e * sqrt_A * std::sin(E_k);
        LOG_DATA("      dt_r {} [s] (Relativistic correction term)", dt_r);

        // SV PRN code phase time offset [s]
        dt_sv = a[0] + a[1] * t_minus_toc + a[2] * std::pow(t_minus_toc, 2) + dt_r;

        // See /cite IRNSS-SIS-ICD-1.1 IRNSS ICD, ch. 6.2.1.5, p.31
        dt_sv -= ratioFreqSquared(S01, freq, -128, -128) * T_GD;

        LOG_DATA("      dt_sv {} [s] (SV PRN code phase time offset)", dt_sv);

        // Groves ch. 9.3.1, eq. 9.78, p. 391
        clkDrift = a[1] + a[2] / 2.0 * t_minus_toc;

        // Correct transmit time for the satellite clock bias
        transTime = transTime0 - std::chrono::duration<double>(dt_sv);
    }
    LOG_DATA("      transTime {} (Time at transmission)", transTime.toGPSweekTow());

    return { .transmitTime = transTime, .bias = dt_sv, .drift = clkDrift };
}

Orbit::PosVelAccel IRNSSEphemeris::calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const
{
    Eigen::Vector3d e_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d e_accel = Eigen::Vector3d::Zero();

    LOG_DATA("Calc Sat Position at transmit time {}", transTime.toGPSweekTow());
    // Earth gravitational constant [m³/s²] (WGS 84 value of the earth's gravitational constant for GPS user)
    const auto mu = InsConst<>::GPS::MU;
    // Earth angular velocity [rad/s] (WGS 84 value of the earth's rotation rate)
    const auto Omega_e_dot = InsConst<>::GPS::omega_ie;

    LOG_DATA("    toe {} (Time of ephemeris)", toe.toGPSweekTow());

    const auto A = sqrt_A * sqrt_A; // Semi-major axis [m]
    LOG_DATA("    A {} [m] (Semi-major axis)", A);
    auto n_0 = std::sqrt(mu / std::pow(A, 3)); // Computed mean motion [rad/s]
    LOG_DATA("    n_0 {} [rad/s] (Computed mean motion)", n_0);
    auto n = n_0 + delta_n; // Corrected mean motion [rad/s]
    LOG_DATA("    n {} [rad/s] (Corrected mean motion)", n);

    // Eccentric anomaly [rad]
    double E_k = 0.0;

    // Time difference from ephemeris reference epoch [s]
    double t_k = static_cast<double>((transTime - toe).count());
    LOG_DATA("    t_k {} [s] (Time difference from ephemeris reference epoch)", t_k);

    // Mean anomaly [rad]
    auto M_k = M_0 + n * t_k;
    LOG_DATA("    M_k {} [s] (Mean anomaly)", M_k);

    E_k = M_k; // Initial Value [rad]
    double E_k_old = 0.0;
    LOG_DATA("    Iterating E_k");
    LOG_DATA("      E_k {} [rad] (Eccentric anomaly)", E_k);
    for (size_t i = 0; std::abs(E_k - E_k_old) > 1e-13 && i < 10; i++)
    {
        E_k_old = E_k;                                                         // Kepler’s equation ( Mk = E_k − e sin E_k ) may be solved for Eccentric anomaly (E_k) by iteration:
        E_k = E_k + (M_k - E_k + e * std::sin(E_k)) / (1 - e * std::cos(E_k)); // – Refined Value, minimum of three iterations, (j=1,2,3)
        LOG_DATA("      E_k {} [rad] (Eccentric anomaly)", E_k);               // – Final Value (radians)
    }

    // auto v_k = 2.0 * std::atan(std::sqrt((1.0 + e) / (1.0 - e)) * std::tan(E_k / 2.0)); // True Anomaly (unambiguous quadrant) [rad] (GPS ICD algorithm)
    // auto v_k = std::atan2(std::sqrt(1 - e * e) * std::sin(E_k) / (1 - e * std::cos(E_k)), (std::cos(E_k) - e) / (1 - e * std::cos(E_k))); // True Anomaly [rad] (GALILEO ICD algorithm)
    auto v_k = std::atan2(std::sqrt(1 - e * e) * std::sin(E_k), (std::cos(E_k) - e)); // True Anomaly [rad] // simplified, since the denominators cancel out
    LOG_DATA("    v_k {} [rad] (True Anomaly (unambiguous quadrant))", v_k);
    auto Phi_k = v_k + omega; // Argument of Latitude [rad]
    LOG_DATA("    Phi_k {} [rad] (Argument of Latitude)", Phi_k);

    // Second Harmonic Perturbations
    auto delta_u_k = Cus * std::sin(2 * Phi_k) + Cuc * std::cos(2 * Phi_k); // Argument of Latitude Correction [rad]
    LOG_DATA("    delta_u_k {} [rad] (Argument of Latitude Correction)", delta_u_k);
    auto delta_r_k = Crs * std::sin(2 * Phi_k) + Crc * std::cos(2 * Phi_k); // Radius Correction [m]
    LOG_DATA("    delta_r_k {} [m] (Radius Correction)", delta_r_k);
    auto delta_i_k = Cis * std::sin(2 * Phi_k) + Cic * std::cos(2 * Phi_k); // Inclination Correction [rad]
    LOG_DATA("    delta_i_k {} [rad] (Inclination Correction)", delta_i_k);

    auto u_k = Phi_k + delta_u_k; // Corrected Argument of Latitude [rad]
    LOG_DATA("    u_k {} [rad] (Corrected Argument of Latitude)", u_k);
    auto r_k = A * (1 - e * std::cos(E_k)) + delta_r_k; // Corrected Radius [m]
    LOG_DATA("    r_k {} [m] (Corrected Radius)", r_k);
    auto i_k = i_0 + delta_i_k + i_dot * t_k; // Corrected Inclination [rad]
    LOG_DATA("    i_k {} [rad] (Corrected Inclination)", i_k);

    auto x_k_op = r_k * std::cos(u_k); // Position in orbital plane [m]
    LOG_DATA("    x_k_op {} [m] (Position in orbital plane)", x_k_op);
    auto y_k_op = r_k * std::sin(u_k); // Position in orbital plane [m]
    LOG_DATA("    y_k_op {} [m] (Position in orbital plane)", y_k_op);

    // Corrected longitude of ascending node [rad]
    auto Omega_k = Omega_0 + (Omega_dot - Omega_e_dot) * t_k - Omega_e_dot * static_cast<double>(toe.toGPSweekTow(IRNSST).tow);
    LOG_DATA("    Omega_k {} [rad] (Corrected longitude of ascending node)", Omega_k);

    // Earth-fixed x coordinates [m]
    auto x_k = x_k_op * std::cos(Omega_k) - y_k_op * std::cos(i_k) * std::sin(Omega_k);
    LOG_DATA("    x_k {} [m] (Earth-fixed x coordinates)", x_k);
    // Earth-fixed y coordinates [m]
    auto y_k = x_k_op * std::sin(Omega_k) + y_k_op * std::cos(i_k) * std::cos(Omega_k);
    LOG_DATA("    y_k {} [m] (Earth-fixed y coordinates)", y_k);
    // Earth-fixed z coordinates [m]
    auto z_k = y_k_op * std::sin(i_k);
    LOG_DATA("    z_k {} [m] (Earth-fixed z coordinates)", z_k);

    e_pos = Eigen::Vector3d{ x_k, y_k, z_k };

    if (calc & Calc_Velocity || calc & Calc_Acceleration)
    {
        // Eccentric Anomaly Rate [rad/s]
        auto E_k_dot = n / (1 - e * std::cos(E_k));
        // True Anomaly Rate [rad/s]
        auto v_k_dot = E_k_dot * std::sqrt(1 - e * e) / (1 - e * std::cos(E_k));
        // Corrected Inclination Angle Rate [rad/s]
        auto i_k_dot = i_dot + 2 * v_k_dot * (Cis * std::cos(2 * Phi_k) - Cic * std::sin(2 * Phi_k));
        // Corrected Argument of Latitude Rate [rad/s]
        auto u_k_dot = v_k_dot + 2 * v_k_dot * (Cus * std::cos(2 * Phi_k) - Cuc * std::sin(2 * Phi_k));
        // Corrected Radius Rate [m/s]
        auto r_k_dot = e * A * E_k_dot * std::sin(E_k) + 2 * v_k_dot * (Crs * std::cos(2 * Phi_k) - Crc * std::sin(2 * Phi_k));
        // Longitude of Ascending Node Rate [rad/s]
        auto Omega_k_dot = Omega_dot - Omega_e_dot;
        // In-plane x velocity [m/s]
        auto vx_k_op = r_k_dot * std::cos(u_k) - r_k * u_k_dot * std::sin(u_k);
        // In-plane y velocity [m/s]
        auto vy_k_op = r_k_dot * std::sin(u_k) + r_k * u_k_dot * std::cos(u_k);
        // Earth-Fixed x velocity [m/s]
        auto vx_k = -x_k_op * Omega_k_dot * std::sin(Omega_k) + vx_k_op * std::cos(Omega_k) - vy_k_op * std::sin(Omega_k) * std::cos(i_k)
                    - y_k_op * (Omega_k_dot * std::cos(Omega_k) * std::cos(i_k) - i_k_dot * std::sin(Omega_k) * std::sin(i_k));
        // Earth-Fixed y velocity [m/s]
        auto vy_k = x_k_op * Omega_k_dot * std::cos(Omega_k) + vx_k_op * std::sin(Omega_k) + vy_k_op * std::cos(Omega_k) * std::cos(i_k)
                    - y_k_op * (Omega_k_dot * std::sin(Omega_k) * std::cos(i_k) + i_k_dot * std::cos(Omega_k) * std::sin(i_k));
        // Earth-Fixed z velocity [m/s]
        auto vz_k = vy_k_op * std::sin(i_k) + y_k_op * i_k_dot * std::cos(i_k);

        if (calc & Calc_Velocity)
        {
            e_vel = Eigen::Vector3d{ vx_k, vy_k, vz_k };
        }

        if (calc & Calc_Acceleration)
        {
            // Oblate Earth acceleration Factor [m/s^2]
            auto F = -(3.0 / 2.0) * InsConst<>::GPS::J2 * (mu / std::pow(r_k, 2)) * std::pow(InsConst<>::GPS::R_E / r_k, 2);
            // Earth-Fixed x acceleration [m/s^2]
            auto ax_k = -mu * (x_k / std::pow(r_k, 3)) + F * ((1.0 - 5.0 * std::pow(z_k / r_k, 2)) * (x_k / r_k))
                        + 2 * vy_k * Omega_e_dot + x_k * std::pow(Omega_e_dot, 2);
            // Earth-Fixed y acceleration [m/s^2]
            auto ay_k = -mu * (y_k / std::pow(r_k, 3)) + F * ((1.0 - 5.0 * std::pow(z_k / r_k, 2)) * (y_k / r_k))
                        + 2 * vx_k * Omega_e_dot + y_k * std::pow(Omega_e_dot, 2);
            // Earth-Fixed z acceleration [m/s^2]
            auto az_k = -mu * (z_k / std::pow(r_k, 3)) + F * ((3.0 - 5.0 * std::pow(z_k / r_k, 2)) * (z_k / r_k));

            e_accel = Eigen::Vector3d{ ax_k, ay_k, az_k };
        }
    }

    return { .e_pos = e_pos,
             .e_vel = e_vel,
             .e_accel = e_accel };
}

bool IRNSSEphemeris::isHealthy() const // TODO Parse Signal Id as a parameter and differentiate depending on the bitset
{
    return svHealth.none();
}

double IRNSSEphemeris::calcSatellitePositionVariance() const
{
    // Getting the index and value again will discretize the URA values
    return std::pow(gpsUraIdx2Val(gpsUraVal2Idx(svAccuracy)), 2);
}

} // namespace NAV