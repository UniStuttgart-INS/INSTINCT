#include "Ephemeris.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "util/Logger.hpp"
#include "util/Assert.h"

namespace NAV
{

SatelliteClockCorrections GPSEphemeris::calcSatelliteClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const
{
    auto satSys = freq.getSatSys();
    INS_ASSERT_USER_ERROR(satSys == GPS || satSys == GAL, "This function was so far only tested with GPS and GAL. Write a units test before using with other systems");

    LOG_DATA("Calc {} Sat Clock corrections at receiver time {}", satSys, recvTime.toGPSweekTow());
    // Earth gravitational constant [m³/s²]
    const auto mu = satSys == GAL ? InsConst::GAL::MU
                                  : InsConst::GPS::MU; // WGS 84 value of the earth's gravitational constant for GPS user
    // Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
    const auto F = satSys == GAL ? InsConst::GAL::F
                                 : InsConst::GPS::F;

    LOG_DATA("    toe {} (Time of ephemeris)", toe.toGPSweekTow());

    const auto A = sqrt_A * sqrt_A; // Semi-major axis [m]
    LOG_DATA("    A {} [m] (Semi-major axis)", A);
    auto n_0 = std::sqrt(mu / std::pow(A, 3)); // Computed mean motion [rad/s]
    LOG_DATA("    n_0 {} [rad/s] (Computed mean motion)", n_0);
    auto n = n_0 + delta_n; // Corrected mean motion [rad/s]
    LOG_DATA("    n {} [rad/s] (Corrected mean motion)", n);

    // Time at transmission
    InsTime transTime = recvTime - std::chrono::duration<double>(dist / InsConst::C);
    LOG_DATA("    transTime {} (Time at transmission)", transTime.toGPSweekTow());

    // [s]
    auto t_minus_toc = static_cast<double>((transTime - toc).count());
    LOG_DATA("    transTime - toc {} [s]", t_minus_toc);

    // Time difference from ephemeris reference epoch [s]
    double t_k = static_cast<double>((transTime - toe).count());
    LOG_DATA("    transTime - toe {} [s] (t_k = Time difference from ephemeris reference epoch)", t_k);

    // Mean anomaly [rad]
    auto M_k = M_0 + n * t_k;
    LOG_DATA("    M_k {} [s] (Mean anomaly)", M_k);

    // Eccentric anomaly [rad]
    double E_k = M_k;
    for (size_t i = 0; i < 7; i++)
    {
        E_k = M_k + e * sin(E_k);
    }

    // Relativistic correction term [s]
    double dt_r = F * e * sqrt_A * std::sin(E_k);
    LOG_DATA("    dt_r {} [s] (Relativistic correction term)", dt_r);

    // SV PRN code phase time offset [s]
    double dt_sv = a[0] + a[1] * t_minus_toc + a[2] * std::pow(t_minus_toc, 2) + dt_r;
    if (satSys == GPS)
    {
        dt_sv -= ratioFreqSquared(G01, freq) * T_GD;
    }
    else if (satSys == GAL)
    {
        dt_sv -= ratioFreqSquared(E01, freq) * (freq == E07 ? BGD_E1_E5b : BGD_E1_E5a);
    }
    LOG_DATA("    dt_sv {} [s] (SV PRN code phase time offset)", dt_sv);

    // Groves ch. 9.3.1, eq. 9.78, p. 391
    double clkDrift = a[1] + a[2] / 2.0 * t_minus_toc;

    // Correct transmit time for the satellite clock bias
    transTime -= std::chrono::duration<double>(dt_sv);

    return { .transmitTime = transTime, .bias = dt_sv, .drift = clkDrift };
}

void GPSEphemeris::calcSatellitePosVelAccel(const InsTime& transTime, const SatelliteSystem& satSys,
                                            Eigen::Vector3d* e_pos, Eigen::Vector3d* e_vel, Eigen::Vector3d* e_accel) const
{
    INS_ASSERT_USER_ERROR(satSys == GPS || satSys == GAL, "This function was so far only tested with GPS and GAL. Write a units test before using with other systems");

    LOG_DATA("Calc {} Sat Position at transmit time {}", satSys, transTime.toGPSweekTow());
    // Earth gravitational constant [m³/s²]
    const auto mu = satSys == GAL ? InsConst::GAL::MU
                                  : InsConst::GPS::MU; // WGS 84 value of the earth's gravitational constant for GPS user
    // Earth angular velocity [rad/s]
    const auto Omega_e_dot = satSys == GAL ? InsConst::GAL::omega_ie
                                           : InsConst::GPS::omega_ie; // WGS 84 value of the earth's rotation rate

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
    auto v_k = std::atan2(std::sqrt(1 - e * e) * std::sin(E_k) / (1 - e * std::cos(E_k)), (std::cos(E_k) - e) / (1 - e * std::cos(E_k))); // True Anomaly [rad] (GALILEO ICD algorithm)
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
    auto Omega_k = Omega_0 + (Omega_dot - Omega_e_dot) * t_k - Omega_e_dot * static_cast<double>(toe.toGPSweekTow().tow);
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

    if (e_pos)
    {
        *e_pos = Eigen::Vector3d{ x_k, y_k, z_k };
    }

    if (e_vel || e_accel)
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

        if (e_vel)
        {
            *e_vel = Eigen::Vector3d{ vx_k, vy_k, vz_k };
        }

        if (e_accel)
        {
            // Oblate Earth acceleration Factor [m/s^2]
            auto F = -(3.0 / 2.0) * InsConst::GPS::J2 * (mu / std::pow(r_k, 2)) * std::pow(InsConst::GPS::R_E / r_k, 2);
            // Earth-Fixed x acceleration [m/s^2]
            auto ax_k = -mu * (x_k / std::pow(r_k, 3)) + F * ((1.0 - 5.0 * std::pow(z_k / r_k, 2)) * (x_k / r_k))
                        + 2 * vy_k * Omega_e_dot + x_k * std::pow(Omega_e_dot, 2);
            // Earth-Fixed y acceleration [m/s^2]
            auto ay_k = -mu * (y_k / std::pow(r_k, 3)) + F * ((1.0 - 5.0 * std::pow(z_k / r_k, 2)) * (y_k / r_k))
                        + 2 * vx_k * Omega_e_dot + y_k * std::pow(Omega_e_dot, 2);
            // Earth-Fixed z acceleration [m/s^2]
            auto az_k = -mu * (z_k / std::pow(r_k, 3)) + F * ((3.0 - 5.0 * std::pow(z_k / r_k, 2)) * (z_k / r_k));

            *e_accel = Eigen::Vector3d{ ax_k, ay_k, az_k };
        }
    }
}

uint8_t GPSEphemeris::galSisaVal2Idx(double val)
{
    if (val < 0.0 || val > 6.0) { return 255; } // No Accuracy Prediction Available (NAPA)
    if (val <= 0.5) { return static_cast<uint8_t>(static_cast<unsigned int>(val) / 0.01); }
    if (val <= 1.0) { return static_cast<uint8_t>(static_cast<unsigned int>((val - 0.5)) / 0.02) + 50; }
    if (val <= 2.0) { return static_cast<uint8_t>(static_cast<unsigned int>((val - 1.0)) / 0.04) + 75; }
    return static_cast<uint8_t>(static_cast<unsigned int>((val - 2.0)) / 0.16) + 100;
}

double GPSEphemeris::galSisaIdx2Val(uint8_t idx)
{
    if (idx <= 49) { return static_cast<double>(idx) * 0.01; }
    if (idx <= 74) { return 0.5 + (static_cast<double>(idx) - 50.0) * 0.02; }
    if (idx <= 99) { return 1.0 + (static_cast<double>(idx) - 75.0) * 0.04; }
    if (idx <= 125) { return 2.0 + (static_cast<double>(idx) - 100.0) * 0.16; }
    return 500.0;
}

uint8_t GPSEphemeris::gpsUraVal2Idx(double val)
{
    constexpr std::array<double, 15> URA = { 2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0 };
    return val < 0.0 ? static_cast<uint8_t>(URA.size())
                     : static_cast<uint8_t>(std::lower_bound(URA.begin(), URA.end(), val) - URA.begin());
}

double GPSEphemeris::gpsUraIdx2Val(uint8_t idx)
{
    constexpr std::array<double, 15> URA = { 2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0 };
    return URA.at(std::min(static_cast<size_t>(idx), URA.size()));
}

// ###################################################################################################################################################
// ###################################################################################################################################################
// ###################################################################################################################################################

void GLONASSEphemeris::calcSatellitePosVelAccelClk(const InsTime& recvTime, double dist, [[maybe_unused]] const SatelliteSystem& satSys, double* clkBias, double* clkDrift, Eigen::Vector3d* e_pos, Eigen::Vector3d* e_vel, Eigen::Vector3d* e_accel) const
{
    // TODO: Fix the GLONASS algorithm and implement the unit test
    INS_ASSERT_USER_ERROR(satSys != GLO, "This function is not working yet. Fix the unit test first to make sure it is producing correct results");

    INS_ASSERT_USER_ERROR(satSys == GLO, "This function was so far only tested with GLONASS. Write a units test before using with other systems");

    LOG_DATA("Calc {} Sat Position at receiver time {} - {}", satSys, recvTime.toGPSweekTow(), recvTime.toYMDHMS());
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