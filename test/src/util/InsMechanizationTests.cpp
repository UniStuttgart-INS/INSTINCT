#include <catch2/catch.hpp>
#include "EigenApprox.hpp"

#include "util/InsMath.hpp"
#include "util/InsMechanization.hpp"
#include "util/InsTransformations.hpp"
#include "util/InsGravity.hpp"

#include "util/Eigen.hpp"

#include <deque>
#include <limits>
#include <iostream>

namespace NAV
{
TEST_CASE("[InsMechanization] Update Quaternions ep Runge-Kutta 3. Order", "[InsMechanization]")
{
    auto checkIntegration = [](const Eigen::Vector3d& angularVelocity_ip_p__t0,
                               double deltaLatLon, double delta,
                               double longitudeMin, double longitudeMax,
                               double latitudeMin, double latitudeMax,
                               double rollMin, double rollMax,
                               double pitchMin, double pitchMax,
                               double yawMin, double yawMax) {
        std::cout << "Testing with\n"
                  << "    angularVelocity_ip_p__t0 = " << trafo::rad2deg3(angularVelocity_ip_p__t0).transpose() << " [°/s]\n"
                  << "    deltaLatLon (test angle steps) = " << trafo::rad2deg(deltaLatLon) << " [°]\n"
                  << "    delta       (test angle steps) = " << trafo::rad2deg(delta) << " [°]\n"
                  << "    longitude  [" << trafo::rad2deg(longitudeMin) << " " << trafo::rad2deg(longitudeMax) << ") [°]\n"
                  << "    latitude   [" << trafo::rad2deg(latitudeMin) << " " << trafo::rad2deg(latitudeMax) << ") [°]\n"
                  << "    roll  [" << trafo::rad2deg(rollMin) << " " << trafo::rad2deg(rollMax) << ") [°]\n"
                  << "    pitch [" << trafo::rad2deg(pitchMin) << " " << trafo::rad2deg(pitchMax) << ") [°]\n"
                  << "    yaw   [" << trafo::rad2deg(yawMin) << " " << trafo::rad2deg(yawMax) << ") [°]\n";

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        long double timeDifferenceSec = 0.001L;
        // ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in navigation coordinates, at the time tₖ
        Eigen::Vector3d angularVelocity_ie_e__t0{ 0, 0, 0 };
        // q (tₖ₋₁) Quaternion, from platform to body coordinates, at the time tₖ₋₁
        Eigen::Quaterniond quaternion_bp__t1 = trafo::quat_bp(0, 0, 0);

        Eigen::Vector3d angularVelocity_ip_b__t0 = quaternion_bp__t1 * angularVelocity_ip_p__t0;

        for (double longitude = longitudeMin; longitude < longitudeMax - std::numeric_limits<float>::epsilon(); longitude += deltaLatLon) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double latitude = latitudeMin; latitude < latitudeMax - std::numeric_limits<float>::epsilon(); latitude += deltaLatLon) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                // q (tₖ₋₁) Quaternion, from navigation earth coordinates, at the time tₖ₋₁
                Eigen::Quaterniond quaternion_en__t1 = trafo::quat_en(latitude, longitude);

                // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
                for (double roll = rollMin; roll < rollMax - std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
                {
                    for (double pitch = pitchMin; pitch < pitchMax - std::numeric_limits<float>::epsilon(); pitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
                    {
                        for (double yaw = yawMin; yaw < yawMax - std::numeric_limits<float>::epsilon(); yaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
                        {
                            // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
                            Eigen::Quaterniond quaternion_nb__t1 = trafo::quat_nb(roll, pitch, yaw);

                            Eigen::Quaterniond quaternion_ep__t1 = quaternion_en__t1 * quaternion_nb__t1 * quaternion_bp__t1;

                            // q (tₖ) Quaternion, from earth to platform coordinates, at the time tₖ
                            Eigen::Quaterniond quaternion_ep__t0 = updateQuaternion_ep_RungeKutta3(
                                timeDifferenceSec,        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
                                timeDifferenceSec,        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
                                angularVelocity_ip_p__t0, // ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ
                                angularVelocity_ip_p__t0, // ω_ip_p (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
                                angularVelocity_ie_e__t0, // ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
                                quaternion_ep__t1,        // q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
                                quaternion_ep__t1);       // q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂

                            Eigen::Quaterniond quaternion_nb__t0 = quaternion_en__t1.conjugate() * quaternion_ep__t0 * quaternion_bp__t1.conjugate();

                            // Roll, Pitch and Yaw angle at the time tₖ
                            Eigen::Vector3d rollPitchYaw__t0 = trafo::rad2deg3(trafo::quat2eulerZYX(quaternion_nb__t0));

                            // Titterton Ch. 3.6.3.3, eq. 3.52, p. 42
                            Eigen::Vector3d expectedRollPitchYaw = Eigen::Vector3d{ roll, pitch, yaw }
                                                                   + Eigen::Vector3d{ (angularVelocity_ip_b__t0.y() * std::sin(roll) + angularVelocity_ip_b__t0.z() * std::cos(roll)) * std::tan(pitch) + angularVelocity_ip_b__t0.x(),
                                                                                      angularVelocity_ip_b__t0.y() * std::cos(roll) - angularVelocity_ip_b__t0.z() * std::sin(roll),
                                                                                      (angularVelocity_ip_b__t0.y() * std::sin(roll) + angularVelocity_ip_b__t0.z() * std::cos(roll)) / std::cos(pitch) }
                                                                         * 2.0 * static_cast<double>(timeDifferenceSec);

                            Eigen::Quaterniond expectedQuaternion_nb = trafo::quat_nb(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

                            // std::cout << "quaternion_nb__t1 (x,y,z,w) = " << quaternion_nb__t1.coeffs().transpose() << "\n";

                            Eigen::Vector3d vec_b{ 1, 2, 3 };
                            Eigen::Vector3d vec_n = quaternion_nb__t0 * vec_b;
                            Eigen::Vector3d expected_vec_n = expectedQuaternion_nb * vec_b;
                            CHECK(vec_n == EigApprox(expected_vec_n).margin(1e-6).epsilon(0));

                            REQUIRE(rollPitchYaw__t0 == EigApprox(trafo::rad2deg3(expectedRollPitchYaw)).margin(1e-3).epsilon(0));
                        }
                    }
                }
            }
        }
    };

    double delta = trafo::deg2rad(10);
    double deltaLatLon = trafo::deg2rad(30);
    /* ########################################################################################################### */
    checkIntegration(trafo::deg2rad3(Eigen::Vector3d{ 7, -3, 2 }), // angularVelocity_ip_b__t0
                     deltaLatLon, delta,                           // deltaLatLon, delta (test angle step size)
                     -M_PI + delta, M_PI,                          // [longitudeMin, longitudeMax)
                     -M_PI / 2 + delta, M_PI / 2,                  // [latitudeMin, latitudeMax)
                     -M_PI + delta, M_PI,                          // [rollMin, rollMax)
                     -M_PI / 2.0 + delta, M_PI / 2.0,              // [pitchMin, pitchMax)
                     -M_PI + delta, M_PI);                         // [yawMin, yawMax)
}

TEST_CASE("[InsMechanization] Update Quaternions nb Runge-Kutta 1. Order", "[InsMechanization]")
{
    auto checkIntegration = [](const Eigen::Vector3d& angularVelocity_ip_b__t0,
                               double delta,
                               double rollMin, double rollMax,
                               double pitchMin, double pitchMax,
                               double yawMin, double yawMax) {
        std::cout << "Testing with\n"
                  << "    angularVelocity_ip_b__t0 = " << trafo::rad2deg3(angularVelocity_ip_b__t0).transpose() << " [°/s]\n"
                  << "    delta (test angle steps) = " << trafo::rad2deg(delta) << " [°]\n"
                  << "    roll  [" << trafo::rad2deg(rollMin) << " " << trafo::rad2deg(rollMax) << ") [°]\n"
                  << "    pitch [" << trafo::rad2deg(pitchMin) << " " << trafo::rad2deg(pitchMax) << ") [°]\n"
                  << "    yaw   [" << trafo::rad2deg(yawMin) << " " << trafo::rad2deg(yawMax) << ") [°]\n";

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        long double timeDifferenceSec__t0 = 0.001L;
        // ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in navigation coordinates, at the time tₖ
        Eigen::Vector3d angularVelocity_ie_n__t1{ 0, 0, 0 };
        // ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame in navigation coordinates
        Eigen::Vector3d angularVelocity_en_n__t1{ 0, 0, 0 };

        // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
        for (double roll = rollMin; roll < rollMax - std::numeric_limits<float>::epsilon(); roll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double pitch = pitchMin; pitch < pitchMax - std::numeric_limits<float>::epsilon(); pitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                for (double yaw = yawMin; yaw < yawMax - std::numeric_limits<float>::epsilon(); yaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
                {
                    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
                    Eigen::Quaterniond quaternion_nb__t1 = trafo::quat_nb(roll, pitch, yaw);

                    // q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
                    Eigen::Quaterniond quaternion_nb__t0 = updateQuaternion_nb_RungeKutta1(timeDifferenceSec__t0,
                                                                                           angularVelocity_ip_b__t0,
                                                                                           angularVelocity_ie_n__t1,
                                                                                           angularVelocity_en_n__t1,
                                                                                           quaternion_nb__t1);

                    // Roll, Pitch and Yaw angle at the time tₖ
                    Eigen::Vector3d rollPitchYaw__t0 = trafo::rad2deg3(trafo::quat2eulerZYX(quaternion_nb__t0));

                    // Titterton Ch. 3.6.3.3, eq. 3.52, p. 42
                    Eigen::Vector3d expectedRollPitchYaw = Eigen::Vector3d{ roll, pitch, yaw }
                                                           + Eigen::Vector3d{ (angularVelocity_ip_b__t0.y() * std::sin(roll) + angularVelocity_ip_b__t0.z() * std::cos(roll)) * std::tan(pitch) + angularVelocity_ip_b__t0.x(),
                                                                              angularVelocity_ip_b__t0.y() * std::cos(roll) - angularVelocity_ip_b__t0.z() * std::sin(roll),
                                                                              (angularVelocity_ip_b__t0.y() * std::sin(roll) + angularVelocity_ip_b__t0.z() * std::cos(roll)) / std::cos(pitch) }
                                                                 * static_cast<double>(timeDifferenceSec__t0);

                    Eigen::Quaterniond expectedQuaternion_nb = trafo::quat_nb(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

                    // std::cout << "quaternion_nb__t1 (x,y,z,w) = " << quaternion_nb__t1.coeffs().transpose() << "\n";

                    Eigen::Vector3d vec_b{ 1, 2, 3 };
                    Eigen::Vector3d vec_n = quaternion_nb__t0 * vec_b;
                    Eigen::Vector3d expected_vec_n = expectedQuaternion_nb * vec_b;
                    CHECK(vec_n == EigApprox(expected_vec_n).margin(1e-6).epsilon(0));

                    REQUIRE(rollPitchYaw__t0 == EigApprox(trafo::rad2deg3(expectedRollPitchYaw)).margin(1e-4).epsilon(0));
                }
            }
        }
    };

    double delta = trafo::deg2rad(5);
    /* ########################################################################################################### */
    checkIntegration(Eigen::Vector3d{ 0, trafo::deg2rad(5), 0 }, // angularVelocity_ip_b__t0
                     delta,                                      // delta (test angle step size)
                     -M_PI + delta, M_PI,                        // [rollMin, rollMax)
                     0, trafo::deg2rad(1),                       // [pitchMin, pitchMax)
                     0, trafo::deg2rad(1));                      // [yawMin, yawMax)
    /* ########################################################################################################### */
    checkIntegration(Eigen::Vector3d{ 0, trafo::deg2rad(5), 0 }, // angularVelocity_ip_b__t0
                     delta,                                      // delta (test angle step size)
                     0, trafo::deg2rad(1),                       // [rollMin, rollMax)
                     -M_PI / 2.0 + delta, M_PI / 2.0,            // [pitchMin, pitchMax)
                     0, trafo::deg2rad(1));                      // [yawMin, yawMax)
    /* ########################################################################################################### */
    checkIntegration(Eigen::Vector3d{ 0, trafo::deg2rad(5), 0 }, // angularVelocity_ip_b__t0
                     delta,                                      // delta (test angle step size)
                     0, trafo::deg2rad(1),                       // [rollMin, rollMax)
                     0, trafo::deg2rad(1),                       // [pitchMin, pitchMax)
                     -M_PI + delta, M_PI);                       // [yawMin, yawMax)
    /* ########################################################################################################### */
    /* ########################################################################################################### */
    checkIntegration(Eigen::Vector3d{ trafo::deg2rad(10), 0, 0 }, // angularVelocity_ip_b__t0
                     delta,                                       // delta (test angle step size)
                     -M_PI + delta, M_PI,                         // [rollMin, rollMax)
                     -M_PI / 2.0 + delta, M_PI / 2.0,             // [pitchMin, pitchMax)
                     -M_PI + delta, M_PI);                        // [yawMin, yawMax)
    /* ########################################################################################################### */
    checkIntegration(Eigen::Vector3d{ 0, trafo::deg2rad(4), 0 }, // angularVelocity_ip_b__t0
                     delta,                                      // delta (test angle step size)
                     -M_PI + delta, M_PI,                        // [rollMin, rollMax)
                     -M_PI / 2.0 + delta, M_PI / 2.0,            // [pitchMin, pitchMax)
                     -M_PI + delta, M_PI);                       // [yawMin, yawMax)
    /* ########################################################################################################### */
    checkIntegration(Eigen::Vector3d{ 0, 0, trafo::deg2rad(-6) }, // angularVelocity_ip_b__t0
                     delta,                                       // delta (test angle step size)
                     -M_PI + delta, M_PI,                         // [rollMin, rollMax)
                     -M_PI / 2.0 + delta, M_PI / 2.0,             // [pitchMin, pitchMax)
                     -M_PI + delta, M_PI);                        // [yawMin, yawMax)
    /* ########################################################################################################### */
    /* ########################################################################################################### */
    checkIntegration(trafo::deg2rad3(Eigen::Vector3d{ 7, -3, 2 }), // angularVelocity_ip_b__t0
                     delta,                                        // delta (test angle step size)
                     -M_PI + delta, M_PI,                          // [rollMin, rollMax)
                     -M_PI / 2.0 + delta, M_PI / 2.0,              // [pitchMin, pitchMax)
                     -M_PI + delta, M_PI);                         // [yawMin, yawMax)
}

TEST_CASE("[InsMechanization] Update Quaternions nb Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;
    /// ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates
    Eigen::Vector3d angularVelocity_ip_b{ 0, 0, 1.0 };
    /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in navigation coordinates, at the time tₖ
    Eigen::Vector3d angularVelocity_ie_n{ 0, 0, 0 };
    /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame in navigation coordinates
    Eigen::Vector3d angularVelocity_en_n{ 0, 0, 0 };

    std::deque<Eigen::Quaterniond> quats_nb;
    quats_nb.emplace_back(Eigen::Quaterniond::Identity());
    quats_nb.emplace_back(Eigen::Quaterniond::Identity());

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        Eigen::Quaterniond q_nb = updateQuaternion_nb_RungeKutta3(timeDifferenceSec, timeDifferenceSec,
                                                                  angularVelocity_ip_b, angularVelocity_ip_b,
                                                                  angularVelocity_ie_n,
                                                                  angularVelocity_en_n,
                                                                  quats_nb.at(1), quats_nb.at(0));
        quats_nb.push_back(q_nb);
        quats_nb.pop_front();
    }

    auto q_nb = quats_nb.at(quats_nb.size() - 1);
    auto rollPitchYaw = trafo::quat2eulerZYX(q_nb);

    Eigen::Vector3d expectedRollPitchYaw = angularVelocity_ip_b * (static_cast<double>(timeDifferenceSec) * static_cast<double>(count));
    Eigen::Quaterniond expectedQuat_nb = trafo::quat_nb(expectedRollPitchYaw.x(), expectedRollPitchYaw.y(), expectedRollPitchYaw.z());

    CHECK(q_nb.x() == Approx(expectedQuat_nb.x()).margin(1e-13));
    CHECK(q_nb.y() == Approx(expectedQuat_nb.y()).margin(1e-13));
    CHECK(q_nb.z() == Approx(expectedQuat_nb.z()).margin(1e-13));
    CHECK(q_nb.w() == Approx(expectedQuat_nb.w()).margin(1e-13));

    CHECK(rollPitchYaw.x() == Approx(expectedRollPitchYaw.x()).margin(1e-13));
    CHECK(rollPitchYaw.y() == Approx(expectedRollPitchYaw.y()).margin(1e-13));
    CHECK(rollPitchYaw.z() == Approx(expectedRollPitchYaw.z()).margin(1e-13));
}

TEST_CASE("[InsMechanization] Update Velocity e-frame Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double altitude = 254;

    double roll = 0;
    double pitch = 0;
    double yaw = trafo::deg2rad(45);

    double mountingAngleX = 90;
    double mountingAngleY = 180;
    double mountingAngleZ = 0;

    auto gravity = gravity::gravityMagnitude_SomiglianaAltitude(latitude, altitude);
    Eigen::Vector3d gravity_n{ 0, 0, gravity };
    Eigen::Vector3d gravity_e = trafo::quat_en(latitude, longitude) * gravity_n;

    /// a_p Acceleration in [m/s^2], in navigation coordinates
    Eigen::Vector3d acceleration_n(1, -1, -gravity);

    Eigen::Vector3d acceleration_p = trafo::quat_pb(mountingAngleX, mountingAngleY, mountingAngleZ)
                                     * trafo::quat_bn(roll, pitch, yaw)
                                     * acceleration_n;

    Eigen::Vector3d position = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });

    Eigen::Quaterniond quaternion_ep = trafo::quat_en(latitude, longitude)
                                       * trafo::quat_nb(roll, pitch, yaw)
                                       * trafo::quat_bp(mountingAngleX, mountingAngleY, mountingAngleZ);

    bool suppressCoriolis = false;

    std::deque<Eigen::Vector3d> velocities;
    velocities.emplace_back(Eigen::Vector3d::Zero());
    velocities.emplace_back(Eigen::Vector3d::Zero());

    size_t count = 10000;
    for (size_t i = 0; i <= count; i++)
    {
        Eigen::Vector3d v_e = updateVelocity_e_Simpson(timeDifferenceSec, timeDifferenceSec,
                                                       acceleration_p, acceleration_p,
                                                       velocities.at(0),
                                                       position,
                                                       gravity_e,
                                                       quaternion_ep,
                                                       quaternion_ep,
                                                       quaternion_ep,
                                                       suppressCoriolis);
        velocities.push_back(v_e);
        velocities.pop_front();
    }

    auto v_e = velocities.at(velocities.size() - 1);

    auto v_n = trafo::quat_ne(latitude, longitude) * v_e;

    // Exact values are not achieved
    CHECK(v_n.x() == Approx(1).margin(0.03));
    CHECK(v_n.y() == Approx(-1).margin(0.01));
    CHECK(v_n.z() == Approx(0).margin(0.02));
}

TEST_CASE("[InsMechanization] Update Velocity n-frame Runge-Kutta 3. Order", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double altitude = 254;

    double roll = 0;
    double pitch = 0;
    double yaw = trafo::deg2rad(45);

    auto gravity = gravity::gravityMagnitude_SomiglianaAltitude(latitude, altitude);
    Eigen::Vector3d gravity_n{ 0, 0, gravity };

    /// a_p Acceleration in [m/s^2], in navigation coordinates
    Eigen::Vector3d acceleration_n{ 1, 1, -gravity };
    Eigen::Vector3d acceleration_b = trafo::quat_bn(roll, pitch, yaw) * acceleration_n;

    Eigen::Quaterniond quaternion_nb = trafo::quat_nb(roll, pitch, yaw);

    /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
    Eigen::Vector3d angularVelocity_ie_n = trafo::quat_ne(latitude, longitude) * InsConst::angularVelocity_ie_e;

    /// North/South (meridian) earth radius [m]
    double R_N = earthRadius_N(latitude, InsConst::WGS84_a, InsConst::WGS84_e_squared);
    /// East/West (prime vertical) earth radius [m]
    double R_E = earthRadius_E(latitude, InsConst::WGS84_a, InsConst::WGS84_e_squared);

    std::deque<Eigen::Vector3d> velocities;
    velocities.emplace_back(Eigen::Vector3d::Zero());
    velocities.emplace_back(Eigen::Vector3d::Zero());

    bool suppressCoriolis = false;

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
        Eigen::Vector3d angularVelocity_en_n = transportRate({ latitude, longitude, altitude }, velocities.at(1), R_N, R_E);

        Eigen::Vector3d v_n = updateVelocity_n_Simpson(timeDifferenceSec, timeDifferenceSec,
                                                       acceleration_b,
                                                       acceleration_b,
                                                       velocities.at(1),
                                                       velocities.at(0),
                                                       gravity_n,
                                                       angularVelocity_ie_n,
                                                       angularVelocity_en_n,
                                                       quaternion_nb,
                                                       quaternion_nb,
                                                       quaternion_nb,
                                                       suppressCoriolis);
        velocities.push_back(v_n);
        velocities.pop_front();
    }

    auto v_n = velocities.at(velocities.size() - 1);

    // Exact values are not achieved
    CHECK(v_n.x() == Approx(1).margin(0.001));
    CHECK(v_n.y() == Approx(1).margin(0.001));
    CHECK(v_n.z() == Approx(0).margin(1e-4));
}

TEST_CASE("[InsMechanization] Update Position e-frame", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0.0001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double altitude = 254;

    Eigen::Vector3d velocity_n{ 2, 0, 0 };
    Eigen::Vector3d velocity_e = trafo::quat_en(latitude, longitude) * velocity_n;

    Eigen::Vector3d position_e = trafo::lla2ecef_WGS84({ latitude, longitude, altitude });

    size_t count = 10000;
    for (size_t i = 0; i < count; i++)
    {
        position_e = updatePosition_e(timeDifferenceSec, position_e, velocity_e);
    }
    auto lla = trafo::ecef2lla_WGS84(position_e);

    CHECK(measureDistance(latitude, longitude, lla(0), lla(1)) == Approx(2.0).margin(0.002));

    CHECK(measureDistance(latitude, longitude, lla(0), longitude) == Approx(2.0).margin(0.002));
    CHECK(longitude == Approx(lla(1)).margin(1e-13));

    CHECK(latitude < lla(0));
}

TEST_CASE("[InsMechanization] Update Position lla-frame", "[InsMechanization]")
{
    /// Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double dt = 0.001L;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    double latitude = trafo::deg2rad(48.78081);
    double longitude = trafo::deg2rad(9.172012);
    double altitude = 254;

    double roll = 0;
    double pitch = 0;
    double yaw = trafo::deg2rad(45);

    Eigen::Vector3d velocity_b{ 2, 0, 0 };

    Eigen::Vector3d velocity_n = trafo::quat_nb(roll, pitch, yaw) * velocity_b;

    Eigen::Vector3d latLonAlt{ latitude, longitude, altitude };

    size_t count = 4000;
    for (size_t i = 0; i < count; i++)
    {
        // North/South (meridian) earth radius [m]
        double R_N = earthRadius_N(latLonAlt(0), InsConst::WGS84_a, InsConst::WGS84_e_squared);
        // East/West (prime vertical) earth radius [m]
        double R_E = earthRadius_E(latLonAlt(0), InsConst::WGS84_a, InsConst::WGS84_e_squared);

        latLonAlt = updatePosition_lla(dt, latLonAlt, velocity_n, R_N, R_E);
    }
    double distance = static_cast<double>(count) * static_cast<double>(dt) * velocity_b.norm();

    // updatePosition_n with lat lon formula shows really bad accuracy
    CHECK(measureDistance(latitude, longitude, latLonAlt(0), latLonAlt(1)) == Approx(distance).margin(0.004));
    CHECK(measureDistance(latitude, longitude, latLonAlt(0), longitude) == Approx(distance * std::cos(yaw)).margin(0.02));
    CHECK(measureDistance(latitude, longitude, latitude, latLonAlt(1)) == Approx(distance * std::sin(yaw)).margin(0.02));

    CHECK(latitude < latLonAlt(0));
    CHECK(longitude < latLonAlt(1));
}

// TEST_CASE("[InsMechanization] PVAError correction", "[InsMechanization]")
// {
//     Eigen::Vector3d latLonAlt{ trafo::deg2rad(30), trafo::deg2rad(9), 400 };
//     Eigen::Vector3d vel_n{ 15, 2, -5 };
//     double roll = trafo::deg2rad(10.0);
//     double pitch = trafo::deg2rad(0.0);
//     double yaw = trafo::deg2rad(0.0);
//     auto posVelAtt = std::make_shared<PosVelAtt>();
//     posVelAtt->setState_n(latLonAlt, vel_n, trafo::quat_nb(roll, pitch, yaw));

//     auto pvaError = std::make_shared<PVAError>();
//     pvaError->positionError_lla() = Eigen::Vector3d{ trafo::deg2rad(4), trafo::deg2rad(-7), 5 };
//     pvaError->velocityError_n() = Eigen::Vector3d{ 3, -10, 15 };
//     pvaError->attitudeError_n() = trafo::deg2rad3({ 1, 0, 0 });

//     Eigen::Vector3d expectedRollPitchYaw = posVelAtt->rollPitchYaw() - pvaError->attitudeError_n();
//     auto correctedPVA = correctPosVelAtt(posVelAtt, pvaError);

//     CHECK(posVelAtt->latLonAlt() - pvaError->positionError_lla() == correctedPVA->latLonAlt());
//     CHECK(posVelAtt->velocity_n() - pvaError->velocityError_n() == correctedPVA->velocity_n());

//     CHECK(trafo::rad2deg3(expectedRollPitchYaw) == trafo::rad2deg3(correctedPVA->rollPitchYaw()));

//     // ###########################################################################################################

//     posVelAtt->setAttitude_nb(trafo::quat_nb(0, trafo::deg2rad(10.0), 0));
//     pvaError->attitudeError_n() = trafo::deg2rad3({ 0, 1, 0 });

//     expectedRollPitchYaw = posVelAtt->rollPitchYaw() - pvaError->attitudeError_n();
//     correctedPVA = correctPosVelAtt(posVelAtt, pvaError);

//     CHECK(trafo::rad2deg3(expectedRollPitchYaw) == trafo::rad2deg3(correctedPVA->rollPitchYaw()));

//     // ###########################################################################################################

//     posVelAtt->setAttitude_nb(trafo::quat_nb(0, 0, trafo::deg2rad(10.0)));
//     pvaError->attitudeError_n() = trafo::deg2rad3({ 0, 0, 1 });

//     expectedRollPitchYaw = posVelAtt->rollPitchYaw() - pvaError->attitudeError_n();
//     correctedPVA = correctPosVelAtt(posVelAtt, pvaError);

//     CHECK(trafo::rad2deg3(expectedRollPitchYaw) == trafo::rad2deg3(correctedPVA->rollPitchYaw()));
// }

} // namespace NAV
