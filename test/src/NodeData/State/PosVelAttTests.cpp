#include <catch2/catch.hpp>
#include "EigenApprox.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "fmt/core.h"

namespace NAV
{
TEST_CASE("[PosVelAtt] Position Functions", "[PosVelAtt]")
{
    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d latLonAlt{ trafo::deg2rad(48.78081), trafo::deg2rad(9.172012), 254 };
    Eigen::Vector3d pos_ecef = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;

    Pos state;
    state.setPosition_e(pos_ecef);

    CHECK(state.position_ecef() == pos_ecef);
    CHECK(state.latLonAlt() == Eigen::Vector3d{ state.latitude(), state.longitude(), state.altitude() });
    CHECK(state.latitude() == Approx(latLonAlt(0)));
    CHECK(state.longitude() == Approx(latLonAlt(1)));
    CHECK(state.altitude() == Approx(latLonAlt(2)).margin(0.3));

    CHECK(state.quaternion_en() == EigApproxQ(trafo::quat_en(latLonAlt(0), latLonAlt(1))));
    CHECK(state.quaternion_ne() == state.quaternion_en().conjugate());

    // ###########################################################################################################

    state.setPosition_lla(latLonAlt);

    CHECK(state.position_ecef() == EigApprox(pos_ecef));
    CHECK(state.latLonAlt() == latLonAlt);
}

TEST_CASE("[PosVelAtt] Velocity Functions", "[PosVelAtt]")
{
    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d latLonAlt{ trafo::deg2rad(48.78081), trafo::deg2rad(9.172012), 254 };
    Eigen::Vector3d vel_e{ 30, -25.5, 4774.723 };
    Eigen::Vector3d vel_n = trafo::quat_ne(latLonAlt(0), latLonAlt(1)) * vel_e;

    PosVel state;
    state.setPosition_lla(latLonAlt);
    state.setVelocity_e(vel_e);

    CHECK(state.latLonAlt() == latLonAlt);
    CHECK(state.velocity_e() == vel_e);
    CHECK(state.velocity_n() == EigApprox(vel_n));
    CHECK(state.velocity_e().norm() == Approx(state.velocity_n().norm()));

    state.setVelocity_n(vel_n);
    CHECK(state.latLonAlt() == latLonAlt);
    CHECK(state.velocity_e() == EigApprox(vel_e));
    CHECK(state.velocity_n() == vel_n);
    CHECK(state.velocity_e().norm() == Approx(state.velocity_n().norm()));
}

TEST_CASE("[PosVelAtt] Attitude Functions", "[PosVelAtt]")
{
    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d latLonAlt{ trafo::deg2rad(48.78081), trafo::deg2rad(9.172012), 254 };
    Eigen::Vector3d vel_e{ 30, -25.5, 4774.723 };
    Eigen::Vector3d vel_n = trafo::quat_ne(latLonAlt(0), latLonAlt(1)) * vel_e;
    double roll = trafo::deg2rad(5);
    double pitch = trafo::deg2rad(-30);
    double yaw = trafo::deg2rad(66);

    PosVelAtt state;
    state.setState_e(trafo::lla2ecef_WGS84(latLonAlt), vel_e, trafo::quat_en(latLonAlt(0), latLonAlt(1)) * trafo::quat_nb(roll, pitch, yaw));

    CHECK(state.latLonAlt() == EigApprox(latLonAlt));
    CHECK(state.velocity_e() == vel_e);
    CHECK(state.quaternion_nb() == EigApproxQ(trafo::quat_nb(roll, pitch, yaw)));
    CHECK(state.quaternion_bn() == EigApproxQ(trafo::quat_bn(roll, pitch, yaw)));
    CHECK(state.quaternion_eb() == trafo::quat_en(latLonAlt(0), latLonAlt(1)) * trafo::quat_nb(roll, pitch, yaw));
    CHECK(state.quaternion_be() == trafo::quat_bn(roll, pitch, yaw) * trafo::quat_ne(latLonAlt(0), latLonAlt(1)));

    state.setState_n(latLonAlt, vel_n, trafo::quat_nb(roll, pitch, yaw));
    CHECK(state.velocity_n() == vel_n);
    CHECK(state.quaternion_nb() == trafo::quat_nb(roll, pitch, yaw));
    CHECK(state.quaternion_bn() == trafo::quat_bn(roll, pitch, yaw));
    CHECK(state.quaternion_eb() == trafo::quat_en(latLonAlt(0), latLonAlt(1)) * trafo::quat_nb(roll, pitch, yaw));
    CHECK(state.quaternion_be() == trafo::quat_bn(roll, pitch, yaw) * trafo::quat_ne(latLonAlt(0), latLonAlt(1)));
}

TEST_CASE("[PosVelAtt] Attitude RollPitchYaw", "[PosVelAtt]")
{
    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    PosVelAtt state;
    state.setPosition_lla({ trafo::deg2rad(48.78081), trafo::deg2rad(9.172012), 254 });

    double delta = trafo::deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double expectedRoll = -M_PI + delta; expectedRoll <= M_PI - delta + std::numeric_limits<float>::epsilon(); expectedRoll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double expectedPitch = -M_PI / 2.0 + delta; expectedPitch <= M_PI / 2.0 - delta + std::numeric_limits<float>::epsilon(); expectedPitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double expectedYaw = -M_PI + delta; expectedYaw <= M_PI - delta + std::numeric_limits<float>::epsilon(); expectedYaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                state.setAttitude_nb(trafo::quat_nb(expectedRoll, expectedPitch, expectedYaw));
                auto actualRollPitchYaw = trafo::rad2deg(state.rollPitchYaw());
                REQUIRE(trafo::rad2deg(Eigen::Vector3d{ expectedRoll, expectedPitch, expectedYaw }) == EigApprox(actualRollPitchYaw).margin(1e-8).epsilon(0));
            }
        }
    }
}

} // namespace NAV
