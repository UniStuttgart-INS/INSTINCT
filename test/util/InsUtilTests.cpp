#include <catch2/catch.hpp>

#include "util/InsUtil.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace NAV
{
TEST_CASE("[InsUtil] Euler angles to Quaternion", "[InsUtil]")
{
    // Conversions with https://www.andre-gaschler.com/rotationconverter

    Eigen::Quaterniond q = euler2quaternion(0, 0, M_PI_2);
    //                         w                   x          y    z
    Eigen::Quaterniond q1(1.0 / std::sqrt(2), 1.0 / sqrt(2), 0.0, 0.0);

    REQUIRE(std::abs(q.x() - q1.x()) <= 0.0000001);
    REQUIRE(std::abs(q.y() - q1.y()) <= 0.0000001);
    REQUIRE(std::abs(q.z() - q1.z()) <= 0.0000001);
    REQUIRE(std::abs(q.w() - q1.w()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */

    q = euler2quaternion(0, M_PI, 0);
    //                       w    x    y    z
    q1 = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);

    REQUIRE(std::abs(q.x() - q1.x()) <= 0.0000001);
    REQUIRE(std::abs(q.y() - q1.y()) <= 0.0000001);
    REQUIRE(std::abs(q.z() - q1.z()) <= 0.0000001);
    REQUIRE(std::abs(q.w() - q1.w()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */

    q = euler2quaternion(M_PI * 4, 0, 0);
    //                       w    x    y    z
    q1 = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    REQUIRE(std::abs(q.x() - q1.x()) <= 0.0000001);
    REQUIRE(std::abs(q.y() - q1.y()) <= 0.0000001);
    REQUIRE(std::abs(q.z() - q1.z()) <= 0.0000001);
    REQUIRE(std::abs(q.w() - q1.w()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */
    q = euler2quaternion(-5.6, 2, 1.4);
    //                         w           x           y          z
    q1 = Eigen::Quaterniond(-0.5709635, -0.1123656, -0.7230073, 0.3723373);

    REQUIRE(std::abs(q.x() - q1.x()) <= 0.0000001);
    REQUIRE(std::abs(q.y() - q1.y()) <= 0.0000001);
    REQUIRE(std::abs(q.z() - q1.z()) <= 0.0000001);
    REQUIRE(std::abs(q.w() - q1.w()) <= 0.0000001);
}

TEST_CASE("[InsUtil] Quaternion to Euler angles", "[InsUtil]")
{
    // Conversions with https://www.andre-gaschler.com/rotationconverter

    //                        w                   x          y    z
    Eigen::Quaterniond q(1.0 / std::sqrt(2), 1.0 / sqrt(2), 0.0, 0.0);
    Eigen::Vector3d yawPitchRoll = quaternion2euler(q);
    Eigen::Vector3d yawPitchRoll1(0, 0, M_PI_2);

    REQUIRE(std::abs(yawPitchRoll.x() - yawPitchRoll1.x()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.y() - yawPitchRoll1.y()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.z() - yawPitchRoll1.z()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */

    //                          w               x          y        z
    q = Eigen::Quaterniond(1.0 / std::sqrt(2), 0.0, 1.0 / sqrt(2), 0.0);
    yawPitchRoll = quaternion2euler(q);
    yawPitchRoll1 = Eigen::Vector3d(0, M_PI_2, 0);

    REQUIRE(std::abs(yawPitchRoll.x() - yawPitchRoll1.x()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.y() - yawPitchRoll1.y()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.z() - yawPitchRoll1.z()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */

    //                          w               x    y          z
    q = Eigen::Quaterniond(1.0 / std::sqrt(2), 0.0, 0.0, 1.0 / sqrt(2));
    yawPitchRoll = quaternion2euler(q);
    yawPitchRoll1 = Eigen::Vector3d(M_PI_2, 0, 0);

    REQUIRE(std::abs(yawPitchRoll.x() - yawPitchRoll1.x()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.y() - yawPitchRoll1.y()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.z() - yawPitchRoll1.z()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */

    //                         w          x          y           z
    q = Eigen::Quaterniond(0.6789701, 0.2436967, 0.2208607, 0.656378);
    yawPitchRoll = quaternion2euler(q);
    yawPitchRoll1 = Eigen::Vector3d(1.53, -0.02, 0.67);

    REQUIRE(std::abs(yawPitchRoll.x() - yawPitchRoll1.x()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.y() - yawPitchRoll1.y()) <= 0.0000001);
    REQUIRE(std::abs(yawPitchRoll.z() - yawPitchRoll1.z()) <= 0.0000001);

    /* -------------------------------------------------------------------------- */

    //                      w    x    y    z
    q = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);
    yawPitchRoll = quaternion2euler(q);
    yawPitchRoll1 = Eigen::Vector3d(0, M_PI, 0);
    Eigen::Quaterniond q1 = euler2quaternion(yawPitchRoll1.x(), yawPitchRoll1.y(), yawPitchRoll1.z());

    REQUIRE(std::abs(q.x() - q1.x()) <= 0.0000001);
    REQUIRE(std::abs(q.y() - q1.y()) <= 0.0000001);
    REQUIRE(std::abs(q.z() - q1.z()) <= 0.0000001);
    REQUIRE(std::abs(q.w() - q1.w()) <= 0.0000001);
}

} // namespace NAV
