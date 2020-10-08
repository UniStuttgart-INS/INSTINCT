#include <catch2/catch.hpp>

#include "util/LinearAlgebra.hpp"
#include "util/Logger.hpp"

namespace NAV
{
TEST_CASE("[LinearAlgebra] Vector-Matrix arithmetic", "[LinearAlgebra]")
{
    auto dcm_bn = Matrix<Body, Navigation, double, 3, 3>(Eigen::Matrix3d::Identity());
    auto dcm_ne = Matrix<Navigation, Earth, double, 3, 3>(Eigen::Matrix3d::Identity());
    auto dcm_ei = Matrix<Earth, Inertial, double, 3, 3>(Eigen::Matrix3d::Identity());

    auto dcm_be = dcm_bn * dcm_ne;
    CHECK(dcm_be == Eigen::Matrix3d::Identity());

    auto dcm_bi = dcm_bn * dcm_ne * dcm_ei;
    CHECK(dcm_bi == Eigen::Matrix3d::Identity());

    Vector3<CoordinateSystem::Body, double> vec1_b(1, 2, 4);
    Vector3d<Body> vec2_b;
    vec2_b << 8, 16, 32;

    CHECK(vec1_b + vec2_b == Eigen::Vector3d(9, 18, 36));
    CHECK(vec1_b - vec2_b == Eigen::Vector3d(-7, -14, -28));

    Vector3d<Body> vecSum_b = vec1_b;
    vecSum_b += vec2_b;
    CHECK(vecSum_b == vec1_b + vec2_b);

    Vector3d<Body> vecDiff_b = vec1_b;
    vecDiff_b -= vec2_b;
    CHECK(vecDiff_b == vec1_b - vec2_b);

    Vector3d<Navigation> vec3_n(1, 2, 4);
    CHECK(vec1_b + dcm_bn * vec3_n == Eigen::Vector3d(2, 4, 8));
}

TEST_CASE("[LinearAlgebra] Quaternion arithmetic", "[LinearAlgebra]")
{
    auto dcm_bn = Quaternion<Body, Navigation, double>(Eigen::Quaterniond::Identity());
    auto dcm_ne = Quaterniond<Navigation, Earth>(Eigen::Quaterniond::Identity());
    auto dcm_ei = Quaterniond<Earth, Inertial>(1, 0, 0, 0);

    auto dcm_be = dcm_bn * dcm_ne;
    CHECK(dcm_be.coeffs() == Eigen::Quaterniond::Identity().coeffs());

    auto dcm_bi = dcm_bn * dcm_ne * dcm_ei;
    CHECK(dcm_bi.coeffs() == Eigen::Quaterniond::Identity().coeffs());

    Vector3d<Body> vec1_b(1, 2, 4);
    Vector3d<Navigation> vec3_n(1, 2, 4);
    CHECK(vec1_b + dcm_bn * vec3_n == Eigen::Vector3d(2, 4, 8));
}

} // namespace NAV