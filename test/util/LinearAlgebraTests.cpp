#include <catch2/catch.hpp>

#include "util/LinearAlgebra.hpp"
#include "util/Logger.hpp"

namespace NAV
{
TEST_CASE("[LinearAlgebra] Vector construction", "[LinearAlgebra]")
{
    InsVector3<CoordinateSystem::Body, double> vec1_b(1, 2, 4);
    // vec1_b << 1, 2, 4;
    InsVector3d<CoordinateSystem::Body> vec2_b;
    vec2_b << 8, 16, 32;

    Eigen::Matrix<double, 3, 1> vec1t_b = vec1_b.transpose();
    auto vecTrans_b = InsMatrix<CoordinateSystem::Body, double, 3, 1>(vec1t_b);

    LOG_INFO("{}", vecTrans_b);

    InsMatrix<CoordinateSystem::Body, double, 3, 1> vecSum_b = vec1_b;
    vecSum_b -= vec2_b;
    LOG_INFO("{}", vecSum_b.transpose());

    InsMatrix<CoordinateSystem::Navigation, double, 3, 1> vec1_n;
    vec1_n << 64, 128, 256;
    LOG_INFO("{}", (vec1_b - vec2_b).transpose());

    // Eigen::Vector3d vec2_n;
    // vec2_n << 1, 5, 8;
    // vec2_n -= vec1_b;
    // LOG_INFO("{}", vec2_n.transpose());

    CHECK(true == false);
}

} // namespace NAV