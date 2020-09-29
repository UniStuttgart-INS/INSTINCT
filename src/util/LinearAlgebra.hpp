/// @file LinearAlgebra.hpp
/// @brief Vector space operations
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-24

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace Eigen
{
using Array3ld = Array<long double, 3, 1>;

using Vector3ld = Matrix<long double, 3, 1>;
using Vector4ld = Matrix<long double, 4, 1>;

using Matrix3ld = Matrix<long double, 3, 3>;
using Matrix4ld = Matrix<long double, 4, 4>;

using Quaternionld = Quaternion<long double>;

using AngleAxisld = AngleAxis<long double>;
} // namespace Eigen

namespace NAV::alg
{
} // namespace NAV::alg
