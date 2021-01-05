/// @file Eigen.hpp
/// @brief Vector space operations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-05

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace Eigen
{
using Array3ld = Array<long double, 3, 1>;

using Vector3ld = Matrix<long double, 3, 1>;
using Vector4ld = Matrix<long double, 4, 1>;

using Matrix3ld = Matrix<long double, 3, 3>;
using Matrix4ld = Matrix<long double, 4, 4>;

using Quaternionld = Quaternion<long double>;

using AngleAxisld = AngleAxis<long double>;

template<typename _Scalar, int _Rows, int _Cols>
void to_json(json& j, const Matrix<_Scalar, _Rows, _Cols>& matrix)
{
    for (int r = 0; r < _Rows; r++)
    {
        for (int c = 0; c < _Cols; c++)
        {
            j[std::to_string(r).c_str()][std::to_string(c).c_str()] = matrix(r, c);
        }
    }
}

template<typename _Scalar, int _Rows, int _Cols>
void from_json(const json& j, Matrix<_Scalar, _Rows, _Cols>& matrix)
{
    for (int r = 0; r < _Rows; r++)
    {
        for (int c = 0; c < _Cols; c++)
        {
            j.at(std::to_string(r).c_str()).at(std::to_string(c).c_str()).get_to(matrix(r, c));
        }
    }
}

} // namespace Eigen
