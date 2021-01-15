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
    for (int r = 0; r < matrix.rows(); r++)
    {
        for (int c = 0; c < matrix.cols(); c++)
        {
            j[std::to_string(r)][std::to_string(c)] = matrix(r, c);
        }
    }
}

template<typename _Scalar, int _Rows, int _Cols>
void from_json(const json& j, Matrix<_Scalar, _Rows, _Cols>& matrix)
{
    if constexpr (_Rows == -1 || _Cols == -1)
    {
        int rows = -1;
        int cols = -1;
        for (int r = 0; j.contains(std::to_string(r)); r++)
        {
            rows = std::max(r, rows);
            for (int c = 0; j.at(std::to_string(r)).contains(std::to_string(c)); c++)
            {
                cols = std::max(c, cols);
            }
        }
        matrix = Eigen::Matrix<_Scalar, _Rows, _Cols>::Zero(rows + 1, cols + 1);
    }
    for (int r = 0; j.contains(std::to_string(r)); r++)
    {
        for (int c = 0; j.at(std::to_string(r)).contains(std::to_string(c)); c++)
        {
            j.at(std::to_string(r)).at(std::to_string(c)).get_to(matrix(r, c));
        }
    }
}

} // namespace Eigen
