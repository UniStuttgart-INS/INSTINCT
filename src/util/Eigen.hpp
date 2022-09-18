// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Eigen.hpp
/// @brief Vector space operations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-05

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace Eigen
{
using Array3ld = Array<long double, 3, 1>; ///< Long double 3x1 Eigen::Array

using Vector3ld = Matrix<long double, 3, 1>; ///< Long double 3x1 Eigen::Vector
using Vector4ld = Matrix<long double, 4, 1>; ///< Long double 3x1 Eigen::Vector

using Matrix3ld = Matrix<long double, 3, 3>; ///< Long double 3x3 Eigen::Matrix
using Matrix4ld = Matrix<long double, 4, 4>; ///< Long double 4x4 Eigen::Matrix

using Quaternionld = Quaternion<long double>; ///< Long double Eigen::Quaternion

using AngleAxisld = AngleAxis<long double>; ///< Long double Eigen::AngleAxis

/// @brief Converts the provided matrix into a json objetc
/// @tparam _Scalar Data Type of the matrix
/// @tparam _Rows Amount of rows of the matrix
/// @tparam _Cols Amount of cols of the matrix
/// @param[out] j Json object to fill with
/// @param[in] matrix Matrix to convert into json
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

/// @brief Converts the provided json object into a matrix
/// @tparam _Scalar Data Type of the matrix
/// @tparam _Rows Amount of rows of the matrix
/// @tparam _Cols Amount of cols of the matrix
/// @param[in] j Json object to read the coefficients from
/// @param[out] matrix Matrix object to fill
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

    for (int r = 0; j.contains(std::to_string(r)); r++) // NOLINT(readability-misleading-indentation)
    {
        for (int c = 0; j.at(std::to_string(r)).contains(std::to_string(c)); c++)
        {
            j.at(std::to_string(r)).at(std::to_string(c)).get_to(matrix(r, c));
        }
    }
}

} // namespace Eigen
