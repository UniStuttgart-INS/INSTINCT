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

#include <fmt/ostream.h>

#include "Assert.h"

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

namespace NAV
{

/// @brief Removes rows from a matrix or vector
/// @param matrix Matrix to remove from
/// @param index Index to start removing
/// @param length Length to remove
template<typename Derived>
void removeRows(Eigen::DenseBase<Derived>& matrix, size_t index, size_t length)
{
    INS_ASSERT_USER_ERROR(static_cast<size_t>(matrix.rows()) >= index + length, "Tried to remove rows which do not exist");

    std::vector<int> indicesToKeep;
    indicesToKeep.reserve(static_cast<size_t>(matrix.rows()) - length);
    for (int i = 0; i < static_cast<int>(index); i++) { indicesToKeep.push_back(i); }
    for (int i = static_cast<int>(index + length); i < matrix.rows(); i++) { indicesToKeep.push_back(i); }

    matrix = matrix(indicesToKeep, Eigen::all).eval();
}

/// @brief Removes rows from a matrix or vector
/// @param matrix Matrix to remove from
/// @param rowIndices List with indices of rows to remove
template<typename Derived>
void removeRows(Eigen::DenseBase<Derived>& matrix, const std::vector<int>& rowIndices)
{
    std::vector<int> rowIndicesToKeep;
    rowIndicesToKeep.reserve(static_cast<size_t>(matrix.rows()) - rowIndices.size());
    for (int i = 0; i < matrix.rows(); i++)
    {
        if (std::find(rowIndices.begin(), rowIndices.end(), i) == rowIndices.end())
        {
            rowIndicesToKeep.push_back(i);
        }
    }

    matrix = matrix(rowIndicesToKeep, Eigen::all).eval();
}

/// @brief Removes columns from a matrix or vector
/// @param matrix Matrix to remove from
/// @param index Index to start removing
/// @param length Length to remove
template<typename Derived>
void removeCols(Eigen::DenseBase<Derived>& matrix, size_t index, size_t length)
{
    INS_ASSERT_USER_ERROR(static_cast<size_t>(matrix.cols()) >= index + length, "Tried to remove cols which do not exist");

    std::vector<int> indicesToKeep;
    indicesToKeep.reserve(static_cast<size_t>(matrix.cols()) - length);
    for (int i = 0; i < static_cast<int>(index); i++) { indicesToKeep.push_back(i); }
    for (int i = static_cast<int>(index + length); i < matrix.cols(); i++) { indicesToKeep.push_back(i); }

    matrix = matrix(Eigen::all, indicesToKeep).eval();
}

/// @brief Removes cols from a matrix or vector
/// @param matrix Matrix to remove from
/// @param colIndices List with indices of cols to remove
template<typename Derived>
void removeCols(Eigen::DenseBase<Derived>& matrix, const std::vector<int>& colIndices)
{
    std::vector<int> colIndicesToKeep;
    colIndicesToKeep.reserve(static_cast<size_t>(matrix.cols()) - colIndices.size());
    for (int i = 0; i < matrix.cols(); i++)
    {
        if (std::find(colIndices.begin(), colIndices.end(), i) == colIndices.end())
        {
            colIndicesToKeep.push_back(i);
        }
    }

    matrix = matrix(Eigen::all, colIndicesToKeep).eval();
}

/// @brief Removes rows and columns from a matrix or vector
/// @param matrix Matrix to remove from
/// @param row Row index to start removing
/// @param rows Amount of rows to remove
/// @param col Col index to start removing
/// @param cols Amount of cols to remove
template<typename Derived>
void removeRowsAndCols(Eigen::DenseBase<Derived>& matrix, size_t row, size_t rows, size_t col, size_t cols)
{
    INS_ASSERT_USER_ERROR(static_cast<size_t>(matrix.rows()) >= row + rows, "Tried to remove rows which do not exist");
    INS_ASSERT_USER_ERROR(static_cast<size_t>(matrix.cols()) >= col + cols, "Tried to remove cols which do not exist");

    std::vector<int> rowsToKeep;
    rowsToKeep.reserve(static_cast<size_t>(matrix.rows()) - rows);
    for (int i = 0; i < static_cast<int>(row); i++) { rowsToKeep.push_back(i); }
    for (int i = static_cast<int>(row + rows); i < matrix.rows(); i++) { rowsToKeep.push_back(i); }

    std::vector<int> colsToKeep;
    colsToKeep.reserve(static_cast<size_t>(matrix.cols()) - cols);
    for (int i = 0; i < static_cast<int>(col); i++) { colsToKeep.push_back(i); }
    for (int i = static_cast<int>(col + cols); i < matrix.cols(); i++) { colsToKeep.push_back(i); }

    matrix = matrix(rowsToKeep, colsToKeep).eval();
}

/// @brief Removes rows and columns from a matrix or vector
/// @param matrix Matrix to remove from
/// @param rowIndices List with indices of rows to remove
/// @param colIndices List with indices of cols to remove
template<typename Derived>
void removeRowsAndCols(Eigen::DenseBase<Derived>& matrix, const std::vector<int>& rowIndices, const std::vector<int>& colIndices)
{
    std::vector<int> rowIndicesToKeep;
    rowIndicesToKeep.reserve(static_cast<size_t>(matrix.rows()) - rowIndices.size());
    for (int i = 0; i < matrix.rows(); i++)
    {
        if (std::find(rowIndices.begin(), rowIndices.end(), i) == rowIndices.end())
        {
            rowIndicesToKeep.push_back(i);
        }
    }

    std::vector<int> colIndicesToKeep;
    colIndicesToKeep.reserve(static_cast<size_t>(matrix.cols()) - colIndices.size());
    for (int i = 0; i < matrix.cols(); i++)
    {
        if (std::find(colIndices.begin(), colIndices.end(), i) == colIndices.end())
        {
            colIndicesToKeep.push_back(i);
        }
    }

    matrix = matrix(rowIndicesToKeep, colIndicesToKeep).eval();
}

} // namespace NAV

#ifndef DOXYGEN_IGNORE

template<typename T>
    requires std::is_base_of_v<Eigen::DenseBase<T>, T>
struct fmt::formatter<T> : ostream_formatter
{};

// FIXME: This is not compiling with gcc 11.3 but with >12.1.
// template<typename T>
// requires std::is_base_of_v<Eigen::QuaternionBase<T>, T>
// struct fmt::formatter<T> : ostream_formatter
// {};
template<>
struct fmt::formatter<Eigen::Quaterniond> : ostream_formatter
{};

#endif