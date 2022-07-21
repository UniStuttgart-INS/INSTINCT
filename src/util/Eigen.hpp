/// @file Eigen.hpp
/// @brief Vector space operations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-05

#pragma once

#include <util/Logger.hpp>
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
            if (std::isnan(matrix(r, c))) { j[std::to_string(r)][std::to_string(c)] = "NaN"; }
            else { j[std::to_string(r)][std::to_string(c)] = matrix(r, c); }
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
            if (j.at(std::to_string(r)).at(std::to_string(c)).is_string())
            {
                auto str = j.at(std::to_string(r)).at(std::to_string(c)).get<std::string>();
                if (str == "NaN") { matrix(r, c) = std::nan(""); }
                else { LOG_WARN("Reading matrix value failed at position ({}, {}). Value is an unknown string '{}'.", r, c, str); }
            }
            else if (j.at(std::to_string(r)).at(std::to_string(c)).is_number())
            {
                j.at(std::to_string(r)).at(std::to_string(c)).get_to(matrix(r, c));
            }
            else
            {
                LOG_WARN("Reading matrix value failed at position ({}, {}). Value has the type '{}' which cannot be converted into a floating point number.", r, c, j.at(std::to_string(r)).at(std::to_string(c)).type_name());
            }
        }
    }
}

} // namespace Eigen

namespace NAV
{
namespace experimental
{
class Matrix;
} // namespace experimental

/// Wrapper for Eigen::Block<Eigen::MatrixXd>
class BlockMatrix
{
  public:
    /// @brief Default constructor
    BlockMatrix() = default;

    /// @brief Constructor
    /// @param[in, out] matrix Matrix object where the block is taken from
    /// @param[in] pinName Name of the pin, where the block will be represented
    /// @param[in] startRow The first row in the block
    /// @param[in] startCol The first column in the block
    /// @param[in] blockRows The number of rows in the block
    /// @param[in] blockCols The number of columns in the block
    BlockMatrix(Eigen::MatrixXd& matrix, std::string pinName, int startRow, int startCol, int blockRows, int blockCols);

    /// @brief Gets the block matrix
    Eigen::Block<Eigen::MatrixXd> operator()();

    /// @brief Converts the class to a json object
    /// @return The json object
    [[nodiscard]] json to_json() const;
    /// @brief Fills this object with the information from the provided json object
    /// @param[in] j Json object
    void from_json(const json& j);

    friend class NAV::experimental::Matrix;

  private:
    /// @brief Pointer to the underlying Matrix
    Eigen::MatrixXd* matrix = nullptr;

    /// @brief Name of the pin
    std::string pinName;

    /// @brief Start row in the underlying matrix to access with index 0
    int startRow = 0;
    /// @brief Start col in the underlying matrix to access with index 0
    int startCol = 0;
    /// @brief Amount of rows to access in the underlying matrix
    int blockRows = 1;
    /// @brief Amount of cols to access in the underlying matrix
    int blockCols = 1;
};

/// @brief Converts the BlockMatrix into a json object
/// @param[out] j Json object to return
/// @param[in] data BlockMatrix to convert into json
void to_json(json& j, const BlockMatrix& data);
/// @brief Converts the json object into a BlockMatrix
/// @param[in] j Json object to read information from
/// @param[out] data Object to read the information into
void from_json(const json& j, BlockMatrix& data);
} // namespace NAV
