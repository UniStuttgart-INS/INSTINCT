// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedMatrix.hpp
/// @brief Matrix which can be accessed by keys
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @note Based on ideas from Kevin Gutsche (kevin.gutsche@ins.uni-stuttgart.de) and Bayram Stucke (bayram.stucke@ins.uni-stuttgart.de)
/// @date 2023-07-06

#include <unordered_set>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <ranges>
#include "util/Assert.h"
#include "util/Eigen.hpp"

namespace NAV
{

namespace internal
{
/// @brief All type to request all rows or columns in KeyedMatrices
struct all_t
{
    /// @brief Default Constructor
    all_t() = default;
};

/// @brief KeyedMatrix Base class to inherit common methods for static and dynamic sized
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, typename ColKeyType = RowKeyType, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
class KeyedMatrixBase
{
  public:
    /// @brief Constructor
    /// @param matrix Eigen Matrix to initialize from
    explicit KeyedMatrixBase(Eigen::Matrix<Scalar, Rows, Cols> matrix) : matrix(std::move(matrix)) {}

    // #######################################################################################################
    //                                                Access
    // #######################################################################################################

    /// @brief Returns the row keys
    const std::vector<RowKeyType>& rowKeys() const { return rowKeysVector; }
    /// @brief Returns the col keys
    const std::vector<ColKeyType>& colKeys() const { return colKeysVector; }

    /// @brief Checks if the matrix has the key
    /// @param key Row key to check for
    bool hasRow(const RowKeyType& key) const { return rowIndices.contains(key); }
    /// @brief Checks if the matrix has the key
    /// @param key Col key to check for
    bool hasCol(const ColKeyType& key) const { return colIndices.contains(key); }
    /// @brief Checks if the matrix has multiple keys
    /// @param keys Row keys to check for
    bool hasRows(const std::vector<RowKeyType>& keys) const
    {
        return std::all_of(keys.begin(), keys.end(), [&](const RowKeyType& key) { return hasRow(key); });
    }
    /// @brief Checks if the matrix has multiple keys
    /// @param keys Col keys to check for
    bool hasCols(const std::vector<ColKeyType>& keys) const
    {
        return std::all_of(keys.begin(), keys.end(), [&](const ColKeyType& key) { return hasCol(key); });
    }

    /// @brief Gets the value for the row and col key
    /// @param rowKey Row Key
    /// @param colKey Col Key
    /// @return Scalar value
    const Scalar& operator()(const RowKeyType& rowKey, const ColKeyType& colKey) const
    {
        return matrix(rowIndices.at(rowKey), colIndices.at(colKey));
    }
    /// @brief Gets the value for the row and col key
    /// @param rowKey Row Key
    /// @param colKey Col Key
    /// @return Scalar value
    Scalar& operator()(const RowKeyType& rowKey, const ColKeyType& colKey)
    {
        return matrix(rowIndices.at(rowKey), colIndices.at(colKey));
    }

    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        std::vector<Eigen::Index> rowIdx;
        rowIdx.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIdx.push_back(rowIndices.at(rowKey)); }

        std::vector<Eigen::Index> colIdx;
        colIdx.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIdx.push_back(colIndices.at(colKey)); }

        return matrix(rowIdx, colIdx);
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        std::vector<Eigen::Index> rowIdx;
        rowIdx.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIdx.push_back(rowIndices.at(rowKey)); }

        std::vector<Eigen::Index> colIdx;
        colIdx.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIdx.push_back(colIndices.at(colKey)); }

        return matrix(rowIdx, colIdx);
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const ColKeyType& colKey) const { return (*this)(rowKeys, std::vector{ colKey }); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const ColKeyType& colKey) { return (*this)(rowKeys, std::vector{ colKey }); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const RowKeyType& rowKey, const std::vector<ColKeyType>& colKeys) const { return (*this)(std::vector{ rowKey }, colKeys); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const RowKeyType& rowKey, const std::vector<ColKeyType>& colKeys) { return (*this)(std::vector{ rowKey }, colKeys); }

    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const RowKeyType& rowKey, all_t /* all */) const { return (*this)(std::vector{ rowKey }, colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const RowKeyType& rowKey, all_t /* all */) { return (*this)(std::vector{ rowKey }, colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const ColKeyType& colKey) const { return *this(rowKeys(), std::vector{ colKey }); }
    /// @brief Gets the values for the row and col keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const ColKeyType& colKey) { return (*this)(rowKeys(), std::vector{ colKey }); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, all_t /* all */) const { return (*this)(rowKeys, colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, all_t /* all */) { return (*this)(rowKeys, colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const std::vector<ColKeyType>& colKeys) const { return (*this)(rowKeys(), colKeys); }
    /// @brief Gets the values for the row and col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const std::vector<ColKeyType>& colKeys) { return (*this)(rowKeys(), colKeys); }

    /// @brief Requests the full matrix
    const Eigen::Matrix<Scalar, Rows, Cols>& operator()(all_t /* all */, all_t /* all */) const { return matrix; }
    /// @brief Requests the full matrix
    Eigen::Matrix<Scalar, Rows, Cols>& operator()(all_t /* all */, all_t /* all */) { return matrix; }
    /// @brief Conversion into Eigen::Matrix
    explicit operator Eigen::Matrix<Scalar, Rows, Cols>() { return matrix; }

    // ######################################################################################################
    //                                              Operations
    // ######################################################################################################

    /// @brief Returns the transposed keyed matrix
    KeyedMatrixBase<Scalar, ColKeyType, RowKeyType, Cols, Rows> transposed() const
    {
        return { matrix.transpose(), colKeys(), rowKeys() };
    }

    /// @brief Returns the inverted keyed matrix
    KeyedMatrixBase inverse() const
    {
        return { matrix.inverse(), rowKeys(), colKeys() };
    }

  protected:
    std::unordered_map<RowKeyType, Eigen::Index> rowIndices; ///< RowKey to Row Index mapping
    std::unordered_map<ColKeyType, Eigen::Index> colIndices; ///< ColKey to Col Index mapping

    std::vector<RowKeyType> rowKeysVector; ///< Row Keys
    std::vector<ColKeyType> colKeysVector; ///< Col Keys

    Eigen::Matrix<Scalar, Rows, Cols> matrix; ///< Data storage of the type
};

} // namespace internal

/// @brief Used to request all rows or columns in KeyedMatrices
static const internal::all_t all;

// Static size

/// @brief Static sized KeyedMatrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, typename ColKeyType = RowKeyType, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
class KeyedMatrix
    : public internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>
{
  public:
    /// @brief Non-symmetric matrix constructor
    /// @param matrix Eigen matrix to initialize from
    /// @param rowKeys Row keys describing the matrix
    /// @param colKeys Col keys describing the matrix
    KeyedMatrix(Eigen::Matrix<Scalar, Rows, Cols> matrix,
                const std::array<RowKeyType, static_cast<size_t>(Rows)>& rowKeys,
                const std::array<ColKeyType, static_cast<size_t>(Cols)>& colKeys)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(std::move(matrix))
    {
        std::unordered_set<RowKeyType> rowSet = { rowKeys.begin(), rowKeys.end() };
        INS_ASSERT_USER_ERROR(rowSet.size() == rowKeys.size(), "Each row key must be unique");
        std::unordered_set<ColKeyType> colSet = { colKeys.begin(), colKeys.end() };
        INS_ASSERT_USER_ERROR(colSet.size() == colKeys.size(), "Each col key must be unique");

        for (size_t i = 0; i < rowKeys.size(); i++) { this->rowIndices.insert({ rowKeys.at(i), static_cast<Eigen::Index>(i) }); }
        for (size_t i = 0; i < colKeys.size(); i++) { this->colIndices.insert({ colKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->rowKeysVector.assign(rowKeys.begin(), rowKeys.end());
        this->colKeysVector.assign(colKeys.begin(), colKeys.end());
    }

    /// @brief Symmetric matrix constructor
    /// @param matrix Eigen matrix to initialize from
    /// @param keys Row and col keys describing the matrix
    KeyedMatrix(Eigen::Matrix<Scalar, Rows, Cols> matrix, const std::array<RowKeyType, static_cast<size_t>(Rows)>& keys)
        : KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>(matrix, keys, keys) {}
};

/// @brief Dynamic sized KeyedMatrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
template<typename Scalar, typename RowKeyType, typename ColKeyType>
class KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>
    : public internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>
{
  public:
    /// @brief Default Constructor
    KeyedMatrix()
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(Eigen::MatrixX<Scalar>()) {}

    /// @brief Non-symmetric matrix constructor
    /// @param matrix Eigen matrix to initialize from
    /// @param rowKeys Row keys describing the matrix
    /// @param colKeys Col keys describing the matrix
    KeyedMatrix(Eigen::MatrixX<Scalar> matrix, const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(std::move(matrix))
    {
        std::unordered_set<RowKeyType> rowSet = { rowKeys.begin(), rowKeys.end() };
        INS_ASSERT_USER_ERROR(rowSet.size() == rowKeys.size(), "Each row key must be unique");
        std::unordered_set<ColKeyType> colSet = { colKeys.begin(), colKeys.end() };
        INS_ASSERT_USER_ERROR(colSet.size() == colKeys.size(), "Each col key must be unique");

        INS_ASSERT_USER_ERROR(this->matrix.rows() == static_cast<Eigen::Index>(rowKeys.size()), "Number of matrix rows doesn't correspond to the amount of row keys");
        INS_ASSERT_USER_ERROR(this->matrix.cols() == static_cast<Eigen::Index>(colKeys.size()), "Number of matrix cols doesn't correspond to the amount of col keys");

        for (size_t i = 0; i < rowKeys.size(); i++) { this->rowIndices.insert({ rowKeys.at(i), static_cast<Eigen::Index>(i) }); }
        for (size_t i = 0; i < colKeys.size(); i++) { this->colIndices.insert({ colKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->rowKeysVector = rowKeys;
        this->colKeysVector = colKeys;
    }

    /// @brief Symmetric matrix constructor
    /// @param matrix Eigen matrix to initialize from
    /// @param keys Row and col keys describing the matrix
    KeyedMatrix(const Eigen::MatrixX<Scalar>& matrix, const std::vector<RowKeyType>& keys)
        : KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(matrix, keys, keys) {}

    /// @brief Adds a new row to the matrix
    /// @param rowKey Row key
    void addRow(const RowKeyType& rowKey) { addRows({ rowKey }); }
    /// @brief Adds a new col to the matrix
    /// @param colKey Col key
    void addCol(const ColKeyType& colKey) { addCols({ colKey }); }

    /// @brief Adds new rows to the matrix
    /// @param rowKeys Row keys
    void addRows(const std::vector<RowKeyType>& rowKeys)
    {
        for ([[maybe_unused]] const auto& rowKey : rowKeys)
        {
            INS_ASSERT_USER_ERROR(!this->rowIndices.contains(rowKey), "You cannot add a row key which is already in the matrix.");
        }

        auto initialSize = static_cast<Eigen::Index>(this->rowIndices.size());
        for (const auto& rowKey : rowKeys) { this->rowIndices.insert({ rowKey, static_cast<Eigen::Index>(this->rowIndices.size()) }); }
        this->rowKeysVector.reserve(this->rowKeysVector.size() + rowKeys.size());
        std::copy(rowKeys.begin(), rowKeys.end(), std::back_inserter(this->rowKeysVector));
        auto finalSize = static_cast<Eigen::Index>(this->rowIndices.size());

        if (finalSize > initialSize)
        {
            this->matrix.conservativeResize(finalSize, Eigen::NoChange);
            this->matrix.block(initialSize, 0, finalSize - initialSize, this->matrix.cols()) = Eigen::MatrixX<Scalar>::Zero(finalSize - initialSize, this->matrix.cols());
        }
    }
    /// @brief Adds new cols to the matrix
    /// @param colKeys Col keys
    void addCols(const std::vector<ColKeyType>& colKeys)
    {
        for ([[maybe_unused]] const auto& colKey : colKeys)
        {
            INS_ASSERT_USER_ERROR(!this->colIndices.contains(colKey), "You cannot add a col key which is already in the matrix.");
        }

        auto initialSize = static_cast<Eigen::Index>(this->colIndices.size());
        for (const auto& colKey : colKeys) { this->colIndices.insert({ colKey, static_cast<Eigen::Index>(this->colIndices.size()) }); }
        this->colKeysVector.reserve(this->colKeysVector.size() + colKeys.size());
        std::copy(colKeys.begin(), colKeys.end(), std::back_inserter(this->colKeysVector));
        auto finalSize = static_cast<Eigen::Index>(this->colIndices.size());

        if (finalSize > initialSize)
        {
            this->matrix.conservativeResize(Eigen::NoChange, finalSize);
            this->matrix.block(0, initialSize, this->matrix.rows(), finalSize - initialSize) = Eigen::MatrixX<Scalar>::Zero(this->matrix.rows(), finalSize - initialSize);
        }
    }

    /// @brief Removes the row from the matrix
    /// @param rowKey Row Key
    void removeRow(const RowKeyType& rowKey) { removeRows({ rowKey }); }
    /// @brief Removes the col from the matrix
    /// @param colKey Col Key
    void removeCol(const ColKeyType& colKey) { removeCols({ colKey }); }

    /// @brief Removes the rows from the matrix
    /// @param rowKeys Row Keys
    void removeRows(const std::vector<RowKeyType>& rowKeys)
    {
        std::vector<int> indices;
        for (const auto& rowKey : rowKeys)
        {
            auto iter = std::find_if(this->rowIndices.begin(), this->rowIndices.end(), [&](const auto& item) { return item.first == rowKey; });
            if (iter != this->rowIndices.end())
            {
                indices.push_back(static_cast<int>(iter->second));
            }
        }
        NAV::removeRows(this->matrix, indices);

        for (const auto& rowKey : rowKeys)
        {
            auto iter = std::find_if(this->rowIndices.begin(), this->rowIndices.end(), [&](const auto& item) { return item.first == rowKey; });
            if (iter != this->rowIndices.end())
            {
                std::erase_if(this->rowKeysVector, [&](const auto& item) { return item == rowKey; });

                auto idx = iter->second;
                for (auto& rowIndex : this->rowIndices)
                {
                    if (rowIndex.second > idx) { rowIndex.second--; }
                }
                this->rowIndices.erase(iter);
            }
        }
    }

    /// @brief Removes the cols from the matrix
    /// @param colKeys Col Keys
    void removeCols(const std::vector<ColKeyType>& colKeys)
    {
        std::vector<int> indices;
        for (const auto& colKey : colKeys)
        {
            auto iter = std::find_if(this->colIndices.begin(), this->colIndices.end(), [&](const auto& item) { return item.first == colKey; });
            if (iter != this->colIndices.end())
            {
                indices.push_back(static_cast<int>(iter->second));
            }
        }
        NAV::removeCols(this->matrix, indices);

        for (const auto& colKey : colKeys)
        {
            auto iter = std::find_if(this->colIndices.begin(), this->colIndices.end(), [&](const auto& item) { return item.first == colKey; });
            if (iter != this->colIndices.end())
            {
                std::erase_if(this->colKeysVector, [&](const auto& item) { return item == colKey; });

                auto idx = iter->second;
                for (auto& colIndex : this->colIndices)
                {
                    if (colIndex.second > idx) { colIndex.second--; }
                }
                this->colIndices.erase(iter);
            }
        }
    }
};

} // namespace NAV
