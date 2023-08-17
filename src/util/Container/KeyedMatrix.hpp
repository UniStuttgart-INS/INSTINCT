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

#pragma once

#include <unordered_set>
#include <vector>
#include <array>
#include <algorithm>
#include <ranges>
#include <type_traits>
#include "util/Assert.h"
#include "util/Eigen.hpp"
#include "util/Container/Unordered_map.hpp"

#pragma GCC diagnostic push
#if !defined(__clang__) && defined(__GNUC__)
    #pragma GCC diagnostic ignored "-Wvirtual-move-assign" // NOLINT(clang-diagnostic-unknown-warning-option)
#endif

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

/// @brief KeyedMatrix storage class
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, int Rows, int Cols>
class KeyedMatrixStorage
{
  protected:
    Eigen::Matrix<Scalar, Rows, Cols> matrix; ///< Data storage of the type

  private:
    template<typename Scalar_, typename RowKeyType_, typename ColKeyType_, int Rows_, int Cols_>
    friend class KeyedMatrixBase;
    template<typename Scalar_, typename RowKeyType_, int Rows_>
    friend class KeyedVectorBase;
    template<typename Scalar_, typename ColKeyType_, int Cols_>
    friend class KeyedRowVectorBase;
};

// ###########################################################################################################

/// @brief Base class for Keyed matrices with multiple rows
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Rows, int Cols>
class KeyedMatrixRowsBase : virtual public KeyedMatrixStorage<Scalar, Rows, Cols>
{
  public:
    /// @brief Return the rows of the underlying Eigen matrix
    [[nodiscard]] decltype(auto) rows() const { return this->matrix.rows(); }

    /// @brief Returns the row keys
    const std::vector<RowKeyType>& rowKeys() const { return rowKeysVector; }

    /// @brief Checks if the matrix has the key
    /// @param key Row key to check for
    bool hasRow(const RowKeyType& key) const { return rowIndices.contains(key); }

    /// @brief Checks if the matrix has multiple keys
    /// @param keys Row keys to check for
    bool hasRows(const std::vector<RowKeyType>& keys) const
    {
        return std::all_of(keys.begin(), keys.end(), [&](const RowKeyType& key) { return hasRow(key); });
    }

    /// @brief Checks if the matrix has any key
    /// @param keys Row keys to check for
    bool hasAnyRows(const std::vector<RowKeyType>& keys) const
    {
        return std::any_of(keys.begin(), keys.end(), [&](const RowKeyType& key) { return hasRow(key); });
    }

  protected:
    /// RowKey to Row Index mapping
    unordered_map<RowKeyType, Eigen::Index> rowIndices;
    /// Row Keys
    std::vector<RowKeyType> rowKeysVector;

    /// Row Slice used for accessing
    mutable std::vector<Eigen::Index> rowSlice;

  private:
    template<typename Scalar_, typename RowKeyType_, typename ColKeyType_, int Rows_, int Cols_>
    friend class KeyedMatrixBase;
    template<typename Scalar_, typename RowKeyType_, int Rows_>
    friend class KeyedVectorBase;
};

/// @brief Base class for Keyed matrices with multiple rows of static size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Rows, int Cols>
class KeyedMatrixRows : public KeyedMatrixRowsBase<Scalar, RowKeyType, Rows, Cols>
{};

/// @brief Base class for Keyed matrices with multiple rows of dynamic size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Cols>
class KeyedMatrixRows<Scalar, RowKeyType, Eigen::Dynamic, Cols> : public KeyedMatrixRowsBase<Scalar, RowKeyType, Eigen::Dynamic, Cols>
{
  public:
    /// @brief Adds a new row to the matrix
    /// @param rowKey Row key
    void addRow(const RowKeyType& rowKey) { addRows({ rowKey }); }

    /// @brief Adds new rows to the matrix
    /// @param rowKeys Row keys
    void addRows(const std::vector<RowKeyType>& rowKeys)
    {
        INS_ASSERT_USER_ERROR(!this->hasAnyRows(rowKeys), "You cannot add a row key which is already in the matrix.");
        INS_ASSERT_USER_ERROR(std::unordered_set<RowKeyType>(rowKeys.begin(), rowKeys.end()).size() == rowKeys.size(), "Each row key must be unique");

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
        this->rowSlice.reserve(this->rowKeysVector.size());
    }

    /// @brief Removes the row from the matrix
    /// @param rowKey Row Key
    void removeRow(const RowKeyType& rowKey) { removeRows({ rowKey }); }

    /// @brief Removes the rows from the matrix
    /// @param rowKeys Row Keys
    void removeRows(const std::vector<RowKeyType>& rowKeys)
    {
        std::vector<int> indices;
        for (const auto& rowKey : rowKeys)
        {
            auto iter = std::find_if(this->rowIndices.begin(), this->rowIndices.end(), [&](const auto& item) { return item.first == rowKey; });
            INS_ASSERT_USER_ERROR(iter != this->rowIndices.end(), "You tried removing a row key, which did not exist.");
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
};

// ###########################################################################################################

/// @brief Base class for Keyed matrices with multiple columns
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Rows, int Cols>
class KeyedMatrixColsBase : virtual public KeyedMatrixStorage<Scalar, Rows, Cols>
{
  public:
    /// @brief Return the cols of the underlying Eigen matrix
    [[nodiscard]] decltype(auto) cols() const { return this->matrix.cols(); }

    /// @brief Returns the col keys
    const std::vector<ColKeyType>& colKeys() const { return colKeysVector; }

    /// @brief Checks if the matrix has the key
    /// @param key Col key to check for
    bool hasCol(const ColKeyType& key) const { return colIndices.contains(key); }

    /// @brief Checks if the matrix has multiple keys
    /// @param keys Col keys to check for
    bool hasCols(const std::vector<ColKeyType>& keys) const
    {
        return std::all_of(keys.begin(), keys.end(), [&](const ColKeyType& key) { return hasCol(key); });
    }

    /// @brief Checks if the matrix has any keys
    /// @param keys Col keys to check for
    bool hasAnyCols(const std::vector<ColKeyType>& keys) const
    {
        return std::any_of(keys.begin(), keys.end(), [&](const ColKeyType& key) { return hasCol(key); });
    }

  protected:
    /// ColKey to Col Index mapping
    unordered_map<ColKeyType, Eigen::Index> colIndices;
    /// Col Keys
    std::vector<ColKeyType> colKeysVector;

    /// Col Slice used for accessing
    mutable std::vector<Eigen::Index> colSlice;

  private:
    template<typename Scalar_, typename RowKeyType_, typename ColKeyType_, int Rows_, int Cols_>
    friend class KeyedMatrixBase;
    template<typename Scalar_, typename ColKeyType_, int Cols_>
    friend class KeyedRowVectorBase;
};

/// @brief Base class for Keyed matrices with multiple columns of static size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Rows, int Cols>
class KeyedMatrixCols : public KeyedMatrixColsBase<Scalar, ColKeyType, Rows, Cols>
{};

/// @brief Base class for Keyed matrices with multiple columns of dynamic size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Rows>
class KeyedMatrixCols<Scalar, ColKeyType, Rows, Eigen::Dynamic> : public KeyedMatrixColsBase<Scalar, ColKeyType, Rows, Eigen::Dynamic>
{
  public:
    /// @brief Adds a new col to the matrix
    /// @param colKey Col key
    void addCol(const ColKeyType& colKey) { addCols({ colKey }); }

    /// @brief Adds new cols to the matrix
    /// @param colKeys Col keys
    void addCols(const std::vector<ColKeyType>& colKeys)
    {
        INS_ASSERT_USER_ERROR(!this->hasAnyCols(colKeys), "You cannot add a col key which is already in the matrix.");
        INS_ASSERT_USER_ERROR(std::unordered_set<ColKeyType>(colKeys.begin(), colKeys.end()).size() == colKeys.size(), "Each col key must be unique");

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
        this->colSlice.reserve(this->colKeysVector.size());
    }

    /// @brief Removes the col from the matrix
    /// @param colKey Col Key
    void removeCol(const ColKeyType& colKey) { removeCols({ colKey }); }

    /// @brief Removes the cols from the matrix
    /// @param colKeys Col Keys
    void removeCols(const std::vector<ColKeyType>& colKeys)
    {
        std::vector<int> indices;
        for (const auto& colKey : colKeys)
        {
            auto iter = std::find_if(this->colIndices.begin(), this->colIndices.end(), [&](const auto& item) { return item.first == colKey; });
            INS_ASSERT_USER_ERROR(iter != this->colIndices.end(), "You tried removing a col key, which did not exist.");
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

// ###########################################################################################################

template<typename Scalar, typename ColKeyType, int Cols>
class KeyedRowVectorBase;

/// @brief Class to inherit common methods for static and dynamic sized vectors
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Rows>
class KeyedVectorBase : public KeyedMatrixRows<Scalar, RowKeyType, Rows, 1>
{
  public:
    /// @brief Constructor
    /// @param vector Eigen vector to initialize from
    template<typename Derived>
    explicit KeyedVectorBase(const Eigen::MatrixBase<Derived>& vector)
    {
        this->matrix = vector;
    }

    /// @brief Constructor
    /// @param vector Eigen vector to initialize from
    /// @param rowKeys Row keys describing the vector
    template<typename Derived>
    KeyedVectorBase(const Eigen::MatrixBase<Derived>& vector, const std::vector<RowKeyType>& rowKeys)
    {
        INS_ASSERT_USER_ERROR(std::unordered_set<RowKeyType>(rowKeys.begin(), rowKeys.end()).size() == rowKeys.size(), "Each row key must be unique");

        INS_ASSERT_USER_ERROR(vector.cols() == 1, "Only vectors with 1 column are allowed.");
        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || vector.rows() == static_cast<int>(rowKeys.size()), "Number of vector rows doesn't correspond to the amount of row keys");

        for (size_t i = 0; i < rowKeys.size(); i++) { this->rowIndices.insert({ rowKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->matrix = vector;
        this->rowKeysVector = rowKeys;
        this->rowSlice.reserve(this->rowKeysVector.size());
    }

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedVectorBase() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedVectorBase(const KeyedVectorBase& other)
    {
        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedVectorBase& operator=(const KeyedVectorBase& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedVectorBase(KeyedVectorBase&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedVectorBase& operator=(KeyedVectorBase&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oRows>
    KeyedVectorBase(const KeyedVectorBase<Scalar, RowKeyType, oRows>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    {
        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || other.rows() == Rows, "Can only copy construct dynamic<=>static matrices if the rows match");

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    template<int oRows>
    KeyedVectorBase& operator=(const KeyedVectorBase<Scalar, RowKeyType, oRows>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rowKeys() == this->rowKeys(), "Can only copy assign matrices if the row keys match");

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    template<int oRows>
    KeyedVectorBase(KeyedVectorBase<Scalar, RowKeyType, oRows>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    {
        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || other.rows() == Rows, "Can only copy construct dynamic<=>static matrices if the rows match");

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Move assignment operator
    /// @param other The other object
    template<int oRows>
    KeyedVectorBase& operator=(KeyedVectorBase<Scalar, RowKeyType, oRows>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rowKeys() == this->rowKeys(), "Can only copy assign matrices if the row keys match");

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }

    // #######################################################################################################
    //                                                Access
    // #######################################################################################################

    /// @brief Gets the value for the row key
    /// @param rowKey Row Key
    /// @return Scalar value
    const Scalar& operator()(const RowKeyType& rowKey) const
    {
        return this->matrix(this->rowIndices.at(rowKey), 0);
    }
    /// @brief Gets the value for the row key
    /// @param rowKey Row Key
    /// @return Scalar value
    Scalar& operator()(const RowKeyType& rowKey)
    {
        return this->matrix(this->rowIndices.at(rowKey), 0);
    }

    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys) const
    {
        this->rowSlice.clear();
        for (const auto& rowKey : rowKeys) { this->rowSlice.push_back(this->rowIndices.at(rowKey)); }

        return this->matrix(this->rowSlice, 0);
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys)
    {
        this->rowSlice.clear();
        for (const auto& rowKey : rowKeys) { this->rowSlice.push_back(this->rowIndices.at(rowKey)); }

        return this->matrix(this->rowSlice, 0);
    }

    /// @brief Requests the full vector
    const Eigen::Matrix<Scalar, Rows, 1>& operator()(all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full vector
    Eigen::Matrix<Scalar, Rows, 1>& operator()(all_t /* all */) { return this->matrix; }
    /// @brief Conversion into Eigen::Vector
    explicit operator Eigen::Vector<Scalar, Rows>() { return this->matrix; }

    // #######################################################################################################
    //                                           Block operations
    // #######################################################################################################

    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    template<size_t P>
    decltype(auto) segment(const std::vector<RowKeyType>& rowKeys) const
    {
        checkContinuousSegment(rowKeys, P);

        return this->matrix.template middleRows<P>(this->rowIndices.at(rowKeys.at(0)));
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    template<size_t P>
    decltype(auto) segment(const std::vector<RowKeyType>& rowKeys)
    {
        checkContinuousSegment(rowKeys, P);

        return this->matrix.template middleRows<P>(this->rowIndices.at(rowKeys.at(0)));
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) segment(const std::vector<RowKeyType>& rowKeys) const
    {
        checkContinuousSegment(rowKeys, rowKeys.size());

        return this->matrix.middleRows(this->rowIndices.at(rowKeys.at(0)), rowKeys.size());
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) segment(const std::vector<RowKeyType>& rowKeys)
    {
        checkContinuousSegment(rowKeys, rowKeys.size());

        return this->matrix.middleRows(this->rowIndices.at(rowKeys.at(0)), rowKeys.size());
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed vector
    [[nodiscard]] KeyedRowVectorBase<Scalar, RowKeyType, Rows> transposed() const
    {
        return { this->matrix.transpose(), this->rowKeys() };
    }

  private:
    /// @brief Checks if the row keys are describing a continuous block
    /// @param rowKeys Row keys
    /// @param P Size of the row keys
    void checkContinuousSegment([[maybe_unused]] const std::vector<RowKeyType>& rowKeys, [[maybe_unused]] size_t P) const
    {
#ifndef NDEBUG
        INS_ASSERT_USER_ERROR(P == rowKeys.size(), "The block size must be equivalent to the amount of row keys.");

        std::vector<Eigen::Index> consecutiveRows(rowKeys.size());
        std::iota(std::begin(consecutiveRows), std::end(consecutiveRows), this->rowIndices.at(rowKeys.at(0)));
        std::vector<Eigen::Index> rowIndices;
        rowIndices.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIndices.push_back(this->rowIndices.at(rowKey)); }
        INS_ASSERT_USER_ERROR(rowIndices == consecutiveRows, "The given rowKeys must describe a consecutive part in the matrix.");
#endif
    }
};

/// @brief Class to inherit common methods for static and dynamic sized row vectors
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Cols>
class KeyedRowVectorBase : public KeyedMatrixCols<Scalar, ColKeyType, 1, Cols>
{
  public:
    /// @brief Constructor
    /// @param vector Eigen vector to initialize from
    template<typename Derived>
    explicit KeyedRowVectorBase(const Eigen::MatrixBase<Derived>& vector)
    {
        this->matrix = vector;
    }

    /// @brief Constructor
    /// @param vector Eigen vector to initialize from
    /// @param colKeys Col keys describing the vector
    template<typename Derived>
    KeyedRowVectorBase(const Eigen::MatrixBase<Derived>& vector, const std::vector<ColKeyType>& colKeys)
    {
        INS_ASSERT_USER_ERROR(std::unordered_set<ColKeyType>(colKeys.begin(), colKeys.end()).size() == colKeys.size(), "Each col key must be unique");

        INS_ASSERT_USER_ERROR(vector.rows() == 1, "Only vectors with 1 row are allowed.");
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || vector.cols() == static_cast<Eigen::Index>(colKeys.size()), "Number of vector cols doesn't correspond to the amount of col keys");

        for (size_t i = 0; i < colKeys.size(); i++) { this->colIndices.insert({ colKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->matrix = vector;
        this->colKeysVector = colKeys;
        this->colSlice.reserve(this->colKeysVector.size());
    }

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedRowVectorBase() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedRowVectorBase(const KeyedRowVectorBase& other)
    {
        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedRowVectorBase& operator=(const KeyedRowVectorBase& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedRowVectorBase(KeyedRowVectorBase&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedRowVectorBase& operator=(KeyedRowVectorBase&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oCols>
    KeyedRowVectorBase(const KeyedRowVectorBase<Scalar, ColKeyType, oCols>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    {
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || other.cols() == Cols, "Can only copy construct dynamic<=>static matrices if the cols match");

        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    template<int oCols>
    KeyedRowVectorBase& operator=(const KeyedRowVectorBase<Scalar, ColKeyType, oCols>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.colKeys() == this->colKeys(), "Can only copy assign matrices if the col keys match");

        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    template<int oCols>
    KeyedRowVectorBase(KeyedRowVectorBase<Scalar, ColKeyType, oCols>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    {
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || other.cols() == Cols, "Can only copy construct dynamic<=>static matrices if the cols match");

        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());
    }
    /// @brief Move assignment operator
    /// @param other The other object
    template<int oCols>
    KeyedRowVectorBase& operator=(KeyedRowVectorBase<Scalar, ColKeyType, oCols>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.colKeys() == this->colKeys(), "Can only copy assign matrices if the col keys match");

        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());

        return *this;
    }

    // #######################################################################################################
    //                                                Access
    // #######################################################################################################

    /// @brief Gets the value for the col key
    /// @param colKey Col Key
    /// @return Scalar value
    const Scalar& operator()(const ColKeyType& colKey) const
    {
        return this->matrix(0, this->colIndices.at(colKey));
    }
    /// @brief Gets the value for the col key
    /// @param colKey Col Key
    /// @return Scalar value
    Scalar& operator()(const ColKeyType& colKey)
    {
        return this->matrix(0, this->colIndices.at(colKey));
    }

    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) operator()(const std::vector<ColKeyType>& colKeys) const
    {
        this->colSlice.clear();
        for (const auto& colKey : colKeys) { this->colSlice.push_back(this->colIndices.at(colKey)); }

        return this->matrix(0, this->colSlice);
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) operator()(const std::vector<ColKeyType>& colKeys)
    {
        this->colSlice.clear();
        for (const auto& colKey : colKeys) { this->colSlice.push_back(this->colIndices.at(colKey)); }

        return this->matrix(0, this->colSlice);
    }

    /// @brief Requests the full vector
    const Eigen::Matrix<Scalar, 1, Cols>& operator()(all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full vector
    Eigen::Matrix<Scalar, 1, Cols>& operator()(all_t /* all */) { return this->matrix; }
    /// @brief Conversion into Eigen::RowVector
    explicit operator Eigen::RowVector<Scalar, Cols>() { return this->matrix; }

    // #######################################################################################################
    //                                           Block operations
    // #######################################################################################################

    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    template<size_t Q>
    decltype(auto) segment(const std::vector<ColKeyType>& colKeys) const
    {
        checkContinuousSegment(colKeys, Q);

        return this->matrix.template middleCols<Q>(this->colIndices.at(colKeys.at(0)));
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    template<size_t Q>
    decltype(auto) segment(const std::vector<ColKeyType>& colKeys)
    {
        checkContinuousSegment(colKeys, Q);

        return this->matrix.template middleCols<Q>(this->colIndices.at(colKeys.at(0)));
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) segment(const std::vector<ColKeyType>& colKeys) const
    {
        checkContinuousSegment(colKeys, colKeys.size());

        return this->matrix.middleCols(this->colIndices.at(colKeys.at(0)), colKeys.size());
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) segment(const std::vector<ColKeyType>& colKeys)
    {
        checkContinuousSegment(colKeys, colKeys.size());

        return this->matrix.middleCols(this->colIndices.at(colKeys.at(0)), colKeys.size());
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed vector
    [[nodiscard]] KeyedVectorBase<Scalar, ColKeyType, Cols> transposed() const
    {
        return { this->matrix.transpose(), this->colKeys() };
    }

  private:
    /// @brief Checks if the col keys are describing a continuous block
    /// @param colKeys Col keys
    /// @param Q Size of the col keys
    void checkContinuousSegment([[maybe_unused]] const std::vector<ColKeyType>& colKeys, [[maybe_unused]] size_t Q) const
    {
#ifndef NDEBUG
        INS_ASSERT_USER_ERROR(Q == colKeys.size(), "The block size must be equivalent to the amount of col keys.");

        std::vector<Eigen::Index> consecutiveCols(colKeys.size());
        std::iota(std::begin(consecutiveCols), std::end(consecutiveCols), this->colIndices.at(colKeys.at(0)));
        std::vector<Eigen::Index> colIndices;
        colIndices.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIndices.push_back(this->colIndices.at(colKey)); }
        INS_ASSERT_USER_ERROR(colIndices == consecutiveCols, "The given colKeys must describe a consecutive part in the matrix.");
#endif
    }
};

/// @brief Class to inherit common methods for static and dynamic sized matrices
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
class KeyedMatrixBase : public KeyedMatrixRows<Scalar, RowKeyType, Rows, Cols>, public KeyedMatrixCols<Scalar, ColKeyType, Rows, Cols>
{
  public:
    /// @brief Constructor
    /// @param matrix Eigen Matrix to initialize from
    template<typename Derived>
    explicit KeyedMatrixBase(const Eigen::MatrixBase<Derived>& matrix)
    {
        this->matrix = matrix;
    }
    /// @brief Non-symmetric matrix constructor
    /// @param matrix Eigen Matrix to initialize from
    /// @param rowKeys Row keys describing the matrix
    /// @param colKeys Col keys describing the matrix
    template<typename Derived>
    KeyedMatrixBase(const Eigen::MatrixBase<Derived>& matrix, const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        INS_ASSERT_USER_ERROR(std::unordered_set<RowKeyType>(rowKeys.begin(), rowKeys.end()).size() == rowKeys.size(), "Each row key must be unique");
        INS_ASSERT_USER_ERROR(std::unordered_set<ColKeyType>(colKeys.begin(), colKeys.end()).size() == colKeys.size(), "Each col key must be unique");

        INS_ASSERT_USER_ERROR(matrix.rows() == static_cast<Eigen::Index>(rowKeys.size()), "Number of matrix rows doesn't correspond to the amount of row keys");
        INS_ASSERT_USER_ERROR(matrix.cols() == static_cast<Eigen::Index>(colKeys.size()), "Number of matrix cols doesn't correspond to the amount of col keys");

        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || Rows == static_cast<int>(rowKeys.size()), "Number of matrix rows doesn't correspond to the static amount of row keys");
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || Cols == static_cast<int>(colKeys.size()), "Number of matrix cols doesn't correspond to the static amount of col keys");

        for (size_t i = 0; i < rowKeys.size(); i++) { this->rowIndices.insert({ rowKeys.at(i), static_cast<Eigen::Index>(i) }); }
        for (size_t i = 0; i < colKeys.size(); i++) { this->colIndices.insert({ colKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->matrix = matrix;
        this->rowKeysVector = rowKeys;
        this->colKeysVector = colKeys;
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());
    }

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedMatrixBase() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixBase(const KeyedMatrixBase& other)
    {
        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixBase& operator=(const KeyedMatrixBase& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixBase(KeyedMatrixBase&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixBase& operator=(KeyedMatrixBase&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrixBase(const KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, oRows, oCols>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    {
        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || other.rows() == Rows, "Can only copy construct dynamic<=>static matrices if the rows match");
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || other.cols() == Cols, "Can only copy construct dynamic<=>static matrices if the cols match");

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrixBase& operator=(const KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, oRows, oCols>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rowKeys() == this->rowKeys(), "Can only copy assign matrices if the row keys match");
        INS_ASSERT_USER_ERROR(other.colKeys() == this->colKeys(), "Can only copy assign matrices if the col keys match");

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrixBase(KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, oRows, oCols>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    {
        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || other.rows() == Rows, "Can only copy construct dynamic<=>static matrices if the rows match");
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || other.cols() == Cols, "Can only copy construct dynamic<=>static matrices if the cols match");

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());
    }
    /// @brief Move assignment operator
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrixBase& operator=(KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, oRows, oCols>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rowKeys() == this->rowKeys(), "Can only copy assign matrices if the row keys match");
        INS_ASSERT_USER_ERROR(other.colKeys() == this->colKeys(), "Can only copy assign matrices if the col keys match");

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());

        return *this;
    }

    // #######################################################################################################
    //                                                Access
    // #######################################################################################################

    /// @brief Gets the value for the row and col key
    /// @param rowKey Row Key
    /// @param colKey Col Key
    /// @return Scalar value
    const Scalar& operator()(const RowKeyType& rowKey, const ColKeyType& colKey) const
    {
        return this->matrix(this->rowIndices.at(rowKey), this->colIndices.at(colKey));
    }
    /// @brief Gets the value for the row and col key
    /// @param rowKey Row Key
    /// @param colKey Col Key
    /// @return Scalar value
    Scalar& operator()(const RowKeyType& rowKey, const ColKeyType& colKey)
    {
        auto rowIdx = this->rowIndices.at(rowKey);
        auto colIdx = this->colIndices.at(colKey);
        return this->matrix(rowIdx, colIdx);
    }

    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        this->rowSlice.clear();
        for (const auto& rowKey : rowKeys) { this->rowSlice.push_back(this->rowIndices.at(rowKey)); }

        this->colSlice.clear();
        for (const auto& colKey : colKeys) { this->colSlice.push_back(this->colIndices.at(colKey)); }

        return this->matrix(this->rowSlice, this->colSlice);
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        this->rowSlice.clear();
        for (const auto& rowKey : rowKeys) { this->rowSlice.push_back(this->rowIndices.at(rowKey)); }

        this->colSlice.clear();
        for (const auto& colKey : colKeys) { this->colSlice.push_back(this->colIndices.at(colKey)); }

        return this->matrix(this->rowSlice, this->colSlice);
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

    /// @brief Gets the values for the row key
    /// @param rowKey Row Key
    /// @return View into the matrix for the row key
    decltype(auto) operator()(const RowKeyType& rowKey, all_t /* all */) const { return (*this)(std::vector{ rowKey }, this->colKeys()); }
    /// @brief Gets the values for the row key
    /// @param rowKey Row Key
    /// @return View into the matrix for the row key
    decltype(auto) operator()(const RowKeyType& rowKey, all_t /* all */) { return (*this)(std::vector{ rowKey }, this->colKeys()); }
    /// @brief Gets the values for the col key
    /// @param colKey Col Key
    /// @return View into the matrix for the col key
    decltype(auto) operator()(all_t /* all */, const ColKeyType& colKey) const { return *this(this->rowKeys(), std::vector{ colKey }); }
    /// @brief Gets the values for the col keys
    /// @param colKey Col Key
    /// @return View into the matrix for the col key
    decltype(auto) operator()(all_t /* all */, const ColKeyType& colKey) { return (*this)(this->rowKeys(), std::vector{ colKey }); }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, all_t /* all */) const { return (*this)(rowKeys, this->colKeys()); }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, all_t /* all */) { return (*this)(rowKeys, this->colKeys()); }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) operator()(all_t /* all */, const std::vector<ColKeyType>& colKeys) const { return (*this)(this->rowKeys(), colKeys); }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) operator()(all_t /* all */, const std::vector<ColKeyType>& colKeys) { return (*this)(this->rowKeys(), colKeys); }

    /// @brief Requests the full matrix
    const Eigen::Matrix<Scalar, Rows, Cols>& operator()(all_t /* all */, all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full matrix
    Eigen::Matrix<Scalar, Rows, Cols>& operator()(all_t /* all */, all_t /* all */) { return this->matrix; }
    /// @brief Conversion into Eigen::Matrix
    explicit operator Eigen::Matrix<Scalar, Rows, Cols>() { return this->matrix; }

    // ###########################################################################################################
    //                                          Static Block operations
    // ###########################################################################################################

    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    template<size_t P, size_t Q = P>
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        checkContinuousBlock(rowKeys, colKeys, P, Q);

        return this->matrix.template block<P, Q>(this->rowIndices.at(rowKeys.at(0)), this->colIndices.at(colKeys.at(0)));
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    template<size_t P, size_t Q = P>
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        checkContinuousBlock(rowKeys, colKeys, P, Q);

        return this->matrix.template block<P, Q>(this->rowIndices.at(rowKeys.at(0)), this->colIndices.at(colKeys.at(0)));
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    template<size_t P>
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const ColKeyType& colKey) const
    {
        return this->block<P, 1>(rowKeys, std::vector{ colKey });
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    template<size_t P>
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const ColKeyType& colKey)
    {
        return this->block<P, 1>(rowKeys, std::vector{ colKey });
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    template<size_t Q>
    decltype(auto) block(const RowKeyType& rowKey, const std::vector<ColKeyType>& colKeys) const
    {
        return this->block<1, Q>(std::vector{ rowKey }, colKeys);
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    template<size_t Q>
    decltype(auto) block(const RowKeyType& rowKey, const std::vector<ColKeyType>& colKeys)
    {
        return this->block<1, Q>(std::vector{ rowKey }, colKeys);
    }

    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    template<size_t P>
    decltype(auto) middleRows(const std::vector<RowKeyType>& rowKeys) const
    {
        checkContinuousBlock(rowKeys, this->colKeys(), P, this->colKeys().size());

        return this->matrix.template middleRows<P>(this->rowIndices.at(rowKeys.at(0)));
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    template<size_t P>
    decltype(auto) middleRows(const std::vector<RowKeyType>& rowKeys)
    {
        checkContinuousBlock(rowKeys, this->colKeys(), P, this->colKeys().size());

        return this->matrix.template middleRows<P>(this->rowIndices.at(rowKeys.at(0)));
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    template<size_t Q>
    decltype(auto) middleCols(const std::vector<ColKeyType>& colKeys) const
    {
        checkContinuousBlock(this->rowKeys(), colKeys, this->rowKeys().size(), Q);

        return this->matrix.template middleCols<Q>(this->colIndices.at(colKeys.at(0)));
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    template<size_t Q>
    decltype(auto) middleCols(const std::vector<ColKeyType>& colKeys)
    {
        checkContinuousBlock(this->rowKeys(), colKeys, this->rowKeys().size(), Q);

        return this->matrix.template middleCols<Q>(this->colIndices.at(colKeys.at(0)));
    }
    /// @brief Gets the values for the row key
    /// @param rowKey Row Key
    /// @return View into the matrix for the row key
    decltype(auto) row(const RowKeyType& rowKey) const { return this->matrix.row(this->rowIndices.at(rowKey)); }
    /// @brief Gets the values for the row key
    /// @param rowKey Row Key
    /// @return View into the matrix for the row key
    decltype(auto) row(const RowKeyType& rowKey) { return this->matrix.row(this->rowIndices.at(rowKey)); }
    /// @brief Gets the values for the col key
    /// @param colKey Col Key
    /// @return View into the matrix for the col key
    decltype(auto) col(const ColKeyType& colKey) const { return this->matrix.col(this->colIndices.at(colKey)); }
    /// @brief Gets the values for the col key
    /// @param colKey Col Key
    /// @return View into the matrix for the col key
    decltype(auto) col(const ColKeyType& colKey) { return this->matrix.col(this->colIndices.at(colKey)); }

    /// @brief Requests the full matrix
    const Eigen::Matrix<Scalar, Rows, Cols>& block(all_t /* all */, all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full matrix
    Eigen::Matrix<Scalar, Rows, Cols>& block(all_t /* all */, all_t /* all */) { return this->matrix; }

    // ###########################################################################################################
    //                                         Dynamic block operations
    // ###########################################################################################################

    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        checkContinuousBlock(rowKeys, colKeys, rowKeys.size(), colKeys.size());

        return this->matrix.block(this->rowIndices.at(rowKeys.at(0)), this->colIndices.at(colKeys.at(0)), rowKeys.size(), colKeys.size());
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        checkContinuousBlock(rowKeys, colKeys, rowKeys.size(), colKeys.size());

        return this->matrix.block(this->rowIndices.at(rowKeys.at(0)), this->colIndices.at(colKeys.at(0)), rowKeys.size(), colKeys.size());
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const ColKeyType& colKey) const
    {
        return this->block(rowKeys, std::vector{ colKey });
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) block(const std::vector<RowKeyType>& rowKeys, const ColKeyType& colKey)
    {
        return this->block(rowKeys, std::vector{ colKey });
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) block(const RowKeyType& rowKey, const std::vector<ColKeyType>& colKeys) const
    {
        return this->block(std::vector{ rowKey }, colKeys);
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) block(const RowKeyType& rowKey, const std::vector<ColKeyType>& colKeys)
    {
        return this->block(std::vector{ rowKey }, colKeys);
    }

    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) middleRows(const std::vector<RowKeyType>& rowKeys) const
    {
        checkContinuousBlock(rowKeys, this->colKeys(), rowKeys.size(), this->colKeys().size());

        return this->matrix.middleRows(this->rowIndices.at(rowKeys.at(0)), rowKeys.size());
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) middleRows(const std::vector<RowKeyType>& rowKeys)
    {
        checkContinuousBlock(rowKeys, this->colKeys(), rowKeys.size(), this->colKeys().size());

        return this->matrix.middleRows(this->rowIndices.at(rowKeys.at(0)), rowKeys.size());
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) middleCols(const std::vector<ColKeyType>& colKeys) const
    {
        checkContinuousBlock(this->rowKeys(), colKeys, this->rowKeys().size(), colKeys.size());

        return this->matrix.middleCols(this->colIndices.at(colKeys.at(0)), colKeys.size());
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) middleCols(const std::vector<ColKeyType>& colKeys)
    {
        checkContinuousBlock(this->rowKeys(), colKeys, this->rowKeys().size(), colKeys.size());

        return this->matrix.middleCols(this->colIndices.at(colKeys.at(0)), colKeys.size());
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed matrix
    [[nodiscard]] KeyedMatrixBase<Scalar, ColKeyType, RowKeyType, Cols, Rows> transposed() const
    {
        return { this->matrix.transpose(), this->colKeys(), this->rowKeys() };
    }

    /// @brief Calculates the inverse matrix
    [[nodiscard]] KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols> inverse() const
    {
        return { this->matrix.inverse(), this->rowKeys(), this->colKeys() };
    }

  private:
    /// @brief Checks if the row and col keys are describing a continuous block
    /// @param rowKeys Row keys
    /// @param colKeys Col keys
    /// @param P Size of the row keys
    /// @param Q Size of the col keys
    void checkContinuousBlock([[maybe_unused]] const std::vector<RowKeyType>& rowKeys, [[maybe_unused]] const std::vector<ColKeyType>& colKeys,
                              [[maybe_unused]] size_t P, [[maybe_unused]] size_t Q) const
    {
#ifndef NDEBUG
        INS_ASSERT_USER_ERROR(P == rowKeys.size(), "The block size must be equivalent to the amount of row keys.");
        INS_ASSERT_USER_ERROR(Q == colKeys.size(), "The block size must be equivalent to the amount of col keys.");

        std::vector<Eigen::Index> consecutiveRows(rowKeys.size());
        std::iota(std::begin(consecutiveRows), std::end(consecutiveRows), this->rowIndices.at(rowKeys.at(0)));
        std::vector<Eigen::Index> rowIndices;
        rowIndices.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIndices.push_back(this->rowIndices.at(rowKey)); }
        INS_ASSERT_USER_ERROR(rowIndices == consecutiveRows, "The given rowKeys must describe a consecutive part in the matrix.");

        std::vector<Eigen::Index> consecutiveCols(colKeys.size());
        std::iota(std::begin(consecutiveCols), std::end(consecutiveCols), this->colIndices.at(colKeys.at(0)));
        std::vector<Eigen::Index> colIndices;
        colIndices.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIndices.push_back(this->colIndices.at(colKey)); }
        INS_ASSERT_USER_ERROR(colIndices == consecutiveCols, "The given colKeys must describe a consecutive part in the matrix.");
#endif
    }
};

} // namespace internal

/// @brief Used to request all rows or columns in KeyedMatrices
static const internal::all_t all;

template<typename Scalar, typename ColKeyType, int Cols>
class KeyedRowVector;

/// @brief Static sized KeyedVector
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Rows>
class KeyedVector : public internal::KeyedVectorBase<Scalar, RowKeyType, Rows>
{
  public:
    /// @brief Vector constructor
    /// @tparam Derived Derived Eigen Type
    /// @param vector Eigen vector to initialize from
    /// @param rowKeys Row keys describing the vector
    template<typename Derived>
    KeyedVector(const Eigen::MatrixBase<Derived>& vector, const std::vector<RowKeyType>& rowKeys)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Rows>(vector, rowKeys)
    {}

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedVector() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedVector(const KeyedVector& other)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Rows>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedVector& operator=(const KeyedVector& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(*this) =
            static_cast<const internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedVector(KeyedVector&& other) noexcept
        : internal::KeyedVectorBase<Scalar, RowKeyType, Rows>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedVector& operator=(KeyedVector&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(*this) =
            std::move(static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(other));

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    KeyedVector(const KeyedVector<Scalar, RowKeyType, Eigen::Dynamic>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Rows>(other)
    {
        INS_ASSERT_USER_ERROR(other.rows() == Rows, "Can only copy assign dynamic matrices from static ones if the size matches");
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedVector& operator=(const KeyedVector<Scalar, RowKeyType, Eigen::Dynamic>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rows() == Rows, "Can only copy assign dynamic matrices from static ones if the size matches");

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(*this) =
            static_cast<const internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedVector(KeyedVector<Scalar, RowKeyType, Eigen::Dynamic>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Rows>(std::move(other))
    {
        INS_ASSERT_USER_ERROR(this->rows() == Rows, "Can only move construct dynamic matrices from static ones if the size matches");
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedVector& operator=(KeyedVector<Scalar, RowKeyType, Eigen::Dynamic>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rows() == Rows, "Can only move assign dynamic matrices from static ones if the size matches");

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(*this) =
            std::move(static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(other));

        return *this;
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed vector
    [[nodiscard]] KeyedRowVector<Scalar, RowKeyType, Rows> transposed() const
    {
        auto transpose = static_cast<const internal::KeyedVectorBase<Scalar, RowKeyType, Rows>&>(*this).transposed();
        return { transpose(all), transpose.colKeys() };
    }
};

/// @brief Dynamic sized KeyedVector
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
template<typename Scalar, typename RowKeyType>
class KeyedVector<Scalar, RowKeyType, Eigen::Dynamic> : public internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>
{
  public:
    /// @brief Default Constructor
    KeyedVector()
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(Eigen::VectorX<Scalar>()) {}

    /// @brief Vector constructor
    /// @tparam Derived Derived Eigen Type
    /// @param vector Eigen vector to initialize from
    /// @param rowKeys Row keys describing the vector
    template<typename Derived>
    KeyedVector(const Eigen::MatrixBase<Derived>& vector, const std::vector<RowKeyType>& rowKeys)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(vector, rowKeys)
    {}

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedVector() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedVector(const KeyedVector& other)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedVector& operator=(const KeyedVector& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(*this) =
            static_cast<const internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedVector(KeyedVector&& other) noexcept
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedVector& operator=(KeyedVector&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(*this) =
            std::move(static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(other));

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oRows>
    KeyedVector(const KeyedVector<Scalar, RowKeyType, oRows>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    template<int oRows>
    KeyedVector& operator=(const KeyedVector<Scalar, RowKeyType, oRows>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(*this) =
            static_cast<const internal::KeyedVectorBase<Scalar, RowKeyType, oRows>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    template<int oRows>
    KeyedVector(KeyedVector<Scalar, RowKeyType, oRows>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    template<int oRows>
    KeyedVector& operator=(KeyedVector<Scalar, RowKeyType, oRows>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(*this) =
            std::move(static_cast<internal::KeyedVectorBase<Scalar, RowKeyType, oRows>&>(other));

        return *this;
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed vector
    [[nodiscard]] KeyedRowVector<Scalar, RowKeyType, Eigen::Dynamic> transposed() const
    {
        auto transpose = static_cast<const internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>&>(*this).transposed();
        return { transpose(all), transpose.colKeys() };
    }
};

/// @brief Static sized KeyedRowVector
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Cols>
class KeyedRowVector : public internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>
{
  public:
    /// @brief RowVector constructor
    /// @tparam Derived Derived Eigen Type
    /// @param vector Eigen vector to initialize from
    /// @param colKeys Col keys describing the vector
    template<typename Derived>
    KeyedRowVector(const Eigen::MatrixBase<Derived>& vector, const std::vector<ColKeyType>& colKeys)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>(vector, colKeys)
    {}

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedRowVector() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedRowVector(const KeyedRowVector& other)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedRowVector& operator=(const KeyedRowVector& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(*this) =
            static_cast<const internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedRowVector(KeyedRowVector&& other) noexcept
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedRowVector& operator=(KeyedRowVector&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(*this) =
            std::move(static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(other));

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    KeyedRowVector(const KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>(other)
    {
        INS_ASSERT_USER_ERROR(other.cols() == Cols, "Can only copy assign dynamic matrices from static ones if the size matches");
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedRowVector& operator=(const KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.cols() == Cols, "Can only copy assign dynamic matrices from static ones if the size matches");

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(*this) =
            static_cast<const internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedRowVector(KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>(std::move(other))
    {
        INS_ASSERT_USER_ERROR(this->cols() == Cols, "Can only move construct dynamic matrices from static ones if the size matches");
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedRowVector& operator=(KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.cols() == Cols, "Can only move assign dynamic matrices from static ones if the size matches");

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(*this) =
            std::move(static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(other));

        return *this;
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed vector
    [[nodiscard]] KeyedVector<Scalar, ColKeyType, Cols> transposed() const
    {
        auto transpose = static_cast<const internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>&>(*this).transposed();
        return { transpose(all), transpose.rowKeys() };
    }
};

/// @brief Dynamic sized KeyedRowVector
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
template<typename Scalar, typename ColKeyType>
class KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>
    : public internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>
{
  public:
    /// @brief Default Constructor
    KeyedRowVector()
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(Eigen::RowVectorX<Scalar>()) {}

    /// @brief RowVector constructor
    /// @tparam Derived Derived Eigen Type
    /// @param vector Eigen vector to initialize from
    /// @param colKeys Col keys describing the vector
    template<typename Derived>
    KeyedRowVector(const Eigen::MatrixBase<Derived>& vector, const std::vector<ColKeyType>& colKeys)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(vector, colKeys)
    {}

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedRowVector() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedRowVector(const KeyedRowVector& other)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedRowVector& operator=(const KeyedRowVector& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(*this) =
            static_cast<const internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedRowVector(KeyedRowVector&& other) noexcept
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedRowVector& operator=(KeyedRowVector&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(*this) =
            std::move(static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(other));

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oCols>
    KeyedRowVector(const KeyedRowVector<Scalar, ColKeyType, oCols>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    template<int oCols>
    KeyedRowVector& operator=(const KeyedRowVector<Scalar, ColKeyType, oCols>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(*this) =
            static_cast<const internal::KeyedRowVectorBase<Scalar, ColKeyType, oCols>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    template<int oCols>
    KeyedRowVector(KeyedRowVector<Scalar, ColKeyType, oCols>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    template<int oCols>
    KeyedRowVector& operator=(KeyedRowVector<Scalar, ColKeyType, oCols>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(*this) =
            std::move(static_cast<internal::KeyedRowVectorBase<Scalar, ColKeyType, oCols>&>(other));

        return *this;
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Calculates the transposed vector
    [[nodiscard]] KeyedVector<Scalar, ColKeyType, Eigen::Dynamic> transposed() const
    {
        auto transpose = static_cast<const internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>&>(*this).transposed();
        return { transpose(all), transpose.rowKeys() };
    }
};

/// @brief Static sized KeyedMatrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
class KeyedMatrix : public internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>
{
  public:
    /// @brief Non-symmetric matrix constructor
    /// @tparam Derived Derived Eigen Type
    /// @param matrix Eigen matrix to initialize from
    /// @param rowKeys Row keys describing the matrix
    /// @param colKeys Col keys describing the matrix
    template<typename Derived>
    KeyedMatrix(const Eigen::MatrixBase<Derived>& matrix, const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(matrix, rowKeys, colKeys)
    {}

    /// @brief Symmetric matrix constructor
    /// @tparam Derived Derived Eigen Type
    /// @param matrix Eigen matrix to initialize from
    /// @param keys Row and col keys describing the matrix
    template<typename Derived>
    KeyedMatrix(const Eigen::MatrixBase<Derived>& matrix, const std::vector<RowKeyType>& keys)
        : KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>(matrix, keys, keys)
    {}

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedMatrix() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrix(const KeyedMatrix& other)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrix& operator=(const KeyedMatrix& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(*this) =
            static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrix(KeyedMatrix&& other) noexcept
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrix& operator=(KeyedMatrix&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(*this) =
            std::move(static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(other));

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrix(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(other)
    {
        INS_ASSERT_USER_ERROR(other.rows() == Rows && other.cols() == Cols, "Can only copy assign dynamic matrices from static ones if the size matches");
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrix& operator=(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rows() == Rows && other.cols() == Cols, "Can only copy assign dynamic matrices from static ones if the size matches");

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(*this) =
            static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrix(KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(std::move(other))
    {
        INS_ASSERT_USER_ERROR(this->rows() == Rows && this->cols() == Cols, "Can only move construct dynamic matrices from static ones if the size matches");
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrix& operator=(KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        INS_ASSERT_USER_ERROR(other.rows() == Rows && other.cols() == Cols, "Can only move assign dynamic matrices from static ones if the size matches");

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(*this) =
            std::move(static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(other));

        return *this;
    }

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Returns a submatrix specified by the row and col keys
    /// @param rowKeys Row keys
    /// @param colKeys Col keys
    [[nodiscard]] KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>
        getSubMatrix(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        if (rowKeys == this->rowKeysVector && colKeys == this->colKeysVector)
        {
            return *this;
        }

        return { (*this)(rowKeys, colKeys), rowKeys, colKeys };
    }

    /// @brief Calculates the transposed matrix
    [[nodiscard]] KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols> transposed() const
    {
        auto transpose = static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(*this).transposed();
        return { transpose(all, all), transpose.rowKeys(), transpose.colKeys() };
    }

    /// @brief Calculates the inverse matrix
    [[nodiscard]] KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols> inverse() const
    {
        auto inv = static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>&>(*this).inverse();
        return { inv(all, all), inv.rowKeys(), inv.colKeys() };
    }
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
    /// @tparam Derived Derived Eigen Type
    /// @param matrix Eigen matrix to initialize from
    /// @param rowKeys Row keys describing the matrix
    /// @param colKeys Col keys describing the matrix
    template<typename Derived>
    KeyedMatrix(const Eigen::MatrixBase<Derived>& matrix, const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(matrix, rowKeys, colKeys)
    {}

    /// @brief Symmetric matrix constructor
    /// @tparam Derived Derived Eigen Type
    /// @param matrix Eigen matrix to initialize from
    /// @param keys Row and col keys describing the matrix
    template<typename Derived>
    KeyedMatrix(const Eigen::MatrixBase<Derived>& matrix, const std::vector<RowKeyType>& keys)
        : KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(matrix, keys, keys)
    {}

    // #######################################################################################################
    //                                       Special member functions
    // #######################################################################################################

    /// @brief Destructor
    ~KeyedMatrix() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrix(const KeyedMatrix& other)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrix& operator=(const KeyedMatrix& other)
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(*this) =
            static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrix(KeyedMatrix&& other) noexcept
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrix& operator=(KeyedMatrix&& other) noexcept
    {
        if (this == &other) { return *this; } // Guard self assignment

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(*this) =
            std::move(static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(other));

        return *this;
    }

    // ###########################################################################################################
    //                             Special member functions with different Rows/Cols
    // ###########################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrix(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, oRows, oCols>& other) // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(other)
    {}
    /// @brief Copy assignment operator
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrix& operator=(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, oRows, oCols>& other)
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(*this) =
            static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, oRows, oCols>&>(other);

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrix(KeyedMatrix<Scalar, RowKeyType, ColKeyType, oRows, oCols>&& other) noexcept // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(std::move(other))
    {}
    /// @brief Move assignment operator
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrix& operator=(KeyedMatrix<Scalar, RowKeyType, ColKeyType, oRows, oCols>&& other) noexcept
    {
        // No need to guard self assignment, as the types are different, so it cannot be the same object

        static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(*this) =
            std::move(static_cast<internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, oRows, oCols>&>(other));

        return *this;
    }

    // #######################################################################################################
    //                                           Modifying methods
    // #######################################################################################################

    /// @brief Adds new rows and cols to the matrix
    /// @param rowKeys Row keys
    /// @param colKeys Col keys
    void addRowsCols(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        INS_ASSERT_USER_ERROR(!this->hasAnyRows(rowKeys), "You cannot add a row key which is already in the matrix.");
        INS_ASSERT_USER_ERROR(!this->hasAnyCols(colKeys), "You cannot add a col key which is already in the matrix.");
        INS_ASSERT_USER_ERROR(std::unordered_set<RowKeyType>(rowKeys.begin(), rowKeys.end()).size() == rowKeys.size(), "Each row key must be unique");
        INS_ASSERT_USER_ERROR(std::unordered_set<ColKeyType>(colKeys.begin(), colKeys.end()).size() == colKeys.size(), "Each col key must be unique");

        auto initialRowSize = static_cast<Eigen::Index>(this->rowIndices.size());
        for (const auto& rowKey : rowKeys) { this->rowIndices.insert({ rowKey, static_cast<Eigen::Index>(this->rowIndices.size()) }); }
        this->rowKeysVector.reserve(this->rowKeysVector.size() + rowKeys.size());
        std::copy(rowKeys.begin(), rowKeys.end(), std::back_inserter(this->rowKeysVector));
        auto finalRowSize = static_cast<Eigen::Index>(this->rowIndices.size());

        auto initialColSize = static_cast<Eigen::Index>(this->colIndices.size());
        for (const auto& colKey : colKeys) { this->colIndices.insert({ colKey, static_cast<Eigen::Index>(this->colIndices.size()) }); }
        this->colKeysVector.reserve(this->colKeysVector.size() + colKeys.size());
        std::copy(colKeys.begin(), colKeys.end(), std::back_inserter(this->colKeysVector));
        auto finalColSize = static_cast<Eigen::Index>(this->colIndices.size());

        auto rows = finalRowSize - initialRowSize;
        auto cols = finalColSize - initialColSize;
        if (rows > 0 || cols > 0)
        {
            this->matrix.conservativeResize(finalRowSize, finalColSize);
            this->matrix.block(initialRowSize, 0, rows, this->matrix.cols()) = Eigen::MatrixX<Scalar>::Zero(rows, this->matrix.cols());
            this->matrix.block(0, initialColSize, this->matrix.rows(), cols) = Eigen::MatrixX<Scalar>::Zero(this->matrix.rows(), cols);
        }
        this->colSlice.reserve(this->colKeysVector.size());
        this->rowSlice.reserve(this->rowKeysVector.size());
    }

    /// @brief Removes the rows and cols from the matrix
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    void removeRowsCols(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        std::vector<int> rowIndices;
        for (const auto& rowKey : rowKeys)
        {
            auto iter = std::find_if(this->rowIndices.begin(), this->rowIndices.end(), [&](const auto& item) { return item.first == rowKey; });
            INS_ASSERT_USER_ERROR(iter != this->rowIndices.end(), "You tried removing a row key, which did not exist.");
            if (iter != this->rowIndices.end())
            {
                rowIndices.push_back(static_cast<int>(iter->second));
            }
        }
        std::vector<int> colIndices;
        for (const auto& colKey : colKeys)
        {
            auto iter = std::find_if(this->colIndices.begin(), this->colIndices.end(), [&](const auto& item) { return item.first == colKey; });
            INS_ASSERT_USER_ERROR(iter != this->colIndices.end(), "You tried removing a col key, which did not exist.");
            if (iter != this->colIndices.end())
            {
                colIndices.push_back(static_cast<int>(iter->second));
            }
        }

        NAV::removeRowsAndCols(this->matrix, rowIndices, colIndices);

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

    // #######################################################################################################
    //                                                Methods
    // #######################################################################################################

    /// @brief Returns a submatrix specified by the row and col keys
    /// @param rowKeys Row keys
    /// @param colKeys Col keys
    [[nodiscard]] KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>
        getSubMatrix(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        if (rowKeys == this->rowKeysVector && colKeys == this->colKeysVector)
        {
            return *this;
        }

        return { (*this)(rowKeys, colKeys), rowKeys, colKeys };
    }

    /// @brief Calculates the transposed matrix
    [[nodiscard]] KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic> transposed() const
    {
        auto transpose = static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(*this).transposed();
        return { transpose(all, all), transpose.rowKeys(), transpose.colKeys() };
    }

    /// @brief Calculates the inverse matrix
    [[nodiscard]] KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic> inverse() const
    {
        auto inv = static_cast<const internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>&>(*this).inverse();
        return { inv(all, all), inv.rowKeys(), inv.colKeys() };
    }
};

/// @brief Dynamic size KeyedMatrix
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
template<typename Scalar, typename RowKeyType, typename ColKeyType = RowKeyType>
using KeyedMatrixX = KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>;
/// @brief Dynamic size KeyedMatrix with double types
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
template<typename RowKeyType, typename ColKeyType = RowKeyType>
using KeyedMatrixXd = KeyedMatrixX<double, RowKeyType, ColKeyType>;
/// @brief Static 2x2 squared size KeyedMatrix with double types
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
template<typename RowKeyType, typename ColKeyType = RowKeyType>
using KeyedMatrix2d = KeyedMatrix<double, RowKeyType, ColKeyType, 2, 2>;
/// @brief Static 3x3 squared size KeyedMatrix with double types
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
template<typename RowKeyType, typename ColKeyType = RowKeyType>
using KeyedMatrix3d = KeyedMatrix<double, RowKeyType, ColKeyType, 3, 3>;
/// @brief Static 4x4 squared size KeyedMatrix with double types
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam ColKeyType Type of the key used for col lookup
template<typename RowKeyType, typename ColKeyType = RowKeyType>
using KeyedMatrix4d = KeyedMatrix<double, RowKeyType, ColKeyType, 4, 4>;

/// @brief Dynamic size KeyedVector
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
template<typename Scalar, typename RowKeyType>
using KeyedVectorX = KeyedVector<Scalar, RowKeyType, Eigen::Dynamic>;
/// @brief Dynamic size KeyedVector with double types
/// @tparam RowKeyType Type of the key used for row lookup
template<typename RowKeyType>
using KeyedVectorXd = KeyedVectorX<double, RowKeyType>;
/// @brief Static 2 row KeyedVector with double types
/// @tparam RowKeyType Type of the key used for row lookup
template<typename RowKeyType>
using KeyedVector2d = KeyedVector<double, RowKeyType, 2>;
/// @brief Static 3 row KeyedVector with double types
/// @tparam RowKeyType Type of the key used for row lookup
template<typename RowKeyType>
using KeyedVector3d = KeyedVector<double, RowKeyType, 3>;
/// @brief Static 4 row KeyedVector with double types
/// @tparam RowKeyType Type of the key used for row lookup
template<typename RowKeyType>
using KeyedVector4d = KeyedVector<double, RowKeyType, 4>;

/// @brief Dynamic size KeyedRowVector
/// @tparam ColKeyType Type of the key used for col lookup
template<typename Scalar, typename ColKeyType>
using KeyedRowVectorX = KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>;
/// @brief Dynamic size KeyedRowVector with double types
/// @tparam ColKeyType Type of the key used for col lookup
template<typename ColKeyType>
using KeyedRowVectorXd = KeyedRowVectorX<double, ColKeyType>;
/// @brief Static 2 col KeyedRowVector with double types
/// @tparam ColKeyType Type of the key used for col lookup
template<typename ColKeyType>
using KeyedRowVector2d = KeyedRowVector<double, ColKeyType, 2>;
/// @brief Static 3 col KeyedRowVector with double types
/// @tparam ColKeyType Type of the key used for col lookup
template<typename ColKeyType>
using KeyedRowVector3d = KeyedRowVector<double, ColKeyType, 3>;
/// @brief Static 4 col KeyedRowVector with double types
/// @tparam ColKeyType Type of the key used for col lookup
template<typename ColKeyType>
using KeyedRowVector4d = KeyedRowVector<double, ColKeyType, 4>;

} // namespace NAV

#pragma GCC diagnostic pop

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for Frequency
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
struct fmt::formatter<NAV::KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format KeyedMatrix structs
    /// @param[in] mat Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    auto format(const NAV::KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>& mat, format_context& ctx)
    {
        std::string result;
        auto rows = static_cast<size_t>(mat.rows());
        auto cols = static_cast<size_t>(mat.cols());

        if (rows > 0 && cols > 0)
        {
            std::vector<std::string> rowKeysStr;
            std::vector<size_t> rowKeysLength;
            rowKeysStr.reserve(rows);
            rowKeysLength.reserve(rows);
            size_t rowKeysColSpace = 0;
            for (const auto& rowKey : mat.rowKeys())
            {
                rowKeysStr.push_back(fmt::format("{}", rowKey));
                auto rowKeyLength = rowKeysStr.back().length();
                rowKeysColSpace = std::max(rowKeysColSpace, rowKeyLength);
                rowKeysLength.push_back(rowKeyLength);
            }

            constexpr size_t colMinLength = 9UL;

            std::vector<std::string> colKeysStr;
            std::vector<size_t> colKeysLength;
            size_t rowLineLength = rowKeysColSpace + 1; // '\n' at the end of line
            colKeysStr.reserve(cols);
            colKeysLength.reserve(cols);
            for (const auto& colKey : mat.colKeys())
            {
                colKeysStr.push_back(fmt::format("{}", colKey));
                auto colKeyLength = colKeysStr.back().length();
                colKeysLength.push_back(colKeyLength);
                rowLineLength += 2 + std::max(colKeysStr.back().length(), colMinLength); // 2 spaces before each column
            }

            result.reserve((rows + 1) * rowLineLength);
            // ---------------------------------------- Column keys ------------------------------------------
            result += " " + std::string(rowKeysColSpace, ' ');
            for (size_t c = 0; c < cols; c++)
            {
                result += " ";
                if (colMinLength > colKeysLength.at(c))
                {
                    result += std::string(colMinLength - colKeysLength.at(c), ' '); // Spaces in front of column name (if too short)
                }
                result += colKeysStr.at(c);
            }
            result += '\n';
            // ------------------------------------------- Rows ----------------------------------------------
            for (size_t r = 0; r < rows; r++)
            {
                if (rowKeysColSpace > rowKeysLength.at(r))
                {
                    result += std::string(rowKeysColSpace - rowKeysLength.at(r), ' '); // Spaces in front of row name (if too short)
                }
                result += rowKeysStr.at(r) + " ";
                for (size_t c = 0; c < cols; c++)
                {
                    auto colLength = std::max(colKeysStr.at(c).length(), colMinLength);

                    std::string tmp = fmt::format(" {:> {}.{}g}", mat(NAV::all, NAV::all)(static_cast<int>(r), static_cast<int>(c)), colLength, colLength - 2);
                    if (tmp.length() > colLength)
                    {
                        tmp = fmt::format(" {:> {}.{}g}", mat(NAV::all, NAV::all)(static_cast<int>(r), static_cast<int>(c)), colLength, colLength - 6);
                    }
                    result += tmp;
                }
                if (r != rows - 1) { result += '\n'; }
            }
        }

        return fmt::format_to(ctx.out(), "{}", result);
    }
};

/// @brief Formatter for Frequency
template<typename Scalar, typename RowKeyType, int Rows>
struct fmt::formatter<NAV::KeyedVector<Scalar, RowKeyType, Rows>>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format KeyedVector structs
    /// @param[in] vec Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    auto format(const NAV::KeyedVector<Scalar, RowKeyType, Rows>& vec, format_context& ctx)
    {
        std::string result;
        auto rows = static_cast<size_t>(vec.rows());

        if (rows > 0)
        {
            std::vector<std::string> rowKeysStr;
            std::vector<size_t> rowKeysLength;
            rowKeysStr.reserve(rows);
            rowKeysLength.reserve(rows);
            size_t rowKeysColSpace = 0;
            for (const auto& rowKey : vec.rowKeys())
            {
                rowKeysStr.push_back(fmt::format("{}", rowKey));
                auto rowKeyLength = rowKeysStr.back().length();
                rowKeysColSpace = std::max(rowKeysColSpace, rowKeyLength);
                rowKeysLength.push_back(rowKeyLength);
            }

            size_t colLength = 9UL;

            result.reserve(rows * (rowKeysColSpace + 2 + colLength));
            // ------------------------------------------- Rows ----------------------------------------------
            for (size_t r = 0; r < rows; r++)
            {
                if (rowKeysColSpace > rowKeysLength.at(r))
                {
                    result += std::string(rowKeysColSpace - rowKeysLength.at(r), ' '); // Spaces in front of row name (if too short)
                }
                result += rowKeysStr.at(r);

                std::string tmp = fmt::format("  {:> {}.{}g}", vec(NAV::all)(static_cast<int>(r)), colLength, colLength - 2);
                if (tmp.length() > colLength)
                {
                    tmp = fmt::format("  {:> {}.{}g}", vec(NAV::all)(static_cast<int>(r)), colLength, colLength - 6);
                }
                result += tmp;

                if (r != rows - 1) { result += '\n'; }
            }
        }

        return fmt::format_to(ctx.out(), "{}", result);
    }
};

/// @brief Formatter for Frequency
template<typename Scalar, typename ColKeyType, int Cols>
struct fmt::formatter<NAV::KeyedRowVector<Scalar, ColKeyType, Cols>>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format KeyedRowVector structs
    /// @param[in] vec Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    auto format(const NAV::KeyedRowVector<Scalar, ColKeyType, Cols>& vec, format_context& ctx)
    {
        std::string result;
        auto cols = static_cast<size_t>(vec.cols());

        if (cols > 0)
        {
            size_t colMinLength = 9UL;

            std::vector<std::string> colKeysStr;
            std::vector<size_t> colKeysLength;
            size_t rowLineLength = 1; // '\n' at the end of line
            colKeysStr.reserve(cols);
            colKeysLength.reserve(cols);
            for (const auto& colKey : vec.colKeys())
            {
                colKeysStr.push_back(fmt::format("{}", colKey));
                auto colKeyLength = colKeysStr.back().length();
                colKeysLength.push_back(colKeyLength);
                rowLineLength += 2 + std::max(colKeysStr.back().length(), colMinLength); // 2 spaces before each column
            }

            result.reserve(2 * rowLineLength);
            // ---------------------------------------- Column keys ------------------------------------------
            for (size_t c = 0; c < cols; c++)
            {
                if (c != 0) { result += " "; }
                if (colMinLength > colKeysLength.at(c))
                {
                    result += std::string(colMinLength - colKeysLength.at(c), ' '); // Spaces in front of column name (if too short)
                }
                result += colKeysStr.at(c);
            }
            result += '\n';
            // ------------------------------------------ Values ---------------------------------------------

            for (size_t c = 0; c < cols; c++)
            {
                auto colLength = std::max(colKeysStr.at(c).length(), colMinLength);
                if (c != 0) { result += " "; }
                std::string tmp = fmt::format("{:> {}.{}g}", vec(NAV::all)(static_cast<int>(c)), colLength, colLength - 2);
                if (tmp.length() > colLength)
                {
                    tmp = fmt::format("{:> {}.{}g}", vec(NAV::all)(static_cast<int>(c)), colLength, colLength - 6);
                }
                result += tmp;
            }
        }

        return fmt::format_to(ctx.out(), "{}", result);
    }
};

#endif