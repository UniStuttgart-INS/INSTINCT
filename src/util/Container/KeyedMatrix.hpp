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
#include <type_traits>
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

/// @brief KeyedMatrix storage class
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, int Rows, int Cols>
class KeyedMatrixStorage
{
  public:
    /// @brief Default Constructor
    KeyedMatrixStorage() = default;
    /// @brief Destructor
    virtual ~KeyedMatrixStorage() = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixStorage(const KeyedMatrixStorage& other)
    {
        this->matrix = other.matrix;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixStorage& operator=(const KeyedMatrixStorage& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixStorage(KeyedMatrixStorage&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixStorage& operator=(KeyedMatrixStorage&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);

        return *this;
    }

  protected:
    Eigen::Matrix<Scalar, Rows, Cols> matrix; ///< Data storage of the type
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
    /// @brief Default Constructor
    KeyedMatrixRowsBase() = default;
    /// @brief Destructor
    ~KeyedMatrixRowsBase() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixRowsBase(const KeyedMatrixRowsBase& other)
    {
        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixRowsBase& operator=(const KeyedMatrixRowsBase& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixRowsBase(KeyedMatrixRowsBase&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixRowsBase& operator=(KeyedMatrixRowsBase&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);

        return *this;
    }

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
    std::unordered_map<RowKeyType, Eigen::Index> rowIndices;
    /// Row Keys
    std::vector<RowKeyType> rowKeysVector;
};

/// @brief Base class for Keyed matrices with multiple rows of static size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Rows, int Cols>
class KeyedMatrixRows : virtual public KeyedMatrixRowsBase<Scalar, RowKeyType, Rows, Cols>
{
  public:
    /// @brief Default Constructor
    KeyedMatrixRows() = default;
    /// @brief Destructor
    ~KeyedMatrixRows() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixRows(const KeyedMatrixRows& other)
    {
        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixRows& operator=(const KeyedMatrixRows& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixRows(KeyedMatrixRows&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixRows& operator=(KeyedMatrixRows&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);

        return *this;
    }
};

/// @brief Base class for Keyed matrices with multiple rows of dynamic size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam RowKeyType Type of the key used for row lookup
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename RowKeyType, int Cols>
class KeyedMatrixRows<Scalar, RowKeyType, Eigen::Dynamic, Cols>
    : virtual public KeyedMatrixRowsBase<Scalar, RowKeyType, Eigen::Dynamic, Cols>
{
  public:
    /// @brief Default Constructor
    KeyedMatrixRows() = default;
    /// @brief Destructor
    ~KeyedMatrixRows() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixRows(const KeyedMatrixRows& other)
    {
        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixRows& operator=(const KeyedMatrixRows& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixRows(KeyedMatrixRows&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixRows& operator=(KeyedMatrixRows&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);

        return *this;
    }

    /// @brief Adds a new row to the matrix
    /// @param rowKey Row key
    void addRow(const RowKeyType& rowKey) { addRows({ rowKey }); }

    /// @brief Adds new rows to the matrix
    /// @param rowKeys Row keys
    void addRows(const std::vector<RowKeyType>& rowKeys)
    {
        INS_ASSERT_USER_ERROR(!this->hasAnyRows(rowKeys), "You cannot add a row key which is already in the matrix.");
        std::unordered_set<RowKeyType> rowSet = { rowKeys.begin(), rowKeys.end() };
        INS_ASSERT_USER_ERROR(rowSet.size() == rowKeys.size(), "Each row key must be unique");

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
    /// @brief Default Constructor
    KeyedMatrixColsBase() = default;
    /// @brief Destructor
    ~KeyedMatrixColsBase() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixColsBase(const KeyedMatrixColsBase& other)
    {
        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixColsBase& operator=(const KeyedMatrixColsBase& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixColsBase(KeyedMatrixColsBase&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixColsBase& operator=(KeyedMatrixColsBase&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);

        return *this;
    }

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
    std::unordered_map<ColKeyType, Eigen::Index> colIndices;
    /// Col Keys
    std::vector<ColKeyType> colKeysVector;
};

/// @brief Base class for Keyed matrices with multiple columns of static size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
/// @tparam Cols Number of columns, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Rows, int Cols>
class KeyedMatrixCols : virtual public KeyedMatrixColsBase<Scalar, ColKeyType, Rows, Cols>
{
  public:
    /// @brief Default Constructor
    KeyedMatrixCols() = default;
    /// @brief Destructor
    ~KeyedMatrixCols() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixCols(const KeyedMatrixCols& other)
    {
        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixCols& operator=(const KeyedMatrixCols& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixCols(KeyedMatrixCols&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixCols& operator=(KeyedMatrixCols&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);

        return *this;
    }
};

/// @brief Base class for Keyed matrices with multiple columns of dynamic size
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam ColKeyType Type of the key used for col lookup
/// @tparam Rows Number of rows, or \b Dynamic
template<typename Scalar, typename ColKeyType, int Rows>
class KeyedMatrixCols<Scalar, ColKeyType, Rows, Eigen::Dynamic>
    : virtual public KeyedMatrixColsBase<Scalar, ColKeyType, Rows, Eigen::Dynamic>
{
  public:
    /// @brief Default Constructor
    KeyedMatrixCols() = default;
    /// @brief Destructor
    ~KeyedMatrixCols() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixCols(const KeyedMatrixCols& other)
    {
        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixCols& operator=(const KeyedMatrixCols& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;

        return *this;
    }
    /// @brief Move constructor
    /// @param other The other object
    KeyedMatrixCols(KeyedMatrixCols&& other) noexcept
    {
        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixCols& operator=(KeyedMatrixCols&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);

        return *this;
    }

    /// @brief Adds a new col to the matrix
    /// @param colKey Col key
    void addCol(const ColKeyType& colKey) { addCols({ colKey }); }

    /// @brief Adds new cols to the matrix
    /// @param colKeys Col keys
    void addCols(const std::vector<ColKeyType>& colKeys)
    {
        INS_ASSERT_USER_ERROR(!this->hasAnyCols(colKeys), "You cannot add a col key which is already in the matrix.");
        std::unordered_set<ColKeyType> colSet = { colKeys.begin(), colKeys.end() };
        INS_ASSERT_USER_ERROR(colSet.size() == colKeys.size(), "Each col key must be unique");

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
        std::unordered_set<RowKeyType> rowSet = { rowKeys.begin(), rowKeys.end() };
        INS_ASSERT_USER_ERROR(rowSet.size() == rowKeys.size(), "Each row key must be unique");

        INS_ASSERT_USER_ERROR(vector.cols() == 1, "Only vectors with 1 column are allowed.");
        INS_ASSERT_USER_ERROR(vector.rows() == static_cast<Eigen::Index>(rowKeys.size()), "Number of vector rows doesn't correspond to the amount of row keys");

        for (size_t i = 0; i < rowKeys.size(); i++) { this->rowIndices.insert({ rowKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->matrix = vector;
        this->rowKeysVector = rowKeys;
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
        std::vector<Eigen::Index> rowIdx;
        rowIdx.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIdx.push_back(this->rowIndices.at(rowKey)); }

        return this->matrix(rowIdx, 0);
    }
    /// @brief Gets the values for the row keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys)
    {
        std::vector<Eigen::Index> rowIdx;
        rowIdx.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIdx.push_back(this->rowIndices.at(rowKey)); }

        return this->matrix(rowIdx, 0);
    }

    /// @brief Requests the full vector
    const Eigen::Matrix<Scalar, Rows, 1>& operator()(all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full vector
    Eigen::Matrix<Scalar, Rows, 1>& operator()(all_t /* all */) { return this->matrix; }
    /// @brief Conversion into Eigen::Vector
    explicit operator Eigen::Vector<Scalar, Rows>() { return this->matrix; }
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
        std::unordered_set<ColKeyType> colSet = { colKeys.begin(), colKeys.end() };
        INS_ASSERT_USER_ERROR(colSet.size() == colKeys.size(), "Each col key must be unique");

        INS_ASSERT_USER_ERROR(vector.rows() == 1, "Only vectors with 1 row are allowed.");
        INS_ASSERT_USER_ERROR(vector.cols() == static_cast<Eigen::Index>(colKeys.size()), "Number of vector cols doesn't correspond to the amount of col keys");

        for (size_t i = 0; i < colKeys.size(); i++) { this->colIndices.insert({ colKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->matrix = vector;
        this->colKeysVector = colKeys;
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
        std::vector<Eigen::Index> colIdx;
        colIdx.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIdx.push_back(this->colIndices.at(colKey)); }

        return this->matrix(0, colIdx);
    }
    /// @brief Gets the values for the col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the col keys
    decltype(auto) operator()(const std::vector<ColKeyType>& colKeys)
    {
        std::vector<Eigen::Index> colIdx;
        colIdx.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIdx.push_back(this->colIndices.at(colKey)); }

        return this->matrix(0, colIdx);
    }

    /// @brief Requests the full vector
    const Eigen::Matrix<Scalar, 1, Cols>& operator()(all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full vector
    Eigen::Matrix<Scalar, 1, Cols>& operator()(all_t /* all */) { return this->matrix; }
    /// @brief Conversion into Eigen::RowVector
    explicit operator Eigen::RowVector<Scalar, Cols>() { return this->matrix; }
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
        std::unordered_set<RowKeyType> rowSet = { rowKeys.begin(), rowKeys.end() };
        INS_ASSERT_USER_ERROR(rowSet.size() == rowKeys.size(), "Each row key must be unique");
        std::unordered_set<ColKeyType> colSet = { colKeys.begin(), colKeys.end() };
        INS_ASSERT_USER_ERROR(colSet.size() == colKeys.size(), "Each col key must be unique");

        INS_ASSERT_USER_ERROR(matrix.rows() == static_cast<Eigen::Index>(rowKeys.size()), "Number of matrix rows doesn't correspond to the amount of row keys");
        INS_ASSERT_USER_ERROR(matrix.cols() == static_cast<Eigen::Index>(colKeys.size()), "Number of matrix cols doesn't correspond to the amount of col keys");

        INS_ASSERT_USER_ERROR(Rows == Eigen::Dynamic || Rows == static_cast<int>(rowKeys.size()), "Number of matrix rows doesn't correspond to the static amount of row keys");
        INS_ASSERT_USER_ERROR(Cols == Eigen::Dynamic || Cols == static_cast<int>(colKeys.size()), "Number of matrix cols doesn't correspond to the static amount of col keys");

        for (size_t i = 0; i < rowKeys.size(); i++) { this->rowIndices.insert({ rowKeys.at(i), static_cast<Eigen::Index>(i) }); }
        for (size_t i = 0; i < colKeys.size(); i++) { this->colIndices.insert({ colKeys.at(i), static_cast<Eigen::Index>(i) }); }

        this->matrix = matrix;
        this->rowKeysVector = rowKeys;
        this->colKeysVector = colKeys;
    }

    /// @brief Destructor
    ~KeyedMatrixBase() override = default;
    /// @brief Copy constructor
    /// @param other The other object
    KeyedMatrixBase(const KeyedMatrixBase& other)
        : KeyedMatrixRows<Scalar, RowKeyType, Rows, Cols>(),
          KeyedMatrixCols<Scalar, ColKeyType, Rows, Cols>()
    {
        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;
    }
    /// @brief Copy assignment operator
    /// @param other The other object
    KeyedMatrixBase& operator=(const KeyedMatrixBase& other)
    {
        // Guard self assignment
        if (this == &other) { return *this; }

        this->matrix = other.matrix;
        this->rowIndices = other.rowIndices;
        this->rowKeysVector = other.rowKeysVector;
        this->colIndices = other.colIndices;
        this->colKeysVector = other.colKeysVector;

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
    }
    /// @brief Move assignment operator
    /// @param other The other object
    KeyedMatrixBase& operator=(KeyedMatrixBase&& other) noexcept
    {
        if (this == &other) { return *this; }

        this->matrix = std::move(other.matrix);
        this->rowIndices = std::move(other.rowIndices);
        this->rowKeysVector = std::move(other.rowKeysVector);
        this->colIndices = std::move(other.colIndices);
        this->colKeysVector = std::move(other.colKeysVector);

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
        return this->matrix(this->rowIndices.at(rowKey), this->colIndices.at(colKey));
    }

    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys) const
    {
        std::vector<Eigen::Index> rowIdx;
        rowIdx.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIdx.push_back(this->rowIndices.at(rowKey)); }

        std::vector<Eigen::Index> colIdx;
        colIdx.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIdx.push_back(this->colIndices.at(colKey)); }

        return this->matrix(rowIdx, colIdx);
    }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, const std::vector<ColKeyType>& colKeys)
    {
        std::vector<Eigen::Index> rowIdx;
        rowIdx.reserve(rowKeys.size());
        for (const auto& rowKey : rowKeys) { rowIdx.push_back(this->rowIndices.at(rowKey)); }

        std::vector<Eigen::Index> colIdx;
        colIdx.reserve(colKeys.size());
        for (const auto& colKey : colKeys) { colIdx.push_back(this->colIndices.at(colKey)); }

        return this->matrix(rowIdx, colIdx);
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
    decltype(auto) operator()(const RowKeyType& rowKey, all_t /* all */) const { return (*this)(std::vector{ rowKey }, this->colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKey Row Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const RowKeyType& rowKey, all_t /* all */) { return (*this)(std::vector{ rowKey }, this->colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const ColKeyType& colKey) const { return *this(this->rowKeys(), std::vector{ colKey }); }
    /// @brief Gets the values for the row and col keys
    /// @param colKey Col Key
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const ColKeyType& colKey) { return (*this)(this->rowKeys(), std::vector{ colKey }); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, all_t /* all */) const { return (*this)(rowKeys, this->colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param rowKeys Row Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(const std::vector<RowKeyType>& rowKeys, all_t /* all */) { return (*this)(rowKeys, this->colKeys()); }
    /// @brief Gets the values for the row and col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const std::vector<ColKeyType>& colKeys) const { return (*this)(this->rowKeys(), colKeys); }
    /// @brief Gets the values for the row and col keys
    /// @param colKeys Col Keys
    /// @return View into the matrix for the row and col keys
    decltype(auto) operator()(all_t /* all */, const std::vector<ColKeyType>& colKeys) { return (*this)(this->rowKeys(), colKeys); }

    /// @brief Requests the full matrix
    const Eigen::Matrix<Scalar, Rows, Cols>& operator()(all_t /* all */, all_t /* all */) const { return this->matrix; }
    /// @brief Requests the full matrix
    Eigen::Matrix<Scalar, Rows, Cols>& operator()(all_t /* all */, all_t /* all */) { return this->matrix; }
    /// @brief Conversion into Eigen::Matrix
    explicit operator Eigen::Matrix<Scalar, Rows, Cols>() { return this->matrix; }
};

} // namespace internal

/// @brief Used to request all rows or columns in KeyedMatrices
static const internal::all_t all;

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

    // ------------------------------- Operators from a Dynamic KeyedVector ----------------------------------

    /// @brief Copy constructor
    /// @param other The other object
    KeyedVector(const KeyedVector<Scalar, RowKeyType, Eigen::Dynamic>& other) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Rows>(other(all), other.rowKeys())
    {}
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

    // TODO
    template<int oRows>
    KeyedVector(const KeyedVector<Scalar, RowKeyType, oRows>& other) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : internal::KeyedVectorBase<Scalar, RowKeyType, Eigen::Dynamic>(other(all), other.rowKeys())
    {
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

    // TODO
    KeyedRowVector(const KeyedRowVector<Scalar, ColKeyType, Eigen::Dynamic>& other) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Cols>(other(all), other.colKeys())
    {}
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

    // TODO
    template<int oCols>
    KeyedRowVector(const KeyedRowVector<Scalar, ColKeyType, oCols>& other) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : internal::KeyedRowVectorBase<Scalar, ColKeyType, Eigen::Dynamic>(other(all), other.colKeys())
    {
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

    // TODO
    KeyedMatrix(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>& other) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Rows, Cols>(other(all, all), other.rowKeys(), other.colKeys())
    {}
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
    //                                 Operators from a Static KeyedMatrix
    // #######################################################################################################

    /// @brief Copy constructor
    /// @param other The other object
    template<int oRows, int oCols>
    KeyedMatrix(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, oRows, oCols>& other) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : internal::KeyedMatrixBase<Scalar, RowKeyType, ColKeyType, Eigen::Dynamic, Eigen::Dynamic>(other(all, all), other.rowKeys(), other.colKeys())
    {}

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
        std::unordered_set<RowKeyType> rowSet = { rowKeys.begin(), rowKeys.end() };
        INS_ASSERT_USER_ERROR(rowSet.size() == rowKeys.size(), "Each row key must be unique");
        std::unordered_set<ColKeyType> colSet = { colKeys.begin(), colKeys.end() };
        INS_ASSERT_USER_ERROR(colSet.size() == colKeys.size(), "Each col key must be unique");

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
