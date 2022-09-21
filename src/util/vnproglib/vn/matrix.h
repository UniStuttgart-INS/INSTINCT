// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file matrix.h
/// @brief Extract from the vnproglib
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_VECTORNAV_LIBRARY

// NOLINTBEGIN

    #include <cassert>
    #include <iostream>
    #include <limits>

    #include "vector.h"

namespace vn
{
namespace math
{

/// \brief Template for a matrix.
template<size_t m, size_t n = m, typename T = float>
struct mat
{
    /// \brief The matrix's elements.
    union
    {
        T e[m * n];
    };

    /// \brief Creates a new matrix with uninitialized elements.
    mat() = default;

    /// \brief Creates a new matrix with ints elements initialized to val.
    /// \param[in] val The initialization value.
    explicit mat(T val)
    {
        std::fill_n(e, m * n, val);
    }

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    T& operator()(size_t row, size_t col)
    {
        return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
    }

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    const T& operator()(size_t row, size_t col) const
    {
        assert(row < m && col < n);

        // return e[col * m + row];
        return e[col + row * m];
    }

    /// \brief The matrix's row dimension.
    /// \return The matrix's row dimension.
    [[nodiscard]] size_t dimRow() const { return m; }

    /// \brief The matrix's column dimension.
    /// \return The matrix's column dimension.
    [[nodiscard]] size_t dimCol() const { return n; }
};

/// \brief 2x2 matrix specialization.
template<typename T>
struct mat<2, 2, T>
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic" // error: ISO C++ prohibits anonymous structs
    union
    {
        struct
        {
            /// \brief Element 0,0.
            T e00;

            /// \brief Element 1,0.
            T e10;

            /// \brief Element 0,1.
            T e01;

            /// \brief Element 1,1.
            T e11;
        };

        /// \brief The matrix's elements.
        T e[2 * 2];
    };
    #pragma GCC diagnostic pop

    /// \brief Creates a new matrix with uninitialized elements.
    mat() = default;

    /// \brief Creates a new matrix with its elements initialized to val.
    /// \param[in] val The initialization value.
    explicit mat(T val) : e00(val), e10(val), e01(val), e11(val) {}

    /// \brief Creates a new matrix with its components initialized to the provided values.
    /// \param[in] e00v Element 0,0 value.
    /// \param[in] e01v Element 0,1 value.
    /// \param[in] e10v Element 1,0 value.
    /// \param[in] e11v Element 1,1 value.
    mat(T e00v, T e01v,
        T e10v, T e11v) : e00(e00v), e10(e10v), e01(e01v), e11(e11v) {}

    /// \brief Constructs a matrix from 4 column vectors.
    /// \param[in] col0 The 0 column vector.
    /// \param[in] col1 The 1 column vector.
    mat(vec<2, T> col0, vec<2, T> col1) : e00(col0.x), e10(col1.x), e01(col0.y), e11(col1.y) {}

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    T& operator()(size_t row, size_t col)
    {
        return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
    }

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    const T& operator()(size_t row, size_t col) const
    {
        assert(row < 2 && col < 2);

        return e[col * 2 + row];
    }

    /// \brief The matrix's row dimension.
    /// \return The matrix's row dimension.
    [[nodiscard]] size_t dimRow() const { return 2; }

    /// \brief The matrix's column dimension.
    /// \return The matrix's column dimension.
    [[nodiscard]] size_t dimCol() const { return 2; }
};

/// \brief 3x3 matrix specialization.
/// matrix is column ordered in memory
template<typename T>
struct mat<3, 3, T>
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic" // error: ISO C++ prohibits anonymous structs
    union
    {
        struct
        {
            /// \brief Element 0,0.
            T e00;

            /// \brief Element 1,0.
            T e10;

            /// \brief Element 2,0.
            T e20;

            /// \brief Element 0,1.
            T e01;

            /// \brief Element 1,1.
            T e11;

            /// \brief Element 2,1.
            T e21;

            /// \brief Element 0,2.
            T e02;

            /// \brief Element 1,2.
            T e12;

            /// \brief Element 2,2.
            T e22;
        };

        /// \brief The matrix's elements.
        T e[3 * 3];
    };
    #pragma GCC diagnostic pop

    /// \brief Creates a new matrix with uninitialized elements.
    mat() = default;

    /// \brief Creates a new matrix with its elements initialized to val.
    /// \param[in] val The initialization value.
    explicit mat(T val) : e00(val), e10(val), e20(val), e01(val), e11(val), e21(val), e02(val), e12(val), e22(val) {}

    /// \brief Creates a new matrix with its components intialized to the provided values.
    /// \param[in] e00v Element 0,0 value.
    /// \param[in] e01v Element 0,1 value.
    /// \param[in] e02v Element 0,2 value.
    /// \param[in] e10v Element 1,0 value.
    /// \param[in] e11v Element 1,1 value.
    /// \param[in] e12v Element 1,2 value.
    /// \param[in] e20v Element 2,0 value.
    /// \param[in] e21v Element 2,1 value.
    /// \param[in] e22v Element 2,2 value.
    mat(T e00v, T e01v, T e02v,
        T e10v, T e11v, T e12v,
        T e20v, T e21v, T e22v) : e00(e00v), e10(e10v), e20(e20v), e01(e01v), e11(e11v), e21(e21v), e02(e02v), e12(e12v), e22(e22v) {}

    /// \brief Constructs a matrix from 3 column vectors.  Vectors are stored in column order
    /// \param[in] col0 The 0 column vector.
    /// \param[in] col1 The 1 column vector.
    /// \param[in] col2 The 2 column vector.
    mat(vec<3, T> col0, vec<3, T> col1, vec<3, T> col2) : e00(col0.x), e10(col0.y), e20(col0.z), e01(col1.x), e11(col1.y), e21(col1.z), e02(col2.x), e12(col2.y), e22(col2.z) {}

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    T& operator()(size_t row, size_t col)
    {
        return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
    }

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    const T& operator()(size_t row, size_t col) const
    {
        assert(row < 3 && col < 3);

        return e[col * 3 + row];
    }

    /// \brief The matrix's row dimension.
    /// \return The matrix's row dimension.
    [[nodiscard]] size_t dimRow() const { return 3; }

    /// \brief The matrix's column dimension.
    /// \return The matrix's column dimension.
    [[nodiscard]] size_t dimCol() const { return 3; }
};

/// \brief 4x4 matrix specialization.
template<typename T>
struct mat<4, 4, T>
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic" // error: ISO C++ prohibits anonymous structs
    union
    {
        struct
        {
            /// \brief Element 0,0.
            T e00;

            /// \brief Element 1,0.
            T e10;

            /// \brief Element 2,0.
            T e20;

            /// \brief Element 3,0.
            T e30;

            /// \brief Element 0,1.
            T e01;

            /// \brief Element 1,1.
            T e11;

            /// \brief Element 2,1.
            T e21;

            /// \brief Element 3,1.
            T e31;

            /// \brief Element 0,2.
            T e02;

            /// \brief Element 1,2.
            T e12;

            /// \brief Element 2,2.
            T e22;

            /// \brief Element 3,2.
            T e32;

            /// \brief Element 0,3.
            T e03;

            /// \brief Element 1,3.
            T e13;

            /// \brief Element 2,3.
            T e23;

            /// \brief Element 3,3.
            T e33;
        };

        /// \brief The matrix's elements.
        T e[4 * 4];
    };
    #pragma GCC diagnostic pop

    /// \brief Creates a new matrix with uninitialized elements.
    mat() = default;

    /// \brief Creates a new matrix with its elements initialized to val.
    /// \param[in] val The initialization value.
    explicit mat(T val) : e00(val), e01(val), e02(val), e03(val), e10(val), e11(val), e12(val), e13(val), e20(val), e21(val), e22(val), e23(val), e30(val), e31(val), e32(val), e33(val) {}

    /// \brief Creates a new matrix with its components intialized to the provided values.
    /// \param[in] e00v Element 0,0 value.
    /// \param[in] e01v Element 0,1 value.
    /// \param[in] e02v Element 0,2 value.
    /// \param[in] e03v Element 0,3 value.
    /// \param[in] e10v Element 1,0 value.
    /// \param[in] e11v Element 1,1 value.
    /// \param[in] e12v Element 1,2 value.
    /// \param[in] e13v Element 1,3 value.
    /// \param[in] e20v Element 2,0 value.
    /// \param[in] e21v Element 2,1 value.
    /// \param[in] e22v Element 2,2 value.
    /// \param[in] e23v Element 2,3 value.
    /// \param[in] e30v Element 3,0 value.
    /// \param[in] e31v Element 3,1 value.
    /// \param[in] e32v Element 3,2 value.
    /// \param[in] e33v Element 3,3 value.
    mat(T e00v, T e01v, T e02v, T e03v,
        T e10v, T e11v, T e12v, T e13v,
        T e20v, T e21v, T e22v, T e23v,
        T e30v, T e31v, T e32v, T e33v) : e00(e00v), e01(e01v), e02(e02v), e03(e03v), e10(e10v), e11(e11v), e12(e12v), e13(e13v), e20(e20v), e21(e21v), e22(e22v), e23(e23v), e30(e30v), e31(e31v), e32(e32v), e33(e33v) {}

    /// \brief Constructs a matrix from 4 column vectors.
    /// \param[in] col0 The 0 column vector.
    /// \param[in] col1 The 1 column vector.
    /// \param[in] col2 The 2 column vector.
    /// \param[in] col3 The 3 column vector.
    mat(vec<4, T> col0, vec<4, T> col1, vec<4, T> col2, vec<4, T> col3) : e00(col0.x), e10(col1.x), e20(col2.x), e30(col3.x), e01(col0.y), e11(col1.y), e21(col2.y), e31(col3.y), e02(col0.z), e12(col1.z), e22(col2.z), e32(col3.z), e03(col0.w), e13(col1.w), e23(col2.w), e33(col3.w) {}

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    T& operator()(size_t row, size_t col)
    {
        return const_cast<T&>(static_cast<const mat&>(*this)(row, col));
    }

    /// \brief Indexing into the matrix's elements.
    /// \param[in] row The 0-based index row.
    /// \param[in] col The 0-based index column.
    /// \return The requested element.
    const T& operator()(size_t row, size_t col) const
    {
        assert(row < 4 && col < 4);

        return e[col * 4 + row];
    }

    /// \brief The matrix's row dimension.
    /// \return The matrix's row dimension.
    [[nodiscard]] size_t dimRow() const { return 4; }

    /// \brief The matrix's column dimension.
    /// \return The matrix's column dimension.
    [[nodiscard]] size_t dimCol() const { return 4; }
};

template<size_t m, size_t n, typename T, typename S>
bool operator==(const mat<m, n, T>& lhs, const mat<m, n, S>& rhs)
{
    constexpr T EPSILON = 10.0 * std::numeric_limits<T>::epsilon();

    for (size_t i = 0; i < m; i++)
    {
        for (size_t j = 0; j < n; j++)
        {
            if (std::abs(lhs.e[i * n + j] - rhs.e[i * n + j]) > EPSILON)
            {
                return false;
            }
        }
    }
    return true;
}

template<size_t m, size_t n, typename T, typename S>
bool operator!=(const mat<m, n, T>& lhs, const mat<m, n, S>& rhs)
{
    return !(lhs == rhs);
}

// Specific Typedefs //////////////////////////////////////////////////////////

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2> mat2;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3> mat3;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4> mat4;

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2> mat22;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3> mat33;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4> mat44;

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2, 2, float> mat2f;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3, 3, float> mat3f;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4, 4, float> mat4f;

/// \brief 2x2 matrix using <c>double</c> as its underlying data type.
typedef mat<2, 2, double> mat2d;

/// \brief 3x3 matrix using <c>double</c> as its underlying data type.
typedef mat<3, 3, double> mat3d;

/// \brief 4x4 matrix using <c>double</c> as its underlying data type.
typedef mat<4, 4, double> mat4d;

/// \brief 2x2 matrix using <c>long double</c> as its underlying data type.
typedef mat<2, 2, long double> mat2ld;

/// \brief 3x3 matrix using <c>long double</c> as its underlying data type.
typedef mat<3, 3, long double> mat3ld;

/// \brief 4x4 matrix using <c>long double</c> as its underlying data type.
typedef mat<4, 4, long double> mat4ld;

/// \brief 2x2 matrix using <c>float</c> as its underlying data type.
typedef mat<2, 2, float> mat22f;

/// \brief 3x3 matrix using <c>float</c> as its underlying data type.
typedef mat<3, 3, float> mat33f;

/// \brief 4x4 matrix using <c>float</c> as its underlying data type.
typedef mat<4, 3, float> mat44f;

/// \brief 2x2 matrix using <c>double</c> as its underlying data type.
typedef mat<2, 2, double> mat22d;

/// \brief 3x3 matrix using <c>double</c> as its underlying data type.
typedef mat<3, 3, double> mat33d;

/// \brief 4x4 matrix using <c>double</c> as its underlying data type.
typedef mat<4, 4, double> mat44d;

/// \brief 2x2 matrix using <c>long double</c> as its underlying data type.
typedef mat<2, 2, long double> mat22ld;

/// \brief 3x3 matrix using <c>long double</c> as its underlying data type.
typedef mat<3, 3, long double> mat33ld;

/// \brief 4x4 matrix using <c>long double</c> as its underlying data type.
typedef mat<4, 4, long double> mat44ld;

// Common functions for working with matrices.

/// \brief Provides a method to generate a representable string from a provided
/// matrix.
///
/// \param[in] m The matrix to convert to string.
/// \return The string representation.
template<size_t mDim, size_t nDim, typename T>
std::string str(mat<mDim, nDim, T> m)
{
    std::stringstream ss;
    ss << "[";

    for (size_t row_index = 0; row_index < m.dimRow(); row_index++)
    {
        ss << "(";

        for (size_t col_index = 0; col_index < m.dimCol(); col_index++)
        {
            ss << m(row_index, col_index);

            if (col_index + 1 < m.dimCol())
                ss << "; ";
        }

        ss << ")";
    }

    ss << "]";

    return ss.str();
}

/// \brief Overloads the ostream << operator for easy usage in displaying
/// matrices.
///
/// \param[in] out The ostream being output to.
/// \param[in] m The matrix to output to ostream.
/// \return Reference to the current ostream.
template<size_t mDim, size_t nDim, typename T>
std::ostream& operator<<(std::ostream& out, mat<mDim, nDim, T> m)
{
    out << str(m);
    return out;
}

} // namespace math
} // namespace vn

// NOLINTEND

#endif