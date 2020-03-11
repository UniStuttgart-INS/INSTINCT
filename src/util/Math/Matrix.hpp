/**
 * @file Matrix.hpp
 * @brief [Deprecated] Matrix Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 * @attention [Deprecated] Eigen library will be used in the future
 */

#pragma once

#include <cassert>
#include <iostream>

#include "Vector.hpp"

namespace NAV
{
// Forward declaration of class
template<size_t m, size_t n = m, typename T = double>
class Matrix;

/// @brief Template for the matrix's components
template<size_t m, size_t n = m, typename T = double>
struct MatrixComponents
{
    /// @brief The matrix's elements
    std::array<std::array<T, n>, m> e;

    /// @brief Creates a new matrixComponent
    MatrixComponents() {}
};

/// @brief Matrix's components for 2x2 specialization
template<typename T>
struct MatrixComponents<2, 2, T>
{
    union
    {
        struct
        {
            /// @brief Element 0,0
            T e00;

            /// @brief Element 0,1
            T e01;

            /// @brief Element 1,0
            T e10;

            /// @brief Element 1,1
            T e11;
        };

        /// @brief The matrix's elements
        std::array<std::array<T, 2>, 2> e;
    };

    /// @brief Creates a new matrixComponent
    MatrixComponents() {}
};

/// @brief Matrix's components for 3x3 specialization
template<typename T>
struct MatrixComponents<3, 3, T>
{
    union
    {
        struct
        {
            /// @brief Element 0,0
            T e00;

            /// @brief Element 0,1
            T e01;

            /// @brief Element 0,2
            T e02;

            /// @brief Element 1,0
            T e10;

            /// @brief Element 1,1
            T e11;

            /// @brief Element 1,2
            T e12;

            /// @brief Element 2,0
            T e20;

            /// @brief Element 2,1
            T e21;

            /// @brief Element 2,2
            T e22;
        };

        /// @brief The matrix's elements
        std::array<std::array<T, 3>, 3> e;
    };

    /// @brief Creates a new matrixComponent
    MatrixComponents() {}
};

/// @brief Matrix's components for 4x4 specialization
template<typename T>
struct MatrixComponents<4, 4, T>
{
    union
    {
        struct
        {
            /// @brief Element 0,0
            T e00;

            /// @brief Element 0,1
            T e01;

            /// @brief Element 0,2
            T e02;

            /// @brief Element 0,3
            T e03;

            /// @brief Element 1,0
            T e10;

            /// @brief Element 1,1
            T e11;

            /// @brief Element 1,2
            T e12;

            /// @brief Element 1,3
            T e13;

            /// @brief Element 2,0
            T e20;

            /// @brief Element 2,1
            T e21;

            /// @brief Element 2,2
            T e22;

            /// @brief Element 2,3
            T e23;

            /// @brief Element 3,0
            T e30;

            /// @brief Element 3,1
            T e31;

            /// @brief Element 3,2
            T e32;

            /// @brief Element 3,3
            T e33;
        };

        /// @brief The matrix's elements
        std::array<std::array<T, 4>, 4> e;
    };

    /// @brief Creates a new matrixComponent
    MatrixComponents() {}
};

/// @brief Template for the matrix's base
template<size_t m, size_t n = m, typename T = double>
class MatrixBase : public MatrixComponents<m, n, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new matrix with uninitialized elements
    MatrixBase() {}

    /// @brief Creates a new matrix with ints elements initialized to val
    /// @param[in] val: The initialization value
    explicit MatrixBase(T val)
    {
        std::fill_n(&MatrixComponents<m, n, T>::e[0][0], m * n, val);
    }

    MatrixBase(Vector<m, T> v)
    {
        for (size_t i = 0; i < m; i++)
            MatrixComponents<m, n, T>::e[i][0] = v[i];
    }

  public: // ############### Helper Methods ###############
    /// @brief Matrix with all of its elements set to 0
    /// @return The 0 matrix
    static MatrixBase zero()
    {
        return MatrixBase<m, n, T>(0);
    }

    /// @brief Matrix with all of its elements set to 1
    /// @return The 1 matrix
    static MatrixBase one()
    {
        return MatrixBase<m, n, T>(1);
    }

    /// @brief Identity matrix with its diagonal elements set to 1
    /// @return The identity matrix
    static MatrixBase<m, m, T> identity()
    {
        assert(m == n);

        MatrixBase<m, m, T> nm(0);

        for (size_t i = 0; i < m; i++)
            nm(i, i) = 1;

        return nm;
    }

  public: // ############# Operator Overloads #############
    /// @brief Indexing into the matrix's elements
    /// @param[in] row: The 0-based index row
    /// @param[in] col: The 0-based index column
    /// @return The requested element
    T& operator()(size_t row, size_t col)
    {
        return const_cast<T&>(static_cast<const MatrixBase&>(*this)(row, col));
    }

    /// @brief Indexing into the matrix's elements
    /// @param[in] row: The 0-based index row
    /// @param[in] col: The 0-based index column
    /// @return The requested element
    const T& operator()(size_t row, size_t col) const
    {
        assert(row < m && col < n);

        return MatrixComponents<m, n, T>::e[row][col];
    }

    /// @brief Converts a 1-dimensional matrix into a vector
    /// @return The converted vector
    operator Vector<m, T>() const
    {
        // Only 1 dimensional matrix convertible
        assert(n == 1);

        Vector<m, T> return_vec;
        for (size_t i = 0; i < m; i++)
            return_vec[i] = MatrixComponents<m, n, T>::e[i][0];

        return return_vec;
    }

    /// @brief Negates the matrix
    /// @return The negated matrix
    MatrixBase<m, n, T> operator-() const
    {
        return neg();
    }

    /// @brief Multiplies the matrix by a scalar
    /// @param[in] rhs: The scalar
    /// @return The multiplied matrix
    template<typename S,
             typename = typename std::enable_if<std::is_arithmetic<S>::value, S>::type>
    MatrixBase<m, n, T>& operator*=(const S& rhs)
    {
        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                MatrixComponents<m, n, T>::e[i][j] *= rhs;

        return *this;
    }

    /// @brief Multiplies the matrix by a matrix
    /// @param[in] rhs: The right-side matrix
    /// @return The multiplied matrix
    template<typename S>
    MatrixBase<m, n, T> operator*=(const Matrix<m, n, S>& rhs)
    {
        Matrix<m, n, T> return_mat(0);

        for (size_t mi = 0; mi < m; mi++)
            for (size_t si = 0; si < n; si++)
                for (size_t ni = 0; ni < n; ni++)
                    return_mat(mi, si) += MatrixComponents<m, n, T>::e[mi][ni] * rhs(ni, si);

        *this = return_mat;

        return *this;
    }

    /// @brief Divides the matrix by a scalar
    /// @param[in] rhs: The scalar
    /// @return The divided matrix
    template<typename S>
    MatrixBase<m, n, T>& operator/=(const S& rhs)
    {
        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                MatrixComponents<m, n, T>::e[i][j] /= rhs;

        return *this;
    }

    /// @brief Adds a matrix to this matrix
    /// @param[in] rhs: The right-side matrix
    /// @return The resulting matrix
    template<typename S>
    MatrixBase<m, n, T>& operator+=(const MatrixBase<m, n, S>& rhs)
    {
        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                MatrixComponents<m, n, T>::e[i][j] += rhs.e[i][j];

        return *this;
    }

    /// @brief Subtracts a matrix from this matrix
    /// @param[in] rhs: The right-side matrix
    /// @return The resulting matrix
    template<typename S>
    MatrixBase<m, n, T>& operator-=(const MatrixBase<m, n, S>& rhs)
    {
        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                MatrixComponents<m, n, T>::e[i][j] -= rhs.e[i][j];

        return *this;
    }

    /// @brief Compares equality from a matrix to this matrix
    /// @param[in] rhs: The right-side matrix
    /// @return True if components equal, else false
    bool operator==(const MatrixBase<m, n, T>& rhs) const
    {
        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                if (MatrixComponents<m, n, T>::e[i][j] != rhs.e[i][j])
                    return false;

        return true;
    }

    /// @brief Compares inequality from a matrix to this matrix
    /// @param[in] rhs: The right-side matrix
    /// @return True if components inequal, else false
    bool operator!=(const MatrixBase<m, n, T>& rhs) const
    {
        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                if (MatrixComponents<m, n, T>::e[i][j] != rhs.e[i][j])
                    return true;

        return false;
    }

  public: // ############### Public Methods ###############
    /// @brief The matrix's row dimension
    /// @return The matrix's row dimension
    size_t dimRow() const
    {
        return m;
    }

    /// @brief The matrix's column dimension
    /// @return The matrix's column dimension
    size_t dimCol() const
    {
        return n;
    }

    /// @brief Negates the matrix
    /// @return The negated matrix
    MatrixBase neg() const
    {
        MatrixBase nm;

        for (size_t i = 0; i < m; i++)
            for (size_t j = 0; j < n; j++)
                nm.e[i][j] = -MatrixComponents<m, n, T>::e[i][j];

        return nm;
    }

    /// @brief Transposes the matrix
    /// @return The computed transpose
    MatrixBase<n, m, T> transpose() const
    {
        MatrixBase<n, m, T> nm;

        for (size_t row = 0; row < m; row++)
            for (size_t col = 0; col < n; col++)
                nm.e[col][row] = MatrixComponents<m, n, T>::e[row][col];

        return nm;
    }
};

/// @brief Template for a matrix
template<size_t m, size_t n, typename T>
class Matrix : public MatrixBase<m, n, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new matrix with uninitialized components
    Matrix() {}

    /// @brief Creates new matrix with components initialized to val
    /// @param[in] val: The initialization value
    explicit Matrix(T val)
        : MatrixBase<m, n, T>(val) {}

    /// @brief Creates a new matrix from a MatrixBase
    Matrix(MatrixBase<m, n, T> mb)
        : MatrixBase<m, n, T>(mb) {}
};

/// @brief 2x2 matrix specialization
template<typename T>
class Matrix<2, 2, T> : public MatrixBase<2, 2, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new matrix with uninitialized components
    Matrix() {}

    /// @brief Creates new matrix with components initialized to val
    /// @param[in] val: The initialization value
    explicit Matrix(T val)
        : MatrixBase<2, 2, T>(val) {}

    /// @brief Creates a new matrix with its components initialized to the provided values
    /// @param[in] e00: Element 0,0 value
    /// @param[in] e01: Element 0,1 value
    /// @param[in] e10: Element 1,0 value
    /// @param[in] e11: Element 1,1 value
    Matrix(T e00, T e01, T e10, T e11)
    {
        MatrixComponents<2, 2, T>::e00 = e00;
        MatrixComponents<2, 2, T>::e01 = e01;
        MatrixComponents<2, 2, T>::e10 = e10;
        MatrixComponents<2, 2, T>::e11 = e11;
    }

    /// @brief Constructs a matrix from 2 column vectors
    /// @param[in] col0: The 0 column vector
    /// @param[in] col1: The 1 column vector
    Matrix(Vector<2, T> col0, Vector<2, T> col1)
    {
        MatrixComponents<2, 2, T>::e00 = col0.x;
        MatrixComponents<2, 2, T>::e01 = col1.x;
        MatrixComponents<2, 2, T>::e10 = col0.y;
        MatrixComponents<2, 2, T>::e11 = col1.y;
    }

    /// @brief Creates a new matrix from a MatrixBase
    Matrix(MatrixBase<2, 2, T> mb)
        : MatrixBase<2, 2, T>(mb) {}

  public: // ############### Helper Methods ###############
    /// @brief Rotational Matrix
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation matrix
    static Matrix<2, 2, T> rot(T const& angle)
    {
        return Matrix<2, 2, T>(cos(angle), -sin(angle),
                               sin(angle), cos(angle));
    }

    /// @brief Rotational Matrix
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation matrix
    static Matrix<2, 2, T> rot_deg(T const& angle)
    {
        return rot(angle * M_PI / 180);
    }

  public: // ############### Public Methods ###############
    /// @brief The matrix's determinant
    /// @return The matrix's determinant
    T determinant() const
    {
        return MatrixComponents<2, 2, T>::e00 * MatrixComponents<2, 2, T>::e11
               - MatrixComponents<2, 2, T>::e10 * MatrixComponents<2, 2, T>::e01;
    }
};

/// @brief 3x3 matrix specialization
template<typename T>
class Matrix<3, 3, T> : public MatrixBase<3, 3, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new matrix with uninitialized components
    Matrix() {}

    /// @brief Creates new matrix with components initialized to val
    /// @param[in] val: The initialization value
    explicit Matrix(T val)
        : MatrixBase<3, 3, T>(val) {}

    /// @brief Creates a new matrix with its components initialized to the provided values
    /// @param[in] e00: Element 0,0 value
    /// @param[in] e01: Element 0,1 value
    /// @param[in] e02: Element 0,2 value
    /// @param[in] e10: Element 1,0 value
    /// @param[in] e11: Element 1,1 value
    /// @param[in] e12: Element 1,2 value
    /// @param[in] e20: Element 2,0 value
    /// @param[in] e21: Element 2,1 value
    /// @param[in] e22: Element 2,2 value
    Matrix(T e00, T e01, T e02,
           T e10, T e11, T e12,
           T e20, T e21, T e22)
    {
        MatrixComponents<3, 3, T>::e00 = e00;
        MatrixComponents<3, 3, T>::e01 = e01;
        MatrixComponents<3, 3, T>::e02 = e02;
        MatrixComponents<3, 3, T>::e10 = e10;
        MatrixComponents<3, 3, T>::e11 = e11;
        MatrixComponents<3, 3, T>::e12 = e12;
        MatrixComponents<3, 3, T>::e20 = e20;
        MatrixComponents<3, 3, T>::e21 = e21;
        MatrixComponents<3, 3, T>::e22 = e22;
    }

    /// @brief Constructs a matrix from 3 column vectors
    /// @param[in] col0: The 0 column vector
    /// @param[in] col1: The 1 column vector
    /// @param[in] col2: The 2 column vector
    Matrix(Vector<3, T> col0, Vector<3, T> col1, Vector<3, T> col2)
    {
        MatrixComponents<3, 3, T>::e00 = col0.x;
        MatrixComponents<3, 3, T>::e01 = col1.x;
        MatrixComponents<3, 3, T>::e02 = col2.x;
        MatrixComponents<3, 3, T>::e10 = col0.y;
        MatrixComponents<3, 3, T>::e11 = col1.y;
        MatrixComponents<3, 3, T>::e12 = col2.y;
        MatrixComponents<3, 3, T>::e20 = col0.z;
        MatrixComponents<3, 3, T>::e21 = col1.z;
        MatrixComponents<3, 3, T>::e22 = col2.z;
    }

    /// @brief Creates a new matrix from a MatrixBase
    Matrix(MatrixBase<3, 3, T> mb)
        : MatrixBase<3, 3, T>(mb) {}

  public: // ############### Helper Methods ###############
    /// @brief Rotational Matrix around the x axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotX(T const& angle)
    {
        return Matrix<3, 3, T>(1, 0, 0,
                               0, cos(angle), -sin(angle),
                               0, sin(angle), cos(angle));
    }

    /// @brief Rotational Matrix around the x axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotX_deg(T const& angle)
    {
        return rotX(angle * M_PI / 180);
    }

    /// @brief Rotational Matrix around the y axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotY(T const& angle)
    {
        return Matrix<3, 3, T>(cos(angle), 0, sin(angle),
                               0, 1, 0,
                               -sin(angle), 0, cos(angle));
    }

    /// @brief Rotational Matrix around the y axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotY_deg(T const& angle)
    {
        return rotY(angle * M_PI / 180);
    }

    /// @brief Rotational Matrix around the z axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotZ(T const& angle)
    {
        return Matrix<3, 3, T>(cos(angle), -sin(angle), 0,
                               sin(angle), cos(angle), 0,
                               0, 0, 1);
    }

    /// @brief Rotational Matrix around the z axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotZ_deg(T const& angle)
    {
        return rotZ(angle * M_PI / 180);
    }

    /// @brief Rotational Matrix around the x, then y, then z axis
    /// @param[in] angle: The rotation angle around the x axis in radians
    /// @param[in] angle: The rotation angle around the y axis in radians
    /// @param[in] angle: The rotation angle around the z axis in radians
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotXYZ(T const& alpha, T const& beta, T const& gamma)
    {
        return Matrix<3, 3, T>(cos(beta) * cos(gamma), -cos(beta) * sin(gamma), sin(beta),
                               sin(gamma) * cos(alpha) + sin(beta) * cos(gamma) * sin(alpha), cos(gamma) * cos(alpha) - sin(beta) * sin(gamma) * sin(alpha), -cos(beta) * sin(alpha),
                               sin(gamma) * sin(alpha) - sin(beta) * cos(gamma) * cos(alpha), cos(gamma) * sin(alpha) + sin(beta) * sin(gamma) * cos(alpha), cos(beta) * cos(alpha));
    }

    /// @brief Rotational Matrix around the x, then y, then z axis
    /// @param[in] angle: The rotation angle around the x axis in degree
    /// @param[in] angle: The rotation angle around the y axis in degree
    /// @param[in] angle: The rotation angle around the z axis in degree
    /// @return The rotation matrix
    static Matrix<3, 3, T> rotXYZ_deg(T const& alpha, T const& beta, T const& gamma)
    {
        return rotXYZ(alpha * M_PI / 180, beta * M_PI / 180, gamma * M_PI / 180);
    }

    /// @brief Rotational Matrix around the provided axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @param[in] axis: The vector of the axis to rotate around
    /// @return The rotation matrix
    static Matrix<3, 3, T> rot(T const& angle, Vector<3, T> const& axis)
    {
        Vector<3, T> f = axis.normalized();
        return Matrix<3, 3, T>(f(0) * f(0) * (1 - cos(angle)) + cos(angle), f(0) * f(1) * (1 - cos(angle)) - f(2) * sin(angle), f(0) * f(2) * (1 - cos(angle)) + f(1) * sin(angle),
                               f(0) * f(1) * (1 - cos(angle)) + f(2) * sin(angle), f(1) * f(1) * (1 - cos(angle)) + cos(angle), f(1) * f(2) * (1 - cos(angle)) - f(0) * sin(angle),
                               f(0) * f(2) * (1 - cos(angle)) - f(1) * sin(angle), f(1) * f(2) * (1 - cos(angle)) + f(0) * sin(angle), f(2) * f(2) * (1 - cos(angle)) + cos(angle));
    }

    /// @brief Rotational Matrix around the provided axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @param[in] axis: The vector of the axis to rotate around
    /// @return The rotation matrix
    static Matrix<3, 3, T> rot_deg(T const& angle, Vector<3, T> const& f)
    {
        return rot(angle * M_PI / 180, f);
    }

  public: // ############### Public Methods ###############
    /// @brief The matrix's determinant
    /// @return The matrix's determinant
    T determinant() const
    {
        return MatrixComponents<3, 3, T>::e00 * MatrixComponents<3, 3, T>::e11 * MatrixComponents<3, 3, T>::e22
               + MatrixComponents<3, 3, T>::e01 * MatrixComponents<3, 3, T>::e12 * MatrixComponents<3, 3, T>::e30
               + MatrixComponents<3, 3, T>::e02 * MatrixComponents<3, 3, T>::e10 * MatrixComponents<3, 3, T>::e21
               - MatrixComponents<3, 3, T>::e20 * MatrixComponents<3, 3, T>::e11 * MatrixComponents<3, 3, T>::e02
               - MatrixComponents<3, 3, T>::e21 * MatrixComponents<3, 3, T>::e12 * MatrixComponents<3, 3, T>::e00
               - MatrixComponents<3, 3, T>::e22 * MatrixComponents<3, 3, T>::e10 * MatrixComponents<3, 3, T>::e01;
    }

    /// @brief Converts the DCM Matrix into Euler Angles
    /// @return Vector with the Euler Angles
    Vector<3, T> toEuler()
    {
        return Vector<3, T>(atan2(-MatrixComponents<3, 3, T>::e12, MatrixComponents<3, 3, T>::e22),
                            asin(MatrixComponents<3, 3, T>::e02),
                            atan2(-MatrixComponents<3, 3, T>::e01, MatrixComponents<3, 3, T>::e00));
    }
};

/// @brief 4x4 matrix specialization
template<typename T>
class Matrix<4, 4, T> : public MatrixBase<4, 4, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new matrix with uninitialized components
    Matrix() {}

    /// @brief Creates new matrix with components initialized to val
    /// @param[in] val: The initialization value
    explicit Matrix(T val)
        : MatrixBase<4, 4, T>(val) {}

    /// @brief Creates a new matrix with its components initialized to the provided values
    /// @param[in] e00: Element 0,0 value
    /// @param[in] e01: Element 0,1 value
    /// @param[in] e02: Element 0,2 value
    /// @param[in] e03: Element 0,3 value
    /// @param[in] e10: Element 1,0 value
    /// @param[in] e11: Element 1,1 value
    /// @param[in] e12: Element 1,2 value
    /// @param[in] e13: Element 1,3 value
    /// @param[in] e20: Element 2,0 value
    /// @param[in] e21: Element 2,1 value
    /// @param[in] e22: Element 2,2 value
    /// @param[in] e23: Element 2,3 value
    /// @param[in] e30: Element 3,0 value
    /// @param[in] e31: Element 3,1 value
    /// @param[in] e32: Element 3,2 value
    /// @param[in] e33: Element 3,3 value
    Matrix(T e00, T e01, T e02, T e03,
           T e10, T e11, T e12, T e13,
           T e20, T e21, T e22, T e23,
           T e30, T e31, T e32, T e33)
    {
        MatrixComponents<4, 4, T>::e00 = e00;
        MatrixComponents<4, 4, T>::e01 = e01;
        MatrixComponents<4, 4, T>::e02 = e02;
        MatrixComponents<4, 4, T>::e03 = e03;
        MatrixComponents<4, 4, T>::e10 = e10;
        MatrixComponents<4, 4, T>::e11 = e11;
        MatrixComponents<4, 4, T>::e12 = e12;
        MatrixComponents<4, 4, T>::e13 = e13;
        MatrixComponents<4, 4, T>::e20 = e20;
        MatrixComponents<4, 4, T>::e21 = e21;
        MatrixComponents<4, 4, T>::e22 = e22;
        MatrixComponents<4, 4, T>::e23 = e23;
        MatrixComponents<4, 4, T>::e30 = e30;
        MatrixComponents<4, 4, T>::e31 = e31;
        MatrixComponents<4, 4, T>::e32 = e32;
        MatrixComponents<4, 4, T>::e33 = e33;
    }

    /// @brief Constructs a matrix from 4 column vectors
    /// @param[in] col0: The 0 column vector
    /// @param[in] col1: The 1 column vector
    /// @param[in] col2: The 2 column vector
    /// @param[in] col3: The 3 column vector
    Matrix(Vector<4, T> col0, Vector<4, T> col1, Vector<4, T> col2, Vector<4, T> col3)
    {
        MatrixComponents<4, 4, T>::e00 = col0.x;
        MatrixComponents<4, 4, T>::e01 = col1.x;
        MatrixComponents<4, 4, T>::e02 = col2.x;
        MatrixComponents<4, 4, T>::e03 = col3.x;
        MatrixComponents<4, 4, T>::e10 = col0.y;
        MatrixComponents<4, 4, T>::e11 = col1.y;
        MatrixComponents<4, 4, T>::e12 = col2.y;
        MatrixComponents<4, 4, T>::e13 = col3.y;
        MatrixComponents<4, 4, T>::e20 = col0.z;
        MatrixComponents<4, 4, T>::e21 = col1.z;
        MatrixComponents<4, 4, T>::e22 = col2.z;
        MatrixComponents<4, 4, T>::e23 = col3.z;
        MatrixComponents<4, 4, T>::e30 = col0.w;
        MatrixComponents<4, 4, T>::e31 = col1.w;
        MatrixComponents<4, 4, T>::e32 = col2.w;
        MatrixComponents<4, 4, T>::e33 = col3.w;
    }

    /// @brief Creates a new matrix from a MatrixBase
    Matrix(MatrixBase<4, 4, T> mb)
        : MatrixBase<4, 4, T>(mb) {}
};

// ################# Operator Overloads #################

/// @brief Multiplies two matrices together
/// @param[in] lhs: The left-side matrix
/// @param[in] rhs: The right-side matrix
/// @return The result
template<size_t m, size_t n, typename T, size_t r, size_t s, typename S>
Matrix<m, s, T> operator*(Matrix<m, n, T>& lhs, const Matrix<r, s, S>& rhs)
{
    // columns from the matrix must match rows of the input matrix
    assert(n == r);

    Matrix<m, s, T> return_mat(0);

    for (size_t mi = 0; mi < m; mi++)
        for (size_t si = 0; si < s; si++)
            for (size_t ni = 0; ni < n; ni++)
                return_mat(mi, si) += lhs(mi, ni) * rhs(ni, si);

    return return_mat;
}

/// @brief Adds two matrices together
/// @param[in] lhs: The left-side matrix
/// @param[in] rhs: The right-side matrix
/// @return The resulting matrix
template<size_t m, size_t n, typename T>
Matrix<m, n, T> operator+(Matrix<m, n, T> lhs, const Matrix<m, n, T>& rhs)
{
    lhs += rhs;

    return lhs;
}

/// @brief Substracts two matrices
/// @param[in] lhs: The left-side matrix
/// @param[in] rhs: The right-side matrix
/// @return The resulting matrix
template<size_t m, size_t n, typename T>
Matrix<m, n, T> operator-(Matrix<m, n, T> lhs, const Matrix<m, n, T>& rhs)
{
    lhs -= rhs;

    return lhs;
}

/// @brief Multiplies a matrix by a scalar.
/// @param[in] lhs: The matrix
/// @param[in] rhs: The scalar
/// @return The result
template<size_t m, size_t n, typename T, typename S>
Matrix<m, n, T> operator*(Matrix<m, n, T> lhs, const S& rhs)
{
    lhs *= rhs;

    return lhs;
}

/// @brief Multiplies a scalar by a matrix.
/// @param[in] lhs: The scalar
/// @param[in] rhs: The matrix
/// @return The result
template<size_t m, size_t n, typename T, typename S,
         typename = typename std::enable_if<std::is_arithmetic<S>::value, S>::type>
Matrix<m, n, T> operator*(const S& lhs, Matrix<m, n, T> rhs)
{
    rhs *= lhs;

    return rhs;
}

/// @brief Divides a matrix by a scalar
/// @param[in] lhs: The matrix
/// @param[in] rhs: The scalar
/// @return The result
template<size_t m, size_t n, typename T, typename S,
         typename = typename std::enable_if<std::is_arithmetic<S>::value, S>::type>
Matrix<m, n, T> operator/(Matrix<m, n, T> lhs, const S& rhs)
{
    lhs /= rhs;

    return lhs;
}

/// @brief Multiplies a matrix by a vector.
/// @param[in] lhs: The matrix
/// @param[in] rhs: The vector
/// @return The result
template<size_t m, size_t n, typename T, typename S>
Vector<m, T> operator*(Matrix<m, n, T> lhs, const Vector<n, S>& rhs)
{
    return lhs * static_cast<Matrix<n, 1, T>>(rhs);
}

/// @brief Multiplies a vector by a matrix.
/// @param[in] lhs: The vector
/// @param[in] rhs: The matrix
/// @return The result
template<size_t m, typename T, typename S>
Matrix<m, m, T> operator*(Vector<m, S>& lhs, const Matrix<1, m, T>& rhs)
{
    Matrix<m, 1, T> lhsmat = static_cast<Matrix<m, 1, T>>(lhs);
    return lhsmat * rhs;
}

// ################## Common functions ##################

/// @brief Provides a method to generate a representable string from a provided matrix
/// @param[in] m: The matrix to convert to string
/// @return The string representation
template<size_t m, size_t n, typename T>
std::string str(Matrix<m, n, T> mat)
{
    std::stringstream ss;
    ss << "[";

    for (size_t row_index = 0; row_index < mat.dimRow(); row_index++)
    {
        ss << "(";

        for (size_t col_index = 0; col_index < mat.dimCol(); col_index++)
        {
            ss << mat(row_index, col_index);

            if (col_index + 1 < mat.dimCol())
                ss << "; ";
        }

        ss << ")";
    }

    ss << "]";

    return ss.str();
}

/// @brief Overloads the ostream << operator for easy usage in displaying matrices
/// @param[in] out: The ostream being output to
/// @param[in] m The matrix to output to ostream
/// @return Reference to the current ostream
template<size_t m, size_t n, typename T>
std::ostream& operator<<(std::ostream& out, Matrix<m, n, T> mat)
{
    out << str(mat);
    return out;
}

} // namespace NAV