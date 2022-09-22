// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file vector.h
/// @brief Extract from the vnproglib
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_VECTORNAV_LIBRARY

    #include <cassert>
    #include <sstream>
    #include <ostream>
    #include <cmath>
    #include <limits>
    #include <cstdint>

// NOLINTBEGIN

namespace vn
{
namespace math
{

/// \brief Template for a Euclidean vector.
template<size_t tdim, typename T = float>
struct vec
{
    /// \brief The vector's components.
    T c[tdim];

    /// \brief Creates a new vector with uninitialized components.
    vec() = default;

    /// \brief Creates new vector with components initialized to val.
    /// \param[in] val The initialization value.
    explicit vec(T val)
    {
        std::fill_n(c, tdim, val);
    }

    /// \brief The vector's dimension.
    /// \return The vector's dimension.
    [[nodiscard]] size_t dim() const { return tdim; }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    T& operator[](size_t index)
    {
        return const_cast<T&>(static_cast<const vec&>(*this)[index]);
    }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    const T& operator[](size_t index) const
    {
        assert(index < tdim);

        return c[index];
    }
};

// Specializations ////////////////////////////////////////////////////////////

/// \brief Vector with 2 component specialization.
template<typename T>
struct vec<2, T>
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic" // error: ISO C++ prohibits anonymous structs
    union
    {
        struct
        {
            /// \brief X (0-component).
            T x;

            /// \brief Y (1-component).
            T y;
        };

        /// \brief The vector's components.
        T c[2];
    };
    #pragma GCC diagnostic pop

    /// \brief Creates a new vector with uninitialized components.
    vec() = default;

    /// \brief Creates new vector with components initialized to val.
    /// \param[in] val The initialization value.
    explicit vec(T val) : x(val), y(val) {}

    /// \brief Creates a new vector with its components inintialized to the provided values.
    /// \param[in] x_val The x value.
    /// \param[in] y_val The y value.
    vec(T x_val, T y_val) : x(x_val), y(y_val) {}

    /// \brief The vector's dimension.
    /// \return The vector's dimension.
    [[nodiscard]] size_t dim() const { return 2; }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    T& operator[](size_t index)
    {
        return const_cast<T&>(static_cast<const vec&>(*this)[index]);
    }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    const T& operator[](size_t index) const
    {
        assert(index < 2);

        return c[index];
    }
};

/// \brief Vector with 3 component specialization.
template<typename T>
struct vec<3, T>
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic" // error: ISO C++ prohibits anonymous structs
    union
    {
        struct
        {
            /// \brief X (0-component).
            T x;

            /// \brief Y (1-component).
            T y;

            /// \brief Z (2-component).
            T z;
        };

        struct
        {
            /// \brief Red (0-component).
            T r;

            /// \brief Green (1-component).
            T g;

            /// \brief Blue (2-component).
            T b;
        };

        /// \brief XY (0,1-components).
        vec<2, T> xy;

        /// \brief The vector's components.
        T c[3];
    };
    #pragma GCC diagnostic pop

    /// \brief Creates a new vector with uninitialized components.
    vec() = default;

    /// \brief Creates new vector with components initialized to val.
    /// \param[in] val The initialization value.
    explicit vec(T val) : x(val), y(val), z(val) {}

    /// \brief Creates a new vector with its components initialized to the provided values.
    /// \param[in] x_val The x value.
    /// \param[in] y_val The y value.
    /// \param[in] z_val The z value.
    vec(const T& x_val, const T& y_val, const T& z_val) : x(x_val), y(y_val), z(z_val) {}

    /// \brief The vector's dimension.
    /// \return The vector's dimension.
    [[nodiscard]] size_t dim() const { return 3; }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    T& operator[](size_t index)
    {
        return const_cast<T&>(static_cast<const vec&>(*this)[index]);
    }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    const T& operator[](size_t index) const
    {
        assert(index < 3);

        return c[index];
    }
};

/// \brief Vector with 4 component specialization.
template<typename T>
struct vec<4, T>
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic" // error: ISO C++ prohibits anonymous structs
    union
    {
        struct
        {
            /// \brief X (0-component).
            T x;

            /// \brief Y (1-component).
            T y;

            /// \brief Z (2-component).
            T z;

            /// \brief W (3-component).
            T w;
        };

        struct
        {
            /// \brief Red (0-component).
            T r;

            /// \brief Green (1-component).
            T g;

            /// \brief Blue (2-component).
            T b;

            /// \brief Alpha (3-component).
            T a;
        };

        /// \brief XY (0,1-components).
        vec<2, T> xy;

        /// \brief XYZ (0,1,2-components).
        vec<3, T> xyz;

        /// \brief RGB (0,1,2-components).
        vec<3, T> rgb;

        /// \brief The vector's components.
        T c[4];
    };
    #pragma GCC diagnostic pop

    /// \brief Creates a new vector with uninitialized components.
    vec() = default;

    /// \brief Creates new vector with components initialized to val.
    /// \param[in] val The initialization value.
    explicit vec(T val) : x(val), y(val), z(val), w(val) {}

    /// \brief Creates a new vector with its components inintialized to the provided values.
    /// \param[in] x_val The x value.
    /// \param[in] y_val The y value.
    /// \param[in] z_val The z value.
    /// \param[in] w_val The w value.
    vec(T x_val, T y_val, T z_val, T w_val) : x(x_val), y(y_val), z(z_val), w(w_val) {}

    /// \brief The vector's dimension.
    /// \return The vector's dimension.
    [[nodiscard]] size_t dim() const { return 4; }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    T& operator[](size_t index)
    {
        return const_cast<T&>(static_cast<const vec&>(*this)[index]);
    }

    /// \brief Indexing into the vector's components.
    /// \param[in] index 0-based component index.
    /// \exception dimension_error The index exceeded the dimension of the vector.
    const T& operator[](size_t index) const
    {
        assert(index < 4);

        return c[index];
    }
};

    #if defined(_MSC_VER)
        #pragma warning(pop)
    #endif

// Operator Overloads /////////////////////////////////////////////////////////

template<size_t tdim, typename T, typename S>
bool operator==(const vec<tdim, T>& lhs, const vec<tdim, S>& rhs)
{
    constexpr T EPSILON = 10.0 * std::numeric_limits<T>::epsilon();

    for (size_t i = 0; i < tdim; i++)
    {
        if (std::abs(lhs.c[i] - rhs.c[i]) > EPSILON)
        {
            return false;
        }
    }
    return true;
}

template<size_t tdim, typename T, typename S>
bool operator!=(const vec<tdim, T>& lhs, const vec<tdim, S>& rhs)
{
    return !(lhs == rhs);
}

    #if defined(_MSC_VER)
        #pragma warning(pop)
    #endif

// Specific Typedefs //////////////////////////////////////////////////////////

/// \brief 2-component vector using <c>float</c> as its underlying data type.
typedef vec<2> vec2;

/// \brief 3-component vector using <c>float</c> as its underlying data type.
typedef vec<3> vec3;

/// \brief 4-component vector using <c>float</c> as its underlying data type.
typedef vec<4> vec4;

/// \brief 2-component vector using <c>float</c> as its underlying data type.
typedef vec<2, float> vec2f;

/// \brief 3-component vector using <c>float</c> as its underlying data type.
typedef vec<3, float> vec3f;

/// \brief 4-component vector using <c>float</c> as its underlying data type.
typedef vec<4, float> vec4f;

/// \brief 2-component vector using <c>double</c> as its underlying data type.
typedef vec<2, double> vec2d;

/// \brief 3-component vector using <c>double</c> as its underlying data type.
typedef vec<3, double> vec3d;

/// \brief 4-component vector using <c>double</c> as its underlying data type.
typedef vec<4, double> vec4d;

/// \brief 2-component vector using <c>long double</c> as its underlying data type.
typedef vec<2, long double> vec2ld;

/// \brief 3-component vector using <c>long double</c> as its underlying data type.
typedef vec<3, long double> vec3ld;

/// \brief 4-component vector using <c>long double</c> as its underlying data type.
typedef vec<4, long double> vec4ld;

/// \brief 2-component vector using <c>int32_t</c> as its underlying data type.
typedef vec<2, int32_t> vec2i32;

/// \brief Namenclature used by OpenGL API.
typedef vec2i32 ivec2;

/// \brief 3-component vector using <c>int32_t</c> as its underlying data type.
typedef vec<3, int32_t> vec3i32;

/// \brief 4-component vector using <c>int32_t</c> as its underlying data type.
typedef vec<4, int32_t> vec4i32;

/// \brief 2-component vector using <c>uint32_t</c> as its underlying data type.
typedef vec<2, uint32_t> vec2u32;

/// \brief 3-component vector using <c>uint32_t</c> as its underlying data type.
typedef vec<3, uint32_t> vec3u32;

/// \brief 4-component vector using <c>uint32_t</c> as its underlying data type.
typedef vec<4, uint32_t> vec4u32;

// Common functions for working with vectors.

/// \brief Provides a method to generate a representable string from a provided
/// vector.
///
/// \param[in] v The vector to convert to string.
/// \return The string representation.
template<size_t tdim, typename T>
std::string str(vec<tdim, T> v)
{
    std::stringstream ss;
    ss << "(";
    for (size_t i = 0; i < v.dim(); i++)
    {
        ss << v[i];

        if (i + 1 < v.dim())
            ss << "; ";
    }
    ss << ")";

    return ss.str();
}

/// \brief Overloads the ostream << operator for easy usage in displaying
/// vectors.
///
/// \param[in] out The ostream being output to.
/// \param[in] v The vector to output to ostream.
/// \return Reference to the current ostream.
template<size_t tdim, typename T>
std::ostream& operator<<(std::ostream& out, vec<tdim, T> v)
{
    out << str(v);
    return out;
}

} // namespace math
} // namespace vn

// NOLINTEND

#endif