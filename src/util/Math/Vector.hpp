/**
 * @file Vector.hpp
 * @brief [Deprecated] Vector class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 * @attention [Deprecated] Eigen library will be used in the future
 */

#pragma once

#include <cassert>
#include <sstream>
#include <ostream>
#include <cmath>
#include <array>

#include <vn/vector.h>

namespace NAV
{
// Forward declaration of class
template<size_t dimension, typename T = double>
class Vector;

/// @brief Template for the vector's components
template<size_t dimension, typename T = double>
struct VectorComponents
{
    /// @brief The vector's components
    std::array<T, dimension> c;

    /// @brief Creates a new vectorComponent
    VectorComponents() {}
};

/// @brief Vector's components with 2 component specialization
template<typename T>
struct VectorComponents<2, T>
{
    union
    {
        struct
        {
            /// @brief X (0-component)
            T x;

            /// @brief Y (1-component)
            T y;
        };

        /// @brief The vector's components
        std::array<T, 2> c;
    };

    /// @brief Creates a new vectorComponent
    VectorComponents() {}
};

/// @brief Vector's components with 3 component specialization
template<typename T>
struct VectorComponents<3, T>
{
    union
    {
        struct
        {
            /// @brief X (0-component)
            T x;

            /// @brief Y (1-component)
            T y;

            /// @brief Z (2-component)
            T z;
        };

        struct
        {
            /// @brief N (0-component)
            T n;

            /// @brief E (1-component)
            T e;

            /// @brief D (2-component)
            T d;
        };

        struct
        {
            /// @brief Red (0-component)
            T r;

            /// @brief Green (1-component)
            T g;

            /// @brief Blue (2-component)
            T b;
        };

        /// @brief XY (0,1-components)
        Vector<2, T> xy;

        /// @brief The vector's components
        std::array<T, 3> c;
    };

    /// @brief Creates a new vectorComponent
    VectorComponents() {}
};

/// @brief Vector's components with 4 component specialization
template<typename T>
struct VectorComponents<4, T>
{
    union
    {
        struct
        {
            /// @brief X (0-component)
            T x;

            /// @brief Y (1-component)
            T y;

            /// @brief Z (2-component)
            T z;

            /// @brief W (3-component)
            T w;
        };

        struct
        {
            /// @brief Red (0-component)
            T r;

            /// @brief Green (1-component)
            T g;

            /// @brief Blue (2-component)
            T b;

            /// @brief Alpha (3-component)
            T a;
        };

        /// @brief XY (0,1-components)
        Vector<2, T> xy;

        /// @brief XYZ (0,1,2-components)
        Vector<3, T> xyz;

        /// @brief RGB (0,1,2-components)
        Vector<3, T> rgb;

        /// @brief The vector's components
        std::array<T, 4> c;
    };

    /// @brief Creates a new vectorComponent
    VectorComponents() {}
};

/// @brief Template for the vector's base
template<size_t dimension, typename T = double>
class VectorBase : public VectorComponents<dimension, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new vectorBase with uninitialized components
    VectorBase() {}

    /// @brief Creates new vector with components initialized to val
    /// @param[in] val: The initialization value
    explicit VectorBase(T val)
    {
        VectorComponents<dimension, T>::c.fill(val);
        // std::fill_n(VectorComponents<dimension, T>::c, dimension, val);
    }

    /// @brief Creates a new vector from a std::array
    VectorBase(std::array<T, dimension> a)
    {
        VectorComponents<dimension, T>::c = a;
    }

  public: // ############### Helper Methods ###############
    /// @brief Vector with all of its components set to 0
    /// @return The 0 vector
    static VectorBase zero()
    {
        return VectorBase<dimension, T>(0);
    }

    /// @brief Vector with all of its components set to 1
    /// @return The 1 vector
    static VectorBase one()
    {
        return VectorBase<dimension, T>(1);
    }

  public: // ############# Operator Overloads #############
    /// @brief Indexing into the vector's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the vector
    T& operator[](size_t index)
    {
        return const_cast<T&>(static_cast<const VectorBase&>(*this)[index]);
    }

    /// @brief Indexing into the vector's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the vector
    const T& operator[](size_t index) const
    {
        assert(index < dimension);
        return VectorComponents<dimension, T>::c[index];
    }

    /// @brief Indexing into the vector's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the vector
    T& operator()(size_t index)
    {
        return const_cast<T&>(static_cast<const VectorBase&>(*this)(index));
    }

    /// @brief Indexing into the vector's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the vector
    const T& operator()(size_t index) const
    {
        assert(index < dimension);
        return VectorComponents<dimension, T>::c[index];
    }

    /// @brief Negates the vector
    /// @return The negated vector
    VectorBase<dimension, T> operator-() const
    {
        return neg();
    }

    /// @brief Adds a vector to this vector
    /// @param[in] rhs: The right-side vector
    /// @return The resulting vector
    VectorBase<dimension, T>& operator+=(const VectorBase& rhs)
    {
        for (size_t i = 0; i < dimension; i++)
            VectorComponents<dimension, T>::c[i] += rhs.c[i];

        return *this;
    }

    /// @brief Subtracts a vector from this vector
    /// @param[in] rhs: The right-side vector
    /// @return The resulting vector
    VectorBase<dimension, T>& operator-=(const VectorBase& rhs)
    {
        for (size_t i = 0; i < dimension; i++)
            VectorComponents<dimension, T>::c[i] -= rhs.c[i];

        return *this;
    }

    /// @brief Multiplies the vector by a scalar
    /// @param[in] rhs: The scalar
    /// @return The multiplied vector
    VectorBase<dimension, T>& operator*=(const T& rhs)
    {
        for (size_t i = 0; i < dimension; i++)
            VectorComponents<dimension, T>::c[i] *= rhs;

        return *this;
    }

    /// @brief Divides the vector by a scalar
    /// @param[in] rhs: The scalar
    /// @return The divided vector
    VectorBase<dimension, T>& operator/=(const T& rhs)
    {
        for (size_t i = 0; i < dimension; i++)
            VectorComponents<dimension, T>::c[i] /= rhs;

        return *this;
    }

    /// @brief Compares equality from a vector to this vector
    /// @param[in] rhs: The right-side vector
    /// @return True if components equal, else false
    bool operator==(const VectorBase<dimension, T>& rhs) const
    {
        for (size_t i = 0; i < dimension; i++)
            if (VectorComponents<dimension, T>::c[i] != rhs.c[i])
                return false;

        return true;
    }

    /// @brief Compares inequality from a vector to this vector
    /// @param[in] rhs: The right-side vector
    /// @return True if components inequal, else false
    bool operator!=(const VectorBase<dimension, T>& rhs) const
    {
        for (size_t i = 0; i < dimension; i++)
            if (VectorComponents<dimension, T>::c[i] != rhs.c[i])
                return true;

        return false;
    }

  public: // ############### Public Methods ###############
    /// @brief The vector's dimension
    /// @return The vector's dimension
    size_t dim() const
    {
        return dimension;
    }

    /// @brief Negates the vector
    /// @return The negated vector
    VectorBase<dimension, T> neg() const
    {
        VectorBase v;

        for (size_t i = 0; i < dimension; i++)
            v.c[i] = -VectorComponents<dimension, T>::c[i];

        return v;
    }

    /// @brief The vector's magnitude
    /// @return The magnitude
    T mag() const
    {
        T sumOfSquares = 0;

        for (size_t i = 0; i < dimension; i++)
            sumOfSquares += VectorComponents<dimension, T>::c[i] * VectorComponents<dimension, T>::c[i];

        return sqrt(sumOfSquares);
    }

    /// @brief Normalizes the vector
    /// @return The normalized vector
    VectorBase<dimension, T> normalized() const
    {
        VectorBase v;

        T m = mag();

        for (size_t i = 0; i < dimension; i++)
            v.c[i] = VectorComponents<dimension, T>::c[i] / m;

        return v;
    }

    /// @brief Normalizes the vector
    /// @return Nothing
    void normalize()
    {
        T m = mag();

        for (size_t i = 0; i < dimension; i++)
            VectorComponents<dimension, T>::c[i] = VectorComponents<dimension, T>::c[i] / m;
    }

    /// @brief Computes the dot product of this and the provided vector
    /// @param[in] rhs: The right-side vector
    /// @return The computed dot product
    T dot(const VectorBase& rhs) const
    {
        T runningSum = 0;

        for (size_t i = 0; i < dimension; i++)
            runningSum += VectorComponents<dimension, T>::c[i] * rhs.c[i];

        return runningSum;
    }
};

/// @brief Template for an Euclidean vector
template<size_t dimension, typename T>
class Vector : public VectorBase<dimension, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new vector with uninitialized components
    Vector() {}

    /// @brief Creates new vector with components initialized to val
    /// @param[in] val: The initialization value
    explicit Vector(T val)
        : VectorBase<dimension, T>(val) {}

    /// @brief Creates a new vector from a VectorBase
    Vector(VectorBase<dimension, T> vb)
        : VectorBase<dimension, T>(vb) {}

    /// @brief Creates a new vector from a std::array
    Vector(std::array<T, dimension> a)
        : VectorBase<dimension, T>(a) {}

    /// @brief Creates a new vector from another Vector
    template<typename S>
    Vector(Vector<dimension, S> vec)
    {
        for (size_t i = 0; i < dimension; i++)
        {
            VectorBase<dimension, T>::c[i] = vec[i];
        }
    }
};

/// @brief Vector with 2 component specialization
template<typename T>
class Vector<2, T> : public VectorBase<2, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new vector with uninitialized components
    Vector() {}

    /// @brief Creates new vector with components initialized to val
    /// @param[in] val: The initialization value
    explicit Vector(T val)
        : VectorBase<2, T>(val) {}

    /// @brief Creates a new vector with its components intialized to the provided values
    /// @param[in] x_val: The x value
    /// @param[in] y_val: The y value
    Vector(const T& x_val, const T& y_val)
    {
        VectorBase<2, T>::x = x_val;
        VectorBase<2, T>::y = y_val;
    }

    /// @brief Creates a new vector from a VectorBase
    Vector(VectorBase<2, T> vb)
        : VectorBase<2, T>(vb) {}

    /// @brief Creates a new vector from another Vector
    template<typename S>
    Vector(Vector<2, S> vec)
    {
        for (size_t i = 0; i < 2; i++)
        {
            VectorBase<2, T>::c[i] = vec[i];
        }
    }

  public: // ############### Helper Methods ###############
    /// @brief Unit vector pointing in the X (0-component) direction
    /// @return The unit vector
    static Vector unitX()
    {
        return Vector<2, T>(1, 0);
    }

    /// @brief Unit vector pointing in the Y (1-component) direction
    /// @return The unit vector
    static Vector unitY()
    {
        return Vector<2, T>(0, 1);
    }
};

/// @brief Vector with 3 component specialization
template<typename T>
class Vector<3, T> : public VectorBase<3, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new vector with uninitialized components
    Vector() {}

    /// @brief Creates new vector with components initialized to val
    /// @param[in] val: The initialization value
    explicit Vector(T val)
        : VectorBase<3, T>(val) {}

    /// @brief Creates a new vector with its components initialized to the provided values
    /// @param[in] x_val: The x value
    /// @param[in] y_val: The y value
    /// @param[in] z_val: The z value
    Vector(const T& x_val, const T& y_val, const T& z_val)
    {
        VectorBase<3, T>::x = x_val;
        VectorBase<3, T>::y = y_val;
        VectorBase<3, T>::z = z_val;
    }

    /// @brief Creates a new vector from a VectorBase
    Vector(VectorBase<3, T> vb)
        : VectorBase<3, T>(vb) {}

    /// @brief Creates a new vector from a VectorNav Vector
    template<typename S>
    Vector(vn::math::vec<3, S> vnVec)
    {
        VectorBase<3, T>::x = vnVec[0];
        VectorBase<3, T>::y = vnVec[1];
        VectorBase<3, T>::z = vnVec[2];
    }

    /// @brief Creates a new vector from another Vector
    template<typename S>
    Vector(Vector<3, S> vec)
    {
        for (size_t i = 0; i < 3; i++)
        {
            VectorBase<3, T>::c[i] = vec[i];
        }
    }

  public: // ############### Helper Methods ###############
    /// @brief Unit vector pointing in the X (0-component) direction
    /// @return The unit vector
    static Vector unitX()
    {
        return Vector<3, T>(1, 0, 0);
    }

    /// @brief Unit vector pointing in the Y (1-component) direction
    /// @return The unit vector
    static Vector unitY()
    {
        return Vector<3, T>(0, 1, 0);
    }

    /// @brief Unit vector pointing in the Z (2-component) direction
    /// @return The unit vector
    static Vector unitZ()
    {
        return Vector<3, T>(0, 0, 1);
    }

  public: // ############### Public Methods ###############
    /// @brief Computes the cross product of this and the provided vector
    /// @param[in] rhs: The right-side vector
    /// @return The computed cross product
    /// @exception dimension_error: The dimension of the vector is not 3
    Vector<3, T> cross(const Vector<3, T>& rhs) const
    {
        Vector<3, T> v;

        v.c[0] = VectorBase<3, T>::c[1] * rhs.c[2] - VectorBase<3, T>::c[2] * rhs.c[1];
        v.c[1] = VectorBase<3, T>::c[2] * rhs.c[0] - VectorBase<3, T>::c[0] * rhs.c[2];
        v.c[2] = VectorBase<3, T>::c[0] * rhs.c[1] - VectorBase<3, T>::c[1] * rhs.c[0];

        return v;
    }
};

/// @brief Vector with 4 component specialization
template<typename T>
class Vector<4, T> : public VectorBase<4, T>
{
  public: // ################ Constructors ################
    /// @brief Creates a new vector with uninitialized components
    Vector() {}

    /// @brief Creates new vector with components initialized to val
    /// @param[in] val: The initialization value
    explicit Vector(T val)
        : VectorBase<4, T>(val) {}

    /// @brief Creates a new vector with its components initialized to the provided values
    /// @param[in] x_val: The x value
    /// @param[in] y_val: The y value
    /// @param[in] z_val: The z value
    /// @param[in] w_val: The w value
    Vector(const T& x_val, const T& y_val, const T& z_val, const T& w_val)
    {
        VectorBase<4, T>::x = x_val;
        VectorBase<4, T>::y = y_val;
        VectorBase<4, T>::z = z_val;
        VectorBase<4, T>::w = w_val;
    }

    /// @brief Creates a new vector from a VectorBase
    Vector(VectorBase<4, T> vb)
        : VectorBase<4, T>(vb) {}

    /// @brief Creates a new vector from another Vector
    template<typename S>
    Vector(Vector<4, S> vec)
    {
        for (size_t i = 0; i < 4; i++)
        {
            VectorBase<4, T>::c[i] = vec[i];
        }
    }

  public: // ############### Helper Methods ###############
    /// @brief Unit vector pointing in the X (0-component) direction
    /// @return The unit vector
    static Vector unitX()
    {
        return Vector<4, T>(1, 0, 0, 0);
    }

    /// @brief Unit vector pointing in the Y (1-component) direction
    /// @return The unit vector
    static Vector unitY()
    {
        return Vector<4, T>(0, 1, 0, 0);
    }

    /// @brief Unit vector pointing in the Z (2-component) direction
    /// @return The unit vector
    static Vector unitZ()
    {
        return Vector<4, T>(0, 0, 1, 0);
    }

    /// @brief Unit vector pointing in the W (3-component) direction
    /// @return The unit vector
    static Vector unitW()
    {
        return Vector<4, T>(0, 0, 0, 1);
    }
};

// ################# Operator Overloads #################

/// @brief Adds two vectors together
/// @param[in] lhs: The left-side vector
/// @param[in] rhs: The right-side vector
/// @return The resulting vector
template<size_t dimension, typename T>
Vector<dimension, T> operator+(Vector<dimension, T> lhs, const Vector<dimension, T>& rhs)
{
    lhs += rhs;

    return lhs;
}

/// @brief Subtracts a vector from another vector
/// @param[in] lhs: The left-side vector
/// @param[in] rhs: The right-side vector
/// @return The resulting vector
template<size_t dimension, typename T>
Vector<dimension, T> operator-(Vector<dimension, T> lhs, const Vector<dimension, T>& rhs)
{
    lhs -= rhs;

    return lhs;
}

/// @brief Multiplies a vector by a scalar.
/// @param[in] lhs: The scalar
/// @param[in] rhs: The vector
/// @return The result
template<size_t dimension, typename T, typename S>
Vector<dimension, T> operator*(Vector<dimension, T> lhs, const S& rhs)
{
    lhs *= rhs;

    return lhs;
}

/// @brief Multiplies a scalar by a vector.
/// @param[in] lhs: The vector
/// @param[in] rhs: The scalar
/// @return The result
template<size_t dimension, typename T, typename S,
         typename = typename std::enable_if<std::is_arithmetic<S>::value, S>::type>
Vector<dimension, T> operator*(const S& rhs, Vector<dimension, T> lhs)
{
    lhs *= rhs;

    return lhs;
}

/// @brief Divides a vector by a scalar
/// @param[in] lhs: The vector
/// @param[in] rhs: The scalar
/// @return The result
template<size_t dimension, typename T, typename S>
Vector<dimension, T> operator/(Vector<dimension, T> lhs, const S& rhs)
{
    lhs /= rhs;

    return lhs;
}

// ################## Common functions ##################

/// \brief Provides a method to generate a representable string from a provided vector
/// \param[in] v: The vector to convert to string
/// \return The string representation
template<size_t dimension, typename T>
std::string str(Vector<dimension, T> v)
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

/// \brief Overloads the ostream << operator for easy usage in displaying vectors
/// \param[in] out: The ostream being output to
/// \param[in] v: The vector to output to ostream
/// \return Reference to the current ostream
template<size_t dimension, typename T>
std::ostream& operator<<(std::ostream& out, Vector<dimension, T> v)
{
    out << str(v);
    return out;
}

} // namespace NAV