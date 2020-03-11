/**
 * @file Quaternion.hpp
 * @brief [Deprecated] Quaternion class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 * @attention [Deprecated] Eigen library will be used in the future
 */

#pragma once

#include <array>
#include <cmath>

#include "Vector.hpp"
#include "Matrix.hpp"

#include "vn/vector.h"

namespace NAV
{
/// @brief Template for the vector's components
/// q0 = cos(phi/2), q1 = f1 * sin(phi/2), q2 = f2 * sin(phi/2), q3 = f3 * sin(phi/2)
template<typename T = double>
class Quaternion
{
  public: // ############### Public Members ###############
    /// @brief The quaternion's components
    /// q0 = cos(phi/2), q1 = f1 * sin(phi/2), q2 = f2 * sin(phi/2), q3 = f3 * sin(phi/2)
    std::array<T, 4> q;

  public: // ################ Constructors ################
    /// @brief Creates a new quaternion with uninitialized components
    Quaternion() {}

    /// @brief Creates new quaternion with components initialized to val
    /// @param[in] val: The initialization value
    explicit Quaternion(T val)
    {
        Quaternion<T>::q.fill(val);
    }

    /// @brief Creates new quaternion with its components intialized to the provided values and q0 = 0
    /// @param[in] q1: The q1 value
    /// @param[in] q2: The q2 value
    /// @param[in] q3: The q3 value
    Quaternion(const T& q1, const T& q2, const T& q3)
    {
        q = { 0, q1, q2, q3 };
    }

    /// @brief Creates new quaternion with its components intialized to the provided values
    /// @param[in] q0: The q0 value
    /// @param[in] q1: The q1 value
    /// @param[in] q2: The q2 value
    /// @param[in] q3: The q3 value
    Quaternion(const T& q0, const T& q1, const T& q2, const T& q3)
    {
        q = { q0, q1, q2, q3 };
    }

    /// @brief Creates new quaternion with its components intialized to the provided values
    /// @param[in] q0: The q0 value
    /// @param[in] f: The vector
    Quaternion(const T& q0, const Vector<3, T>& f)
    {
        q = { q0, f(0), f(1), f(2) };
    }

    /// @brief Creates new quaternion from a 3-dimensional vector with q0 = 0
    /// @param[in] f: The vector
    Quaternion(const Vector<3, T>& f)
    {
        q = { 0, f(0), f(1), f(2) };
    }

    /// @brief Creates new quaternion from a 4-dimensional vector
    /// @param[in] f: The vector
    Quaternion(const Vector<4, T>& f)
    {
        q = { f(0), f(1), f(2), f(3) };
    }

    /// @brief Creates a new quaternion from a VectorNav Vector
    template<typename S>
    Quaternion(vn::math::vec<4, S> vnVec)
    {
        q[0] = vnVec[3];
        q[1] = vnVec[0];
        q[2] = vnVec[1];
        q[3] = vnVec[2];
    }

  public: // ############### Helper Methods ###############
    /// @brief Rotational quaternion around the x axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation quaternion
    static Quaternion<T> rotX(T const& angle)
    {
        return Quaternion<T>(cos(angle / 2), sin(angle / 2), 0, 0);
    }

    /// @brief Rotational quaternion around the x axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation quaternion
    static Quaternion<T> rotX_deg(T const& angle)
    {
        return rotX(angle * M_PI / 180);
    }
    /// @brief Rotational quaternion around the y axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation quaternion
    static Quaternion<T> rotY(T const& angle)
    {
        return Quaternion<T>(cos(angle / 2), 0, sin(angle / 2), 0);
    }

    /// @brief Rotational quaternion around the y axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation quaternion
    static Quaternion<T> rotY_deg(T const& angle)
    {
        return rotY(angle * M_PI / 180);
    }
    /// @brief Rotational quaternion around the z axis
    /// @param[in] angle: The angle to rotate around in radians
    /// @return The rotation quaternion
    static Quaternion<T> rotZ(T const& angle)
    {
        return Quaternion<T>(cos(angle / 2), 0, 0, sin(angle / 2));
    }

    /// @brief Rotational quaternion around the z axis
    /// @param[in] angle: The angle to rotate around in degree
    /// @return The rotation quaternion
    static Quaternion<T> rotZ_deg(T const& angle)
    {
        return rotZ(angle * M_PI / 180);
    }

    /// @brief Rotational quaternion around the provided axis
    /// @param[in] angle: Angle to rotate in radians
    /// @param[in] axis: Axis to rotate around
    /// @return The rotation quaternion
    static Quaternion<T> rot(T const& angle, Vector<3, T> const& axis)
    {
        Vector<3, T> f = axis.normalized();
        return Quaternion<T>(cos(angle / 2),
                             f(0) * sin(angle / 2),
                             f(1) * sin(angle / 2),
                             f(2) * sin(angle / 2));
    }

    /// @brief Rotational quaternion around the provided axis
    /// @param[in] angle: Angle to rotate in degree
    /// @param[in] axis: Axis to rotate around
    /// @return The rotation quaternion
    static Quaternion<T> rot_deg(T const& angle, Vector<3, T> const& axis)
    {
        return rot(angle * M_PI / 180, axis);
    }

    /// @brief Rotational quaternion from provided DCM Matrix
    /// @param[in] m: DCM Matrix
    /// @return The rotation quaternion
    static Quaternion<T> dcm2quat(Matrix<3, 3, T> const& dcm)
    {
        Quaternion<T> quat;
        quat(0) = 0.5 * sqrt(1 + dcm(0, 0) + dcm(1, 1) + dcm(2, 2));
        quat(1) = 0.5 * sqrt(1 + dcm(0, 0) - dcm(1, 1) - dcm(2, 2));
        quat(2) = 0.5 * sqrt(1 - dcm(0, 0) + dcm(1, 1) - dcm(2, 2));
        quat(3) = 0.5 * sqrt(1 - dcm(0, 0) - dcm(1, 1) + dcm(2, 2));
        if (!std::isnan(quat(0)) && (quat(0) > quat(1) || std::isnan(quat(1))) && (quat(0) > quat(2) || std::isnan(quat(2))) && (quat(0) > quat(3) || std::isnan(quat(3))))
        {
            quat(1) = (dcm(2, 1) - dcm(1, 2)) / (4 * quat(0));
            quat(2) = (dcm(0, 2) - dcm(2, 0)) / (4 * quat(0));
            quat(3) = (dcm(1, 0) - dcm(0, 1)) / (4 * quat(0));
        }
        else if (!std::isnan(quat(1)) && (quat(1) > quat(2) || std::isnan(quat(2))) && (quat(1) > quat(3) || std::isnan(quat(3))))
        {
            quat(0) = (dcm(2, 1) - dcm(1, 2)) / (4 * quat(1));
            quat(2) = (dcm(1, 0) + dcm(0, 1)) / (4 * quat(1));
            quat(3) = (dcm(0, 2) + dcm(2, 0)) / (4 * quat(1));
        }
        else if (!std::isnan(quat(2)) && (quat(2) > quat(3) || std::isnan(quat(3))))
        {
            quat(0) = (dcm(0, 2) - dcm(2, 0)) / (4 * quat(2));
            quat(1) = (dcm(1, 0) + dcm(0, 1)) / (4 * quat(2));
            quat(3) = (dcm(2, 1) + dcm(1, 2)) / (4 * quat(2));
        }
        else
        {
            quat(0) = (dcm(1, 0) - dcm(0, 1)) / (4 * quat(3));
            quat(1) = (dcm(0, 2) + dcm(2, 0)) / (4 * quat(3));
            quat(2) = (dcm(2, 1) + dcm(1, 2)) / (4 * quat(3));
        }
        return quat;
    }

  public: // ############# Operator Overloads #############
    /// @brief Converts a quaternion into a 4x1 vector
    /// @return The converted vector
    operator Vector<3, T>() const
    {
        return Vector<3, T>(q[1], q[2], q[3]);
    }

    /// @brief Converts a quaternion into a 4x1 vector
    /// @return The converted vector
    operator Vector<4, T>() const
    {
        return Vector<4, T>(q);
    }

    /// @brief Indexing into the quaternion's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the quaternion
    T& operator[](size_t index)
    {
        return const_cast<T&>(static_cast<const Quaternion&>(*this)[index]);
    }

    /// @brief Indexing into the quaternion's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the quaternion
    const T& operator[](size_t index) const
    {
        assert(index < 4);
        return q[index];
    }

    /// @brief Indexing into the quaternion's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the quaternion
    T& operator()(size_t index)
    {
        return const_cast<T&>(static_cast<const Quaternion&>(*this)(index));
    }

    /// @brief Indexing into the quaternion's components
    /// @param[in] index: 0-based component index
    /// @exception dimension_error: The index exceeded the dimension of the quaternion
    const T& operator()(size_t index) const
    {
        assert(index < 4);
        return q[index];
    }

    /// @brief Multiplies the quaternion by a scalar
    /// @param[in] rhs: The scalar
    /// @return The multiplied quaternion
    Quaternion& operator*=(const T& rhs)
    {
        for (size_t i = 0; i < 4; i++)
            q[i] *= rhs;

        return *this;
    }

    /// @brief Multiplies the quaternion by another quaternion
    /// @param[in] rhs: The right-side quaternion
    /// @return The multiplied quaternion
    Quaternion& operator+=(const Quaternion<T>& rhs)
    {
        for (size_t i = 0; i < 4; i++)
            q[i] += rhs.q[i];

        return *this;
    }

    /// @brief Multiplies the quaternion by another quaternion
    /// @param[in] rhs: The right-side quaternion
    /// @return The multiplied quaternion
    Quaternion& operator*=(const Quaternion<T>& rhs)
    {
        Vector<3, T> v_lhs = *this;
        Vector<3, T> v_rhs = rhs;
        Vector<3, T> result = q[0] * v_rhs + rhs.q[0] * v_lhs + v_lhs.cross(v_rhs);

        q[0] = q[0] * rhs.q[0] - v_lhs.dot(v_rhs);
        q[1] = result[0];
        q[2] = result[1];
        q[3] = result[2];

        return *this;
    }

    /// @brief Divides the quaternion by a scalar
    /// @param[in] rhs: The scalar
    /// @return The divided quaternion
    Quaternion& operator/=(const T& rhs)
    {
        for (size_t i = 0; i < 4; i++)
            q[i] /= rhs;

        return *this;
    }

    /// @brief Compares equality from a quaternion to this quaternion
    /// @param[in] rhs: The right-side quaternion
    /// @return True if components equal, else false
    bool operator==(const Quaternion<T>& rhs) const
    {
        for (size_t i = 0; i < 4; i++)
            if (q[i] != rhs.q[i])
                return false;

        return true;
    }

    /// @brief Compares inequality from a quaternion to this quaternion
    /// @param[in] rhs: The right-side quaternion
    /// @return True if components inequal, else false
    bool operator!=(const Quaternion<T>& rhs) const
    {
        for (size_t i = 0; i < 4; i++)
            if (q[i] != rhs.q[i])
                return true;

        return false;
    }

    // multiplication with quaternion
    // multiplication with vector

  public: // ############### Public Methods ###############
    /// @brief Conjugates the quaternion
    /// @return The conjugated quaternion
    Quaternion<T> conjugate() const
    {
        return Quaternion<T>(q[0], -q[1], -q[2], -q[3]);
    }

    /// @brief Calculates the inverse of the quaternion
    /// @return The quaternion's inverse
    Quaternion<T> inverse()
    {
        return conjugate() / mag_sqr();
        ;
    }

    /// @brief The quaternion's magnitude
    /// @return The magnitude
    T mag() const
    {
        T sumOfSquares = 0;

        for (size_t i = 0; i < 4; i++)
            sumOfSquares += q[i] * q[i];

        return sqrt(sumOfSquares);
    }

    /// @brief The quaternion's squared magnitude
    /// @return The squared magnitude
    T mag_sqr() const
    {
        T sumOfSquares = 0;

        for (size_t i = 0; i < 4; i++)
            sumOfSquares += q[i] * q[i];

        return sumOfSquares;
    }

    /// @brief Returns the normalized quaternion
    /// @return The normalized quaternion
    Quaternion<T> normalized() const
    {
        Quaternion<T> q_ret;

        T m = mag();

        for (size_t i = 0; i < 4; i++)
            q_ret.q[i] = q[i] / m;

        return q_ret;
    }

    /// @brief Normalizes the quaternion
    /// @return Nothing
    void normalize()
    {
        T m = mag();

        for (size_t i = 0; i < 4; i++)
            q[i] = q[i] / m;
    }

    /// @brief Computes the dot product of this and the provided quaternion
    /// @param[in] rhs: The right-side quaternion
    /// @return The computed dot product
    T dot(const Quaternion& rhs) const
    {
        return q[0] * rhs.q[0] + q[1] * rhs.q[1] + q[2] * rhs.q[2] + q[3] * rhs.q[3];
    }

    /// @brief Computes the cross product of this and the provided quaternion
    /// @param[in] rhs: The right-side quaternion
    /// @return The computed cross product
    Vector<3, T> cross(const Quaternion<T>& rhs) const
    {
        return ((Vector<3, T>)(*this)).cross((Vector<3, T>)(rhs));
    }

    /// @brief Converts the quaternion vector to a direction cosine matrix
    /// @return The direction cosine matrix
    Matrix<3, 3, T> toDCM() const
    {
        T s = 1.0 / mag_sqr();
        return Matrix<3, 3, T>(1 - 2 * s * (q[2] * q[2] + q[3] * q[3]), 2 * s * (q[1] * q[2] - q[3] * q[0]), 2 * s * (q[1] * q[3] + q[2] * q[0]),
                               2 * s * (q[1] * q[2] + q[3] * q[0]), 1 - 2 * s * (q[1] * q[1] + q[3] * q[3]), 2 * s * (q[2] * q[3] - q[1] * q[0]),
                               2 * s * (q[1] * q[3] - q[2] * q[0]), 2 * s * (q[2] * q[3] + q[1] * q[0]), 1 - 2 * s * (q[1] * q[1] + q[2] * q[2]));
        // return Matrix<3, 3, T>(q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3], 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[2] * q[0]),
        //                        2 * (q[1] * q[2] + q[3] * q[0]), q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3], 2 * (q[2] * q[3] - q[1] * q[0]),
        //                        2 * (q[1] * q[3] - q[2] * q[0]), 2 * (q[2] * q[3] + q[1] * q[0]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    }

    /// @brief Converts the Quaternion into Euler Angles
    /// @return Vector with the Euler Angles
    Vector<3, T> toEuler()
    {
        return toDCM().toEuler();
    }

    /// @brief Rotates the Vector with the quaternion
    /// @return The rotatet Vector
    Vector<3, T> rotVector(const Vector<3, T>& v)
    {
        return *this * v * inverse();
    }
};

// ################# Operator Overloads #################

/// @brief Multiplies a quaternion by a vector.
/// @param[in] lhs: The quaternion
/// @param[in] rhs: The vector
/// @return The result
template<typename T, typename S>
Quaternion<T> operator*(Quaternion<T> lhs, const Vector<3, S>& rhs)
{
    Quaternion<S> quat = static_cast<Quaternion<S>>(rhs);
    lhs *= quat;

    return lhs;
}

/// @brief Multiplies a vector by a quaternion.
/// @param[in] lhs: The vector
/// @param[in] rhs: The quaternion
/// @return The result
template<typename T, typename S>
Quaternion<T> operator*(Vector<3, S>& lhs, const Quaternion<T>& rhs)
{
    Quaternion<S> quat = static_cast<Quaternion<S>>(lhs);
    quat *= rhs;

    return quat;
}

/// @brief Multiplies a quaternion by a quaternion.
/// @param[in] lhs: The left-side quaternion
/// @param[in] rhs: The left-side quaternion
/// @return The result
template<typename T, typename S>
Quaternion<T> operator*(Quaternion<T> lhs, const Quaternion<S>& rhs)
{
    lhs *= rhs;

    return lhs;
}

/// @brief Multiplies a quaternion by a scalar.
/// @param[in] lhs: The quaternion
/// @param[in] rhs: The scalar
/// @return The result
template<typename T, typename S>
Quaternion<T> operator*(Quaternion<T> lhs, const S& rhs)
{
    lhs *= rhs;

    return lhs;
}

/// @brief Multiplies a scalar by a quaternion.
/// @param[in] lhs: The scalar
/// @param[in] rhs: The quaternion
/// @return The result
template<typename T, typename S>
Quaternion<T> operator*(const S& rhs, Quaternion<T> lhs)
{
    lhs *= rhs;

    return lhs;
}

/// @brief Divides a quaternion by a scalar
/// @param[in] lhs: The quaternion
/// @param[in] rhs: The scalar
/// @return The result
template<typename T, typename S>
Quaternion<T> operator/(Quaternion<T> lhs, const S& rhs)
{
    lhs /= rhs;

    return lhs;
}

// ################## Common functions ##################

/// \brief Provides a method to generate a representable string from a provided quaternion
/// \param[in] v: The quaternion to convert to string
/// \return The string representation
template<typename T>
std::string str(Quaternion<T> quat)
{
    std::stringstream ss;
    ss << "(";
    for (size_t i = 0; i < 4; i++)
    {
        ss << quat[i];

        if (i + 1 < 4)
            ss << "; ";
    }
    ss << ")";

    return ss.str();
}

/// \brief Overloads the ostream << operator for easy usage in displaying quaternions
/// \param[in] out: The ostream being output to
/// \param[in] v: The quaternion to output to ostream
/// \return Reference to the current ostream
template<typename T>
std::ostream& operator<<(std::ostream& out, Quaternion<T> quat)
{
    out << str(quat);
    return out;
}

} // namespace NAV