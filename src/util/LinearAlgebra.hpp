/// @file LinearAlgebra.hpp
/// @brief Vector space operations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-24
/// @note To use Eigen directly again and ignore the frame checks, comment out the Matrix class and use this
///         constexpr auto StorageOptionsDefault = (Eigen::StorageOptions::ColMajor | Eigen::StorageOptions::AutoAlign);
///         template<CoordinateSystem _System, typename _Scalar, int _Rows, int _Cols,
///                 int _Options = StorageOptionsDefault, int _MaxRows = _Rows, int _MaxCols = _Cols>
///         using Matrix = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Eigen
{
using Array3ld = Array<long double, 3, 1>;

using Vector3ld = Matrix<long double, 3, 1>;
using Vector4ld = Matrix<long double, 4, 1>;

using Matrix3ld = Matrix<long double, 3, 3>;
using Matrix4ld = Matrix<long double, 4, 4>;

using Quaternionld = Quaternion<long double>;

using AngleAxisld = AngleAxis<long double>;
} // namespace Eigen

namespace NAV
{
/// @brief Available Coordinate Systems
enum CoordinateSystem
{
    /// Inertial frame (i-frame)
    Inertial,
    /// Earth-centered-Earth-fixed frame (e-frame)
    Earth,
    /// Local Navigation frame (n-frame) = North, East, Down frame
    Navigation,
    /// Body frame (b-frame)
    Body,
    /// Platform frame (p-frame)
    Platform,
    /// 𝜙 Geodetic latitude, λ Geodetic longitude, Altitude (Height above ground)
    LLA,
};

/// @brief Matrix class which rotates data from a coordinate system into another
/// @tparam _System_To The target coordinate system
/// @tparam _System_From The source coordinate system
template<CoordinateSystem _System_To, CoordinateSystem _System_From, typename _Scalar, int _Rows, int _Cols>
class Matrix : public Eigen::Matrix<_Scalar, _Rows, _Cols>
{
  public:
    /// @brief Default constructor leaving the matrix uninitialized
    Matrix() = default;
    /// @brief Destructor
    ~Matrix() = default;
    /// @brief Copy constructor
    Matrix(const Matrix&) = default;
    /// @brief Move constructor
    Matrix(Matrix&&) noexcept = default;
    /// @brief Copy assignment operator
    Matrix& operator=(const Matrix&) = default;
    /// @brief Move assignment operator
    Matrix& operator=(Matrix&&) noexcept = default;

    /// @brief Construct an object from an Eigen::Matrix
    /// @param[in] eigen The base object
    explicit Matrix(const Eigen::Matrix<_Scalar, _Rows, _Cols>& eigen)
        : Eigen::Matrix<_Scalar, _Rows, _Cols>(eigen) {}

    /// @brief Construct an object from any single argument
    /// @tparam T Type of the argument
    /// @param[in] arg The argument which should be passed to Eigen::Matrix
    template<class T>
    explicit Matrix(const T& arg)
        : Eigen::Matrix<_Scalar, _Rows, _Cols>(arg)
    {}

    /// @brief Constructs and initializes a 3D Vector from its three coefficients x, y and z
    /// @param[in] x First component
    /// @param[in] y Second component
    /// @param[in] z Third component
    Matrix(const _Scalar& x, const _Scalar& y, const _Scalar& z)
        : Eigen::Matrix<_Scalar, _Rows, _Cols>(x, y, z) {}

    /// @brief Constructs and initializes a 4D Vector from its three coefficients x, y, z and w
    /// @param[in] x First component
    /// @param[in] y Second component
    /// @param[in] z Third component
    /// @param[in] w Fourth component
    Matrix(const _Scalar& x, const _Scalar& y, const _Scalar& z, const _Scalar& w)
        : Eigen::Matrix<_Scalar, _Rows, _Cols>(x, y, z, w) {}

    template<CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar>
    [[nodiscard]] Matrix<_System_To, _System_From, _Scalar, 3, 1> cross(const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, 3, 1>& rhs) const
    {
        static_assert(_System_To == rhs_System_To && _System_From == rhs_System_From && "Can not summate different coordinate frames");

        const Eigen::Matrix<_Scalar, _Rows, _Cols>& _eig = *this;
        const Eigen::Matrix<rhs_Scalar, 3, 1>& rhs_eig = rhs;

        const auto crossProduct = Eigen::Matrix<rhs_Scalar, 3, 1>(_eig.cross(rhs_eig));
        return Matrix<_System_To, _System_From, _Scalar, 3, 1>(crossProduct);
    }

    /// @brief Returns the identity matrix or vector
    /// @return An expression of the identity matrix (not necessarily square).
    [[nodiscard]] static Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols> Identity()
    {
        auto eig = Eigen::Matrix<_Scalar, _Rows, _Cols>(Eigen::Matrix<_Scalar, _Rows, _Cols>::Identity());
        return Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols>(eig);
    }

    /// @brief Returns a zero matrix or vector
    /// @return An expression of a fixed-size zero matrix or vector
    [[nodiscard]] static Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols> Zero()
    {
        auto eig = Eigen::Matrix<_Scalar, _Rows, _Cols>(Eigen::Matrix<_Scalar, _Rows, _Cols>::Zero());
        return Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols>(eig);
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                            Operator Overloads                                            */
    /* -------------------------------------------------------------------------------------------------------- */

    template<CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar, int rhs_Rows, int rhs_Cols>
    Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols>& operator+=(const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>& rhs)
    {
        static_assert(_System_To == rhs_System_To && _System_From == rhs_System_From && "Can not summate different coordinate frames");

        Eigen::Matrix<_Scalar, _Rows, _Cols>& _eig = *this;
        const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols>& rhs_eig = rhs;

        _eig += rhs_eig;

        *this = Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols>(_eig);

        return *this;
    }

    template<CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar, int rhs_Rows, int rhs_Cols>
    Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols>& operator-=(const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>& rhs)
    {
        static_assert(_System_To == rhs_System_To && _System_From == rhs_System_From && "Can not substract different coordinate frames");

        Eigen::Matrix<_Scalar, _Rows, _Cols>& _eig = *this;
        const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols>& rhs_eig = rhs;

        _eig -= rhs_eig;

        *this = Matrix<_System_To, _System_From, _Scalar, _Rows, _Cols>(_eig);

        return *this;
    }
};

/// @brief Quaternion class which rotates data from a coordinate system into another
/// @tparam _System_To The target coordinate system
/// @tparam _System_From The source coordinate system
template<CoordinateSystem _System_To, CoordinateSystem _System_From, typename _Scalar>
class Quaternion : public Eigen::Quaternion<_Scalar>
{
  public:
    /// @brief Default constructor leaving the quaternion uninitialized
    Quaternion() = default;
    /// @brief Destructor
    ~Quaternion() = default;
    /// @brief Copy constructor
    Quaternion(const Quaternion&) = default;
    /// @brief Move constructor
    Quaternion(Quaternion&&) noexcept = default;
    /// @brief Copy assignment operator
    Quaternion& operator=(const Quaternion&) = default;
    /// @brief Move assignment operator
    Quaternion& operator=(Quaternion&&) noexcept = default;

    /// @brief Construct an object from an Eigen::Quaternion
    /// @param[in] eigen The base object
    explicit Quaternion(const Eigen::Quaternion<_Scalar>& eigen)
        : Eigen::Quaternion<_Scalar>(eigen) {}

    /// @brief Constructs and initializes the quaternion $ w+xi+yj+zk $ from its four coefficients w, x, y and z
    /// @param[in] w Real coefficient
    /// @param[in] x Imaginary coefficient
    /// @param[in] y Imaginary coefficient
    /// @param[in] z Imaginary coefficient
    Quaternion(const _Scalar& w, const _Scalar& x, const _Scalar& y, const _Scalar& z)
        : Eigen::Quaternion<_Scalar>(w, x, y, z) {}

    /// @brief Constructs and initializes a quaternion from the angle-axis aa
    /// @param[in] aa The rotation angle-axis expression
    explicit Quaternion(const Eigen::AngleAxis<_Scalar>& aa)
        : Eigen::Quaternion<_Scalar>(aa) {}

    /// @brief Constructs and initializes a quaternion from a rotation matrix expression
    /// @param[in] rot The rotation matrix expression
    explicit Quaternion(const Eigen::Matrix<_Scalar, 3, 3>& rot)
        : Eigen::Quaternion<_Scalar>(rot) {}

    /// @brief Constructs and initializes a quaternion from a 4D vector expression representing quaternion coefficients
    /// @param[in, out] coefficients 4D vector expression representing quaternion coefficients
    explicit Quaternion(const Eigen::Matrix<_Scalar, 4, 1>& coefficients)
        : Eigen::Quaternion<_Scalar>(coefficients) {}

    /// @brief Constructs and initialize a quaternion from the array data
    /// @param[in] data 4 element array representing quaternion coefficients
    explicit Quaternion(const _Scalar* data)
        : Eigen::Quaternion<_Scalar>(data) {}

    /// @brief The conjugated quaternion
    /// @return The conjugate of the *this which is equal to the multiplicative inverse if the quaternion is normalized.
    ///         The conjugate of a quaternion represents the opposite rotation.
    [[nodiscard]] Quaternion<_System_From, _System_To, _Scalar> conjugate() const
    {
        const Eigen::Quaternion<_Scalar>& _eig = *this;
        return Quaternion<_System_From, _System_To, _Scalar>(Eigen::Quaternion<_Scalar>(_eig.conjugate()));
    }

    /// @brief The quaternion describing the inverse rotation
    /// @return The multiplicative inverse of *this. Note that in most cases, i.e., if you simply want the opposite rotation,
    ///         and/or the quaternion is normalized, then it is enough to use the conjugate.
    [[nodiscard]] Quaternion<_System_From, _System_To, _Scalar> inverse() const
    {
        const Eigen::Quaternion<_Scalar>& _eig = *this;
        return Quaternion<_System_From, _System_To, _Scalar>(Eigen::Quaternion<_Scalar>(_eig.inverse()));
    }

    /// @brief Returns the identity quaternion
    /// @return An expression of the identity quaternion
    [[nodiscard]] static Quaternion<_System_To, _System_From, _Scalar> Identity()
    {
        auto eig = Eigen::Quaternion<_Scalar>(Eigen::Quaternion<_Scalar>::Identity());
        return Quaternion<_System_To, _System_From, _Scalar>(eig);
    }
};

/* -------------------------------------------------------------------------------------------------------- */
/*                                                 Type Defs                                                */
/* -------------------------------------------------------------------------------------------------------- */

template<CoordinateSystem _System, typename _Scalar, int _Rows>
using Vector = Matrix<_System, _System, _Scalar, _Rows, 1>;
template<CoordinateSystem _System, typename _Scalar>
using Vector3 = Vector<_System, _Scalar, 3>;
template<CoordinateSystem _System>
using Vector3d = Vector3<_System, double>;

template<CoordinateSystem _System, typename _Scalar>
using Vector4 = Vector<_System, _Scalar, 4>;
template<CoordinateSystem _System>
using Vector4d = Vector4<_System, double>;

template<CoordinateSystem _System_To, CoordinateSystem _System_From, typename _Scalar>
using Matrix3 = Matrix<_System_To, _System_From, _Scalar, 3, 3>;
template<CoordinateSystem _System_To, CoordinateSystem _System_From>
using Matrix3d = Matrix3<_System_To, _System_From, double>;

template<CoordinateSystem _System_To, CoordinateSystem _System_From, typename _Scalar>
using Matrix4 = Matrix<_System_To, _System_From, _Scalar, 4, 4>;
template<CoordinateSystem _System_To, CoordinateSystem _System_From>
using Matrix4d = Matrix4<_System_To, _System_From, double>;

template<CoordinateSystem _System_To, CoordinateSystem _System_From>
using Quaterniond = Quaternion<_System_To, _System_From, double>;

/* -------------------------------------------------------------------------------------------------------- */
/*                                      Vector summation & substraction                                     */
/* -------------------------------------------------------------------------------------------------------- */

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar, int rhs_Rows, int rhs_Cols>
Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols> operator+(const Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>& lhs,
                                                                                 const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>& rhs)
{
    static_assert(lhs_System_To == rhs_System_To && lhs_System_From == rhs_System_From && "Can not summate different coordinate frames");

    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols> sum = lhs_eig + rhs_eig;

    return Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>(sum);
}

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar, int rhs_Rows, int rhs_Cols>
Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols> operator-(const Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>& lhs,
                                                                                 const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>& rhs)
{
    static_assert(lhs_System_To == rhs_System_To && lhs_System_From == rhs_System_From && "Can not substract different coordinate frames");

    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols> difference = lhs_eig - rhs_eig;

    return Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>(difference);
}

/* -------------------------------------------------------------------------------------------------------- */
/*                                          Rotation concatenation                                          */
/* -------------------------------------------------------------------------------------------------------- */

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar, int rhs_Rows, int rhs_Cols>
Matrix<lhs_System_To, rhs_System_From, lhs_Scalar, lhs_Rows, rhs_Cols> operator*(const Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>& lhs,
                                                                                 const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>& rhs)
{
    static_assert(lhs_System_From == rhs_System_To && "Can not concatenate rotations with different coordinate frames");

    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, rhs_Cols> concatenation = lhs_eig * rhs_eig;

    return Matrix<lhs_System_To, rhs_System_From, lhs_Scalar, lhs_Rows, rhs_Cols>(concatenation);
}

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar,
         CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar>
Quaternion<lhs_System_To, rhs_System_From, lhs_Scalar> operator*(const Quaternion<lhs_System_To, lhs_System_From, lhs_Scalar>& lhs,
                                                                 const Quaternion<rhs_System_To, rhs_System_From, rhs_Scalar>& rhs)
{
    static_assert(lhs_System_From == rhs_System_To && "Can not concatenate rotations with different coordinate frames");

    const Eigen::Quaternion<lhs_Scalar>& lhs_eig = lhs;
    const Eigen::Quaternion<rhs_Scalar>& rhs_eig = rhs;

    Eigen::Quaternion<lhs_Scalar> concatenation = lhs_eig * rhs_eig;

    return Quaternion<lhs_System_To, rhs_System_From, lhs_Scalar>(concatenation);
}

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Vector rotation                                             */
/* -------------------------------------------------------------------------------------------------------- */

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         CoordinateSystem rhs_System, typename rhs_Scalar, int rhs_Rows>
Vector<lhs_System_To, lhs_Scalar, lhs_Rows> operator*(const Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>& lhs,
                                                      const Vector<rhs_System, rhs_Scalar, rhs_Rows>& rhs)
{
    static_assert(lhs_System_From == rhs_System && "Coordinate frames of rotation are not matching");

    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, 1>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, 1> concatenation = lhs_eig * rhs_eig;

    return Vector<lhs_System_To, lhs_Scalar, lhs_Rows>(concatenation);
}

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar,
         CoordinateSystem rhs_System, typename rhs_Scalar>
Vector3<lhs_System_To, lhs_Scalar> operator*(const Quaternion<lhs_System_To, lhs_System_From, lhs_Scalar>& lhs,
                                             const Vector3<rhs_System, rhs_Scalar>& rhs)
{
    static_assert(lhs_System_From == rhs_System && "Coordinate frames of rotation are not matching");

    const Eigen::Quaternion<lhs_Scalar>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, 3, 1>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, 3, 1> concatenation = lhs_eig * rhs_eig;

    return Vector3<lhs_System_To, lhs_Scalar>(concatenation);
}

/* -------------------------------------------------------------------------------------------------------- */
/*                                     Scalar Multiplication & Division                                     */
/* -------------------------------------------------------------------------------------------------------- */

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         class T>
Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols> operator*(const Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>& lhs,
                                                                                 const T& rhs)
{
    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols>& lhs_eig = lhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols> product = lhs_eig * rhs;

    return Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>(product);
}

template<class T,
         CoordinateSystem rhs_System_To, CoordinateSystem rhs_System_From, typename rhs_Scalar, int rhs_Rows, int rhs_Cols>
Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols> operator*(const T& lhs,
                                                                                 const Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>& rhs)
{
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols>& rhs_eig = rhs;

    Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols> product = lhs * rhs_eig;

    return Matrix<rhs_System_To, rhs_System_From, rhs_Scalar, rhs_Rows, rhs_Cols>(product);
}

template<CoordinateSystem lhs_System_To, CoordinateSystem lhs_System_From, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         class T>
Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols> operator/(const Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>& lhs,
                                                                                 const T& rhs)
{
    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols>& lhs_eig = lhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols> division = lhs_eig / rhs;

    return Matrix<lhs_System_To, lhs_System_From, lhs_Scalar, lhs_Rows, lhs_Cols>(division);
}

} // namespace NAV
