// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Math.hpp
/// @brief Simple Math functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author N. Stahl (HiWi: Elliptical integral)
/// @date 2023-07-04

#pragma once

#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <gcem.hpp>
#include <fmt/format.h>

#include "util/Assert.h"

namespace NAV::math
{

/// @brief Calculates the factorial of an unsigned integer
/// @param[in] n Unsigned integer
/// @return The factorial of 'n'
uint64_t factorial(uint64_t n);

/// @brief Round the number to the specified amount of digits
/// @param[in] value Value to round
/// @param[in] digits Amount of digits
/// @return The rounded value
template<std::floating_point T>
constexpr T round(const T& value, size_t digits)
{
    auto factor = std::pow(10, digits);
    return std::round(value * factor) / factor;
}

/// @brief Round the number to the specified amount of significant digits
/// @param[in] value Value to round
/// @param[in] digits Amount of digits
/// @return The rounded value
template<std::floating_point T>
constexpr T roundSignificantDigits(T value, size_t digits)
{
    if (value == 0.0) { return 0.0; }
    // LOG_DEBUG("value = {:.13e} --> Round to {} digits", value, digits);
    auto absVal = gcem::abs(value);
    auto log10 = static_cast<int32_t>(gcem::log10(absVal));
    auto exp = log10 + (log10 > 0 || (log10 == 0 && absVal >= 1.0));
    auto fac = static_cast<T>(digits) - static_cast<T>(exp);
    // LOG_DEBUG("  log10  = {}, exp = {}, fac = {}", log10, exp, fac);
    auto factor = static_cast<T>(gcem::pow(10.0, fac));
    // LOG_DEBUG("  factor = {:.0e} --> value * factor = {}", factor, value * factor);
    // LOG_DEBUG("  round = {} --> ... / factor = {}", gcem::round(value * factor), gcem::round(value * factor) / factor);
    return static_cast<T>(gcem::round(value * factor) / factor);
}

/// @brief Interprets the input integer with certain amount of Bits as Output type. Takes care of sign extension
/// @tparam Out Output type
/// @tparam Bits Size of the input data
/// @tparam In Input data type (needs to be bigger than the amount of Bits)
/// @param[in] in Number as two's complement, with the sign bit (+ or -) occupying the MSB
/// @return Output type
template<std::integral Out, size_t Bits, std::integral In>
constexpr Out interpretAs(In in)
{
    static_assert(Bits < sizeof(In) * 8);
    static_assert(Bits < sizeof(Out) * 8);

    constexpr size_t N = sizeof(Out) * 8 - Bits;
    return static_cast<Out>(static_cast<Out>((in & static_cast<In>(gcem::pow(2, Bits) - 1)) << N) >> N);
}

/// @brief Calculates the skew symmetric matrix of the given vector.
///        This is needed to perform the cross product with a scalar product operation
/// @tparam Derived Derived Eigen Type
/// @param[in] a The vector
/// @return Skew symmetric matrix
/// @note See Groves (2013) equation (2.50)
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetricMatrix(const Eigen::MatrixBase<Derived>& a)
{
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMat;
    skewMat << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;

    return skewMat;
}

/// @brief Calculates the square of a skew symmetric matrix of the given vector.
/// @tparam Derived Derived Eigen Type
/// @param[in] a The vector
/// @return Square of skew symmetric matrix
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetricMatrixSquared(const Eigen::MatrixBase<Derived>& a)
{
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMat2;
    skewMat2 << std::pow(a(2), 2) + std::pow(a(1), 2), a(0) * a(1), a(0) * a(2),
        a(0) * a(1), std::pow(a(2), 2) + std::pow(a(0), 2), a(1) * a(2),
        a(0) * a(2), a(1) * a(2), std::pow(a(0), 2) + std::pow(a(1), 2);

    return skewMat2;
}

/// @brief Calculates the matrix exponential map of the given vector.
/// @tparam Derived Derived Eigen Type
/// @param[in] v The vector
/// @return The matrix exponential map
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> expMapMatrix(const Eigen::MatrixBase<Derived>& v)
{
    INS_ASSERT_USER_ERROR(v.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(v.rows() == 3, "Given Vector must have 3 Rows");

    return math::skewSymmetricMatrix(v).exp();
}

/// @brief Calculates the quaternionic exponential map of the given vector.
/// @tparam Derived Derived Eigen Type
/// @param[in] v The vector
/// @return The quaternionic exponential map
template<typename Derived>
Eigen::Quaternion<typename Derived::Scalar> expMapQuat(const Eigen::MatrixBase<Derived>& v)
{
    INS_ASSERT_USER_ERROR(v.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(v.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Vector3<typename Derived::Scalar> omega = 0.5 * v;
    auto omegaNorm = omega.norm();
    if (omegaNorm < 1e-9) { return Eigen::Quaternion<typename Derived::Scalar>::Identity(); }
    Eigen::Vector3<typename Derived::Scalar> quatVec = omega / omegaNorm * std::sin(omegaNorm);

    return { std::cos(omegaNorm), quatVec.x(), quatVec.y(), quatVec.z() };
}

/// @brief Calculates the right Jacobian of SO(3) which relates additive increments in the tangent space to multiplicative increments applied on the right-hand side
/// @param[in] phi Vector applied on the right side
/// @return Right Jacobian J_r
template<typename Derived>
[[nodiscard]] Eigen::Matrix3<typename Derived::Scalar> J_r(const Eigen::MatrixBase<Derived>& phi)
{
    INS_ASSERT_USER_ERROR(phi.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(phi.rows() == 3, "Given Vector must have 3 Rows");

    auto phiNorm = phi.norm();
    auto phiNorm2 = phiNorm * phiNorm;
    auto phiNorm3 = phiNorm2 * phiNorm;
    return Eigen::Matrix3<typename Derived::Scalar>::Identity()
           - (1.0 - std::cos(phiNorm)) / phiNorm2 * skewSymmetricMatrix(phi)
           + (phiNorm - std::sin(phiNorm)) / phiNorm3 * skewSymmetricMatrixSquared(phi);
}

/// @brief Calculates the secant of a value (sec(x) = csc(pi/2 - x) = 1 / cos(x))
template<std::floating_point T>
T sec(const T& x)
{
    return 1.0 / std::cos(x);
}

/// @brief Calculates the cosecant of a value (csc(x) = sec(pi/2 - x) = 1 / sin(x))
template<std::floating_point T>
T csc(const T& x)
{
    return 1.0 / std::sin(x);
}

/// @brief Returns the sign of the given value
/// @param[in] val Value to get the sign from
/// @return Sign of the given value
template<typename T>
int sgn(const T& val)
{
    return (T(0) < val) - (val < T(0));
}

/// @brief Calculates the state transition matrix 𝚽 limited to specified order in 𝐅𝜏ₛ
/// @param[in] X Matrix
/// @param[in] order The order of the Taylor polynom to calculate
/// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.34, p. 98
template<typename Derived>
typename Derived::PlainObject expm(const Eigen::MatrixBase<Derived>& X, size_t order)
{
    INS_ASSERT_USER_ERROR(X.rows() == X.cols(), "Matrix exponential calculation only possible for n x n matrices");

    typename Derived::PlainObject e_X;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic)
    {
        e_X = Eigen::MatrixBase<Derived>::Identity(X.rows(), X.cols());
    }
    else
    {
        e_X = Eigen::MatrixBase<Derived>::Identity();
    }
    typename Derived::PlainObject Xpower = X;
    for (size_t i = 1; i <= order; i++)
    {
        e_X += Xpower / static_cast<double>(math::factorial(i));

        if (i < order)
        {
            Xpower *= X;
        }
    }

    return e_X;
}

/// @brief Find (L^T D L)-decomposition of Q-matrix via outer product method
/// @param[in] Qmatrix Symmetric positive definite matrix to be factored
/// @return L - Factor matrix (strict lower triangular)
/// @return D - Vector with entries of the diagonal matrix
/// @note See \cite deJonge1996 de Jonge 1996, Algorithm FMFAC5
/// @attention Consider using NAV::math::LtDLdecomp_choleskyFact() because it is faster by up to a factor 10
template<typename Derived>
std::optional<std::pair<Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
                        Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime>>>
    LtDLdecomp_outerProduct(const Eigen::MatrixBase<Derived>& Qmatrix)
{
    using Eigen::seq;

    auto n = Qmatrix.rows();
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> Q = Qmatrix.template triangularView<Eigen::Lower>();
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> L;
    Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime> D;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic)
    {
        L = Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
        D.setZero(n);
    }
    else
    {
        L = Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>::Zero();
        D.setZero();
    }

    for (Eigen::Index i = n - 1; i >= 0; i--)
    {
        D(i) = Q(i, i);
        if (Q(i, i) <= 0.0) { return {}; }
        L(i, seq(0, i)) = Q(i, seq(0, i)) / std::sqrt(Q(i, i));

        for (Eigen::Index j = 0; j <= i - 1; j++)
        {
            Q(j, seq(0, j)) -= L(i, seq(0, j)) * L(i, j);
        }
        L(i, seq(0, i)) /= L(i, i);
    }

    return std::make_pair(L, D);
}

/// @brief Find (L^T D L)-decomposition of Q-matrix via a backward Cholesky factorization in a bordering method formulation
/// @param[in] Q Symmetric positive definite matrix to be factored
/// @return L - Factor matrix (strict lower triangular)
/// @return D - Vector with entries of the diagonal matrix
/// @note See \cite deJonge1996 de Jonge 1996, Algorithm FMFAC6
template<typename Derived>
std::optional<std::pair<Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
                        Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime>>>
    LtDLdecomp_choleskyFact(const Eigen::MatrixBase<Derived>& Q)
{
    using Eigen::seq;

    auto n = Q.rows();
    typename Derived::PlainObject L = Q.template triangularView<Eigen::Lower>();
    Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime> D;
    double cmin = 1;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic) { D.setZero(n); }
    else { D.setZero(); }

    for (Eigen::Index j = n - 1; j >= 0; j--)
    {
        for (Eigen::Index i = n - 1; i >= j + 1; i--)
        {
            L(i, j) = (L(i, j) - L(seq(i + 1, n - 1), j).dot(L(seq(i + 1, n - 1), i))) / L(i, i);
        }
        double t = L(j, j) - L(seq(j + 1, n - 1), j).dot(L(seq(j + 1, n - 1), j));
        if (t <= 0.0) { return {}; }
        double c = t / L(j, j);
        cmin = std::min(c, cmin);
        L(j, j) = std::sqrt(t);
    }
    for (Eigen::Index i = 0; i < n; i++)
    {
        L.row(i).leftCols(i) /= L(i, i);
        D(i) = std::pow(L(i, i), 2.0);
        L(i, i) = 1;
    }

    return std::make_pair(L, D);
}

/// @brief Calculates the squared norm of the vector and matrix
///
/// \anchor eq-squaredNorm \f{equation}{ \label{eq:eq-squaredNorm}
/// ||\mathbf{\dots}||^2_{\mathbf{Q}} = (\dots)^T \mathbf{Q}^{-1} (\dots)
/// \f}
/// @param a Vector
/// @param Q Covariance matrix of the vector
/// @return Squared norm
template<typename DerivedA, typename DerivedQ>
typename DerivedA::Scalar squaredNormVectorMatrix(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Q)
{
    static_assert(DerivedA::ColsAtCompileTime == Eigen::Dynamic || DerivedA::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Parameter 'a' has to be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == Q.rows(), "Parameter 'a' and 'Q' need to have same size");
    INS_ASSERT_USER_ERROR(Q.cols() == Q.rows(), "Parameter 'Q' needs to be quadratic");

    return a.transpose() * Q.inverse() * a;
}

/// @brief Calculates the cumulative distribution function (CDF) of the standard normal distribution
///
/// \anchor eq-normalDistCDF \f{equation}{ \label{eq:eq-normalDistCDF}
///  \Phi(x) = \int\displaylimits_{-\infty}^x \frac{1}{\sqrt{2\pi}} \exp{\left(-\frac{1}{2} v^2\right)} \text{d}v
/// \f}
/// which can be expressed with the error function
/// \anchor eq-normalDistCDF-erf \f{equation}{ \label{eq:eq-normalDistCDF-erf}
///  \Phi(x) = \frac{1}{2} \left[ 1 + \text{erf}{\left(\frac{x}{\sqrt{2}}\right)} \right]
/// \f}
/// Using the property
/// \anchor eq-erf-minus \f{equation}{ \label{eq:eq-erf-minus}
///  \text{erf}{\left( -x \right)} = -\text{erf}{\left( x \right)}
/// \f}
/// and the complementary error function
/// \anchor eq-erfc \f{equation}{ \label{eq:eq-erfc}
///  \text{erfc}{\left( x \right)} = 1 - \text{erf}{\left( x \right)}
/// \f}
/// we can simplify equation \eqref{eq-normalDistCDF-erf} to
/// \anchor eq-normalDistCDF-erfc \f{equation}{ \label{eq:eq-normalDistCDF-erfc}
/// \begin{aligned}
///  \Phi(x) &= \frac{1}{2} \left[ 1 - \text{erf}{\left(-\frac{x}{\sqrt{2}}\right)} \right] \\
///          &= \frac{1}{2} \text{erfc}{\left(-\frac{x}{\sqrt{2}}\right)}
/// \end{aligned}
/// \f}
///
/// @param value Value to calculate the CDF for
double normalCDF(double value);

/// @brief Returns the inverse square root of a matrix
/// @param matrix Matrix to use
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject inverseSqrt(const Eigen::MatrixBase<Derived>& matrix)
{
    INS_ASSERT_USER_ERROR(matrix.rows() == matrix.cols(), "Only square matrix supported");
    if constexpr (std::is_floating_point_v<typename Derived::Scalar>)
    {
        return matrix.inverse().sqrt(); // Eigen::SelfAdjointEigenSolver<Eigen::MatrixX<T>>{ covMatrix }.operatorInverseSqrt();
    }
    else // Eigen gets problems with ceres::Jet in the .sqrt() function
    {
        Eigen::JacobiSVD<Eigen::MatrixX<typename Derived::Scalar>> svd(matrix.inverse(), Eigen::ComputeFullV);
        typename Derived::PlainObject sqrtInverse = svd.matrixV() * svd.singularValues().cwiseSqrt().asDiagonal() * svd.matrixV().transpose();
        INS_ASSERT_USER_ERROR(!sqrtInverse.hasNaN(), "The matrix is not invertible");
        return sqrtInverse;
    }
}

/// @brief Change the sign of x according to the value of y
/// @param[in] x input value
/// @param[in] y input value
/// @return -x or +x
template<typename T>
T sign(const T& x, const T& y)
{
    // similar function 'sign' in fortran
    if (y >= 0.0)
    {
        return fabs(x);
    }
    return -1.0 * fabs(x);
}

/// @brief Linear interpolation between vectors
/// @param a Left value
/// @param b Right value
/// @param t Multiplier. [0, 1] for interpolation
/// @return a + t * (b - a)
template<typename Derived>
typename Derived::PlainObject lerp(const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& b, auto t)
{
    return a + t * (b - a);
}

/// Lerp Search Result
struct LerpSearchResult
{
    size_t l; ///< Lower bound index
    size_t u; ///< Upper bound index (l + 1)
    double t; ///< [0, 1] for Interpolation, otherwise Extrapolation
};

/// @brief Searches the value in the data container
/// @param[in] data Data container
/// @param[in] value Value to search
LerpSearchResult lerpSearch(const auto& data, const auto& value)
{
    auto i = static_cast<size_t>(std::distance(data.begin(), std::upper_bound(data.begin(), data.end(), value)));
    if (i > 0) { i--; }
    if (i == data.size() - 1) { i--; }
    const auto& lb = data.at(i);
    const auto& ub = data.at(i + 1);
    double t = (value - lb) / (ub - lb);

    return { .l = i, .u = i + 1, .t = t };
}

/// @brief Bilinear interpolation
/// @param[in] tx Distance in x component to interpolate [0, 1]
/// @param[in] ty Distance in y component to interpolate [0, 1]
/// @param[in] c00 Value for tx = ty = 0
/// @param[in] c10 Value for tx = 1 and ty = 0
/// @param[in] c01 Value for tx = 0 and ty = 1
/// @param[in] c11 Value for tx = ty = 1
///
/// c01 ------ c11
///  |          |
///  |          |
///  |          |
/// c00 ------ c10
///
/// @note See https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/interpolation/bilinear-filtering.html
auto bilinearInterpolation(const auto& tx, const auto& ty,
                           const auto& c00, const auto& c10,
                           const auto& c01, const auto& c11)
{
    auto a = c00 * (1.0 - tx) + c10 * tx;
    auto b = c01 * (1.0 - tx) + c11 * tx;
    return a * (1.0 - ty) + b * ty;
    // Alternative implementation:
    // return (1.0 - tx) * (1.0 - ty) * c00 + tx * (1.0 - ty) * c10 + (1.0 - tx) * ty * c01 + tx * ty * c11;
}

/// @brief Calculates the incomplete elliptical integral of the second kind
/// @param[in] phi Interval bound the integration uses from 0 to phi
/// @param[in] m Function parameter that is integrated 1-m*sin(t)^2
/// @return Incomplete elliptical integral of the second kind
/// @note See http://www2.iap.fr/users/pichon/doc/html_xref/elliptic-es.html
double calcEllipticalIntegral(double phi, double m);

} // namespace NAV::math
