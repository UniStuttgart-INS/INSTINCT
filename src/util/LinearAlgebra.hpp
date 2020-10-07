/// @file LinearAlgebra.hpp
/// @brief Vector space operations
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-24
/// @note To use Eigen directly again and ignore the frame checks, comment out the InsMatrix class and use this
///         template<CoordinateSystem _System, typename _Scalar, int _Rows, int _Cols,
///                 int _Options = StorageOptionsDefault, int _MaxRows = _Rows, int _MaxCols = _Cols>
///         using InsMatrix = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

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
/// Default Alignment options for the Eigen types
constexpr auto StorageOptionsDefault = (Eigen::StorageOptions::ColMajor | Eigen::StorageOptions::AutoAlign);

/// @brief Available Coordinate Systems
enum CoordinateSystem
{
    Inertial,   ///< Inertial frame (i-frame)
    Earth,      ///< Earth-fixed frame (e-frame)
    Navigation, ///< Local Navigation frame (n-frame)
    Body,       ///< Body frame (b-frame)
    Platform,   ///< Platform frame (p-frame)
    ECEF,       ///< Earth-centered-Earth-fixed frame [m]
    LLH,        ///< Latitude, Longitude, Height frame [rad, rad, m]
};

template<CoordinateSystem _System, typename _Scalar, int _Rows, int _Cols,
         int _Options = StorageOptionsDefault, int _MaxRows = _Rows, int _MaxCols = _Cols>
class InsMatrix : public Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
{
  public:
    /// @brief Default constructor
    InsMatrix() = default;
    /// @brief Destructor
    ~InsMatrix() = default;
    /// @brief Copy constructor
    InsMatrix(const InsMatrix&) = default;
    /// @brief Move constructor
    InsMatrix(InsMatrix&&) noexcept = default;
    /// @brief Copy assignment operator
    InsMatrix& operator=(const InsMatrix&) = default;
    /// @brief Move assignment operator
    InsMatrix& operator=(InsMatrix&&) noexcept = default;

    /// @brief Construct an object from an Eigen::Matrix
    /// @param[in] eigen The base object
    explicit InsMatrix(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& eigen)
        : Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>(eigen) {}

    /// @brief Coordinate System of the values
    /// @tparam rhs_System 
    /// @tparam rhs_Scalar Numeric type, e.g. float, double, int or std::complex<float>. User defined scalar types are supported as well 
    /// @tparam rhs_Rows Number of rows, or Dynamic
    /// @tparam rhs_Cols Number of columns, or Dynamic
    /// @tparam rhs_Options A combination of either RowMajor or ColMajor, and of either AutoAlign or DontAlign.
    ///                     The former controls storage order, and defaults to column-major.
    ///                     The latter controls alignment, which is required for vectorization.
    ///                     It defaults to aligning matrices except for fixed sizes that aren't a multiple of the packet size.
    /// @tparam rhs_MaxRows Maximum number of rows. Defaults to _Rows
    /// @tparam rhs_MaxCols Maximum number of columns. Defaults to _Cols
    /// @param[in] rhs The right-hand side objetc
    /// @return 
    template<CoordinateSystem rhs_System, typename rhs_Scalar, int rhs_Rows, int rhs_Cols,
             int rhs_Options = StorageOptionsDefault, int rhs_MaxRows = rhs_Rows, int rhs_MaxCols = rhs_Cols>
    InsMatrix<_System, _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>&
        operator+=(const InsMatrix<rhs_System, rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs)
    {
        static_assert(_System == rhs_System);

        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& _eig = *this;
        const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs_eig = rhs;

        _eig += rhs_eig;

        return *this;
    }

    template<CoordinateSystem rhs_System, typename rhs_Scalar, int rhs_Rows, int rhs_Cols,
             int rhs_Options = StorageOptionsDefault, int rhs_MaxRows = rhs_Rows, int rhs_MaxCols = rhs_Cols>
    InsMatrix<_System, _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>&
        operator-=(const InsMatrix<rhs_System, rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs)
    {
        static_assert(_System == rhs_System);

        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& _eig = *this;
        const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs_eig = rhs;

        _eig -= rhs_eig;

        return *this;
    }
};

template<CoordinateSystem _System, typename _Scalar>
using InsVector3 = InsMatrix<_System, _Scalar, 3, 1>;
template<CoordinateSystem _System, typename _Scalar>
using InsVector4 = InsMatrix<_System, _Scalar, 4, 1>;

template<CoordinateSystem _System, typename _Scalar>
using InsMatrix3 = InsMatrix<_System, _Scalar, 3, 3>;
template<CoordinateSystem _System, typename _Scalar>
using InsMatrix4 = InsMatrix<_System, _Scalar, 4, 4>;

// using Array3 = Eigen::Array3d;

template<CoordinateSystem _System>
using InsVector3d = InsVector3<_System, double>;
template<CoordinateSystem _System>
using InsVector4d = InsVector4<_System, double>;
template<CoordinateSystem _System>
using InsMatrix3d = InsMatrix3<_System, double>;
template<CoordinateSystem _System>
using InsMatrix4d = InsMatrix4<_System, double>;

// using Quaternion = Eigen::Quaterniond;

// using AngleAxis = Eigen::AngleAxisd;

template<CoordinateSystem lhs_System, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         int lhs_Options = StorageOptionsDefault, int lhs_MaxRows = lhs_Rows, int lhs_MaxCols = lhs_Cols,
         CoordinateSystem rhs_System, typename rhs_Scalar, int rhs_Rows, int rhs_Cols,
         int rhs_Options = StorageOptionsDefault, int rhs_MaxRows = rhs_Rows, int rhs_MaxCols = rhs_Cols>
InsMatrix<lhs_System, lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>
    operator+(const InsMatrix<lhs_System, lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>& lhs,
              const InsMatrix<rhs_System, rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs)
{
    static_assert(lhs_System == rhs_System);

    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols> sum = lhs_eig + rhs_eig;

    return InsMatrix<lhs_System, lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>(sum);
}

template<CoordinateSystem lhs_System, typename lhs_Scalar, int lhs_Rows, int lhs_Cols,
         int lhs_Options = StorageOptionsDefault, int lhs_MaxRows = lhs_Rows, int lhs_MaxCols = lhs_Cols,
         CoordinateSystem rhs_System, typename rhs_Scalar, int rhs_Rows, int rhs_Cols,
         int rhs_Options = StorageOptionsDefault, int rhs_MaxRows = rhs_Rows, int rhs_MaxCols = rhs_Cols>
InsMatrix<lhs_System, lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>
    operator-(const InsMatrix<lhs_System, lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>& lhs,
              const InsMatrix<rhs_System, rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs)
{
    static_assert(lhs_System == rhs_System);

    const Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>& lhs_eig = lhs;
    const Eigen::Matrix<rhs_Scalar, rhs_Rows, rhs_Cols, rhs_Options, rhs_MaxRows, rhs_MaxCols>& rhs_eig = rhs;

    Eigen::Matrix<lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols> difference = lhs_eig - rhs_eig;

    return InsMatrix<lhs_System, lhs_Scalar, lhs_Rows, lhs_Cols, lhs_Options, lhs_MaxRows, lhs_MaxCols>(difference);
}

} // namespace NAV
