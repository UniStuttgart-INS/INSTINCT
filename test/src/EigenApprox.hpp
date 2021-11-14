#include "util/Eigen.hpp"

#include <iomanip>

template<typename _Scalar, int _Rows, int _Cols>
class EigApprox : public Catch::Detail::Approx
{
  public:
    explicit EigApprox(Eigen::Matrix<_Scalar, _Rows, _Cols> matrix) : Approx(static_cast<_Scalar>(0.0)), _matrix(std::move(matrix)) {}

    friend bool operator==(const Eigen::Matrix<_Scalar, _Rows, _Cols>& lhs, const EigApprox& rhs)
    {
        bool result = true;
        for (Eigen::Index row = 0; row < lhs.rows(); ++row)
        {
            for (Eigen::Index col = 0; col < lhs.cols(); ++col)
            {
                result &= (lhs(row, col) == Approx(rhs._matrix(row, col)).margin(rhs.m_margin).epsilon(rhs.m_epsilon).scale(rhs.m_scale));
            }
        }
        return result;
    }
    [[nodiscard]] const Eigen::Matrix<_Scalar, _Rows, _Cols>& Matrix() const { return _matrix; }

    template<typename T, typename = typename std::enable_if<std::is_constructible<double, T>::value>::type>
    EigApprox& margin(T const& newMargin)
    {
        Approx::margin(newMargin);
        m_margin = static_cast<double>(newMargin);
        return *this;
    }

    template<typename T, typename = typename std::enable_if<std::is_constructible<double, T>::value>::type>
    EigApprox& epsilon(T const& newEpsilon)
    {
        Approx::epsilon(newEpsilon);
        m_epsilon = static_cast<double>(newEpsilon);
        return *this;
    }

    template<typename T, typename = typename std::enable_if<std::is_constructible<double, T>::value>::type>
    EigApprox& scale(T const& newScale)
    {
        Approx::scale(newScale);
        m_scale = static_cast<double>(newScale);
        return *this;
    }

  private:
    Eigen::Matrix<_Scalar, _Rows, _Cols> _matrix;

    double m_epsilon{ std::numeric_limits<float>::epsilon() };
    double m_margin{ 0.0 };
    double m_scale{ 0.0 };
};

template<typename _Scalar>
class EigApproxQ : public Catch::Detail::Approx
{
  public:
    explicit EigApproxQ(Eigen::Quaternion<_Scalar> quat) : Approx(static_cast<_Scalar>(0.0)), _quat(std::move(quat)) {}

    friend bool operator==(const Eigen::Quaternion<_Scalar>& lhs, const EigApproxQ& rhs)
    {
        bool result = true;
        for (Eigen::Index row = 0; row < 4; ++row)
        {
            result &= (lhs.coeffs()(row) == Approx(rhs._quat.coeffs()(row)).margin(rhs.m_margin).epsilon(rhs.m_epsilon).scale(rhs.m_scale));
        }
        return result;
    }
    [[nodiscard]] const Eigen::Quaternion<_Scalar>& Quat() const { return _quat; }

    template<typename T, typename = typename std::enable_if<std::is_constructible<double, T>::value>::type>
    EigApproxQ& margin(T const& newMargin)
    {
        Approx::margin(newMargin);
        m_margin = static_cast<double>(newMargin);
        return *this;
    }

    template<typename T, typename = typename std::enable_if<std::is_constructible<double, T>::value>::type>
    EigApproxQ& epsilon(T const& newEpsilon)
    {
        Approx::epsilon(newEpsilon);
        m_epsilon = static_cast<double>(newEpsilon);
        return *this;
    }

    template<typename T, typename = typename std::enable_if<std::is_constructible<double, T>::value>::type>
    EigApproxQ& scale(T const& newScale)
    {
        Approx::scale(newScale);
        m_scale = static_cast<double>(newScale);
        return *this;
    }

  private:
    Eigen::Quaternion<_Scalar> _quat;

    double m_epsilon{ std::numeric_limits<float>::epsilon() };
    double m_margin{ 0.0 };
    double m_scale{ 0.0 };
};

namespace Catch
{
template<typename _Scalar, int _Rows, int _Cols>
struct StringMaker<Eigen::Matrix<_Scalar, _Rows, _Cols>>
{
    static std::string convert(const Eigen::Matrix<_Scalar, _Rows, _Cols>& matrix)
    {
        std::ostringstream sstr;
        sstr << std::setprecision(14);
        sstr << matrix;
        return sstr.str();
    }
};

template<typename _Scalar, int _Rows, int _Cols>
struct StringMaker<::EigApprox<_Scalar, _Rows, _Cols>>
{
    static std::string convert(const ::EigApprox<_Scalar, _Rows, _Cols>& approx_matrix)
    {
        return StringMaker<Eigen::Matrix<_Scalar, _Rows, _Cols>>::convert(approx_matrix.Matrix());
    }
};

template<typename _Scalar>
struct StringMaker<Eigen::Quaternion<_Scalar>>
{
    static std::string convert(const Eigen::Quaternion<_Scalar>& quat)
    {
        std::ostringstream sstr;
        sstr << std::setprecision(14);
        sstr << quat;
        return sstr.str();
    }
};

template<typename _Scalar>
struct StringMaker<::EigApproxQ<_Scalar>>
{
    static std::string convert(const ::EigApproxQ<_Scalar>& approx_quat)
    {
        return StringMaker<Eigen::Quaternion<_Scalar>>::convert(approx_quat.Quat());
    }
};
} // namespace Catch