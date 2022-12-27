// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CatchMatchers.hpp
/// @brief Catch2 Testing operations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-11-13

#pragma once

#include <catch2/matchers/catch_matchers_templated.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "util/Eigen.hpp"

#include <iomanip>
#include <limits>

namespace NAV::TESTS
{

/// Numeric precision for doubles
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();
/// Numeric precision for floats
constexpr double EPSILON_FLOAT = 10.0 * std::numeric_limits<float>::epsilon();
/// Numeric precision for doubles
constexpr long double EPSILON_LDOUBLE = 10.0 * std::numeric_limits<long double>::epsilon();

} // namespace NAV::TESTS

namespace Catch::Matchers
{

template<typename Derived>
struct WithinAbsMatcherEigen : Catch::Matchers::MatcherGenericBase
{
    WithinAbsMatcherEigen(const Eigen::DenseBase<Derived>& target, double margin)
        : m_target(target), m_margin(margin) {}

    template<typename OtherDerived>
    bool match(const Eigen::DenseBase<OtherDerived>& matchee) const
    {
        if (matchee.rows() != m_target.rows() || matchee.cols() != m_target.cols()) { return false; }

        for (Eigen::Index row = 0; row < m_target.rows(); ++row)
        {
            for (Eigen::Index col = 0; col < m_target.cols(); ++col)
            {
                if (matchee(row, col) + m_margin < m_target(row, col)
                    || m_target(row, col) + m_margin < matchee(row, col))
                {
                    return false;
                }
            }
        }
        return true;
    }

    std::string describe() const override
    {
        return "\n    is within " + fmt::format("{}", m_margin) + " of \n" + Catch::Detail::stringify(m_target);
    }

  private:
    const Eigen::DenseBase<Derived>& m_target;
    const double m_margin;
};

/// @brief Creates a matcher that accepts Eigen matrices within certain range of target
/// @tparam Derived Eigen derived type
/// @param[in] target Target value
/// @param[in] margin Accepted range around target
template<typename Derived>
auto WithinAbs(const Eigen::DenseBase<Derived>& target, double margin) -> WithinAbsMatcherEigen<Derived>
{
    return WithinAbsMatcherEigen<Derived>{ target, margin };
}

template<typename Derived>
struct WithinAbsMatcherEigenQuaternion : Catch::Matchers::MatcherGenericBase
{
    WithinAbsMatcherEigenQuaternion(const Eigen::QuaternionBase<Derived>& target, double margin)
        : m_target(target), m_margin(margin) {}

    template<typename OtherDerived>
    bool match(const Eigen::QuaternionBase<OtherDerived>& matchee) const
    {
        return matchee.isApprox(m_target, m_margin);
    }

    std::string describe() const override
    {
        return "\n    is within " + fmt::format("{}", m_margin) + " of \n" + Catch::Detail::stringify(m_target);
    }

  private:
    const Eigen::QuaternionBase<Derived>& m_target;
    const double m_margin;
};

/// @brief Creates a matcher that accepts Eigen quaternions within certain range of target
/// @tparam Derived Eigen derived type
/// @param[in] target Target value
/// @param[in] margin Accepted range around target
template<typename Derived>
auto WithinAbs(const Eigen::QuaternionBase<Derived>& target, double margin) -> WithinAbsMatcherEigenQuaternion<Derived>
{
    return WithinAbsMatcherEigenQuaternion<Derived>{ target, margin };
}

/// @brief Creates a matcher that accepts numbers within certain range of target
/// @param[in] target Target value
/// @param[in] margin Accepted range around target
auto WithinAbs(long double target, long double margin) -> WithinAbsMatcher;

template<size_t N>
struct WithinAbsMatcherArray : Catch::Matchers::MatcherGenericBase
{
    WithinAbsMatcherArray(const std::array<double, N>& target, double margin)
        : m_target(target), m_margin(margin) {}

    template<typename Other>
    bool match(const Other& matchee) const
    {
        return std::equal(std::begin(m_target), std::end(m_target),
                          std::begin(matchee), std::end(matchee),
                          [](double lhs, double rhs) { return lhs == rhs; });
    }

    std::string describe() const override
    {
        return "\n    is within " + fmt::format("{}", m_margin) + " of " + Catch::rangeToString(m_target);
    }

  private:
    const std::array<double, N>& m_target;
    const double m_margin;
};

/// @brief Creates a matcher that accepts Ranges within certain range of target
/// @tparam N Amount of elements in the array
/// @param[in] target Target values
/// @param[in] margin Accepted range around target
template<size_t N>
auto WithinAbs(const std::array<double, N>& target, double margin) -> WithinAbsMatcherArray<N>
{
    return WithinAbsMatcherArray<N>{ target, margin };
}

} // namespace Catch::Matchers
