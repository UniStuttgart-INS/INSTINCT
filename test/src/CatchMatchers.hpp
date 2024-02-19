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
#include "util/Container/STL.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Math/Math.hpp"

#include <fmt/ostream.h>
#include <iomanip>
#include <limits>
#include <algorithm>

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

// ###########################################################################################################

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

// ###########################################################################################################

/// @brief Creates a matcher that accepts numbers within certain range of target
/// @param[in] target Target value
/// @param[in] margin Accepted range around target
auto WithinAbs(long double target, long double margin) -> WithinAbsMatcher;

// ###########################################################################################################

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

// ###########################################################################################################

template<class Rep,
         class Period = std::ratio<1>>
struct WithinAbsMatcherInsTime : Catch::Matchers::MatcherGenericBase
{
    WithinAbsMatcherInsTime(const NAV::InsTime& target, const std::chrono::duration<Rep, Period>& margin)
        : m_target(target), m_margin(margin) {}

    bool match(const NAV::InsTime& matchee) const
    {
        return matchee + m_margin > m_target
               && m_target + m_margin > matchee;
    }

    std::string describe() const override
    {
        return "\n    is within " + fmt::format("{}", fmt::streamed(m_margin)) + " of \n" + Catch::Detail::stringify(m_target);
    }

  private:
    const NAV::InsTime& m_target;
    const std::chrono::duration<Rep, Period> m_margin;
};

/// @brief Creates a matcher that accepts Ranges within certain range of target
/// @tparam Rep An arithmetic type representing the number of ticks
/// @tparam Period A std::ratio representing the tick period (i.e. the number of second's fractions per tick)
/// @param[in] target Target values
/// @param[in] margin Accepted range around target
template<class Rep,
         class Period = std::ratio<1>>
auto WithinAbs(const NAV::InsTime& target, const std::chrono::duration<Rep, Period>& margin) -> WithinAbsMatcherInsTime<Rep, Period>
{
    return WithinAbsMatcherInsTime<Rep, Period>{ target, margin };
}

// ###########################################################################################################

template<typename Scalar>
struct EqualsSigDigitsMatcher : Catch::Matchers::MatcherGenericBase
{
    EqualsSigDigitsMatcher(const Scalar& target, size_t digits)
        : m_target(fmt::format("{:.{p}e}", target, fmt::arg("p", digits > 0 ? digits - 1 : digits))), m_digits(digits) {}

    bool match(const Scalar& matchee) const
    {
        m_matchee = fmt::format("{:.{p}e}", matchee, fmt::arg("p", m_digits > 0 ? m_digits - 1 : m_digits));
        return m_target == m_matchee;
    }

    std::string describe() const override
    {
        return fmt::format("({}) is within {} sig digits of {}", m_matchee, m_digits, m_target);
    }

  private:
    const std::string m_target;
    mutable std::string m_matchee;
    const size_t m_digits;
};

/// @brief Creates a matcher that rounds to significant digits and compares for equality
/// @param[in] target Target value
/// @param[in] digits Significant digits to round to
template<typename Scalar>
auto EqualsSigDigits(const Scalar& target, size_t digits) -> EqualsSigDigitsMatcher<Scalar>
{
    return EqualsSigDigitsMatcher<Scalar>{ target, digits };
}

// ###########################################################################################################

template<typename T>
struct EqualsSigDigitsMatcherContainer : Catch::Matchers::MatcherGenericBase
{
    EqualsSigDigitsMatcherContainer(const T& target, size_t digits)
        : m_digits(digits)
    {
        std::transform(target.cbegin(), target.cend(), std::back_inserter(m_target), [&](const auto& element) {
            return fmt::format("{:.{p}e}", element, fmt::arg("p", m_digits > 0 ? m_digits - 1 : m_digits));
        });
    }

    bool match(const T& matchee) const
    {
        std::transform(matchee.cbegin(), matchee.cend(), std::back_inserter(m_matchee), [&](const auto& element) {
            return fmt::format("{:.{p}e}", element, fmt::arg("p", m_digits > 0 ? m_digits - 1 : m_digits));
        });
        return m_target == m_matchee;
    }

    std::string describe() const override
    {
        return fmt::format("\n  ({})\n    is within {} significant digits of\n  ({})",
                           NAV::joinToString(m_matchee),
                           m_digits,
                           NAV::joinToString(m_target));
    }

  private:
    std::vector<std::string> m_target;
    mutable std::vector<std::string> m_matchee;
    const size_t m_digits;
};

/// @brief Creates a matcher that rounds to significant digits and compares for equality
/// @param[in] target Target value
/// @param[in] digits Significant digits to round to
template<typename T>
auto EqualsSigDigitsContainer(const T& target, size_t digits) -> EqualsSigDigitsMatcherContainer<T>
{
    return EqualsSigDigitsMatcherContainer<T>{ target, digits };
}

} // namespace Catch::Matchers
