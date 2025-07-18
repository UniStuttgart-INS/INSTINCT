// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Validate.hpp
/// @brief Ambiguity resolution validation algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-06

#pragma once

#include "Navigation/Math/Math.hpp"
#include "util/Assert.h"
#include "util/Eigen.hpp"
#include "util/Logger.hpp"

namespace NAV::Ambiguity
{

/// @brief Calculates the bootstrapped success rate
/// @param[in] D_LTDL_Q Diagonal entries from the L^T * D * L decomposition of Q
/// @return The bootstrapped success rate
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.2.4, eq. 23.28
template<typename Derived>
double successRateBootstrapping(const Eigen::MatrixBase<Derived>& D_LTDL_Q)
{
    INS_ASSERT_USER_ERROR(D_LTDL_Q.cols() == 1, "Parameter 'D_LTDL_Q' has to be a vector");

    double prod = 1.0;
    for (Eigen::Index i = 0; i < D_LTDL_Q.rows(); i++)
    {
        prod *= 2.0 * math::normalCDF(0.5 / std::sqrt(D_LTDL_Q(i))) - 1;
    }

    return prod;
}

/// @brief Difference test for the ambiguity resolution see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::DifferenceTest
/// @param[in] aFix1 Best integer ambiguities solution
/// @param[in] aFix2 Second best integer ambiguities solution
/// @param[in] aFloat Float ambiguities
/// @param[in] Qa Covariance matrix of the ambiguities
/// @param[in] criticalValue Critical value to check against (empirically determined value)
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite Verhagen2006 Verhagen 2006, eq. 31
template<typename DerivedAFix1, typename DerivedAFix2, typename DerivedAFloat, typename DerivedQ>
bool differenceTest(const Eigen::MatrixBase<DerivedAFix1>& aFix1, const Eigen::MatrixBase<DerivedAFix2>& aFix2,
                    const Eigen::MatrixBase<DerivedAFloat>& aFloat, const Eigen::MatrixBase<DerivedQ>& Qa, double criticalValue)
{
    static_assert(DerivedAFix1::ColsAtCompileTime == Eigen::Dynamic || DerivedAFix1::ColsAtCompileTime == 1);
    static_assert(DerivedAFix2::ColsAtCompileTime == Eigen::Dynamic || DerivedAFix2::ColsAtCompileTime == 1);
    static_assert(DerivedAFloat::ColsAtCompileTime == Eigen::Dynamic || DerivedAFloat::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(aFix1.cols() == 1, "Parameter 'aFix1' has to be a vector");
    INS_ASSERT_USER_ERROR(aFix2.cols() == 1, "Parameter 'aFix2' has to be a vector");
    INS_ASSERT_USER_ERROR(aFloat.cols() == 1, "Parameter 'aFloat' has to be a vector");
    INS_ASSERT_USER_ERROR(aFix1.rows() == aFix2.rows(), "Parameter 'aFix1' needs to have the same size as 'aFix2' and 'aFloat'");
    INS_ASSERT_USER_ERROR(aFix1.rows() == aFloat.rows(), "Parameter 'aFix1' needs to have the same size as 'aFix2' and 'aFloat'");

    return math::squaredNormVectorMatrix(aFloat - aFix2, Qa) - math::squaredNormVectorMatrix(aFloat - aFix1, Qa) >= criticalValue;
}

/// @brief Difference test for the ambiguity resolution see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::DifferenceTest
/// @param[in] sqNormFix1 Squared norm of the best integer ambiguities solution
/// @param[in] sqNormFix2 Squared norm of the second best integer ambiguities solution
/// @param[in] criticalValue Critical value to check against (empirically determined value)
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite Verhagen2006 Verhagen 2006, eq. 31
bool differenceTest(double sqNormFix1, double sqNormFix2, double criticalValue);

/// @brief Ratio test for the ambiguity resolution see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestCriticalValue
/// @param[in] aFix1 Best integer ambiguities solution
/// @param[in] aFix2 Second best integer ambiguities solution
/// @param[in] aFloat Float ambiguities
/// @param[in] Qa Covariance matrix of the ambiguities
/// @param[in] mu Critical value to check against, with range (0, 1]
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.6.4, eq. 23.78 or \cite Verhagen2006 Verhagen 2006, eq. 28, 29
template<typename DerivedAFix1, typename DerivedAFix2, typename DerivedAFloat, typename DerivedQ>
bool ratioTest(const Eigen::MatrixBase<DerivedAFix1>& aFix1, const Eigen::MatrixBase<DerivedAFix2>& aFix2,
               const Eigen::MatrixBase<DerivedAFloat>& aFloat, const Eigen::MatrixBase<DerivedQ>& Qa, double mu)
{
    static_assert(DerivedAFix1::ColsAtCompileTime == Eigen::Dynamic || DerivedAFix1::ColsAtCompileTime == 1);
    static_assert(DerivedAFix2::ColsAtCompileTime == Eigen::Dynamic || DerivedAFix2::ColsAtCompileTime == 1);
    static_assert(DerivedAFloat::ColsAtCompileTime == Eigen::Dynamic || DerivedAFloat::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(aFix1.cols() == 1, "Parameter 'aFix1' has to be a vector");
    INS_ASSERT_USER_ERROR(aFix2.cols() == 1, "Parameter 'aFix2' has to be a vector");
    INS_ASSERT_USER_ERROR(aFloat.cols() == 1, "Parameter 'aFloat' has to be a vector");
    INS_ASSERT_USER_ERROR(aFix1.rows() == aFix2.rows(), "Parameter 'aFix1' needs to have the same size as 'aFix2' and 'aFloat'");
    INS_ASSERT_USER_ERROR(aFix1.rows() == aFloat.rows(), "Parameter 'aFix1' needs to have the same size as 'aFix2' and 'aFloat'");

    return math::squaredNormVectorMatrix(aFloat - aFix1, Qa) / math::squaredNormVectorMatrix(aFloat - aFix2, Qa) <= mu;
}

/// @brief Ratio test for the ambiguity resolution see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestCriticalValue
/// @param[in] sqNormFix1 Squared norm of the best integer ambiguities solution
/// @param[in] sqNormFix2 Squared norm of the second best integer ambiguities solution
/// @param[in] mu Critical value to check against, with range (0, 1]
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.6.4, eq. 23.78 or \cite Verhagen2006 Verhagen 2006, eq. 28, 29
bool ratioTest(double sqNormFix1, double sqNormFix2, double mu);

/// @brief Look-up table for the critical value µ, depending on failure rate Pf_ILS
/// @param Pf Failure probability to not be higher
/// @param Pf_ILS Upper bound of the failure probability
/// @param nAmb Amount of ambiguities
/// @return Critical value µ
double criticalValueFailureRateLookup(double Pf, double Pf_ILS, size_t nAmb);

/// @brief Ratio test with a fixed-failure rate for ambiguity resolution, see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestFailureRate
/// @param[in] Pf Fixed-failure rate
/// @param[in] aFix1 Best integer ambiguities solution
/// @param[in] aFix2 Second best integer ambiguities solution
/// @param[in] aFloat Float ambiguities
/// @param[in] Qa Covariance matrix of the ambiguities
/// @param[in] D_LTDL_Q Diagonal entries from the L^T * D * L decomposition of Q
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite Verhagen2013 Verhagen 2013
template<typename DerivedAFix1, typename DerivedAFix2, typename DerivedAFloat, typename DerivedQ, typename DerivedD>
bool fixedFailureRateRatioTest(double Pf, const Eigen::MatrixBase<DerivedAFix1>& aFix1, const Eigen::MatrixBase<DerivedAFix2>& aFix2,
                               const Eigen::MatrixBase<DerivedAFloat>& aFloat, const Eigen::MatrixBase<DerivedQ>& Qa,
                               const Eigen::MatrixBase<DerivedD>& D_LTDL_Q)
{
    // Bootstrapped failure rate is an upper bound for the ILS failure rate
    double Pf_ILS = 1.0 - successRateBootstrapping(D_LTDL_Q);
    LOG_DATA("Pf_ILS = {}", Pf_ILS);

    // If Pf_ILS <= Pf, set µ equal to 1.0 which always accept the solution in the ratio test. So we can directly return true
    if (Pf_ILS <= Pf) { return true; }

    double mu = criticalValueFailureRateLookup(Pf, Pf_ILS, static_cast<size_t>(aFloat.rows()));

    return ratioTest(aFix1, aFix2, aFloat, Qa, mu);
}

/// @brief Ratio test with a fixed-failure rate for ambiguity resolution, see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestFailureRate
/// @param[in] Pf Fixed-failure rate
/// @param[in] sqNormFix1 Squared norm of the best integer ambiguities solution
/// @param[in] sqNormFix2 Squared norm of the second best integer ambiguities solution
/// @param[in] nAmb Number of ambiguities
/// @param[in] D_LTDL_Q Diagonal entries from the L^T * D * L decomposition of Q
/// @param[in] validateBootstrappedSuccessRate Whether to check the bootstrapped success rate before doing the ratio test
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite Verhagen2013 Verhagen 2013
template<typename DerivedD>
bool fixedFailureRateRatioTest(double Pf, double sqNormFix1, double sqNormFix2, size_t nAmb, const Eigen::MatrixBase<DerivedD>& D_LTDL_Q, bool validateBootstrappedSuccessRate = true)
{
    // Bootstrapped failure rate is an upper bound for the ILS failure rate
    double Pf_ILS = 1.0 - successRateBootstrapping(D_LTDL_Q);
    LOG_DATA("Pf_ILS = {}", Pf_ILS);

    // If Pf_ILS <= Pf, set µ equal to 1.0 which always accept the solution in the ratio test. So we can directly return true
    if (validateBootstrappedSuccessRate && Pf_ILS <= Pf) { return true; }

    double mu = criticalValueFailureRateLookup(Pf, Pf_ILS, nAmb);

    return ratioTest(sqNormFix1, sqNormFix2, mu);
}

/// @brief Projector test for the ambiguity resolution see NAV::AmbiguityResolutionParameters::ValidationAlgorithm::ProjectorTest
/// @param[in] aFix1 Best integer ambiguities solution
/// @param[in] aFix2 Second best integer ambiguities solution
/// @param[in] aFloat Float ambiguities
/// @param[in] Qa Covariance matrix of the ambiguities
/// @param[in] mu Critical value to check against, with range (0, 1]
/// @return True if the test passes and the best integer solution should be accepted
/// @note See \cite Verhagen2006 Verhagen 2006, eq. 35
template<typename DerivedAFix1, typename DerivedAFix2, typename DerivedAFloat, typename DerivedQ>
bool projectorTest(const Eigen::MatrixBase<DerivedAFix1>& aFix1, const Eigen::MatrixBase<DerivedAFix2>& aFix2,
                   const Eigen::MatrixBase<DerivedAFloat>& aFloat, const Eigen::MatrixBase<DerivedQ>& Qa, double mu)
{
    static_assert(DerivedAFix1::ColsAtCompileTime == Eigen::Dynamic || DerivedAFix1::ColsAtCompileTime == 1);
    static_assert(DerivedAFix2::ColsAtCompileTime == Eigen::Dynamic || DerivedAFix2::ColsAtCompileTime == 1);
    static_assert(DerivedAFloat::ColsAtCompileTime == Eigen::Dynamic || DerivedAFloat::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(aFix1.cols() == 1, "Parameter 'aFix1' has to be a vector");
    INS_ASSERT_USER_ERROR(aFix2.cols() == 1, "Parameter 'aFix2' has to be a vector");
    INS_ASSERT_USER_ERROR(aFloat.cols() == 1, "Parameter 'aFloat' has to be a vector");
    INS_ASSERT_USER_ERROR(aFix1.rows() == aFix2.rows(), "Parameter 'aFix1' needs to have the same size as 'aFix2' and 'aFloat'");
    INS_ASSERT_USER_ERROR(aFix1.rows() == aFloat.rows(), "Parameter 'aFix1' needs to have the same size as 'aFix2' and 'aFloat'");

    double proj = (aFix2 - aFix1).transpose() * Qa.inverse() * (aFloat - aFix1);
    return std::abs(proj / math::squaredNormVectorMatrix(aFix2 - aFix1, Qa)) <= mu;
}

} // namespace NAV::Ambiguity