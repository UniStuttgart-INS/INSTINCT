// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Search.hpp
/// @brief Ambiguity Search algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-13
/// @note Algorithm mostly taken from \cite deJonge1996 de Jonge 1996.
/// Matlab code from https://www.tudelft.nl/citg/over-faculteit/afdelingen/geoscience-remote-sensing/research/lambda/lambda was used to clarify points in literature.

#pragma once

#include <Eigen/Core>
#include <cmath>
#include "Navigation/Math/Sort.hpp"
#include <Eigen/src/Core/ArithmeticSequence.h>
#include <gcem.hpp>
#include "EllipsoidalRegion.hpp"
#include <limits>

namespace NAV::Ambiguity
{

/// @brief Searches for the integer ambiguities by integer rounding
/// @param[in] a Float ambiguity vector [cycles]
/// @return Integer ambiguity vector
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.2.2
template<typename Derived>
typename Derived::PlainObject integerSearchRounding(const Eigen::MatrixBase<Derived>& a)
{
    static_assert(Derived::ColsAtCompileTime == Eigen::Dynamic || Derived::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Parameter 'a' has to be a vector");

    return a.unaryExpr([](const double& x) { return std::round(x); });
}

/// @brief Searches for the integer ambiguities by integer bootstrapping
/// @param[in] a Decorrelated float ambiguity vector [cycles]
/// @param[in] Qa Variance/covariance matrix of the ambiguities
/// @return Integer ambiguity vector
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.2.3
template<typename DerivedA, typename DerivedQ>
typename DerivedA::PlainObject integerSearchBootstrapping(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Qa)
{
    static_assert(DerivedA::ColsAtCompileTime == Eigen::Dynamic || DerivedA::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Parameter 'a' has to be a vector");

    typename DerivedA::PlainObject a_fix = a;
    typename DerivedA::PlainObject a_float = a;

    for (Eigen::Index n = 0; n < a.rows(); n++)
    {
        for (Eigen::Index j = 0; j < n; j++)
        {
            a_float(n) -= Qa(n, j) / Qa(j, j) * (a_float(j) - a_fix(j));
        }

        a_fix(n) = std::round(a_float(n));
    }

    return a_fix;
}

/// @brief Performs an integer least-squares to search for integer candidates for the ambiguities
/// @param[in] a Decorrelated float ambiguity vector [cycles]
/// @param[in] Q Variance/covariance matrix of the ambiguities
/// @param[in] L_LTDL_Q Lower-triangular matrix from the L^T * D * L decomposition of Q
/// @param[in] D_LTDL_Q Diagonal entries from the L^T * D * L decomposition of Q
/// @param[in] numCandidates Requested number of candidates (default = 2)
/// @return Pair of: First a list of integer candidates (last column is the most likely), Vector with squared norm of the integer candidates
/// @note See \cite deJonge1996 de Jonge 1996, Algorithm FI71
template<typename DerivedA, typename DerivedQ, typename DerivedL, typename DerivedD>
std::pair<Eigen::MatrixXd, Eigen::VectorXd>
    integerLeastSquaresSearch(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Q,
                              const Eigen::MatrixBase<DerivedL>& L_LTDL_Q, const Eigen::MatrixBase<DerivedD>& D_LTDL_Q,
                              const Eigen::Index& numCandidates = 2)
{
    static_assert(DerivedA::ColsAtCompileTime == Eigen::Dynamic || DerivedA::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Parameter 'a' has to be a vector");
    static_assert(DerivedD::ColsAtCompileTime == Eigen::Dynamic || DerivedD::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(D_LTDL_Q.cols() == 1, "Parameter 'D_LTDL_Q' has to be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == D_LTDL_Q.rows(), "Parameter 'a' and 'D_LTDL_Q' must have same dimension");

    using Eigen::seq;

    auto n = a.rows(); // Amount of ambiguities
    if (n == 1) { return {}; }
    double chi2 = calcChi2(a, Q, L_LTDL_Q, D_LTDL_Q, numCandidates, 1.0); // Size of the ellipsoidal region
    LOG_DATA("chi2 = {}", chi2);

    // Q^-1 = L * D * L^T
    typename DerivedL::PlainObject L = L_LTDL_Q.inverse();
    typename DerivedD::PlainObject D = (1.0 / D_LTDL_Q.array()).matrix();
    LOG_DATA("L =\n{}", L);
    LOG_DATA("D = {}", D.transpose());

    Eigen::VectorXd right = Eigen::VectorXd::Zero(n + 1);
    right(n) = chi2;
    Eigen::VectorXd left = Eigen::VectorXd::Zero(n + 1);

    Eigen::VectorXd lef = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd end = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd disall = Eigen::VectorXd::Zero(numCandidates);
    Eigen::MatrixXd cands = Eigen::MatrixXd::Zero(n, numCandidates);
    double tmax = 0.0;
    Eigen::Index imax = 0;

    bool ende = false;
    Eigen::Index ncan = 0;

    Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
    dq.head(n - 1) = D(seq(1, n - 1)).array() / D(seq(0, n - 2)).array();
    dq(n - 1) = 1.0 / D(n - 1);
    LOG_DATA("dq = {}", dq.transpose());

    Eigen::Index i = n;
    Eigen::Index iold = i;

    auto BACKTS = [](const Eigen::Index& n, Eigen::Index& i, const auto& end, auto& dist, const auto& lef, auto& left, auto& ende) {
        for (i++; i < n; i++)
        {
            if (dist(i) <= end(i))
            {
                dist(i)++;
                left(i) = std::pow(dist(i) + lef(i), 2);
                break;
            }
            if (i == n - 1) { ende = true; }
        }
    };

    auto COLLECTs = [&chi2, &a]<typename DerD>(const Eigen::Index& n, const Eigen::Index& maxcan, const Eigen::MatrixBase<DerD>& D, const auto& lef, const auto& left, const auto& right,
                                               auto& dist, auto& end, Eigen::Index& ncan, auto& disall, auto& cands, double& tmax, Eigen::Index& imax) {
        auto STOREs = [&n, &a](const Eigen::Index& ican, const Eigen::Index& ipos, Eigen::Index& imax, const double& t, double& tmax, const auto& dist, auto& cands, auto& disall) {
            cands(seq(0, n - 1), ipos) = dist(seq(0, n - 1)) + a;
            LOG_DATA("      cands =\n{}", cands);
            disall(ipos) = t;
            LOG_DATA("      disall = {}", disall.transpose());
            tmax = t;
            imax = ipos;
            for (Eigen::Index i = 0; i < ican; i++)
            {
                if (disall(i) > tmax)
                {
                    imax = i;
                    tmax = disall(i);
                }
            }
        };

        double t = chi2 - (right(0) - left(0)) * D(0);
        LOG_DATA("    t = {}", t);
        end(0)++;
        LOG_DATA("    maxcan = {}", maxcan);
        LOG_DATA("    tmax = {}", tmax);
        LOG_DATA("    end(0)  = {}", end(0));
        LOG_DATA("    dist(0) = {}", dist(0));
        while (dist(0) <= end(0))
        {
            LOG_DATA("    ncan = {}", ncan);
            if (ncan < maxcan)
            {
                ncan++;
                STOREs(ncan, ncan - 1, imax, t, tmax, dist, cands, disall);
            }
            else if (t < tmax)
            {
                STOREs(maxcan, imax, imax, t, tmax, dist, cands, disall);
            }
            t += (2 * (dist(0) + lef(0)) + 1) * D(0);
            LOG_DATA("    t = {}", t);
            dist(0)++;
            LOG_DATA("    dist(0) = {}", dist(0));
        }
    };

    while (!ende)
    {
        i--;
        LOG_DATA("i = {}", i);

        if (iold <= i)
        {
            lef(i) += L(i + 1, i);
        }
        else
        {
            lef(i) = L(seq(i + 1, n - 1), i).dot(dist(seq(i + 1, n - 1)));
        }
        LOG_DATA("  lef = {}", lef.transpose());
        iold = i;
        right(i) = (right(i + 1) - left(i + 1)) * dq(i);
        LOG_DATA("  right = {}", right.transpose());
        auto reach = std::sqrt(right(i));
        LOG_DATA("  reach = {}", reach);
        auto delta = a(i) - reach - lef(i);
        LOG_DATA("  delta = {}", delta);
        dist(i) = std::ceil(delta) - a(i);
        LOG_DATA("  dist = {}", dist.transpose());
        if (dist(i) > reach - lef(i)) // nothing at this level -> backtrack
        {
            BACKTS(n, i, end, dist, lef, left, ende);
            continue;
        }
        // set the right border
        end(i) = reach - lef(i) - 1;
        LOG_DATA("  end = {}", end.transpose());
        left(i) = std::pow(dist(i) + lef(i), 2);
        LOG_DATA("  left = {}", left.transpose());

        if (i == 0)
        {
            COLLECTs(n, numCandidates, D, lef, left, right, dist, end, ncan, disall, cands, tmax, imax);
            BACKTS(n, i, end, dist, lef, left, ende);
        }
    }

    // Sort candidates by norm
    auto p = sort_permutation(disall, std::less<>{});
    apply_permutation_in_place(disall, p);
    apply_permutation_colwise_in_place(cands, p);

    return { cands.rightCols(ncan), disall.tail(ncan) };
}

/// @brief Performs an integer least-squares to search for integer candidates for the ambiguities using the search-and-shrink technique (MLAMBDA)
/// @param[in] zh Decorrelated float ambiguity vector [cycles]
/// @param[in] L_LTDL_Q Lower-triangular matrix from the L^T * D * L decomposition of Q
/// @param[in] D_LTDL_Q Diagonal entries from the L^T * D * L decomposition of Q
/// @param[in] numCandidates Requested number of candidates (default = 2)
/// @return Pair of: First a list of integer candidates (last column is the most likely), Vector with squared norm of the integer candidates
/// @note See \cite chang2005 Chang 2005, Argorithm 3.3
/// @note See \cite deJonge1996 de Jonge 1996, ch. 4.8
/// @note See ssearch.m file from TUDelft Matlab code
template<typename DerivedA, typename DerivedL, typename DerivedD>
std::pair<Eigen::MatrixXd, Eigen::VectorXd>
    integerLeastSquaresSearchAndShrink(const Eigen::MatrixBase<DerivedA>& zh, const Eigen::MatrixBase<DerivedL>& L_LTDL_Q,
                                       const Eigen::MatrixBase<DerivedD>& D_LTDL_Q, const Eigen::Index& numCandidates = 2)
{
    static_assert(DerivedA::ColsAtCompileTime == Eigen::Dynamic || DerivedA::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(zh.cols() == 1, "Parameter 'zh' has to be a vector");
    static_assert(DerivedD::ColsAtCompileTime == Eigen::Dynamic || DerivedD::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(D_LTDL_Q.cols() == 1, "Parameter 'D' has to be a vector");
    INS_ASSERT_USER_ERROR(zh.rows() == D_LTDL_Q.rows(), "Parameter 'zh' and 'D' must have same dimension");

    using Eigen::seq;

    auto n = zh.rows(); // Amount of ambiguities
    if (n == 1) { return {}; }

    auto sgn = [](auto x) {
        int sgn = gcem::sgn(x);
        return sgn ? sgn : 1;
    };

    Eigen::VectorXd dist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd zb = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd step = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);

    Eigen::MatrixXd cands = Eigen::MatrixXd::Zero(n, numCandidates);
    Eigen::VectorXd sqnorm = Eigen::VectorXd::Zero(numCandidates);

    auto maxDist = std::numeric_limits<double>::infinity(); // current chi^2
    auto k = n - 1;
    Eigen::Index count = 0; // the number of candidates

    zb(n - 1) = zh(n - 1);
    z(n - 1) = std::round(zb(n - 1));
    double y = zb(n - 1) - z(n - 1);
    step(n - 1) = sgn(y);
    auto imax = numCandidates - 1;
    for (size_t loop = 0; loop < 10000; loop++)
    {
        double newDist = dist(k) + std::pow(y, 2) / D_LTDL_Q(k);
        if (newDist < maxDist)
        {
            if (k != 0) // Case 1: move down
            {
                k--;
                dist(k) = newDist;
                S(k, seq(0, k)) = S(k + 1, seq(0, k)) + (z(k + 1) - zb(k + 1)) * L_LTDL_Q(k + 1, seq(0, k));
                zb(k) = zh(k) + S(k, k);
                z(k) = std::round(zb(k));
                y = zb(k) - z(k);
                step(k) = sgn(y);
            }
            else // Case 2: store the found candidate and try next valid integer
            {
                if (count < numCandidates) // store the first p − 1 initial points
                {
                    if (count == 0 || newDist > sqnorm(imax)) { imax = count; } // Addition from Rtklib
                    cands.col(count) = z;
                    sqnorm(count) = newDist;
                    count++;
                }
                else
                {
                    if (newDist < sqnorm(imax))
                    {
                        cands.col(imax) = z;
                        sqnorm(imax) = newDist;
                        sqnorm.maxCoeff(&imax);
                    }
                    maxDist = sqnorm(imax);
                }
                z(0) += step(0); // next valid integer
                y = zb(0) - z(0);
                step(0) = -step(0) - sgn(step(0));
            }
        }
        else // Case 3: exit or move up
        {
            if (k == n - 1) { break; }

            k++;             // move up
            z(k) += step(k); // next valid integer
            y = zb(k) - z(k);
            step(k) = -step(k) - sgn(step(k));
        }
    }

    // Sort candidates by norm
    auto p = sort_permutation(sqnorm, std::less<>{});
    apply_permutation_in_place(sqnorm, p);
    apply_permutation_colwise_in_place(cands, p);

    return { cands.rightCols(count), sqnorm.tail(count) };
}

} // namespace NAV::Ambiguity