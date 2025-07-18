// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EllipsoidalRegion.hpp
/// @brief Calculate the ellipsoidal region Chi^2
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-29

#pragma once

#include <cmath>
#include <Eigen/Core>
#include <gcem.hpp>

namespace NAV::Ambiguity
{

namespace internal
{

/// @brief Calculates \f$ \chi^2 \f$, the size of the ellipsoidal region, via volume of the ellipsoidal region
///
/// The volume, expressed in \f$ [\text{cycles}^n] \f$, of the ellipsoidal region is (\cite deJonge1996 de Jonge 1996, ch. 4.9, eq. 4.19)
/// \anchor eq-GNSS-chi2-volume \f{equation}{ \label{eq:eq-GNSS-chi2-volume}
///   E_n = \chi^n \sqrt{|Q_{\hat{a}}|} V_n
/// \f}
///
/// The volume function is (\cite deJonge1996 de Jonge 1996, ch. 4.9, eq. 4.20)
/// \anchor eq-GNSS-chi2-volume-function \f{equation}{ \label{eq:eq-GNSS-chi2-volume-function}
///   V_n = \frac{2}{n} \frac{\pi^{\frac{n}{2}}}{\Gamma \left( \frac{n}{2} \right)}
/// \f}
///
/// The determinant of the vairance covariance matrix is (\cite deJonge1996 de Jonge 1996, ch. 4.9, eq. 4.24)
/// \anchor eq-GNSS-chi2-volume-detQ \f{equation}{ \label{eq:eq-GNSS-chi2-volume-detQ}
///   |Q_{\hat{a}}| = \prod_{i=1}^{n} \sigma^2_{\hat{a}_{i|i+1, \dots, n}}
/// \f}
///
/// The volume \f$ E_n \f$ is good indicator for the number of candidates (\cite deJonge1996 de Jonge 1996, ch. 4.10)
/// \anchor eq-GNSS-chi2-volume-cand \f{equation}{ \label{eq:eq-GNSS-chi2-volume-cand}
///   n_{\text{cand}} = \text{(int)}E_n
/// \f}
///
/// @param[in] D Vector containing all the variances of the ambiguities (L^T * D * L decomposition)
/// @param[in] ncand Requested number of candidates (default = 2)
/// @param[in] factor Multiplication factor for the volume of the resulting search ellipsoid (default = 1.5)
/// @return Size of the ellipsoidal region \f$ \chi^2 \f$
template<typename DerivedD>
double calcChi2_volume(const Eigen::MatrixBase<DerivedD>& D, const Eigen::Index& ncand = 2, double factor = 1.5)
{
    auto n = static_cast<double>(D.rows());

    // Volume function (eq. 4.20)
    double Vn = 2.0 * std::pow(M_PI, n / 2.0) / (n * std::tgamma(n / 2.0));
    // Determinant of the variance covariance matrix (eq. 4.24)
    double detQ = D(0);
    for (Eigen::Index i = 1; i < D.rows(); i++) { detQ *= D(i); }

    return factor * std::pow(static_cast<double>(ncand) / (std::sqrt(detQ * Vn)), 2.0 / n); // Matlab code has: std::sqrt(detQ * Vn), paper not
}

/// @brief Calculates \f$ \chi^2 \f$, the size of the ellipsoidal region, via bootrapping
///
/// \f$ \boldsymbol{\check{z}}_B \f$ is a good candidate for setting the size of the search space (\cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.4.2, eq. 23.58)
/// \anchor eq-GNSS-chi2-bootstrap \f{equation}{ \label{eq:eq-GNSS-chi2-bootstrap}
///   \chi^2 = || \boldsymbol{\hat{z}} - \boldsymbol{\check{z}}_B ||_{\mathbf{Q}_{z}}^2 = (\boldsymbol{\hat{z}} - \boldsymbol{\check{z}}_B)^T \mathbf{Q}_{z}^{-1} (\boldsymbol{\hat{z}} - \boldsymbol{\check{z}}_B)
/// \f}
///
/// @param[in] a Float ambiguity vector [cycles]
/// @param[in] Q Variance/covariance matrix of the ambiguities
/// @param[in] L_LTDL_Q Lower-triangular matrix from the L^T * D * L decomposition of Q_z
/// @param[in] ncand Requested number of candidates (default = 2)
/// @return Size of the ellipsoidal region \f$ \chi^2 \f$
/// @note See \cite deJonge1996 de Jonge 1996, ch. 4.11
template<typename DerivedA, typename DerivedQ, typename DerivedL>
double calcChi2_bootstrap(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Q,
                          const Eigen::MatrixBase<DerivedL>& L_LTDL_Q, const Eigen::Index& ncand = 2)
{
    typename DerivedL::PlainObject Qz_inv = Q.inverse();

    std::vector<double> chi(static_cast<size_t>(a.rows()) + 1);
    for (Eigen::Index k = a.rows(); k >= 0; k--)
    {
        typename DerivedA::PlainObject a_fix = a;
        typename DerivedA::PlainObject a_float = a;

        for (Eigen::Index i = a.rows() - 1; i >= 0; i--)
        {
            double da = 0.0;
            for (Eigen::Index j = a.rows() - 1; j >= i; j--)
            {
                da += L_LTDL_Q(j, i) * (a_float(j) - a_fix(j));
            }
            a_float(i) -= da;

            if (i != k - 1) // Other candidates with small norms can be found through rounding all ambiguities but one to their nearest integer
            {
                a_fix(i) = std::round(a_float(i));
            }
            else // and one ambiguity to the next-nearest integer.
            {
                auto nearest = std::round(a_float(i));
                a_fix(i) = nearest + gcem::sgn(a_float(i) - nearest);
            }
        }

        chi.at(static_cast<size_t>(k)) = (a - a_fix).transpose() * Qz_inv * (a - a_fix);
    }
    std::ranges::sort(chi);

    return chi.at(static_cast<size_t>(ncand - 1)) + 1e-6; // Add a small amount to avoid boundary problems
}

} // namespace internal

/// @brief Calculates \f$ \chi^2 \f$, the size of the ellipsoidal region
/// @param[in] a Float ambiguity vector [cycles]
/// @param[in] Q Variance/covariance matrix of the ambiguities
/// @param[in] L_LTDL_Q Lower-triangular matrix from the L^T * D * L decomposition of Q
/// @param[in] D_LTDL_Q Diagonal matrix from the L^T * D * L decomposition of Q
/// @param[in] ncand Requested number of candidates (default = 2)
/// @param[in] factor Multiplication factor for the volume of the resulting search ellipsoid (default = 1.5)
/// @return Size of the ellipsoidal region \f$ \chi^2 \f$
template<typename DerivedA, typename DerivedQ, typename DerivedL, typename DerivedD>
double calcChi2(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Q,
                const Eigen::MatrixBase<DerivedL>& L_LTDL_Q, const Eigen::MatrixBase<DerivedD>& D_LTDL_Q,
                const Eigen::Index& ncand = 2, double factor = 1.5)
{
    if (ncand <= a.rows() + 1) // We get one more chi candidate then the amount of ambiguities
    {
        return internal::calcChi2_bootstrap(a, Q, L_LTDL_Q, ncand);
    }

    // Setting chi^2 over the volume is not accurate for number of candidates k less than a few (de Jonge 1996, ch. 4.10)
    // so this calculation can be inaccurate
    return internal::calcChi2_volume(D_LTDL_Q, ncand, factor);
}

} // namespace NAV::Ambiguity