// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ProcessNoise.hpp"

#include <Eigen/Dense>

namespace NAV
{

Eigen::Matrix3d G_RandomWalk(const Eigen::Vector3d& sigma2)
{
    // Math: \mathbf{G} = \begin{bmatrix} \sqrt{\sigma_{1}^2} & 0 & 0 \\ 0 & \sqrt{\sigma_{2}^2} & 0 \\ 0 & 0 & \sqrt{\sigma_{3}^2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)

    return Eigen::DiagonalMatrix<double, 3>{ sigma2.cwiseSqrt() };
}

Eigen::Matrix3d G_GaussMarkov1(const Eigen::Vector3d& sigma2, const Eigen::Vector3d& beta)
{
    // Math: \mathbf{G} = \begin{bmatrix} \sqrt{2 \sigma_{1}^2 \beta_{1}} & 0 & 0 \\ 0 & \sqrt{2 \sigma_{2}^2 \beta_{2}} & 0 \\ 0 & 0 & \sqrt{2 \sigma_{3}^2 \beta_{3}} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return Eigen::DiagonalMatrix<double, 3>{ (2.0 * beta.cwiseProduct(sigma2)).cwiseSqrt() };
}

Eigen::Vector3d psdNoise(const Eigen::Vector3d& sigma2_r, const double& tau_i)
{
    // Math: S_{r} = \sigma_{r}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_r * tau_i;
}

Eigen::Vector3d psdBiasVariation(const Eigen::Vector3d& sigma2_bd, const Eigen::Vector3d& tau_bd)
{
    // Math: S_{bd} = \frac{\sigma_{bd}^2}{\tau_{bd}} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bd.array() / tau_bd.array();
}

// double psdClockPhaseDrift(const double& sigma2_cPhi, const double& tau_i)
// {
//     return sigma2_cPhi / tau_i;
// }

// double psdClockFreqDrift(const double& sigma2_cf, const double& tau_i)
// {
//     return sigma2_cf / tau_i;
// }

Eigen::Matrix3d Q_psi_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const double& tau_s)
{
    return (S_rg.asDiagonal() * tau_s + 1.0 / 3.0 * S_bgd.asDiagonal() * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d ien_Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s)
{
    return (0.5 * S_rg.asDiagonal() * std::pow(tau_s, 2) + 0.25 * S_bgd.asDiagonal() * std::pow(tau_s, 4)) * ien_F_21;
}

Eigen::Matrix3d ien_Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const double& tau_s)
{
    return (S_ra.asDiagonal() * tau_s + 1.0 / 3.0 * S_bad.asDiagonal() * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity()
           + (1.0 / 3.0 * S_rg.asDiagonal() * std::pow(tau_s, 3) + 0.2 * S_bgd.asDiagonal() * std::pow(tau_s, 5)) * ien_F_21 * ien_F_21.transpose();
}

Eigen::Matrix3d ien_Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ien_F_21, const Eigen::Matrix3d& ien_Dcm_b, const double& tau_s)
{
    return 1.0 / 3.0 * S_bgd.asDiagonal() * std::pow(tau_s, 3) * ien_F_21 * ien_Dcm_b;
}

Eigen::Matrix3d n_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (1.0 / 3.0 * S_rg.asDiagonal() * std::pow(tau_s, 3) + 0.2 * S_bgd.asDiagonal() * std::pow(tau_s, 5)) * T_rn_p * n_F_21;
}

Eigen::Matrix3d ie_Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s)
{
    return (1.0 / 3.0 * S_rg.asDiagonal() * std::pow(tau_s, 3) + 0.2 * S_bgd.asDiagonal() * std::pow(tau_s, 5)) * ie_F_21;
}

Eigen::Matrix3d n_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (0.5 * S_ra.asDiagonal() * std::pow(tau_s, 2) + 0.25 * S_bad.asDiagonal() * std::pow(tau_s, 4)) * T_rn_p
           + (0.25 * S_rg.asDiagonal() * std::pow(tau_s, 4) + 1.0 / 6.0 * S_bgd.asDiagonal() * std::pow(tau_s, 6)) * T_rn_p * n_F_21 * n_F_21.transpose();
}

Eigen::Matrix3d ie_Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s)
{
    return (0.5 * S_ra.asDiagonal() * std::pow(tau_s, 2) + 0.25 * S_bad.asDiagonal() * std::pow(tau_s, 4)) * Eigen::Matrix3d::Identity()
           + (0.25 * S_rg.asDiagonal() * std::pow(tau_s, 4) + 1.0 / 6.0 * S_bgd.asDiagonal() * std::pow(tau_s, 6)) * ie_F_21 * ie_F_21.transpose();
}

Eigen::Matrix3d n_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (1.0 / 3.0 * S_ra.asDiagonal() * std::pow(tau_s, 3) + 0.2 * S_bad.asDiagonal() * std::pow(tau_s, 5)) * T_rn_p * T_rn_p
           + (0.2 * S_rg.asDiagonal() * std::pow(tau_s, 5) + 1.0 / 7.0 * S_bgd.asDiagonal() * std::pow(tau_s, 7)) * T_rn_p * n_F_21 * n_F_21.transpose() * T_rn_p;
}

Eigen::Matrix3d ie_Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const double& tau_s)
{
    return (1.0 / 3.0 * S_ra.asDiagonal() * std::pow(tau_s, 3) + 0.2 * S_bad.asDiagonal() * std::pow(tau_s, 5)) * Eigen::Matrix3d::Identity()
           + (0.2 * S_rg.asDiagonal() * std::pow(tau_s, 5) + 1.0 / 7.0 * S_bgd.asDiagonal() * std::pow(tau_s, 7)) * ie_F_21 * ie_F_21.transpose();
}

Eigen::Matrix3d n_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s)
{
    return 1.0 / 3.0 * S_bad.asDiagonal() * std::pow(tau_s, 3) * T_rn_p * n_Dcm_b;
}

Eigen::Matrix3d ie_Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s)
{
    return 1.0 / 3.0 * S_bad.asDiagonal() * std::pow(tau_s, 3) * ie_Dcm_b;
}

Eigen::Matrix3d n_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const double& tau_s)
{
    return 0.25 * S_bgd.asDiagonal() * std::pow(tau_s, 4) * T_rn_p * n_F_21 * n_Dcm_b;
}

Eigen::Matrix3d ie_Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& ie_F_21, const Eigen::Matrix3d& ie_Dcm_b, const double& tau_s)
{
    return 0.25 * S_bgd.asDiagonal() * std::pow(tau_s, 4) * ie_F_21 * ie_Dcm_b;
}

Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s)
{
    return 0.5 * S_bad.asDiagonal() * std::pow(tau_s, 2) * b_Dcm_ien;
}

Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s)
{
    return S_bad.asDiagonal() * tau_s * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& b_Dcm_ien, const double& tau_s)
{
    return 0.5 * S_bgd.asDiagonal() * std::pow(tau_s, 2) * b_Dcm_ien;
}

Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s)
{
    return S_bgd.asDiagonal() * tau_s * Eigen::Matrix3d::Identity();
}

Eigen::Matrix2d Q_gnss(const double& S_cPhi, const double& S_cf, const double& tau_s)
{
    Eigen::Matrix2d Qg = Eigen::Matrix2d::Zero();
    Qg(0, 0) = S_cPhi * tau_s + 1. / 3. * S_cf * std::pow(tau_s, 3);
    Qg(0, 1) = 0.5 * S_cf * std::pow(tau_s, 2);
    Qg(1, 0) = 0.5 * S_cf * std::pow(tau_s, 2);
    Qg(1, 1) = S_cf * tau_s;
    return Qg;
}

} // namespace NAV
