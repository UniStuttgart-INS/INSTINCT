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

Eigen::Matrix3d Q_psi_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const double& tau_s)
{
    return (Eigen::DiagonalMatrix<double, 3>(S_rg) * tau_s + 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (0.5 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 2) + 0.25 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 4)) * F_21_n;
}

Eigen::Matrix3d Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (Eigen::DiagonalMatrix<double, 3>(S_ra) * tau_s + 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity()
           + (1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 3) + 0.2 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 5)) * F_21_n * F_21_n.transpose();
}

Eigen::Matrix3d Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 3) * F_21_n * DCM_nb;
}

Eigen::Matrix3d Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 3) + 0.2 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 5)) * T_rn_p * F_21_n;
}

Eigen::Matrix3d Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (0.5 * Eigen::DiagonalMatrix<double, 3>(S_ra) * std::pow(tau_s, 2) + 0.25 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 4)) * T_rn_p
           + (0.25 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 4) + 1.0 / 6.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 6)) * T_rn_p * F_21_n * F_21_n.transpose();
}

Eigen::Matrix3d Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_ra) * std::pow(tau_s, 3) + 0.2 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 5)) * T_rn_p * T_rn_p
           + (0.2 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 5) + 1.0 / 7.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 7)) * T_rn_p * F_21_n * F_21_n.transpose() * T_rn_p;
}

Eigen::Matrix3d Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 3) * T_rn_p * DCM_nb;
}

Eigen::Matrix3d Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.25 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 4) * T_rn_p * F_21_n * DCM_nb;
}

Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.5 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 2) * DCM_nb.transpose();
}

Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s)
{
    return Eigen::DiagonalMatrix<double, 3>(S_bad) * tau_s * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.5 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 2) * DCM_nb.transpose();
}

Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s)
{
    return Eigen::DiagonalMatrix<double, 3>(S_bgd) * tau_s * Eigen::Matrix3d::Identity();
}

} // namespace NAV
