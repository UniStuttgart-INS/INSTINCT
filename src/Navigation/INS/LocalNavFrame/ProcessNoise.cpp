#include "ProcessNoise.hpp"

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

double psdGyroNoise(const Eigen::Vector3d& sigma2_ra, const double& tau_i)
{
    // Math: S_{ra} = \sigma_{ra}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_ra.mean() * tau_i; // TODO: This is only a temporary fix. Here the values should be accounted for, for each axis separately.
}

double psdAccelNoise(const Eigen::Vector3d& sigma2_rg, const double& tau_i)
{
    // Math: S_{rg} = \sigma_{rg}^2\tau_i \qquad \text{P. Groves}\,(14.83)
    return sigma2_rg.mean() * tau_i; // TODO: This is only a temporary fix. Here the values should be accounted for, for each axis separately.
}

double psdAccelBiasVariation(const double& sigma2_bad, const double& tau_i)
{
    // Math: S_{bad} = \frac{\sigma_{bad}^2}{\tau_i} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bad / tau_i;
}

double psdGyroBiasVariation(const double& sigma2_bgd, const double& tau_i)
{
    // Math: S_{bgd} = \frac{\sigma_{bgd}^2}{\tau_i} \qquad \text{P. Groves}\,(14.84)
    return sigma2_bgd / tau_i;
}

Eigen::Matrix3d Q_psi_psi(const double& S_rg, const double& S_bgd, const double& tau_s)
{
    return (S_rg * tau_s + 1.0 / 3.0 * S_bgd * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_dv_psi(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (0.5 * S_rg * std::pow(tau_s, 2) + 0.25 * S_bgd * std::pow(tau_s, 4)) * F_21_n;
}

Eigen::Matrix3d Q_dv_dv(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (S_ra * tau_s + 1.0 / 3.0 * S_bad * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity()
           + (1.0 / 3.0 * S_rg * std::pow(tau_s, 3) + 0.2 * S_bgd * std::pow(tau_s, 5)) * F_21_n * F_21_n.transpose();
}

Eigen::Matrix3d Q_dv_domega(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 1.0 / 3.0 * S_bgd * std::pow(tau_s, 3) * F_21_n * DCM_nb;
}

Eigen::Matrix3d Q_dr_psi(const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (1.0 / 3.0 * S_rg * std::pow(tau_s, 3) + 0.2 * S_bgd * std::pow(tau_s, 5)) * T_rn_p * F_21_n;
}

Eigen::Matrix3d Q_dr_dv(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (0.5 * S_ra * std::pow(tau_s, 2) + 0.25 * S_bad * std::pow(tau_s, 4)) * T_rn_p
           + (0.25 * S_rg * std::pow(tau_s, 4) + 1.0 / 6.0 * S_bgd * std::pow(tau_s, 6)) * T_rn_p * F_21_n * F_21_n.transpose();
}

Eigen::Matrix3d Q_dr_dr(const double& S_ra, const double& S_bad, const double& S_rg, const double& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& F_21_n, const double& tau_s)
{
    return (1.0 / 3.0 * S_ra * std::pow(tau_s, 3) + 0.2 * S_bad * std::pow(tau_s, 5)) * T_rn_p * T_rn_p
           + (0.2 * S_rg * std::pow(tau_s, 5) + 1.0 / 7.0 * S_bgd * std::pow(tau_s, 7)) * T_rn_p * F_21_n * F_21_n.transpose() * T_rn_p;
}

Eigen::Matrix3d Q_dr_df(const double& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 1.0 / 3.0 * S_bad * std::pow(tau_s, 3) * T_rn_p * DCM_nb.transpose();
}

Eigen::Matrix3d Q_dr_domega(const double& S_bgd, const Eigen::Matrix3d& F_21_n, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.25 * S_bgd * std::pow(tau_s, 4) * T_rn_p * F_21_n * DCM_nb.transpose();
}

Eigen::Matrix3d Q_df_dv(const double& S_bad, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.5 * S_bad * std::pow(tau_s, 2) * DCM_nb;
}

Eigen::Matrix3d Q_df_df(const double& S_bad, const double& tau_s)
{
    return S_bad * tau_s * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_domega_psi(const double& S_bgd, const Eigen::Matrix3d& DCM_nb, const double& tau_s)
{
    return 0.5 * S_bgd * std::pow(tau_s, 2) * DCM_nb;
}

Eigen::Matrix3d Q_domega_domega(const double& S_bgd, const double& tau_s)
{
    return S_bgd * tau_s * Eigen::Matrix3d::Identity();
}

} // namespace NAV
