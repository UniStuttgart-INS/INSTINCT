#include "ProcessNoise.hpp"

#include <Eigen/Dense>

namespace NAV
{

Eigen::Matrix3d Q_psi_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const double& tau_s)
{
    return (Eigen::DiagonalMatrix<double, 3>(S_rg) * tau_s + 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_dv_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const double& tau_s)
{
    return (0.5 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 2) + 0.25 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 4)) * n_F_21;
}

Eigen::Matrix3d Q_dv_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const double& tau_s)
{
    return (Eigen::DiagonalMatrix<double, 3>(S_ra) * tau_s + 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 3)) * Eigen::Matrix3d::Identity()
           + (1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 3) + 0.2 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 5)) * n_F_21 * n_F_21.transpose();
}

Eigen::Matrix3d Q_dv_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s)
{
    return 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 3) * n_F_21 * b_n_Dcm;
}

Eigen::Matrix3d Q_dr_psi(const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 3) + 0.2 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 5)) * T_rn_p * n_F_21;
}

Eigen::Matrix3d Q_dr_dv(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const double& tau_s)
{
    return (0.5 * Eigen::DiagonalMatrix<double, 3>(S_ra) * std::pow(tau_s, 2) + 0.25 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 4)) * T_rn_p
           + (0.25 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 4) + 1.0 / 6.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 6)) * T_rn_p * n_F_21 * n_F_21.transpose();
}

Eigen::Matrix3d Q_dr_dr(const Eigen::Vector3d& S_ra, const Eigen::Vector3d& S_bad, const Eigen::Vector3d& S_rg, const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_F_21, const double& tau_s)
{
    return (1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_ra) * std::pow(tau_s, 3) + 0.2 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 5)) * T_rn_p * T_rn_p
           + (0.2 * Eigen::DiagonalMatrix<double, 3>(S_rg) * std::pow(tau_s, 5) + 1.0 / 7.0 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 7)) * T_rn_p * n_F_21 * n_F_21.transpose() * T_rn_p;
}

Eigen::Matrix3d Q_dr_df(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s)
{
    return 1.0 / 3.0 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 3) * T_rn_p * b_n_Dcm;
}

Eigen::Matrix3d Q_dr_domega(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s)
{
    return 0.25 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 4) * T_rn_p * n_F_21 * b_n_Dcm;
}

Eigen::Matrix3d Q_df_dv(const Eigen::Vector3d& S_bad, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s)
{
    return 0.5 * Eigen::DiagonalMatrix<double, 3>(S_bad) * std::pow(tau_s, 2) * b_n_Dcm.transpose();
}

Eigen::Matrix3d Q_df_df(const Eigen::Vector3d& S_bad, const double& tau_s)
{
    return Eigen::DiagonalMatrix<double, 3>(S_bad) * tau_s * Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d Q_domega_psi(const Eigen::Vector3d& S_bgd, const Eigen::Matrix3d& b_n_Dcm, const double& tau_s)
{
    return 0.5 * Eigen::DiagonalMatrix<double, 3>(S_bgd) * std::pow(tau_s, 2) * b_n_Dcm.transpose();
}

Eigen::Matrix3d Q_domega_domega(const Eigen::Vector3d& S_bgd, const double& tau_s)
{
    return Eigen::DiagonalMatrix<double, 3>(S_bgd) * tau_s * Eigen::Matrix3d::Identity();
}

} // namespace NAV
