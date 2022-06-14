#include "ErrorEquations.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV
{

Eigen::Matrix3d e_F_dpsi_dpsi(double omega_ie)
{
    // Math: \mathbf{F}_{11}^e = -\mathbf{\Omega}_{ie}^e
    Eigen::Matrix3d e_F_11;
    // clang-format off
    e_F_11 <<      0   , omega_ie, 0,
              -omega_ie,     0   , 0,
                   0   ,     0   , 0;
    // clang-format on
    return e_F_11;
}

Eigen::Matrix3d e_F_dpsi_dw(const Eigen::Matrix3d& e_Dcm_b)
{
    return e_Dcm_b;
}

Eigen::Matrix3d e_F_dv_dpsi(const Eigen::Vector3d& e_force_ib)
{
    // Math: \mathbf{F}_{21}^e = \begin{bmatrix} (\mathbf{C}_{b}^e \hat{f}_{ib}^b) \land \end{bmatrix}
    Eigen::Matrix3d e_F_21;
    // clang-format off
    e_F_21 <<        0       ,  e_force_ib.z(), -e_force_ib.y(),
              -e_force_ib.z(),        0       ,  e_force_ib.x(),
               e_force_ib.y(), -e_force_ib.x(),        0       ;
    // clang-format on
    return e_F_21;
}

Eigen::Matrix3d e_F_dv_dv(double omega_ie)
{
    // Math: \mathbf{F}_{22}^e = -2\mathbf{\Omega}_{ie}^e
    Eigen::Matrix3d e_F_22;
    // clang-format off
    e_F_22 <<        0     , 2 * omega_ie, 0,
              -2 * omega_ie,       0     , 0,
                     0     ,       0     , 0;
    // clang-format on
    return e_F_22;
}

Eigen::Matrix3d e_F_dv_dr(const Eigen::Vector3d& e_position, const Eigen::Vector3d& e_gravitation, double r_eS_e, const Eigen::Vector3d& e_omega_ie)
{
    Eigen::Matrix3d e_Omega_ie = skewSymmetricMatrix(e_omega_ie);

    // Math: \mathbf{F}_{23}^e = -\left( \dfrac{2 \boldsymbol{\gamma}_{ib}^e}{r_{eS}^e} \dfrac{{\mathbf{r}_{eb}^e}^T}{\left| \mathbf{r}_{eb}^e \right|} + \boldsymbol{\Omega}_{ie}^e \boldsymbol{\Omega}_{ie}^e \right)
    Eigen::Matrix3d e_F_23 = -((2 * e_gravitation * e_position.transpose()) / (r_eS_e * e_position.norm()) + e_Omega_ie * e_Omega_ie);

    return e_F_23;
}

Eigen::Matrix3d e_F_dv_df(const Eigen::Matrix3d& e_Dcm_b)
{
    return e_Dcm_b;
}

Eigen::Matrix3d e_F_dr_dv()
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d e_F_df_df(const Eigen::Vector3d& beta_a)
{
    // Math: \mathbf{F}_{a} = - \begin{bmatrix} \beta_{a,1} & 0 & 0 \\ 0 & \beta_{a,2} & 0 \\ 0 & 0 & \beta_{a,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * beta_a.asDiagonal();
}

Eigen::Matrix3d e_F_dw_dw(const Eigen::Vector3d& beta_omega)
{
    // Math: \mathbf{F}_{\omega} = - \begin{bmatrix} \beta_{\omega,1} & 0 & 0 \\ 0 & \beta_{\omega,2} & 0 \\ 0 & 0 & \beta_{\omega,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * beta_omega.asDiagonal();
}

} // namespace NAV