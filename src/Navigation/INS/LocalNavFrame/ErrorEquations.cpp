#include "ErrorEquations.hpp"

#include "Navigation/Constants.hpp"

namespace NAV
{

Eigen::Matrix3d F_dotpsi_psi_n(const Eigen::Vector3d& omega_in_n)
{
    // Math: \mathbf{F}_{11}^n = -[\mathbf{\omega}_{in}^n \land] \qquad \text{P. Groves}\,(14.64)
    Eigen::Matrix3d skewMat;
    // clang-format off
    skewMat <<       0       ,  omega_in_n(2), -omega_in_n(1),
               -omega_in_n(2),       0       ,  omega_in_n(0),
                omega_in_n(1), -omega_in_n(0),       0       ;
    // clang-format on
    return skewMat;
}

Eigen::Matrix3d F_dotpsi_dv_n(double latitude, double height, double R_N, double R_E)
{
    // Math: \mathbf{F}_{12}^n = \begin{bmatrix} 0 & \frac{1}{R_E + h} & 0 \\ -\frac{1}{R_N + h} & 0 & 0 \\ 0 & -\frac{\tan{\phi}}{R_E + h} & 0 \end{bmatrix} \qquad \text{P. Groves}\,(14.65) (\text{sign flip})
    Eigen::Matrix3d F_12_n = Eigen::Matrix3d::Zero(3, 3);

    F_12_n(0, 1) = 1 / (R_E + height);
    F_12_n(1, 0) = -1 / (R_N + height);
    F_12_n(2, 1) = -std::tan(latitude) / (R_E + height);

    return F_12_n;
}

Eigen::Matrix3d F_dotpsi_dr_n(double latitude, double height, const Eigen::Vector3d& v_n, double R_N, double R_E)
{
    const double& v_N = v_n(0); // North velocity in [m/s]
    const double& v_E = v_n(1); // East velocity in [m/s]

    // Math: \mathbf{F}_{13}^n = \begin{bmatrix} -\omega_{ie}\sin{\phi} & 0 & -\frac{v_E}{(R_E + h)^2} \\ 0 & 0 & \frac{v_N}{(R_N + h)^2} \\ -\omega_{ie}\cos{\phi} - \frac{v_E}{(R_E + h)\cos^2{\phi}} & 0 & \frac{v_E\tan{\phi}}{(R_E + h)^2} \end{bmatrix} \qquad \text{P. Groves}\,(14.66) (\text{sign flip})
    Eigen::Matrix3d F_13_n = Eigen::Matrix3d::Zero(3, 3);

    F_13_n(0, 0) = -InsConst::omega_ie * std::sin(latitude);
    F_13_n(0, 2) = -v_E / std::pow(R_E + height, 2.0);
    F_13_n(1, 2) = v_N / std::pow(R_N + height, 2.0);
    F_13_n(2, 0) = -InsConst::omega_ie * std::cos(latitude)
                   - v_E / ((R_E + height) * std::pow(std::cos(latitude), 2));
    F_13_n(2, 2) = v_E * std::tan(latitude) / std::pow(R_E + height, 2.0);

    return F_13_n;
}

Eigen::Matrix3d F_dotpsi_dw_n(const Eigen::Matrix3d& C_nb)
{
    return -C_nb;
}

Eigen::Matrix3d F_dotdv_psi_n(const Eigen::Vector3d& f_ib_n)
{
    const auto& f_N = f_ib_n(0); // Specific force in North direction in [m/s^2]
    const auto& f_E = f_ib_n(1); // Specific force in East direction in [m/s^2]
    const auto& f_D = f_ib_n(2); // Specific force in Down direction in [m/s^2]

    // Math: \mathbf{F}_{21}^n = -\begin{bmatrix} (\mathbf{C}_{b}^n \hat{f}_{ib}^b) \land \end{bmatrix} \qquad \text{P. Groves}\,(14.67) (\text{sign flip})
    Eigen::Matrix3d skewMat;
    // clang-format off
    skewMat <<   0 , -f_D,  f_E,
                f_D,   0 , -f_N,
               -f_E,  f_N,   0 ;
    // clang-format on

    return skewMat;
}

Eigen::Matrix3d F_dotdv_dv_n(const Eigen::Vector3d& v_n, double latitude, double height, double R_N, double R_E)
{
    const double& v_N = v_n(0); // North velocity in [m/s]
    const double& v_E = v_n(1); // East velocity in [m/s]
    const double& v_D = v_n(2); // Down velocity in [m/s]

    // Math: \mathbf{F}_{22}^n = \begin{bmatrix} \frac{v_D}{R_N+h} & -2\frac{v_E\tan{\phi}}{R_E+h}-2\omega_{ie}\sin{\phi} & \frac{v_N}{R_N+h} \\ \frac{v_E\tan{\phi}}{R_E+h}+2\omega_{ie}\sin{\phi} & \frac{v_N\tan{\phi}+v_D}{R_E+h} & \frac{v_E}{R_E+h}+2\omega_{ie}\cos{\phi} \\ -\frac{2v_N}{R_N+h} & -\frac{2v_E}{R_E+h}-2\omega_{ie}\cos{\phi} & 0 \end{bmatrix} \qquad \text{P. Groves}\,(14.68)
    Eigen::Matrix3d F_22_n = Eigen::Matrix3d::Zero(3, 3);

    F_22_n(0, 0) = v_D / (R_N + height);
    F_22_n(0, 1) = -2.0 * v_E * std::tan(latitude) / (R_E + height)
                   - 2.0 * InsConst::omega_ie * std::sin(latitude);
    F_22_n(0, 2) = v_N / (R_N + height);
    F_22_n(1, 0) = v_E * std::tan(latitude) / (R_E + height)
                   + 2.0 * InsConst::omega_ie * std::sin(latitude);
    F_22_n(1, 1) = (v_N * std::tan(latitude) + v_D) / (R_E + height);
    F_22_n(1, 2) = v_E / (R_E + height)
                   + 2.0 * InsConst::omega_ie * std::cos(latitude);
    F_22_n(2, 0) = -2.0 * v_N / (R_N + height);
    F_22_n(2, 1) = -2.0 * v_E / (R_E + height)
                   - 2.0 * InsConst::omega_ie * std::cos(latitude);

    return F_22_n;
}

Eigen::Matrix3d F_dotdv_dr_n(const Eigen::Vector3d& v_n, double latitude, double height, double R_N, double R_E, double g_0, double r_eS_e)
{
    // Math: \mathbf{F}_{23}^n = \begin{bmatrix} -\frac{v_E^2}{(R_E+h)\cos^2{\phi}}-2v_E\omega_{ie}\cos{\phi} & 0 & \frac{v_E^2\tan{\phi}}{(R_E+h)^2}-\frac{v_Nv_D}{(R_N+h)^2} \\ \frac{v_Nv_E}{(R_E+h)\cos^2{\phi}}+2v_N\omega_{ie}\cos{\phi}-2v_D\omega_{ie}\sin{\phi} & 0 & -\frac{v_Nv_E\tan{\phi}+v_Ev_D}{(R_E+h)^2} \\ 2v_E\omega_{ie}\sin{\phi} & 0 & \frac{v_E^2}{(R_E+h)^2}+\frac{v_N^2}{(R_N+h)^2}-\frac{2g_0}{r_{eS}^e} \end{bmatrix} \qquad \text{P. Groves}\,(14.69)
    Eigen::Matrix3d F_23_n = Eigen::Matrix3d::Zero(3, 3);

    const double& v_N = v_n(0);
    const double& v_E = v_n(1);
    const double& v_D = v_n(2);

    F_23_n(0, 0) = -std::pow(v_E, 2) / ((R_E + height) * std::pow(std::cos(latitude), 2.0))
                   - 2.0 * v_E * InsConst::omega_ie * std::cos(latitude);

    F_23_n(0, 2) = std::pow(v_E, 2) * std::tan(latitude) / std::pow(R_E + height, 2.0)
                   - (v_N * v_D) / std::pow(R_N + height, 2.0);

    F_23_n(1, 0) = v_N * v_E / ((R_E + height) * std::pow(std::cos(latitude), 2.0))
                   + 2.0 * v_N * InsConst::omega_ie * std::cos(latitude)
                   - 2.0 * v_D * InsConst::omega_ie * std::sin(latitude);

    F_23_n(1, 2) = -(v_N * v_E * std::tan(latitude) + v_E * v_D) / std::pow(R_E + height, 2.0);

    F_23_n(2, 0) = 2.0 * v_E * InsConst::omega_ie * std::sin(latitude);

    F_23_n(2, 2) = std::pow(v_E, 2) / std::pow(R_E + height, 2.0)
                   + std::pow(v_N, 2) / std::pow(R_N + height, 2.0)
                   - 2 * g_0 / r_eS_e;

    return F_23_n;
}

Eigen::Matrix3d F_dotdv_df_n(const Eigen::Matrix3d& C_nb)
{
    return C_nb;
}

Eigen::Matrix3d F_dotdr_dv_n(double latitude, double height, double R_N, double R_E)
{
    // Math: \mathbf{F}_{32}^n = \begin{bmatrix} \frac{1}{R_N + h} & 0 & 0 \\ 0 & \frac{1}{(R_E + h)\cos{\phi}} & 0 \\ 0 & 0 & -1 \end{bmatrix} \quad \text{P. Groves}\,(14.70)
    Eigen::Matrix3d F_32_n = Eigen::Matrix3d::Zero(3, 3);
    F_32_n(0, 0) = 1.0 / (R_N + height);
    F_32_n(1, 1) = 1.0 / ((R_E + height) * std::cos(latitude));
    F_32_n(2, 2) = -1;

    return F_32_n;
}

Eigen::Matrix3d F_dotdr_dr_n(const Eigen::Vector3d& v_n, double latitude, double height, double R_N, double R_E)
{
    const double& v_N = v_n(0);
    const double& v_E = v_n(1);

    // Math: \mathbf{F}_{33}^n = \begin{bmatrix} 0 & 0 & -\frac{v_N}{(R_N + h)^2} \\ \frac{v_E \tan{\phi}}{(R_E + h) \cos{\phi}} & 0 & -\frac{v_E}{(R_E + h)^2 \cos{\phi}} \\ 0 & 0 & 0 \end{bmatrix} \quad \text{P. Groves}\,(14.71)
    Eigen::Matrix3d F_33_n = Eigen::Matrix3d::Zero(3, 3);

    F_33_n(0, 2) = -v_N / std::pow(R_N + height, 2.0);
    F_33_n(1, 0) = v_E * std::tan(latitude) / ((R_E + height) * std::cos(latitude));
    F_33_n(1, 2) = -v_E / (std::pow(R_E + height, 2.0) * std::cos(latitude));

    return F_33_n;
}

Eigen::Matrix3d F_dotdf_df_n(const Eigen::Vector3d& beta_a)
{
    // Math: \mathbf{F}_{a} = - \begin{bmatrix} \beta_{a,1} & 0 & 0 \\ 0 & \beta_{a,2} & 0 \\ 0 & 0 & \beta_{a,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * Eigen::DiagonalMatrix<double, 3>{ beta_a };
}

Eigen::Matrix3d F_dotdw_dw_n(const Eigen::Vector3d& beta_omega)
{
    // Math: \mathbf{F}_{\omega} = - \begin{bmatrix} \beta_{\omega,1} & 0 & 0 \\ 0 & \beta_{\omega,2} & 0 \\ 0 & 0 & \beta_{\omega,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * Eigen::DiagonalMatrix<double, 3>{ beta_omega };
}

} // namespace NAV