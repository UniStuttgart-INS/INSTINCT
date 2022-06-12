#include "ErrorEquations.hpp"

#include "Navigation/Constants.hpp"

namespace NAV
{

Eigen::Matrix3d n_F_dpsi_dpsi(const Eigen::Vector3d& n_omega_in)
{
    // Math: \mathbf{F}_{11}^n = -[\mathbf{\omega}_{in}^n \land]
    Eigen::Matrix3d skewMat;
    // clang-format off
    skewMat <<       0       ,  n_omega_in(2), -n_omega_in(1),
               -n_omega_in(2),       0       ,  n_omega_in(0),
                n_omega_in(1), -n_omega_in(0),       0       ;
    // clang-format on
    return skewMat;
}

Eigen::Matrix3d n_F_dpsi_dv(double latitude, double height, double R_N, double R_E)
{
    // Math: \mathbf{F}_{12}^n = \begin{bmatrix} 0 & -\frac{1}{R_E + h} & 0 \\ \frac{1}{R_N + h} & 0 & 0 \\ 0 & \frac{\tan{\phi}}{R_E + h} & 0 \end{bmatrix}
    Eigen::Matrix3d n_F_12 = Eigen::Matrix3d::Zero(3, 3);

    n_F_12(0, 1) = -1 / (R_E + height);
    n_F_12(1, 0) = 1 / (R_N + height);
    n_F_12(2, 1) = std::tan(latitude) / (R_E + height);

    return n_F_12;
}

Eigen::Matrix3d n_F_dpsi_dr(double latitude, double height, const Eigen::Vector3d& n_velocity, double R_N, double R_E)
{
    const double& v_N = n_velocity(0); // North velocity in [m/s]
    const double& v_E = n_velocity(1); // East velocity in [m/s]

    // Math: \mathbf{F}_{13}^n = \begin{bmatrix} \omega_{ie}\sin{\phi} & 0 & \frac{v_E}{(R_E + h)^2} \\ 0 & 0 & -\frac{v_N}{(R_N + h)^2} \\ \omega_{ie}\cos{\phi} + \frac{v_E}{(R_E + h)\cos^2{\phi}} & 0 & -\frac{v_E\tan{\phi}}{(R_E + h)^2} \end{bmatrix}
    Eigen::Matrix3d n_F_13 = Eigen::Matrix3d::Zero(3, 3);

    n_F_13(0, 0) = InsConst::omega_ie * std::sin(latitude);
    n_F_13(0, 2) = v_E / std::pow(R_E + height, 2.0);
    n_F_13(1, 2) = -v_N / std::pow(R_N + height, 2.0);
    n_F_13(2, 0) = InsConst::omega_ie * std::cos(latitude)
                   + v_E / ((R_E + height) * std::pow(std::cos(latitude), 2));
    n_F_13(2, 2) = -v_E * std::tan(latitude) / std::pow(R_E + height, 2.0);

    return n_F_13;
}

Eigen::Matrix3d n_F_dpsi_dw(const Eigen::Matrix3d& n_Dcm_b)
{
    return n_Dcm_b;
}

Eigen::Matrix3d n_F_dv_dpsi(const Eigen::Vector3d& n_force_ib)
{
    const auto& f_N = n_force_ib(0); // Specific force in North direction in [m/s^2]
    const auto& f_E = n_force_ib(1); // Specific force in East direction in [m/s^2]
    const auto& f_D = n_force_ib(2); // Specific force in Down direction in [m/s^2]

    // Math: \mathbf{F}_{21}^n = \begin{bmatrix} (-\mathbf{C}_{b}^n \hat{f}_{ib}^b) \land \end{bmatrix}
    Eigen::Matrix3d skewMat;
    // clang-format off
    skewMat <<   0 ,  f_D, -f_E,
               -f_D,   0 ,  f_N,
                f_E, -f_N,   0 ;
    // clang-format on

    return skewMat;
}

Eigen::Matrix3d n_F_dv_dv(const Eigen::Vector3d& n_velocity, double latitude, double height, double R_N, double R_E)
{
    const double& v_N = n_velocity(0); // North velocity in [m/s]
    const double& v_E = n_velocity(1); // East velocity in [m/s]
    const double& v_D = n_velocity(2); // Down velocity in [m/s]

    // Math: \mathbf{F}_{22}^n = \begin{bmatrix} \frac{v_D}{R_N+h} & -2\frac{v_E\tan{\phi}}{R_E+h}-2\omega_{ie}\sin{\phi} & \frac{v_N}{R_N+h} \\ \frac{v_E\tan{\phi}}{R_E+h}+2\omega_{ie}\sin{\phi} & \frac{v_N\tan{\phi}+v_D}{R_E+h} & \frac{v_E}{R_E+h}+2\omega_{ie}\cos{\phi} \\ -\frac{2v_N}{R_N+h} & -\frac{2v_E}{R_E+h}-2\omega_{ie}\cos{\phi} & 0 \end{bmatrix}
    Eigen::Matrix3d n_F_22 = Eigen::Matrix3d::Zero(3, 3);

    n_F_22(0, 0) = v_D / (R_N + height);
    n_F_22(0, 1) = -2.0 * v_E * std::tan(latitude) / (R_E + height)
                   - 2.0 * InsConst::omega_ie * std::sin(latitude);
    n_F_22(0, 2) = v_N / (R_N + height);
    n_F_22(1, 0) = v_E * std::tan(latitude) / (R_E + height)
                   + 2.0 * InsConst::omega_ie * std::sin(latitude);
    n_F_22(1, 1) = (v_N * std::tan(latitude) + v_D) / (R_E + height);
    n_F_22(1, 2) = v_E / (R_E + height)
                   + 2.0 * InsConst::omega_ie * std::cos(latitude);
    n_F_22(2, 0) = -2.0 * v_N / (R_N + height);
    n_F_22(2, 1) = -2.0 * v_E / (R_E + height)
                   - 2.0 * InsConst::omega_ie * std::cos(latitude);

    return n_F_22;
}

Eigen::Matrix3d n_F_dv_dr(const Eigen::Vector3d& n_velocity, double latitude, double height, double R_N, double R_E, double g_0, double r_eS_e)
{
    // Math: \mathbf{F}_{23}^n = \begin{bmatrix} -\frac{v_E^2}{(R_E+h)\cos^2{\phi}}-2v_E\omega_{ie}\cos{\phi} & 0 & \frac{v_E^2\tan{\phi}}{(R_E+h)^2}-\frac{v_Nv_D}{(R_N+h)^2} \\ \frac{v_Nv_E}{(R_E+h)\cos^2{\phi}}+2v_N\omega_{ie}\cos{\phi}-2v_D\omega_{ie}\sin{\phi} & 0 & -\frac{v_Nv_E\tan{\phi}+v_Ev_D}{(R_E+h)^2} \\ 2v_E\omega_{ie}\sin{\phi} & 0 & \frac{v_E^2}{(R_E+h)^2}+\frac{v_N^2}{(R_N+h)^2}-\frac{2g_0}{r_{eS}^e} \end{bmatrix}
    Eigen::Matrix3d n_F_23 = Eigen::Matrix3d::Zero(3, 3);

    const double& v_N = n_velocity(0);
    const double& v_E = n_velocity(1);
    const double& v_D = n_velocity(2);

    n_F_23(0, 0) = -std::pow(v_E, 2) / ((R_E + height) * std::pow(std::cos(latitude), 2.0))
                   - 2.0 * v_E * InsConst::omega_ie * std::cos(latitude);

    n_F_23(0, 2) = std::pow(v_E, 2) * std::tan(latitude) / std::pow(R_E + height, 2.0)
                   - (v_N * v_D) / std::pow(R_N + height, 2.0);

    n_F_23(1, 0) = v_N * v_E / ((R_E + height) * std::pow(std::cos(latitude), 2.0))
                   + 2.0 * v_N * InsConst::omega_ie * std::cos(latitude)
                   - 2.0 * v_D * InsConst::omega_ie * std::sin(latitude);

    n_F_23(1, 2) = -(v_N * v_E * std::tan(latitude) + v_E * v_D) / std::pow(R_E + height, 2.0);

    n_F_23(2, 0) = 2.0 * v_E * InsConst::omega_ie * std::sin(latitude);

    n_F_23(2, 2) = std::pow(v_E, 2) / std::pow(R_E + height, 2.0)
                   + std::pow(v_N, 2) / std::pow(R_N + height, 2.0)
                   - 2 * g_0 / r_eS_e;

    return n_F_23;
}

Eigen::Matrix3d n_F_dv_df(const Eigen::Matrix3d& n_Dcm_b)
{
    return n_Dcm_b;
}

Eigen::Matrix3d n_F_dr_dv(double latitude, double height, double R_N, double R_E)
{
    // Math: \mathbf{F}_{32}^n = \begin{bmatrix} \frac{1}{R_N + h} & 0 & 0 \\ 0 & \frac{1}{(R_E + h)\cos{\phi}} & 0 \\ 0 & 0 & -1 \end{bmatrix} \quad \text{P. Groves}\,(14.70)
    Eigen::Matrix3d n_F_32 = Eigen::Matrix3d::Zero(3, 3);
    n_F_32(0, 0) = 1.0 / (R_N + height);
    n_F_32(1, 1) = 1.0 / ((R_E + height) * std::cos(latitude));
    n_F_32(2, 2) = -1;

    return n_F_32;
}

Eigen::Matrix3d n_F_dr_dr(const Eigen::Vector3d& n_velocity, double latitude, double height, double R_N, double R_E)
{
    const double& v_N = n_velocity(0);
    const double& v_E = n_velocity(1);

    // Math: \mathbf{F}_{33}^n = \begin{bmatrix} 0 & 0 & -\frac{v_N}{(R_N + h)^2} \\ \frac{v_E \tan{\phi}}{(R_E + h) \cos{\phi}} & 0 & -\frac{v_E}{(R_E + h)^2 \cos{\phi}} \\ 0 & 0 & 0 \end{bmatrix} \quad \text{P. Groves}\,(14.71)
    Eigen::Matrix3d n_F_33 = Eigen::Matrix3d::Zero(3, 3);

    n_F_33(0, 2) = -v_N / std::pow(R_N + height, 2.0);
    n_F_33(1, 0) = v_E * std::tan(latitude) / ((R_E + height) * std::cos(latitude));
    n_F_33(1, 2) = -v_E / (std::pow(R_E + height, 2.0) * std::cos(latitude));

    return n_F_33;
}

Eigen::Matrix3d n_F_df_df(const Eigen::Vector3d& beta_a)
{
    // Math: \mathbf{F}_{a} = - \begin{bmatrix} \beta_{a,1} & 0 & 0 \\ 0 & \beta_{a,2} & 0 \\ 0 & 0 & \beta_{a,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * beta_a.asDiagonal();
}

Eigen::Matrix3d n_F_dw_dw(const Eigen::Vector3d& beta_omega)
{
    // Math: \mathbf{F}_{\omega} = - \begin{bmatrix} \beta_{\omega,1} & 0 & 0 \\ 0 & \beta_{\omega,2} & 0 \\ 0 & 0 & \beta_{\omega,2} \end{bmatrix} \quad \text{T. Hobiger}\,(6.3)
    return -1.0 * beta_omega.asDiagonal();
}

} // namespace NAV