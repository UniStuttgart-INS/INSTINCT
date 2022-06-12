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
} // namespace NAV
