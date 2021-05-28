#include "AssociatedLegendre.hpp"
#include "util/Logger.hpp"

#include "util/InsConstants.hpp"
#include "util/InsMath.hpp"
#include <cmath>

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> NAV::utilGravity::associatedLegendre(int N, double x)
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd Pd = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd full_derivs = Eigen::MatrixXd::Zero(N + 1, N);

    // Compute P^n_0^M
    Eigen::VectorXd L = Eigen::VectorXd::Zero(N).transpose();
    Eigen::VectorXd Ld = Eigen::VectorXd::Zero(N).transpose();

    L(0) = 1.0;
    L(1) = x;
    Ld(0) = 0.0;
    Ld(1) = 1.0;

    for (int i = 2; i < N - 1; i++)
    {
        auto n = static_cast<double>(i - 1);
        L(i) = ((2.0 * n + 1.0) * x * L(i - 1) - n * L(i - 2)) / (n + 1.0);
        Ld(i) = ((2.0 * n + 1.0) * (L(i - 1) + x * Ld(i - 1)) - n * Ld(i - 2)) / (n + 1.0);
    }

    P.row(0) = L;
    Pd.row(0) = Ld;

    full_derivs.row(0) = Pd.row(0);

    for (int j = 1; j <= N; j++)
    {
        for (int k = 2; k <= N - 1; k++)
        {
            auto m = static_cast<double>(k - 1);
            full_derivs(j, k) = ((2.0 * m + 1.0) * (static_cast<double>(j + 1) * full_derivs(j - 1, k - 1) + x * full_derivs(j, k - 1)) - m * full_derivs(j, k - 2)) / (m + 1.0);
        }
    }

    // Compute P^m_n^M
    for (int mm = 1; mm <= N - 1; mm++)
    {
        for (int nn = mm; nn <= N - 1; nn++)
        {
            // double factor = std::pow(-1.0, (static_cast<double>(mm) - 1.0));
            double factor = 1.0;

            P(mm, nn) = factor * std::pow((1.0 - std::pow(x, 2.0)), ((static_cast<double>(mm + 1) - 1.0) / 2.0)) * full_derivs(mm - 1, nn);

            Pd(mm, nn) = -factor * ((static_cast<double>(mm + 1) - 1.0) / 2.0) * std::pow((1.0 - std::pow(x, 2.0)), ((((static_cast<double>(mm + 1) - 1.0) / 2.0) - 1.0))) * 2.0 * x
                         + factor * std::pow((1.0 - std::pow(x, 2.0)), (((static_cast<double>(mm + 1) - 1.0) / 2.0))) * full_derivs(mm, nn);
        }
    }

    // Normalize values^M
    for (int n = 0; n < N - 1; n++)
    {
        for (int m = 0; m < N - 1; m++)
        {
            if (n >= m)
            {
                uint32_t factor = (2 * static_cast<uint32_t>(n + 1)) * NAV::factorial(static_cast<uint32_t>(n - m)) / NAV::factorial(static_cast<uint32_t>(n + m));

                if (m != 0)
                {
                    factor = factor * 2;
                }

                double Factor = std::sqrt(static_cast<double>(factor));

                P(m, n) = P(m, n) * Factor;
                Pd(m, n) = Pd(m, n) * Factor;
            }
        }
    }

    return std::make_pair(P, Pd);
}