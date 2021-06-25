#include "AssociatedLegendre.hpp"
#include "util/Logger.hpp"

#include "util/InsConstants.hpp"
#include "util/InsMath.hpp"
#include <cmath>
#include <array>

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> NAV::util::gravity::associatedLegendre(int N, double x)
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

    for (int i = 2; i <= N - 1; i++)
    {
        auto n = static_cast<double>(i - 1);
        L(i) = ((2.0 * n + 1.0) * x * L(i - 1) - n * L(i - 2)) / (n + 1.0);
        Ld(i) = ((2.0 * n + 1.0) * (L(i - 1) + x * Ld(i - 1)) - n * Ld(i - 2)) / (n + 1.0);
    }
    LOG_DATA("First rows of the Associated Legendre Polynomial Coefficient Matrix and its derivative:\nL =\n{}\nLd =\n{}", L.transpose(), Ld.transpose());

    P.row(0) = L.transpose();
    Pd.row(0) = Ld.transpose();

    full_derivs.row(0) = Pd.row(0);

    for (int j = 1; j <= N; j++)
    {
        double jj = static_cast<double>(j) + 1.0;
        for (int k = 2; k <= N - 1; k++)
        {
            auto m = static_cast<double>(k - 1);
            full_derivs(j, k) = ((2.0 * m + 1.0) * (jj * full_derivs(j - 1, k - 1) + x * full_derivs(j, k - 1)) - m * full_derivs(j, k - 2)) / (m + 1.0);
        }
    }
    LOG_DATA("Derivative matrix 'full_derivs' of the Associated Legendre Polynomials =\n{}", full_derivs);

    for (int m = 1; m <= N - 1; m++)
    {
        double mm = static_cast<double>(m) + 1.0;
        for (int n = m; n <= N - 1; n++)
        {
            P(m, n) = std::pow((1.0 - std::pow(x, 2.0)), ((mm - 1.0) / 2.0)) * full_derivs(m - 1, n);

            Pd(m, n) = -((mm - 1.0) / 2.0) * std::pow((1.0 - std::pow(x, 2.0)), (((mm - 1.0) / 2.0) - 1.0)) * 2.0 * x
                       + std::pow((1.0 - std::pow(x, 2.0)), (mm - 1.0) / 2.0) * full_derivs(m, n);
        }
    }
    LOG_DATA("Associated Legendre Polynomial coefficients:\nP =\n{}\nPd =\n{}", P, Pd);

    // Normalize values
    auto Nlong = static_cast<uint64_t>(N);

    for (uint64_t n = 0; n <= Nlong - 1; n++)
    {
        for (uint64_t m = 0; m <= Nlong - 1; m++)
        {
            if (n >= m)
            {
                auto nn = static_cast<double>(n);

                // Normalization factor, consistent with equation (4.1.6) from "GUT User Guide" (https://earth.esa.int/documents/10174/1500266/GUT_UserGuide)
                double factor = (2.0 * nn + 1.0) * static_cast<double>(NAV::factorial(n - m)) / static_cast<double>(NAV::factorial(n + m));

                if (m != 0)
                {
                    factor *= 2.0;
                }

                auto mi = static_cast<int>(m);
                auto ni = static_cast<int>(n);

                P(mi, ni) *= std::sqrt(factor);
                Pd(mi, ni) *= std::sqrt(factor);
            }
        }
    }
    LOG_DATA("Associated Legendre Polynomial coefficients NORMALIZED:\nP =\n{}\nPd =\n{}", P, Pd);

    return std::make_pair(P, Pd);
}