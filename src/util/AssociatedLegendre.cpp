#include "AssociatedLegendre.hpp"

#include "InsConstants.hpp"
#include "InsMath.hpp"
#include <cmath>

Eigen::ArrayXXd associatedLegendre(int N, double x)
{
    Eigen::ArrayXXd P = Eigen::ArrayXXd::Zero(N, N);
    Eigen::ArrayXXd Pd = Eigen::ArrayXXd::Zero(N, N);
    Eigen::ArrayXXd full_derivs = Eigen::ArrayXXd::Zero(N + 1, N);

    // Compute P^n_0^M
    // [L, Ld] = legendre_order0(N,x);
    Eigen::ArrayXd L = Eigen::ArrayXd::Zero(N).transpose();
    Eigen::ArrayXd Ld = Eigen::ArrayXd::Zero(N).transpose();

    L(0) = 1;
    L(1) = x;
    Ld(0) = 0;
    Ld(1) = 1;

    for (int i = 2; i < N - 1; i++)
    {
        int n = i - 1;
        L(i) = ((2 * n + 1) * x * L(i - 1) - n * L(i - 2)) / (n + 1);
        Ld(i) = ((2 * n + 1) * (L(1, i - 1) + x * Ld(1, i - 1)) - n * Ld(1, i - 2)) / (n + 1);
    }

    P.row(0) = L;
    Pd.row(0) = Ld;

    full_derivs.row(0) = Pd.row(0);

    for (int j = 1; j < N; j++)
    {
        for (int k = 2; k < N - 1; k++)
        {
            int m = k - 1;
            full_derivs(j, k) = ((2 * m + 1) * (j * full_derivs(j - 1, k - 1) + x * full_derivs(j, k - 1)) - m * full_derivs(j, k - 2)) / (m + 1);
        }
    }

    // Compute P^m_n^M
    for (int mm = 1; mm < N - 1; mm++)
    {
        for (int nn = mm; nn < N - 1; nn++)
        {
            double factor = std::pow(-1.0, (static_cast<double>(mm) - 1.0));

            P(mm, nn) = factor * std::pow((1.0 - std::pow(x, 2.0)), ((static_cast<double>(mm) - 1.0) / 2.0)) * full_derivs(mm - 1, nn);

            Pd(mm, nn) = -factor * ((static_cast<double>(mm) - 1.0) / 2.0) * std::pow((1.0 - std::pow(x, 2.0)), ((((static_cast<double>(mm) - 1.0) / 2.0) - 1.0))) * 2.0 * x
                         + factor * std::pow(1.0 - std::pow(x, 2.0), (((static_cast<double>(mm) - 1.0) / 2.0))) * full_derivs(mm, nn);

            // Pd(m,n) = -factor*((m-1)/2)*(1 - x^2)^(((m-1)/2)-1)*2*x + ...
            //     factor*(1-x^2)^((m-1)/2)*full_derivs(m,n);
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

                P(m + 1, n + 1) = P(m + 1, n + 1) * Factor;
                Pd(m + 1, n + 1) = Pd(m + 1, n + 1) * Factor;
            }
        }
    }

    Eigen::ArrayXXd P2 = Eigen::ArrayXXd::Zero(N, 2 * N);
    P2 << P, Pd;

    return P2;
}