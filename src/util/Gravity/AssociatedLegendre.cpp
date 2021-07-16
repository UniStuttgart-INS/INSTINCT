#include "AssociatedLegendre.hpp"
#include "util/Logger.hpp"

#include "util/InsConstants.hpp"
#include "util/InsMath.hpp"
#include <cmath>
#include <array>

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> NAV::util::gravity::associatedLegendre(int ndegree, double theta)
{
    // Index needed for the calculation of all necessary 'Pd' entries up to 'ndegree' (--> Pd(n = ndegree, m = ndegree) is dependent on P(n = ndegree, m = ndegree + 1))
    int N = ndegree + 2;

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd Pd = Eigen::MatrixXd::Zero(N, N);

    // Recurrence relations for the normalized associated Legendre functions (see 'GUT User Guide' eq. 4.2.2 and eq. 4.2.3)
    P(0, 0) = 1.0;
    P(1, 0) = std::sqrt(3.0) * std::cos(theta);
    P(1, 1) = std::sqrt(3.0) * std::sin(theta);

    for (int n = 2; n <= N - 1; n++)
    {
        auto nd = static_cast<double>(n);
        P(n, n) = std::sqrt((2.0 * nd + 1.0) / (2.0 * nd)) * std::sin(theta) * P(n - 1, n - 1);

        for (int m = 0; m <= n; m++)
        {
            auto md = static_cast<double>(m);

            if (n == m + 1)
            {
                P(n, m) = std::sqrt(((2.0 * nd + 1.0) * (2.0 * nd - 1.0)) / ((nd + md) * (nd - md))) * std::cos(theta) * P(n - 1, m);
            }
            else if (n > m + 1)
            {
                P(n, m) = std::sqrt(((2.0 * nd + 1.0) * (2.0 * nd - 1.0)) / ((nd + md) * (nd - md))) * std::cos(theta) * P(n - 1, m) 
                - sqrt(((2.0 * nd + 1.0) * (nd + md - 1.0) * (nd - md - 1.0)) / ((2.0 * nd - 3.0) * (nd + md) * (nd - md))) * std::cos(theta) * P(n - 2, m);
            }
        }
    }

    // Recurrence relations for the derivative of the normalized associated Legendre functions (see 'GUT User Guide' eq. 4.2.6)
    Pd(0, 0) = 0.0;
    Pd(1, 0) = - std::sqrt(3.0) * std::sin(theta);
    Pd(1, 1) = std::sqrt(3.0) * std::cos(theta);

    for (int n = 2; n <= N - 1; n++) // 2nd for-loop is necessary, because for the calculation of 'Pd', all coefficients of 'P' must be available
    {
        auto nd = static_cast<double>(n);

        Pd(n, 0) = -0.5 * std::sqrt(2.0 * nd * (nd + 1.0)) * P(n, 1);
        Pd(n, 1) = 0.5 * (std::sqrt(2.0 * nd * (nd + 1.0)) * P(n, 0) - std::sqrt((nd - 1.0) * (nd + 2.0)) * std::pow(P(n, 2), 2.0));

        for (int m = 2; m <= n - 1; m++)
        {
            auto md = static_cast<double>(m);

            // else if ((m > 1) && (m < N - 1))
            Pd(n, m) = 0.5 * (std::sqrt((nd + md) * (nd - md + 1.0)) * P(n, m - 1) - std::sqrt((nd - md) * (nd + md + 1.0)) * P(n, m + 1));
        }
    }

    return std::make_pair(P, Pd);;
}