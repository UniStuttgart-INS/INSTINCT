// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Math.hpp"

namespace NAV::math
{

uint64_t factorial(uint64_t n)
{
    // uint64_t is required to calculate factorials of n > 12 (Limit of uint32_t). The limit of uint64_t is at n = 20
    constexpr std::array factorials = {
        uint64_t(1),                   // 0
        uint64_t(1),                   // 1
        uint64_t(2),                   // 2
        uint64_t(6),                   // 3
        uint64_t(24),                  // 4
        uint64_t(120),                 // 5
        uint64_t(720),                 // 6
        uint64_t(5040),                // 7
        uint64_t(40320),               // 8
        uint64_t(362880),              // 9
        uint64_t(3628800),             // 10
        uint64_t(39916800),            // 11
        uint64_t(479001600),           // 12
        uint64_t(6227020800),          // 13
        uint64_t(87178291200),         // 14
        uint64_t(1307674368000),       // 15
        uint64_t(20922789888000),      // 16
        uint64_t(355687428096000),     // 17
        uint64_t(6402373705728000),    // 18
        uint64_t(121645100408832000),  // 19
        uint64_t(2432902008176640000), // 20
    };

    if (n < factorials.size())
    {
        return factorials.at(n);
    }
    return n * factorial(n - 1);
}

double normalCDF(double value)
{
    return 0.5 * std::erfc(-value * M_SQRT1_2);
}

double calcEllipticalIntegral(double phi, double m)
{
    double origM = m;
    double scale = 0.0;
    if (m > 1.0)
    {
        scale = std::sqrt(m);
        phi = std::asin(scale * std::sin(phi));
        m = 1.0 / m;
    }
    else if (m < 0.0)
    {
        scale = std::sqrt(1.0 - m);
        phi = 0.5 * M_PI - phi;
        m /= m - 1.0;
    }
    double a = 0.0;
    double eok = 0.0;
    double f = 0.0;
    if (m == 1.0)
    {
        double per = std::floor((phi + 0.5 * M_PI) / M_PI);
        phi = std::sin(phi) * math::sgn(0.5 - std::abs(std::fmod(per, 2.0))) + 2 * per;
        a = 0.0;
    }
    else
    { // compute using arithmetic-geometric mean
        double sgn = math::sgn(phi);
        phi = std::abs(phi),
        a = 1.0;
        double b = std::sqrt(1.0 - m);
        eok = 1.0 - 0.5 * m;
        double cs = 0.0;
        double twon = 1.0;
        double pi2 = 0.5 * M_PI;
        while (true)
        { // maximum of 8 passes for 64-bit double
            double c = 0.5 * (a - b);
            if (std::abs(c) <= std::numeric_limits<double>::epsilon() * c) { break; }
            phi += std::atan((b / a) * std::tan(phi)) + M_PI * std::floor((phi + pi2) / M_PI);
            cs += c * std::sin(phi);
            eok -= twon * c * c;
            twon *= 2.0;
            double am = a - c;
            if (am == a) { break; }
            b = std::sqrt(a * b);
            a = am;
        }
        double f = sgn * phi / (twon * a);
        phi = eok * f + sgn * cs;
    }

    if (origM > 1.0)
    {
        phi = (phi - (1.0 - m) * f) / scale;
    }
    else if (origM < 0.0)
    {
        phi = (eok * 0.5 * M_PI / a - phi) * scale;
    }

    return phi;
}
} // namespace NAV::math