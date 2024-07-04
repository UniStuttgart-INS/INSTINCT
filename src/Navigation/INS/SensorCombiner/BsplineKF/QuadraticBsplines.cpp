// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "QuadraticBsplines.hpp"

#include <cmath>
#include <array>

#include "util/Logger.hpp"

std::array<double, 3> NAV::BsplineKF::quadraticBsplines(const double& ti, const double& splineSpacing)
{
    std::array<double, 3> qBsplines{ 0.0, 0.0, 0.0 }; // Stacked B-spline, which consists of three single B-splines
    std::array<double, 4> knots{};                    // Knot vector for a single quadratic B-spline

    for (size_t splineIterator = 0; splineIterator < 3; splineIterator++)
    {
        if (ti >= 2.0 * splineSpacing)
        {
            knots.at(3) = ti - std::fmod(ti, splineSpacing) + splineSpacing * static_cast<double>(splineIterator + 1); // t_k+1
            knots.at(2) = knots.at(3) - splineSpacing;                                                                 // t_k
            knots.at(1) = knots.at(3) - 2.0 * splineSpacing;                                                           // t_k-1
            knots.at(0) = knots.at(3) - 3.0 * splineSpacing;                                                           // t_k-2
        }
        else if ((ti >= splineSpacing) && (ti < 2.0 * splineSpacing))
        {
            switch (splineIterator)
            {
            case 0:
                knots = { 0.0, 0.0, 1.0 * splineSpacing, 2.0 * splineSpacing };
                break;
            case 1:
                knots = { 0.0, 1.0 * splineSpacing, 2.0 * splineSpacing, 3.0 * splineSpacing };
                break;
            case 2:
                knots = { 1.0 * splineSpacing, 2.0 * splineSpacing, 3.0 * splineSpacing, 4.0 * splineSpacing };
                break;

            default:
                break;
            }
        }
        else if (ti < 1.0 * splineSpacing)
        {
            switch (splineIterator)
            {
            case 0:
                knots = { 0.0, 0.0, 0.0, 1.0 * splineSpacing };
                break;
            case 1:
                knots = { 0.0, 0.0, 1.0 * splineSpacing, 2.0 * splineSpacing };
                break;
            case 2:
                knots = { 0.0, 1.0 * splineSpacing, 2.0 * splineSpacing, 3.0 * splineSpacing };
                break;

            default:
                break;
            }
        }

        LOG_DATA("ti = {}, B-spline '{}', knots = {}, {}, {}, {}", ti, splineIterator, knots.at(0), knots.at(1), knots.at(2), knots.at(3));

        // B-spline of degree 0
        std::array<double, 3> N0{};
        if ((knots.at(0) <= ti) && (ti < knots.at(1)))
        {
            N0.at(0) = 1.0;
        }
        else if ((knots.at(1) <= ti) && (ti < knots.at(2)))
        {
            N0.at(1) = 1.0;
        }
        else if ((knots.at(2) <= ti) && (ti < knots.at(3)))
        {
            N0.at(2) = 1.0;
        }

        // B-spline of degree 1
        std::array<double, 2> N1{};
        double N11_factor1{};
        double N11_factor2{};
        double N12_factor1{};
        double N12_factor2{};

        if ((knots.at(0) <= ti) && (ti < knots.at(2)))
        {
            if (knots.at(1) - knots.at(0) != 0.0)
            {
                N11_factor1 = (ti - knots.at(0)) / (knots.at(1) - knots.at(0));
            }
            if (knots.at(2) - knots.at(1) != 0.0)
            {
                N11_factor2 = (knots.at(2) - ti) / (knots.at(2) - knots.at(1));
            }
            N1.at(0) = N11_factor1 * N0.at(0) + N11_factor2 * N0.at(1);
        }
        if ((knots.at(1) <= ti) && (ti < knots.at(3)))
        {
            if (knots.at(2) - knots.at(1) != 0.0)
            {
                N12_factor1 = (ti - knots.at(1)) / (knots.at(2) - knots.at(1));
            }
            if (knots.at(3) - knots.at(2) != 0.0)
            {
                N12_factor2 = (knots.at(3) - ti) / (knots.at(3) - knots.at(2));
            }
            N1.at(1) = N12_factor1 * N0.at(1) + N12_factor2 * N0.at(2);
        }

        // B-spline of degree 2 (i.e. quadratic)
        double N2_factor1{};
        double N2_factor2{};

        if (knots.at(2) - knots.at(0) != 0.0)
        {
            N2_factor1 = (ti - knots.at(0)) / (knots.at(2) - knots.at(0));
        }
        if (knots.at(3) - knots.at(1) != 0.0)
        {
            N2_factor2 = (knots.at(3) - ti) / (knots.at(3) - knots.at(1));
        }
        qBsplines.at(splineIterator) = N2_factor1 * N1.at(0) + N2_factor2 * N1.at(1);
    }

    return qBsplines;
}