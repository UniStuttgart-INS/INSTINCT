// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PolynomialRegressor.hpp"

namespace NAV
{

const char* to_string(PolynomialRegressor<>::Strategy strategy)
{
    switch (strategy)
    {
    case PolynomialRegressor<>::Strategy::IncrementalLeastSquares:
        return "Incremental Least Squares";
    case PolynomialRegressor<>::Strategy::LeastSquares:
        return "Least Squares";
    case PolynomialRegressor<>::Strategy::HouseholderQR:
        return "Householder QR";
    case PolynomialRegressor<>::Strategy::BDCSVD:
        return "Bidiagonal Divide and Conquer SVD";
    case PolynomialRegressor<>::Strategy::COD:
        return "Complete Orthogonal Decomposition";
    case PolynomialRegressor<>::Strategy::COUNT:
        return "";
    }
    return "";
}

} // namespace NAV