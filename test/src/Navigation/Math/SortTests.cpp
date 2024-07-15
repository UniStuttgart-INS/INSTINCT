// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SortTests.cpp
/// @brief Test for sorting algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-15

#include <catch2/catch_test_macros.hpp>

#include "Logger.hpp"
#include "Navigation/Math/Sort.hpp"
#include "util/Eigen.hpp"

namespace NAV::TESTS
{

TEST_CASE("[Sort] Sort std::vector", "[Sort]")
{
    auto logger = initializeTestLogger();

    std::vector<int> vectorA{ 2, 1, 3, 4, 1 };
    std::vector<int> vectorB{ 1, 2, 3, 4, 5 };
    LOG_INFO("vectorA {}", fmt::join(vectorA, ", "));
    LOG_INFO("vectorB {}", fmt::join(vectorB, ", "));

    auto p = sort_permutation(vectorA, std::less<>{});
    LOG_INFO("permutation {}", fmt::join(p, ", "));

    std::vector<int> sortedVectorA = apply_permutation(vectorA, p);
    std::vector<int> sortedVectorB = apply_permutation(vectorB, p);

    LOG_INFO("vectorA {}", fmt::join(sortedVectorA, ", "));
    LOG_INFO("vectorB {}", fmt::join(sortedVectorB, ", "));

    REQUIRE(sortedVectorA == std::vector<int>{ 1, 1, 2, 3, 4 });
    REQUIRE(sortedVectorB == std::vector<int>{ 2, 5, 1, 3, 4 });

    LOG_INFO("vectorA {}", fmt::join(vectorA, ", "));
    LOG_INFO("vectorB {}", fmt::join(vectorB, ", "));

    apply_permutation_in_place(vectorA, p);
    apply_permutation_in_place(vectorB, p);

    REQUIRE(vectorA == sortedVectorA);
    REQUIRE(vectorB == sortedVectorB);
}

TEST_CASE("[Sort] Sort Eigen::Matrix", "[Sort]")
{
    auto logger = initializeTestLogger();

    Eigen::VectorXi vec = Eigen::VectorXi(5);
    vec << 2, 1, 3, 4, 1;
    Eigen::MatrixXi mat = Eigen::MatrixXi(5, 5);
    mat << 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10,
        11, 12, 13, 14, 15,
        16, 17, 18, 19, 20,
        21, 22, 23, 24, 25;
    LOG_INFO("vec {}", vec.transpose());
    LOG_INFO("mat\n{}", mat);

    auto p = sort_permutation(vec, std::less<>{});
    LOG_INFO("permutation {}", fmt::join(p, ", "));

    auto sortedVec = apply_permutation(vec, p);
    auto sortedMatRowwise = apply_permutation_rowwise(mat, p);
    auto sortedMatColwise = apply_permutation_colwise(mat, p);

    LOG_INFO("vec {}", sortedVec.transpose());
    LOG_INFO("mat (row-wise sorted)\n{}", sortedMatRowwise);
    LOG_INFO("mat (col-wise sorted)\n{}", sortedMatColwise);

    REQUIRE(sortedVec == (Eigen::VectorXi(5) << 1, 1, 2, 3, 4).finished());
    REQUIRE(sortedMatRowwise == (Eigen::MatrixXi(5, 5) << 6, 7, 8, 9, 10, //
                                 21, 22, 23, 24, 25,                      //
                                 1, 2, 3, 4, 5,                           //
                                 11, 12, 13, 14, 15,                      //
                                 16, 17, 18, 19, 20)
                                    .finished());

    REQUIRE(sortedMatColwise == (Eigen::MatrixXi(5, 5) << 2, 5, 1, 3, 4, //
                                 7, 10, 6, 8, 9,                         //
                                 12, 15, 11, 13, 14,                     //
                                 17, 20, 16, 18, 19,                     //
                                 22, 25, 21, 23, 24)
                                    .finished());

    Eigen::MatrixXi mat_col = mat;
    apply_permutation_in_place(vec, p);
    apply_permutation_rowwise_in_place(mat, p);
    apply_permutation_colwise_in_place(mat_col, p);

    LOG_INFO("vec {}", vec.transpose());
    LOG_INFO("mat (row-wise sorted in-place)\n{}", mat);
    LOG_INFO("mat (col-wise sorted in-place)\n{}", mat_col);

    REQUIRE(vec == sortedVec);
    REQUIRE(sortedMatRowwise == mat);
    REQUIRE(sortedMatColwise == mat_col);
}

} // namespace NAV::TESTS