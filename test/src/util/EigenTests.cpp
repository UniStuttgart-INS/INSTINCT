// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EigenTests.cpp
/// @brief Tests for the Eigen library defined functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-06-21

#include <catch2/catch_test_macros.hpp>

#include "util/Eigen.hpp"
#include "Logger.hpp"

namespace NAV::TESTS
{

TEST_CASE("[Eigen] removeRows Matrix", "[Eigen][Debug]")
{
    auto logger = initializeTestLogger();

    Eigen::MatrixXi m = Eigen::MatrixXi(8, 2);
    m << 0, 1,
        2, 3,
        4, 5,
        6, 7,
        8, 9,
        10, 11,
        12, 13,
        14, 15;
    LOG_TRACE("m:\n{}", m);

    removeRows(m, 1, 1);
    LOG_TRACE("m (2nd row removed):\n{}", m);
    Eigen::MatrixXi r = Eigen::MatrixXi(7, 2);
    r << 0, 1,
        4, 5,
        6, 7,
        8, 9,
        10, 11,
        12, 13,
        14, 15;
    REQUIRE(m == r);

    removeRows(m, 3, 2);
    LOG_TRACE("m (4th and 5th rows removed):\n{}", m);
    r = Eigen::MatrixXi(5, 2);
    r << 0, 1,
        4, 5,
        6, 7,
        12, 13,
        14, 15;
    REQUIRE(m == r);

    removeRows(m, 0, 1);
    LOG_TRACE("m (1st row removed):\n{}", m);
    r = Eigen::MatrixXi(4, 2);
    r << 4, 5,
        6, 7,
        12, 13,
        14, 15;
    REQUIRE(m == r);

    removeRows(m, 3, 1);
    LOG_TRACE("m (last row removed):\n{}", m);
    r = Eigen::MatrixXi(3, 2);
    r << 4, 5,
        6, 7,
        12, 13;
    REQUIRE(m == r);
}

TEST_CASE("[Eigen] removeRows Vector", "[Eigen]")
{
    auto logger = initializeTestLogger();

    Eigen::VectorXi v = Eigen::VectorXi(4);
    v << 0,
        1,
        2,
        3;
    LOG_TRACE("v:\n{}", v);

    removeRows(v, 1, 1);
    LOG_TRACE("v (2nd row removed):\n{}", v);
    Eigen::VectorXi r = Eigen::VectorXi(3);
    r << 0,
        2,
        3;
    REQUIRE(v == r);

    removeRows(v, 1, 2);
    LOG_TRACE("m (2nd and 3rd rows removed):\n{}", v);
    r = Eigen::VectorXi(1);
    r << 0;
    REQUIRE(v == r);
}

TEST_CASE("[Eigen] removeCols Matrix", "[Eigen]")
{
    auto logger = initializeTestLogger();

    Eigen::MatrixXi m = Eigen::MatrixXi(2, 8);
    m << 0, 1, 2, 3, 4, 5, 6, 7,
        8, 9, 10, 11, 12, 13, 14, 15;
    LOG_TRACE("m:\n{}", m);

    removeCols(m, 1, 1);
    LOG_TRACE("m (2nd row removed):\n{}", m);
    Eigen::MatrixXi r = Eigen::MatrixXi(2, 7);
    r << 0, 2, 3, 4, 5, 6, 7,
        8, 10, 11, 12, 13, 14, 15;
    REQUIRE(m == r);

    removeCols(m, 3, 2);
    LOG_TRACE("m (4th and 5th rows removed):\n{}", m);
    r = Eigen::MatrixXi(2, 5);
    r << 0, 2, 3, 6, 7,
        8, 10, 11, 14, 15;
    REQUIRE(m == r);

    removeCols(m, 0, 1);
    LOG_TRACE("m (1st row removed):\n{}", m);
    r = Eigen::MatrixXi(2, 4);
    r << 2, 3, 6, 7,
        10, 11, 14, 15;
    REQUIRE(m == r);

    removeCols(m, 3, 1);
    LOG_TRACE("m (last row removed):\n{}", m);
    r = Eigen::MatrixXi(2, 3);
    r << 2, 3, 6,
        10, 11, 14;
    REQUIRE(m == r);
}

TEST_CASE("[Eigen] removeCols RowVector", "[Eigen]")
{
    auto logger = initializeTestLogger();

    Eigen::RowVectorXi v = Eigen::RowVectorXi(4);
    v << 0, 1, 2, 3;
    LOG_TRACE("v:\n{}", v);

    removeCols(v, 1, 1);
    LOG_TRACE("v (2nd row removed):\n{}", v);
    Eigen::RowVectorXi r = Eigen::RowVectorXi(3);
    r << 0, 2, 3;
    REQUIRE(v == r);

    removeCols(v, 1, 2);
    LOG_TRACE("m (2nd and 3rd rows removed):\n{}", v);
    r = Eigen::RowVectorXi(1);
    r << 0;
    REQUIRE(v == r);
}

TEST_CASE("[Eigen] removeRowsAndCols", "[Eigen]")
{
    auto logger = initializeTestLogger();

    Eigen::MatrixXi m = Eigen::MatrixXi(4, 8);
    m << 0, 1, 2, 3, 4, 5, 6, 7,
        8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29, 30, 31;
    LOG_TRACE("m:\n{}", m);

    removeRowsAndCols(m, 1, 1, 2, 1);
    LOG_TRACE("m (2nd row and 3rd col removed):\n{}", m);
    Eigen::MatrixXi r = Eigen::MatrixXi(3, 7);
    r << 0, 1, 3, 4, 5, 6, 7,
        16, 17, 19, 20, 21, 22, 23,
        24, 25, 27, 28, 29, 30, 31;
    REQUIRE(m == r);

    removeRowsAndCols(m, 0, 2, 3, 4);
    LOG_TRACE("m (1st-2nd row and 4th-7th cols removed):\n{}", m);
    r = Eigen::MatrixXi(1, 3);
    r << 24, 25, 27;
    REQUIRE(m == r);

    removeRowsAndCols(m, 0, 0, 0, 2);
    LOG_TRACE("m (1st-2nd cols removed):\n{}", m);
    r = Eigen::MatrixXi(1, 1);
    r << 27;
    REQUIRE(m == r);

    removeRowsAndCols(m, 0, 0, 0, 1);
    LOG_TRACE("m (last element removed):\n{}", m);
    REQUIRE(m.rows() == 1);
    REQUIRE(m.cols() == 0);
}

} // namespace NAV::TESTS