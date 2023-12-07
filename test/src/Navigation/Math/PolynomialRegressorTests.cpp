// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PolynomialRegressorTests.cpp
/// @brief Tests for the polynomial fit algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-23

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include <random>
#include <chrono>

#include "Logger.hpp"
#include "Navigation/Math/PolynomialRegressor.hpp"

namespace NAV::TESTS
{

TEST_CASE("[PolynomialRegressor] More data than degree", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    std::vector<std::pair<double, double>> data = {
        { 1, 5 },
        { 2, 16 },
        { 3, 31 },
        { 4, 50 },
    };

    size_t polynomialDegree = 2;
    PolynomialRegressor<double> polynomial(polynomialDegree, data.size());
    for (const auto& d : data) { polynomial.push_back(d); }

    auto coeffs = polynomial.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs = {}", fmt::join(coeffs, ", "));
    REQUIRE(coeffs.rows() == 3);
    REQUIRE_THAT(coeffs(0), Catch::Matchers::WithinAbs(-2, 1e-12));
    REQUIRE_THAT(coeffs(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(coeffs(2), Catch::Matchers::WithinAbs(2, 1e-12));

    polynomial.reset();
    for (size_t i = 0; i < data.size() - 1; i++) { polynomial.push_back(data.at(i)); }

    coeffs = polynomial.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs = {}", fmt::join(coeffs, ", "));
    REQUIRE(coeffs.rows() == 3);
    REQUIRE_THAT(coeffs(0), Catch::Matchers::WithinAbs(-2, 1e-11));
    REQUIRE_THAT(coeffs(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(coeffs(2), Catch::Matchers::WithinAbs(2, 1e-12));
}

TEST_CASE("[PolynomialRegressor] Less data than degree", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    std::vector<std::pair<double, double>> data = {
        { 1, 5 },
        { 2, 16 },
    };

    size_t polynomialDegree = 2;
    PolynomialRegressor<double> polynomial(polynomialDegree, polynomialDegree + 1);
    for (const auto& d : data) { polynomial.push_back(d); }

    auto coeffs = polynomial.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs = {}", fmt::join(coeffs, ", "));
    REQUIRE(coeffs.rows() == 2);
    REQUIRE_THAT(coeffs(0), Catch::Matchers::WithinAbs(-6, 1e-12));
    REQUIRE_THAT(coeffs(1), Catch::Matchers::WithinAbs(11, 1e-12));

    polynomial.reset();
    for (size_t i = 0; i < data.size() - 1; i++) { polynomial.push_back(data.at(i)); }

    coeffs = polynomial.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs = {}", fmt::join(coeffs, ", "));
    REQUIRE(coeffs.rows() == 1);
    REQUIRE_THAT(coeffs(0), Catch::Matchers::WithinAbs(5, 1e-12));
}

TEST_CASE("[PolynomialRegressor] Data wrong order", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    std::vector<std::pair<double, double>> data = {
        { 0.50, 0.60 },
        { 1.00, 0.30 },
        { 0.75, 0.40 },
    };

    size_t polynomialDegree = 2;
    PolynomialRegressor<double> polynomial(polynomialDegree, data.size());
    for (const auto& d : data) { polynomial.push_back(d); }

    auto coeffs = polynomial.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs = {}", fmt::join(coeffs, ", "));
    REQUIRE(coeffs.rows() == 3);
    REQUIRE_THAT(coeffs(0), Catch::Matchers::WithinAbs(1.30, 1e-12));
    REQUIRE_THAT(coeffs(1), Catch::Matchers::WithinAbs(-1.80, 1e-12));
    REQUIRE_THAT(coeffs(2), Catch::Matchers::WithinAbs(0.80, 1e-12));
}

TEST_CASE("[PolynomialRegressor] Moving window", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    std::vector<std::pair<double, double>> data = {
        { -2, 2 },
        { -1, 2 },
        { 0, 2 },
        { 1, 5 },
        { 2, 16 },
        { 3, 31 },
        { 4, 50 },
        { 5, 73 },
    };

    size_t polynomialDegree = 2;
    PolynomialRegressor<double> polynomial(polynomialDegree, 4);
    for (const auto& d : data) { polynomial.push_back(d); }
    PolynomialRegressor<double> polynomial2(polynomialDegree, 4);
    for (size_t d = 4; d < data.size(); d++) { polynomial2.push_back(data.at(d)); }

    auto coeffs = polynomial.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs  = {}", fmt::join(coeffs, ", "));
    REQUIRE(coeffs.rows() == 3);
    REQUIRE_THAT(coeffs(0), Catch::Matchers::WithinAbs(-2, 1e-11));
    REQUIRE_THAT(coeffs(1), Catch::Matchers::WithinAbs(5, 1e-11));
    REQUIRE_THAT(coeffs(2), Catch::Matchers::WithinAbs(2, 1e-11));

    auto coeffs2 = polynomial2.calcPolynomial().coeffs();
    LOG_DEBUG("coeffs2 = {}", fmt::join(coeffs2, ", "));
    REQUIRE(coeffs == coeffs2);
}

TEST_CASE("[PolynomialRegressor] Strategy comparison (4 points)", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    std::vector<std::pair<double, double>> data = {
        { 1, 5 },
        { 2, 16 },
        { 3, 31 },
        { 4, 50 },
    };

    size_t polynomialDegree = 2;
    PolynomialRegressor<double> polynomialReg(polynomialDegree, data.size());

    polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::IncrementalLeastSquares);
    for (const auto& d : data) { polynomialReg.push_back(d); }
    Polynomial<double> polynomial = polynomialReg.calcPolynomial();

    LOG_DEBUG("{} (IncrementalLeastSquares)", polynomial);
    REQUIRE(polynomial.coeffs().rows() == 3);
    REQUIRE_THAT(polynomial.coeffs()(0), Catch::Matchers::WithinAbs(-2, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(2), Catch::Matchers::WithinAbs(2, 1e-12));

    polynomialReg.reset();

    polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::LeastSquares);
    for (const auto& d : data) { polynomialReg.push_back(d); }
    polynomial = polynomialReg.calcPolynomial();

    LOG_DEBUG("{} (LeastSquares)", polynomial);
    REQUIRE(polynomial.coeffs().rows() == 3);
    REQUIRE_THAT(polynomial.coeffs()(0), Catch::Matchers::WithinAbs(-2, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(2), Catch::Matchers::WithinAbs(2, 1e-12));

    polynomialReg.reset();

    polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::HouseholderQR);
    for (const auto& d : data) { polynomialReg.push_back(d); }
    polynomial = polynomialReg.calcPolynomial();

    LOG_DEBUG("{} (HouseholderQR)", polynomial);
    REQUIRE(polynomial.coeffs().rows() == 3);
    REQUIRE_THAT(polynomial.coeffs()(0), Catch::Matchers::WithinAbs(-2, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(2), Catch::Matchers::WithinAbs(2, 1e-12));

    polynomialReg.reset();

    polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::BDCSVD);
    for (const auto& d : data) { polynomialReg.push_back(d); }
    polynomial = polynomialReg.calcPolynomial();

    LOG_DEBUG("{} (BDCSVD)", polynomial);
    REQUIRE(polynomial.coeffs().rows() == 3);
    REQUIRE_THAT(polynomial.coeffs()(0), Catch::Matchers::WithinAbs(-2, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(2), Catch::Matchers::WithinAbs(2, 1e-12));

    polynomialReg.reset();

    polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::COD);
    for (const auto& d : data) { polynomialReg.push_back(d); }
    polynomial = polynomialReg.calcPolynomial();

    LOG_DEBUG("{} (COD)", polynomial);
    REQUIRE(polynomial.coeffs().rows() == 3);
    REQUIRE_THAT(polynomial.coeffs()(0), Catch::Matchers::WithinAbs(-2, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(1), Catch::Matchers::WithinAbs(5, 1e-12));
    REQUIRE_THAT(polynomial.coeffs()(2), Catch::Matchers::WithinAbs(2, 1e-12));
}

TEST_CASE("[PolynomialRegressor] Strategy comparison (large dataset)", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    std::vector<std::pair<double, double>> data = {
        { 0.0, 10.5032689944 },
        { 1.0, 10.4928402081 },
        { 2.0, 10.4927045479 },
        { 3.0, 10.5004613288 },
        { 4.0, 10.5014111623 },
        { 5.0, 10.5056036487 },
        { 6.0, 10.5000109971 },
        { 7.0, 10.4965119287 },
        { 8.0, 10.5055892728 },
        { 9.0, 10.4935342334 },
        { 10.0, 10.5011300072 },
        { 11.0, 10.5009524152 },
        { 12.0, 10.5076183379 },
        { 13.0, 10.4979062676 },
        { 14.0, 10.5000170507 },
        { 15.0, 10.5035446510 },
        { 16.0, 10.5041439869 },
        { 17.0, 10.5033495240 },
        { 18.0, 10.5022767372 },
        { 19.0, 10.5051276647 },
        { 20.0, 10.5106629096 },
        { 21.0, 10.5043378547 },
        { 22.0, 10.5036535375 },
        { 23.0, 10.5073335618 },
        { 24.0, 10.5083890110 },
        { 25.0, 10.5118709393 },
        { 26.0, 10.5078938305 },
        { 27.0, 10.5063267648 },
        { 28.0, 10.5127589665 },
        { 29.0, 10.5006655268 },
        { 30.0, 10.5058111250 },
        { 31.0, 10.5136427321 },
        { 32.0, 10.5109565817 },
        { 33.0, 10.5013761595 },
        { 34.0, 10.5053314604 },
        { 35.0, 10.5068425648 },
        { 36.0, 10.5075501278 },
        { 37.0, 10.5043748803 },
        { 38.0, 10.5112128183 },
        { 39.0, 10.5099457763 },
        { 40.0, 10.5099879242 },
        { 41.0, 10.5018393099 },
        { 42.0, 10.5140595064 },
        { 43.0, 10.5055635087 },
        { 44.0, 10.5078862198 },
        { 45.0, 10.5113004521 },
        { 46.0, 10.5056047849 },
        { 47.0, 10.5122571364 },
        { 48.0, 10.5072522014 },
        { 49.0, 10.5083993562 },
        { 50.0, 10.5100174025 },
        { 51.0, 10.5145371817 },
        { 52.0, 10.5037612841 },
        { 53.0, 10.5115943104 },
        { 54.0, 10.5080674663 },
        { 55.0, 10.5114356056 },
        { 56.0, 10.5154241659 },
        { 57.0, 10.5109262243 },
        { 58.0, 10.5176419020 },
        { 59.0, 10.5157067962 },
        { 60.0, 10.5129166767 },
        { 61.0, 10.5098213144 },
        { 62.0, 10.5193604380 },
        { 63.0, 10.5114216171 },
        { 64.0, 10.5135106109 },
        { 65.0, 10.5221199095 },
        { 66.0, 10.5173917934 },
        { 67.0, 10.5139755905 },
        { 68.0, 10.5162851848 },
        { 69.0, 10.5102055110 },
        { 70.0, 10.5240646526 },
        { 71.0, 10.5199046955 },
        { 72.0, 10.5149965174 },
        { 73.0, 10.5195890404 },
        { 74.0, 10.5209942870 },
        { 75.0, 10.5200884379 },
        { 76.0, 10.5244410895 },
        { 77.0, 10.5193013735 },
        { 78.0, 10.5250729173 },
        { 79.0, 10.5258727446 },
        { 80.0, 10.5150534026 },
        { 81.0, 10.5234391801 },
        { 82.0, 10.5168938860 },
        { 83.0, 10.5235537961 },
        { 84.0, 10.5208747908 },
        { 85.0, 10.5227243118 },
        { 86.0, 10.5258031264 },
        { 87.0, 10.5191617906 },
        { 88.0, 10.5241313390 },
        { 89.0, 10.5294549242 },
        { 90.0, 10.5258190967 },
        { 91.0, 10.5220969841 },
        { 92.0, 10.5267076790 },
        { 93.0, 10.5227898248 },
        { 94.0, 10.5243692435 },
        { 95.0, 10.5202084370 },
        { 96.0, 10.5276863016 },
        { 97.0, 10.5248416625 },
        { 98.0, 10.5290862173 },
        { 99.0, 10.5281934105 },
        { 100.0, 10.5266603790 },
        { 101.0, 10.5207190476 },
        { 102.0, 10.5235692486 },
        { 103.0, 10.5231023170 },
        { 104.0, 10.5309259146 },
        { 105.0, 10.5232605189 },
        { 106.0, 10.5251916572 },
        { 107.0, 10.5257811695 },
        { 108.0, 10.5261695981 },
        { 109.0, 10.5327762105 },
        { 110.0, 10.5264033265 },
        { 111.0, 10.5314692669 },
        { 112.0, 10.5288489982 },
        { 113.0, 10.5279104225 },
        { 114.0, 10.5252994150 },
        { 115.0, 10.5255643651 },
        { 116.0, 10.5336559638 },
        { 117.0, 10.5337851308 },
        { 118.0, 10.5259142853 },
        { 119.0, 10.5319501720 },
        { 120.0, 10.5267554484 },
        { 121.0, 10.5285950042 },
        { 122.0, 10.5324250124 },
        { 123.0, 10.5279713161 },
        { 124.0, 10.5321635343 },
        { 125.0, 10.5307828151 },
        { 126.0, 10.5311907753 },
        { 127.0, 10.5347132869 },
        { 128.0, 10.5289014019 },
        { 129.0, 10.5369367599 },
        { 130.0, 10.5380302593 },
        { 131.0, 10.5320403762 },
        { 132.0, 10.5335087553 },
        { 133.0, 10.5413393453 },
        { 134.0, 10.5285463743 },
        { 135.0, 10.5343633704 },
        { 136.0, 10.5451618806 },
        { 137.0, 10.5319402553 },
        { 138.0, 10.5393847972 },
        { 139.0, 10.5362127908 },
        { 140.0, 10.5282270350 },
        { 141.0, 10.5348960236 },
        { 142.0, 10.5384444818 },
        { 143.0, 10.5381324962 },
        { 144.0, 10.5365640372 },
        { 145.0, 10.5323141925 },
        { 146.0, 10.5371929258 },
        { 147.0, 10.5420931876 },
        { 148.0, 10.5378035009 },
        { 149.0, 10.5390377454 },
        { 150.0, 10.5426166691 },
        { 151.0, 10.5357807763 },
        { 152.0, 10.5419239923 },
        { 153.0, 10.5396200232 },
        { 154.0, 10.5337421186 },
        { 155.0, 10.5345193297 },
        { 156.0, 10.5422994085 },
        { 157.0, 10.5428866781 },
        { 158.0, 10.5351261012 },
        { 159.0, 10.5370303132 },
        { 160.0, 10.5456459969 },
        { 161.0, 10.5343741067 },
        { 162.0, 10.5435205773 },
        { 163.0, 10.5440203138 },
        { 164.0, 10.5439084470 },
        { 165.0, 10.5466758385 },
        { 166.0, 10.5443308279 },
        { 167.0, 10.5456208400 },
        { 168.0, 10.5436254814 },
        { 169.0, 10.5467885882 },
        { 170.0, 10.5553885177 },
        { 171.0, 10.5417865515 },
        { 172.0, 10.5442878678 },
        { 173.0, 10.5442121588 },
        { 174.0, 10.5408280417 },
        { 175.0, 10.5454369895 },
        { 176.0, 10.5450762138 },
        { 177.0, 10.5423196442 },
        { 178.0, 10.5509391055 },
        { 179.0, 10.5425947346 },
        { 180.0, 10.5426968113 },
        { 181.0, 10.5438769571 },
        { 182.0, 10.5501681715 },
        { 183.0, 10.5457795076 },
        { 184.0, 10.5518724099 },
        { 185.0, 10.5476269834 },
        { 186.0, 10.5419462137 },
        { 187.0, 10.5488168560 },
        { 188.0, 10.5388594158 },
        { 189.0, 10.5556934029 },
        { 190.0, 10.5463349968 },
        { 191.0, 10.5520945974 },
        { 192.0, 10.5471803434 },
        { 193.0, 10.5603190586 },
        { 194.0, 10.5503472425 },
        { 195.0, 10.5513014905 },
        { 196.0, 10.5531610362 },
        { 197.0, 10.5553473048 },
        { 198.0, 10.5550378487 },
        { 199.0, 10.5591683127 },
        { 200.0, 10.5456135534 },
        { 201.0, 10.5550811365 },
        { 202.0, 10.5491623208 },
        { 203.0, 10.5444624126 },
        { 204.0, 10.5522540323 },
        { 205.0, 10.5517374538 },
        { 206.0, 10.5596279986 },
        { 207.0, 10.5507931188 },
        { 208.0, 10.5481790043 },
        { 209.0, 10.5556670241 },
        { 210.0, 10.5550425276 },
        { 211.0, 10.5565815307 },
        { 212.0, 10.5556065254 },
        { 213.0, 10.5591129102 },
        { 214.0, 10.5547998473 },
        { 215.0, 10.5562797189 },
        { 216.0, 10.5543767512 },
        { 217.0, 10.5584973916 },
        { 218.0, 10.5601907335 },
        { 219.0, 10.5637963526 },
        { 220.0, 10.5581584759 },
        { 221.0, 10.5656860732 },
        { 222.0, 10.5583878532 },
        { 223.0, 10.5589205921 },
        { 224.0, 10.5516630486 },
        { 225.0, 10.5623439215 },
        { 226.0, 10.5547195300 },
        { 227.0, 10.5593002923 },
        { 228.0, 10.5589334704 },
        { 229.0, 10.5630582534 },
        { 230.0, 10.5621402860 },
        { 231.0, 10.5599614903 },
        { 232.0, 10.5662039146 },
        { 233.0, 10.5570939183 },
        { 234.0, 10.5659355670 },
        { 235.0, 10.5668665990 },
        { 236.0, 10.5646536201 },
        { 237.0, 10.5589958988 },
        { 238.0, 10.5542539321 },
        { 239.0, 10.5667048059 },
        { 240.0, 10.5598691814 },
        { 241.0, 10.5620691739 },
        { 242.0, 10.5619990565 },
        { 243.0, 10.5664967075 },
        { 244.0, 10.5685483702 },
        { 245.0, 10.5696316808 },
        { 246.0, 10.5619454123 },
        { 247.0, 10.5657440685 },
        { 248.0, 10.5645256601 },
        { 249.0, 10.5637520850 },
        { 250.0, 10.5726367868 },
        { 251.0, 10.5640617050 },
        { 252.0, 10.5746481232 },
        { 253.0, 10.5621791892 },
        { 254.0, 10.5639520548 },
        { 255.0, 10.5625511147 },
        { 256.0, 10.5666177757 },
        { 257.0, 10.5669945180 },
        { 258.0, 10.5648168847 },
        { 259.0, 10.5695892237 },
        { 260.0, 10.5665089302 },
        { 261.0, 10.5694743432 },
        { 262.0, 10.5707400702 },
        { 263.0, 10.5685266107 },
        { 264.0, 10.5671695322 },
        { 265.0, 10.5663882680 },
        { 266.0, 10.5726386681 },
        { 267.0, 10.5614060760 },
        { 268.0, 10.5691725574 },
        { 269.0, 10.5747917145 },
        { 270.0, 10.5743685998 },
        { 271.0, 10.5717129260 },
        { 272.0, 10.5715693384 },
        { 273.0, 10.5792337619 },
        { 274.0, 10.5753057674 },
        { 275.0, 10.5810578428 },
        { 276.0, 10.5719887689 },
        { 277.0, 10.5720598474 },
        { 278.0, 10.5749899261 },
        { 279.0, 10.5747611374 },
        { 280.0, 10.5702232979 },
        { 281.0, 10.5749219134 },
        { 282.0, 10.5706613734 },
        { 283.0, 10.5791934095 },
        { 284.0, 10.5759138092 },
        { 285.0, 10.5773310289 },
        { 286.0, 10.5698356144 },
        { 287.0, 10.5727249980 },
        { 288.0, 10.5817559026 },
        { 289.0, 10.5800062828 },
        { 290.0, 10.5767600909 },
        { 291.0, 10.5742673054 },
        { 292.0, 10.5733374022 },
        { 293.0, 10.5832290165 },
        { 294.0, 10.5733099431 },
        { 295.0, 10.5846894272 },
        { 296.0, 10.5794668011 },
        { 297.0, 10.5776550733 },
        { 298.0, 10.5795795210 },
        { 299.0, 10.5823707879 },
        { 300.0, 10.5888541006 },
        { 301.0, 10.5749823228 },
        { 302.0, 10.5798938833 },
        { 303.0, 10.3854427077 },
        { 304.0, 10.3919379488 },
        { 305.0, 10.3890500888 },
        { 306.0, 10.3941000737 },
        { 307.0, 10.3942912333 },
        { 308.0, 10.3933735751 },
        { 309.0, 10.3985592127 },
        { 310.0, 10.3914649524 },
        { 311.0, 10.3993737139 },
        { 312.0, 10.3892258704 },
        { 313.0, 10.3937487453 },
        { 314.0, 10.3966500424 },
        { 315.0, 10.3931962773 },
        { 316.0, 10.3916014992 },
        { 317.0, 10.3960101232 },
        { 318.0, 10.3918777704 },
        { 319.0, 10.3977103569 },
        { 320.0, 10.3979815170 },
        { 321.0, 10.3975647911 },
        { 322.0, 10.3976561800 },
        { 323.0, 10.3943507522 },
        { 324.0, 10.3951297104 },
        { 325.0, 10.4022538997 },
        { 326.0, 10.3949313499 },
        { 327.0, 10.3958596028 },
        { 328.0, 10.4001895562 },
        { 329.0, 10.3975437284 },
        { 330.0, 10.3896588720 },
        { 331.0, 10.3994059898 },
        { 332.0, 10.4014380276 },
        { 333.0, 10.4050353579 },
        { 334.0, 10.3996415287 },
        { 335.0, 10.3902341239 },
        { 336.0, 10.3945939541 },
        { 337.0, 10.4001749009 },
        { 338.0, 10.4033149779 },
        { 339.0, 10.4035278261 },
        { 340.0, 10.3944557123 },
        { 341.0, 10.4037359655 },
        { 342.0, 10.3977415822 },
        { 343.0, 10.3914306685 },
        { 344.0, 10.4021390304 },
        { 345.0, 10.4062121436 },
        { 346.0, 10.3958472423 },
        { 347.0, 10.3996867314 },
        { 348.0, 10.4063025936 },
        { 349.0, 10.3995028883 },
        { 350.0, 10.3967808597 },
        { 351.0, 10.3967749253 },
        { 352.0, 10.4013828561 },
        { 353.0, 10.4078625329 },
        { 354.0, 10.3982222639 },
        { 355.0, 10.3970641941 },
        { 356.0, 10.4012084454 },
        { 357.0, 10.4116339050 },
        { 358.0, 10.3977418020 },
        { 359.0, 10.4084262252 },
        { 360.0, 10.4075046629 },
        { 361.0, 10.4044795558 },
        { 362.0, 10.4079987369 },
        { 363.0, 10.4019902460 },
        { 364.0, 10.4031293727 },
        { 365.0, 10.4102333747 },
        { 366.0, 10.4059330001 },
        { 367.0, 10.4018243775 },
        { 368.0, 10.4045475312 },
        { 369.0, 10.4113876820 },
        { 370.0, 10.4119842611 },
        { 371.0, 10.4047385566 },
        { 372.0, 10.4023888968 },
        { 373.0, 10.4019795991 },
        { 374.0, 10.4135388359 },
        { 375.0, 10.4149540104 },
        { 376.0, 10.4072021917 },
        { 377.0, 10.4048682190 },
        { 378.0, 10.4049825184 },
        { 379.0, 10.4077568986 },
        { 380.0, 10.4100747928 },
        { 381.0, 10.4104554802 },
        { 382.0, 10.4173067510 },
        { 383.0, 10.4056121297 },
        { 384.0, 10.4205765985 },
        { 385.0, 10.4125868902 },
        { 386.0, 10.4138652161 },
        { 387.0, 10.4148098193 },
        { 388.0, 10.4186798856 },
        { 389.0, 10.4135450609 },
        { 390.0, 10.4108974859 },
        { 391.0, 10.4064713940 },
        { 392.0, 10.4191489592 },
        { 393.0, 10.4172309153 },
        { 394.0, 10.4132155254 },
        { 395.0, 10.4177219383 },
        { 396.0, 10.4139028750 },
        { 397.0, 10.4177007303 },
        { 398.0, 10.4244098514 },
        { 399.0, 10.4184322022 },
        { 400.0, 10.4213573895 },
        { 401.0, 10.4186656065 },
        { 402.0, 10.4188943207 },
        { 403.0, 10.4160984792 },
        { 404.0, 10.4260827303 },
        { 405.0, 10.4167673327 },
        { 406.0, 10.4169332236 },
        { 407.0, 10.4266052395 },
        { 408.0, 10.4281577319 },
        { 409.0, 10.4180361666 },
        { 410.0, 10.4262116365 },
        { 411.0, 10.4187667631 },
        { 412.0, 10.4194025658 },
        { 413.0, 10.4290844090 },
        { 414.0, 10.4256849661 },
        { 415.0, 10.4162866101 },
        { 416.0, 10.4161891416 },
        { 417.0, 10.4188968763 },
        { 418.0, 10.4249301292 },
        { 419.0, 10.4214342535 },
        { 420.0, 10.4190918915 },
        { 421.0, 10.4201078750 },
        { 422.0, 10.4279951453 },
        { 423.0, 10.4198212698 },
        { 424.0, 10.4176924080 },
        { 425.0, 10.4330035523 },
        { 426.0, 10.4256925732 },
        { 427.0, 10.4238678068 },
        { 428.0, 10.4303324819 },
        { 429.0, 10.4231552631 },
        { 430.0, 10.4308313355 },
        { 431.0, 10.4209790975 },
        { 432.0, 10.4239300601 },
        { 433.0, 10.4261236787 },
        { 434.0, 10.4294014052 },
        { 435.0, 10.4267484844 },
        { 436.0, 10.4353080243 },
        { 437.0, 10.4247830324 },
        { 438.0, 10.4223557748 },
        { 439.0, 10.4281543978 },
        { 440.0, 10.4290752858 },
        { 441.0, 10.4307258539 },
        { 442.0, 10.4331776425 },
        { 443.0, 10.4335122779 },
        { 444.0, 10.4341250062 },
        { 445.0, 10.4305874370 },
        { 446.0, 10.4321896285 },
        { 447.0, 10.4330531098 },
        { 448.0, 10.4347455204 },
        { 449.0, 10.4394368865 },
        { 450.0, 10.4312270656 },
        { 451.0, 10.4286790974 },
        { 452.0, 10.4271251149 },
        { 453.0, 10.4352304786 },
        { 454.0, 10.4313647449 },
        { 455.0, 10.4268958755 },
        { 456.0, 10.4399188347 },
        { 457.0, 10.4350450300 },
        { 458.0, 10.4357112870 },
        { 459.0, 10.4423245043 },
        { 460.0, 10.4384165034 },
        { 461.0, 10.4295007214 },
        { 462.0, 10.4436917491 },
        { 463.0, 10.4332637601 },
        { 464.0, 10.4363313392 },
        { 465.0, 10.4335168824 },
        { 466.0, 10.4310464673 },
        { 467.0, 10.4433065131 },
        { 468.0, 10.4439327195 },
        { 469.0, 10.4448847584 },
        { 470.0, 10.4472289756 },
        { 471.0, 10.4378533736 },
        { 472.0, 10.4407068901 },
        { 473.0, 10.4351965412 },
        { 474.0, 10.4424089417 },
        { 475.0, 10.4405422136 },
        { 476.0, 10.4399632886 },
        { 477.0, 10.4389559291 },
        { 478.0, 10.4380388632 },
        { 479.0, 10.4356010966 },
        { 480.0, 10.4426748231 },
        { 481.0, 10.4463553168 },
        { 482.0, 10.4440831728 },
        { 483.0, 10.4327841215 },
        { 484.0, 10.4418261945 },
        { 485.0, 10.4449523129 },
        { 486.0, 10.4390564263 },
        { 487.0, 10.4451304190 },
        { 488.0, 10.4467000812 },
        { 489.0, 10.4447435029 },
        { 490.0, 10.4456265643 },
        { 491.0, 10.4451525584 },
        { 492.0, 10.4498116970 },
        { 493.0, 10.4367664084 },
        { 494.0, 10.4403494447 },
        { 495.0, 10.4393987656 },
        { 496.0, 10.4502092265 },
        { 497.0, 10.4465369768 },
        { 498.0, 10.4461979382 },
        { 499.0, 10.4412797093 },
        { 500.0, 10.4501294456 },
        { 501.0, 10.4364481382 },
        { 502.0, 10.4423332997 },
        { 503.0, 10.4482809342 },
        { 504.0, 10.4487062208 },
        { 505.0, 10.4490831196 },
        { 506.0, 10.4524870962 },
        { 507.0, 10.4422625750 },
        { 508.0, 10.4476451278 },
        { 509.0, 10.4496930949 },
        { 510.0, 10.4457590058 },
        { 511.0, 10.4535030834 },
        { 512.0, 10.4500645958 },
        { 513.0, 10.4485059977 },
        { 514.0, 10.4528582357 },
        { 515.0, 10.4496239014 },
        { 516.0, 10.4500105232 },
        { 517.0, 10.4486680664 },
        { 518.0, 10.4525872767 },
        { 519.0, 10.4585985728 },
        { 520.0, 10.4514936805 },
        { 521.0, 10.4574401379 },
        { 522.0, 10.4513816200 },
        { 523.0, 10.4461559579 },
        { 524.0, 10.4599629305 },
        { 525.0, 10.4527649246 },
        { 526.0, 10.4583486281 },
        { 527.0, 10.4582051113 },
        { 528.0, 10.4494818673 },
        { 529.0, 10.4497621208 },
        { 530.0, 10.4480114467 },
        { 531.0, 10.4571564309 },
        { 532.0, 10.4684354439 },
        { 533.0, 10.4541689083 },
        { 534.0, 10.4600246623 },
        { 535.0, 10.4578660429 },
        { 536.0, 10.4563045017 },
        { 537.0, 10.4555318989 },
        { 538.0, 10.4620724730 },
        { 539.0, 10.4611999802 },
        { 540.0, 10.4563899413 },
        { 541.0, 10.4641852416 },
        { 542.0, 10.4574276693 },
        { 543.0, 10.4597932771 },
        { 544.0, 10.4511045143 },
        { 545.0, 10.4704018794 },
        { 546.0, 10.4601059780 },
        { 547.0, 10.4606079832 },
        { 548.0, 10.4613176025 },
        { 549.0, 10.4660576247 },
        { 550.0, 10.4591761641 },
        { 551.0, 10.4585100263 },
        { 552.0, 10.4634867683 },
        { 553.0, 10.4553637169 },
        { 554.0, 10.4615915865 },
        { 555.0, 10.4639975578 },
        { 556.0, 10.4640304856 },
        { 557.0, 10.4588550888 },
        { 558.0, 10.4655814990 },
        { 559.0, 10.4649608657 },
        { 560.0, 10.4659023620 },
        { 561.0, 10.4636493400 },
        { 562.0, 10.4623291083 },
        { 563.0, 10.4644785263 },
        { 564.0, 10.4639845155 },
        { 565.0, 10.4627109244 },
        { 566.0, 10.4602171294 },
        { 567.0, 10.4686755948 },
        { 568.0, 10.4606223553 },
        { 569.0, 10.4678727500 },
        { 570.0, 10.4758050516 },
        { 571.0, 10.4640112594 },
        { 572.0, 10.4676577412 },
        { 573.0, 10.4722592384 },
        { 574.0, 10.4711840376 },
        { 575.0, 10.4664448239 },
        { 576.0, 10.4733914696 },
        { 577.0, 10.4703745693 },
        { 578.0, 10.4775136672 },
        { 579.0, 10.4721448906 },
        { 580.0, 10.4705227688 },
        { 581.0, 10.4667594470 },
        { 582.0, 10.4628200494 },
        { 583.0, 10.4772466309 },
        { 584.0, 10.4659681544 },
        { 585.0, 10.4719231091 },
        { 586.0, 10.4698135480 },
        { 587.0, 10.4717754349 },
        { 588.0, 10.4741994739 },
        { 589.0, 10.4698770195 },
        { 590.0, 10.4771975204 },
        { 591.0, 10.4666406065 },
        { 592.0, 10.4726733603 },
        { 593.0, 10.4742928892 },
        { 594.0, 10.4802663513 },
        { 595.0, 10.4779647291 },
        { 596.0, 10.4787243530 },
        { 597.0, 10.4791214392 },
        { 598.0, 10.4792273492 },
        { 599.0, 10.4745072946 },
        { 600.0, 10.4764703289 },
    };

    constexpr size_t WINDOW_SIZE = 30;

    size_t polynomialDegree = 2;
    std::array<PolynomialRegressor<double>, 5> polynomialReg = { PolynomialRegressor<double>(polynomialDegree, WINDOW_SIZE),
                                                                 PolynomialRegressor<double>(polynomialDegree, WINDOW_SIZE),
                                                                 PolynomialRegressor<double>(polynomialDegree, WINDOW_SIZE),
                                                                 PolynomialRegressor<double>(polynomialDegree, WINDOW_SIZE),
                                                                 PolynomialRegressor<double>(polynomialDegree, WINDOW_SIZE) };
    polynomialReg[0].setStrategy(PolynomialRegressor<>::Strategy::IncrementalLeastSquares);
    polynomialReg[1].setStrategy(PolynomialRegressor<>::Strategy::LeastSquares);
    polynomialReg[2].setStrategy(PolynomialRegressor<>::Strategy::HouseholderQR);
    polynomialReg[3].setStrategy(PolynomialRegressor<>::Strategy::BDCSVD);
    polynomialReg[4].setStrategy(PolynomialRegressor<>::Strategy::COD);

    for (size_t i = 0; i < data.size(); ++i)
    {
        const auto& d = data.at(i);
        std::vector<Polynomial<double>> polynomials;
        for (auto& regressor : polynomialReg)
        {
            regressor.push_back(d.first, d.second);
            polynomials.push_back(regressor.calcPolynomial());
        }

        LOG_DEBUG("Point {}", i);
        LOG_DEBUG("{} (IncrementalLeastSquares)", polynomials.at(0).toString("{:.10e}"));
        LOG_DEBUG("{} (LeastSquares)", polynomials.at(1).toString("{:.10e}"));
        LOG_DEBUG("{} (HouseholderQR)", polynomials.at(2).toString("{:.10e}"));
        LOG_DEBUG("{} (BDCSVD)", polynomials.at(3).toString("{:.10e}"));
        LOG_DEBUG("{} (COD)", polynomials.at(4).toString("{:.10e}"));
        LOG_DEBUG("-------------------------------------------------------");

        REQUIRE_THAT(polynomials.at(2).coeffs() - polynomials.at(0).coeffs(), Catch::Matchers::WithinAbs(Eigen::VectorXd::Zero(polynomials.at(0).coeffs().rows()), 1e-6));
        REQUIRE_THAT(polynomials.at(2).coeffs() - polynomials.at(1).coeffs(), Catch::Matchers::WithinAbs(Eigen::VectorXd::Zero(polynomials.at(0).coeffs().rows()), 1e-6));
        REQUIRE_THAT(polynomials.at(2).coeffs() - polynomials.at(3).coeffs(), Catch::Matchers::WithinAbs(Eigen::VectorXd::Zero(polynomials.at(0).coeffs().rows()), 1e-10));
        REQUIRE_THAT(polynomials.at(2).coeffs() - polynomials.at(4).coeffs(), Catch::Matchers::WithinAbs(Eigen::VectorXd::Zero(polynomials.at(0).coeffs().rows()), 1e-10));
    }

    LOG_DEBUG("Used data:");
    for ([[maybe_unused]] const auto& d : data) { LOG_DEBUG("{:6.02f}, {:6.02f}", d.first, d.second); }
}

TEST_CASE("[PolynomialRegressor] Strategy Benchmark (insert all at once)", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    const std::vector<size_t> POINTS = { 10, 100, 1000, 10000 };
    const size_t ITERATIONS = 100;
    const size_t polynomialDegree = 2;

    for (const auto& points : POINTS)
    {
        CAPTURE(points);

        std::vector<std::pair<double, double>> data(points);
        Eigen::VectorXd dataX = Eigen::VectorXd(points);
        Eigen::VectorXd dataY = Eigen::VectorXd(points);

        std::default_random_engine en(1000); // NOLINT(cert-msc32-c,cert-msc51-cpp)
        std::normal_distribution<double> dist(0, 20);
        for (size_t i = 0; i < points; i++)
        {
            auto x = static_cast<double>(i);
            double y = dist(en);

            dataX(static_cast<int>(i)) = x;
            dataY(static_cast<int>(i)) = y;

            data.at(i).first = x;
            data.at(i).second = y;
        }

        std::array<double, 9> avgTimes{};

        for (size_t i = 0; i < ITERATIONS; i++)
        {
            size_t a = 0;
            {
                PolynomialRegressor<double> polynomialReg(polynomialDegree, data.size());
                polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::IncrementalLeastSquares);

                const auto start{ std::chrono::steady_clock::now() };
                for (const auto& d : data) { polynomialReg.push_back(d); }
                auto coeffs = polynomialReg.calcPolynomial().coeffs();
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                PolynomialRegressor<double> polynomialReg(polynomialDegree, data.size());
                polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::LeastSquares);

                const auto start{ std::chrono::steady_clock::now() };
                for (const auto& d : data) { polynomialReg.push_back(d); }
                auto coeffs = polynomialReg.calcPolynomial().coeffs();
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                PolynomialRegressor<double> polynomialReg(polynomialDegree, data.size());
                polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::HouseholderQR);

                const auto start{ std::chrono::steady_clock::now() };
                for (const auto& d : data) { polynomialReg.push_back(d); }
                auto coeffs = polynomialReg.calcPolynomial().coeffs();
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                PolynomialRegressor<double> polynomialReg(polynomialDegree, data.size());
                polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::BDCSVD);

                const auto start{ std::chrono::steady_clock::now() };
                for (const auto& d : data) { polynomialReg.push_back(d); }
                auto coeffs = polynomialReg.calcPolynomial().coeffs();
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                PolynomialRegressor<double> polynomialReg(polynomialDegree, data.size());
                polynomialReg.setStrategy(PolynomialRegressor<>::Strategy::COD);

                const auto start{ std::chrono::steady_clock::now() };
                for (const auto& d : data) { polynomialReg.push_back(d); }
                auto coeffs = polynomialReg.calcPolynomial().coeffs();
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                const auto start{ std::chrono::steady_clock::now() };
                auto coeffs = LeastSquares<double>::calcCoefficients(dataX, dataY, polynomialDegree);
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                const auto start{ std::chrono::steady_clock::now() };
                auto coeffs = HouseholderQr<double>::calcCoefficients(dataX, dataY, polynomialDegree);
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                const auto start{ std::chrono::steady_clock::now() };
                auto coeffs = BDCSVD<double>::calcCoefficients(dataX, dataY, polynomialDegree);
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
            {
                const auto start{ std::chrono::steady_clock::now() };
                auto coeffs = COD<double>::calcCoefficients(dataX, dataY, polynomialDegree);
                const auto end{ std::chrono::steady_clock::now() };
                avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                REQUIRE(coeffs.rows() == polynomialDegree + 1);
            }
        }
        for (auto& t : avgTimes) { t /= static_cast<double>(ITERATIONS); }

        [[maybe_unused]] double fastest = *std::min_element(avgTimes.begin(), avgTimes.end());

        [[maybe_unused]] size_t a = 0;
        LOG_INFO("Calculating polynomial {} order for {} data points (avg over {} iterations)", polynomialDegree, points, ITERATIONS);
        LOG_INFO("    IncrementalLeastSquares      takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    LeastSquares                 takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    HouseholderQr                takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    BDCSVD                       takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    COD                          takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;

        LOG_INFO("    LeastSquares directly        takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    HouseholderQr directly       takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    BDCSVD directly              takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    COD directly                 takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
    }
}

TEST_CASE("[PolynomialRegressor] Strategy Benchmark (insert sequentially)", "[PolynomialRegressor]")
{
    auto logger = initializeTestLogger();

    const std::vector<size_t> POINTS = { 10, 100, 1000 };
    const size_t ITERATIONS = 10;
    const size_t polynomialDegree = 2;

    for (const auto& points : POINTS)
    {
        CAPTURE(points);
        std::default_random_engine en(2000); // NOLINT(cert-msc32-c,cert-msc51-cpp)
        std::normal_distribution<double> dist(0, 20);

        std::array<double, 9> avgTimes{};

        for (size_t i = 0; i < ITERATIONS; i++)
        {
            CAPTURE(i);

            std::array<PolynomialRegressor<double>, 5> polynomialReg = { PolynomialRegressor<double>(polynomialDegree, points),
                                                                         PolynomialRegressor<double>(polynomialDegree, points),
                                                                         PolynomialRegressor<double>(polynomialDegree, points),
                                                                         PolynomialRegressor<double>(polynomialDegree, points),
                                                                         PolynomialRegressor<double>(polynomialDegree, points) };
            polynomialReg[0].setStrategy(PolynomialRegressor<>::Strategy::IncrementalLeastSquares);
            polynomialReg[1].setStrategy(PolynomialRegressor<>::Strategy::LeastSquares);
            polynomialReg[2].setStrategy(PolynomialRegressor<>::Strategy::HouseholderQR);
            polynomialReg[3].setStrategy(PolynomialRegressor<>::Strategy::BDCSVD);
            polynomialReg[4].setStrategy(PolynomialRegressor<>::Strategy::COD);

            Eigen::VectorXd dataX_LSQ;
            Eigen::VectorXd dataY_LSQ;

            Eigen::VectorXd dataX_HQR;
            Eigen::VectorXd dataY_HQR;

            Eigen::VectorXd dataX_BDCSVD;
            Eigen::VectorXd dataY_BDCSVD;

            Eigen::VectorXd dataX_COD;
            Eigen::VectorXd dataY_COD;

            for (size_t p = 0; p < points; p++)
            {
                CAPTURE(p);
                auto x = static_cast<double>(p);
                double y = dist(en);

                size_t a = 0;
                for (; a < polynomialReg.size(); a++)
                {
                    const auto start{ std::chrono::steady_clock::now() };
                    polynomialReg.at(a).push_back(x, y);
                    auto coeffs = polynomialReg.at(a).calcPolynomial().coeffs();
                    const auto end{ std::chrono::steady_clock::now() };
                    avgTimes.at(a) += std::chrono::duration<double>(end - start).count();

                    REQUIRE(coeffs.rows() <= static_cast<int>(polynomialDegree) + 1);
                }

                {
                    const auto start{ std::chrono::steady_clock::now() };
                    dataX_LSQ.conservativeResize(static_cast<int>(p) + 1);
                    dataY_LSQ.conservativeResize(static_cast<int>(p) + 1);
                    dataX_LSQ(static_cast<int>(p)) = x;
                    dataY_LSQ(static_cast<int>(p)) = y;
                    auto coeffs = LeastSquares<double>::calcCoefficients(dataX_LSQ, dataY_LSQ, polynomialDegree);
                    const auto end{ std::chrono::steady_clock::now() };
                    avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                    REQUIRE(coeffs.rows() <= static_cast<int>(polynomialDegree) + 1);
                }
                {
                    const auto start{ std::chrono::steady_clock::now() };
                    dataX_HQR.conservativeResize(static_cast<int>(p) + 1);
                    dataY_HQR.conservativeResize(static_cast<int>(p) + 1);
                    dataX_HQR(static_cast<int>(p)) = x;
                    dataY_HQR(static_cast<int>(p)) = y;
                    auto coeffs = HouseholderQr<double>::calcCoefficients(dataX_HQR, dataY_HQR, polynomialDegree);
                    const auto end{ std::chrono::steady_clock::now() };
                    avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                    REQUIRE(coeffs.rows() <= static_cast<int>(polynomialDegree) + 1);
                }
                {
                    const auto start{ std::chrono::steady_clock::now() };
                    dataX_BDCSVD.conservativeResize(static_cast<int>(p) + 1);
                    dataY_BDCSVD.conservativeResize(static_cast<int>(p) + 1);
                    dataX_BDCSVD(static_cast<int>(p)) = x;
                    dataY_BDCSVD(static_cast<int>(p)) = y;
                    auto coeffs = BDCSVD<double>::calcCoefficients(dataX_BDCSVD, dataY_BDCSVD, polynomialDegree);
                    const auto end{ std::chrono::steady_clock::now() };
                    avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                    REQUIRE(coeffs.rows() <= static_cast<int>(polynomialDegree) + 1);
                }
                {
                    const auto start{ std::chrono::steady_clock::now() };
                    dataX_COD.conservativeResize(static_cast<int>(p) + 1);
                    dataY_COD.conservativeResize(static_cast<int>(p) + 1);
                    dataX_COD(static_cast<int>(p)) = x;
                    dataY_COD(static_cast<int>(p)) = y;
                    auto coeffs = COD<double>::calcCoefficients(dataX_COD, dataY_COD, polynomialDegree);
                    const auto end{ std::chrono::steady_clock::now() };
                    avgTimes.at(a++) += std::chrono::duration<double>(end - start).count();

                    REQUIRE(coeffs.rows() <= static_cast<int>(polynomialDegree) + 1);
                }
            }
        }
        for (auto& t : avgTimes) { t /= static_cast<double>(ITERATIONS); }

        [[maybe_unused]] double fastest = *std::min_element(avgTimes.begin(), avgTimes.end());

        [[maybe_unused]] size_t a = 0;
        LOG_INFO("Calculating polynomial {} order for {} data points (avg over {} iterations)", polynomialDegree, points, ITERATIONS);
        LOG_INFO("    IncrementalLeastSquares      takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    LeastSquares                 takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    HouseholderQr                takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    BDCSVD                       takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    COD                          takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;

        LOG_INFO("    LeastSquares directly        takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    HouseholderQr directly       takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    BDCSVD directly              takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
        a++;
        LOG_INFO("    COD directly                 takes {:.3e} s ({:.2f}x slower than fastest)", avgTimes.at(a), avgTimes.at(a) / fastest);
    }
}

} // namespace NAV::TESTS