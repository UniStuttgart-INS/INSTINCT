// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SearchTests.cpp
/// @brief Ambiguity Search Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-04

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"
#include "util/Eigen.hpp"

#include "Navigation/Math/Math.hpp"
#include "Navigation/GNSS/Ambiguity/AmbiguityResolution.hpp"
#include "Navigation/GNSS/Ambiguity/internal/Decorrelation.hpp"
#include "Navigation/GNSS/Ambiguity/internal/Search.hpp"
#include <catch2/matchers/catch_matchers.hpp>

namespace NAV::TESTS::AmbiguityTests
{

TEST_CASE("[Ambiguity] Integer Rounding", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Vector3d a(5.45, -3.10, 2.97);

    auto a_int = Ambiguity::integerSearchRounding(a);

    REQUIRE(a_int == Eigen::Vector3d(5, -3, 3));
}

TEST_CASE("[Ambiguity] Integer Bootstrapping", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d a(5.45, 3.10, 2.97);
    Eigen::Vector3d a_int = Ambiguity::integerSearchBootstrapping(a, Qa);

    REQUIRE(a_int == Eigen::Vector3d(5, 3, 3));

    [[maybe_unused]] auto [Qz, Z, L, D, z] = Ambiguity::decorrelate_ztrafo(a, Qa).value();
    Eigen::Vector3d z_int = Ambiguity::integerSearchBootstrapping(z, Qz);

    REQUIRE(z_int == Eigen::Vector3d(-5, 10, 2)); // Example taken from 'de Jonge 1996, ch. 4.12'
    a_int = Z.inverse().transpose() * z_int;
    REQUIRE(a_int == Eigen::Vector3d(5, 3, 4));
}

TEST_CASE("[Ambiguity] Chi2 initialization", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d a(5.45, 3.10, 2.97);
    double factor = 1.0;

    // Values for comparison from Matlab code of TU Delft
    std::vector<std::tuple<int, double, double>> refChi = {
        // ncand, bootstrap, volume
        { 1, 0.21833, 0.42715 },
        { 2, 0.30727, 0.67806 },
        { 3, 0.71462, 0.88851 },
        { 4, 1.03198, 1.07636 },
        { 5, 0.00000, 1.24900 },
    };

    [[maybe_unused]] auto [Qz, Z, L, D, z] = Ambiguity::decorrelate_ztrafo(a, Qa).value();

    for (const auto& [ncand, chiBootstrap, chiVolume] : refChi)
    {
        CAPTURE(ncand);
        if (ncand < 5)
        {
            REQUIRE_THAT(Ambiguity::internal::calcChi2_bootstrap(z, Qz, L, ncand), Catch::Matchers::WithinAbs(chiBootstrap, 1e-5));
            REQUIRE_THAT(Ambiguity::calcChi2(z, Qz, L, D, ncand, factor), Catch::Matchers::WithinAbs(chiBootstrap, 1e-5));
        }
        else
        {
            REQUIRE_THAT(Ambiguity::calcChi2(z, Qz, L, D, ncand, factor), Catch::Matchers::WithinAbs(chiVolume, 1e-5));
        }
        REQUIRE_THAT(Ambiguity::internal::calcChi2_volume(D, ncand, factor), Catch::Matchers::WithinAbs(chiVolume, 1e-5));
    }
}

TEST_CASE("[Ambiguity] Integer Least-Squares Search (3x3 example de Jonge 1996)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d a(5.45, 3.10, 2.97);

    // Avoid integer overflows by reducing the ambiguities between -1.0 and +1.0
    Eigen::Vector3d ambIntPart = a.cast<int>().cast<double>();
    a -= ambIntPart;
    LOG_INFO("ambIntPart = {}", ambIntPart.transpose());
    LOG_INFO("afloat = {}", a.transpose());

    [[maybe_unused]] auto [Qz, Z, L, D, z] = Ambiguity::decorrelate_ztrafo(a, Qa).value();
    LOG_INFO("Qz = \n{}", Qz);
    LOG_INFO("Z = \n{}", Z);
    LOG_INFO("L = \n{}", L);
    LOG_INFO("D = {}", D.transpose());
    LOG_INFO("z       = {}", z.transpose());

    // Values for comparison from Matlab code of TU Delft
    std::vector<std::pair<double, Eigen::Vector3d>> ref = {
        // ncand, bootstrap, volume
        { 0.218331095336938, { 5, 3, 4 } },
        { 0.307272575790267, { 6, 4, 4 } },
        { 0.593409683466898, { 4, 2, 4 } },
        { 0.714614150106924, { 6, 3, 1 } },
        { 0.779889844438622, { 5, 2, 1 } },
    };
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(ref.size()); i++)
    {
        CAPTURE(i);
        auto [cands, sqnorm] = Ambiguity::integerLeastSquaresSearch(z, Qz, L, D, i + 1);
        LOG_INFO("cands(i = {}) = \n{}", i, cands);
        LOG_INFO("sqnorm = {}", sqnorm.transpose());

        for (Eigen::Index j = 0; j < cands.cols(); j++)
        {
            // Back transformation
            cands.col(j) = Z.transpose().inverse() * cands.col(j);
            // Reapply the integer part from before
            cands.col(j) += ambIntPart;
        }
        LOG_INFO("afixed = \n{}", cands);

        for (Eigen::Index j = 0; j <= i; j++)
        {
            CAPTURE(j);
            REQUIRE(cands.col(j) == ref.at(static_cast<size_t>(j)).second);
            REQUIRE_THAT(sqnorm(j), Catch::Matchers::WithinAbs(ref.at(static_cast<size_t>(j)).first, 1e-15));
        }
    }
}

TEST_CASE("[Ambiguity] Integer Least-Squares Search-and-Shrink (3x3 example de Jonge 1996)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d a(5.45, 3.10, 2.97);

    // Avoid integer overflows by reducing the ambiguities between -1.0 and +1.0
    Eigen::Vector3d ambIntPart = a.cast<int>().cast<double>();
    a -= ambIntPart;
    LOG_INFO("ambIntPart = {}", ambIntPart.transpose());
    LOG_INFO("afloat = {}", a.transpose());

    [[maybe_unused]] auto [Qz, Z, L, D, z] = Ambiguity::decorrelate_ztrafo(a, Qa).value();
    LOG_INFO("Qz = \n{}", Qz);
    LOG_INFO("Z = \n{}", Z);
    LOG_INFO("L = \n{}", L);
    LOG_INFO("D = {}", D.transpose());
    LOG_INFO("z       = {}", z.transpose());

    // Values for comparison from Matlab code of TU Delft
    std::vector<std::pair<double, Eigen::Vector3d>> ref = {
        // ncand, bootstrap, volume
        { 0.218331095336938, { 5, 3, 4 } },
        { 0.307272575790267, { 6, 4, 4 } },
        { 0.593409683466898, { 4, 2, 4 } },
        { 0.714614150106924, { 6, 3, 1 } },
        { 0.779889844438622, { 5, 2, 1 } },
    };
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(ref.size()); i++)
    {
        CAPTURE(i);
        auto [cands, sqnorm] = Ambiguity::integerLeastSquaresSearchAndShrink(z, L, D, i + 1);
        LOG_INFO("cands(i = {}) = \n{}", i, cands);
        LOG_INFO("sqnorm = {}", sqnorm.transpose());

        for (Eigen::Index j = 0; j < cands.cols(); j++)
        {
            // Back transformation
            cands.col(j) = Z.transpose().inverse() * cands.col(j);
            // Reapply the integer part from before
            cands.col(j) += ambIntPart;
        }
        LOG_INFO("afixed = \n{}", cands);

        for (Eigen::Index j = 0; j <= i; j++)
        {
            CAPTURE(j);
            REQUIRE(cands.col(j) == ref.at(static_cast<size_t>(j)).second);
            REQUIRE_THAT(sqnorm(j), Catch::Matchers::WithinAbs(ref.at(static_cast<size_t>(j)).first, 1e-15));
        }
    }
}

TEST_CASE("[Ambiguity] Integer Least-Squares Search (init ambiguities)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix<double, 10, 10> Qa;
    Qa << 0.926761, 0.027221, 0.774188, 0.370043, -0.168675, 0.133558, 0.618877, -0.253822, 0.40793, 0.0801427,
        0.027221, 0.940028, 0.0505142, 0.604638, 0.864356, -0.0477683, 0.356481, 0.562952, -0.155334, 0.224592,
        0.774188, 0.0505142, 1.01335, 0.0781564, 0.3753, 0.421396, 0.244868, 0.0198712, 0.571501, -0.105983,
        0.370043, 0.604638, 0.0781564, 0.694751, 0.14738, -0.19205, 0.654662, 0.112403, -0.103666, 0.29541,
        -0.168675, 0.864356, 0.3753, 0.14738, 1.49834, 0.347161, -0.180243, 0.864831, 0.0877236, -0.0506733,
        0.133558, -0.0477683, 0.421396, -0.19205, 0.347161, 0.289148, -0.170987, 0.115955, 0.26921, -0.154182,
        0.618877, 0.356481, 0.244868, 0.654662, -0.180243, -0.170987, 0.756149, -0.131137, 0.0296832, 0.27312,
        -0.253822, 0.562952, 0.0198712, 0.112403, 0.864831, 0.115955, -0.131137, 0.544643, -0.074458, 0.00697279,
        0.40793, -0.155334, 0.571501, -0.103666, 0.0877236, 0.26921, 0.0296832, -0.074458, 0.362659, -0.120374,
        0.0801427, 0.224592, -0.105983, 0.29541, -0.0506733, -0.154182, 0.27312, 0.00697279, -0.120374, 0.149138;
    Eigen::Vector<double, 10> a;
    a << -7.23735, -6.65679, -18.8424, -25.386, -7.54516, -36.2794, -32.6687, -21.5819, -0.0643077, -30.352;

    Eigen::Vector<double, 10> expected;
    //     G1C  08   24   07   18   20   14   23   05 30   15 (SatSigId)
    // -------------------------------------------------------
    //          12,   8,   0,  -9,   7, -18, -15,  -6, 19,-13 (simulated ambiguities)
    //         -18  -18  -18  -18  -18  -18  -18  -18 -18 -18 (pivot satellite ambiguity, as all relative to this one)
    expected << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31;

    AmbiguityResolutionParameters params{
        .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
        .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearch,
        .partialFixing = false,
    };
    auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(10, 3), Eigen::MatrixXd(3, 10), params,
                                     "[Ambiguity] Integer Least-Squares Search (init ambiguities)");
    REQUIRE(result.failure == AmbiguityResolutionFailure::None);

    REQUIRE_THAT(result.fixedAmb.at(0).a - expected, Catch::Matchers::WithinAbs(Eigen::VectorXd::Zero(a.rows()), 1e-10));

    // Values for comparison from Matlab code of TU Delft
    REQUIRE_THAT(result.fixedAmb.at(0).sqnorm, Catch::Matchers::WithinAbs(15.582, 1e-3));
    REQUIRE_THAT(result.fixedAmb.at(1).sqnorm, Catch::Matchers::WithinAbs(60.5788, 1e-4));
    REQUIRE_THAT(result.fixedAmb.at(1).a, Catch::Matchers::WithinAbs((Eigen::VectorXd(10) << -8, -8, -19, -27, -8, -36, -34, -22, 0, -31).finished(), 1e-10));
}

TEST_CASE("[Ambiguity] Integer Least-Squares Search-and-Shrink (init ambiguities)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix<double, 10, 10> Qa;
    Qa << 0.926761, 0.027221, 0.774188, 0.370043, -0.168675, 0.133558, 0.618877, -0.253822, 0.40793, 0.0801427,
        0.027221, 0.940028, 0.0505142, 0.604638, 0.864356, -0.0477683, 0.356481, 0.562952, -0.155334, 0.224592,
        0.774188, 0.0505142, 1.01335, 0.0781564, 0.3753, 0.421396, 0.244868, 0.0198712, 0.571501, -0.105983,
        0.370043, 0.604638, 0.0781564, 0.694751, 0.14738, -0.19205, 0.654662, 0.112403, -0.103666, 0.29541,
        -0.168675, 0.864356, 0.3753, 0.14738, 1.49834, 0.347161, -0.180243, 0.864831, 0.0877236, -0.0506733,
        0.133558, -0.0477683, 0.421396, -0.19205, 0.347161, 0.289148, -0.170987, 0.115955, 0.26921, -0.154182,
        0.618877, 0.356481, 0.244868, 0.654662, -0.180243, -0.170987, 0.756149, -0.131137, 0.0296832, 0.27312,
        -0.253822, 0.562952, 0.0198712, 0.112403, 0.864831, 0.115955, -0.131137, 0.544643, -0.074458, 0.00697279,
        0.40793, -0.155334, 0.571501, -0.103666, 0.0877236, 0.26921, 0.0296832, -0.074458, 0.362659, -0.120374,
        0.0801427, 0.224592, -0.105983, 0.29541, -0.0506733, -0.154182, 0.27312, 0.00697279, -0.120374, 0.149138;
    Eigen::Vector<double, 10> a;
    a << -7.23735, -6.65679, -18.8424, -25.386, -7.54516, -36.2794, -32.6687, -21.5819, -0.0643077, -30.352;

    Eigen::Vector<double, 10> expected;
    //     G1C  08   24   07   18   20   14   23   05 30   15 (SatSigId)
    // -------------------------------------------------------
    //          12,   8,   0,  -9,   7, -18, -15,  -6, 19,-13 (simulated ambiguities)
    //         -18  -18  -18  -18  -18  -18  -18  -18 -18 -18 (pivot satellite ambiguity, as all relative to this one)
    expected << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31;

    AmbiguityResolutionParameters params{
        .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
        .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearchAndShrink,
        .partialFixing = false,
    };
    auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(10, 3), Eigen::MatrixXd(3, 10), params,
                                     "[Ambiguity] Integer Least-Squares Search-and-Shrink (init ambiguities)");
    REQUIRE(result.failure == AmbiguityResolutionFailure::None);

    REQUIRE_THAT(result.fixedAmb.at(0).a, Catch::Matchers::WithinAbs(expected, 1e-10));

    // Values for comparison from Matlab code of TU Delft
    REQUIRE_THAT(result.fixedAmb.at(0).sqnorm, Catch::Matchers::WithinAbs(15.582, 1e-3));
    REQUIRE_THAT(result.fixedAmb.at(1).sqnorm, Catch::Matchers::WithinAbs(60.5788, 1e-4));
    REQUIRE_THAT(result.fixedAmb.at(1).a, Catch::Matchers::WithinAbs((Eigen::VectorXd(10) << -8, -8, -19, -27, -8, -36, -34, -22, 0, -31).finished(), 1e-10));
}

TEST_CASE("[Ambiguity] Integer Least-Squares Search (new Ambiguity added)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::MatrixXd Qa(11, 11);
    Qa << 5.61245e-09, -4.21862e-14, 1.7824e-12, 7.8514e-13, -4.84459e-13, 3.23767e-13, 1.38073e-12, -6.52594e-13, 9.69395e-13, 1.56503e-13, 7.94952e-10,
        -4.21838e-14, 5.6123e-09, -2.22e-13, 1.43152e-12, 1.54178e-12, -3.33021e-13, 8.75478e-13, 1.10334e-12, -5.33335e-13, 5.86097e-13, 5.17675e-10,
        1.7824e-12, -2.22003e-13, 5.61285e-09, -1.57372e-13, 8.33098e-13, 1.14212e-12, 2.87437e-13, -3.84298e-14, 1.5139e-12, -4.23504e-13, 2.59704e-09,
        7.85144e-13, 1.43152e-12, -1.57371e-13, 5.6121e-09, 3.64696e-14, -6.70442e-13, 1.67544e-12, 1.58199e-13, -4.49443e-13, 8.13675e-13, -5.5665e-10,
        -4.84456e-13, 1.54178e-12, 8.33099e-13, 3.64679e-14, 5.61345e-09, 8.0004e-13, -6.56981e-13, 1.80454e-12, 2.62393e-13, -2.47076e-13, 2.87943e-09,
        3.23768e-13, -3.33023e-13, 1.14212e-12, -6.70443e-13, 8.00039e-13, 5.6111e-09, -5.7441e-13, 2.17299e-13, 7.6731e-13, -4.79712e-13, 1.57721e-09,
        1.38074e-12, 8.75477e-13, 2.87436e-13, 1.67544e-12, -6.56984e-13, -5.74411e-13, 5.6122e-09, -3.83748e-13, -1.038e-13, 7.45253e-13, -7.04391e-10,
        -6.52593e-13, 1.10334e-12, -3.84278e-14, 1.58198e-13, 1.80455e-12, 2.173e-13, -3.83746e-13, 5.61147e-09, -1.91399e-13, -1.74526e-14, 1.22527e-09,
        9.69396e-13, -5.33336e-13, 1.5139e-12, -4.49444e-13, 2.62392e-13, 7.6731e-13, -1.038e-13, -1.914e-13, 5.6113e-09, -3.99194e-13, 1.47078e-09,
        1.56504e-13, 5.86098e-13, -4.23504e-13, 8.13675e-13, -2.47076e-13, -4.79712e-13, 7.45253e-13, -1.74521e-14, -3.99194e-13, 5.61074e-09, -7.24252e-10,
        7.94952e-10, 5.17675e-10, 2.59704e-09, -5.5665e-10, 2.87943e-09, 1.57721e-09, -7.04391e-10, 1.22527e-09, 1.47078e-09, -7.24252e-10, 0.00114974;
    Eigen::VectorXd a(11);
    a << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31, -27.9678;

    Eigen::Vector<double, 11> expected;
    //     G1C  08   24   07   18   20   14   23   05 30   15   17 (SatSigId)
    // -----------------------------------------------------------
    //          12,   8,   0,  -9,   7, -18, -15,  -6, 19,-13, -10 (simulated ambiguities)
    //         -18  -18  -18  -18  -18  -18  -18  -18 -18 -18  -18 (pivot satellite ambiguity, as all relative to this one)
    expected << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31, -28;

    AmbiguityResolutionParameters params{
        .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
        .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearch,
        .partialFixing = false,
    };
    auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(11, 3), Eigen::MatrixXd(3, 11), params,
                                     "[Ambiguity] Integer Least-Squares Search (new Ambiguity added)");
    REQUIRE(result.failure == AmbiguityResolutionFailure::None);

    REQUIRE_THAT(result.fixedAmb.at(0).a, Catch::Matchers::WithinAbs(expected, 1e-10));

    // Values for comparison from Matlab code of TU Delft
    REQUIRE_THAT(result.fixedAmb.at(0).sqnorm, Catch::Matchers::WithinAbs(0.9018, 1e-4));
    REQUIRE(result.fixedAmb.size() == 1);
}

TEST_CASE("[Ambiguity] Integer Least-Squares Search-And-Shrink (new Ambiguity added)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::MatrixXd Qa(11, 11);
    Qa << 5.61245e-09, -4.21862e-14, 1.7824e-12, 7.8514e-13, -4.84459e-13, 3.23767e-13, 1.38073e-12, -6.52594e-13, 9.69395e-13, 1.56503e-13, 7.94952e-10,
        -4.21838e-14, 5.6123e-09, -2.22e-13, 1.43152e-12, 1.54178e-12, -3.33021e-13, 8.75478e-13, 1.10334e-12, -5.33335e-13, 5.86097e-13, 5.17675e-10,
        1.7824e-12, -2.22003e-13, 5.61285e-09, -1.57372e-13, 8.33098e-13, 1.14212e-12, 2.87437e-13, -3.84298e-14, 1.5139e-12, -4.23504e-13, 2.59704e-09,
        7.85144e-13, 1.43152e-12, -1.57371e-13, 5.6121e-09, 3.64696e-14, -6.70442e-13, 1.67544e-12, 1.58199e-13, -4.49443e-13, 8.13675e-13, -5.5665e-10,
        -4.84456e-13, 1.54178e-12, 8.33099e-13, 3.64679e-14, 5.61345e-09, 8.0004e-13, -6.56981e-13, 1.80454e-12, 2.62393e-13, -2.47076e-13, 2.87943e-09,
        3.23768e-13, -3.33023e-13, 1.14212e-12, -6.70443e-13, 8.00039e-13, 5.6111e-09, -5.7441e-13, 2.17299e-13, 7.6731e-13, -4.79712e-13, 1.57721e-09,
        1.38074e-12, 8.75477e-13, 2.87436e-13, 1.67544e-12, -6.56984e-13, -5.74411e-13, 5.6122e-09, -3.83748e-13, -1.038e-13, 7.45253e-13, -7.04391e-10,
        -6.52593e-13, 1.10334e-12, -3.84278e-14, 1.58198e-13, 1.80455e-12, 2.173e-13, -3.83746e-13, 5.61147e-09, -1.91399e-13, -1.74526e-14, 1.22527e-09,
        9.69396e-13, -5.33336e-13, 1.5139e-12, -4.49444e-13, 2.62392e-13, 7.6731e-13, -1.038e-13, -1.914e-13, 5.6113e-09, -3.99194e-13, 1.47078e-09,
        1.56504e-13, 5.86098e-13, -4.23504e-13, 8.13675e-13, -2.47076e-13, -4.79712e-13, 7.45253e-13, -1.74521e-14, -3.99194e-13, 5.61074e-09, -7.24252e-10,
        7.94952e-10, 5.17675e-10, 2.59704e-09, -5.5665e-10, 2.87943e-09, 1.57721e-09, -7.04391e-10, 1.22527e-09, 1.47078e-09, -7.24252e-10, 0.00114974;
    Eigen::VectorXd a(11);
    a << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31, -27.9678;

    Eigen::Vector<double, 11> expected;
    //     G1C  08   24   07   18   20   14   23   05 30   15   17 (SatSigId)
    // -----------------------------------------------------------
    //          12,   8,   0,  -9,   7, -18, -15,  -6, 19,-13, -10 (simulated ambiguities)
    //         -18  -18  -18  -18  -18  -18  -18  -18 -18 -18  -18 (pivot satellite ambiguity, as all relative to this one)
    expected << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31, -28;

    AmbiguityResolutionParameters params{
        .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
        .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearchAndShrink,
        .partialFixing = false,
    };
    auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(11, 3), Eigen::MatrixXd(3, 11), params,
                                     "[Ambiguity] Integer Least-Squares Search-And-Shrink (new Ambiguity added)");
    REQUIRE(result.failure == AmbiguityResolutionFailure::None);

    REQUIRE_THAT(result.fixedAmb.at(0).a, Catch::Matchers::WithinAbs(expected, 1e-10));

    // Values for comparison from Matlab code of TU Delft
    REQUIRE_THAT(result.fixedAmb.at(0).sqnorm, Catch::Matchers::WithinAbs(0.9018, 1e-4));
    REQUIRE_THAT(result.fixedAmb.at(1).sqnorm, Catch::Matchers::WithinAbs(814.6540, 1e-4));
    REQUIRE_THAT(result.fixedAmb.at(1).a, Catch::Matchers::WithinAbs((Eigen::VectorXd(11) << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31, -27).finished(), 1e-10));
}

// TEST_CASE("[Ambiguity] Partial AR - Integer Least-Squares Search (init ambiguities)", "[Ambiguity]")
// {
//     auto logger = initializeTestLogger();

//     Eigen::Matrix<double, 10, 10> Qa;
//     Qa << 0.926761, 0.027221, 0.774188, 0.370043, -0.168675, 0.133558, 0.618877, -0.253822, 0.40793, 0.0801427,
//         0.027221, 0.940028, 0.0505142, 0.604638, 0.864356, -0.0477683, 0.356481, 0.562952, -0.155334, 0.224592,
//         0.774188, 0.0505142, 1.01335, 0.0781564, 0.3753, 0.421396, 0.244868, 0.0198712, 0.571501, -0.105983,
//         0.370043, 0.604638, 0.0781564, 0.694751, 0.14738, -0.19205, 0.654662, 0.112403, -0.103666, 0.29541,
//         -0.168675, 0.864356, 0.3753, 0.14738, 1.49834, 0.347161, -0.180243, 0.864831, 0.0877236, -0.0506733,
//         0.133558, -0.0477683, 0.421396, -0.19205, 0.347161, 0.289148, -0.170987, 0.115955, 0.26921, -0.154182,
//         0.618877, 0.356481, 0.244868, 0.654662, -0.180243, -0.170987, 0.756149, -0.131137, 0.0296832, 0.27312,
//         -0.253822, 0.562952, 0.0198712, 0.112403, 0.864831, 0.115955, -0.131137, 0.544643, -0.074458, 0.00697279,
//         0.40793, -0.155334, 0.571501, -0.103666, 0.0877236, 0.26921, 0.0296832, -0.074458, 0.362659, -0.120374,
//         0.0801427, 0.224592, -0.105983, 0.29541, -0.0506733, -0.154182, 0.27312, 0.00697279, -0.120374, 1.149138; // Last Term 1 was added
//     Eigen::Vector<double, 10> a;
//     a << -7.23735, -6.65679, -18.8424, -25.386, -7.54516, -36.2794, -32.6687, -21.5819, -0.0643077, -30.352;

//     AmbiguityResolutionParameters params{
//         .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
//         .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearchAndShrink,
//         .partialFixing = true,
//     };
//     auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(10, 3), Eigen::MatrixXd(3, 10), params,
//                                      "[Ambiguity] Partial AR - Integer Least-Squares Search (init ambiguities)");
//     REQUIRE(result.failure == AmbiguityResolutionFailure::None);

//     // Values for comparison from Matlab code of TU Delft
//     REQUIRE_THAT(result.fixedAmb.at(0).a, Catch::Matchers::WithinAbs((Eigen::VectorXd(10) << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31.0052).finished(), 1e-4));
//     REQUIRE_THAT(result.fixedAmb.at(1).a, Catch::Matchers::WithinAbs((Eigen::VectorXd(10) << -8, -8, -19, -27, -8, -36, -34, -22, 0, -30.9681).finished(), 1e-4));
//     REQUIRE_THAT(result.fixedAmb.at(0).sqnorm, Catch::Matchers::WithinAbs(15.5568, 1e-3));
//     REQUIRE_THAT(result.fixedAmb.at(1).sqnorm, Catch::Matchers::WithinAbs(59.6311, 1e-4));
// }

TEST_CASE("[Ambiguity] Integer Least-Squares Search/Search-And-Shrink (performance)", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::MatrixXd Qa(11, 11);
    Qa << 5.61245e-09, -4.21862e-14, 1.7824e-12, 7.8514e-13, -4.84459e-13, 3.23767e-13, 1.38073e-12, -6.52594e-13, 9.69395e-13, 1.56503e-13, 7.94952e-10,
        -4.21838e-14, 5.6123e-09, -2.22e-13, 1.43152e-12, 1.54178e-12, -3.33021e-13, 8.75478e-13, 1.10334e-12, -5.33335e-13, 5.86097e-13, 5.17675e-10,
        1.7824e-12, -2.22003e-13, 5.61285e-09, -1.57372e-13, 8.33098e-13, 1.14212e-12, 2.87437e-13, -3.84298e-14, 1.5139e-12, -4.23504e-13, 2.59704e-09,
        7.85144e-13, 1.43152e-12, -1.57371e-13, 5.6121e-09, 3.64696e-14, -6.70442e-13, 1.67544e-12, 1.58199e-13, -4.49443e-13, 8.13675e-13, -5.5665e-10,
        -4.84456e-13, 1.54178e-12, 8.33099e-13, 3.64679e-14, 5.61345e-09, 8.0004e-13, -6.56981e-13, 1.80454e-12, 2.62393e-13, -2.47076e-13, 2.87943e-09,
        3.23768e-13, -3.33023e-13, 1.14212e-12, -6.70443e-13, 8.00039e-13, 5.6111e-09, -5.7441e-13, 2.17299e-13, 7.6731e-13, -4.79712e-13, 1.57721e-09,
        1.38074e-12, 8.75477e-13, 2.87436e-13, 1.67544e-12, -6.56984e-13, -5.74411e-13, 5.6122e-09, -3.83748e-13, -1.038e-13, 7.45253e-13, -7.04391e-10,
        -6.52593e-13, 1.10334e-12, -3.84278e-14, 1.58198e-13, 1.80455e-12, 2.173e-13, -3.83746e-13, 5.61147e-09, -1.91399e-13, -1.74526e-14, 1.22527e-09,
        9.69396e-13, -5.33336e-13, 1.5139e-12, -4.49444e-13, 2.62392e-13, 7.6731e-13, -1.038e-13, -1.914e-13, 5.6113e-09, -3.99194e-13, 1.47078e-09,
        1.56504e-13, 5.86098e-13, -4.23504e-13, 8.13675e-13, -2.47076e-13, -4.79712e-13, 7.45253e-13, -1.74521e-14, -3.99194e-13, 5.61074e-09, -7.24252e-10,
        7.94952e-10, 5.17675e-10, 2.59704e-09, -5.5665e-10, 2.87943e-09, 1.57721e-09, -7.04391e-10, 1.22527e-09, 1.47078e-09, -7.24252e-10, 0.00114974;
    Eigen::VectorXd a(11);
    a << -6, -10, -18, -27, -11, -36, -33, -24, 1, -31, -27.9678;

    constexpr size_t N = 100;
    {
        [[maybe_unused]] double avg = 0.0;
        AmbiguityResolutionParameters params{
            .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
            .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearch,
            .partialFixing = false,
        };
        for (size_t i = 0; i < N; i++)
        {
            const auto start{ std::chrono::steady_clock::now() };
            auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(11, 3), Eigen::MatrixXd(3, 11), params,
                                             "[Ambiguity] Integer Least-Squares Search/Search-And-Shrink (performance)");
            const auto end{ std::chrono::steady_clock::now() };
            REQUIRE(result.failure == AmbiguityResolutionFailure::None);
            avg += std::chrono::duration<double>(end - start).count();
        }
        avg /= static_cast<double>(N);
        LOG_INFO("Elapsed time: {} (Search)", avg);
    }
    {
        [[maybe_unused]] double avg = 0.0;
        AmbiguityResolutionParameters params{
            .decorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation,
            .searchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearchAndShrink,
            .partialFixing = false,
        };
        for (size_t i = 0; i < N; i++)
        {
            const auto start{ std::chrono::steady_clock::now() };
            auto result = ResolveAmbiguities(a, Qa, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::MatrixXd(11, 3), Eigen::MatrixXd(3, 11), params,
                                             "[Ambiguity] Integer Least-Squares Search/Search-And-Shrink (performance)");
            const auto end{ std::chrono::steady_clock::now() };
            REQUIRE(result.failure == AmbiguityResolutionFailure::None);
            avg += std::chrono::duration<double>(end - start).count();
        }
        avg /= static_cast<double>(N);
        LOG_INFO("Elapsed time: {} (Search-and-Shrink)", avg);
    }
}

} // namespace NAV::TESTS::AmbiguityTests