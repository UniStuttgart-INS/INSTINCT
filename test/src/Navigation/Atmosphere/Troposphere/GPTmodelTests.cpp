// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GPTmodelTests.cpp
/// @brief GPT2/3 model tests
/// @author Rui Wang (rui.wang@ins.uni-stuttgart.de)
/// @date 2023-10-13

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include "Navigation/Atmosphere/Troposphere/Models/GPT.hpp"

#include "GPTmodelData.hpp"

namespace NAV::TESTS::GPTmodels
{
TEST_CASE("[GPTmodel_Series] GPT2/3 Model Functions", "[GPTmodel_Series]")
{
    auto logger = initializeTestLogger();

    std::array<std::pair<double, GPT2output>, GPT2ref_series.size()> GPT2_series_results{};
    std::array<std::pair<double, GPT3output>, GPT3ref_series.size()> GPT3_series_results{};

    for (unsigned int i = 0; i < GPT2_series_results.size(); i++)
    {
        double mjd = 58849.0 + i * 1.0 / 4.0;
        GPT2_series_results.at(i).first = mjd;
        GPT2_series_results.at(i).second = GPT2_param(mjd, GRAZ);
    }

    for (unsigned int i = 0; i < GPT3_series_results.size(); i++)
    {
        double mjd = 58849.0 + i * 1.0 / 4.0;
        GPT3_series_results.at(i).first = mjd;
        GPT3_series_results.at(i).second = GPT3_param(mjd, GRAZ);
    }

    std::array<double, 14> epsilon{ 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-7, 1e-7, 1e-4, 1e-2, 1e-7, 1e-7, 1e-7, 1e-7 };

    for (size_t i = 0; i < GPT2ref_series.size(); i++)
    {
        CAPTURE(i);
        REQUIRE_THAT(GPT2_series_results.at(i).first, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).first, epsilon[0]));

        REQUIRE_THAT(GPT2_series_results.at(i).second.p, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.p, epsilon[1]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.T, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.T, epsilon[2]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.dT, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.dT, epsilon[3]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.Tm, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.Tm, epsilon[4]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.e, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.e, epsilon[5]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.ah, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.ah, epsilon[6]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.aw, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.aw, epsilon[7]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.la, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.la, epsilon[8]));
        REQUIRE_THAT(GPT2_series_results.at(i).second.undu, Catch::Matchers::WithinAbs(GPT2ref_series.at(i).second.undu, epsilon[9]));
    }
    for (unsigned int i = 0; i < GPT3ref_series.size(); i++)
    {
        CAPTURE(i);
        REQUIRE_THAT(GPT3_series_results.at(i).first, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).first, epsilon[0]));

        REQUIRE_THAT(GPT3_series_results.at(i).second.p, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.p, epsilon[1]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.T, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.T, epsilon[2]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.dT, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.dT, epsilon[3]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.Tm, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.Tm, epsilon[4]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.e, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.e, epsilon[5]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.ah, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.ah, epsilon[6]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.aw, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.aw, epsilon[7]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.la, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.la, epsilon[8]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.undu, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.undu, epsilon[9]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.Gn_h, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.Gn_h, epsilon[10]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.Ge_h, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.Ge_h, epsilon[11]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.Gn_w, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.Gn_w, epsilon[12]));
        REQUIRE_THAT(GPT3_series_results.at(i).second.Ge_w, Catch::Matchers::WithinAbs(GPT3ref_series.at(i).second.Ge_w, epsilon[13]));
    }
}

TEST_CASE("[GPTmodel_Grid] GPT2/3 Model Functions", "[GPTmodel_Grid]")
{
    auto logger = initializeTestLogger();

    // GPT2/3_Grid: defining lat/lon vectors and epoch for calculation
    Eigen::VectorXd latGrid = Eigen::VectorXd::Zero(orography_ell.rows());
    Eigen::VectorXd lonGrid = Eigen::VectorXd::Zero(orography_ell.cols());

    for (int i = 0; i < latGrid.rows(); i++) { latGrid(i) = (90.0 - i * 10.0) * M_PI / 180.0; }
    for (int i = 0; i < lonGrid.rows(); i++) { lonGrid(i) = (0.0 + i * 10.0) * M_PI / 180.0; }

    double mjd = 58849.0;

    Eigen::Vector3d lla_pos{};

    // GPT2param (9)-------------------------------
    //  0 p,    pressure in hPa
    //  1 T,    temperature in degrees Celsius
    //  2 dT,   temperature lapse rate in degrees per km
    //  3 Tm,   mean temperature of the water vapor in Kelvin
    //  4 e,    water vapour pressure in hPa
    //  5 ah,   hydrostatic mapping function coefficient at zero height (VMF1)
    //  6 aw,   wet mapping function coefficient (VMF1)
    //  7 la,   water vapour decrease factor
    //  8 undu, geoid undulation in m

    // GPT3param (13) more---------------------------
    //  9 Gn_h, hydrostatic north gradient in m
    // 10 Ge_h, hydrostatic east gradient in m
    // 11 Gn_w, wet north gradient in m
    // 12 Ge_w, wet east gradient in m

    std::array<Eigen::MatrixXd, 9> GPT2_grid_results{};
    std::array<Eigen::MatrixXd, 13> GPT3_grid_results{};
    for (auto& grid : GPT2_grid_results) { grid = Eigen::MatrixXd::Zero(orography_ell.rows(), orography_ell.cols()); }
    for (auto& grid : GPT3_grid_results) { grid = Eigen::MatrixXd::Zero(orography_ell.rows(), orography_ell.cols()); }

    for (int i = 0; i < orography_ell.cols(); i++)
    {
        lla_pos(1) = lonGrid(i);
        for (int j = 0; j < orography_ell.rows(); j++)
        {
            lla_pos(0) = latGrid(j);
            lla_pos(2) = orography_ell(j, i);
            // GPT2
            GPT2output gpt2outputs = GPT2_param(mjd, lla_pos);
            GPT2_grid_results.at(0)(j, i) = gpt2outputs.p;
            GPT2_grid_results.at(1)(j, i) = gpt2outputs.T;
            GPT2_grid_results.at(2)(j, i) = gpt2outputs.dT;
            GPT2_grid_results.at(3)(j, i) = gpt2outputs.Tm;
            GPT2_grid_results.at(4)(j, i) = gpt2outputs.e;
            GPT2_grid_results.at(5)(j, i) = gpt2outputs.ah;
            GPT2_grid_results.at(6)(j, i) = gpt2outputs.aw;
            GPT2_grid_results.at(7)(j, i) = gpt2outputs.la;
            GPT2_grid_results.at(8)(j, i) = gpt2outputs.undu;

            // GPT3
            GPT3output gpt3outputs = GPT3_param(mjd, lla_pos);
            GPT3_grid_results.at(0)(j, i) = gpt3outputs.p;
            GPT3_grid_results.at(1)(j, i) = gpt3outputs.T;
            GPT3_grid_results.at(2)(j, i) = gpt3outputs.dT;
            GPT3_grid_results.at(3)(j, i) = gpt3outputs.Tm;
            GPT3_grid_results.at(4)(j, i) = gpt3outputs.e;
            GPT3_grid_results.at(5)(j, i) = gpt3outputs.ah;
            GPT3_grid_results.at(6)(j, i) = gpt3outputs.aw;
            GPT3_grid_results.at(7)(j, i) = gpt3outputs.la;
            GPT3_grid_results.at(8)(j, i) = gpt3outputs.undu;
            GPT3_grid_results.at(9)(j, i) = gpt3outputs.Gn_h;
            GPT3_grid_results.at(10)(j, i) = gpt3outputs.Ge_h;
            GPT3_grid_results.at(11)(j, i) = gpt3outputs.Gn_w;
            GPT3_grid_results.at(12)(j, i) = gpt3outputs.Ge_w;
        }
    }

    std::array<double, 13> epsilon{ 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-7, 1e-7, 1e-4, 1e-2, 1e-7, 1e-7, 1e-7, 1e-7 };

    std::map<int, Eigen::MatrixXd> GPT2ref;
    GPT2ref[0] = GPT2ref_p;
    GPT2ref[1] = GPT2ref_T;
    GPT2ref[2] = GPT2ref_dT;
    GPT2ref[3] = GPT2ref_Tm;
    GPT2ref[4] = GPT2ref_e;
    GPT2ref[5] = GPT2ref_ah;
    GPT2ref[6] = GPT2ref_aw;
    GPT2ref[7] = GPT2ref_la;
    GPT2ref[8] = GPT2ref_undu;

    for (size_t i = 0; i < GPT2_grid_results.size(); i++)
    {
        CAPTURE(i);
        REQUIRE_THAT(GPT2_grid_results.at(i), Catch::Matchers::WithinAbs(GPT2ref.at(static_cast<int>(i)), epsilon.at(i)));
    }

    std::map<int, Eigen::MatrixXd> GPT3ref;
    GPT3ref[0] = GPT3ref_p;
    GPT3ref[1] = GPT3ref_T;
    GPT3ref[2] = GPT3ref_dT;
    GPT3ref[3] = GPT3ref_Tm;
    GPT3ref[4] = GPT3ref_e;
    GPT3ref[5] = GPT3ref_ah;
    GPT3ref[6] = GPT3ref_aw;
    GPT3ref[7] = GPT3ref_la;
    GPT3ref[8] = GPT3ref_undu;
    GPT3ref[9] = GPT3ref_Gn_h;
    GPT3ref[10] = GPT3ref_Ge_h;
    GPT3ref[11] = GPT3ref_Gn_w;
    GPT3ref[12] = GPT3ref_Ge_w;

    for (size_t i = 0; i < GPT3_grid_results.size(); i++)
    {
        CAPTURE(i);
        REQUIRE_THAT(GPT3_grid_results.at(i), Catch::Matchers::WithinAbs(GPT3ref.at(static_cast<int>(i)), epsilon.at(i)));
    }
}

} // namespace NAV::TESTS::GPTmodels