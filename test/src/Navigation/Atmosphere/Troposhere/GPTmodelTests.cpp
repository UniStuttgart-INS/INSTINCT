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

    GPToutput gpt2outputs;
    GPToutput gpt3outputs;

    Eigen::MatrixXd GPT2_series_results = Eigen::MatrixXd::Zero(41, 10);
    Eigen::MatrixXd GPT3_series_results = Eigen::MatrixXd::Zero(41, 14);

    for (int i = 0; i < 41; i++)
    {
        double mjd = 58849.0 + i * 1.0 / 4.0;

        // GPT2
        GPT2_param(mjd, GRAZ, internal::GPT2_grid, gpt2outputs);
        GPT2_series_results(i, 0) = mjd;
        GPT2_series_results(i, 1) = gpt2outputs.p;
        GPT2_series_results(i, 2) = gpt2outputs.T;
        GPT2_series_results(i, 3) = gpt2outputs.dT;
        GPT2_series_results(i, 4) = gpt2outputs.Tm;
        GPT2_series_results(i, 5) = gpt2outputs.e;
        GPT2_series_results(i, 6) = gpt2outputs.ah;
        GPT2_series_results(i, 7) = gpt2outputs.aw;
        GPT2_series_results(i, 8) = gpt2outputs.la;
        GPT2_series_results(i, 9) = gpt2outputs.undu;

        // GPT3
        GPT3_param(mjd, GRAZ, internal::GPT3_grid, gpt3outputs);
        GPT3_series_results(i, 0) = mjd;
        GPT3_series_results(i, 1) = gpt3outputs.p;
        GPT3_series_results(i, 2) = gpt3outputs.T;
        GPT3_series_results(i, 3) = gpt3outputs.dT;
        GPT3_series_results(i, 4) = gpt3outputs.Tm;
        GPT3_series_results(i, 5) = gpt3outputs.e;
        GPT3_series_results(i, 6) = gpt3outputs.ah;
        GPT3_series_results(i, 7) = gpt3outputs.aw;
        GPT3_series_results(i, 8) = gpt3outputs.la;
        GPT3_series_results(i, 9) = gpt3outputs.undu;
        GPT3_series_results(i, 10) = gpt3outputs.Gn_h;
        GPT3_series_results(i, 11) = gpt3outputs.Ge_h;
        GPT3_series_results(i, 12) = gpt3outputs.Gn_w;
        GPT3_series_results(i, 13) = gpt3outputs.Ge_w;
    }

    std::array<double, 14> epsilon{ 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-7, 1e-7, 1e-4, 1e-2, 1e-7, 1e-7, 1e-7, 1e-7 };

    for (unsigned int i = 0; i < 10; i++)
    {
        CHECK_THAT(GPT2_series_results.col(i), Catch::Matchers::WithinAbs(GPT2ref_series.col(i), epsilon.at(i)));
    }
    for (unsigned int i = 0; i < 14; i++)
    {
        CHECK_THAT(GPT3_series_results.col(i), Catch::Matchers::WithinAbs(GPT3ref_series.col(i), epsilon.at(i)));
    }
}

TEST_CASE("[GPTmodel_Grid] GPT2/3 Model Functions", "[GPTmodel_Grid]")
{
    auto logger = initializeTestLogger();

    for (int i = 0; i < 19; i++)
    {
        latGrid(i, 0) = (90.0 - i * 10.0) * M_PI / 180.0;
    }

    for (int i = 0; i < 36; i++)
    {
        lonGrid(i, 0) = (0.0 + i * 10.0) * M_PI / 180.0;
    }

    double mjd = 58849.0;

    GPToutput gpt2outputs;
    GPToutput gpt3outputs;

    Eigen::Vector3d lla_pos{};

    // GPT2param (9)-------------------------------
    // p,    pressure in hPa
    // T,    temperature in degrees Celsius
    // dT,   temperature lapse rate in degrees per km
    // Tm,   mean temperature of the water vapor in Kelvin
    // e,    water vapour pressure in hPa
    // ah,   hydrostatic mapping function coefficient at zero height (VMF1)
    // aw,   wet mapping function coefficient (VMF1)
    // la,   water vapour decrease factor
    // undu, geoid undulation in m

    // GPT3param (13) more---------------------------
    // Gn_h, hydrostatic north gradient in m
    // Ge_h, hydrostatic east gradient in m
    // Gn_w, wet north gradient in m
    // Ge_w, wet east gradient in m

    std::map<int, Eigen::MatrixXd> GPT2_grid_results;
    std::map<int, Eigen::MatrixXd> GPT3_grid_results;
    for (int i = 0; i < 9; i++)
    {
        GPT2_grid_results[i] = Eigen::MatrixXd::Zero(19, 36);
    }
    for (int i = 0; i < 13; i++)
    {
        GPT3_grid_results[i] = Eigen::MatrixXd::Zero(19, 36);
    }

    for (int i = 0; i < 36; i++)
    {
        lla_pos(1) = lonGrid(i, 0);
        for (int j = 0; j < 19; j++)
        {
            lla_pos(0) = latGrid(j, 0);
            lla_pos(2) = orography_ell(j, i);
            // GPT2
            GPT2_param(mjd, lla_pos, internal::GPT2_grid, gpt2outputs);
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
            GPT3_param(mjd, lla_pos, internal::GPT3_grid, gpt3outputs);
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

    for (int i = 0; i < 9; i++)
    {
        CHECK_THAT(GPT2_grid_results.at(i), Catch::Matchers::WithinAbs(GPT2ref.at(i), epsilon.at(static_cast<unsigned int>(i))));
    }

    for (int i = 0; i < 13; i++)
    {
        CHECK_THAT(GPT3_grid_results.at(i), Catch::Matchers::WithinAbs(GPT3ref.at(i), epsilon.at(static_cast<unsigned int>(i))));
    }
}

} // namespace NAV::TESTS::GPTmodels