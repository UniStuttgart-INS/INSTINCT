// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GMFTests.cpp
/// @brief Global Mapping Function (GMF) tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-04-22

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include "Navigation/Atmosphere/Troposphere/MappingFunctions/GMF.hpp"

#include "GMFmodelData.hpp"

namespace NAV::TESTS::GMF
{

TEST_CASE("[GMF] GMF Series data", "[GMF]")
{
    auto logger = initializeTestLogger();

    for (const auto& data : ref_GmfSeriesData)
    {
        double gmfh = calcTropoMapFunc_GMFH(data.mjd, ref_pos_GRAZ, ref_elevation);
        REQUIRE_THAT(gmfh, Catch::Matchers::WithinAbs(data.gmfh, 1e-4));
        double gmfw = calcTropoMapFunc_GMFW(data.mjd, ref_pos_GRAZ, ref_elevation);
        REQUIRE_THAT(gmfw, Catch::Matchers::WithinAbs(data.gmfw, 1e-4));
    }
}

TEST_CASE("[GMF] GMF Grid data", "[GMF]")
{
    auto logger = initializeTestLogger();

    double mjd = 58849.0;
    double elevation = deg2rad(5.0);

    for (int r = 0; r < ref_orography_ell.rows(); r++)
    {
        double lat = 90.0 - r * 10.0;
        CAPTURE(r);
        CAPTURE(lat);
        for (int c = 0; c < ref_orography_ell.cols(); c++)
        {
            double lon = c * 10.0;
            CAPTURE(c);
            CAPTURE(lon);

            auto pos = Eigen::Vector3d(deg2rad(lat), deg2rad(lon), ref_orography_ell(r, c));

            double gmfh = calcTropoMapFunc_GMFH(mjd, pos, elevation);
            REQUIRE_THAT(gmfh, Catch::Matchers::WithinAbs(ref_GmfhGridData(r, c), 1e-4));
            double gmfw = calcTropoMapFunc_GMFW(mjd, pos, elevation);
            REQUIRE_THAT(gmfw, Catch::Matchers::WithinAbs(ref_GmfwGridData(r, c), 1e-4));
        }
    }
}

} // namespace NAV::TESTS::GMF