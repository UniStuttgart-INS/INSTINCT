// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file lla_gpst2_001_pos.hpp
/// @author
/// @date

#pragma once

#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV::TESTS::RtklibPosFileTests::ecef
{
/// @brief Test Data for the file ecef_gpst2_001.pos
const std::vector<RtklibPosObs> ecef_gpst2_001_pos = {
    {
        /* .insTime = */ InsTime{ 2023, 1, 8, 10, 00, 00.000, GPST },
        /* ._e_position = */ Eigen::Vector3d{ -481817.9432, 5507212.2485, 3170368.5142 },
        /* ._lla_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._e_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._n_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .Q = */ 5,
        /* .ns = */ 8,
        /* .sdXYZ = */ Eigen::Vector3d{ 2.2007, 6.7978, 3.4026 },
        /* .sdNED = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdxy = */ 0.4304,
        /* .sdyz = */ 3.9550,
        /* .sdzx = */ 3.9550,
        /* .sdne = */ NAN,
        /* .sded = */ NAN,
        /* .sddn = */ NAN,
        /* .age = */ 0.00,
        /* .ratio = */ 0.0,
        /* .sdvXYZ = */ Eigen::Vector3d{ 0.12567, 0.42372, 0.20237 },
        /* .sdvNED = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvne = */ NAN,
        /* .sdved = */ NAN,
        /* .sdvdn = */ NAN,
    },
    {
        /* .insTime = */ InsTime{ 2023, 1, 8, 14, 00, 00.000, GPST },
        /* ._e_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._lla_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._e_velocity = */ Eigen::Vector3d{ -0.00273, 0.00727, -0.00654 },
        /* ._n_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .Q = */ 5,
        /* .ns = */ 7,
        /* .sdXYZ = */ Eigen::Vector3d{ 2.0367, 4.3235, 3.6147 },
        /* .sdNED = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdxy = */ -0.9771,
        /* .sdyz = */ 3.1496,
        /* .sdzx = */ 0.8002,
        /* .sdne = */ NAN,
        /* .sded = */ NAN,
        /* .sddn = */ NAN,
        /* .age = */ 0.00,
        /* .ratio = */ 0.0,
        /* .sdvNED = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvne = */ NAN,
        /* .sdved = */ NAN,
        /* .sdvdn = */ NAN,
    },
};

} // namespace NAV::TESTS::RtklibPosFileTests::ecef