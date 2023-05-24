// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ecef_gpst2_111.hpp
/// @author N. Stahl (HiWi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-04-26

#pragma once

#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV::TESTS::RtklibPosFileTests::ecef
{
/// @brief Test Data for the file ecef_gpst2_111.pos
const std::vector<RtklibPosObs> ecef_gpst2_111 = {
    {
        /* .insTime = */ InsTime{ 2023, 1, 8, 10, 00, 00.000, GPST },
        /* ._e_position = */ Eigen::Vector3d{ -481817.9432, 5507212.2485, 3170368.5142 },
        /* ._lla_position = */ std::nullopt,
        /* ._e_velocity = */ Eigen::Vector3d{ -0.00394, -0.00927, 0.00891 },
        /* ._n_velocity = */ std::nullopt,
        /* .Q = */ 5,
        /* .ns = */ 8,
        /* .sdXYZ = */ Eigen::Vector3d{ 2.2007, 6.7978, 3.4026 },
        /* .sdNED = */ std::nullopt,
        /* .sdxy = */ 0.4304,
        /* .sdyz = */ 3.9550,
        /* .sdzx = */ 0.3408,
        /* .sdne = */ std::nullopt,
        /* .sded = */ std::nullopt,
        /* .sddn = */ std::nullopt,
        /* .age = */ 0.00,
        /* .ratio = */ 0.0,
        /* .sdvNED = */ std::nullopt,
        /* .sdvne = */ std::nullopt,
        /* .sdved = */ std::nullopt,
        /* .sdvdn = */ std::nullopt,
        /* .sdvXYZ = */ Eigen::Vector3d{ 0.12567, 0.42372, 0.20237 },
        /* .sdvxy = */ 0.04010,
        /* .sdvyz = */ 0.25072,
        /* .sdvzx = */ 0.04342,
    },
    {
        /* .insTime = */ InsTime{ 2023, 1, 8, 14, 00, 00.000, GPST },
        /* ._e_position = */ Eigen::Vector3d{ -481818.0230, 5507210.3373, 3170368.8157 },
        /* ._lla_position = */ std::nullopt,
        /* ._e_velocity = */ Eigen::Vector3d{ -0.00273, 0.00727, -0.00654 },
        /* ._n_velocity = */ std::nullopt,
        /* .Q = */ 5,
        /* .ns = */ 7,
        /* .sdXYZ = */ Eigen::Vector3d{ 2.0367, 4.3235, 3.6147 },
        /* .sdNED = */ std::nullopt,
        /* .sdxy = */ -0.9771,
        /* .sdyz = */ 3.1496,
        /* .sdzx = */ 0.8002,
        /* .sdne = */ std::nullopt,
        /* .sded = */ std::nullopt,
        /* .sddn = */ std::nullopt,
        /* .age = */ 0.00,
        /* .ratio = */ 0.0,
        /* .sdvNED = */ std::nullopt,
        /* .sdvne = */ std::nullopt,
        /* .sdved = */ std::nullopt,
        /* .sdvdn = */ std::nullopt,
        /* .sdvXYZ = */ Eigen::Vector3d{ 0.13212, 0.28605, 0.25272 },
        /* .sdvxy = */ -0.04746,
        /* .sdvyz = */ 0.21463,
        /* .sdvzx = */ 0.06568,
    },
};

} // namespace NAV::TESTS::RtklibPosFileTests::ecef