// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file lla_jst_111_dd_pos.hpp
/// @author
/// @date

#pragma once

#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV::TESTS::RtklibPosFileTests::lla
{
/// @brief Test Data for the file lla_jst_111_dd.pos
const std::vector<RtklibPosObs> lla_jst_111_dd_pos = {
    {
        /* .insTime = */ InsTime{ 2023, 1, 8, 18, 59, 42.000, JST },
        /* ._e_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._lla_position = */ Eigen::Vector3d{ 29.999994371, 94.999992812, -9.3615 },
        /* ._e_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._n_velocity = */ Eigen::Vector3d{ 0.01216, 0.00473, 0.00324 }, // *.pos uses north-east-up, here north-east-down
        /* .Q = */ 5,
        /* .ns = */ 8,
        /* .sdXYZ = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdNED = */ Eigen::Vector3d{ 2.5813, 2.2781, 7.1259 },
        /* .sdxy = */ NAN,
        /* .sdyz = */ NAN,
        /* .sdzx = */ NAN,
        /* .sdne = */ 0.7786,
        /* .sded = */ -2.0020,
        /* .sddn = */ -2.6572,
        /* .age = */ 0.00,
        /* .ratio = */ 0.0,
        /* .sdvNED = */ Eigen::Vector3d{ 0.14536, 0.13159, 0.44479 },
        /* .sdvne = */ 0.03911,
        /* .sdved = */ -0.13176,
        /* .sdvdn = */ -0.16769,
        /* .sdvXYZ = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvxy = */ NAN,
        /* .sdvyz = */ NAN,
        /* .sdvzx = */ NAN,
        {
            /* .insTime = */ InsTime{ 2023, 1, 8, 22, 59, 42.000, JST },
            /* ._e_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
            /* ._lla_position = */ Eigen::Vector3d{ 30.000005283, 94.999995363, -10.8537 },
            /* ._e_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
            /* ._n_velocity = */ Eigen::Vector3d{ -0.00941, 0.00209, -0.00321 }, // *.pos uses north-east-up, here north-east-down
            /* .Q = */ 5,
            /* .ns = */ 7,
            /* .sdXYZ = */ Eigen::Vector3d{ NAN, NAN, NAN },
            /* .sdNED = */ Eigen::Vector3d{ 2.4447, 2.0231, 5.0831 },
            /* .sdxy = */ NAN,
            /* .sdyz = */ NAN,
            /* .sdzx = */ NAN,
            /* .sdne = */ -1.0676,
            /* .sded = */ -1.0152,
            /* .sddn = */ 1.5662,
            /* .age = */ 0.00,
            /* .ratio = */ 0.0,
            /* .sdvNED = */ Eigen::Vector3d{ 0.17005, 0.13249, 0.34158 },
            /* .sdvne = */ -0.07425,
            /* .sdved = */ -0.08412,
            /* .sdvdn = */ 0.12257,
            /* .sdvXYZ = */ Eigen::Vector3d{ NAN, NAN, NAN },
            /* .sdvxy = */ NAN,
            /* .sdvyz = */ NAN,
            /* .sdvzx = */ NAN,
        },
    };

} // namespace NAV::TESTS::RtklibPosFileTests::lla