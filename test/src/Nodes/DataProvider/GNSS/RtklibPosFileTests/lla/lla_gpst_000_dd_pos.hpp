// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file lla_gpst_000_dd_pos.hpp
/// @author N. Stahl (HiWi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-04-26

#pragma once

#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV::TESTS::RtklibPosFileTests::lla
{
/// @brief Test Data for the file lla_gpst_000_dd.pos
const std::vector<RtklibPosObs> lla_gpst_000_dd_pos = {
    {
        /* .insTime = */ InsTime{ 0, 2244, 36000.000, GPST }, // *.pos does not give a value for gps cycle, so it is set to 0 since INSTINCT needs a value (seems to calculate that cycle=2)
        /* ._e_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._lla_position = */ Eigen::Vector3d{ 29.999994371, 94.999992812, -9.3615 },
        /* ._e_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._n_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
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
        /* .sdvNED = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvne = */ NAN,
        /* .sdved = */ NAN,
        /* .sdvdn = */ NAN,
        /* .sdvXYZ = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvxy = */ NAN,
        /* .sdvyz = */ NAN,
        /* .sdvzx = */ NAN,
    },
    {
        /* .insTime = */ InsTime{ 0, 2244, 50400.000, GPST }, // *.pos does not give a value for gps cycle, so it is set to 0 since INSTINCT needs a value (seems to calculate that cycle=2)
        /* ._e_position = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._lla_position = */ Eigen::Vector3d{ 30.000005283, 94.999995363, -10.8537 },
        /* ._e_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* ._n_velocity = */ Eigen::Vector3d{ NAN, NAN, NAN },
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
        /* .sdvNED = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvne = */ NAN,
        /* .sdved = */ NAN,
        /* .sdvdn = */ NAN,
        /* .sdvXYZ = */ Eigen::Vector3d{ NAN, NAN, NAN },
        /* .sdvxy = */ NAN,
        /* .sdvyz = */ NAN,
        /* .sdvzx = */ NAN,
    },
};

} // namespace NAV::TESTS::RtklibPosFileTests::lla