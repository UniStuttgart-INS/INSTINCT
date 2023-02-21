// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Saastamoinen.hpp"

#include <cmath>
#include "util/Logger.hpp"

namespace NAV
{

double calcZHD_Saastamoinen(const Eigen::Vector3d& lla_pos, double p)
{
    const double& lat = lla_pos(0); // Latitude in [rad]
    const double& alt = lla_pos(2); // Altitude/Height in [m]

    // Zenith hydrostatic delay [m] - Davis, Appendix A, eq. A11, p. 1604
    double ZHD = 0.0022768 * p / (1 - 0.00266 * std::cos(2 * lat) - 0.00028 * alt * 1e-3);
    LOG_DATA("ZHD {} [m] (Zenith hydrostatic delay)", ZHD);

    return ZHD;
}

double calcZWD_Saastamoinen(double T, double e)
{
    // Zenith wet delay [m]
    double ZWD = 0.002277 * (1255.0 / T + 0.05) * e;
    LOG_DATA("ZWD {} [m] (Zenith wet delay)", ZWD);

    return ZWD;
}

} // namespace NAV
