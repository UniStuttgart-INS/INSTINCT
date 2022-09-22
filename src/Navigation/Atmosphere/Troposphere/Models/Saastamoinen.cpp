// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Saastamoinen.hpp"

#include <cmath>
#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Atmosphere/Functions.hpp"
#include "util/Logger.hpp"

namespace NAV
{

ZenithDelay calcTroposphericRangeDelay_Saastamoinen(const Eigen::Vector3d& lla_pos)
{
    const double& lat = lla_pos(0); // Latitude in [rad]
    const double& alt = lla_pos(2); // Altitude/Height in [m]

    // Total barometric pressure in [millibar] - RTKLIB ch. E.5, p. 149 approximates geodetic height by the ellipsoidal height
    double p = calcTotalPressure(alt);
    LOG_DATA("p {} [millibar] (Total barometric pressure)", p);

    // Absolute temperature in [K] - RTKLIB ch. E.5, p. 149 approximates geodetic height by the ellipsoidal height
    double T = calcAbsoluteTemperature(alt);
    LOG_DATA("T {} [K] (Absolute temperature)", T);

    // Partial pressure of water vapour in [millibar] - RTKLIB ch. E.5, p. 149 specifies 70%
    double e = calcWaterVaporPartialPressure(T, 0.7);
    LOG_DATA("e {} [millibar] (Partial pressure of water vapour)", e);

    // Zenith hydrostatic delay [m] - Davis, Appendix A, eq. A11, p. 1604
    double ZHD = 0.0022768 * p / (1 - 0.00266 * std::cos(2 * lat) - 0.00028 * alt * 1e-3);
    LOG_DATA("ZHD {} [m] (Zenith hydrostatic delay)", ZHD);

    // Zenith wet delay [m]
    double ZWD = 0.002277 * (1255.0 / T + 0.05) * e;
    LOG_DATA("ZWD {} [m] (Zenith wet delay)", ZWD);

    return { .ZHD = ZHD,
             .ZWD = ZWD };
}

} // namespace NAV
