// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssNavInfo.hpp
/// @brief Navigation message information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-21

#pragma once

#include <array>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>

#include "util/Assert.h"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/NavigationMessage/IonosphericCorrections.hpp"
#include "Navigation/GNSS/NavigationMessage/Ephemeris.hpp"

namespace NAV
{
/// GNSS Navigation message information
class GnssNavInfo
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "GnssNavInfo";
    }

    /// @brief Get the ephemeris for the satellite which is closest to the specified time
    /// @param[in] satId Satellite identifier
    /// @param[in] recvTime Receiver time to get the ephemeris for
    /// @return Reference to the closest ephemeris to the specified time
    const std::variant<GPSEphemeris, GLONASSEphemeris>& getEphemeris(const SatId& satId, const InsTime& recvTime) const
    {
        const auto& data = broadcastEphemeris.at(satId);
        if (data.size() == 1)
        {
            return data.front().second;
        }

        long double prevDiff = InsTimeUtil::SECONDS_PER_WEEK;
        for (auto riter = data.rbegin(); riter != data.rend(); riter++)
        {
            long double diff = std::abs((riter->first - recvTime).count());
            if (diff < prevDiff)
            {
                prevDiff = diff;
            }
            else
            {
                return (--riter)->second;
            }
        }
        return data.front().second;
    }

    /// @brief Checks whether the signal is healthy
    /// @param[in] satId Satellite identifier
    /// @param[in] recvTime Receive time for the broadcast data lookup
    [[nodiscard]] bool isHealthy(const SatId& satId, const InsTime& recvTime) const
    {
        const auto& eph = getEphemeris(satId, recvTime);
        if (std::holds_alternative<GPSEphemeris>(eph))
        {
            return std::get<GPSEphemeris>(eph).svHealth == 0;
        }
        // else
        // {
        //     return std::get<GLONASSEphemeris>(eph).calcSatelliteClockCorrections(recvTime, dist, satSigId.freq);
        // }
        return false;
    }

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] satSigId Satellite signal identifier
    /// @param[in] recvTime Receive time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    [[nodiscard]] SatelliteClockCorrections calcSatelliteClockCorrections(const SatSigId& satSigId, const InsTime& recvTime, double dist) const
    {
        const auto& eph = getEphemeris(satSigId.toSatId(), recvTime);
        if (std::holds_alternative<GPSEphemeris>(eph))
        {
            return std::get<GPSEphemeris>(eph).calcSatelliteClockCorrections(recvTime, dist, satSigId.freq);
        }
        // else
        // {
        //     return std::get<GLONASSEphemeris>(eph).calcSatelliteClockCorrections(recvTime, dist, satSigId.freq);
        // }
        return {};
    }

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] satId Satellite identifier
    /// @param[in] transTime Transmit time to calculate the satellite position for
    /// @param[out] e_pos The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m] (nullptr disables calculation)
    /// @param[out] e_vel The WGS84 frame velocity of the satellite at the requested time [m/s] (nullptr disables calculation)
    /// @param[out] e_accel The WGS84 frame acceleration of the satellite at the requested time [m/s^2] (nullptr disables calculation)
    void calcSatellitePosVelAccel(const SatId& satId, const InsTime& transTime,
                                  Eigen::Vector3d* e_pos = nullptr, Eigen::Vector3d* e_vel = nullptr, Eigen::Vector3d* e_accel = nullptr) const
    {
        const auto& eph = getEphemeris(satId, transTime);
        if (std::holds_alternative<GPSEphemeris>(eph))
        {
            std::get<GPSEphemeris>(eph).calcSatellitePosVelAccel(transTime, satId.satSys, e_pos, e_vel, e_accel);
        }
        // else
        // {
        //     std::get<GLONASSEphemeris>(eph).calcSatellitePosVelAccelClk(transTime, dist, satId.satSys, clkBias, e_pos, e_vel, e_accel);
        // }
    }

    /// @brief Satellite Systems available
    SatelliteSystem satelliteSystems = SatSys_None;

    /// @brief Ionospheric correction values
    IonosphericCorrections ionosphericCorrections;

    /// @brief Time system correction parameters
    struct TimeSystemCorrections
    {
        double a0 = std::nan(""); ///< a0 / tau_c Coefficient of linear polynomial [s] Δt = a0 + a1 * (t - t_ref)
        double a1 = std::nan(""); ///< a1 Coefficient of linear polynomial [s/s]
    };

    /// Time system correction parameters
    std::unordered_map<SatelliteSystem, TimeSystemCorrections> timeSysCorr;

    /// Map of all data contained in the navigation message
    /// Key {SatSys, SatNum}, InsTime e.g. [{GPS, 1}][time]
    std::unordered_map<SatId, std::vector<std::pair<InsTime, std::variant<GPSEphemeris, GLONASSEphemeris>>>> broadcastEphemeris;
};

} // namespace NAV
