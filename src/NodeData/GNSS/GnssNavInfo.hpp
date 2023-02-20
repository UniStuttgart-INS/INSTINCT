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

#include <unordered_map>
#include <utility>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/Atmosphere/Ionosphere/IonosphericCorrections.hpp"
#include "Navigation/GNSS/Satellite/Satellite.hpp"
#include "util/Container/Pair.hpp"

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

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] satId Satellite identifier
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Orbit::Pos calcSatellitePos(const SatId& satId, const InsTime& transTime) const
    {
        return m_satellites.at(satId).calcSatellitePos(transTime);
    }
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] satId Satellite identifier
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Orbit::PosVel calcSatellitePosVel(const SatId& satId, const InsTime& transTime) const
    {
        return m_satellites.at(satId).calcSatellitePosVel(transTime);
    }
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] satId Satellite identifier
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Orbit::PosVelAccel calcSatellitePosVelAccel(const SatId& satId, const InsTime& transTime) const
    {
        return m_satellites.at(satId).calcSatellitePosVelAccel(transTime);
    }

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] satId Satellite identifier
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] Clock::Corrections calcSatelliteClockCorrections(const SatId& satId, const InsTime& recvTime, double dist, const Frequency& freq) const
    {
        return m_satellites.at(satId).calcClockCorrections(recvTime, dist, freq);
    }

    /// @brief Calculates the Variance of the satellite position in [m]
    /// @param[in] satId Satellite identifier
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    [[nodiscard]] double calcSatellitePositionVariance(const SatId& satId, const InsTime& recvTime) const
    {
        return m_satellites.at(satId).calcSatellitePositionVariance(recvTime);
    }

    /// @brief Checks whether the signal is healthy
    /// @param[in] satId Satellite identifier
    /// @param[in] recvTime Receive time for the data lookup
    [[nodiscard]] bool isHealthy(const SatId& satId, const InsTime& recvTime) const
    {
        return m_satellites.at(satId).isHealthy(recvTime);
    }

    /// @brief Adds the provided satellite navigation data to the satellite
    /// @param[in] satId Satellite identifier
    /// @param[in] satNavData Satellite Navigation Data to add
    void addSatelliteNavData(const SatId& satId, const std::shared_ptr<SatNavData>& satNavData)
    {
        m_satellites[satId].addSatNavData(satNavData);
    }

    /// @brief Checks whether the satellite is included in the internal data
    /// @param[in] satId Satellite identifier
    /// @param[in] recvTime Receive time for the data lookup
    [[nodiscard]] std::shared_ptr<NAV::SatNavData> searchNavigationData(const SatId& satId, const InsTime& recvTime) const
    {
        if (!m_satellites.contains(satId)) { return nullptr; }

        auto satNavData = m_satellites.at(satId).searchNavigationData(recvTime);
        if (satNavData == nullptr)
        {
            [[maybe_unused]] auto printNavData = [&]() {
                std::string ret;
                for (const auto& navData : m_satellites.at(satId).getNavigationData())
                {
                    ret += fmt::format("[{} - diff {:.0f}s], ", navData->refTime, std::abs((navData->refTime - recvTime).count()));
                }
                return ret.substr(0, ret.length() - 2);
            };

            LOG_TRACE("[{}][{}]: No navigation data found. Available data are at time: {}", satId, recvTime, printNavData());
        }

        return satNavData;
    }

    /// @brief Returns the amount of satellites contained in this message
    [[nodiscard]] size_t nSatellites() const
    {
        return m_satellites.size();
    }

    /// @brief Get the satellites
    /// @return Reference to the internal satellites map
    const auto& satellites() const
    {
        return m_satellites;
    }

    /// @brief Resets the data by clearing the member variables
    void reset()
    {
        satelliteSystems = SatSys_None;
        ionosphericCorrections.clear();
        timeSysCorr.clear();
        m_satellites.clear();
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

    /// Time system correction parameters. Difference between GNSS system time and UTC or other time systems
    std::unordered_map<std::pair<TimeSystem, TimeSystem>, TimeSystemCorrections> timeSysCorr;

  private:
    /// Map of satellites containing the navigation message data
    std::unordered_map<SatId, Satellite> m_satellites;
};

} // namespace NAV
