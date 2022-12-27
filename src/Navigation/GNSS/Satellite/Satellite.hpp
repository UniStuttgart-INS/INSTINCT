// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Satellite.hpp
/// @brief Calculations and data related to satellite orbit, clock, ...
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-02

#pragma once

#include <memory>
#include <vector>

#include "Navigation/Time/InsTime.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "util/Eigen.hpp"

#include "internal/SatNavData.hpp"

namespace NAV
{

/// @brief Satellite class
class Satellite
{
  public:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Orbit::Pos calcSatellitePos(const InsTime& transTime) const;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Orbit::PosVel calcSatellitePosVel(const InsTime& transTime) const;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Orbit::PosVelAccel calcSatellitePosVelAccel(const InsTime& transTime) const;

    /// @brief Calculates the Variance of the satellite position in [m]
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    [[nodiscard]] double calcSatellitePositionVariance(const InsTime& recvTime) const;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] Clock::Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const;

    /// @brief Checks whether the signal is healthy
    /// @param[in] recvTime Receive time for the data lookup
    [[nodiscard]] bool isHealthy(const InsTime& recvTime) const;

    /// @brief Adds the provided data into the internal time sorted list
    /// @param[in] satNavData Satellite Navigation Data to add
    void addSatNavData(const std::shared_ptr<SatNavData>& satNavData);

    /// @brief Get the navigation data list
    [[nodiscard]] const std::vector<std::shared_ptr<SatNavData>>& getNavigationData() const;

    /// @brief Searches the closest navigation data to the given time
    /// @param time Time the navigation data is requested for
    [[nodiscard]] std::shared_ptr<SatNavData> searchNavigationData(const InsTime& time) const;

  private:
    /// Time sorted list of orbit and clock information of the satellite
    std::vector<std::shared_ptr<SatNavData>> m_navigationData;
};

} // namespace NAV
