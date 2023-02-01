// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SatNavData.hpp
/// @brief Satellite Navigation data (to calculate SatNavData and clock)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-07

#pragma once

#include "Clock.hpp"
#include "Orbit.hpp"

namespace NAV
{

/// Satellite Navigation data (to calculate SatNavData and clock)
class SatNavData : public Clock, public Orbit
{
  public:
    /// @brief Child type
    enum Type
    {
        GPSEphemeris,     ///< GPS Broadcast Ephemeris
        GalileoEphemeris, ///< Galileo Broadcast Ephemeris
        GLONASSEphemeris, ///< GLONASS Broadcast Ephemeris
        BeiDouEphemeris,  ///< BeiDou Broadcast Ephemeris
        QZSSEphemeris,    ///< QZSS Broadcast Ephemeris
        IRNSSEphemeris,   ///< IRNSS Broadcast Ephemeris
        SBASEphemeris,    ///< SBAS Broadcast Ephemeris
    };

    /// @brief Constructor
    /// @param[in] type Child type
    /// @param[in] refTime Time when the information is calculated
    explicit SatNavData(Type type, const InsTime& refTime);

    /// @brief Destructor
    ~SatNavData() override = default;
    /// @brief Copy constructor
    SatNavData(const SatNavData&) = default;
    /// @brief Move constructor
    SatNavData(SatNavData&&) = default;
    /// @brief Copy assignment operator
    SatNavData& operator=(const SatNavData&) = delete;
    /// @brief Move assignment operator
    SatNavData& operator=(SatNavData&&) = delete;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] virtual bool isHealthy() const = 0;

    /// @brief Child type (for down-casting)
    const Type type;

    /// Reference time of the information
    const InsTime refTime;
};

} // namespace NAV
