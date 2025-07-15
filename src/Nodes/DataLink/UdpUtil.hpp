// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UdpUtil.hpp
/// @brief Utility for the UDP Send and Receive nodes
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2025-07-02

#pragma once

#include <cstdint>
#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV::UdpUtil
{
/// Network data stream maximum buffer size in [bytes] (Maximum payload size of a UDP package)
constexpr static unsigned int MAXIMUM_BYTES = 65507;
/// Range a port can be in [0, 2^16-1]
static constexpr std::array<int, 2> PORT_LIMITS = { 0, 65535 };

/// Enum specifying the type of the output message
enum class MessageType : uint8_t
{
    PosVelAtt, ///< Extract PosVelAtt data
    PosVel,    ///< Extract PosVel data
    Pos,       ///< Extract Pos data
    GnssObs,   ///< Extract GnssObs data
    COUNT,     ///< Number of items in the enum
};

namespace Size
{
/// Size of the message type
constexpr static size_t MSGTYPE = sizeof(uint8_t);
/// Size of a GPS cycle
constexpr static size_t GPSCYCLE = sizeof(int32_t);
/// Size of a GPS week
constexpr static size_t GPSWEEK = sizeof(int32_t);
/// Size of a GPS TOW
constexpr static size_t GPSTOW = sizeof(double);
/// Size of a Pos (LLA)
constexpr static size_t POS = 3 * sizeof(double);
/// Size of a Vel (NED)
constexpr static size_t VEL = 3 * sizeof(double);
/// Size of a Quaternion element
constexpr static size_t QUAT = sizeof(double);

/// Size of a total 'Pos' message
constexpr static size_t TOTAL_POS = MSGTYPE + GPSCYCLE + GPSWEEK + GPSTOW + POS;
/// Size of a total 'PosVel' message
constexpr static size_t TOTAL_POSVEL = TOTAL_POS + VEL;
/// Size of a total 'PosVelAtt' message
constexpr static size_t TOTAL_POSVELATT = TOTAL_POSVEL + 4 * QUAT;

/// Size of the size of a GNSS observation
constexpr static size_t SIZE = sizeof(size_t);
/// Size of a single GNSS observation
constexpr static size_t SINGLE_OBSERVATION_DATA = sizeof(GnssObs::ObservationData);
} // namespace Size

namespace Offset
{
/// Offset of the GPS cycle
constexpr static size_t GPSCYCLE = Size::MSGTYPE;
/// Offset of the GPS week
constexpr static size_t GPSWEEK = GPSCYCLE + Size::GPSCYCLE;
/// Offset of the GPS tow
constexpr static size_t GPSTOW = GPSWEEK + Size::GPSWEEK;
/// Offset of the position
constexpr static size_t POS = GPSTOW + Size::GPSTOW;
/// Offset of the velocity
constexpr static size_t VEL = POS + Size::POS;
/// Offset of the quaternion
constexpr static size_t QUAT = VEL + Size::VEL;

/// Offset of the GNSS data size
constexpr static size_t SIZE = Offset::POS;
/// Offset of the GNSS data
constexpr static size_t GNSSDATA = SIZE + Size::SIZE;
} // namespace Offset

} // namespace NAV::UdpUtil
