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

#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV
{

class UdpUtil
{
  public:
    /// @brief Default Constructor
    UdpUtil() = delete;

    /// Network data stream maximum buffer size in [bytes] (Maximum payload size of a UDP package)
    constexpr static unsigned int MAXIMUM_BYTES = 65507;
    /// Range a port can be in [0, 2^16-1]
    static constexpr std::array<int, 2> PORT_LIMITS = { 0, 65535 };

    /// Size of the message type
    constexpr static size_t SIZE_MSGTYPE = sizeof(int);
    /// Size of a timestamp
    constexpr static size_t SIZE_TIMESTAMP = sizeof(NodeData::insTime);
    /// Size of a Pos (LLA)
    constexpr static size_t SIZE_POS = 3 * sizeof(double);
    /// Size of a Vel (NED)
    constexpr static size_t SIZE_VEL = 3 * sizeof(double);
    /// Size of a Quaternion element
    constexpr static size_t SIZE_QUAT = sizeof(double);

    /// Offset of the timestamp
    constexpr static size_t OFFSET_TIMESTAMP = SIZE_MSGTYPE;
    /// Offset of the position
    constexpr static size_t OFFSET_POS = OFFSET_TIMESTAMP + SIZE_TIMESTAMP;
    /// Offset of the velocity
    constexpr static size_t OFFSET_VEL = OFFSET_POS + SIZE_POS;
    /// Offset of the quaternion
    constexpr static size_t OFFSET_QUAT = OFFSET_VEL + SIZE_VEL;

    /// Total size of the data
    constexpr static size_t SIZE_TOTAL = OFFSET_QUAT + 4 * SIZE_QUAT;

    /// Size of the size of a GNSS observation
    constexpr static size_t SIZE_SIZE = 8;
    /// Size of a single GNSS observation
    constexpr static size_t SIZE_SINGLE_OBSERVATION_DATA = sizeof(GnssObs::ObservationData);

    /// Offset of the GNSS data size
    constexpr static size_t OFFSET_SIZE = OFFSET_POS;
    /// Offset of the GNSS data
    constexpr static size_t OFFSET_GNSSDATA = OFFSET_SIZE + SIZE_SIZE;
};

} // namespace NAV
