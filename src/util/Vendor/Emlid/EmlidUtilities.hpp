// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EmlidUtilities.hpp
/// @brief Helper Functions to work with Emlid Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-22

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "uart/protocol/packet.hpp"
#include "NodeData/GNSS/EmlidObs.hpp"

namespace NAV::vendor::emlid
{
/// @brief Decrypts the provided Emlid observation
///
/// @param[in, out] obs Emlid Observation to decrypt
/// @param[in, out] packet Uart packet with the data (content gets changed because data gets extracted)
/// @param[in] peek Specifies if the data should be peeked or read
void decryptEmlidObs(const std::shared_ptr<NAV::EmlidObs>& obs, uart::protocol::Packet& packet, bool peek = false);

/// @brief Calculates the two UBX checksums for the provided data vector
///
/// @param[in] data Data Vector for which the checksum should be calculated
/// @return The checksums CK_A and CK_B
std::pair<uint8_t, uint8_t> checksumUBX(const std::vector<uint8_t>& data);

} // namespace NAV::vendor::emlid