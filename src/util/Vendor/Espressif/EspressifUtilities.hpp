// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EspressifUtilities.hpp
/// @brief Helper Functions to work with Espressif Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-22

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "uart/protocol/packet.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"

namespace NAV::vendor::espressif
{
/// @brief Decrypts the provided Espressif observation with device time
/// @param[in] obs Espressif Observation to decrypt
/// @param[in, out] packet Uart packet with the data (content gets changed because data gets extracted)
void decryptSingleWiFiObsDeviceTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet);

/// @brief Decrypts the provided multiple Espressif observation with device time
/// @param[in] obs Espressif Observation to decrypt
/// @param[in, out] packet Uart packet with the data (content gets changed because data gets extracted)
void decryptMultipleWiFiObsDeviceTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet);

/// @brief Decrypts the provided Espressif observation
/// @param[in] obs Espressif Observation to decrypt
/// @param[in, out] packet Uart packet with the data (content gets changed because data gets extracted)
void decryptSingleWiFiObsInstinctTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet);

/// @brief Decrypts the provided multiple Espressif observation
/// @param[in] obs Espressif Observation to decrypt
/// @param[in, out] packet Uart packet with the data (content gets changed because data gets extracted)
void decryptMultipleWiFiObsInstinctTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet);

/// @brief Calculates the two UBX checksums for the provided data vector
/// @param[in] data Data Vector for which the checksum should be calculated
/// @return The checksums CK_A and CK_B
std::pair<uint8_t, uint8_t> checksumUBX(const std::vector<uint8_t>& data); // TODO Name

} // namespace NAV::vendor::espressif