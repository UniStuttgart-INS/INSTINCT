// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EspressifUtilities.hpp
/// @brief Helper Functions to work with Espressif Sensors
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-08

#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "uart/protocol/packet.hpp"
#include "util/Vendor/Ublox/UbloxUtilities.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"

namespace NAV::vendor::espressif
{
/// @brief Decrypts the provided Espressif observation
/// @param[in] obs Espressif Observation to decrypt
/// @param[in, out] packet Uart packet with the data (content gets changed because data gets extracted)
/// @param[in] nameId NameId of the calling node for Log output
bool decryptWiFiObs(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet, [[maybe_unused]] const std::string& nameId);
} // namespace NAV::vendor::espressif