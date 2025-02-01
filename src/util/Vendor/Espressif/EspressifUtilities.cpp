// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EspressifUtilities.hpp"

#include "util/Eigen.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"
#include <cstring>

#include "util/Time/TimeBase.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/TimeOutputs.hpp"

bool NAV::vendor::espressif::decryptWiFiObs(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet, [[maybe_unused]] const std::string& nameId)
{
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        packet.extractUint16(); // payloadLength
        // Mac address
        std::array<uint8_t, 6> mac{};
        std::generate(mac.begin(), mac.end(), [&]() { return packet.extractUint8(); });
        // Format the MAC address in the correct order (independent of compiler)
        obs->macAddress = fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        std::transform(obs->macAddress.begin(), obs->macAddress.end(), obs->macAddress.begin(), ::toupper); // Convert to uppercase
        // Distance
        int rtt = packet.extractInt32(); // Round trip time in picoseconds
        obs->distance = static_cast<double>(rtt) * InsConst::C_AIR * 1e-12 / 2;
        int rttStd = packet.extractInt32(); // Standard deviation of the round trip time in picoseconds
        obs->distanceStd = static_cast<double>(rttStd) * InsConst::C_AIR * 1e-12 / 2;
        // Time of measurement
        InsTime_YMDHMS yearMonthDayHMS(packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32());
        InsTime timeOfMeasurement(yearMonthDayHMS, UTC);
        [[maybe_unused]] uint32_t microseconds = packet.extractUint32();
        obs->insTime = timeOfMeasurement + std::chrono::microseconds(microseconds);
        // Time outputs
        std::shared_ptr<vendor::vectornav::TimeOutputs> timeOutputs = std::make_shared<vendor::vectornav::TimeOutputs>();
        obs->timeOutputs.syncInCnt = packet.extractUint32();
        obs->timeOutputs.timeSyncIn = packet.extractUint64();
        // Log the measurement details
        LOG_DATA("WiFiObs mac Address: {}, measured distance: {}, time of measurement: {}", obs->macAddress, obs->distance, obs->insTime);
    }
    else
    {
        LOG_DATA("Received non-binary packet. Ignoring.");
        return false;
    }
    return true;
}
