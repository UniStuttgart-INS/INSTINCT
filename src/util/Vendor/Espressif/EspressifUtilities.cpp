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
#include <string.h>

#include "util/Time/TimeBase.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/TimeOutputs.hpp"

/// Speed of light in air [m/s]
constexpr double cAir = 299702547.0;

bool NAV::vendor::espressif::decryptWiFiObs(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet, [[maybe_unused]] const std::string& nameId)
{
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->payloadLength = packet.extractUint16(); // TODO remove
        // Mac address
        obs->macAddress = fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8());
        std::transform(obs->macAddress.begin(), obs->macAddress.end(), obs->macAddress.begin(), ::toupper); // Convert to uppercase
        // Distance
        int rtt = packet.extractInt32(); // TODO check if ps or ns
        obs->distance = static_cast<double>(rtt) * cAir * 1e-12 / 2;
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
        LOG_DATA("WiFiObs mac Address: {}, measured distance: {}", obs->macAddress, obs->distance);
        LOG_DEBUG("WiFiObs mac Address: {}, measured distance: {}, time of measurement: {}", obs->macAddress, obs->distance, obs->insTime);
    }
    else
    {
        LOG_DEBUG("Received non-binary packet. Ignoring.");
        return false;
    }
    return true;
}

std::pair<uint8_t, uint8_t> NAV::vendor::espressif::checksumUBX(const std::vector<uint8_t>& data)
{
    uint8_t cka = 0;
    uint8_t ckb = 0;

    for (size_t i = 2; i < data.size() - 2; i++)
    {
        cka += data.at(i);
        ckb += cka;
    }
    return std::make_pair(cka, ckb);
}