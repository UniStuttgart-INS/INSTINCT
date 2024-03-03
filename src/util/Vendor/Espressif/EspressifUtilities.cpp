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

/// Speed of light in air [m/s]
constexpr double cAir = 299702547.0;

void NAV::vendor::espressif::decryptSingleWiFiObsDeviceTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet)
{
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->payloadLength = packet.extractUint16(); // TODO remove
        // Mac address
        std::string macAddress = fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8());
        std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper); // Convert to uppercase
        // Distance
        int rtt = packet.extractInt32();
        double measuredDistance = static_cast<double>(rtt) * cAir * 1e-9 / 2;
        // Time of measurement
        InsTime_YMDHMS yearMonthDayHMS(packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32());
        InsTime timeOfMeasurement(yearMonthDayHMS, UTC);
        [[maybe_unused]] int ms = packet.extractInt32();
        // Add the measurement to the WiFiObs
        obs->data.push_back({ macAddress, timeOfMeasurement, measuredDistance });
        // Log the measurement details
        LOG_DATA("WiFiObs mac Address: {}, measured distance: {}", macAddress, measuredDistance);
    }
    else
    {
        LOG_DEBUG("Received non-binary packet. Ignoring.");
    }
}

void NAV::vendor::espressif::decryptMultipleWiFiObsDeviceTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet)
{
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->payloadLength = packet.extractUint16();                                           // TODO remove
        size_t measurementLength = 14;                                                         // TODO irgendwo anders definieren
        size_t numOfMeasurement = static_cast<size_t>(obs->payloadLength) / measurementLength; // TODO checksum length = 2U
        for (size_t i = 0; i < numOfMeasurement; i++)
        {
            std::string macAddress = fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8());
            std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper); // Convert to uppercase
            double measuredDistance = packet.extractDouble();
            obs->data.push_back({ macAddress, util::time::GetCurrentInsTime(), measuredDistance });
            // Log the measurement details
            LOG_DATA("WiFiObs mac Address: {}, measured distance: {}", macAddress, measuredDistance);
        }
    }
    else
    {
        LOG_DEBUG("Received non-binary packet. Ignoring.");
    }
}

void NAV::vendor::espressif::decryptSingleWiFiObsInstinctTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet)
{
    obs->insTime = util::time::GetCurrentInsTime();
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->payloadLength = packet.extractUint16(); // TODO remove
        // Mac address
        std::string macAddress = fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8());
        std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper); // Convert to uppercase
        // Distance
        int rtt = packet.extractInt32();
        double measuredDistance = static_cast<double>(rtt) * cAir * 1e-9 / 2;
        // Time of measurement
        InsTime_YMDHMS yearMonthDayHMS(packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32(), packet.extractInt32());
        InsTime timeOfMeasurement(yearMonthDayHMS, UTC);
        [[maybe_unused]] int ms = packet.extractInt32();
        // Add the measurement to the WiFiObs
        obs->data.push_back({ macAddress, util::time::GetCurrentInsTime(), measuredDistance });
        // Log the measurement details
        LOG_DATA("WiFiObs mac Address: {}, measured distance: {}", macAddress, measuredDistance);
    }
    else
    {
        LOG_DEBUG("Received non-binary packet. Ignoring.");
    }
}

void NAV::vendor::espressif::decryptMultipleWiFiObsInstinctTime(const std::shared_ptr<NAV::WiFiObs>& obs, uart::protocol::Packet& packet)
{
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->payloadLength = packet.extractUint16();                                           // TODO remove
        size_t measurementLength = 14;                                                         // TODO irgendwo anders definieren
        size_t numOfMeasurement = static_cast<size_t>(obs->payloadLength) / measurementLength; // TODO checksum length = 2U
        for (size_t i = 0; i < numOfMeasurement; i++)
        {
            std::string macAddress = fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8());
            std::transform(macAddress.begin(), macAddress.end(), macAddress.begin(), ::toupper); // Convert to uppercase
            double measuredDistance = packet.extractDouble();
            obs->data.push_back({ macAddress, util::time::GetCurrentInsTime(), measuredDistance });
            // Log the measurement details
            LOG_DATA("WiFiObs mac Address: {}, measured distance: {}", macAddress, measuredDistance);
        }
    }
    else
    {
        LOG_DEBUG("Received non-binary packet. Ignoring.");
    }
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