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

#include "util/Time/TimeBase.hpp"

void NAV::vendor::espressif::decryptEspressifObs(const std::shared_ptr<NAV::EspressifObs>& obs, uart::protocol::Packet& packet)
{
    obs->insTime = util::time::GetCurrentInsTime();
    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        obs->payloadLength = packet.extractUint16();                                           // TODO remove
        size_t measurementLength = 14;                                                         // TODO irgendwo anders definieren
        size_t numOfMeasurement = static_cast<size_t>(obs->payloadLength) / measurementLength; // TODO checksum length = 2U
        for (size_t i = 0; i < numOfMeasurement; i++)
        {
            std::array<uint8_t, 6> macAdress{ packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8(), packet.extractUint8() };
            double measuredDistance = packet.extractDouble();
            obs->data.push_back({ macAdress, measuredDistance });
            // Log the measurement details
            LOG_DATA("EspressifObs Measurement {}: RouterID {}, Position ({}, {}, {}), Distance {}",
                     i + 1, routerId, routerPosition[0], routerPosition[1], routerPosition[2], measuredDistance);
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