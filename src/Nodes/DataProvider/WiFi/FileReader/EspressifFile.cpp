// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EspressifFile.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/Vendor/Espressif/EspressifUtilities.hpp"
#include "util/Time/TimeBase.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"

NAV::EspressifFile::EspressifFile()
    : Node(typeStatic()), _sensor(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateOutputPin(this, "WiFiObs", Pin::Type::Flow, { NAV::WiFiObs::type() }, &EspressifFile::pollData);
}

NAV::EspressifFile::~EspressifFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::EspressifFile::typeStatic()
{
    return "EspressifFile";
}

std::string NAV::EspressifFile::type() const
{
    return typeStatic();
}

std::string NAV::EspressifFile::category()
{
    return "Data Provider";
}

void NAV::EspressifFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".bin,.*", { ".bin" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            doReinitialize();
        }
        else
        {
            doDeinitialize();
        }
    }
}

[[nodiscard]] json NAV::EspressifFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::EspressifFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::EspressifFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::EspressifFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::EspressifFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::EspressifFile::pollData()
{
    uint8_t i = 0;
    std::unique_ptr<uart::protocol::Packet> packet = nullptr;
    std::shared_ptr<WiFiObs> obs;
    while (true)
    {
        while (!eof() && read(reinterpret_cast<char*>(&i), 1))
        {
            packet = _sensor.findPacket(i);

            if (packet != nullptr)
            {
                break;
            }
        }

        if (!packet || eof())
        {
            LOG_DEBUG("{}: End of file reached.", nameId());
            return nullptr;
        }

        if (packet->getRawDataLength() == 0)
        {
            LOG_TRACE("{}: Packet has empty payload", nameId());
            return nullptr;
        }

        obs = std::make_shared<WiFiObs>();
        if (!vendor::espressif::decryptWiFiObs(obs, *packet, nameId())) { continue; };
        if (packet->type() != uart::protocol::Packet::Type::TYPE_BINARY) { continue; };

        if (!obs->insTime.empty())
        {
            _lastObsTime = obs->insTime;
        }
        else
        {
            if (!_lastObsTime.empty())
            {
                obs->insTime = _lastObsTime;
            }
            else
            {
                LOG_DATA("{}: Could not set valid time. Skipping package.", nameId());
                continue;
            }
        }
        break;
    }

    LOG_DATA("{}: [{}] Packet found [{}][{}]", nameId(), obs->insTime.toYMDHMS(GPST),
             obs->msgClass, vendor::ublox::getStringFromMsgId(obs->msgClass, obs->msgId));

    invokeCallbacks(OUTPUT_PORT_INDEX_WiFiObs_OBS, obs);
    return obs;
}

NAV::FileReader::FileType NAV::EspressifFile::determineFileType()
{
    LOG_TRACE("called for {}", nameId());

    auto filestream = std::ifstream(getFilepath());

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == vendor::espressif::EspressifUartSensor::BINARY_SYNC_CHAR_1
             && static_cast<uint8_t>(buffer.at(1)) == vendor::espressif::EspressifUartSensor::BINARY_SYNC_CHAR_2))
        {
            filestream.close();
            LOG_DEBUG("{} has the file type: Binary", nameId());
            return FileType::BINARY;
        }
        filestream.close();

        LOG_ERROR("{} could not determine file type", nameId());
        return FileType::NONE;
    }
    LOG_ERROR("{} could not open file {}", nameId(), getFilepath());
    return FileType::NONE;
}