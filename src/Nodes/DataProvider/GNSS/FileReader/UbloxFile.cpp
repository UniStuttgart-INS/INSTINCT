// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UbloxFile.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/Vendor/Ublox/UbloxUtilities.hpp"
#include "util/Time/TimeBase.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxFile::UbloxFile()
    : Node(typeStatic()), _sensor(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateOutputPin(this, "UbloxObs", Pin::Type::Flow, { NAV::UbloxObs::type() }, &UbloxFile::pollData);
}

NAV::UbloxFile::~UbloxFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxFile::typeStatic()
{
    return "UbloxFile";
}

std::string NAV::UbloxFile::type() const
{
    return typeStatic();
}

std::string NAV::UbloxFile::category()
{
    return "Data Provider";
}

void NAV::UbloxFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".ubx,.*", { ".ubx" }, size_t(id), nameId()))
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

[[nodiscard]] json NAV::UbloxFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::UbloxFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::UbloxFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _lastObsTime.reset();

    return FileReader::initialize();
}

void NAV::UbloxFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::UbloxFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::UbloxFile::pollData()
{
    uint8_t i = 0;
    std::unique_ptr<uart::protocol::Packet> packet = nullptr;
    std::shared_ptr<UbloxObs> obs;
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

        obs = std::make_shared<UbloxObs>();
        if (!vendor::ublox::decryptUbloxObs(obs, *packet, nameId())) { continue; };
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

    invokeCallbacks(OUTPUT_PORT_INDEX_UBLOX_OBS, obs);
    return obs;
}

NAV::FileReader::FileType NAV::UbloxFile::determineFileType()
{
    LOG_TRACE("called for {}", nameId());

    auto filestream = std::ifstream(getFilepath());

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == vendor::ublox::UbloxUartSensor::BINARY_SYNC_CHAR_1
             && static_cast<uint8_t>(buffer.at(1)) == vendor::ublox::UbloxUartSensor::BINARY_SYNC_CHAR_2)
            || buffer.at(0) == vendor::ublox::UbloxUartSensor::ASCII_START_CHAR)
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