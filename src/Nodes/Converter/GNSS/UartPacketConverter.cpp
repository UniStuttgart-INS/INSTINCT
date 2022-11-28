// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UartPacketConverter.hpp"

#include <cmath>

#include "util/Logger.hpp"
#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/Vendor/Ublox/UbloxUtilities.hpp"
#include "util/Vendor/Emlid/EmlidUtilities.hpp"

NAV::UartPacketConverter::UartPacketConverter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 340, 65 };

    nm::CreateOutputPin(this, "UbloxObs", Pin::Type::Flow, { NAV::UbloxObs::type() });

    nm::CreateInputPin(this, "UartPacket", Pin::Type::Flow, { NAV::UartPacket::type() }, &UartPacketConverter::receiveObs);
}

NAV::UartPacketConverter::~UartPacketConverter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UartPacketConverter::typeStatic()
{
    return "UartPacketConverter";
}

std::string NAV::UartPacketConverter::type() const
{
    return typeStatic();
}

std::string NAV::UartPacketConverter::category()
{
    return "Converter";
}

void NAV::UartPacketConverter::guiConfig()
{
    if (ImGui::Combo(fmt::format("Output Type##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_outputType), "UbloxObs\0EmlidObs\0\0"))
    {
        LOG_DEBUG("{}: Output Type changed to {}", nameId(), _outputType == OutputType_UbloxObs ? "UbloxObs" : "EmlidObs");

        if (_outputType == OutputType_UbloxObs)
        {
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::UbloxObs::type() };
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::UbloxObs::type();
        }
        else if (_outputType == OutputType_EmlidObs)
        {
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::EmlidObs::type() };
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::EmlidObs::type();
        }

        for (auto& link : outputPins.front().links)
        {
            if (auto* connectedPin = link.getConnectedPin())
            {
                outputPins.front().recreateLink(*connectedPin);
            }
        }

        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::UartPacketConverter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["outputType"] = _outputType;

    return j;
}

void NAV::UartPacketConverter::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputType"))
    {
        j.at("outputType").get_to(_outputType);

        if (!outputPins.empty())
        {
            if (_outputType == OutputType_UbloxObs)
            {
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::UbloxObs::type() };
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::UbloxObs::type();
            }
            else if (_outputType == OutputType_EmlidObs)
            {
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::EmlidObs::type() };
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::EmlidObs::type();
            }
        }
    }
}

bool NAV::UartPacketConverter::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::UartPacketConverter::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto uartPacket = std::static_pointer_cast<const UartPacket>(queue.extract_front());

    std::shared_ptr<NodeData> convertedData = nullptr;

    if (_outputType == OutputType_UbloxObs)
    {
        auto obs = std::make_shared<UbloxObs>();
        auto packet = uartPacket->raw;
        vendor::ublox::decryptUbloxObs(obs, packet);
        convertedData = obs;
    }
    else /* if (_outputType == OutputType_EmlidObs) */
    {
        auto obs = std::make_shared<EmlidObs>();
        auto packet = uartPacket->raw;
        vendor::emlid::decryptEmlidObs(obs, packet);
        convertedData = obs;
    }

    if (!convertedData->insTime.empty())
    {
        if (util::time::GetMode() == util::time::Mode::REAL_TIME)
        {
            util::time::SetCurrentTime(convertedData->insTime);
        }
    }
    else if (auto currentTime = util::time::GetCurrentInsTime();
             !currentTime.empty())
    {
        convertedData->insTime = currentTime;
    }

    if (convertedData)
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_CONVERTED, convertedData);
    }
}