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
#include "util/Vendor/Espressif/EspressifUtilities.hpp"

#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"

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
    if (ImGui::Combo(fmt::format("Output Type ##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_outputType), "UbloxObs\0EmlidObs\0WiFiObs\0\0"))
    {
        std::string outputTypeString;
        switch (_outputType)
        {
        case OutputType_UbloxObs:
            outputTypeString = "UbloxObs";
            break;
        case OutputType_EmlidObs:
            outputTypeString = "EmlidObs";
            break;
        default:
            outputTypeString = "WiFiObs";
        }

        LOG_DEBUG("{}: Output Type changed to {}", nameId(), outputTypeString);

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
        else if (_outputType == OutputType_WiFiObs)
        {
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::WiFiObs::type() };
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::WiFiObs::type();
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

    if (_outputType == OutputType_WiFiObs)
    {
        if (ImGui::Checkbox(fmt::format("Show SyncIn Pin##{}", size_t(id)).c_str(), &_syncInPin))
        {
            LOG_DEBUG("{}: syncInPin changed to {}", nameId(), _syncInPin);
            flow::ApplyChanges();
            if (_syncInPin && (inputPins.size() <= 1))
            {
                nm::CreateInputPin(this, "SyncIn", Pin::Type::Object, { "TimeSync" });
            }
            else if (!_syncInPin)
            {
                nm::DeleteInputPin(inputPins.at(INPUT_PORT_INDEX_SYNC_IN));
            }
        }
        // Remove the extra flow::ApplyChanges() statement here
    }
}

[[nodiscard]] json NAV::UartPacketConverter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["outputType"] = _outputType;
    j["syncInPin"] = _syncInPin;

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
            else if (_outputType == OutputType_WiFiObs)
            {
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::WiFiObs::type() };
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::WiFiObs::type();
            }
        }
    }
    if (j.contains("syncInPin"))
    {
        j.at("syncInPin").get_to(_syncInPin);
        if (_syncInPin && inputPins.size() <= 1)
        {
            nm::CreateInputPin(this, "SyncIn", Pin::Type::Object, { "TimeSync" });
        }
    }
}

bool NAV::UartPacketConverter::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::UartPacketConverter::receiveObs(NAV::InputPin::NodeDataQueue& queue, [[maybe_unused]] size_t pinIdx)
{
    auto uartPacket = std::static_pointer_cast<const UartPacket>(queue.extract_front());
    // auto timeSyncMaster = std::static_pointer_cast<const TimeSync>(queue.extract_front());

    std::shared_ptr<NodeData>
        convertedData = nullptr;

    if (_outputType == OutputType_UbloxObs)
    {
        auto obs = std::make_shared<UbloxObs>();
        auto packet = uartPacket->raw;
        if (!vendor::ublox::decryptUbloxObs(obs, packet, nameId())) { return; };
        convertedData = obs;
    }
    else if (_outputType == OutputType_WiFiObs)
    {
        auto obs = std::make_shared<WiFiObs>();
        auto packet = uartPacket->raw;

        vendor::espressif::decryptWiFiObs(obs, packet, nameId());
        if (_syncInPin && inputPins.at(1).isPinLinked())
        {
            auto timeSyncMaster = getInputValue<const NAV::VectorNavSensor::TimeSync>(0);
            if (_lastSyncInCnt > obs->timeOutputs.syncInCnt) // reset of the slave
            {
                _syncOutCntCorr = 0;
                _syncInCntCorr = timeSyncMaster->v->syncOutCnt;
            }
            else if (_lastSyncOutCnt > timeSyncMaster->v->syncOutCnt) // reset of the master
            {
                _syncInCntCorr = 0;
                _syncOutCntCorr = obs->timeOutputs.syncInCnt;
            }
            else if (_lastSyncOutCnt == 0 && timeSyncMaster->v->syncOutCnt > 1) // slave counter started later
            {
                _syncInCntCorr = timeSyncMaster->v->syncOutCnt;
            }
            else if (_lastSyncOutCnt == 0 && obs->timeOutputs.syncInCnt > 1) // master counter started later
            {
                _syncOutCntCorr = obs->timeOutputs.syncInCnt;
            }
            _lastSyncOutCnt = timeSyncMaster->v->syncOutCnt;
            _lastSyncInCnt = obs->timeOutputs.syncInCnt;
            int64_t syncCntDiff = obs->timeOutputs.syncInCnt + _syncInCntCorr - timeSyncMaster->v->syncOutCnt - _syncOutCntCorr;
            obs->insTime = timeSyncMaster->v->ppsTime + std::chrono::microseconds(obs->timeOutputs.timeSyncIn)
                           + std::chrono::seconds(syncCntDiff);
            // LOG_DATA("{}: Syncing time {}, pps {}, syncOutCnt {}, syncInCnt {}, syncCntDiff {}, syncInCntCorr {}, syncOutCntCorr {}",
            //          nameId(), obs->insTime.toGPSweekTow(), timeSyncMaster->ppsTime.toGPSweekTow(),
            //          timeSyncMaster->syncOutCnt, obs->timeOutputs.syncInCnt, syncCntDiff, _syncInCntCorr, _syncOutCntCorr);
        }
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