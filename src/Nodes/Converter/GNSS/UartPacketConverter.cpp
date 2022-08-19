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

        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.front()))
        {
            nm::RefreshLink(link->id);
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

void NAV::UartPacketConverter::receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto uartPacket = std::static_pointer_cast<const UartPacket>(nodeData);

    std::shared_ptr<InsObs> convertedData = nullptr;

    if (_outputType == OutputType_UbloxObs)
    {
        auto obs = std::make_shared<UbloxObs>();
        auto packet = uartPacket->raw; // FIXME: We have to copy our data here because of the const qualifier
        vendor::ublox::decryptUbloxObs(obs, packet, false);
        convertedData = obs;
    }
    else if (_outputType == OutputType_EmlidObs)
    {
        auto obs = std::make_shared<EmlidObs>();
        auto packet = uartPacket->raw; // FIXME: We have to copy our data here because of the const qualifier
        vendor::emlid::decryptEmlidObs(obs, packet, false);
        convertedData = obs;
    }

    if (convertedData->insTime.has_value())
    {
        if (util::time::GetMode() == util::time::Mode::REAL_TIME)
        {
            util::time::SetCurrentTime(convertedData->insTime.value());
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