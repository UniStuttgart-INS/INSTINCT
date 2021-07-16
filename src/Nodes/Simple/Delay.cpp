#include "Delay.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/InsObs.hpp"

NAV::Delay::Delay()
{
    name = fmt::format("z^-{}", delayLength);

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 305, 70 };
    kind = Kind::Simple;

    nm::CreateInputPin(this, "", Pin::Type::Flow, { InsObs::type() }, &Delay::delayObs);
    nm::CreateOutputPin(this, "", Pin::Type::Flow, InsObs::type());
}

NAV::Delay::~Delay()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Delay::typeStatic()
{
    return "Delay";
}

std::string NAV::Delay::type() const
{
    return typeStatic();
}

std::string NAV::Delay::category()
{
    return "Simple";
}

void NAV::Delay::guiConfig()
{
    if (ImGui::InputInt(fmt::format("Delay length##{}", size_t(id)).c_str(), &delayLength))
    {
        if (delayLength < 1)
        {
            delayLength = 1;
        }
        LOG_DEBUG("{}: delayLength changed to {}", nameId(), delayLength);
        if (name.starts_with("z^-"))
        {
            name = fmt::format("z^-{}", delayLength);
        }

        while (buffer.size() > static_cast<size_t>(delayLength))
        {
            buffer.pop_front();
        }
    }
}

[[nodiscard]] json NAV::Delay::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["delayLength"] = delayLength;

    return j;
}

void NAV::Delay::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("delayLength"))
    {
        j.at("delayLength").get_to(delayLength);
    }
}

bool NAV::Delay::initialize()
{
    LOG_TRACE("{}: called", nameId());

    buffer.clear();

    return true;
}

void NAV::Delay::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::Delay::onCreateLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if (endPin->parentNode->id != id)
        {
            return true; // Link on Output Port
        }

        // New Link on the Input port, but the previously connected dataIdentifier is different from the new one.
        // Then remove all links.
        if (outputPins.at(OutputPortIndex_Flow).dataIdentifier != startPin->dataIdentifier)
        {
            for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Flow).id))
            {
                nm::DeleteLink(link->id);
            }
        }

        // Update the dataIdentifier of the output pin to the same as input pin
        outputPins.at(OutputPortIndex_Flow).dataIdentifier = startPin->dataIdentifier;

        // Refresh all links connected to the output pin
        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Flow).id))
        {
            nm::RefreshLink(link->id);
        }
    }

    return true;
}

void NAV::Delay::delayObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    if (buffer.size() == static_cast<size_t>(delayLength))
    {
        auto oldest = buffer.front();
        buffer.pop_front();
        buffer.push_back(nodeData);

        if (auto obs = std::dynamic_pointer_cast<InsObs>(nodeData))
        {
            LOG_DATA("{}: Delay pushing out message: {}", nameId(), obs->insTime->toGPSweekTow());
        }
        else
        {
            LOG_DATA("{}: Delay pushing out message", nameId());
        }

        invokeCallbacks(OutputPortIndex_Flow, oldest);
    }
    else
    {
        buffer.push_back(nodeData);
    }
}