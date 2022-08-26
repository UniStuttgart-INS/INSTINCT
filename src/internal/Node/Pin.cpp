#include "internal/Node/Pin.hpp"

#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/PinIcon.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/Assert.h"

#include "NodeRegistry.hpp"

#include <imgui_node_editor.h>
#include <algorithm>

// ###########################################################################################################
//                                                    Pin
// ###########################################################################################################

bool NAV::Pin::canCreateLink(const OutputPin& startPin, const InputPin& endPin)
{
    bool dataTypesMatch = true;

    if (startPin.type == Pin::Type::Flow
        && !NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(startPin.dataIdentifier, endPin.dataIdentifier))
    {
        dataTypesMatch = false;
    }

    if (startPin.type == Pin::Type::Delegate
        && (startPin.parentNode == nullptr
            || std::find(endPin.dataIdentifier.begin(), endPin.dataIdentifier.end(), startPin.parentNode->type()) == endPin.dataIdentifier.end()))
    {
        dataTypesMatch = false;
    }

    if ((startPin.type == Pin::Type::Object || startPin.type == Pin::Type::Matrix) // NOLINT(misc-redundant-expression) // FIXME: error: equivalent expression on both sides of logical operator
        && !dataIdentifierHaveCommon(startPin.dataIdentifier, endPin.dataIdentifier))
    {
        dataTypesMatch = false;
    }

    return startPin.id != endPin.id                    // Different Pins
           && startPin.kind != endPin.kind             // Input <=> Output
           && startPin.type == endPin.type             // Same Type (Flow, Object, ...)
           && startPin.parentNode != endPin.parentNode // Different Nodes
           && dataTypesMatch;                          // Data identifier match
}

bool NAV::Pin::dataIdentifierHaveCommon(const std::vector<std::string>& a, const std::vector<std::string>& b)
{
    return !a.empty() && !b.empty()
           && std::find_if(a.begin(),
                           a.end(),
                           [&b](const std::string& str) { return std::find(b.begin(), b.end(), str) != b.end(); })
                  != a.end();
}

ImColor NAV::Pin::getIconColor() const
{
    switch (Type::Value(type))
    {
    case Type::None:
        return { 0, 0, 0 };
    case Type::Flow:
        if (ax::NodeEditor::GetStyle().Colors[ax::NodeEditor::StyleColor_NodeBg].x
                + ax::NodeEditor::GetStyle().Colors[ax::NodeEditor::StyleColor_NodeBg].y
                + ax::NodeEditor::GetStyle().Colors[ax::NodeEditor::StyleColor_NodeBg].z
            > 2.0F)
        {
            return { 0, 0, 0 };
        }
        return { 255, 255, 255 };
    case Type::Bool:
        return { 220, 48, 48 };
    case Type::Int:
        return { 68, 201, 156 };
    case Type::Float:
        return { 147, 226, 74 };
    case Type::String:
        return { 124, 21, 153 };
    case Type::Object:
        return { 51, 150, 215 };
    case Type::Matrix:
        return { 255, 165, 0 };
    case Type::Delegate:
        return { 255, 48, 48 };
    }
    return { 0, 0, 0 };
}

void NAV::Pin::drawPinIcon(bool connected, int alpha) const
{
    namespace PinIcon = gui::widgets::PinIcon;

    PinIcon::Type iconType = PinIcon::Type::Flow;
    ImColor color = getIconColor();
    color.Value.w = static_cast<float>(alpha) / 255.0F;
    switch (Type::Value(type))
    {
    case Type::None:
        iconType = PinIcon::Type::Grid;
        break;
    case Type::Flow:
        iconType = PinIcon::Type::Flow;
        break;
    case Type::Bool:
        // iconType = PinIcon::Type::Circle;
        // break;
    case Type::Int:
        // iconType = PinIcon::Type::Circle;
        // break;
    case Type::Float:
        iconType = PinIcon::Type::Circle;
        break;
    case Type::String:
        iconType = PinIcon::Type::RoundSquare;
        break;
    case Type::Object:
        // iconType = PinIcon::Type::Diamond;
        // break;
    case Type::Matrix:
        iconType = PinIcon::Type::Diamond;
        break;
    case Type::Delegate:
        iconType = PinIcon::Type::Square;
        break;
    default:
        return;
    }

    gui::widgets::PinIcon::Draw(ImVec2(static_cast<float>(m_PinIconSize), static_cast<float>(m_PinIconSize)),
                                iconType, connected, color, ImColor(32, 32, 32, alpha));
}

bool NAV::Pin::createLink(OutputPin& startPin, InputPin& endPin, ax::NodeEditor::LinkId linkId)
{
    if (!startPin.canCreateLink(endPin)) { return false; }

    if (!startPin.parentNode || !endPin.parentNode) { return false; }
    LOG_TRACE("called: {} of [{}] ==> {} of [{}]", size_t(startPin.id), startPin.parentNode->nameId(), size_t(endPin.id), endPin.parentNode->nameId());

    if (!startPin.parentNode->onCreateLink(startPin, endPin))
    {
        LOG_ERROR("The new Link between node '{}' and '{}' was refused by its start node.",
                  startPin.parentNode->nameId(), endPin.parentNode->nameId());
        return false;
    }
    if (!endPin.parentNode->onCreateLink(startPin, endPin))
    {
        LOG_ERROR("The new Link between node '{}' and '{}' was refused by its end node.",
                  startPin.parentNode->nameId(), endPin.parentNode->nameId());
        return false;
    }

    LOG_DEBUG("Creating link from pin {} of [{}] ==> {} of [{}]",
              size_t(startPin.id), startPin.parentNode->nameId(),
              size_t(endPin.id), endPin.parentNode->nameId());

    startPin.connect(endPin, linkId);
    endPin.connect(startPin, linkId);

    if (endPin.type != Pin::Type::Flow)
    {
        if (startPin.parentNode && endPin.parentNode && !startPin.parentNode->isInitialized())
        {
            if (endPin.parentNode->isInitialized())
            {
                endPin.parentNode->doDeinitialize(true);
            }
        }
    }

    if (startPin.parentNode && endPin.parentNode)
    {
        startPin.parentNode->afterCreateLink(startPin, endPin);
        endPin.parentNode->afterCreateLink(startPin, endPin);
    }

    flow::ApplyChanges();

    return true;
}

bool NAV::Pin::recreateLink(OutputPin& startPin, InputPin& endPin)
{
    if (startPin.isPinLinked(endPin))
    {
        deleteLink(startPin, endPin);
        return createLink(startPin, endPin);
    }
    return false;
}

void NAV::Pin::deleteLink(OutputPin& startPin, InputPin& endPin)
{
    if (!startPin.parentNode || !endPin.parentNode) { return; }

    if (!startPin.isPinLinked(endPin))
    {
        LOG_ERROR("Cannot delete the link, because the nodes '{}' and '{}' are not linked over pins '{}' => '{}'.",
                  startPin.parentNode->nameId(), endPin.parentNode->nameId(),
                  size_t(startPin.id), size_t(endPin.id));
        return;
    }

    LOG_DEBUG("Deleting link {} from pin {} of [{}] ==> pin {} of [{}]", size_t(endPin.link.linkId),
              size_t(startPin.id), startPin.parentNode->nameId(), size_t(endPin.id), endPin.parentNode->nameId());

    startPin.parentNode->onDeleteLink(startPin, endPin);
    endPin.parentNode->onDeleteLink(startPin, endPin);

    if (endPin.type != Pin::Type::Flow) { endPin.parentNode->doDeinitialize(true); }

    startPin.disconnect(endPin);
    endPin.disconnect();

    startPin.parentNode->afterDeleteLink(startPin, endPin);
    endPin.parentNode->afterDeleteLink(startPin, endPin);

    flow::ApplyChanges();
}

// ###########################################################################################################
//                                                 OutputPin
// ###########################################################################################################

NAV::OutputPin::~OutputPin()
{
    deleteLinks();
}

bool NAV::OutputPin::canCreateLink(const NAV::InputPin& other) const
{
    return Pin::canCreateLink(*this, other);
}

bool NAV::OutputPin::isPinLinked() const
{
    return !links.empty();
}

bool NAV::OutputPin::isPinLinked(const NAV::InputPin& endPin) const
{
    auto iter = std::find_if(links.cbegin(), links.cend(), [&endPin](const OutgoingLink& link) {
        return link.connectedNode == endPin.parentNode && link.connectedPinId == endPin.id;
    });
    return iter != links.cend();
}

bool NAV::OutputPin::createLink(InputPin& endPin, ax::NodeEditor::LinkId linkId)
{
    return Pin::createLink(*this, endPin, linkId);
}

bool NAV::OutputPin::recreateLink(InputPin& endPin)
{
    return Pin::recreateLink(*this, endPin);
}

void NAV::OutputPin::deleteLink(InputPin& endPin)
{
    Pin::deleteLink(*this, endPin);
}

void NAV::OutputPin::deleteLinks()
{
    while (!links.empty())
    {
        if (auto* endPin = links.back().getConnectedPin())
        {
            Pin::deleteLink(*this, *endPin);
        }
        else
        {
            links.pop_back();
        }
    }
}

void NAV::OutputPin::connect(NAV::InputPin& endPin, ax::NodeEditor::LinkId linkId)
{
    auto iter = std::find_if(links.begin(), links.end(), [&endPin](const OutgoingLink& link) {
        return link.connectedNode == endPin.parentNode && link.connectedPinId == endPin.id;
    });
    if (iter == links.end()) // Link does not yet exist
    {
        if (linkId)
        {
            // LinkId is given
            links.emplace_back(linkId, endPin.parentNode, endPin.id);
            if (endPin.link.linkId != linkId)
            {
                endPin.disconnect();
                endPin.connect(*this, linkId);
            }
        }
        else if (endPin.link.connectedNode == parentNode && endPin.link.connectedPinId == id)
        {
            // Connected pin is already linked, so get linkId from the connected pin
            links.emplace_back(endPin.link.linkId, endPin.parentNode, endPin.id);
        }
        else
        {
            // Connected pin is not linked, so get new linkId
            links.emplace_back(nm::GetNextLinkId(), endPin.parentNode, endPin.id);
            // Also connect the endPin to this one
            endPin.connect(*this);
        }
    }
}

void NAV::OutputPin::disconnect(InputPin& endPin)
{
    auto iter = std::find_if(links.begin(), links.end(), [&endPin](const OutgoingLink& link) {
        return link.connectedNode == endPin.parentNode && link.connectedPinId == endPin.id;
    });
    if (iter != links.end())
    {
        links.erase(iter);
        if (endPin.link.connectedNode == parentNode && endPin.link.connectedPinId == id)
        {
            endPin.disconnect();
        }
    }
}

NAV::InputPin* NAV::OutputPin::OutgoingLink::getConnectedPin() const
{
    if (connectedNode)
    {
        for (auto& inputPin : connectedNode->inputPins)
        {
            if (inputPin.id == connectedPinId) { return &inputPin; }
        }
    }
    return nullptr;
}

// ###########################################################################################################
//                                                 InputPin
// ###########################################################################################################

NAV::InputPin::~InputPin()
{
    deleteLink();
}

bool NAV::InputPin::canCreateLink(const OutputPin& other) const
{
    return Pin::canCreateLink(other, *this);
}

bool NAV::InputPin::isPinLinked() const
{
    return link.linkId && link.connectedNode && link.connectedPinId;
}

bool NAV::InputPin::createLink(OutputPin& startPin, ax::NodeEditor::LinkId linkId)
{
    return Pin::createLink(startPin, *this, linkId);
}

bool NAV::InputPin::recreateLink(OutputPin& startPin)
{
    return Pin::recreateLink(startPin, *this);
}

void NAV::InputPin::deleteLink()
{
    if (auto* startPin = link.getConnectedPin())
    {
        Pin::deleteLink(*startPin, *this);
    }
}

void NAV::InputPin::connect(NAV::OutputPin& startPin, ax::NodeEditor::LinkId linkId)
{
    if (link.connectedNode != startPin.parentNode || link.connectedPinId != startPin.id) // Link does not yet exist
    {
        link.connectedNode = startPin.parentNode;
        link.connectedPinId = startPin.id;

        auto iter = std::find_if(startPin.links.begin(), startPin.links.end(), [&, this](const OutputPin::OutgoingLink& link) {
            return link.connectedNode == parentNode && link.connectedPinId == id;
        });

        if (linkId)
        {
            // LinkId is given
            link.linkId = linkId;
            if (iter != startPin.links.end() && iter->linkId != linkId)
            {
                startPin.disconnect(*this);
                startPin.connect(*this, linkId);
            }
        }
        else if (iter != startPin.links.end())
        {
            // Connected pin is already linked, so get linkId from the connected pin
            link.linkId = iter->linkId;
        }
        else
        {
            // Connected pin is not linked, so get new linkId
            link.linkId = nm::GetNextLinkId();
            // Also connect the startPin to this one
            startPin.connect(*this);
        }
    }
}

void NAV::InputPin::disconnect()
{
    if (link.connectedNode || link.connectedPinId || link.linkId)
    {
        auto* startPin = link.getConnectedPin();

        link.linkId = 0;
        link.connectedNode = nullptr;
        link.connectedPinId = 0;

        if (startPin)
        {
            auto iter = std::find_if(startPin->links.begin(), startPin->links.end(), [&, this](const OutputPin::OutgoingLink& link) {
                return link.connectedNode == parentNode && link.connectedPinId == id;
            });

            if (iter != startPin->links.end())
            {
                startPin->disconnect(*this);
            }
        }
    }
}

NAV::OutputPin* NAV::InputPin::IncomingLink::getConnectedPin() const
{
    if (connectedNode)
    {
        for (auto& outputPin : connectedNode->outputPins)
        {
            if (outputPin.id == connectedPinId) { return &outputPin; }
        }
    }
    return nullptr;
}

// ###########################################################################################################

void NAV::to_json(json& j, const OutputPin& pin)
{
    j = json{
        { "id", size_t(pin.id) },
        { "name", pin.name },
    };
}
void NAV::from_json(const json& j, OutputPin& pin)
{
    pin.id = j.at("id").get<size_t>();
    if (j.contains("name")) { j.at("name").get_to(pin.name); }
}

void NAV::to_json(json& j, const InputPin& pin)
{
    j = json{
        { "id", size_t(pin.id) },
        { "name", pin.name },
    };
}
void NAV::from_json(const json& j, InputPin& pin)
{
    pin.id = j.at("id").get<size_t>();
    if (j.contains("name")) { j.at("name").get_to(pin.name); }
}