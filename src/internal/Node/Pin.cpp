#include "internal/Node/Pin.hpp"

#include "internal/Node/Node.hpp"

#include <imgui_node_editor.h>

#include "NodeRegistry.hpp"

#include "internal/gui/widgets/PinIcon.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

bool NAV::InputPin::canCreateLink(const OutputPin& other) const
{
    return Pin::canCreateLink(other, *this);
}

bool NAV::OutputPin::canCreateLink(const InputPin& other) const
{
    return Pin::canCreateLink(*this, other);
}

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

bool NAV::OutputPin::connect(NAV::InputPin& endPin)
{
    if (!canCreateLink(endPin))
    {
        return false;
    }

    auto iter = std::find(links.begin(), links.end(), [&endPin](const OutgoingLink& link) {
        return link.connectedNode == endPin.parentNode && link.connectedPinId == endPin.id;
    });
    if (iter == links.end()) // Link does not yet exist
    {
        if (endPin.link.connectedNode == parentNode && endPin.link.connectedPinId == id)
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

    return true;
}

void NAV::OutputPin::disconnect(InputPin& endPin)
{
    auto iter = std::find(links.begin(), links.end(), [&endPin](const OutgoingLink& link) {
        return link.connectedNode == endPin.parentNode && link.connectedPinId == endPin.id;
    });
    if (iter != links.end())
    {
        links.erase(iter);
        if (endPin.link.connectedNode == parentNode && endPin.link.connectedPinId == id)
        {
            endPin.disconnect(*this);
        }
    }
}

bool NAV::InputPin::connect(NAV::OutputPin& startPin)
{
    if (!canCreateLink(startPin))
    {
        return false;
    }

    if (link.connectedNode != startPin.parentNode || link.connectedPinId != startPin.id) // Link does not yet exist
    {
        link.connectedNode = startPin.parentNode;
        link.connectedPinId = startPin.id;

        auto iter = std::find(startPin.links.begin(), startPin.links.end(), [&, this](const OutputPin::OutgoingLink& link) {
            return link.connectedNode == parentNode && link.connectedPinId == id;
        });

        if (iter != startPin.links.end())
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

void NAV::InputPin::disconnect(NAV::OutputPin& startPin)
{
    if (link.connectedNode || link.connectedPinId || link.linkId)
    {
        link.linkId = 0;
        link.connectedNode = nullptr;
        link.connectedPinId = 0;

        auto iter = std::find(startPin.links.begin(), startPin.links.end(), [&, this](const OutputPin::OutgoingLink& link) {
            return link.connectedNode == parentNode && link.connectedPinId == id;
        });

        if (iter != startPin.links.end())
        {
            startPin.disconnect(*this);
        }
    }
}

const NAV::OutputPin* NAV::InputPin::IncomingLink::getConnectedPin() const
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

void NAV::to_json(json& j, const Pin& pin)
{
    j = json{
        { "id", size_t(pin.id) },
        // { "type", std::string(pin.type) },
        { "name", pin.name },
        // { "dataIdentifier", pin.dataIdentifier },
    };
}
void NAV::from_json(const json& j, Pin& pin)
{
    pin.id = j.at("id").get<size_t>();

    // if (j.contains("type"))
    // {
    //     pin.type = Pin::Type(j.at("type").get<std::string>());
    // }

    if (j.contains("name"))
    {
        j.at("name").get_to(pin.name);
    }

    // if (j.contains("dataIdentifier"))
    // {
    //     j.at("dataIdentifier").get_to(pin.dataIdentifier);
    // }
}