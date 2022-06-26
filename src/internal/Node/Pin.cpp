#include "internal/Node/Pin.hpp"

#include "internal/Node/Node.hpp"

#include <imgui_node_editor.h>

#include "NodeRegistry.hpp"

#include "internal/gui/widgets/PinIcon.hpp"

bool NAV::Pin::canCreateLink(const Pin& b) const
{
    bool dataTypesMatch = true;

    const Pin* startPin = (kind == Kind::Output ? this : &b);
    const Pin* endPin = (kind == Kind::Output ? &b : this);

    if (startPin->type == Pin::Type::Flow
        && !NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(startPin->dataIdentifier, endPin->dataIdentifier))
    {
        dataTypesMatch = false;
    }

    if (startPin->type == Pin::Type::Delegate
        && (parentNode == nullptr
            || std::find(endPin->dataIdentifier.begin(), endPin->dataIdentifier.end(), startPin->parentNode->type()) == endPin->dataIdentifier.end()))
    {
        dataTypesMatch = false;
    }

    if ((startPin->type == Pin::Type::Object || startPin->type == Pin::Type::Matrix) // NOLINT(misc-redundant-expression) // FIXME: error: equivalent expression on both sides of logical operator
        && !dataIdentifierHaveCommon(dataIdentifier, b.dataIdentifier))
    {
        dataTypesMatch = false;
    }

    return id != b.id                    // Different Pins
           && kind != b.kind             // Input <=> Output
           && type == b.type             // Same Type (Flow, Object, ...)
           && parentNode != b.parentNode // Different Nodes
           && dataTypesMatch;            // Data identifier match
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