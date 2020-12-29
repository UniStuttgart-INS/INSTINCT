#include "internal/Pin.hpp"

#include "Nodes/Node.hpp"

#include "NodeRegistry.hpp"

#include "gui/widgets/PinIcon.hpp"

bool NAV::Pin::canCreateLink(const Pin& b) const
{
    bool dataTypesMatch = true;

    if (type == Pin::Type::Flow
        && ((kind == Kind::Output && !NAV::NodeRegistry::NodeDataTypeIsChildOf(dataIdentifier, b.dataIdentifier))
            || (kind == Kind::Input && !NAV::NodeRegistry::NodeDataTypeIsChildOf(b.dataIdentifier, dataIdentifier))))
    {
        dataTypesMatch = false;
    }

    if (type == Pin::Type::Delegate
        && (parentNode == nullptr || b.dataIdentifier != parentNode->type()))
    {
        dataTypesMatch = false;
    }

    if (type == Pin::Type::Object
        && (dataIdentifier.empty() || b.dataIdentifier.empty() || dataIdentifier != b.dataIdentifier))
    {
        dataTypesMatch = false;
    }

    return id != b.id && kind != b.kind && type == b.type && parentNode != b.parentNode && dataTypesMatch;
}

ImColor NAV::Pin::getIconColor() const
{
    switch (Type::Value(type))
    {
    case Type::None:
        return ImColor(0, 0, 0);
    case Type::Flow:
        return ImColor(255, 255, 255);
    case Type::Bool:
        return ImColor(220, 48, 48);
    case Type::Int:
        return ImColor(68, 201, 156);
    case Type::Float:
        return ImColor(147, 226, 74);
    case Type::String:
        return ImColor(124, 21, 153);
    case Type::Object:
        return ImColor(51, 150, 215);
    case Type::Matrix:
        return ImColor(255, 165, 0);
    case Type::Function:
        return ImColor(218, 0, 183);
    case Type::Delegate:
        return ImColor(255, 48, 48);
    }
};

void NAV::Pin::drawPinIcon(bool connected, int alpha) const
{
    namespace PinIcon = gui::widgets::PinIcon;

    PinIcon::Type iconType;
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
    case Type::Function:
        iconType = PinIcon::Type::Grid;
        break;
    case Type::Delegate:
        iconType = PinIcon::Type::Square;
        break;
    default:
        return;
    }

    gui::widgets::PinIcon::Draw(ImVec2(static_cast<float>(m_PinIconSize), static_cast<float>(m_PinIconSize)),
                                iconType, connected, color, ImColor(32, 32, 32, alpha));
};