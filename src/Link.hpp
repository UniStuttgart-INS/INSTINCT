/// @file Link.hpp
/// @brief Link between two pins
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

namespace NAV
{
/// Link between two pins
class Link
{
  public:
    /// @brief Default constructor
    Link() = default;
    /// @brief Destructor
    ~Link() = default;
    /// @brief Copy constructor
    Link(const Link&) = default;
    /// @brief Move constructor
    Link(Link&&) = default;
    /// @brief Copy assignment operator
    Link& operator=(const Link&) = default;
    /// @brief Move assignment operator
    Link& operator=(Link&&) = default;

    /// @brief Constructor
    /// @param[in] id Unique Id of the Link
    /// @param[in] startPinId Id of the start pin of the link
    /// @param[in] endPinId Id of the end pin of the link
    /// @param[in] color Color of the link
    Link(ax::NodeEditor::LinkId id, ax::NodeEditor::PinId startPinId, ax::NodeEditor::PinId endPinId, ImColor color)
        : id(id), startPinId(startPinId), endPinId(endPinId), color(color) {}

    /// Unique Id of the Link
    ax::NodeEditor::LinkId id;
    /// Id of the start pin of the link
    ax::NodeEditor::PinId startPinId;
    /// Id of the end pin of the link
    ax::NodeEditor::PinId endPinId;
    /// Color of the link
    ImColor color;
};

} // namespace NAV