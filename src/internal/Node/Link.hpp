/// @file Link.hpp
/// @brief Link between two pins
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

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
    Link(ax::NodeEditor::LinkId id, ax::NodeEditor::PinId startPinId, ax::NodeEditor::PinId endPinId)
        : id(id), startPinId(startPinId), endPinId(endPinId) {}

    /// Unique Id of the Link
    ax::NodeEditor::LinkId id;
    /// Id of the start pin of the link
    ax::NodeEditor::PinId startPinId;
    /// Id of the end pin of the link
    ax::NodeEditor::PinId endPinId;
};

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] link Link to convert into json
void to_json(json& j, const Link& link);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] link Object to fill from the json
void from_json(const json& j, Link& link);

} // namespace NAV