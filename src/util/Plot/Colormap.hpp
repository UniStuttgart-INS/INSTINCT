// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Colormap.hpp
/// @brief Colormap
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-22

#pragma once

#include <imgui.h>
#include <imgui_internal.h>
#include <optional>
#include <functional>
#include <string>
#include <vector>
#include "util/Json.hpp"

namespace NAV
{

class Colormap;

/// @brief Display a colormap button
/// @param[in] label Label to display on the button (unique id for ImGui)
/// @param[in, out] cmap Colormap to show on the button and edit in the popup
/// @param[in] size_arg Size of the button
/// @return True if clicked
/// @note Code from Implot library
bool ColormapButton(const char* label, Colormap& cmap, const ImVec2& size_arg);

/// @brief Converts the provided object into a json object
/// @param[out] j Return Json object
/// @param[in] cmap Colormap to convert
void to_json(json& j, const Colormap& cmap);
/// @brief Converts the provided json object into a struct
/// @param[in] j Json object with the vector values
/// @param[out] cmap Struct to return
void from_json(const json& j, Colormap& cmap);

/// @brief Colormap class
class Colormap
{
  public:
    /// @brief Constructor
    Colormap();

    /// @brief Add a color with value
    /// @param value Value
    /// @param color Color
    void addColor(double value, ImColor color);

    /// @brief Remove the entry at the index (if past the last index or empty, NoOp)
    /// @param idx Index to remove
    void removeColor(size_t idx);

    /// @brief Gets the color for the given value
    /// @param[in] value Value to look for
    /// @param[in] defaultColor Default color to display if no enty in the colormap matches
    /// @return The color for the value or the given defaultColor if nothing matches
    [[nodiscard]] ImColor getColor(double value, const ImColor& defaultColor) const;

    /// Return the id of the colormap
    [[nodiscard]] int64_t getId() const;

    /// @brief Return the map
    [[nodiscard]] const std::vector<std::pair<double, ImColor>>& getColormap() const;

    std::string name = "Colormap"; ///< Name of the Colormap
    bool discrete = false;         ///< Whether to have discrete changes of the colors or continuous
    size_t version = 0;            ///< Version, to tell nodes that the colormap was updated

  private:
    /// Unique id of the colormap
    int64_t id;
    /// Sorted list of value/color combinations (value is active if lookup is greater or equal)
    std::vector<std::pair<double, ImColor>> colormap = { { 0.0, ImColor(1.0F, 1.0F, 1.0F, 1.0F) } };

    /// @brief Renders the colormap
    /// @note Code from Implot library
    void render() const;

    /// @brief Renders the colormap
    /// @param[in] bounds Bounds where to draw the rect
    /// @note Code from Implot library
    void render(const ImRect& bounds) const;

    /// @brief Display a colormap button
    /// @param[in] label Label to display on the button (unique id for ImGui)
    /// @param[in, out] cmap Colormap to show on the button and edit in the popup
    /// @param[in] size_arg Size of the button
    /// @return True if clicked
    /// @note Code from Implot library
    friend bool NAV::ColormapButton(const char* label, Colormap& cmap, const ImVec2& size_arg);

    /// @brief Converts the provided object into a json object
    /// @param[out] j Return Json object
    /// @param[in] cmap Colormap to convert
    friend void to_json(json& j, const Colormap& cmap);
    /// @brief Converts the provided json object into a struct
    /// @param[in] j Json object with the vector values
    /// @param[out] cmap Struct to return
    friend void from_json(const json& j, Colormap& cmap);
};

/// Global colormaps
extern std::vector<Colormap> ColormapsGlobal;
/// Flow colormaps
extern std::vector<Colormap> ColormapsFlow;

/// @brief Type of the Colormap mask
enum class ColormapMaskType : int
{
    None,   ///< Do not use a colormap mask
    Global, ///< Use the global colormap
    Flow,   ///< Use the flow colormap
};

/// @brief Shows a combobox to select a colormap
/// @param[in, out] type Type of the selected colormap
/// @param[in, out] id Id of the selected colormap
/// @return True if a change was made
bool ShowColormapSelector(ColormapMaskType& type, int64_t& id);

/// @brief Searches for the colormap in the Global and Flow colormaps
/// @param type Type of the colormap (global or flow)
/// @param id Id of the colormap
/// @return The colormap if one could be found
std::optional<std::reference_wrapper<const Colormap>> ColormapSearch(const ColormapMaskType& type, const int64_t& id);

} // namespace NAV
