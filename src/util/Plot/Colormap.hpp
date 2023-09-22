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
#include <string>
#include <vector>

namespace NAV
{

class Colormap;

/// @brief Display a colormap button
/// @param[in] label Label to display on the button (unique id for ImGui)
/// @param[in] cmap Colormap to show on the button
/// @param[in] size_arg Size of the button
/// @return True if clicked
/// @note Code from Implot library
bool ColormapButton(const char* label, const Colormap& cmap, const ImVec2& size_arg);

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

    /// @brief Return the map
    [[nodiscard]] const std::vector<std::pair<double, ImColor>>& getColormap() const;

    std::string name;      ///< Name of the Colormap
    bool discrete = false; ///< Whether to have discrete changes of the colors or continuous

  private:
    int64_t id;                                       ///< Unique id of the colormap
    std::vector<std::pair<double, ImColor>> colormap; ///< Sorted list of value/color combinations (value is active if lookup is greater or equal)

    /// @brief Renders the colormap
    /// @note Code from Implot library
    void render() const;

    /// @brief Renders the colormap
    /// @param[in] bounds Bounds where to draw the rect
    /// @note Code from Implot library
    void render(const ImRect& bounds) const;

    friend bool NAV::ColormapButton(const char* label, const Colormap& cmap, const ImVec2& size_arg);
};

/// Global colormaps
extern std::vector<Colormap> GlobalColormaps;

} // namespace NAV
