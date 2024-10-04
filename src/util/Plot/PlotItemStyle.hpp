// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PlotItemStyle.hpp
/// @brief Specifying the look of a certain line in the plot
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-15

#pragma once

#include <string>
#include <imgui.h>
#include <implot.h>

#include "util/Json.hpp"
#include "util/Plot/Colormap.hpp"
#include "util/Plot/PlotEventTooltip.hpp"
#include "util/Container/ScrollingBuffer.hpp"

namespace NAV
{

/// @brief Specifying the look of a certain line in the plot
struct PlotItemStyle
{
    /// @brief Possible line types
    enum class LineType : int
    {
        Scatter, ///< Scatter plot (only markers)
        Line,    ///< Line plot
    };

    /// Display name in the legend (if not set falls back to PlotData::displayName)
    std::string legendName;
    /// Legend name which gets changed in the gui
    std::string legendNameGui;
    /// Line type
    LineType lineType = LineType::Line;
    /// Line Color
    ImVec4 color = IMPLOT_AUTO_COL;
    /// Colormap mask (pair: type and id)
    std::pair<ColormapMaskType, int64_t> colormapMask = { ColormapMaskType::None, -1 };
    /// Index of the plot data to compare for the color
    size_t colormapMaskDataCmpIdx = 0;
    /// Line thickness
    float thickness = 1.0F;
    /// Line Flags (overrides the plot selection)
    std::optional<ImPlotLineFlags> lineFlags;

    /// Amount of points to skip for plotting
    int stride = 0;

    /// Colormap mask (pair: type and id)
    std::pair<ColormapMaskType, int64_t> markerColormapMask = { ColormapMaskType::None, -1 };
    /// Index of the plot data to compare for the color
    size_t markerColormapMaskDataCmpIdx = 0;
    /// Display markers for the line plot (no effect for scatter type)
    bool markers = false;
    /// Style of the marker to display
    ImPlotMarker markerStyle = ImPlotMarker_Cross;
    /// Size of the markers (makes the marker smaller/bigger)
    float markerSize = 1.0F;
    /// Weight of the markers (increases thickness of marker lines)
    float markerWeight = 1.0F;
    /// Fill color for markers
    ImVec4 markerFillColor = IMPLOT_AUTO_COL;
    /// Outline/Border color for markers
    ImVec4 markerOutlineColor = IMPLOT_AUTO_COL;

    /// Show events on this data
    bool eventsEnabled = false;
    /// Style of the marker to display
    ImPlotMarker eventMarkerStyle = ImPlotMarker_Cross;
    /// Size of the markers (makes the marker smaller/bigger)
    float eventMarkerSize = 6.0F;
    /// Weight of the markers (increases thickness of marker lines)
    float eventMarkerWeight = 2.0F;
    /// Fill color for markers
    ImVec4 eventMarkerFillColor = ImVec4(1.0, 0.0, 0.0, 1.0);
    /// Outline/Border color for markers
    ImVec4 eventMarkerOutlineColor = ImVec4(1.0, 0.0, 0.0, 1.0);
    /// Tooltip search regex
    std::string eventTooltipFilterRegex;

    /// @brief Shows a legend popup for plot items
    /// @param[in] id Unique id for the popup (should not change while open)
    /// @param[in] displayTitle Display title
    /// @param[in] plotDataBufferSize Buffer size of the data
    /// @param[in] plotElementIdx Plot element index
    /// @param[in] nameId Name and id of the calling node (logging)
    /// @param[in] plotLineFlags LineFlags from a parent plot
    /// @param[in, out] colormapMaskColors Buffer for the colormap mask of line plots
    /// @param[in, out] markerColormapMaskColors Buffer for the colormap mask of the markers
    /// @param[in] ShowColormapReferenceChooser Function to call to show a Combo to select the colormap reference
    /// @param[in, out] eventMarker Buffer for event markers
    /// @param[in, out] eventTooltips List of tooltips (x,y, tooltip)
    /// @return True if a change was made
    bool showLegendPopup(const char* id,
                         const char* displayTitle,
                         int plotDataBufferSize,
                         int plotElementIdx,
                         const char* nameId,
                         ImPlotLineFlags plotLineFlags = ImPlotLineFlags_NoClip | ImPlotLineFlags_SkipNaN,
                         ScrollingBuffer<ImU32>* colormapMaskColors = nullptr,
                         ScrollingBuffer<ImU32>* markerColormapMaskColors = nullptr,
                         const std::function<bool(size_t&, const char*)>& ShowColormapReferenceChooser = nullptr,
                         ScrollingBuffer<double>* eventMarker = nullptr,
                         std::vector<std::tuple<double, double, PlotEventTooltip>>* eventTooltips = nullptr);

    /// @brief Plots the data with the style
    /// @param[in] plotName Plot name
    /// @param[in] xData Data on the x axis
    /// @param[in] yData Data on the y axis
    /// @param[in] plotElementIdx Plot element index
    /// @param[in] defaultStride Default stride size
    /// @param[in] plotLineFlags LineFlags from a parent plot
    /// @param[in] colormapMaskColors Buffer for the colormap mask of line plots
    /// @param[in] markerColormapMaskColors Buffer for the colormap mask of the markers
    void plotData(const char* plotName,
                  const ScrollingBuffer<double>& xData,
                  const ScrollingBuffer<double>& yData,
                  int plotElementIdx,
                  int defaultStride = 1,
                  ImPlotLineFlags plotLineFlags = ImPlotLineFlags_NoClip | ImPlotLineFlags_SkipNaN,
                  ScrollingBuffer<ImU32>* colormapMaskColors = nullptr,
                  ScrollingBuffer<ImU32>* markerColormapMaskColors = nullptr) const;
};

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] style Object to read info from
void to_json(json& j, const PlotItemStyle& style);
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] style Output object
void from_json(const json& j, PlotItemStyle& style);

} // namespace NAV