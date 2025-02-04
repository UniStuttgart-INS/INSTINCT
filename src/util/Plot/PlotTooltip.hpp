// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PlotTooltip.hpp
/// @brief Plot Tooltips on hover
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-16

#pragma once

#include <cstddef>
#include <optional>
#include <functional>

#include <imgui.h>
#include <implot.h>

#include "Navigation/Time/InsTime.hpp"
#include "util/Container/ScrollingBuffer.hpp"

namespace NAV
{

/// Plot Tooltip windows to show
struct PlotTooltip
{
    /// @brief Constructor
    /// @param[in] plotItemIdx The plot item index the tooltip belongs to
    /// @param[in] dataIdx The data index the tooltip belongs to
    /// @param[in] startPos Initial start position of the tooltip window
    PlotTooltip(size_t plotItemIdx, size_t dataIdx, const ImVec2& startPos)
        : plotItemIdx(plotItemIdx), dataIdx(dataIdx), startPos(startPos) {}

    size_t plotItemIdx = 0;         ///< The plot item index the tooltip belongs to
    size_t dataIdx = 0;             ///< The data index the tooltip belongs to
    std::optional<ImVec2> startPos; ///< Initial start position of the tooltip window
};

/// @brief Shows a tooltip if the plot is hovered
/// @param[in, out] tooltips Tooltip vector to show
/// @param[in] plotItemIdx Plot item index
/// @param[in] plotName Name of the plot item
/// @param[in] axis Axis to check for
/// @param[in] xData X axis data
/// @param[in] yData Y axis data
/// @param[in] otherHoverTooltipsShown Whether other hover tooltips are already shown
/// @param[in] showTooltipCallback Called inside the tooltip. Argument is the data index for which the tooltip is shown
/// @return True if a tooltip is shown
bool ShowPlotTooltip(std::vector<PlotTooltip>& tooltips,
                     size_t plotItemIdx,
                     const std::string& plotName,
                     ImAxis axis,
                     const ScrollingBuffer<double>& xData,
                     const ScrollingBuffer<double>& yData,
                     bool otherHoverTooltipsShown,
                     const std::function<void(size_t)>& showTooltipCallback);

/// @brief Shows all tooltip windows in the vector
/// @param[in, out] tooltips Tooltip vector to show
/// @param[in] plotItemIdx Plot item index
/// @param[in] plotName Name of the plot item
/// @param[in] uid Unique id for the window
/// @param[in] parentWindows Parent windows to stay on top of
/// @param[in] getInsTime Callback to get the time associated with the tooltip
/// @param[in] showTooltipCallback Called inside the tooltip. Argument is the data index and the unique id for which the tooltip is shown
void ShowPlotTooltipWindows(std::vector<PlotTooltip>& tooltips,
                            size_t plotItemIdx,
                            const std::string& plotName,
                            const std::string& uid,
                            const std::vector<int*>& parentWindows,
                            const std::function<InsTime(size_t)>& getInsTime,
                            const std::function<void(size_t, const char*)>& showTooltipCallback);

} // namespace NAV