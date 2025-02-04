// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImPlot.hpp
/// @brief ImPlot utilities
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-27

#pragma once

#include "implot.h"
#include <vector>
#include <cstdint>

namespace NAV
{

/// @brief Loads the ImPlotStyle from a json file
/// @param[in] path Path of the file
/// @param[out] imPlotStyle Object to fill
void loadImPlotStyleFromConfigFile(const char* path, ImPlotStyle& imPlotStyle);

#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W

/// Helper class for simple bitmap manipulation (not particularly efficient!)
struct ImGuiScreenshotImageBuf
{
    /// @brief Creates an image from the current OpenGL buffer at the rectangle
    /// @param[in] x X coordinate
    /// @param[in] y Y coordinate
    /// @param[in] w Width
    /// @param[in] h Height
    ImGuiScreenshotImageBuf(int x, int y, size_t w, size_t h);

    /// @brief Save as fiel
    /// @param[in] filename Filename to save as
    void SaveFile(const char* filename);

  private:
    /// Removes the alpha channel
    void RemoveAlpha();

    /// Flip the image
    void FlipVertical();

    size_t Width = 0;           ///< Width
    size_t Height = 0;          ///< Height
    std::vector<uint32_t> Data; ///< Data buffer
};

#endif

} // namespace NAV