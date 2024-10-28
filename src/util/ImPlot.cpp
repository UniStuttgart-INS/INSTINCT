/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImPlot.hpp"

#include <filesystem>

#include "internal/FlowManager.hpp"
#include "util/Json.hpp"
#include "util/Logger.hpp"

#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W
    #ifdef __APPLE__
        #define GL_SILENCE_DEPRECATION
        #include <OpenGL/gl.h>
    #else
        #include <GL/gl.h>
    #endif
    #define STB_IMAGE_WRITE_IMPLEMENTATION
    #include "stb_image_write.h"
#endif

void NAV::loadImPlotStyleFromConfigFile(const char* path, ImPlotStyle& imPlotStyle)
{
    std::filesystem::path filepath = flow::GetConfigPath();
    if (std::filesystem::path inputPath{ path };
        inputPath.is_relative())
    {
        filepath /= inputPath;
    }
    else
    {
        filepath = inputPath;
    }
    std::ifstream filestream(filepath);

    if (!filestream.good())
    {
        LOG_ERROR("The ImPlot style config file could not be loaded: {}", filepath.string());
    }
    else
    {
        json j;
        filestream >> j;

        if (j.contains("implot") && j.at("implot").contains("style"))
        {
            j.at("implot").at("style").get_to(imPlotStyle);
            LOG_DEBUG("Loaded ImPlot style from file {}", path);
        }
    }
}

#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W

NAV::ImGuiScreenshotImageBuf::ImGuiScreenshotImageBuf(int x, int y, size_t w, size_t h)
    : Width(w), Height(h), Data(Width * Height * 4, 0)
{
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(x, y, static_cast<int>(w), static_cast<int>(h), GL_RGBA, GL_UNSIGNED_BYTE, Data.data());
    RemoveAlpha();
    FlipVertical();
}

void NAV::ImGuiScreenshotImageBuf::SaveFile(const char* filename)
{
    stbi_write_png(filename, static_cast<int>(Width),
                   static_cast<int>(Height),
                   4,
                   Data.data(),
                   static_cast<int>(Width * 4));
}

void NAV::ImGuiScreenshotImageBuf::RemoveAlpha()
{
    uint32_t* p = Data.data();
    auto n = static_cast<int>(Width * Height);
    while (n-- > 0)
    {
        *p |= 0xFF000000;
        p++;
    }
}

void NAV::ImGuiScreenshotImageBuf::FlipVertical()
{
    size_t comp = 4;
    size_t stride = Width * comp;
    std::vector<unsigned char> line_tmp(stride);
    auto* line_a = reinterpret_cast<unsigned char*>(Data.data());
    auto* line_b = reinterpret_cast<unsigned char*>(Data.data()) + (stride * (Height - 1));
    while (line_a < line_b)
    {
        memcpy(line_tmp.data(), line_a, stride);
        memcpy(line_a, line_b, stride);
        memcpy(line_b, line_tmp.data(), stride);
        line_a += stride;
        line_b -= stride;
    }
}

#endif