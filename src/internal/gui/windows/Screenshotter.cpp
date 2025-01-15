// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Screenshotter.hpp"

#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W
    #include <cstdlib>
    #include <string>

    #include <imgui.h>
    #include <imgui_internal.h>
    #include <imgui_node_editor.h>
    #include <imgui_node_editor_internal.h>
namespace ed = ax::NodeEditor;

    #include <fmt/core.h>

    #include "internal/FlowManager.hpp"
    #include "internal/gui/NodeEditorApplication.hpp"
    #include "internal/gui/widgets/FileDialog.hpp"

    #include "internal/gui/windows/NodeEditorStyleEditor.hpp"
    #include "util/ImPlot.hpp"
    #include "util/Logger.hpp"

namespace NAV::gui::windows
{

std::string plotScreenshotImPlotStyleFile = "implot-light.json";
bool copyScreenshotsToClipboard = true;

namespace
{
ImRect _screenshotNavigateRect;
ImRect _screenshotCaptureRect;
bool _showScreenshotCaptureRect = false;
bool _printScreenshotSaveLocation = true;
size_t _screenshotFrameCnt = 0;

} // namespace
} // namespace NAV::gui::windows

void NAV::gui::windows::ShowScreenshotter(bool* show /* = nullptr*/)
{
    if (_screenshotFrameCnt > 0)
    {
        _screenshotFrameCnt++;

        if (_screenshotFrameCnt == 4)
        {
            ImGuiIO& io = ImGui::GetIO();
            ImGuiScreenshotImageBuf Output(static_cast<int>(_screenshotCaptureRect.Min.x),
                                           static_cast<int>(io.DisplaySize.y) - static_cast<int>(_screenshotCaptureRect.Max.y),
                                           static_cast<size_t>(_screenshotCaptureRect.GetWidth()),
                                           static_cast<size_t>(_screenshotCaptureRect.GetHeight()));

            std::time_t t = std::time(nullptr);
            std::tm* now = std::localtime(&t); // NOLINT(concurrency-mt-unsafe)

            auto savePath = flow::GetOutputPath()
                            / fmt::format("Screenshot_{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}.png",
                                          now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
            Output.SaveFile(savePath.c_str());
            if (_printScreenshotSaveLocation)
            {
                LOG_INFO("Screenshot saved as: {}", savePath);
            }

            if (copyScreenshotsToClipboard)
            {
                CopyFileToClipboard(savePath.c_str());
            }
            _screenshotFrameCnt = 0;
        }

        return;
    }

    if (_showScreenshotCaptureRect)
    {
        constexpr float thickness = 1.0F;
        auto rect = _screenshotCaptureRect;
        rect.Expand(thickness);
        ImGui::GetForegroundDrawList()->AddRect(rect.Min, rect.Max, ImColor(255, 0, 0),
                                                0.0F, ImDrawFlags_None, thickness);
    }

    if (!ImGui::Begin("Screenshotter", show, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::End();
        return;
    }

    const float ITEM_WIDTH = 400.0F * NodeEditorApplication::defaultFontRatio();
    const float ITEM_WIDTH_HALF = (ITEM_WIDTH - ImGui::GetStyle().ItemInnerSpacing.x) / 2.0F;

    ImGui::TextUnformatted("Plot screenshot style: ");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(200.0F * NodeEditorApplication::defaultFontRatio());
    if (widgets::FileDialogLoad(plotScreenshotImPlotStyleFile, "Plot screenshot config file", ".json", { ".json" },
                                flow::GetConfigPath(), 1, "ImPlotStyleEditorScreenshot"))
    {
        LOG_DEBUG("Plot screenshot config file changed to: {}", plotScreenshotImPlotStyleFile);
    }

    ImGui::Checkbox("Copy screenshots to clipboard", &copyScreenshotsToClipboard);

    if (ImGui::Checkbox("Light mode", &nodeEditorLightMode))
    {
        ApplyDarkLightMode(NodeEditorApplication::m_colors);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    bool showGridLines = ed::GetStyle().Colors[ed::StyleColor_Grid].w != 0.0F;
    if (ImGui::Checkbox("Grid lines", &showGridLines))
    {
        ed::GetStyle().Colors[ed::StyleColor_Grid].w = showGridLines ? ed::Style().Colors[ed::StyleColor_Grid].w : 0.0F;
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    bool transparentWindows = ImGui::GetStyle().Colors[ImGuiCol_WindowBg].w != 1.0F;
    if (ImGui::Checkbox("Transparent windows", &transparentWindows))
    {
        ImGui::GetStyle().Colors[ImGuiCol_WindowBg].w = transparentWindows ? ImGuiStyle().Colors[ImGuiCol_WindowBg].w : 1.0F;
        flow::ApplyChanges();
    }

    if (ImGui::Checkbox("Hide left pane", &NodeEditorApplication::hideLeftPane))
    {
        flow::ApplyChanges();
    }
    if (!NodeEditorApplication::hideLeftPane)
    {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ITEM_WIDTH_HALF);
        ImGui::DragFloat("Left pane width", &NodeEditorApplication::leftPaneWidth);
    }
    if (ImGui::Checkbox("Hide FPS count", &NodeEditorApplication::hideFPS))
    {
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Print save location", &_printScreenshotSaveLocation);

    ImGui::Separator();

    auto* editor = reinterpret_cast<ed::Detail::EditorContext*>(ed::GetCurrentEditor());
    if (_screenshotNavigateRect.GetArea() == 0.0F) { _screenshotNavigateRect = editor->GetContentBounds(); }

    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::TreeNode("Screenshot navigate area"))
    {
        ImGui::TextUnformatted(fmt::format("Current Content Bounds: Min({},{}), Max({},{})",
                                           editor->GetContentBounds().Min.x, editor->GetContentBounds().Min.y,
                                           editor->GetContentBounds().Max.x, editor->GetContentBounds().Max.y)
                                   .c_str());
        ImGui::TextUnformatted(fmt::format("Current View: Min({},{}), Max({},{})",
                                           editor->GetViewRect().Min.x, editor->GetViewRect().Min.y,
                                           editor->GetViewRect().Max.x, editor->GetViewRect().Max.y)
                                   .c_str());
        if (ImGui::Button("Set area to content")) { _screenshotNavigateRect = editor->GetContentBounds(); }
        ImGui::SameLine();
        if (ImGui::Button("Set area to current view"))
        {
            _screenshotNavigateRect = editor->GetViewRect();

            auto extend = ImMax(_screenshotNavigateRect.GetWidth(), _screenshotNavigateRect.GetHeight());
            constexpr float c_NavigationZoomMargin = 0.1F;
            _screenshotNavigateRect.Expand(-extend * c_NavigationZoomMargin * 0.5F);
        }

        ImGui::SetNextItemWidth(ITEM_WIDTH);
        ImGui::DragFloat2("Min##Nav area", &_screenshotNavigateRect.Min.x);
        ImGui::SetNextItemWidth(ITEM_WIDTH);
        ImGui::DragFloat2("Max##Nav area", &_screenshotNavigateRect.Max.x);

        if (ImGui::Button("Navigate to area"))
        {
            editor->NavigateTo(_screenshotNavigateRect, true);
        }

        ImGui::TreePop();
    }

    ImGuiIO& io = ImGui::GetIO();
    if (_screenshotCaptureRect.GetArea() == 0.0F && ImGui::GetFrameCount() > 10) { _screenshotCaptureRect.Max = io.DisplaySize; }

    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::TreeNode("Screenshot capture area"))
    {
        if (ImGui::Button("Set to display size"))
        {
            _screenshotCaptureRect.Min = ImVec2();
            _screenshotCaptureRect.Max = io.DisplaySize;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ITEM_WIDTH_HALF);
        if (ImGui::BeginCombo("##Window list", "Set to window"))
        {
            ImVector<ImGuiWindow*>& windows = ImGui::GetCurrentContext()->Windows;
            for (ImGuiWindow* window : windows)
            {
                std::string windowName = window->Name;
                if (!window->WasActive || windowName.find('#') != std::string::npos || windowName.starts_with("Content")) { continue; }
                const bool is_selected = window->Rect().Min == _screenshotCaptureRect.Min && window->Rect().Max == _screenshotCaptureRect.Max;
                if (ImGui::Selectable(window->Name, is_selected))
                {
                    _screenshotCaptureRect = window->Rect();
                }

                // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                if (is_selected) { ImGui::SetItemDefaultFocus(); }
            }

            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Show Capture rect", &_showScreenshotCaptureRect);

        ImGui::SetNextItemWidth(ITEM_WIDTH_HALF);
        ImGui::DragFloat("##Min Capture area x", &_screenshotCaptureRect.Min.x, 1.0F, 0.0F, io.DisplaySize.x - 1);
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::SetNextItemWidth(ITEM_WIDTH_HALF);
        ImGui::DragFloat("Min##Capture area y", &_screenshotCaptureRect.Min.y, 1.0F, 0.0F, io.DisplaySize.y - 1);

        ImGui::SetNextItemWidth(ITEM_WIDTH_HALF);
        ImGui::DragFloat("##Max Capture area x", &_screenshotCaptureRect.Max.x, 1.0F, _screenshotCaptureRect.Min.x + 1, io.DisplaySize.x);
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::SetNextItemWidth(ITEM_WIDTH_HALF);
        ImGui::DragFloat("Max##Capture area y", &_screenshotCaptureRect.Max.y, 1.0F, _screenshotCaptureRect.Min.y + 1, io.DisplaySize.y);

        if (ImGui::Button("Take screenshot"))
        {
            _screenshotFrameCnt = 1; // Starts taking a screenshot after some frames
        }

        ImGui::TreePop();
    }

    ImGui::End();
}

void NAV::gui::windows::CopyFileToClipboard(const char* path)
{
    // NOLINTNEXTLINE
    [[maybe_unused]] int exitCode = system(fmt::format("command -v xclip > /dev/null 2>&1 && xclip -selection clipboard -target image/png -i {} && exit 0"
                                                       "|| command -v wl-copy > /dev/null 2>&1 && wl-copy < {}",
                                                       path, path)
                                               .c_str());
}

#endif