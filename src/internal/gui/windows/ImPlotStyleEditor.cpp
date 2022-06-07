#include "ImPlotStyleEditor.hpp"

#include <vector>
#include <string>
#include <filesystem>
#include <fstream>

#include <fmt/core.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <implot.h>
#include <implot_internal.h>
#include <imgui_extra_math.h>

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/FileDialog.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"

#include "util/Logger.hpp"
#include "util/Json.hpp"

namespace NAV::gui::windows
{

bool saveConfigInFlow = false;
bool prefereFlowOverGlobal = true;

} // namespace NAV::gui::windows

void NAV::gui::windows::ShowImPlotStyleEditor(bool* show /* = nullptr*/)
{
    if (!ImGui::Begin("ImPlot Style", show))
    {
        ImGui::End();
        return;
    }

    auto loadImPlotStyleFromConfigFile = [](const std::string& path) {
        std::filesystem::path filepath = flow::GetProgramRootPath();
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
                j.at("implot").at("style").get_to(ImPlot::GetStyle());
                LOG_INFO("Loaded ImPlot style from file {}", path);
            }
        }
    };

    std::filesystem::path filepath = flow::GetProgramRootPath();
    if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("implot-config") };
        inputPath.is_relative())
    {
        filepath /= inputPath;
    }
    else
    {
        filepath = inputPath;
    }

    static std::string path = ConfigManager::Get<std::string>("implot-config");

    if (widgets::FileDialogLoad(path, "ImPlot config file", ".json", { ".json" }, filepath.parent_path() / ".", 0, "ImPlotStyleEditor"))
    {
        if (path.starts_with(flow::GetProgramRootPath().string()))
        {
            path = path.substr(flow::GetProgramRootPath().string().length());
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Load##ImPlotStyleFromFile"))
    {
        loadImPlotStyleFromConfigFile(path);
    }
    ImGui::SameLine();
    if (ImGui::Button("Save##ImPlotStyleToFile"))
    {
        filepath = flow::GetProgramRootPath();
        if (std::filesystem::path inputPath{ path };
            inputPath.is_relative())
        {
            filepath /= inputPath;
        }
        else
        {
            filepath = inputPath;
        }
        std::ofstream filestream(filepath);

        if (!filestream.good())
        {
            LOG_ERROR("Could not save the ImPlot config file: {}", filepath.string());
        }
        else
        {
            json j;
            j["implot"]["style"] = ImPlot::GetStyle();

            filestream << std::setw(4) << j << std::endl;
        }
    }

    if (ImGui::Checkbox("Save into flow file", &saveConfigInFlow))
    {
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Prefere flow file over global settings", &prefereFlowOverGlobal))
    {
        loadImPlotStyleFromConfigFile(prefereFlowOverGlobal ? flow::GetCurrentFilename() : path);
        flow::ApplyChanges();
    }

    ImPlotStyle& style = ImPlot::GetStyle();

    if (ImGui::BeginTabBar("##StyleEditor"))
    {
        if (ImGui::BeginTabItem("Variables"))
        {
            if (ImGui::BeginTable("ImPlotStyleEditorColors", 2, ImGuiTableFlags_SizingFixedFit, ImVec2(0.0F, 0.0F)))
            {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 55.0F);

                auto revertButton = [](auto& value, auto refVal, const char* id) {
                    if (value != refVal)
                    {
                        ImGui::TableNextColumn();
                        if (ImGui::Button(fmt::format("Revert##ImPlotStyle.{}", id).c_str()))
                        {
                            value = refVal;
                            if (saveConfigInFlow)
                            {
                                flow::ApplyChanges();
                            }
                        }
                    }
                };

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Item Styling");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("LineWeight", &style.LineWeight, 0.0F, 5.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.LineWeight, NodeEditorApplication::imPlotReferenceStyle.LineWeight, "LineWeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("MarkerSize", &style.MarkerSize, 2.0F, 10.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MarkerSize, NodeEditorApplication::imPlotReferenceStyle.MarkerSize, "MarkerSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("MarkerWeight", &style.MarkerWeight, 0.0F, 5.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MarkerWeight, NodeEditorApplication::imPlotReferenceStyle.MarkerWeight, "MarkerWeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("FillAlpha", &style.FillAlpha, 0.0F, 1.0F, "%.2F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.FillAlpha, NodeEditorApplication::imPlotReferenceStyle.FillAlpha, "FillAlpha");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("ErrorBarSize", &style.ErrorBarSize, 0.0F, 10.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.ErrorBarSize, NodeEditorApplication::imPlotReferenceStyle.ErrorBarSize, "ErrorBarSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("ErrorBarWeight", &style.ErrorBarWeight, 0.0F, 5.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.ErrorBarWeight, NodeEditorApplication::imPlotReferenceStyle.ErrorBarWeight, "ErrorBarWeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("DigitalBitHeight", &style.DigitalBitHeight, 0.0F, 20.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.DigitalBitHeight, NodeEditorApplication::imPlotReferenceStyle.DigitalBitHeight, "DigitalBitHeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("DigitalBitGap", &style.DigitalBitGap, 0.0F, 20.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.DigitalBitGap, NodeEditorApplication::imPlotReferenceStyle.DigitalBitGap, "DigitalBitGap");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                float indent = ImGui::CalcItemWidth() - ImGui::GetFrameHeight();
                ImGui::Indent(ImGui::CalcItemWidth() - ImGui::GetFrameHeight());
                if (ImGui::Checkbox("AntiAliasedLines", &style.AntiAliasedLines))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                ImGui::Unindent(indent);
                revertButton(style.AntiAliasedLines, NodeEditorApplication::imPlotReferenceStyle.AntiAliasedLines, "AntiAliasedLines");

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Plot Styling");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("PlotBorderSize", &style.PlotBorderSize, 0.0F, 2.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.PlotBorderSize, NodeEditorApplication::imPlotReferenceStyle.PlotBorderSize, "PlotBorderSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat("MinorAlpha", &style.MinorAlpha, 0.0F, 1.0F, "%.2F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MinorAlpha, NodeEditorApplication::imPlotReferenceStyle.MinorAlpha, "MinorAlpha");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MajorTickLen", reinterpret_cast<float*>(&style.MajorTickLen), 0.0F, 20.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MajorTickLen, NodeEditorApplication::imPlotReferenceStyle.MajorTickLen, "MajorTickLen");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MinorTickLen", reinterpret_cast<float*>(&style.MinorTickLen), 0.0F, 20.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MinorTickLen, NodeEditorApplication::imPlotReferenceStyle.MinorTickLen, "MinorTickLen");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MajorTickSize", reinterpret_cast<float*>(&style.MajorTickSize), 0.0F, 2.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MajorTickSize, NodeEditorApplication::imPlotReferenceStyle.MajorTickSize, "MajorTickSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MinorTickSize", reinterpret_cast<float*>(&style.MinorTickSize), 0.0F, 2.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MinorTickSize, NodeEditorApplication::imPlotReferenceStyle.MinorTickSize, "MinorTickSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MajorGridSize", reinterpret_cast<float*>(&style.MajorGridSize), 0.0F, 2.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MajorGridSize, NodeEditorApplication::imPlotReferenceStyle.MajorGridSize, "MajorGridSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MinorGridSize", reinterpret_cast<float*>(&style.MinorGridSize), 0.0F, 2.0F, "%.1F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MinorGridSize, NodeEditorApplication::imPlotReferenceStyle.MinorGridSize, "MinorGridSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("PlotDefaultSize", reinterpret_cast<float*>(&style.PlotDefaultSize), 0.0F, 1000, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.PlotDefaultSize, NodeEditorApplication::imPlotReferenceStyle.PlotDefaultSize, "PlotDefaultSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("PlotMinSize", reinterpret_cast<float*>(&style.PlotMinSize), 0.0F, 300, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.PlotMinSize, NodeEditorApplication::imPlotReferenceStyle.PlotMinSize, "PlotMinSize");

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Plot Padding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("PlotPadding", reinterpret_cast<float*>(&style.PlotPadding), 0.0F, 20.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.PlotPadding, NodeEditorApplication::imPlotReferenceStyle.PlotPadding, "PlotPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("LabelPadding", reinterpret_cast<float*>(&style.LabelPadding), 0.0F, 20.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.LabelPadding, NodeEditorApplication::imPlotReferenceStyle.LabelPadding, "LabelPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("LegendPadding", reinterpret_cast<float*>(&style.LegendPadding), 0.0F, 20.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.LegendPadding, NodeEditorApplication::imPlotReferenceStyle.LegendPadding, "LegendPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("LegendInnerPadding", reinterpret_cast<float*>(&style.LegendInnerPadding), 0.0F, 10.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.LegendInnerPadding, NodeEditorApplication::imPlotReferenceStyle.LegendInnerPadding, "LegendInnerPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("LegendSpacing", reinterpret_cast<float*>(&style.LegendSpacing), 0.0F, 5.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.LegendSpacing, NodeEditorApplication::imPlotReferenceStyle.LegendSpacing, "LegendSpacing");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("MousePosPadding", reinterpret_cast<float*>(&style.MousePosPadding), 0.0F, 20.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.MousePosPadding, NodeEditorApplication::imPlotReferenceStyle.MousePosPadding, "MousePosPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("AnnotationPadding", reinterpret_cast<float*>(&style.AnnotationPadding), 0.0F, 5.0F, "%.0F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.AnnotationPadding, NodeEditorApplication::imPlotReferenceStyle.AnnotationPadding, "AnnotationPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::SliderFloat2("FitPadding", reinterpret_cast<float*>(&style.FitPadding), 0, 0.2F, "%.2F"))
                {
                    if (saveConfigInFlow)
                    {
                        flow::ApplyChanges();
                    }
                }
                revertButton(style.FitPadding, NodeEditorApplication::imPlotReferenceStyle.FitPadding, "FitPadding");

                ImGui::EndTable();
            }

            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Colors"))
        {
            static ImGuiTextFilter filter;
            filter.Draw("Filter colors", ImGui::GetFontSize() * 16);

            static ImGuiColorEditFlags alpha_flags = ImGuiColorEditFlags_AlphaPreviewHalf;
            if (ImGui::RadioButton("Opaque", alpha_flags == ImGuiColorEditFlags_None))
            {
                alpha_flags = ImGuiColorEditFlags_None;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Alpha", alpha_flags == ImGuiColorEditFlags_AlphaPreview))
            {
                alpha_flags = ImGuiColorEditFlags_AlphaPreview;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Both", alpha_flags == ImGuiColorEditFlags_AlphaPreviewHalf))
            {
                alpha_flags = ImGuiColorEditFlags_AlphaPreviewHalf;
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("In the color list:\n"
                                     "Left-click on colored square to open color picker,\n"
                                     "Right-click to open edit options menu.");
            ImGui::Separator();

            if (ImGui::BeginTable("ImPlotStyleEditorColors", 3, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
            {
                for (int i = 0; i < ImPlotCol_COUNT; i++)
                {
                    const char* name = ImPlot::GetStyleColorName(i);
                    if (!filter.PassFilter(name))
                    {
                        continue;
                    }

                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::PushID(i);
                    ImVec4 temp = ImPlot::GetStyleColorVec4(i);
                    const bool isColorAuto = ImPlot::IsColorAuto(i);
                    if (!isColorAuto)
                    {
                        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.25F);
                    }
                    if (ImGui::Button("Auto"))
                    {
                        if (isColorAuto)
                        {
                            style.Colors[i] = temp; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                        }
                        else
                        {
                            style.Colors[i] = IMPLOT_AUTO_COL; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                        }
                        ImPlot::BustItemCache();
                        if (saveConfigInFlow)
                        {
                            flow::ApplyChanges();
                        }
                    }
                    if (!isColorAuto)
                    {
                        ImGui::PopStyleVar();
                    }
                    ImGui::TableNextColumn();
                    if (ImGui::ColorEdit4(name, &temp.x, ImGuiColorEditFlags_NoInputs | alpha_flags))
                    {
                        style.Colors[i] = temp; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                        ImPlot::BustItemCache();
                        if (saveConfigInFlow)
                        {
                            flow::ApplyChanges();
                        }
                    }
                    if (style.Colors[i] != NodeEditorApplication::imPlotReferenceStyle.Colors[i]) // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    {
                        ImGui::TableNextColumn();
                        if (ImGui::Button(fmt::format("Revert##ImPlotStyleColor{}", i).c_str()))
                        {
                            style.Colors[i] = NodeEditorApplication::imPlotReferenceStyle.Colors[i]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                            ImPlot::BustItemCache();
                            if (saveConfigInFlow)
                            {
                                flow::ApplyChanges();
                            }
                        }
                    }
                    ImGui::PopID();
                }

                ImGui::EndTable();
            }

            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Colormaps"))
        {
            ImPlotContext& gp = *ImPlot::GetCurrentContext();

            static std::vector<bool> showEditColormap(static_cast<size_t>(gp.ColormapData.Count));

            if (ImGui::BeginTable("ImPlotStyleEditorColormaps", 2, ImGuiTableFlags_SizingFixedFit))
            {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);

                // built-in/added
                for (int i = 0; i < gp.ColormapData.Count; ++i)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::PushID(i);
                    int size = gp.ColormapData.GetKeyCount(i);
                    bool selected = i == gp.Style.Colormap;

                    const char* name = ImPlot::GetColormapName(i);
                    if (!selected)
                    {
                        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.25F);
                    }
                    if (ImGui::Button(name, ImVec2(100, 0)))
                    {
                        gp.Style.Colormap = i;
                        ImPlot::BustItemCache();
                    }
                    if (!selected)
                    {
                        ImGui::PopStyleVar();
                    }
                    ImGui::TableNextColumn();
                    if (showEditColormap.at(static_cast<size_t>(i)))
                    {
                        for (int c = 0; c < size; ++c)
                        {
                            ImGui::PushID(c);
                            ImVec4 col4 = ImGui::ColorConvertU32ToFloat4(gp.ColormapData.GetKeyColor(i, c));
                            if (ImGui::ColorEdit4("", &col4.x, ImGuiColorEditFlags_NoInputs))
                            {
                                ImU32 col32 = ImGui::ColorConvertFloat4ToU32(col4);
                                gp.ColormapData.SetKeyColor(i, c, col32);
                                ImPlot::BustItemCache();
                            }
                            if ((c + 1) % 12 != 0 && c != size - 1)
                            {
                                ImGui::SameLine();
                            }
                            ImGui::PopID();
                        }
                        ImGui::SameLine();
                        ImGui::Dummy(ImVec2(10.0F, -1.0F));
                        ImGui::SameLine();
                        if (ImGui::Button("x##ImPlotStyleEditorColormap close edit"))
                        {
                            showEditColormap.at(static_cast<size_t>(i)) = false;
                        }
                    }
                    else
                    {
                        if (ImPlot::ColormapButton("##", ImVec2(-1, 0), i))
                        {
                            showEditColormap.at(static_cast<size_t>(i)) = true;
                        }
                    }

                    ImGui::PopID();
                }

                ImGui::EndTable();
            }

            static ImVector<ImVec4> custom;
            if (custom.Size == 0)
            {
                custom.push_back(ImVec4(1, 0, 0, 1));
                custom.push_back(ImVec4(0, 1, 0, 1));
                custom.push_back(ImVec4(0, 0, 1, 1));
            }
            ImGui::Separator();

            ImGui::BeginGroup();
            static std::string name = "Custom";

            if (ImGui::Button("+", ImVec2((100 - ImGui::GetStyle().ItemSpacing.x) / 2, 0)))
            {
                custom.push_back(ImVec4(0, 0, 0, 1));
            }
            ImGui::SameLine();
            if (ImGui::Button("-", ImVec2((100 - ImGui::GetStyle().ItemSpacing.x) / 2, 0)) && custom.Size > 2)
            {
                custom.pop_back();
            }
            ImGui::SetNextItemWidth(100);
            ImGui::InputText("##Name", &name, ImGuiInputTextFlags_CharsNoBlank);
            static bool qual = true;
            ImGui::Checkbox("Qualitative", &qual);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Means, that the colors are separated into distinct levels.\n"
                                  "If unchecked, a color gradient will be applied.");
            }

            if (ImGui::Button("Add", ImVec2(100, 0)) && gp.ColormapData.GetIndex(name.c_str()) == -1)
            {
                ImPlot::AddColormap(name.c_str(), custom.Data, custom.Size, qual);
                showEditColormap.push_back(false);
            }

            ImGui::EndGroup();
            ImGui::SameLine();
            ImGui::BeginGroup();
            for (int c = 0; c < custom.Size; ++c)
            {
                ImGui::PushID(c);
                if (ImGui::ColorEdit4("##Col1", &custom[c].x, ImGuiColorEditFlags_NoInputs))
                {
                }
                if ((c + 1) % 12 != 0)
                {
                    ImGui::SameLine();
                }
                ImGui::PopID();
            }
            ImGui::EndGroup();

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}