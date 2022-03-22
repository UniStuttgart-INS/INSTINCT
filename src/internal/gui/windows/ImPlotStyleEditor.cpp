#include "ImPlotStyleEditor.hpp"

#include <vector>
#include <string>

#include <fmt/core.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <implot.h>
#include <implot_internal.h>

#include "internal/gui/widgets/HelpMarker.hpp"

#include "util/Logger.hpp"

void NAV::gui::windows::ShowImPlotStyleEditor(bool* show /* = nullptr*/)
{
    if (!ImGui::Begin("ImPlot Style", show))
    {
        ImGui::End();
        return;
    }

    // TODO: Save to file (global/flow)
    // if (ImGui::Checkbox("Save to global settings"))
    // {
    // }

    ImPlotStyle& style = ImPlot::GetStyle();
    static ImPlotStyle ref = style;

    if (ImGui::BeginTabBar("##StyleEditor"))
    {
        if (ImGui::BeginTabItem("Variables"))
        {
            if (ImGui::BeginTable("ImPlotStyleEditorColors", 2, ImGuiTableFlags_SizingFixedFit, ImVec2(0.0F, 0.0F)))
            {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 55.0F);

                auto revertButton = [](auto& value, auto refVal, const char* id) {
                    if (memcmp(&value, &refVal, sizeof(refVal)) != 0)
                    {
                        ImGui::TableNextColumn();
                        if (ImGui::Button(fmt::format("Revert##ImPlotStyle.{}", id).c_str()))
                        {
                            value = refVal;
                        }
                    }
                };

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Item Styling");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("LineWeight", &style.LineWeight, 0.0F, 5.0F, "%.1F");
                revertButton(style.LineWeight, ref.LineWeight, "LineWeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("MarkerSize", &style.MarkerSize, 2.0F, 10.0F, "%.1F");
                revertButton(style.MarkerSize, ref.MarkerSize, "MarkerSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("MarkerWeight", &style.MarkerWeight, 0.0F, 5.0F, "%.1F");
                revertButton(style.MarkerWeight, ref.MarkerWeight, "MarkerWeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("FillAlpha", &style.FillAlpha, 0.0F, 1.0F, "%.2F");
                revertButton(style.FillAlpha, ref.FillAlpha, "FillAlpha");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("ErrorBarSize", &style.ErrorBarSize, 0.0F, 10.0F, "%.1F");
                revertButton(style.ErrorBarSize, ref.ErrorBarSize, "ErrorBarSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("ErrorBarWeight", &style.ErrorBarWeight, 0.0F, 5.0F, "%.1F");
                revertButton(style.ErrorBarWeight, ref.ErrorBarWeight, "ErrorBarWeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("DigitalBitHeight", &style.DigitalBitHeight, 0.0F, 20.0F, "%.1F");
                revertButton(style.DigitalBitHeight, ref.DigitalBitHeight, "DigitalBitHeight");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("DigitalBitGap", &style.DigitalBitGap, 0.0F, 20.0F, "%.1F");
                revertButton(style.DigitalBitGap, ref.DigitalBitGap, "DigitalBitGap");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                float indent = ImGui::CalcItemWidth() - ImGui::GetFrameHeight();
                ImGui::Indent(ImGui::CalcItemWidth() - ImGui::GetFrameHeight());
                ImGui::Checkbox("AntiAliasedLines", &style.AntiAliasedLines);
                ImGui::Unindent(indent);
                revertButton(style.AntiAliasedLines, ref.AntiAliasedLines, "AntiAliasedLines");

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Plot Styling");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("PlotBorderSize", &style.PlotBorderSize, 0.0F, 2.0F, "%.0F");
                revertButton(style.PlotBorderSize, ref.PlotBorderSize, "PlotBorderSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat("MinorAlpha", &style.MinorAlpha, 0.0F, 1.0F, "%.2F");
                revertButton(style.MinorAlpha, ref.MinorAlpha, "MinorAlpha");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MajorTickLen", reinterpret_cast<float*>(&style.MajorTickLen), 0.0F, 20.0F, "%.0F");
                revertButton(style.MajorTickLen, ref.MajorTickLen, "MajorTickLen");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MinorTickLen", reinterpret_cast<float*>(&style.MinorTickLen), 0.0F, 20.0F, "%.0F");
                revertButton(style.MinorTickLen, ref.MinorTickLen, "MinorTickLen");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MajorTickSize", reinterpret_cast<float*>(&style.MajorTickSize), 0.0F, 2.0F, "%.1F");
                revertButton(style.MajorTickSize, ref.MajorTickSize, "MajorTickSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MinorTickSize", reinterpret_cast<float*>(&style.MinorTickSize), 0.0F, 2.0F, "%.1F");
                revertButton(style.MinorTickSize, ref.MinorTickSize, "MinorTickSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MajorGridSize", reinterpret_cast<float*>(&style.MajorGridSize), 0.0F, 2.0F, "%.1F");
                revertButton(style.MajorGridSize, ref.MajorGridSize, "MajorGridSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MinorGridSize", reinterpret_cast<float*>(&style.MinorGridSize), 0.0F, 2.0F, "%.1F");
                revertButton(style.MinorGridSize, ref.MinorGridSize, "MinorGridSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("PlotDefaultSize", reinterpret_cast<float*>(&style.PlotDefaultSize), 0.0F, 1000, "%.0F");
                revertButton(style.PlotDefaultSize, ref.PlotDefaultSize, "PlotDefaultSize");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("PlotMinSize", reinterpret_cast<float*>(&style.PlotMinSize), 0.0F, 300, "%.0F");
                revertButton(style.PlotMinSize, ref.PlotMinSize, "PlotMinSize");

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Plot Padding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("PlotPadding", reinterpret_cast<float*>(&style.PlotPadding), 0.0F, 20.0F, "%.0F");
                revertButton(style.PlotPadding, ref.PlotPadding, "PlotPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("LabelPadding", reinterpret_cast<float*>(&style.LabelPadding), 0.0F, 20.0F, "%.0F");
                revertButton(style.LabelPadding, ref.LabelPadding, "LabelPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("LegendPadding", reinterpret_cast<float*>(&style.LegendPadding), 0.0F, 20.0F, "%.0F");
                revertButton(style.LegendPadding, ref.LegendPadding, "LegendPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("LegendInnerPadding", reinterpret_cast<float*>(&style.LegendInnerPadding), 0.0F, 10.0F, "%.0F");
                revertButton(style.LegendInnerPadding, ref.LegendInnerPadding, "LegendInnerPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("LegendSpacing", reinterpret_cast<float*>(&style.LegendSpacing), 0.0F, 5.0F, "%.0F");
                revertButton(style.LegendSpacing, ref.LegendSpacing, "LegendSpacing");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("MousePosPadding", reinterpret_cast<float*>(&style.MousePosPadding), 0.0F, 20.0F, "%.0F");
                revertButton(style.MousePosPadding, ref.MousePosPadding, "MousePosPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("AnnotationPadding", reinterpret_cast<float*>(&style.AnnotationPadding), 0.0F, 5.0F, "%.0F");
                revertButton(style.AnnotationPadding, ref.AnnotationPadding, "AnnotationPadding");
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::SliderFloat2("FitPadding", reinterpret_cast<float*>(&style.FitPadding), 0, 0.2F, "%.2F");
                revertButton(style.FitPadding, ref.FitPadding, "FitPadding");

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
                    }
                    if (memcmp(&style.Colors[i], &ref.Colors[i], sizeof(ImVec4)) != 0) // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    {
                        ImGui::TableNextColumn();
                        if (ImGui::Button(fmt::format("Revert##ImPlotStyleColor{}", i).c_str()))
                        {
                            style.Colors[i] = ref.Colors[i]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                            ImPlot::BustItemCache();
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
            static std::string name = "MyColormap";

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