#include "Json.hpp"

#include <vector>

#include "implot_internal.h"

#include "util/Logger.hpp"

void to_json(json& j, const ImColor& color)
{
    j = json{
        { "r", static_cast<int>(color.Value.x * 255.0F) },
        { "g", static_cast<int>(color.Value.y * 255.0F) },
        { "b", static_cast<int>(color.Value.z * 255.0F) },
        { "a", static_cast<int>(color.Value.w * 255.0F) },
    };
}
void from_json(const json& j, ImColor& color)
{
    int r = 0;
    int g = 0;
    int b = 0;
    int a = 255;
    if (j.contains("r"))
    {
        j.at("r").get_to(r);
    }
    if (j.contains("g"))
    {
        j.at("g").get_to(g);
    }
    if (j.contains("b"))
    {
        j.at("b").get_to(b);
    }
    if (j.contains("a"))
    {
        j.at("a").get_to(a);
    }

    color = ImColor(r, g, b, a);
}

void to_json(json& j, const ImVec2& vec2)
{
    j = json{
        { "x", vec2.x },
        { "y", vec2.y },
    };
}
void from_json(const json& j, ImVec2& vec2)
{
    if (j.contains("x"))
    {
        j.at("x").get_to(vec2.x);
    }
    if (j.contains("y"))
    {
        j.at("y").get_to(vec2.y);
    }
}

void to_json(json& j, const ImVec4& vec4)
{
    j = json{
        { "x", vec4.x },
        { "y", vec4.y },
        { "z", vec4.z },
        { "w", vec4.w },
    };
}
void from_json(const json& j, ImVec4& vec4)
{
    if (j.contains("x"))
    {
        j.at("x").get_to(vec4.x);
    }
    if (j.contains("y"))
    {
        j.at("y").get_to(vec4.y);
    }
    if (j.contains("z"))
    {
        j.at("z").get_to(vec4.z);
    }
    if (j.contains("w"))
    {
        j.at("w").get_to(vec4.w);
    }
}

void to_json(json& j, const ImPlotStyle& style)
{
    j = json{
        { "LineWeight", style.LineWeight },
        { "MarkerSize", style.MarkerSize },
        { "MarkerWeight", style.MarkerWeight },
        { "FillAlpha", style.FillAlpha },
        { "ErrorBarSize", style.ErrorBarSize },
        { "ErrorBarWeight", style.ErrorBarWeight },
        { "DigitalBitHeight", style.DigitalBitHeight },
        { "DigitalBitGap", style.DigitalBitGap },
        { "AntiAliasedLines", style.AntiAliasedLines },
        { "PlotBorderSize", style.PlotBorderSize },
        { "MinorAlpha", style.MinorAlpha },
        { "MajorTickLen", style.MajorTickLen },
        { "MinorTickLen", style.MinorTickLen },
        { "MajorTickSize", style.MajorTickSize },
        { "MinorTickSize", style.MinorTickSize },
        { "MajorGridSize", style.MajorGridSize },
        { "MinorGridSize", style.MinorGridSize },
        { "PlotDefaultSize", style.PlotDefaultSize },
        { "PlotMinSize", style.PlotMinSize },
        { "PlotPadding", style.PlotPadding },
        { "LabelPadding", style.LabelPadding },
        { "LegendPadding", style.LegendPadding },
        { "LegendInnerPadding", style.LegendInnerPadding },
        { "LegendSpacing", style.LegendSpacing },
        { "MousePosPadding", style.MousePosPadding },
        { "AnnotationPadding", style.AnnotationPadding },
        { "FitPadding", style.FitPadding },
    };

    for (int i = 0; i < ImPlotCol_COUNT; i++)
    {
        if (ImPlot::IsColorAuto(i))
        {
            j["Colors"][ImPlot::GetStyleColorName(i)]["col"] = "Auto";
        }
        else
        {
            j["Colors"][ImPlot::GetStyleColorName(i)]["col"] = ImPlot::GetStyleColorVec4(i);
        }
    }

    j["Colormap"]["active"] = style.Colormap;

    for (int i = 16; i < ImPlot::GetCurrentContext()->ColormapData.Count; ++i)
    {
        j["Colormap"]["maps"][static_cast<size_t>(i - 16)]["name"] = ImPlot::GetColormapName(i);
        j["Colormap"]["maps"][static_cast<size_t>(i - 16)]["qualitative"] = ImPlot::GetCurrentContext()->ColormapData.IsQual(i);

        for (int c = 0; c < ImPlot::GetCurrentContext()->ColormapData.GetKeyCount(i); ++c)
        {
            j["Colormap"]["maps"][static_cast<size_t>(i - 16)]["keys"][static_cast<size_t>(c)] = ImGui::ColorConvertU32ToFloat4(ImPlot::GetCurrentContext()->ColormapData.GetKeyColor(i, c));
        }
    }
}
void from_json(const json& j, ImPlotStyle& style)
{
    if (j.contains("LineWeight"))
    {
        j.at("LineWeight").get_to(style.LineWeight);
    }
    if (j.contains("LineWeight"))
    {
        j.at("LineWeight").get_to(style.LineWeight);
    }
    if (j.contains("MarkerSize"))
    {
        j.at("MarkerSize").get_to(style.MarkerSize);
    }
    if (j.contains("MarkerWeight"))
    {
        j.at("MarkerWeight").get_to(style.MarkerWeight);
    }
    if (j.contains("FillAlpha"))
    {
        j.at("FillAlpha").get_to(style.FillAlpha);
    }
    if (j.contains("ErrorBarSize"))
    {
        j.at("ErrorBarSize").get_to(style.ErrorBarSize);
    }
    if (j.contains("ErrorBarWeight"))
    {
        j.at("ErrorBarWeight").get_to(style.ErrorBarWeight);
    }
    if (j.contains("DigitalBitHeight"))
    {
        j.at("DigitalBitHeight").get_to(style.DigitalBitHeight);
    }
    if (j.contains("DigitalBitGap"))
    {
        j.at("DigitalBitGap").get_to(style.DigitalBitGap);
    }
    if (j.contains("AntiAliasedLines"))
    {
        j.at("AntiAliasedLines").get_to(style.AntiAliasedLines);
    }
    if (j.contains("PlotBorderSize"))
    {
        j.at("PlotBorderSize").get_to(style.PlotBorderSize);
    }
    if (j.contains("MinorAlpha"))
    {
        j.at("MinorAlpha").get_to(style.MinorAlpha);
    }
    if (j.contains("MajorTickLen"))
    {
        j.at("MajorTickLen").get_to(style.MajorTickLen);
    }
    if (j.contains("MinorTickLen"))
    {
        j.at("MinorTickLen").get_to(style.MinorTickLen);
    }
    if (j.contains("MajorTickSize"))
    {
        j.at("MajorTickSize").get_to(style.MajorTickSize);
    }
    if (j.contains("MinorTickSize"))
    {
        j.at("MinorTickSize").get_to(style.MinorTickSize);
    }
    if (j.contains("MajorGridSize"))
    {
        j.at("MajorGridSize").get_to(style.MajorGridSize);
    }
    if (j.contains("MinorGridSize"))
    {
        j.at("MinorGridSize").get_to(style.MinorGridSize);
    }
    if (j.contains("PlotDefaultSize"))
    {
        j.at("PlotDefaultSize").get_to(style.PlotDefaultSize);
    }
    if (j.contains("PlotMinSize"))
    {
        j.at("PlotMinSize").get_to(style.PlotMinSize);
    }
    if (j.contains("PlotPadding"))
    {
        j.at("PlotPadding").get_to(style.PlotPadding);
    }
    if (j.contains("LabelPadding"))
    {
        j.at("LabelPadding").get_to(style.LabelPadding);
    }
    if (j.contains("LegendPadding"))
    {
        j.at("LegendPadding").get_to(style.LegendPadding);
    }
    if (j.contains("LegendInnerPadding"))
    {
        j.at("LegendInnerPadding").get_to(style.LegendInnerPadding);
    }
    if (j.contains("LegendSpacing"))
    {
        j.at("LegendSpacing").get_to(style.LegendSpacing);
    }
    if (j.contains("MousePosPadding"))
    {
        j.at("MousePosPadding").get_to(style.MousePosPadding);
    }
    if (j.contains("AnnotationPadding"))
    {
        j.at("AnnotationPadding").get_to(style.AnnotationPadding);
    }
    if (j.contains("FitPadding"))
    {
        j.at("FitPadding").get_to(style.FitPadding);
    }

    if (j.contains("Colors"))
    {
        for (int i = 0; i < ImPlotCol_COUNT; i++)
        {
            if (j.at("Colors").contains(ImPlot::GetStyleColorName(i)))
            {
                if (j.at("Colors").at(ImPlot::GetStyleColorName(i)).contains("col"))
                {
                    if (j.at("Colors").at(ImPlot::GetStyleColorName(i)).at("col").is_string()
                        && j.at("Colors").at(ImPlot::GetStyleColorName(i)).at("col").get<std::string>() == "Auto")
                    {
                        style.Colors[i] = IMPLOT_AUTO_COL; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    }
                    else
                    {
                        j.at("Colors").at(ImPlot::GetStyleColorName(i)).at("col").get_to(style.Colors[i]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    }
                }
            }
            else
            {
                LOG_WARN("Problem reading the ImPlot style: The color '{}' was not found in the json file.", ImPlot::GetStyleColorName(i));
            }
        }
    }

    if (j.contains("Colormap"))
    {
        if (j.at("Colormap").contains("maps"))
        {
            for (size_t i = 0; i < j.at("Colormap").at("maps").size(); ++i)
            {
                std::vector<ImVec4> custom;
                for (size_t c = 0; c < j.at("Colormap").at("maps").at(i).at("keys").size(); ++c)
                {
                    custom.push_back(j.at("Colormap").at("maps").at(i).at("keys").at(c).get<ImVec4>());
                }
                auto name = j.at("Colormap").at("maps").at(i).at("name").get<std::string>();
                auto qual = j.at("Colormap").at("maps").at(i).at("qualitative").get<bool>();

                if (ImPlot::GetCurrentContext()->ColormapData.GetIndex(name.c_str()) == -1 && custom.size() > 1)
                {
                    ImPlot::AddColormap(name.c_str(), custom.data(), static_cast<int>(custom.size()), qual);
                }
            }
        }
        if (j.at("Colormap").contains("active"))
        {
            j.at("Colormap").at("active").get_to(style.Colormap);
        }
    }
}