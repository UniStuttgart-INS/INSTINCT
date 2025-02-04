// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeasurementErrors.hpp"

#include <limits>
#include <implot.h>
#include <fmt/format.h>

#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "Navigation/GNSS/Functions.hpp"

#include "util/Logger.hpp"

namespace NAV
{

GnssMeasurementErrorModel::GnssMeasurementErrorModel()
{
    for (size_t i = 0; i < Model::COUNT; i++)
    {
        updateStdDevCurvePlot(static_cast<Model>(i));
    }
}

double GnssMeasurementErrorModel::psrMeasErrorVar(const SatelliteSystem& satSys, double elevation, double cn0) const
{
    return satSysErrorFactorVariance(satSys) * std::pow(_codeStdDev * weightingFunction(_model, elevation, cn0), 2.0);
}

double GnssMeasurementErrorModel::carrierMeasErrorVar(const SatelliteSystem& satSys, double elevation, double cn0) const
{
    return satSysErrorFactorVariance(satSys) * std::pow(_carrierStdDev * weightingFunction(_model, elevation, cn0), 2.0);
}

double GnssMeasurementErrorModel::psrRateMeasErrorVar(const Frequency& freq, int8_t num, double elevation, double cn0) const
{
    double psrRateMeasStdDev = doppler2rangeRate(_dopplerStdDev, freq, num);
    return satSysErrorFactorVariance(freq.getSatSys()) * std::pow(psrRateMeasStdDev * weightingFunction(_model, elevation, cn0), 2.0);
}

double GnssMeasurementErrorModel::codeBiasErrorVar() const // NOLINT(readability-convert-member-functions-to-static)
{
    // Rtklib model, as no other available

    constexpr double ERR_CBIAS = 0.3; // Code bias error Std [m]
    return std::pow(ERR_CBIAS, 2);
}

double GnssMeasurementErrorModel::satSysErrorFactorVariance(const SatelliteSystem& satSys)
{
    double satSysErrFactor = 1.0;
    switch (static_cast<SatelliteSystem_>(satSys))
    {
    case GPS:
    case GAL:
    case BDS:
    case QZSS:
        satSysErrFactor = 1.0;
        break;
    case GLO:
    case IRNSS:
        satSysErrFactor = 1.5;
        break;
    case SBAS:
        satSysErrFactor = 3.0;
        break;
    case SatSys_None:
        break;
    }
    return std::pow(satSysErrFactor, 2);
}

double GnssMeasurementErrorModel::weightingFunction(Model model, double elevation, double cn0) const
{
    if (std::isnan(elevation)) { return 1.0; }
    elevation = std::max(elevation, deg2rad(0.1));
    switch (model)
    {
    case Model::SINE:
        return _modelParametersSine.a / std::sin(elevation);
    case Model::SINE_OFFSET:
        return _modelParametersSineOffset.a + _modelParametersSineOffset.b / std::sin(elevation);
    case Model::SINE_CN0:
        cn0 = std::pow(10, cn0 / 10); // See Groves, ch 9.1.4.3, eq. 9.22, p. 363
        return _modelParametersSineCN0.a / std::sin(elevation) * (_modelParametersSineCN0.b + _modelParametersSineCN0.c / std::sqrt(cn0));
    case Model::RTKLIB:
        elevation = std::max(elevation, deg2rad(5.0));
        return std::sqrt(std::pow(_modelParametersRtklib.a, 2) + std::pow(_modelParametersRtklib.b / std::sin(elevation), 2));
    case Model::SINE_SQRT:
        return std::sqrt(std::pow(_modelParametersSineSqrt.a, 2) + std::pow(_modelParametersSineSqrt.b, 2) / std::pow(std::sin(elevation), 2));
    case Model::EXPONENTIAL:
        return _modelParametersExponential.a + _modelParametersExponential.b * std::exp(-elevation / deg2rad(_modelParametersExponential.e0));
    case Model::COSINE_TYPE:
        return std::sqrt(_modelParametersCosineType.a + _modelParametersCosineType.b * std::pow(std::cos(elevation), _modelParametersCosineType.n));
    case Model::None:
    case Model::COUNT:
        break;
    }
    return 1.0;
}

void GnssMeasurementErrorModel::updateStdDevCurvePlot(Model model)
{
    for (size_t i = 0; i < _elevation.size(); ++i)
    {
        _stdDevCurvePlot.at(static_cast<size_t>(model)).at(i) = weightingFunction(model, _elevation.at(i), _plotCN0);
    }
}

bool GnssMeasurementErrorModel::ShowGuiWidgets(const char* id, float width)
{
    const float UNIT_WIDTH = 100.0F * gui::NodeEditorApplication::windowFontRatio();
    const float BUTTON_WIDTH = 25.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemWidth(width - BUTTON_WIDTH - 2 * ImGui::GetStyle().ItemInnerSpacing.x);
    bool changed = gui::widgets::EnumCombo(fmt::format("##GNSS Measurement Error Model EnumCombo {}", id).c_str(), _model);
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("...## GnssMeasurementError {}", id).c_str(), ImVec2(BUTTON_WIDTH, 0)))
    {
        ImGui::OpenPopup(fmt::format("{} GnssMeasurementError Popup", id).c_str());
    }
    ImGui::SameLine(0.0F, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextUnformatted("Weighting Function");

    int combo_current_item = 0;
    changed |= gui::widgets::InputDoubleWithUnit(fmt::format("Carrier-Phase StdDev σ₀##{}", id).c_str(), width, UNIT_WIDTH,
                                                 &_carrierStdDev, combo_current_item, "m\0\0", 0.0, 0.0, "%.3g", ImGuiInputTextFlags_CharsScientific);
    changed |= gui::widgets::InputDoubleWithUnit(fmt::format("Code/Pseudorange StdDev σ₀##{}", id).c_str(), width, UNIT_WIDTH,
                                                 &_codeStdDev, combo_current_item, "m\0\0", 0.0, 0.0, "%.3g", ImGuiInputTextFlags_CharsScientific);

    changed |= gui::widgets::InputDoubleWithUnit(fmt::format("Doppler StdDev σ₀##{}", id).c_str(), width, UNIT_WIDTH,
                                                 &_dopplerStdDev, combo_current_item, "Hz\0\0", 0.0, 0.0, "%.3g", ImGuiInputTextFlags_CharsScientific);
    ImGui::SameLine();
    ImGui::Text("= %.2g m/s (G1)", std::abs(doppler2rangeRate(_dopplerStdDev, G01, -128)));

    if (ImGui::BeginPopup(fmt::format("{} GnssMeasurementError Popup", id).c_str()))
    {
        const float PLOT_WIDTH = 500.0F * gui::NodeEditorApplication::windowFontRatio();
        const float PLOT_HEIGHT = 450.0F * gui::NodeEditorApplication::windowFontRatio();
        const float TABLE_WIDTH = 440.0F * gui::NodeEditorApplication::windowFontRatio();
        const float ITEM_WIDTH = 160.0F * gui::NodeEditorApplication::windowFontRatio();

        const float WINDOW_HEIGHT = PLOT_HEIGHT + 30.0F * gui::NodeEditorApplication::windowFontRatio();

        if (ImGui::BeginChild("left pane", ImVec2(PLOT_WIDTH, WINDOW_HEIGHT), false, ImGuiWindowFlags_NoScrollbar))
        {
            ImPlot::SetNextAxesLimits(_elevation_deg[0], _elevation_deg[PLOT_SAMPLES - 1], 0, 5, ImPlotCond_Once);
            if (ImPlot::BeginPlot("Weighting function", ImVec2(ImGui::GetContentRegionAvail().x, PLOT_HEIGHT)))
            {
                ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, _elevation_deg[0], _elevation_deg[PLOT_SAMPLES - 1]);
                ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, 0.0, std::numeric_limits<double>::max());
                ImPlot::SetupAxes("Elevation [deg]", "Weighting function [-]");
                ImPlot::SetupLegend(ImPlotLocation_NorthEast);
                for (size_t i = 0; i < Model::COUNT; i++)
                {
                    ImPlot::PlotLine(to_string(static_cast<Model>(i)), _elevation_deg.data(), _stdDevCurvePlot.at(i).data(), static_cast<int>(_elevation_deg.size()));
                }
                ImPlot::EndPlot();
            }
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 40.0F * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::SliderDouble(fmt::format("c/n₀##{}", id).c_str(), &_plotCN0, 0.0, 60.0, "%.2f dB-Hz"))
            {
                updateStdDevCurvePlot(Model::SINE_CN0);
                changed = true;
            }
        }
        ImGui::EndChild();

        ImGui::SameLine();

        if (ImGui::BeginTable("parameter settings", 2, ImGuiTableFlags_SizingStretchSame, ImVec2(TABLE_WIDTH, 0.0F)))
        {
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::SINE), id).c_str()))
            {
                ImGui::TextUnformatted("wf = a/sin(e)");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::SINE), id).c_str(), &_modelParametersSine.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE);
                    changed = true;
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::SINE_OFFSET), id).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextUnformatted("wf = (a + b/sin(e))");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::SINE_OFFSET), id).c_str(), &_modelParametersSineOffset.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_OFFSET);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("b##{} - {}", fmt::underlying(Model::SINE_OFFSET), id).c_str(), &_modelParametersSineOffset.b, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_OFFSET);
                    changed = true;
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::SINE_CN0), id).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextUnformatted("wf = a/sin(e) * √(b + c/(c/n₀))");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::SINE_CN0), id).c_str(), &_modelParametersSineCN0.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_CN0);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("b##{} - {}", fmt::underlying(Model::SINE_CN0), id).c_str(), &_modelParametersSineCN0.b, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_CN0);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("c##{} - {}", fmt::underlying(Model::SINE_CN0), id).c_str(), &_modelParametersSineCN0.c, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_CN0);
                    changed = true;
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::RTKLIB), id).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextUnformatted("wf = √(a² + b²/sin(e))");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::RTKLIB), id).c_str(), &_modelParametersRtklib.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::RTKLIB);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("b##{} - {}", fmt::underlying(Model::RTKLIB), id).c_str(), &_modelParametersRtklib.b, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::RTKLIB);
                    changed = true;
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::SINE_SQRT), id).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextUnformatted("wf = √(a² + b²/sin²(e))");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::SINE_SQRT), id).c_str(), &_modelParametersSineSqrt.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_SQRT);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("b##{} - {}", fmt::underlying(Model::SINE_SQRT), id).c_str(), &_modelParametersSineSqrt.b, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::SINE_SQRT);
                    changed = true;
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::EXPONENTIAL), id).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextUnformatted("wf = (a + b * exp(-e/e₀))");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::EXPONENTIAL), id).c_str(), &_modelParametersExponential.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::EXPONENTIAL);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("b##{} - {}", fmt::underlying(Model::EXPONENTIAL), id).c_str(), &_modelParametersExponential.b, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::EXPONENTIAL);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("e0##{} - {}", fmt::underlying(Model::EXPONENTIAL), id).c_str(), &_modelParametersExponential.e0, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.1f"))
                {
                    updateStdDevCurvePlot(Model::EXPONENTIAL);
                    changed = true;
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::CollapsingHeader(fmt::format("{}##{}", to_string(Model::COSINE_TYPE), id).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextUnformatted("wf = √(a + b * cosⁿ(e))");
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("a##{} - {}", fmt::underlying(Model::COSINE_TYPE), id).c_str(), &_modelParametersCosineType.a, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::COSINE_TYPE);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::DragDouble(fmt::format("b##{} - {}", fmt::underlying(Model::COSINE_TYPE), id).c_str(), &_modelParametersCosineType.b, 0.1F, 0.0, std::numeric_limits<double>::max(), "%.2f"))
                {
                    updateStdDevCurvePlot(Model::COSINE_TYPE);
                    changed = true;
                }
                ImGui::SetNextItemWidth(ITEM_WIDTH);
                if (ImGui::InputIntL(fmt::format("n##{} - {}", fmt::underlying(Model::COSINE_TYPE), id).c_str(), &_modelParametersCosineType.n, 0))
                {
                    updateStdDevCurvePlot(Model::COSINE_TYPE);
                    changed = true;
                }
            }
            ImGui::EndTable();
        }
        ImGui::EndPopup();
    }
    return changed;
}

const char* to_string(GnssMeasurementErrorModel::Model model)
{
    switch (model)
    {
    case GnssMeasurementErrorModel::Model::None:
        return "None";
    case GnssMeasurementErrorModel::Model::SINE:
        return "Sine";
    case GnssMeasurementErrorModel::Model::SINE_OFFSET:
        return "Sine + Offset";
    case GnssMeasurementErrorModel::Model::SINE_CN0:
        return "Sine + CN0";
    case GnssMeasurementErrorModel::Model::RTKLIB:
        return "RTKLIB";
    case GnssMeasurementErrorModel::Model::SINE_SQRT:
        return "Sine-Sqrt";
    case GnssMeasurementErrorModel::Model::EXPONENTIAL:
        return "Exponential";
    case GnssMeasurementErrorModel::Model::COSINE_TYPE:
        return "Cosine-Type";
    case GnssMeasurementErrorModel::Model::COUNT:
        break;
    }
    return "";
}

void to_json(json& j, const GnssMeasurementErrorModel& obj)
{
    j = json{
        { "model", obj._model },
        { "carrierStdDev", obj._carrierStdDev },
        { "codeStdDev", obj._codeStdDev },
        { "dopplerStdDev", obj._dopplerStdDev },
        { "plotCN0", obj._plotCN0 },
        { "modelParametersSine", obj._modelParametersSine },
        { "modelParametersSineOffset", obj._modelParametersSineOffset },
        { "modelParametersSineCN0", obj._modelParametersSineCN0 },
        { "modelParametersRtklib", obj._modelParametersRtklib },
        { "modelParametersSineSqrt", obj._modelParametersSineSqrt },
        { "modelParametersExponential", obj._modelParametersExponential },
        { "modelParametersCosineType", obj._modelParametersCosineType },
    };
}

void from_json(const json& j, GnssMeasurementErrorModel& obj)
{
    if (j.contains("model")) { j.at("model").get_to(obj._model); }
    if (j.contains("carrierStdDev")) { j.at("carrierStdDev").get_to(obj._carrierStdDev); }
    if (j.contains("codeStdDev")) { j.at("codeStdDev").get_to(obj._codeStdDev); }
    if (j.contains("dopplerStdDev")) { j.at("dopplerStdDev").get_to(obj._dopplerStdDev); }
    if (j.contains("plotCN0")) { j.at("plotCN0").get_to(obj._plotCN0); }
    if (j.contains("modelParametersSine")) { j.at("modelParametersSine").get_to(obj._modelParametersSine); }
    if (j.contains("modelParametersSineOffset")) { j.at("modelParametersSineOffset").get_to(obj._modelParametersSineOffset); }
    if (j.contains("modelParametersSineCN0")) { j.at("modelParametersSineCN0").get_to(obj._modelParametersSineCN0); }
    if (j.contains("modelParametersRtklib")) { j.at("modelParametersRtklib").get_to(obj._modelParametersRtklib); }
    if (j.contains("modelParametersSineSqrt")) { j.at("modelParametersSineSqrt").get_to(obj._modelParametersSineSqrt); }
    if (j.contains("modelParametersExponential")) { j.at("modelParametersExponential").get_to(obj._modelParametersExponential); }
    if (j.contains("modelParametersCosineType")) { j.at("modelParametersCosineType").get_to(obj._modelParametersCosineType); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSine& obj)
{
    j = json{
        { "a", obj.a },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSine& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineOffset& obj)
{
    j = json{
        { "a", obj.a },
        { "b", obj.b },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineOffset& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
    if (j.contains("b")) { j.at("b").get_to(obj.b); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineCN0& obj)
{
    j = json{
        { "a", obj.a },
        { "b", obj.b },
        { "c", obj.c },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineCN0& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
    if (j.contains("b")) { j.at("b").get_to(obj.b); }
    if (j.contains("c")) { j.at("c").get_to(obj.c); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersRtklib& obj)
{
    j = json{
        { "a", obj.a },
        { "b", obj.b },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersRtklib& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
    if (j.contains("b")) { j.at("b").get_to(obj.b); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineSqrt& obj)
{
    j = json{
        { "a", obj.a },
        { "b", obj.b },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineSqrt& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
    if (j.contains("b")) { j.at("b").get_to(obj.b); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersExponential& obj)
{
    j = json{
        { "a", obj.a },
        { "b", obj.b },
        { "e0", obj.e0 },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersExponential& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
    if (j.contains("b")) { j.at("b").get_to(obj.b); }
    if (j.contains("e0")) { j.at("e0").get_to(obj.e0); }
}

void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersCosineType& obj)
{
    j = json{
        { "a", obj.a },
        { "b", obj.b },
        { "n", obj.n },
    };
}
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersCosineType& obj)
{
    if (j.contains("a")) { j.at("a").get_to(obj.a); }
    if (j.contains("b")) { j.at("b").get_to(obj.b); }
    if (j.contains("n")) { j.at("n").get_to(obj.n); }
}

} // namespace NAV