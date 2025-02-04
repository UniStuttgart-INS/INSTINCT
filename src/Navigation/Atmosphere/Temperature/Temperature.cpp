/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Temperature.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "util/Logger.hpp"
#include "util/Assert.h"

#include "Models/StandardAtmosphere.hpp"

namespace NAV
{

bool ComboTemperatureModel(const char* label, TemperatureModel& temperatureModel)
{
    bool changed = false;
    changed |= gui::widgets::EnumCombo(label, temperatureModel._model);
    if (temperatureModel._model == TemperatureModel::Model::Const)
    {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(62.0F * gui::NodeEditorApplication::windowFontRatio());
        changed |= ImGui::InputDoubleL(fmt::format("##TemperatureModel constTemp {}", label).c_str(), &temperatureModel._constantTemperature,
                                       0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.1f K");
        ImGui::SameLine();
        ImGui::Text("%.1f°C", temperatureModel._constantTemperature - 273.15);
    }
    return changed;
}

double TemperatureModel::calcAbsoluteTemperature(double altitudeMSL) const
{
    switch (_model)
    {
    case TemperatureModel::Model::Const:
        return _constantTemperature;
    case TemperatureModel::Model::ISA:
        return calcAbsoluteTemperatureStAtm(altitudeMSL);
    case TemperatureModel::Model::GPT2:
    case TemperatureModel::Model::GPT3:
        LOG_CRITICAL("GPT2/GPT3 Model needs to be called separately because of parameter lookup.");
        break;
    case TemperatureModel::Model::None:
    case TemperatureModel::Model::COUNT:
        break;
    }

    return 0.0;
}

const char* to_string(const TemperatureModel& temperatureModel)
{
    return to_string(temperatureModel._model);
}

const char* to_string(TemperatureModel::Model temperatureModel)
{
    switch (temperatureModel)
    {
    case TemperatureModel::Model::None:
        return "None";
    case TemperatureModel::Model::Const:
        return "Const";
    case TemperatureModel::Model::ISA:
        return "ISA";
    case TemperatureModel::Model::GPT2:
        return "GPT2";
    case TemperatureModel::Model::GPT3:
        return "GPT3";
    case TemperatureModel::Model::COUNT:
        break;
    }
    return "";
}

void to_json(json& j, const TemperatureModel& obj)
{
    j = json{
        { "model", to_string(obj._model) },
        { "constantTemperature", obj._constantTemperature },
    };
}

void from_json(const json& j, TemperatureModel& obj)
{
    auto model = j.at("model").get<std::string>();
    if (model == "None") { obj._model = TemperatureModel::Model::None; }
    else if (model == "Const") { obj._model = TemperatureModel::Model::Const; }
    else if (model == "ISA") { obj._model = TemperatureModel::Model::ISA; }
    else if (model == "GPT2") { obj._model = TemperatureModel::Model::GPT2; }
    else if (model == "GPT3") { obj._model = TemperatureModel::Model::GPT3; }

    j.at("constantTemperature").get_to(obj._constantTemperature);
}

} // namespace NAV
