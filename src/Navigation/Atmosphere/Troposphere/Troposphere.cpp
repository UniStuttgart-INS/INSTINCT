// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Troposphere.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"
#include "util/Logger.hpp"

#include "Models/Saastamoinen.hpp"
#include "Models/GPT.hpp"

#include "MappingFunctions/Cosecant.hpp"
#include <boost/algorithm/string.hpp>

namespace NAV
{

const char* to_string(TroposphereModel troposphereZhdModel)
{
    switch (troposphereZhdModel)
    {
    case TroposphereModel::None:
        return "None";
    case TroposphereModel::Saastamoinen:
        return "Saastamoinen";
    case TroposphereModel::GPT2:
        return "GPT2";
    case TroposphereModel::GPT3:
        return "GPT3";
    case TroposphereModel::COUNT:
        break;
    }
    return "";
}

const char* to_string(MappingFunction mappingFunction)
{
    switch (mappingFunction)
    {
    case MappingFunction::None:
        return "None";
    case MappingFunction::Cosecant:
        return "Cosecant(elevation)";
    case MappingFunction::VMF_GPT2:
        return "VMF(GPT2)";
    case MappingFunction::VMF_GPT3:
        return "VMF(GPT3)";
    case MappingFunction::COUNT:
        break;
    }
    return "";
}

AtmosphereModels MappingFunctionDefaults(MappingFunction mappingFunction)
{
    switch (mappingFunction)
    {
    case MappingFunction::Cosecant:
        return { .pressureModel = PressureModel::ISA,
                 .temperatureModel = TemperatureModel::ISA,
                 .waterVaporModel = WaterVaporModel::ISA };
    case MappingFunction::VMF_GPT2:
        return { .pressureModel = PressureModel::GPT2,
                 .temperatureModel = TemperatureModel::GPT2,
                 .waterVaporModel = WaterVaporModel::GPT2 };
    case MappingFunction::VMF_GPT3:
        return { .pressureModel = PressureModel::GPT3,
                 .temperatureModel = TemperatureModel::GPT3,
                 .waterVaporModel = WaterVaporModel::GPT3 };
    case MappingFunction::None:
    case MappingFunction::COUNT:
        break;
    }
    return { .pressureModel = PressureModel::None,
             .temperatureModel = TemperatureModel::None,
             .waterVaporModel = WaterVaporModel::None };
}

/// @brief Returns the default atmosphere model, mapping function and mapping function atmosphere model for the given troposphere model
/// @param troposphereModel Troposphere model to give the defaults for
/// @return Tuple of <atmosphere model, mapping function, mapping function atmosphere model>
std::tuple<AtmosphereModels, MappingFunction, AtmosphereModels> ModelDefaults(TroposphereModel troposphereModel)
{
    switch (troposphereModel)
    {
    case TroposphereModel::Saastamoinen:
        return { AtmosphereModels{ .pressureModel = PressureModel::ISA,
                                   .temperatureModel = TemperatureModel::ISA,
                                   .waterVaporModel = WaterVaporModel::ISA },
                 MappingFunction::Cosecant,
                 MappingFunctionDefaults(MappingFunction::Cosecant) };
    case TroposphereModel::GPT2:
        return { AtmosphereModels{ .pressureModel = PressureModel::GPT2,
                                   .temperatureModel = TemperatureModel::GPT2,
                                   .waterVaporModel = WaterVaporModel::GPT2 },
                 MappingFunction::VMF_GPT2,
                 MappingFunctionDefaults(MappingFunction::VMF_GPT2) };
    case TroposphereModel::GPT3:
        return { AtmosphereModels{ .pressureModel = PressureModel::GPT3,
                                   .temperatureModel = TemperatureModel::GPT3,
                                   .waterVaporModel = WaterVaporModel::GPT3 },
                 MappingFunction::VMF_GPT3,
                 MappingFunctionDefaults(MappingFunction::VMF_GPT3) };
    case TroposphereModel::None:
    case TroposphereModel::COUNT:
        break;
    }

    return { AtmosphereModels{ .pressureModel = PressureModel::None,
                               .temperatureModel = TemperatureModel::None,
                               .waterVaporModel = WaterVaporModel::None },
             MappingFunction::None,
             MappingFunctionDefaults(MappingFunction::None) };
}

bool ComboTroposphereModel(const char* label, TroposphereModelSelection& troposphereModelSelection, float width)
{
    bool changed = false;

    constexpr float BUTTON_WIDTH = 25.0F;

    ImGui::SetNextItemWidth(width - BUTTON_WIDTH - 2 * ImGui::GetStyle().ItemInnerSpacing.x);
    if (gui::widgets::EnumCombo(fmt::format("##{}", label).c_str(),
                                troposphereModelSelection.zhdModel.first,
                                troposphereModelSelection.zwdModel.first))
    {
        std::tie(troposphereModelSelection.zhdModel.second,
                 troposphereModelSelection.zhdMappingFunction.first,
                 troposphereModelSelection.zhdMappingFunction.second) = ModelDefaults(troposphereModelSelection.zhdModel.first);
        std::tie(troposphereModelSelection.zwdModel.second,
                 troposphereModelSelection.zwdMappingFunction.first,
                 troposphereModelSelection.zwdMappingFunction.second) = ModelDefaults(troposphereModelSelection.zwdModel.first);
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("...##{}", label).c_str(), ImVec2(BUTTON_WIDTH, 0)))
    {
        ImGui::OpenPopup(fmt::format("{} Popup", label).c_str());
    }
    if (ImGui::BeginPopup(fmt::format("{} Popup", label).c_str()))
    {
        constexpr float ATMOSPHERE_COMBO_WIDTH = 95.0F;
        if (ImGui::BeginTable(fmt::format("{} Table", label).c_str(), 5))
        {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Pressure", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Temperature", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Water vapor", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            ImGui::TableNextColumn();
            ImGui::TextUnformatted("ZHD model");
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(width);
            if (gui::widgets::EnumCombo(fmt::format("##{} ZHD - Combo", label).c_str(), troposphereModelSelection.zhdModel.first))
            {
                std::tie(troposphereModelSelection.zhdModel.second,
                         troposphereModelSelection.zhdMappingFunction.first,
                         troposphereModelSelection.zhdMappingFunction.second) = ModelDefaults(troposphereModelSelection.zhdModel.first);
                changed = true;
            }
            if (troposphereModelSelection.zhdModel.first == TroposphereModel::None) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboPressureModel(fmt::format("##{} ZHD - Pressure", label).c_str(), troposphereModelSelection.zhdModel.second.pressureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboTemperatureModel(fmt::format("##{} ZHD - Temperature", label).c_str(), troposphereModelSelection.zhdModel.second.temperatureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboWaterVaporModel(fmt::format("##{} ZHD - Water vapor", label).c_str(), troposphereModelSelection.zhdModel.second.waterVaporModel);
            if (troposphereModelSelection.zhdModel.first == TroposphereModel::None) { ImGui::EndDisabled(); }

            ImGui::TableNextColumn();
            ImGui::TextUnformatted("ZWD model");
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(width);
            if (gui::widgets::EnumCombo(fmt::format("##{} ZWD - Combo", label).c_str(), troposphereModelSelection.zwdModel.first))
            {
                std::tie(troposphereModelSelection.zwdModel.second,
                         troposphereModelSelection.zwdMappingFunction.first,
                         troposphereModelSelection.zwdMappingFunction.second) = ModelDefaults(troposphereModelSelection.zwdModel.first);
                changed = true;
            }
            if (troposphereModelSelection.zwdModel.first == TroposphereModel::None) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboPressureModel(fmt::format("##{} ZWD - Pressure", label).c_str(), troposphereModelSelection.zwdModel.second.pressureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboTemperatureModel(fmt::format("##{} ZWD - Temperature", label).c_str(), troposphereModelSelection.zwdModel.second.temperatureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboWaterVaporModel(fmt::format("##{} ZWD - Water vapor", label).c_str(), troposphereModelSelection.zwdModel.second.waterVaporModel);
            if (troposphereModelSelection.zwdModel.first == TroposphereModel::None) { ImGui::EndDisabled(); }

            if (troposphereModelSelection.zhdModel.first == TroposphereModel::None) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Mapping function ZHD");
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(width);
            if (gui::widgets::EnumCombo(fmt::format("##{} MapZHD - Combo", label).c_str(), troposphereModelSelection.zhdMappingFunction.first))
            {
                troposphereModelSelection.zhdMappingFunction.second = MappingFunctionDefaults(troposphereModelSelection.zhdMappingFunction.first);
                changed = true;
            }
            if (troposphereModelSelection.zhdModel.first != TroposphereModel::None
                && troposphereModelSelection.zhdMappingFunction.first == MappingFunction::None) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboPressureModel(fmt::format("##{} MapZHD - Pressure", label).c_str(), troposphereModelSelection.zhdMappingFunction.second.pressureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboTemperatureModel(fmt::format("##{} MapZHD - Temperature", label).c_str(), troposphereModelSelection.zhdMappingFunction.second.temperatureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboWaterVaporModel(fmt::format("##{} MapZHD - Water vapor", label).c_str(), troposphereModelSelection.zhdMappingFunction.second.waterVaporModel);
            if (troposphereModelSelection.zhdModel.first == TroposphereModel::None
                || troposphereModelSelection.zhdMappingFunction.first == MappingFunction::None) { ImGui::EndDisabled(); }

            if (troposphereModelSelection.zwdModel.first == TroposphereModel::None) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Mapping function ZWD");
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(width);
            if (gui::widgets::EnumCombo(fmt::format("##{} MapZWD - Combo", label).c_str(), troposphereModelSelection.zwdMappingFunction.first))
            {
                troposphereModelSelection.zwdMappingFunction.second = MappingFunctionDefaults(troposphereModelSelection.zwdMappingFunction.first);
                changed = true;
            }
            if (troposphereModelSelection.zwdModel.first != TroposphereModel::None
                && troposphereModelSelection.zwdMappingFunction.first == MappingFunction::None) { ImGui::BeginDisabled(); }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboPressureModel(fmt::format("##{} MapZWD - Pressure", label).c_str(), troposphereModelSelection.zwdMappingFunction.second.pressureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboTemperatureModel(fmt::format("##{} MapZWD - Temperature", label).c_str(), troposphereModelSelection.zwdMappingFunction.second.temperatureModel);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ATMOSPHERE_COMBO_WIDTH);
            changed |= ComboWaterVaporModel(fmt::format("##{} MapZWD - Water vapor", label).c_str(), troposphereModelSelection.zwdMappingFunction.second.waterVaporModel);
            if (troposphereModelSelection.zwdModel.first == TroposphereModel::None
                || troposphereModelSelection.zwdMappingFunction.first == MappingFunction::None) { ImGui::EndDisabled(); }

            ImGui::EndTable();
        }

        ImGui::EndPopup();
    }

    ImGui::SameLine(0.0F, ImGui::GetStyle().ItemInnerSpacing.x);
    std::string labelStr = label;
    ImGui::TextUnformatted(labelStr.substr(0, labelStr.find('#')).c_str());

    return changed;
}

ZenithDelay calcTroposphericDelayAndMapping(const InsTime& insTime, const Eigen::Vector3d& lla_pos, double elevation, double /* azimuth */,
                                            const TroposphereModelSelection& troposphereModels)
{
    if (lla_pos(2) < -1000 || lla_pos(2) > 1e4)
    {
        LOG_TRACE("Not calculating tropospheric delay, due to altitude being invalid: {}m", lla_pos(2));
        return {};
    }

    enum
    {
        ZHD,
        ZWD,
        ZHDMapFunc,
        ZWDMapFunc,
        COUNT,
    };

    const std::array<std::reference_wrapper<const AtmosphereModels>, COUNT> atmosphereModels = {
        troposphereModels.zhdModel.second,
        troposphereModels.zwdModel.second,
        troposphereModels.zhdMappingFunction.second,
        troposphereModels.zwdMappingFunction.second,
    };
    std::array<double, COUNT> pressure{};    // Total barometric pressure in [millibar]
    std::array<double, COUNT> temperature{}; // Absolute temperature in [K]
    std::array<double, COUNT> waterVapor{};  // Partial pressure of water vapour in [hPa]

    GPToutput gpt2outputs;
    GPToutput gpt3outputs;
    // UTC->GPST
    auto epoch_temp = InsTime(insTime) + std::chrono::duration<long double>(insTime.leapGps2UTC());
    double mjd = static_cast<double>(epoch_temp.toMJD().mjd_day) + static_cast<double>(epoch_temp.toMJD().mjd_frac);

    if (troposphereModels.zhdModel.first == TroposphereModel::GPT2
        || troposphereModels.zwdModel.first == TroposphereModel::GPT2
        || troposphereModels.zhdMappingFunction.first == MappingFunction::VMF_GPT2
        || troposphereModels.zwdMappingFunction.first == MappingFunction::VMF_GPT2)
    {
        GPT2_param(mjd, lla_pos, internal::GPT2_grid, gpt2outputs);
    }

    if (troposphereModels.zhdModel.first == TroposphereModel::GPT3
        || troposphereModels.zwdModel.first == TroposphereModel::GPT3
        || troposphereModels.zhdMappingFunction.first == MappingFunction::VMF_GPT3
        || troposphereModels.zwdMappingFunction.first == MappingFunction::VMF_GPT3)
    {
        GPT3_param(mjd, lla_pos, internal::GPT3_grid, gpt3outputs);
    }

    LOG_DATA("Calculating Atmosphere parameters (ZHD={}, ZWD={}, ZHDMapFunc={}, ZWDMapFunc={}, ",
             fmt::underlying(ZHD), fmt::underlying(ZWD), fmt::underlying(ZHDMapFunc), fmt::underlying(ZWDMapFunc));
    for (size_t i = 0; i < COUNT; i++)
    {
        bool alreadyCalculated = false;
        for (size_t j = 0; j < i; j++)
        {
            if (atmosphereModels.at(i).get().pressureModel == atmosphereModels.at(j).get().pressureModel)
            {
                pressure.at(i) = pressure.at(j);
                alreadyCalculated = true;
                break;
            }
        }

        if (atmosphereModels.at(i).get().pressureModel == PressureModel::GPT2)
        {
            GPT2_param(mjd, lla_pos, internal::GPT2_grid, gpt2outputs);
            pressure.at(i) = gpt2outputs.p;
        }

        if (atmosphereModels.at(i).get().pressureModel == PressureModel::GPT3)
        {
            GPT3_param(mjd, lla_pos, internal::GPT3_grid, gpt3outputs);
            pressure.at(i) = gpt3outputs.p;
        }

        if (!alreadyCalculated) { pressure.at(i) = calcTotalPressure(lla_pos(2), atmosphereModels.at(i).get().pressureModel); }
        LOG_DATA("  []: {}: p {} [millibar] (Total barometric pressure) - value {}", i, to_string(atmosphereModels.at(i).get().pressureModel),
                 pressure.at(i), alreadyCalculated ? "reused" : "calculated");

        alreadyCalculated = false;
        for (size_t j = 0; j < i; j++)
        {
            if (atmosphereModels.at(i).get().temperatureModel == atmosphereModels.at(j).get().temperatureModel)
            {
                temperature.at(i) = temperature.at(j);
                alreadyCalculated = true;
                break;
            }
        }

        if (atmosphereModels.at(i).get().pressureModel == PressureModel::GPT2)
        {
            GPT2_param(mjd, lla_pos, internal::GPT2_grid, gpt2outputs);
            temperature.at(i) = gpt2outputs.T;
        }

        if (atmosphereModels.at(i).get().pressureModel == PressureModel::GPT3)
        {
            GPT3_param(mjd, lla_pos, internal::GPT3_grid, gpt3outputs);
            temperature.at(i) = gpt3outputs.T;
        }

        if (!alreadyCalculated) { temperature.at(i) = calcAbsoluteTemperature(lla_pos(2), atmosphereModels.at(i).get().temperatureModel); }
        LOG_DATA("  []: {}: T {} [K] (Absolute temperature) - value {}", i, to_string(atmosphereModels.at(i).get().temperatureModel),
                 temperature.at(i), alreadyCalculated ? "reused" : "calculated");

        alreadyCalculated = false;
        for (size_t j = 0; j < i; j++)
        {
            if (atmosphereModels.at(i).get().waterVaporModel == atmosphereModels.at(j).get().waterVaporModel)
            {
                waterVapor.at(i) = waterVapor.at(j);
                alreadyCalculated = true;
                break;
            }
        }

        if (atmosphereModels.at(i).get().pressureModel == PressureModel::GPT2)
        {
            GPT2_param(mjd, lla_pos, internal::GPT2_grid, gpt2outputs);
            waterVapor.at(i) = gpt2outputs.e;
        }

        if (atmosphereModels.at(i).get().pressureModel == PressureModel::GPT3)
        {
            GPT3_param(mjd, lla_pos, internal::GPT3_grid, gpt3outputs);
            waterVapor.at(i) = gpt3outputs.e;
        }

        // Partial pressure of water vapour in [millibar] - RTKLIB ch. E.5, p. 149 specifies 70%
        if (!alreadyCalculated) { waterVapor.at(i) = calcWaterVaporPartialPressure(temperature.at(i), 0.7, atmosphereModels.at(i).get().waterVaporModel); }
        LOG_DATA("  []: {}: e {} [millibar] (Partial pressure of water vapour) - value {}", i, to_string(atmosphereModels.at(i).get().waterVaporModel),
                 waterVapor.at(i), alreadyCalculated ? "reused" : "calculated");
    }

    double zhd = 0.0;
    switch (troposphereModels.zhdModel.first)
    {
    case TroposphereModel::Saastamoinen:
        zhd = calcZHD_Saastamoinen(lla_pos, pressure[ZHD]);
        break;
    case TroposphereModel::GPT2:
    {
        zhd = calcZHD_Saastamoinen(lla_pos, gpt2outputs.p);
        // LOG_INFO("p {}", p);
        // LOG_INFO("ZHD {}", zhd);
        break;
    }
    case TroposphereModel::GPT3:
    {
        zhd = calcZHD_Saastamoinen(lla_pos, gpt3outputs.p);
        // LOG_INFO("p {}", p);
        // LOG_INFO("ZHD {}", zhd);
        break;
    }
    case TroposphereModel::None:
    case TroposphereModel::COUNT:
        break;
    }

    double zwd = 0.0;
    switch (troposphereModels.zwdModel.first)
    {
    case TroposphereModel::Saastamoinen:
        zwd = calcZWD_Saastamoinen(temperature[ZWD], waterVapor[ZWD]);
        break;
    case TroposphereModel::GPT2:
    {
        zwd = asknewet(gpt2outputs.e, gpt2outputs.Tm, gpt2outputs.la);
        break;
    }
    case TroposphereModel::GPT3:
    {
        zwd = asknewet(gpt3outputs.e, gpt3outputs.Tm, gpt3outputs.la);
        // LOG_INFO("p {}", p);
        // LOG_INFO("ZWD {}", zwd);
        break;
    }
    case TroposphereModel::None:
    case TroposphereModel::COUNT:
        break;
    }

    double zhdMappingFactor = 1.0;
    switch (troposphereModels.zhdMappingFunction.first)
    {
    case MappingFunction::Cosecant:
        zhdMappingFactor = calcTropoMapFunc_cosecant(elevation);
        break;
    case MappingFunction::VMF_GPT2:
    {
        zhdMappingFactor = vmf1h(gpt2outputs.ah, mjd, lla_pos(0), lla_pos(2), M_PI / 2.0 - elevation);
        // LOG_INFO("zhdMappingFactor {}", zhdMappingFactor);
        break;
    }
    case MappingFunction::VMF_GPT3:
    {
        zhdMappingFactor = vmf1h(gpt3outputs.ah, mjd, lla_pos(0), lla_pos(2), M_PI / 2.0 - elevation);
        // LOG_INFO("zhdMappingFactor {}", zhdMappingFactor);
        break;
    }
    case MappingFunction::None:
    case MappingFunction::COUNT:
        break;
    }

    double zwdMappingFactor = 1.0;
    switch (troposphereModels.zwdMappingFunction.first)
    {
    case MappingFunction::Cosecant:
        zwdMappingFactor = calcTropoMapFunc_cosecant(elevation);
        break;
    case MappingFunction::VMF_GPT2:
    {
        zwdMappingFactor = vmf1w(gpt2outputs.aw, M_PI / 2.0 - elevation);
        // LOG_INFO("zwdMappingFactor {}", zwdMappingFactor);
        break;
    }
    case MappingFunction::VMF_GPT3:
    {
        zwdMappingFactor = vmf1w(gpt3outputs.aw, M_PI / 2.0 - elevation);
        // LOG_INFO("zwdMappingFactor {}", zwdMappingFactor);
        break;
    }
    case MappingFunction::None:
    case MappingFunction::COUNT:
        break;
    }

    return { .ZHD = zhd,
             .ZWD = zwd,
             .zhdMappingFactor = zhdMappingFactor,
             .zwdMappingFactor = zwdMappingFactor };
}

double tropoErrorVar(double dpsr_T, double elevation)
{
    constexpr double ERR_SAAS = 0.3; // Saastamoinen model error std [m] (maximum zenith wet delay - formulas with worst possible values)

    return dpsr_T == 0.0 ? 0.0 : std::pow(ERR_SAAS / (std::sin(elevation) + 0.1), 2);
}

void to_json(json& j, const AtmosphereModels& obj)
{
    j = json{
        { "pressureModel", obj.pressureModel },
        { "temperatureModel", obj.temperatureModel },
        { "waterVaporModel", obj.waterVaporModel },
    };
}

void from_json(const json& j, AtmosphereModels& obj)
{
    if (j.contains("pressureModel")) { j.at("pressureModel").get_to(obj.pressureModel); }
    if (j.contains("temperatureModel")) { j.at("temperatureModel").get_to(obj.temperatureModel); }
    if (j.contains("waterVaporModel")) { j.at("waterVaporModel").get_to(obj.waterVaporModel); }
}

void to_json(json& j, const TroposphereModelSelection& obj)
{
    j = json{
        { "zhdModel", obj.zhdModel },
        { "zwdModel", obj.zwdModel },
        { "zhdMappingFunction", obj.zhdMappingFunction },
        { "zwdMappingFunction", obj.zwdMappingFunction },
    };
}

void from_json(const json& j, TroposphereModelSelection& obj)
{
    if (j.contains("zhdModel")) { j.at("zhdModel").get_to(obj.zhdModel); }
    if (j.contains("zwdModel")) { j.at("zwdModel").get_to(obj.zwdModel); }
    if (j.contains("zhdMappingFunction")) { j.at("zhdMappingFunction").get_to(obj.zhdMappingFunction); }
    if (j.contains("zwdMappingFunction")) { j.at("zwdMappingFunction").get_to(obj.zwdMappingFunction); }
}

} // namespace NAV
