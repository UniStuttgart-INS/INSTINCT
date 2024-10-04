// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ErrorModel.hpp"

#include "NodeRegistry.hpp"
#include <algorithm>
#include <imgui.h>
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/GNSS/Functions.hpp"

#include "util/Eigen.hpp"
#include "util/StringUtil.hpp"

#include <imgui_internal.h>
#include <limits>
#include <set>
#include <type_traits>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), ImuObsWDelta::type(), PosVelAtt::type(), GnssObs::type() };

} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::ErrorModel::ErrorModel()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 812, 530 };

    nm::CreateInputPin(this, "True", Pin::Type::Flow, supportedDataIdentifier, &ErrorModel::receiveObs);

    nm::CreateOutputPin(this, "Biased", Pin::Type::Flow, supportedDataIdentifier);

    std::mt19937_64 gen(static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<uint64_t> dist(0, std::numeric_limits<uint64_t>::max() / 2);

    _imuAccelerometerRng.seed = dist(gen);
    _imuGyroscopeRng.seed = dist(gen);

    _imuAccelerometerRWRng.seed = dist(gen);
    _imuGyroscopeRWRng.seed = dist(gen);
    _imuAccelerometerIRWRng.seed = dist(gen);
    _imuGyroscopeIRWRng.seed = dist(gen);

    _positionRng.seed = dist(gen);
    _velocityRng.seed = dist(gen);
    _attitudeRng.seed = dist(gen);

    _pseudorangeRng.seed = dist(gen);
    _carrierPhaseRng.seed = dist(gen);
    _dopplerRng.seed = dist(gen);
    _ambiguityRng.seed = dist(gen);
    _cycleSlipRng.seed = dist(gen);
}

NAV::ErrorModel::~ErrorModel()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ErrorModel::typeStatic()
{
    return "ErrorModel";
}

std::string NAV::ErrorModel::type() const
{
    return typeStatic();
}

std::string NAV::ErrorModel::category()
{
    return "Data Processor";
}

void NAV::ErrorModel::guiConfig()
{
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.size() != 1)
    {
        ImGui::TextUnformatted("Please connect the input pin to show the options");
        return;
    }

    float itemWidth = 470 * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 180 * gui::NodeEditorApplication::windowFontRatio();

    auto inputDoubleWithUnit = [&](const char* title, double& data, auto& unit, const char* combo_items_separated_by_zeros, const char* format) {
        if (auto response = gui::widgets::InputDoubleWithUnit(fmt::format("{}##{}", title, size_t(id)).c_str(), itemWidth, unitWidth,
                                                              &data, reinterpret_cast<int*>(&unit), combo_items_separated_by_zeros, 0.0, 0.0,
                                                              format, ImGuiInputTextFlags_CharsScientific))
        {
            if (response == gui::widgets::InputWithUnitChange_Input) { LOG_DEBUG("{}: {} changed to {}", nameId(), title, data); }
            if (response == gui::widgets::InputWithUnitChange_Unit) { LOG_DEBUG("{}: {} unit changed to {}", nameId(), title, fmt::underlying(unit)); }
            flow::ApplyChanges();
        }
    };

    auto inputVector3WithUnit = [&](const char* title, Eigen::Vector3d& data, auto& unit, const char* combo_items_separated_by_zeros, const char* format) {
        if (auto response = gui::widgets::InputDouble3WithUnit(fmt::format("{}##{}", title, size_t(id)).c_str(), itemWidth, unitWidth,
                                                               data.data(), reinterpret_cast<int*>(&unit), combo_items_separated_by_zeros,
                                                               format, ImGuiInputTextFlags_CharsScientific))
        {
            if (response == gui::widgets::InputWithUnitChange_Input) { LOG_DEBUG("{}: {} changed to {}", nameId(), title, data.transpose()); }
            if (response == gui::widgets::InputWithUnitChange_Unit) { LOG_DEBUG("{}: {} unit changed to {}", nameId(), title, fmt::underlying(unit)); }
            flow::ApplyChanges();
        }
    };

    auto rngInput = [&](const char* title, RandomNumberGenerator& rng) {
        float currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##rng.useSeed {} {}", title, size_t(id)).c_str(), &rng.useSeed))
        {
            LOG_DEBUG("{}: {} rng.useSeed changed to {}", nameId(), title, rng.useSeed);
            flow::ApplyChanges();
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Use seed?"); }
        ImGui::SameLine();
        if (!rng.useSeed)
        {
            ImGui::BeginDisabled();
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX));
        if (ImGui::SliderULong(fmt::format("{} Seed##{}", title, size_t(id)).c_str(), &rng.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu"))
        {
            LOG_DEBUG("{}: {} rng.seed changed to {}", nameId(), title, rng.seed);
            flow::ApplyChanges();
        }
        if (!rng.useSeed)
        {
            ImGui::EndDisabled();
        }
    };

    auto noiseGuiInput = [&]<typename T>(const char* title, T& data, auto& unit, const char* combo_items_separated_by_zeros, const char* format, RandomNumberGenerator& rng) {
        if constexpr (std::is_same_v<T, double>) { inputDoubleWithUnit(title, data, unit, combo_items_separated_by_zeros, format); }
        else if constexpr (std::is_same_v<T, Eigen::Vector3d>) { inputVector3WithUnit(title, data, unit, combo_items_separated_by_zeros, format); }

        rngInput(title, rng);
    };

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta || _inputType == InputType::PosVelAtt)
    {
        ImGui::TextUnformatted("Offsets:");
        ImGui::Indent();
        {
            if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
            {
                inputVector3WithUnit("Accelerometer Bias (platform)", _imuAccelerometerBias_p, _imuAccelerometerBiasUnit, "m/s^2\0\0", "%.2g");
                inputVector3WithUnit("Gyroscope Bias (platform)", _imuGyroscopeBias_p, _imuGyroscopeBiasUnit, "rad/s\0deg/s\0\0", "%.2g");
            }
            else if (_inputType == InputType::PosVelAtt)
            {
                inputVector3WithUnit(fmt::format("Position Bias ({})", _positionBiasUnit == PositionBiasUnits::meter ? "NED" : "LatLonAlt").c_str(),
                                     _positionBias, _positionBiasUnit, "m, m, m\0rad, rad, m\0deg, deg, m\0\0", "%.2g");
                inputVector3WithUnit("Velocity Bias (NED)", _velocityBias, _velocityBiasUnit, "m/s\0\0", "%.2g");
                inputVector3WithUnit("RollPitchYaw Bias", _attitudeBias, _attitudeBiasUnit, "rad\0deg\0\0", "%.2g");
            }
        }
        ImGui::Unindent();
    }

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta || _inputType == InputType::PosVelAtt || _inputType == InputType::GnssObs)
    {
        ImGui::TextUnformatted("Measurement noise:");
        ImGui::Indent();
        {
            if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
            {
                noiseGuiInput(fmt::format("Accelerometer Noise ({})", _imuAccelerometerNoiseUnit == ImuAccelerometerNoiseUnits::m_s2
                                                                          ? "Standard deviation"
                                                                          : "Variance")
                                  .c_str(),
                              _imuAccelerometerNoise, _imuAccelerometerNoiseUnit, "m/s^2\0m^2/s^4\0\0", "%.2g", _imuAccelerometerRng);
                noiseGuiInput(fmt::format("Gyroscope Noise ({})", _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::rad_s || _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::deg_s
                                                                      ? "Standard deviation"
                                                                      : "Variance")
                                  .c_str(),
                              _imuGyroscopeNoise, _imuGyroscopeNoiseUnit, "rad/s\0deg/s\0rad^2/s^2\0deg^2/s^2\0\0", "%.2g", _imuGyroscopeRng);
                if (_inputType == InputType::ImuObsWDelta)
                {
                    ImGui::SetNextItemWidth(itemWidth);
                    if (ImGui::InputDoubleL(fmt::format("Delta Vel & Theta averaging window size##{}", size_t(id)).c_str(), &_imuObsWDeltaAverageWindow,
                                            1.0, std::numeric_limits<double>::max(), 1.0, 1.0, "%.2f"))
                    {
                        flow::ApplyChanges();
                    }
                }
            }
            else if (_inputType == InputType::PosVelAtt)
            {
                noiseGuiInput(fmt::format("Position Noise ({})", _positionNoiseUnit == PositionNoiseUnits::meter
                                                                         || _positionNoiseUnit == PositionNoiseUnits::rad_rad_m
                                                                         || _positionNoiseUnit == PositionNoiseUnits::deg_deg_m
                                                                     ? "Standard deviation"
                                                                     : "Variance")
                                  .c_str(),
                              _positionNoise, _positionNoiseUnit, "m, m, m\0rad, rad, m\0deg, deg, m\0m^2, m^2, m^2\0rad^2, rad^2, m^2\0deg^2, deg^2, m^2\0\0",
                              "%.2g", _positionRng);
                noiseGuiInput(fmt::format("Velocity Noise ({})", _velocityNoiseUnit == VelocityNoiseUnits::m_s ? "Standard deviation"
                                                                                                               : "Variance")
                                  .c_str(),
                              _velocityNoise, _velocityNoiseUnit, "m/s\0m^2/s^2\0\0", "%.2g", _velocityRng);
                noiseGuiInput(fmt::format("Attitude Noise ({})", _attitudeNoiseUnit == AttitudeNoiseUnits::rad || _attitudeNoiseUnit == AttitudeNoiseUnits::deg
                                                                     ? "Standard deviation"
                                                                     : "Variance")
                                  .c_str(),
                              _attitudeNoise, _attitudeNoiseUnit, "rad\0deg\0rad^2\0deg^2\0\0", "%.2g", _attitudeRng);
            }
            else if (_inputType == InputType::GnssObs)
            {
                noiseGuiInput("Pseudorange Noise", _gui_pseudorangeNoise, _gui_pseudorangeNoiseUnit, "m\0\0", "%.3g", _pseudorangeRng);
                noiseGuiInput("Carrier-phase Noise", _gui_carrierPhaseNoise, _gui_carrierPhaseNoiseUnit, "m\0\0", "%.3g", _carrierPhaseRng);
                noiseGuiInput("Doppler/Range-rate Noise", _gui_dopplerNoise, _gui_dopplerNoiseUnit, "m/s\0\0", "%.3g", _dopplerRng);
            }
        }
        ImGui::Unindent();
    }
    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
    {
        ImGui::TextUnformatted("Random walk noise:");
        ImGui::Indent();
        noiseGuiInput("Accelerometer RW (Std. dev)", _imuAccelerometerRW, _imuAccelerometerRWUnit, "m/s^2/√(s)\0m/s^2/√(h)\0\0", "%.2g", _imuAccelerometerRWRng);
        noiseGuiInput("Gyroscope RW (Std. dev)", _imuGyroscopeRW, _imuGyroscopeRWUnit, "rad/s/√(s)\0rad/s/√(h))\0deg/s/√(s)\0deg/s/√(h)\0\0", "%.2g", _imuGyroscopeRWRng);
        ImGui::Unindent();
        ImGui::TextUnformatted("Integrated Random walk noise:");
        ImGui::Indent();
        noiseGuiInput("Accelerometer IRW (Std. dev)", _imuAccelerometerIRW, _imuAccelerometerIRWUnit, "m/s^3/√(s)\0m/s^3/√(h)\0\0", "%.2g", _imuAccelerometerIRWRng);
        noiseGuiInput("Gyroscope IRW (Std. dev)", _imuGyroscopeIRW, _imuGyroscopeIRWUnit, "rad/s^2/√(s)\0rad/s^2/√(h)\0deg/s^2/√(s)\0deg/s^2/√(h)\0\0", "%.2g", _imuGyroscopeIRWRng);
        ImGui::Unindent();
    }

    if (_inputType == InputType::GnssObs)
    {
        ImGui::TextUnformatted("Ambiguities:");
        ImGui::Indent();
        {
            ImGui::SetNextItemWidth((itemWidth - ImGui::GetStyle().ItemInnerSpacing.x) / 2.0F);
            if (ImGui::InputIntL(fmt::format("##Ambiguity Bounds lower {}", size_t(id)).c_str(), _gui_ambiguityLimits.data(),
                                 std::numeric_limits<int>::lowest(), _gui_ambiguityLimits[1], 0, 0))
            {
                LOG_DEBUG("{}: Ambiguity lower bound changed to {}", nameId(), _gui_ambiguityLimits[0]);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::SetNextItemWidth((itemWidth - ImGui::GetStyle().ItemInnerSpacing.x) / 2.0F);
            if (ImGui::InputIntL(fmt::format("Ambiguity Bounds##Ambiguity Bounds upper{}", size_t(id)).c_str(), &_gui_ambiguityLimits[1],
                                 _gui_ambiguityLimits[0], std::numeric_limits<int>::max(), 0, 0))
            {
                LOG_DEBUG("{}: Ambiguity upper bound changed to {}", nameId(), _gui_ambiguityLimits[1]);
                flow::ApplyChanges();
            }
            rngInput("Ambiguity", _ambiguityRng);
        }
        ImGui::Unindent();

        ImGui::TextUnformatted("Cycle-slip:");
        ImGui::Indent();
        {
            noiseGuiInput("Frequency", _gui_cycleSlipFrequency, _gui_cycleSlipFrequencyUnit, "/ day\0/ hour\0/ minute\0\0", "%.2g", _cycleSlipRng);

            if (auto response = gui::widgets::SliderDoubleWithUnit(fmt::format("Detection probability (LLI)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                                                   &_gui_cycleSlipDetectionProbability, 0.0, 100.0,
                                                                   reinterpret_cast<int*>(&_gui_cycleSlipDetectionProbabilityUnit), "%\0\0", "%.2f"))
            {
                if (response == gui::widgets::InputWithUnitChange_Input) { LOG_DEBUG("{}: Detection probability (LLI) changed to {}", nameId(), _gui_cycleSlipDetectionProbability); }
                if (response == gui::widgets::InputWithUnitChange_Unit) { LOG_DEBUG("{}: Detection probability (LLI) unit changed to {}", nameId(), fmt::underlying(_gui_cycleSlipDetectionProbabilityUnit)); }
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("The chance that the Lock-of-Loss (LLI) indicator is set, when a cycle-slip occurs");

            ImGui::SetNextItemWidth(itemWidth);
            if (ImGui::DragInt(fmt::format("Ambiguity Range to slip to##{}", size_t(id)).c_str(), &_gui_cycleSlipRange,
                               1.0F, 1, std::numeric_limits<int>::max(), "+/- %d"))
            {
                LOG_DEBUG("{}: Cycle-slip ambiguity range changed to {}", nameId(), _gui_cycleSlipRange);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("If the ambiguity range is set, the cycle-slips are bound by it too");

            ImGui::SetNextItemWidth(itemWidth);
            if (ShowFrequencySelector(fmt::format("Satellite Frequencies##{}", size_t(id)).c_str(), _filterFreq))
            {
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("Frequency and code selection only affects the cycle-slip simulation");

            ImGui::SetNextItemWidth(itemWidth);
            if (ShowCodeSelector(fmt::format("Signal Codes##{}", size_t(id)).c_str(), _filterCode, _filterFreq))
            {
                flow::ApplyChanges();
            }
        }
        ImGui::Unindent();

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Simulated Ambiguities and Cycle-slips##{}", size_t(id)).c_str()))
        {
            auto nAmb = static_cast<int>(_ambiguities.size());
            if (nAmb > 0)
            {
                std::set<InsTime> ambiguityTimes;
                for (int i = 0; i < nAmb; i++)
                {
                    const auto& ambVec = std::next(_ambiguities.begin(), i)->second;
                    for (const auto& amb : ambVec)
                    {
                        ambiguityTimes.insert(amb.first);
                    }
                }
                if (ambiguityTimes.size() < 64 - 1)
                {
                    ImGui::PushFont(Application::MonoFont());

                    if (ImGui::BeginTable(fmt::format("Ambiguities##{}", size_t(id)).c_str(), static_cast<int>(ambiguityTimes.size() + 1),
                                          ImGuiTableFlags_Borders | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY,
                                          ImVec2(0.0F, 300.0F * gui::NodeEditorApplication::monoFontRatio())))
                    {
                        ImGui::TableSetupColumn("");
                        for (const auto& time : ambiguityTimes)
                        {
                            auto t = time.toYMDHMS(GPST);
                            t.sec = std::round(t.sec * 1e2) / 1e2;
                            ImGui::TableSetupColumn(str::replaceAll_copy(fmt::format("{}", t), " ", "\n").c_str());
                        }
                        ImGui::TableSetupScrollFreeze(1, 1);
                        ImGui::TableHeadersRow();

                        for (int i = 0; i < nAmb; i++)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn();
                            const auto& ambiguities = std::next(_ambiguities.begin(), i);
                            if (ambiguities == _ambiguities.end()) { break; }
                            ImGui::TextUnformatted(fmt::format("{}", ambiguities->first).c_str());
                            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]));

                            for (const auto& time : ambiguityTimes)
                            {
                                ImGui::TableNextColumn();
                                auto iter = std::find_if(ambiguities->second.begin(), ambiguities->second.end(), [&time](const auto& timeAndAmb) {
                                    return timeAndAmb.first == time;
                                });
                                if (iter != ambiguities->second.end())
                                {
                                    ImVec4 color;
                                    auto cycleIter = std::find_if(_cycleSlips.begin(), _cycleSlips.end(), [&](const CycleSlipInfo& cycleSlip) {
                                        return cycleSlip.time == time && cycleSlip.satSigId == ambiguities->first;
                                    });
                                    if (cycleIter != _cycleSlips.end()) { color = ImColor(240, 128, 128); }
                                    else if (ambiguities->second.back().first == time) { color = ImGui::GetStyle().Colors[ImGuiCol_Text]; }
                                    else { color = ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]; }
                                    ImGui::TextColored(color, "%s%s",
                                                       fmt::format("{}", iter->second).c_str(),
                                                       cycleIter != _cycleSlips.end() && cycleIter->LLI ? " (LLI)" : "");
                                    if (ImGui::IsItemHovered() && cycleIter != _cycleSlips.end()) { ImGui::SetTooltip("Cycle-slip"); }
                                }
                                else if (time < ambiguities->second.front().first)
                                {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, ImColor(70, 70, 70));
                                }
                            }
                        }

                        ImGui::EndTable();
                    }
                    ImGui::PopFont();
                }
                else
                {
                    ImGui::TextColored(ImColor(255, 0, 0), "More than 64 timestamps for ambiguities, which cannot be displayed in a table");
                }
            }

            ImGui::TreePop();
        }
    }
}

json NAV::ErrorModel::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuAccelerometerBiasUnit"] = _imuAccelerometerBiasUnit;
    j["imuAccelerometerBias_p"] = _imuAccelerometerBias_p;
    j["imuGyroscopeBiasUnit"] = _imuGyroscopeBiasUnit;
    j["imuGyroscopeBias_p"] = _imuGyroscopeBias_p;
    j["imuAccelerometerNoiseUnit"] = _imuAccelerometerNoiseUnit;
    j["imuAccelerometerNoise"] = _imuAccelerometerNoise;
    j["imuAccelerometerRng"] = _imuAccelerometerRng;
    j["imuGyroscopeNoiseUnit"] = _imuGyroscopeNoiseUnit;
    j["imuGyroscopeNoise"] = _imuGyroscopeNoise;
    j["imuGyroscopeRng"] = _imuGyroscopeRng;
    j["imuAccelerometerRWUnit"] = _imuAccelerometerRWUnit;
    j["imuAccelerometerRW"] = _imuAccelerometerRW;
    j["imuAccelerometerRWRng"] = _imuAccelerometerRWRng;

    j["imuGyroscopeRWUnit"] = _imuGyroscopeRWUnit;
    j["imuGyroscopeRW"] = _imuGyroscopeRW;
    j["imuGyroscopeRWRng"] = _imuGyroscopeRWRng;
    j["imuAccelerometerIRWUnit"] = _imuAccelerometerIRWUnit;
    j["imuAccelerometerIRW"] = _imuAccelerometerIRW;
    j["imuAccelerometerIRWRng"] = _imuAccelerometerIRWRng;
    j["imuGyroscopeIRWUnit"] = _imuGyroscopeIRWUnit;
    j["imuGyroscopeIRW"] = _imuGyroscopeIRW;
    j["imuGyroscopeIRWRng"] = _imuGyroscopeIRWRng;

    // #########################################################################################################################################
    j["positionBiasUnit"] = _positionBiasUnit;
    j["positionBias"] = _positionBias;
    j["velocityBiasUnit"] = _velocityBiasUnit;
    j["velocityBias"] = _velocityBias;
    j["attitudeBiasUnit"] = _attitudeBiasUnit;
    j["attitudeBias"] = _attitudeBias;
    j["positionNoiseUnit"] = _positionNoiseUnit;
    j["positionNoise"] = _positionNoise;
    j["positionRng"] = _positionRng;
    j["velocityNoiseUnit"] = _velocityNoiseUnit;
    j["velocityNoise"] = _velocityNoise;
    j["velocityRng"] = _velocityRng;
    j["attitudeNoiseUnit"] = _attitudeNoiseUnit;
    j["attitudeNoise"] = _attitudeNoise;
    j["attitudeRng"] = _attitudeRng;
    // #########################################################################################################################################
    j["pseudorangeNoiseUnit"] = _gui_pseudorangeNoiseUnit;
    j["pseudorangeNoise"] = _gui_pseudorangeNoise;
    j["pseudorangeRng"] = _pseudorangeRng;
    j["carrierPhaseNoiseUnit"] = _gui_carrierPhaseNoiseUnit;
    j["carrierPhaseNoise"] = _gui_carrierPhaseNoise;
    j["carrierPhaseRng"] = _carrierPhaseRng;
    j["dopplerNoiseUnit"] = _gui_dopplerNoiseUnit;
    j["dopplerNoise"] = _gui_dopplerNoise;
    j["dopplerRng"] = _dopplerRng;
    j["ambiguityLimits"] = _gui_ambiguityLimits;
    j["ambiguityRng"] = _ambiguityRng;
    j["cycleSlipFrequencyUnit"] = _gui_cycleSlipFrequencyUnit;
    j["cycleSlipFrequency"] = _gui_cycleSlipFrequency;
    j["cycleSlipRange"] = _gui_cycleSlipRange;
    j["cycleSlipDetectionProbabilityUnit"] = _gui_cycleSlipDetectionProbabilityUnit;
    j["cycleSlipDetectionProbability"] = _gui_cycleSlipDetectionProbability;
    j["cycleSlipRng"] = _cycleSlipRng;

    j["filterFreq"] = Frequency_(_filterFreq);
    j["filterCode"] = _filterCode;

    return j;
}

void NAV::ErrorModel::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuAccelerometerBiasUnit")) { j.at("imuAccelerometerBiasUnit").get_to(_imuAccelerometerBiasUnit); }
    if (j.contains("imuAccelerometerBias_p")) { j.at("imuAccelerometerBias_p").get_to(_imuAccelerometerBias_p); }
    if (j.contains("imuGyroscopeBiasUnit")) { j.at("imuGyroscopeBiasUnit").get_to(_imuGyroscopeBiasUnit); }
    if (j.contains("imuGyroscopeBias_p")) { j.at("imuGyroscopeBias_p").get_to(_imuGyroscopeBias_p); }
    if (j.contains("imuAccelerometerNoiseUnit")) { j.at("imuAccelerometerNoiseUnit").get_to(_imuAccelerometerNoiseUnit); }
    if (j.contains("imuAccelerometerNoise")) { j.at("imuAccelerometerNoise").get_to(_imuAccelerometerNoise); }
    if (j.contains("imuAccelerometerRng")) { j.at("imuAccelerometerRng").get_to(_imuAccelerometerRng); }
    if (j.contains("imuGyroscopeNoiseUnit")) { j.at("imuGyroscopeNoiseUnit").get_to(_imuGyroscopeNoiseUnit); }
    if (j.contains("imuGyroscopeNoise")) { j.at("imuGyroscopeNoise").get_to(_imuGyroscopeNoise); }
    if (j.contains("imuGyroscopeRng")) { j.at("imuGyroscopeRng").get_to(_imuGyroscopeRng); }

    if (j.contains("imuAccelerometerRWUnit")) { j.at("imuAccelerometerRWUnit").get_to(_imuAccelerometerRWUnit); }
    if (j.contains("imuAccelerometerRW")) { j.at("imuAccelerometerRW").get_to(_imuAccelerometerRW); }
    if (j.contains("imuAccelerometerRWRng")) { j.at("imuAccelerometerRWRng").get_to(_imuAccelerometerRWRng); }
    if (j.contains("imuGyroscopeRWUnit")) { j.at("imuGyroscopeRWUnit").get_to(_imuGyroscopeRWUnit); }
    if (j.contains("imuGyroscopeRW")) { j.at("imuGyroscopeRW").get_to(_imuGyroscopeRW); }
    if (j.contains("imuGyroscopeRWRng")) { j.at("imuGyroscopeRWRng").get_to(_imuGyroscopeRWRng); }
    if (j.contains("imuAccelerometerIRWUnit")) { j.at("imuAccelerometerIRWUnit").get_to(_imuAccelerometerIRWUnit); }
    if (j.contains("imuAccelerometerIRW")) { j.at("imuAccelerometerIRW").get_to(_imuAccelerometerIRW); }
    if (j.contains("imuAccelerometerIRWRng")) { j.at("imuAccelerometerIRWRng").get_to(_imuAccelerometerIRWRng); }
    if (j.contains("imuGyroscopeIRWUnit")) { j.at("imuGyroscopeIRWUnit").get_to(_imuGyroscopeIRWUnit); }
    if (j.contains("imuGyroscopeIRW")) { j.at("imuGyroscopeIRW").get_to(_imuGyroscopeIRW); }
    if (j.contains("imuGyroscopeIRWRng")) { j.at("imuGyroscopeIRWRng").get_to(_imuGyroscopeIRWRng); }
    // #########################################################################################################################################
    if (j.contains("positionBiasUnit")) { j.at("positionBiasUnit").get_to(_positionBiasUnit); }
    if (j.contains("positionBias")) { j.at("positionBias").get_to(_positionBias); }
    if (j.contains("velocityBiasUnit")) { j.at("velocityBiasUnit").get_to(_velocityBiasUnit); }
    if (j.contains("velocityBias")) { j.at("velocityBias").get_to(_velocityBias); }
    if (j.contains("attitudeBiasUnit")) { j.at("attitudeBiasUnit").get_to(_attitudeBiasUnit); }
    if (j.contains("attitudeBias")) { j.at("attitudeBias").get_to(_attitudeBias); }
    if (j.contains("positionNoiseUnit")) { j.at("positionNoiseUnit").get_to(_positionNoiseUnit); }
    if (j.contains("positionNoise")) { j.at("positionNoise").get_to(_positionNoise); }
    if (j.contains("positionRng")) { j.at("positionRng").get_to(_positionRng); }
    if (j.contains("velocityNoiseUnit")) { j.at("velocityNoiseUnit").get_to(_velocityNoiseUnit); }
    if (j.contains("velocityNoise")) { j.at("velocityNoise").get_to(_velocityNoise); }
    if (j.contains("velocityRng")) { j.at("velocityRng").get_to(_velocityRng); }
    if (j.contains("attitudeNoiseUnit")) { j.at("attitudeNoiseUnit").get_to(_attitudeNoiseUnit); }
    if (j.contains("attitudeNoise")) { j.at("attitudeNoise").get_to(_attitudeNoise); }
    if (j.contains("attitudeRng")) { j.at("attitudeRng").get_to(_attitudeRng); }
    // #########################################################################################################################################
    if (j.contains("pseudorangeNoiseUnit")) { j.at("pseudorangeNoiseUnit").get_to(_gui_pseudorangeNoiseUnit); }
    if (j.contains("pseudorangeNoise")) { j.at("pseudorangeNoise").get_to(_gui_pseudorangeNoise); }
    if (j.contains("pseudorangeRng")) { j.at("pseudorangeRng").get_to(_pseudorangeRng); }
    if (j.contains("carrierPhaseNoiseUnit")) { j.at("carrierPhaseNoiseUnit").get_to(_gui_carrierPhaseNoiseUnit); }
    if (j.contains("carrierPhaseNoise")) { j.at("carrierPhaseNoise").get_to(_gui_carrierPhaseNoise); }
    if (j.contains("carrierPhaseRng")) { j.at("carrierPhaseRng").get_to(_carrierPhaseRng); }
    if (j.contains("dopplerNoiseUnit")) { j.at("dopplerNoiseUnit").get_to(_gui_dopplerNoiseUnit); }
    if (j.contains("dopplerNoise")) { j.at("dopplerNoise").get_to(_gui_dopplerNoise); }
    if (j.contains("dopplerRng")) { j.at("dopplerRng").get_to(_dopplerRng); }
    if (j.contains("ambiguityLimits")) { j.at("ambiguityLimits").get_to(_gui_ambiguityLimits); }
    if (j.contains("ambiguityRng")) { j.at("ambiguityRng").get_to(_ambiguityRng); }
    if (j.contains("cycleSlipFrequencyUnit")) { j.at("cycleSlipFrequencyUnit").get_to(_gui_cycleSlipFrequencyUnit); }
    if (j.contains("cycleSlipFrequency")) { j.at("cycleSlipFrequency").get_to(_gui_cycleSlipFrequency); }
    if (j.contains("cycleSlipRange")) { j.at("cycleSlipRange").get_to(_gui_cycleSlipRange); }
    if (j.contains("cycleSlipDetectionProbabilityUnit")) { j.at("cycleSlipDetectionProbabilityUnit").get_to(_gui_cycleSlipDetectionProbabilityUnit); }
    if (j.contains("cycleSlipDetectionProbability")) { j.at("cycleSlipDetectionProbability").get_to(_gui_cycleSlipDetectionProbability); }
    if (j.contains("cycleSlipRng")) { j.at("cycleSlipRng").get_to(_cycleSlipRng); }
    if (j.contains("filterFreq"))
    {
        uint64_t value = 0;
        j.at("filterFreq").get_to(value);
        _filterFreq = Frequency_(value);
    }
    if (j.contains("filterCode")) { j.at("filterCode").get_to(_filterCode); }
}

bool NAV::ErrorModel::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
    {
        _imuAccelerometerRng.resetSeed(size_t(id));
        _imuGyroscopeRng.resetSeed(size_t(id));

        _imuAccelerometerRWRng.resetSeed(size_t(id));
        _imuGyroscopeRWRng.resetSeed(size_t(id));
        _imuAccelerometerIRWRng.resetSeed(size_t(id));
        _imuGyroscopeIRWRng.resetSeed(size_t(id));

        RandomWalkAccelerometer.setZero();
        RandomWalkGyroscope.setZero();

        IntegratedRandomWalkAccelerometer.setZero();
        IntegratedRandomWalkGyroscope.setZero();
        IntegratedRandomWalkAccelerometer_velocity.setZero();
        IntegratedRandomWalkGyroscope_velocity.setZero();

        _lastObservationTime.reset();
    }
    else if (_inputType == InputType::PosVelAtt)
    {
        _positionRng.resetSeed(size_t(id));
        _velocityRng.resetSeed(size_t(id));
        _attitudeRng.resetSeed(size_t(id));
    }
    else if (_inputType == InputType::GnssObs)
    {
        _pseudorangeRng.resetSeed(size_t(id));
        _carrierPhaseRng.resetSeed(size_t(id));
        _dopplerRng.resetSeed(size_t(id));
        _ambiguityRng.resetSeed(size_t(id));
        _ambiguities.clear();
        _cycleSlips.clear();
        _cycleSlipRng.resetSeed(size_t(id));
        _cycleSlipWindowStartTime.reset();
        _lastObservationTime.reset();
        _messageFrequency = 0.0;
    }

    return true;
}

void NAV::ErrorModel::afterCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id)
    {
        return; // Link on Output Port
    }

    // Store previous output pin identifier
    auto previousOutputPinDataIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;
    // Overwrite output pin identifier with input pin identifier
    outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = startPin.dataIdentifier;

    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObsWDelta::type() }))
    {
        _inputType = InputType::ImuObsWDelta;
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObs::type() }))
    {
        _inputType = InputType::ImuObs;
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { PosVelAtt::type() }))
    {
        _inputType = InputType::PosVelAtt;
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { GnssObs::type() }))
    {
        _inputType = InputType::GnssObs;
    }

    if (previousOutputPinDataIdentifier != outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier) // If the identifier changed
    {
        // Check if connected links on output port are still valid
        for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
        {
            if (auto* endPin = link.getConnectedPin())
            {
                if (!outputPins.at(OUTPUT_PORT_INDEX_FLOW).canCreateLink(*endPin))
                {
                    // If the link is not valid anymore, delete it
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).deleteLink(*endPin);
                }
            }
        }

        // Refresh all links connected to the output pin if the type changed
        if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != previousOutputPinDataIdentifier)
        {
            for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
            {
                if (auto* connectedPin = link.getConnectedPin())
                {
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).recreateLink(*connectedPin);
                }
            }
        }
    }
}

void NAV::ErrorModel::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if ((endPin.parentNode->id != id                                  // Link on Output port is removed
         && !inputPins.at(INPUT_PORT_INDEX_FLOW).isPinLinked())       //     and the Input port is not linked
        || (startPin.parentNode->id != id                             // Link on Input port is removed
            && !outputPins.at(OUTPUT_PORT_INDEX_FLOW).isPinLinked())) //     and the Output port is not linked
    {
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = supportedDataIdentifier;
    }
}

void NAV::ErrorModel::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = queue.extract_front();
    if (!_lastObservationTime.empty()) { _messageFrequency = 1.0 / static_cast<double>((obs->insTime - _lastObservationTime).count()); }

    // Select the correct data type and make a copy of the node data to modify
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObsSimulated::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receiveImuObs(std::make_shared<ImuObsSimulated>(*std::static_pointer_cast<const ImuObsSimulated>(obs))));
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObsWDelta::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receiveImuObs(std::make_shared<ImuObsWDelta>(*std::static_pointer_cast<const ImuObsWDelta>(obs))));
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObs::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receiveImuObs(std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(obs))));
    }
    else if (_inputType == InputType::ImuObsWDelta)
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receiveImuObsWDelta(std::make_shared<ImuObsWDelta>(*std::static_pointer_cast<const ImuObsWDelta>(obs))));
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { PosVelAtt::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receivePosVelAtt(std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(obs))));
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { GnssObs::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receiveGnssObs(std::make_shared<GnssObs>(*std::static_pointer_cast<const GnssObs>(obs))));
    }

    _lastObservationTime = obs->insTime;
}

std::shared_ptr<NAV::ImuObs> NAV::ErrorModel::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
    // Accelerometer Bias in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerBias_p = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerBiasUnit)
    {
    case ImuAccelerometerBiasUnits::m_s2:
        accelerometerBias_p = _imuAccelerometerBias_p;
        break;
    }
    LOG_DATA("{}: accelerometerBias_p = {} [m/s^2]", nameId(), accelerometerBias_p.transpose());

    // Gyroscope Bias in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeBias_p = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeBiasUnit)
    {
    case ImuGyroscopeBiasUnits::deg_s:
        gyroscopeBias_p = deg2rad(_imuGyroscopeBias_p);
        break;
    case ImuGyroscopeBiasUnits::rad_s:
        gyroscopeBias_p = _imuGyroscopeBias_p;
        break;
    }
    LOG_DATA("{}: gyroscopeBias_p = {} [rad/s]", nameId(), gyroscopeBias_p.transpose());

    // #########################################################################################################################################

    // Accelerometer Noise standard deviation in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerNoiseUnit)
    {
    case ImuAccelerometerNoiseUnits::m_s2:
        accelerometerNoiseStd = _imuAccelerometerNoise;
        break;
    case ImuAccelerometerNoiseUnits::m2_s4:
        accelerometerNoiseStd = _imuAccelerometerNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: accelerometerNoiseStd = {} [m/s^2]", nameId(), accelerometerNoiseStd.transpose());

    // Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeNoiseUnit)
    {
    case ImuGyroscopeNoiseUnits::rad_s:
        gyroscopeNoiseStd = _imuGyroscopeNoise;
        break;
    case ImuGyroscopeNoiseUnits::deg_s:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeNoise);
        break;
    case ImuGyroscopeNoiseUnits::rad2_s2:
        gyroscopeNoiseStd = _imuGyroscopeNoise.cwiseSqrt();
        break;
    case ImuGyroscopeNoiseUnits::deg2_s2:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: gyroscopeNoiseStd = {} [rad/s]", nameId(), gyroscopeNoiseStd.transpose());

    if (!_lastObservationTime.empty())
    {
        auto dt = std::chrono::duration<double>(imuObs->insTime - _lastObservationTime).count();

        // Accelerometer RW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d accelerometerRWStd = Eigen::Vector3d::Zero();
        switch (_imuAccelerometerRWUnit)
        {
        case ImuAccelerometerRWUnits::m_s2_sqrts:
            accelerometerRWStd = _imuAccelerometerRW * sqrt(dt);
            break;
        case ImuAccelerometerRWUnits::m_s2_sqrth:
            accelerometerRWStd = _imuAccelerometerRW / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: accelerometerRWStd = {} [m/s^2/sqrt(s)]", nameId(), accelerometerRWStd.transpose());

        RandomWalkAccelerometer += Eigen::Vector3d{
            _imuAccelerometerRWRng.getRand_normalDist(0.0, accelerometerRWStd(0)),
            _imuAccelerometerRWRng.getRand_normalDist(0.0, accelerometerRWStd(1)),
            _imuAccelerometerRWRng.getRand_normalDist(0.0, accelerometerRWStd(2))
        };

        // Accelerometer IRW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d accelerometerIRWStd = Eigen::Vector3d::Zero();
        switch (_imuAccelerometerIRWUnit)
        {
        case ImuAccelerometerIRWUnits::m_s3_sqrts:
            accelerometerIRWStd = _imuAccelerometerIRW * sqrt(dt);
            break;
        case ImuAccelerometerIRWUnits::m_s3_sqrth:
            accelerometerIRWStd = _imuAccelerometerIRW / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: accelerometerIRWStd = {} [m/s^3/sqrt(s)]", nameId(), accelerometerIRWStd.transpose());

        // compute velocity RW first
        IntegratedRandomWalkAccelerometer_velocity += Eigen::Vector3d{
            _imuAccelerometerIRWRng.getRand_normalDist(0.0, accelerometerIRWStd(0)),
            _imuAccelerometerIRWRng.getRand_normalDist(0.0, accelerometerIRWStd(1)),
            _imuAccelerometerIRWRng.getRand_normalDist(0.0, accelerometerIRWStd(2))
        };

        // then compute IRW
        IntegratedRandomWalkAccelerometer = IntegratedRandomWalkAccelerometer + IntegratedRandomWalkAccelerometer_velocity * dt;

        // Gyro RW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d gyroscopeRWStd = Eigen::Vector3d::Zero();
        switch (_imuGyroscopeRWUnit)
        {
        case ImuGyroscopeRWUnits::rad_s_sqrts:
            gyroscopeRWStd = _imuGyroscopeRW * sqrt(dt);
            break;
        case ImuGyroscopeRWUnits::rad_s_sqrth:
            gyroscopeRWStd = _imuGyroscopeRW / 60.0 * sqrt(dt);
            break;
        case ImuGyroscopeRWUnits::deg_s_sqrts:
            gyroscopeRWStd = deg2rad(_imuGyroscopeRW) * sqrt(dt);
            break;
        case ImuGyroscopeRWUnits::deg_s_sqrth:
            gyroscopeRWStd = deg2rad(_imuGyroscopeRW) / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: gyroscopeRWStd = {} [rad/s/sqrt(s)]", nameId(), gyroscopeRWStd.transpose());

        RandomWalkGyroscope += Eigen::Vector3d{
            _imuGyroscopeRWRng.getRand_normalDist(0.0, gyroscopeRWStd(0)),
            _imuGyroscopeRWRng.getRand_normalDist(0.0, gyroscopeRWStd(1)),
            _imuGyroscopeRWRng.getRand_normalDist(0.0, gyroscopeRWStd(2))
        };

        // Gyro RW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d gyroscopeIRWStd = Eigen::Vector3d::Zero();
        switch (_imuGyroscopeIRWUnit)
        {
        case ImuGyroscopeIRWUnits::rad_s2_sqrts:
            gyroscopeIRWStd = _imuGyroscopeIRW * sqrt(dt);
            break;
        case ImuGyroscopeIRWUnits::rad_s2_sqrth:
            gyroscopeIRWStd = _imuGyroscopeIRW / 60.0 * sqrt(dt);
            break;
        case ImuGyroscopeIRWUnits::deg_s2_sqrts:
            gyroscopeIRWStd = deg2rad(_imuGyroscopeIRW) * sqrt(dt);
            break;
        case ImuGyroscopeIRWUnits::deg_s2_sqrth:
            gyroscopeIRWStd = deg2rad(_imuGyroscopeIRW) / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: gyroscopeIRWStd = {} [rad/s2/sqrt(s)]", nameId(), gyroscopeIRWStd.transpose());

        // compute velocity RW first
        IntegratedRandomWalkGyroscope_velocity += Eigen::Vector3d{
            _imuGyroscopeIRWRng.getRand_normalDist(0.0, gyroscopeIRWStd(0)),
            _imuGyroscopeIRWRng.getRand_normalDist(0.0, gyroscopeIRWStd(1)),
            _imuGyroscopeIRWRng.getRand_normalDist(0.0, gyroscopeIRWStd(2))
        };

        // then compute IRW
        IntegratedRandomWalkGyroscope += IntegratedRandomWalkGyroscope_velocity * dt;
    }

    // #########################################################################################################################################

    imuObs->p_acceleration += accelerometerBias_p
                              + Eigen::Vector3d{ _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(0)),
                                                 _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(1)),
                                                 _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(2)) }
                              + RandomWalkAccelerometer
                              + IntegratedRandomWalkAccelerometer;
    imuObs->p_angularRate += gyroscopeBias_p
                             + Eigen::Vector3d{ _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(0)),
                                                _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(1)),
                                                _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(2)) }
                             + RandomWalkGyroscope
                             + IntegratedRandomWalkGyroscope;

    return imuObs;
}

std::shared_ptr<NAV::ImuObsWDelta> NAV::ErrorModel::receiveImuObsWDelta(const std::shared_ptr<ImuObsWDelta>& imuObs)
{
    // Accelerometer Bias in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerBias_p = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerBiasUnit)
    {
    case ImuAccelerometerBiasUnits::m_s2:
        accelerometerBias_p = _imuAccelerometerBias_p;
        break;
    }
    LOG_DATA("{}: accelerometerBias_p = {} [m/s^2]", nameId(), accelerometerBias_p.transpose());

    // Gyroscope Bias in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeBias_p = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeBiasUnit)
    {
    case ImuGyroscopeBiasUnits::deg_s:
        gyroscopeBias_p = deg2rad(_imuGyroscopeBias_p);
        break;
    case ImuGyroscopeBiasUnits::rad_s:
        gyroscopeBias_p = _imuGyroscopeBias_p;
        break;
    }
    LOG_DATA("{}: gyroscopeBias_p = {} [rad/s]", nameId(), gyroscopeBias_p.transpose());

    // #########################################################################################################################################

    // Accelerometer Noise standard deviation in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerNoiseUnit)
    {
    case ImuAccelerometerNoiseUnits::m_s2:
        accelerometerNoiseStd = _imuAccelerometerNoise;
        break;
    case ImuAccelerometerNoiseUnits::m2_s4:
        accelerometerNoiseStd = _imuAccelerometerNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: accelerometerNoiseStd = {} [m/s^2]", nameId(), accelerometerNoiseStd.transpose());

    // Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeNoiseUnit)
    {
    case ImuGyroscopeNoiseUnits::rad_s:
        gyroscopeNoiseStd = _imuGyroscopeNoise;
        break;
    case ImuGyroscopeNoiseUnits::deg_s:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeNoise);
        break;
    case ImuGyroscopeNoiseUnits::rad2_s2:
        gyroscopeNoiseStd = _imuGyroscopeNoise.cwiseSqrt();
        break;
    case ImuGyroscopeNoiseUnits::deg2_s2:
        gyroscopeNoiseStd = deg2rad(_imuGyroscopeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: gyroscopeNoiseStd = {} [rad/s]", nameId(), gyroscopeNoiseStd.transpose());

    if (!_lastObservationTime.empty())
    {
        auto dt = std::chrono::duration<double>(imuObs->insTime - _lastObservationTime).count();

        // Accelerometer RW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d accelerometerRWStd = Eigen::Vector3d::Zero();
        switch (_imuAccelerometerRWUnit)
        {
        case ImuAccelerometerRWUnits::m_s2_sqrts:
            accelerometerRWStd = _imuAccelerometerRW * sqrt(dt);
            break;
        case ImuAccelerometerRWUnits::m_s2_sqrth:
            accelerometerRWStd = _imuAccelerometerRW / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: accelerometerRWStd = {} [m/s^2/sqrt(s)]", nameId(), accelerometerRWStd.transpose());

        RandomWalkAccelerometer += Eigen::Vector3d{
            _imuAccelerometerRWRng.getRand_normalDist(0.0, accelerometerRWStd(0)),
            _imuAccelerometerRWRng.getRand_normalDist(0.0, accelerometerRWStd(1)),
            _imuAccelerometerRWRng.getRand_normalDist(0.0, accelerometerRWStd(2))
        };

        // Accelerometer IRW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d accelerometerIRWStd = Eigen::Vector3d::Zero();
        switch (_imuAccelerometerIRWUnit)
        {
        case ImuAccelerometerIRWUnits::m_s3_sqrts:
            accelerometerIRWStd = _imuAccelerometerIRW * sqrt(dt);
            break;
        case ImuAccelerometerIRWUnits::m_s3_sqrth:
            accelerometerIRWStd = _imuAccelerometerIRW / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: accelerometerIRWStd = {} [m/s^3/sqrt(s)]", nameId(), accelerometerIRWStd.transpose());

        // compute velocity RW first
        IntegratedRandomWalkAccelerometer_velocity += Eigen::Vector3d{
            _imuAccelerometerIRWRng.getRand_normalDist(0.0, accelerometerIRWStd(0)),
            _imuAccelerometerIRWRng.getRand_normalDist(0.0, accelerometerIRWStd(1)),
            _imuAccelerometerIRWRng.getRand_normalDist(0.0, accelerometerIRWStd(2))
        };

        // then compute IRW
        IntegratedRandomWalkAccelerometer = IntegratedRandomWalkAccelerometer + IntegratedRandomWalkAccelerometer_velocity * dt;

        // Gyro RW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d gyroscopeRWStd = Eigen::Vector3d::Zero();
        switch (_imuGyroscopeRWUnit)
        {
        case ImuGyroscopeRWUnits::rad_s_sqrts:
            gyroscopeRWStd = _imuGyroscopeRW * sqrt(dt);
            break;
        case ImuGyroscopeRWUnits::rad_s_sqrth:
            gyroscopeRWStd = _imuGyroscopeRW / 60.0 * sqrt(dt);
            break;
        case ImuGyroscopeRWUnits::deg_s_sqrts:
            gyroscopeRWStd = deg2rad(_imuGyroscopeRW) * sqrt(dt);
            break;
        case ImuGyroscopeRWUnits::deg_s_sqrth:
            gyroscopeRWStd = deg2rad(_imuGyroscopeRW) / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: gyroscopeRWStd = {} [rad/s/sqrt(s)]", nameId(), gyroscopeRWStd.transpose());

        RandomWalkGyroscope += Eigen::Vector3d{
            _imuGyroscopeRWRng.getRand_normalDist(0.0, gyroscopeRWStd(0)),
            _imuGyroscopeRWRng.getRand_normalDist(0.0, gyroscopeRWStd(1)),
            _imuGyroscopeRWRng.getRand_normalDist(0.0, gyroscopeRWStd(2))
        };

        // Gyro RW standard deviation in platform frame coordinates [m/s^2]
        Eigen::Vector3d gyroscopeIRWStd = Eigen::Vector3d::Zero();
        switch (_imuGyroscopeIRWUnit)
        {
        case ImuGyroscopeIRWUnits::rad_s2_sqrts:
            gyroscopeIRWStd = _imuGyroscopeIRW * sqrt(dt);
            break;
        case ImuGyroscopeIRWUnits::rad_s2_sqrth:
            gyroscopeIRWStd = _imuGyroscopeIRW / 60.0 * sqrt(dt);
            break;
        case ImuGyroscopeIRWUnits::deg_s2_sqrts:
            gyroscopeIRWStd = deg2rad(_imuGyroscopeIRW) * sqrt(dt);
            break;
        case ImuGyroscopeIRWUnits::deg_s2_sqrth:
            gyroscopeIRWStd = deg2rad(_imuGyroscopeIRW) / 60.0 * sqrt(dt);
            break;
        }
        LOG_DATA("{}: gyroscopeIRWStd = {} [rad/s2/sqrt(s)]", nameId(), gyroscopeIRWStd.transpose());

        // compute velocity RW first
        IntegratedRandomWalkGyroscope_velocity += Eigen::Vector3d{
            _imuGyroscopeIRWRng.getRand_normalDist(0.0, gyroscopeIRWStd(0)),
            _imuGyroscopeIRWRng.getRand_normalDist(0.0, gyroscopeIRWStd(1)),
            _imuGyroscopeIRWRng.getRand_normalDist(0.0, gyroscopeIRWStd(2))
        };

        // then compute IRW
        IntegratedRandomWalkGyroscope += IntegratedRandomWalkGyroscope_velocity * dt;
    }

    // #########################################################################################################################################

    imuObs->dvel += accelerometerBias_p * imuObs->dtime
                    + Eigen::Vector3d{ _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(0) / std::sqrt(_imuObsWDeltaAverageWindow)),
                                       _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(1) / std::sqrt(_imuObsWDeltaAverageWindow)),
                                       _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(2) / std::sqrt(_imuObsWDeltaAverageWindow)) }
                    + imuObs->dtime * (RandomWalkAccelerometer + IntegratedRandomWalkAccelerometer);
    imuObs->dtheta += gyroscopeBias_p * imuObs->dtime
                      + Eigen::Vector3d{ _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(0) / std::sqrt(_imuObsWDeltaAverageWindow)),
                                         _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(1) / std::sqrt(_imuObsWDeltaAverageWindow)),
                                         _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(2) / std::sqrt(_imuObsWDeltaAverageWindow)) }
                      + imuObs->dtime * (RandomWalkGyroscope + IntegratedRandomWalkGyroscope);

    imuObs->p_acceleration += accelerometerBias_p
                              + Eigen::Vector3d{ _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(0)),
                                                 _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(1)),
                                                 _imuAccelerometerRng.getRand_normalDist(0.0, accelerometerNoiseStd(2)) }
                              + RandomWalkAccelerometer
                              + IntegratedRandomWalkAccelerometer;
    imuObs->p_angularRate += gyroscopeBias_p
                             + Eigen::Vector3d{ _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(0)),
                                                _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(1)),
                                                _imuGyroscopeRng.getRand_normalDist(0.0, gyroscopeNoiseStd(2)) }
                             + RandomWalkGyroscope
                             + IntegratedRandomWalkGyroscope;
    return imuObs;
}

std::shared_ptr<NAV::PosVelAtt> NAV::ErrorModel::receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt)
{
    // Position Bias in latLonAlt in [rad, rad, m]
    Eigen::Vector3d lla_positionBias = Eigen::Vector3d::Zero();
    switch (_positionBiasUnit)
    {
    case PositionBiasUnits::meter:
    {
        Eigen::Vector3d e_positionBias = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionBias;
        if (!e_positionBias.isZero())
        {
            lla_positionBias = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionBias) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionBiasUnits::rad_rad_m:
        lla_positionBias = _positionBias;
        break;
    case PositionBiasUnits::deg_deg_m:
        lla_positionBias = Eigen::Vector3d{ deg2rad(_positionBias(0)), deg2rad(_positionBias(1)), _positionBias(2) };
        break;
    }
    LOG_DATA("{}: lla_positionBias = {} [rad, rad, m]", nameId(), lla_positionBias.transpose());

    // Velocity bias in local-navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityBias = Eigen::Vector3d::Zero();
    switch (_velocityBiasUnit)
    {
    case VelocityBiasUnits::m_s:
        n_velocityBias = _velocityBias;
        break;
    }
    LOG_DATA("{}: n_velocityBias = {} [m/s]", nameId(), n_velocityBias.transpose());

    // Roll, pitch, yaw bias in [rad]
    Eigen::Vector3d attitudeBias = Eigen::Vector3d::Zero();
    switch (_attitudeBiasUnit)
    {
    case AttitudeBiasUnits::rad:
        attitudeBias = _attitudeBias;
        break;
    case AttitudeBiasUnits::deg:
        attitudeBias = deg2rad(_attitudeBias);
        break;
    }
    LOG_DATA("{}: attitudeBias = {} [rad]", nameId(), attitudeBias.transpose());

    // #########################################################################################################################################

    // Position Noise standard deviation in latitude, longitude and altitude [rad, rad, m]
    Eigen::Vector3d lla_positionNoiseStd = Eigen::Vector3d::Zero();
    switch (_positionNoiseUnit)
    {
    case PositionNoiseUnits::meter:
    {
        Eigen::Vector3d e_positionNoiseStd = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise;
        if (!e_positionNoiseStd.isZero())
        {
            lla_positionNoiseStd = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionNoiseStd) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionNoiseUnits::rad_rad_m:
        lla_positionNoiseStd = _positionNoise;
        break;
    case PositionNoiseUnits::deg_deg_m:
        lla_positionNoiseStd = deg2rad(_positionNoise);
        break;
    case PositionNoiseUnits::meter2:
    {
        Eigen::Vector3d e_positionNoiseStd = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise.cwiseSqrt();
        if (!e_positionNoiseStd.isZero())
        {
            lla_positionNoiseStd = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionNoiseStd) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionNoiseUnits::rad2_rad2_m2:
        lla_positionNoiseStd = _positionNoise.cwiseSqrt();
        break;
    case PositionNoiseUnits::deg2_deg2_m2:
        lla_positionNoiseStd = deg2rad(_positionNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: lla_positionNoiseStd = {} [rad, rad, m]", nameId(), lla_positionNoiseStd.transpose());

    // Velocity Noise standard deviation in local-navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityNoiseStd = Eigen::Vector3d::Zero();
    switch (_velocityNoiseUnit)
    {
    case VelocityNoiseUnits::m_s:
        n_velocityNoiseStd = _velocityNoise;
        break;
    case VelocityNoiseUnits::m2_s2:
        n_velocityNoiseStd = _velocityNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: n_velocityNoiseStd = {} [m/s]", nameId(), n_velocityNoiseStd.transpose());

    // Attitude Noise standard deviation in [rad]
    Eigen::Vector3d attitudeNoiseStd = Eigen::Vector3d::Zero();
    switch (_attitudeNoiseUnit)
    {
    case AttitudeNoiseUnits::rad:
        attitudeNoiseStd = _attitudeNoise;
        break;
    case AttitudeNoiseUnits::deg:
        attitudeNoiseStd = deg2rad(_attitudeNoise);
        break;
    case AttitudeNoiseUnits::rad2:
        attitudeNoiseStd = _attitudeNoise.cwiseSqrt();
        break;
    case AttitudeNoiseUnits::deg2:
        attitudeNoiseStd = deg2rad(_attitudeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: attitudeNoiseStd = {} [rad]", nameId(), attitudeNoiseStd.transpose());

    // #########################################################################################################################################

    posVelAtt->setState_n(posVelAtt->lla_position()
                              + lla_positionBias
                              + Eigen::Vector3d{ _positionRng.getRand_normalDist(0.0, lla_positionNoiseStd(0)),
                                                 _positionRng.getRand_normalDist(0.0, lla_positionNoiseStd(1)),
                                                 _positionRng.getRand_normalDist(0.0, lla_positionNoiseStd(2)) },
                          posVelAtt->n_velocity()
                              + n_velocityBias
                              + Eigen::Vector3d{ _velocityRng.getRand_normalDist(0.0, n_velocityNoiseStd(0)),
                                                 _velocityRng.getRand_normalDist(0.0, n_velocityNoiseStd(1)),
                                                 _velocityRng.getRand_normalDist(0.0, n_velocityNoiseStd(2)) },
                          trafo::n_Quat_b(posVelAtt->rollPitchYaw()
                                          + attitudeBias
                                          + Eigen::Vector3d{ _attitudeRng.getRand_normalDist(0.0, attitudeNoiseStd(0)),
                                                             _attitudeRng.getRand_normalDist(0.0, attitudeNoiseStd(1)),
                                                             _attitudeRng.getRand_normalDist(0.0, attitudeNoiseStd(2)) }));

    return posVelAtt;
}

std::shared_ptr<NAV::GnssObs> NAV::ErrorModel::receiveGnssObs(const std::shared_ptr<GnssObs>& gnssObs)
{
    LOG_DATA("{}: [{}] Simulating error on GnssObs", nameId(), gnssObs->insTime.toYMDHMS(GPST));
    double pseudorangeNoise{}; // [m]
    switch (_gui_pseudorangeNoiseUnit)
    {
    case PseudorangeNoiseUnits::meter:
        pseudorangeNoise = _gui_pseudorangeNoise;
        break;
    }
    double carrierPhaseNoise{}; // [m]
    switch (_gui_carrierPhaseNoiseUnit)
    {
    case CarrierPhaseNoiseUnits::meter:
        carrierPhaseNoise = _gui_carrierPhaseNoise;
        break;
    }
    double dopplerNoise{}; // [m/s]
    switch (_gui_dopplerNoiseUnit)
    {
    case DopplerNoiseUnits::m_s:
        dopplerNoise = _gui_dopplerNoise;
        break;
    }

    double dtCycleSlipSeconds{}; // [s]
    switch (_gui_cycleSlipFrequencyUnit)
    {
    case CycleSlipFrequencyUnits::per_day:
        dtCycleSlipSeconds = InsTimeUtil::SECONDS_PER_DAY / _gui_cycleSlipFrequency;
        break;
    case CycleSlipFrequencyUnits::per_hour:
        dtCycleSlipSeconds = InsTimeUtil::SECONDS_PER_HOUR / _gui_cycleSlipFrequency;
        break;
    case CycleSlipFrequencyUnits::per_minute:
        dtCycleSlipSeconds = InsTimeUtil::SECONDS_PER_MINUTE / _gui_cycleSlipFrequency;
        break;
    }
    auto dtCycleSlip = std::chrono::nanoseconds(static_cast<int64_t>(dtCycleSlipSeconds * 1e9));

    double cycleSlipDetectionProbability{}; // [0, 1]
    switch (_gui_cycleSlipDetectionProbabilityUnit)
    {
    case CycleSlipDetectionProbabilityUnits::percent:
        cycleSlipDetectionProbability = _gui_cycleSlipDetectionProbability / 100.0;
        break;
    }

    if (_cycleSlipWindowStartTime.empty()
        || gnssObs->insTime >= _cycleSlipWindowStartTime + dtCycleSlip)
    {
        _cycleSlipWindowStartTime = gnssObs->insTime;
        LOG_DATA("{}: [{}] Starting new cycle-slip window", nameId(), _cycleSlipWindowStartTime.toYMDHMS(GPST));
    }

    size_t nObs = 0;
    for (auto& obs : gnssObs->data)
    {
        if (obs.satSigId.freq() & _filterFreq
            && obs.satSigId.code & _filterCode)
        {
            nObs++;
        }
    }

    for (auto& obs : gnssObs->data)
    {
        if (obs.pseudorange) { obs.pseudorange.value().value += _pseudorangeRng.getRand_normalDist(0.0, pseudorangeNoise); }
        if (obs.doppler) { obs.doppler.value() += rangeRate2doppler(_dopplerRng.getRand_normalDist(0.0, dopplerNoise), obs.satSigId.freq(), 0); } // TODO: Add frequency number here for GLONASS

        if (obs.carrierPhase)
        {
            // ------------------------------------------- Noise ---------------------------------------------
            auto lambda = InsConst::C / obs.satSigId.freq().getFrequency(0); // wave-length [m] // TODO: Add frequency number here for GLONASS
            obs.carrierPhase.value().value += _carrierPhaseRng.getRand_normalDist(0.0, carrierPhaseNoise) / lambda;

            // ---------------------------------------- Cycle-slip -------------------------------------------

            if (obs.satSigId.freq() & _filterFreq                                                // GUI selected frequencies
                && obs.satSigId.code & _filterCode                                               // GUI selected codes
                && _gui_cycleSlipFrequency != 0.0                                                // 0 Frequency means disabled
                && !_lastObservationTime.empty()                                                 // Do not apply a cycle slip on the first message
                && (_cycleSlips.empty() || _cycleSlips.back().time < _cycleSlipWindowStartTime)) // In the current window, there was no cycle-slip yet
            {
                double dtMessage = 1.0 / _messageFrequency;                                                       // [s]
                double probabilityCycleSlip = dtMessage / (dtCycleSlipSeconds * static_cast<double>(nObs)) * 2.0; // Double chance, because often does not happen otherwise
                if (_cycleSlipRng.getRand_uniformRealDist(0.0, 1.0) <= probabilityCycleSlip
                    || (gnssObs->insTime >= _cycleSlipWindowStartTime + dtCycleSlip - std::chrono::nanoseconds(static_cast<int64_t>((dtMessage + 0.001) * 1e9)))) // Last message this window
                {
                    int newAmbiguity = 0;
                    int oldAmbiguity = !_ambiguities[obs.satSigId].empty()
                                           ? _ambiguities[obs.satSigId].back().second
                                           : static_cast<int>(_ambiguityRng.getRand_uniformIntDist(_gui_ambiguityLimits[0], _gui_ambiguityLimits[1]));
                    auto deltaAmbiguity = static_cast<int>(_cycleSlipRng.getRand_uniformIntDist(1, _gui_cycleSlipRange));
                    auto signAmbiguity = _cycleSlipRng.getRand_uniformIntDist(0, 1) == 0.0 ? 1 : -1;

                    if (_gui_ambiguityLimits[0] == _gui_ambiguityLimits[1]) // Ambiguities disabled
                    {
                        newAmbiguity = oldAmbiguity + signAmbiguity * deltaAmbiguity;
                    }
                    else if (oldAmbiguity == _gui_ambiguityLimits[0])
                    {
                        newAmbiguity = std::min(oldAmbiguity + deltaAmbiguity, _gui_ambiguityLimits[1]);
                    }
                    else if (oldAmbiguity == _gui_ambiguityLimits[1])
                    {
                        newAmbiguity = std::max(oldAmbiguity - deltaAmbiguity, _gui_ambiguityLimits[0]);
                    }
                    else
                    {
                        newAmbiguity = std::clamp(oldAmbiguity + signAmbiguity * deltaAmbiguity, _gui_ambiguityLimits[0], _gui_ambiguityLimits[1]);
                    }

                    _ambiguities[obs.satSigId].emplace_back(gnssObs->insTime, newAmbiguity);

                    if (_cycleSlipRng.getRand_uniformRealDist(0.0, 1.0) <= cycleSlipDetectionProbability)
                    {
                        obs.carrierPhase.value().LLI = 1;
                    }
                    _cycleSlips.push_back(CycleSlipInfo{ gnssObs->insTime, obs.satSigId, obs.carrierPhase.value().LLI != 0 });
                    LOG_DEBUG("{}: [{}] Simulating cycle-slip for satellite [{}] with LLI {}", nameId(), gnssObs->insTime.toYMDHMS(GPST),
                              obs.satSigId, obs.carrierPhase.value().LLI);
                }
            }

            // ----------------------------------------- Ambiguity -------------------------------------------
            if (!_ambiguities.contains(obs.satSigId) && _gui_ambiguityLimits[0] != _gui_ambiguityLimits[1])
            {
                _ambiguities[obs.satSigId].emplace_back(gnssObs->insTime, _ambiguityRng.getRand_uniformIntDist(_gui_ambiguityLimits[0], _gui_ambiguityLimits[1]));
            }
            if (_ambiguities.contains(obs.satSigId))
            {
                obs.carrierPhase.value().value += _ambiguities.at(obs.satSigId).back().second;
            }
        }
    }

    return gnssObs;
}