// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuFusion.hpp"

#include "util/Logger.hpp"

#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "NodeData/State/LcKfInsGnssErrors.hpp"

#include <imgui_internal.h>
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "util/Json.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

namespace NAV
{
/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const ImuFusion::PinData& data)
{
    j = json{
        // ---------------------------------------- Initialization -------------------------------------------
        { "initCovarianceAngularRate", data.initCovarianceAngularRate },
        { "initCovarianceAngularRateUnit", data.initCovarianceAngularRateUnit },
        { "initCovarianceAngularAcc", data.initCovarianceAngularAcc },
        { "initCovarianceAngularAccUnit", data.initCovarianceAngularAccUnit },
        { "initCovarianceAcceleration", data.initCovarianceAcceleration },
        { "initCovarianceAccelerationUnit", data.initCovarianceAccelerationUnit },
        { "initCovarianceJerk", data.initCovarianceJerk },
        { "initCovarianceJerkUnit", data.initCovarianceJerkUnit },
        { "initCovarianceBiasAngRate", data.initCovarianceBiasAngRate },
        { "initCovarianceBiasAngRateUnit", data.initCovarianceBiasAngRateUnit },
        { "initCovarianceBiasAcc", data.initCovarianceBiasAcc },
        { "initCovarianceBiasAccUnit", data.initCovarianceBiasAccUnit },
        // ----------------------------------------- Process Noise -------------------------------------------
        { "varAngularAccNoise", data.varAngularAccNoise },
        { "varAngularAccNoiseUnit", data.varAngularAccNoiseUnit },
        { "varJerkNoise", data.varJerkNoise },
        { "varJerkNoiseUnit", data.varJerkNoiseUnit },
        { "varBiasAccelerationNoise", data.varBiasAccelerationNoise },
        { "varBiasAccelerationNoiseUnit", data.varBiasAccelerationNoiseUnit },
        { "varBiasAngRateNoise", data.varBiasAngRateNoise },
        { "varBiasAngRateNoiseUnit", data.varBiasAngRateNoiseUnit },
        // --------------------------------------- Measurement Noise -----------------------------------------
        { "measurementUncertaintyAngularRateUnit", data.measurementUncertaintyAngularRateUnit },
        { "measurementUncertaintyAngularRate", data.measurementUncertaintyAngularRate },
        { "measurementUncertaintyAccelerationUnit", data.measurementUncertaintyAccelerationUnit },
        { "measurementUncertaintyAcceleration", data.measurementUncertaintyAcceleration },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, ImuFusion::PinData& data)
{
    // ------------------------------------------ Initialization ---------------------------------------------
    if (j.contains("initCovarianceAngularRate"))
    {
        j.at("initCovarianceAngularRate").get_to(data.initCovarianceAngularRate);
    }
    if (j.contains("initCovarianceAngularRateUnit"))
    {
        j.at("initCovarianceAngularRateUnit").get_to(data.initCovarianceAngularRateUnit);
    }
    if (j.contains("initCovarianceAngularAcc"))
    {
        j.at("initCovarianceAngularAcc").get_to(data.initCovarianceAngularAcc);
    }
    if (j.contains("initCovarianceAngularAccUnit"))
    {
        j.at("initCovarianceAngularAccUnit").get_to(data.initCovarianceAngularAccUnit);
    }
    if (j.contains("initCovarianceAcceleration"))
    {
        j.at("initCovarianceAcceleration").get_to(data.initCovarianceAcceleration);
    }
    if (j.contains("initCovarianceAccelerationUnit"))
    {
        j.at("initCovarianceAccelerationUnit").get_to(data.initCovarianceAccelerationUnit);
    }
    if (j.contains("initCovarianceJerk"))
    {
        j.at("initCovarianceJerk").get_to(data.initCovarianceJerk);
    }
    if (j.contains("initCovarianceJerkUnit"))
    {
        j.at("initCovarianceJerkUnit").get_to(data.initCovarianceJerkUnit);
    }
    if (j.contains("initCovarianceBiasAngRate"))
    {
        j.at("initCovarianceBiasAngRate").get_to(data.initCovarianceBiasAngRate);
    }
    if (j.contains("initCovarianceBiasAngRateUnit"))
    {
        j.at("initCovarianceBiasAngRateUnit").get_to(data.initCovarianceBiasAngRateUnit);
    }
    if (j.contains("initCovarianceBiasAccUnit"))
    {
        j.at("initCovarianceBiasAccUnit").get_to(data.initCovarianceBiasAccUnit);
    }
    if (j.contains("initCovarianceBiasAccUnit"))
    {
        j.at("initCovarianceBiasAccUnit").get_to(data.initCovarianceBiasAccUnit);
    }
    // ------------------------------------------- Process Noise ---------------------------------------------
    if (j.contains("varAngularAccNoise"))
    {
        j.at("varAngularAccNoise").get_to(data.varAngularAccNoise);
    }
    if (j.contains("varAngularAccNoiseUnit"))
    {
        j.at("varAngularAccNoiseUnit").get_to(data.varAngularAccNoiseUnit);
    }
    if (j.contains("varJerkNoise"))
    {
        j.at("varJerkNoise").get_to(data.varJerkNoise);
    }
    if (j.contains("varJerkNoiseUnit"))
    {
        j.at("varJerkNoiseUnit").get_to(data.varJerkNoiseUnit);
    }
    if (j.contains("varBiasAccelerationNoise"))
    {
        j.at("varBiasAccelerationNoise").get_to(data.varBiasAccelerationNoise);
    }
    if (j.contains("varBiasAccelerationNoiseUnit"))
    {
        j.at("varBiasAccelerationNoiseUnit").get_to(data.varBiasAccelerationNoiseUnit);
    }
    if (j.contains("varBiasAngRateNoise"))
    {
        j.at("varBiasAngRateNoise").get_to(data.varBiasAngRateNoise);
    }
    if (j.contains("varBiasAngRateNoiseUnit"))
    {
        j.at("varBiasAngRateNoiseUnit").get_to(data.varBiasAngRateNoiseUnit);
    }
    // ----------------------------------------- Measurement Noise -------------------------------------------
    if (j.contains("measurementUncertaintyAngularRate"))
    {
        j.at("measurementUncertaintyAngularRate").get_to(data.measurementUncertaintyAngularRate);
    }
    if (j.contains("measurementUncertaintyAngularRateUnit"))
    {
        j.at("measurementUncertaintyAngularRateUnit").get_to(data.measurementUncertaintyAngularRateUnit);
    }
    if (j.contains("measurementUncertaintyAcceleration"))
    {
        j.at("measurementUncertaintyAcceleration").get_to(data.measurementUncertaintyAcceleration);
    }
    if (j.contains("measurementUncertaintyAccelerationUnit"))
    {
        j.at("measurementUncertaintyAccelerationUnit").get_to(data.measurementUncertaintyAccelerationUnit);
    }
}
} // namespace NAV

NAV::ImuFusion::ImuFusion()
    : Imu(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 991, 1059 };

    nm::CreateOutputPin(this, "Combined ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
    updateNumberOfInputPins();
}

NAV::ImuFusion::~ImuFusion()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuFusion::typeStatic()
{
    return "ImuFusion";
}

std::string NAV::ImuFusion::type() const
{
    return typeStatic();
}

std::string NAV::ImuFusion::category()
{
    return "Data Processor";
}

void NAV::ImuFusion::guiConfig()
{
    constexpr float configWidth = 380.0F;
    constexpr float unitWidth = 150.0F;

    if (ImGui::BeginTable(fmt::format("Pin Settings##{}", size_t(id)).c_str(), inputPins.size() > 1 ? 2 : 1,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Pin");
        if (inputPins.size() > 2)
        {
            ImGui::TableSetupColumn("");
        }
        ImGui::TableHeadersRow();

        for (size_t pinIndex = 0; pinIndex < _pinData.size(); ++pinIndex)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); // Pin

            ImGui::TextUnformatted(fmt::format("{}", inputPins.at(pinIndex).name).c_str());

            if (inputPins.size() > 2) // Minimum # of pins for the fusion to make sense is two
            {
                ImGui::TableNextColumn(); // Delete
                if (!(pinIndex == 0))     // Don't delete Pin 1, it's the reference for all other sensor (biases) that follow
                {
                    if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                    {
                        nm::DeleteInputPin(inputPins.at(pinIndex));
                        nm::DeleteOutputPin(outputPins.at(pinIndex));
                        _pinData.erase(_pinData.begin() + static_cast<int64_t>(pinIndex - 1));
                        --_nInputPins;
                        flow::ApplyChanges();
                        updateNumberOfInputPins();
                    }
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Delete the pin");
                    }
                }
            }
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str()))
        {
            ++_nInputPins;
            LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nInputPins);
            flow::ApplyChanges();
            updateNumberOfInputPins();
        }

        ImGui::EndTable();
    }

    ImGui::Separator();

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);

    float columnWidth = 130.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemWidth(columnWidth);
    if (ImGui::InputDoubleL(fmt::format("Highest IMU sample rate##{}", size_t(id)).c_str(), &_imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.0f Hz"))
    {
        LOG_DEBUG("{}: imuFrequency changed to {}", nameId(), _imuFrequency);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("The inverse of this rate is used as the initial 'dt' for the Kalman Filter Prediction (Phi and Q).");

    if (ImGui::Checkbox(fmt::format("Rank check for Kalman filter matrices##{}", size_t(id)).c_str(), &_checkKalmanMatricesRanks))
    {
        LOG_DEBUG("{}: checkKalmanMatricesRanks {}", nameId(), _checkKalmanMatricesRanks);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Computationally intensive - only recommended for debugging.");

    ImGui::Separator();

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("P Error covariance matrix (init)##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate covariance ({})##{}",
                                                           _pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                   || _pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].initCovarianceAngularRate.data(), reinterpret_cast<int*>(&_pinData[0].initCovarianceAngularRateUnit), "(rad/s)^2\0"
                                                                                                                                                                                         "rad/s\0"
                                                                                                                                                                                         "(deg/s)^2\0"
                                                                                                                                                                                         "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAngularRate changed to {}", nameId(), _pinData[0].initCovarianceAngularRate);
            LOG_DEBUG("{}: AngRateVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceAngularRateUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular acceleration covariance ({})##{}",
                                                           _pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::rad2_s4
                                                                   || _pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::deg2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].initCovarianceAngularAcc.data(), reinterpret_cast<int*>(&_pinData[0].initCovarianceAngularAccUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                                       "rad/s^2\0"
                                                                                                                                                                                       "(deg^2)/(s^4)\0"
                                                                                                                                                                                       "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAngularAcc changed to {}", nameId(), _pinData[0].initCovarianceAngularAcc);
            LOG_DEBUG("{}: PinData::AngularAccVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceAngularAccUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration covariance ({})##{}",
                                                           _pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].initCovarianceAcceleration.data(), reinterpret_cast<int*>(&_pinData[0].initCovarianceAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                           "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAcceleration changed to {}", nameId(), _pinData[0].initCovarianceAcceleration);
            LOG_DEBUG("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceAccelerationUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk covariance ({})##{}",
                                                           _pinData[0].initCovarianceJerkUnit == PinData::JerkVarianceUnit::m2_s6
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].initCovarianceJerk.data(), reinterpret_cast<int*>(&_pinData[0].initCovarianceJerkUnit), "(m^2)/(s^6)\0"
                                                                                                                                                                           "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceJerk changed to {}", nameId(), _pinData[0].initCovarianceJerk);
            LOG_DEBUG("{}: PinData::JerkVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceJerkUnit);
            flow::ApplyChanges();
        }

        for (size_t pinIndex = 1; pinIndex < _nInputPins; ++pinIndex)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate bias covariance of sensor {} ({})##{}", pinIndex + 2,
                                                               _pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                       || _pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinData[pinIndex].initCovarianceBiasAngRate.data(), reinterpret_cast<int*>(&_pinData[pinIndex].initCovarianceBiasAngRateUnit), "(rad^2)/(s^2)\0"
                                                                                                                                                                                                           "rad/s\0"
                                                                                                                                                                                                           "(deg^2)/(s^2)\0"
                                                                                                                                                                                                           "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasAngRate changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAngRate);
                LOG_DEBUG("{}: PinData::AngRateVarianceUnit changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAngRateUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration bias covariance of sensor {} ({})##{}", pinIndex + 2,
                                                               _pinData[pinIndex].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinData[pinIndex].initCovarianceBiasAcc.data(), reinterpret_cast<int*>(&_pinData[pinIndex].initCovarianceBiasAccUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                   "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasAcc changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAcc);
                LOG_DEBUG("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAccUnit);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Q - System/Process noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the angular acceleration ({})##{}",
                                                           _pinData[0].varAngularAccNoiseUnit == PinData::AngularAccVarianceUnit::rad2_s4
                                                                   || _pinData[0].varAngularAccNoiseUnit == PinData::AngularAccVarianceUnit::deg2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].varAngularAccNoise.data(), reinterpret_cast<int*>(&_pinData[0].varAngularAccNoiseUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                           "rad/s^2\0"
                                                                                                                                                                           "(deg^2)/(s^4)\0"
                                                                                                                                                                           "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varAngularAccNoise changed to {}", nameId(), _pinData[0].varAngularAccNoise.transpose());
            LOG_DEBUG("{}: varAngularAccNoiseUnit changed to {}", nameId(), _pinData[0].varAngularAccNoiseUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the jerk ({})##{}",
                                                           _pinData[0].varJerkNoiseUnit == PinData::PinData::JerkVarianceUnit::m2_s6
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].varJerkNoise.data(), reinterpret_cast<int*>(&_pinData[0].varJerkNoiseUnit), "(m^2)/(s^6)\0"
                                                                                                                                                               "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varJerkNoise changed to {}", nameId(), _pinData[0].varJerkNoise.transpose());
            LOG_DEBUG("{}: varJerkNoiseUnit changed to {}", nameId(), _pinData[0].varJerkNoiseUnit);
            flow::ApplyChanges();
        }

        for (size_t pinIndex = 1; pinIndex < _nInputPins; ++pinIndex)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the bias of the angular rate of sensor {} ({})##{}", pinIndex + 2,
                                                               _pinData[pinIndex].varBiasAngRateNoiseUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                       || _pinData[pinIndex].varBiasAngRateNoiseUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                                   configWidth, unitWidth, _pinData[pinIndex].varBiasAngRateNoise.data(), reinterpret_cast<int*>(&_pinData[pinIndex].varBiasAngRateNoiseUnit), "(rad/s)^2\0"
                                                                                                                                                                                               "rad/s\0"
                                                                                                                                                                                               "(deg/s)^2\0"
                                                                                                                                                                                               "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: varBiasAngRateNoise changed to {}", nameId(), _pinData[pinIndex].varBiasAngRateNoise.transpose());
                LOG_DEBUG("{}: varBiasAngRateNoiseUnit changed to {}", nameId(), _pinData[pinIndex].varBiasAngRateNoiseUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the bias of the acceleration of sensor {} ({})##{}", pinIndex + 2,
                                                               _pinData[pinIndex].varBiasAccelerationNoiseUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                                   configWidth, unitWidth, _pinData[pinIndex].varBiasAccelerationNoise.data(), reinterpret_cast<int*>(&_pinData[pinIndex].varBiasAccelerationNoiseUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                         "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: varBiasAccelerationNoise changed to {}", nameId(), _pinData[pinIndex].varBiasAccelerationNoise.transpose());
                LOG_DEBUG("{}: varBiasAccelerationNoiseUnit changed to {}", nameId(), _pinData[pinIndex].varBiasAccelerationNoiseUnit);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        for (size_t pinIndex = 0; pinIndex < _nInputPins; ++pinIndex)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Measurement uncertainty of the angular rate of sensor {} ({})##{}", pinIndex + 1,
                                                               _pinData[pinIndex].measurementUncertaintyAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                       || _pinData[pinIndex].measurementUncertaintyAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinData[pinIndex].measurementUncertaintyAngularRate.data(), reinterpret_cast<int*>(&_pinData[pinIndex].measurementUncertaintyAngularRateUnit), "(rad/s)^2\0"
                                                                                                                                                                                                                           "rad/s\0"
                                                                                                                                                                                                                           "(deg/s)^2\0"
                                                                                                                                                                                                                           "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdevAngularAcc changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAngularRate.transpose());
                LOG_DEBUG("{}: stdevAngularAccUnit changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAngularRateUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Measurement uncertainty of the acceleration of sensor {} ({})##{}", pinIndex + 1,
                                                               _pinData[pinIndex].measurementUncertaintyAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinData[pinIndex].measurementUncertaintyAcceleration.data(), reinterpret_cast<int*>(&_pinData[pinIndex].measurementUncertaintyAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                                             "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdevJerk changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAcceleration.transpose());
                LOG_DEBUG("{}: stdevJerkUnit changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAccelerationUnit);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::ImuFusion::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["checkKalmanMatricesRanks"] = _checkKalmanMatricesRanks;

    j["nInputPins"] = _nInputPins;
    j["imuRotations_accel"] = _imuRotations_accel;
    j["imuRotations_gyro"] = _imuRotations_gyro;
    j["imuFrequency"] = _imuFrequency;
    j["designMatrixInitialized"] = _designMatrixInitialized;
    j["numStates"] = _numStates;
    j["numMeasurements"] = _numMeasurements;
    j["pinData"] = _pinData;

    return j;
}

void NAV::ImuFusion::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("checkKalmanMatricesRanks"))
    {
        j.at("checkKalmanMatricesRanks").get_to(_checkKalmanMatricesRanks);
    }
    if (j.contains("nInputPins"))
    {
        j.at("nInputPins").get_to(_nInputPins);
        updateNumberOfInputPins();
    }
    if (j.contains("imuRotations_accel"))
    {
        j.at("imuRotations_accel").get_to(_imuRotations_accel);
    }
    if (j.contains("imuRotations_gyro"))
    {
        j.at("imuRotations_gyro").get_to(_imuRotations_gyro);
    }
    if (j.contains("imuFrequency"))
    {
        j.at("imuFrequency").get_to(_imuFrequency);
    }
    if (j.contains("designMatrixInitialized"))
    {
        j.at("designMatrixInitialized").get_to(_designMatrixInitialized);
    }
    if (j.contains("numStates"))
    {
        j.at("numStates").get_to(_numStates);
    }
    if (j.contains("numMeasurements"))
    {
        j.at("numMeasurements").get_to(_numMeasurements);
    }
    if (j.contains("pinData"))
    {
        j.at("pinData").get_to(_pinData);
    }
}

bool NAV::ImuFusion::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _numStates = _numStatesEst + static_cast<uint8_t>((_nInputPins - 1) * _numStatesPerPin);

    _kalmanFilter = KalmanFilter{ _numStates, _numMeasurements };

    _kalmanFilter.setZero();
    _imuRotations_accel.clear();
    _imuRotations_gyro.clear();
    _biasCovariances.clear();
    _processNoiseVariances.clear();
    _measurementNoiseVariances.clear();
    _imuPosSet = false;

    _latestTimestamp = InsTime{};

    initializeMountingAngles();
    initializeKalmanFilter();

    LOG_DEBUG("ImuFusion initialized");

    return true;
}

void NAV::ImuFusion::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::ImuFusion::updateNumberOfInputPins()
{
    while (inputPins.size() < _nInputPins)
    {
        nm::CreateInputPin(this, fmt::format("Pin {}", inputPins.size() + 1).c_str(), Pin::Type::Flow,
                           { NAV::ImuObs::type() }, &ImuFusion::recvSignal);
        _pinData.emplace_back();
        if (outputPins.size() < _nInputPins)
        {
            nm::CreateOutputPin(this, fmt::format("ImuBiases {}1", outputPins.size() + 1).c_str(), Pin::Type::Flow, { NAV::LcKfInsGnssErrors::type() });
        }
    }
    while (inputPins.size() > _nInputPins) // TODO: while loop still necessary here? guiConfig also deletes pins
    {
        nm::DeleteInputPin(inputPins.back());
        nm::DeleteOutputPin(outputPins.back());
        _pinData.pop_back();
    }
    _pinData.resize(_nInputPins);
    initializeMountingAngles();
}

void NAV::ImuFusion::initializeMountingAngles()
{
    _imuRotations_accel.resize(_nInputPins);
    _imuRotations_gyro.resize(_nInputPins);
    for (size_t i = 0; i < _nInputPins; ++i)
    {
        _imuRotations_accel[i] = Eigen::Matrix3d::Zero();
        _imuRotations_gyro[i] = Eigen::Matrix3d::Zero();

        // Assigning nan for an efficient check during runtime, whether mounting angles have been read for sensor i
        _imuRotations_accel[i](0, 0) = std::nan("");
        _imuRotations_gyro[i](0, 0) = std::nan("");
    }
}

void NAV::ImuFusion::initializeKalmanFilter()
{
    LOG_TRACE("{}: called", nameId());

    _designMatrixInitialized = false;

    for (size_t pinIndex = 0; pinIndex < _pinData.size(); pinIndex++)
    {
        if (!inputPins.at(pinIndex).isPinLinked())
        {
            LOG_INFO("Fewer links than input pins - Consider deleting pins that are not connected to limit KF matrices to the necessary size.");
        }
    }

    // ------------------------------------------------------ Error covariance matrix P --------------------------------------------------------

    // Initial Covariance of the angular rate in [rad²/s²]
    Eigen::Vector3d variance_angularRate = Eigen::Vector3d::Zero();
    if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
    {
        variance_angularRate = _pinData[0].initCovarianceAngularRate;
    }
    else if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
    {
        variance_angularRate = deg2rad(_pinData[0].initCovarianceAngularRate);
    }
    else if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad_s)
    {
        variance_angularRate = _pinData[0].initCovarianceAngularRate.array().pow(2);
    }
    else if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg_s)
    {
        variance_angularRate = deg2rad(_pinData[0].initCovarianceAngularRate).array().pow(2);
    }

    // Initial Covariance of the angular acceleration in [(rad^2)/(s^4)]
    Eigen::Vector3d variance_angularAcceleration = Eigen::Vector3d::Zero();
    if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::rad2_s4)
    {
        variance_angularAcceleration = _pinData[0].initCovarianceAngularAcc;
    }
    else if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::deg2_s4)
    {
        variance_angularAcceleration = deg2rad(_pinData[0].initCovarianceAngularAcc);
    }
    else if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::rad_s2)
    {
        variance_angularAcceleration = _pinData[0].initCovarianceAngularAcc.array().pow(2);
    }
    else if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::deg_s2)
    {
        variance_angularAcceleration = deg2rad(_pinData[0].initCovarianceAngularAcc).array().pow(2);
    }

    // Initial Covariance of the acceleration in [(m^2)/(s^4)]
    Eigen::Vector3d variance_acceleration = Eigen::Vector3d::Zero();
    if (_pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4)
    {
        variance_acceleration = _pinData[0].initCovarianceAcceleration;
    }
    else if (_pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m_s2)
    {
        variance_acceleration = _pinData[0].initCovarianceAcceleration.array().pow(2);
    }

    // Initial Covariance of the jerk in [(m^2)/(s^6)]
    Eigen::Vector3d variance_jerk = Eigen::Vector3d::Zero();
    if (_pinData[0].initCovarianceJerkUnit == PinData::JerkVarianceUnit::m2_s6)
    {
        variance_jerk = _pinData[0].initCovarianceJerk;
    }
    else if (_pinData[0].initCovarianceJerkUnit == PinData::JerkVarianceUnit::m_s3)
    {
        variance_jerk = _pinData[0].initCovarianceJerk.array().pow(2);
    }

    // Initial Covariance of the bias of the angular acceleration in [(rad^2)/(s^4)]
    _biasCovariances.resize(2 * _nInputPins);
    for (size_t pinIndex = 0; pinIndex < _nInputPins - 1UL; ++pinIndex)
    {
        if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
        {
            _biasCovariances[2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAngRate;
        }
        else if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
        {
            _biasCovariances[2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].initCovarianceBiasAngRate);
        }
        else if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad_s)
        {
            _biasCovariances[2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAngRate.array().pow(2);
        }
        else if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg_s)
        {
            _biasCovariances[2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].initCovarianceBiasAngRate).array().pow(2);
        }

        // Initial Covariance of the bias of the jerk in [(m^2)/(s^6)]
        if (_pinData[pinIndex].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4)
        {
            _biasCovariances[1 + 2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAcc;
        }
        else if (_pinData[pinIndex].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m_s2)
        {
            _biasCovariances[1 + 2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAcc.array().pow(2);
        }
    }

    // ------------------------------------------------------- Process noise matrix Q ----------------------------------------------------------
    _processNoiseVariances.resize(2 * _nInputPins);

    // 𝜎_AngAcc Standard deviation of the noise on the angular acceleration state [rad/s²]
    switch (_pinData[0].varAngularAccNoiseUnit)
    {
    case PinData::AngularAccVarianceUnit::rad2_s4:
        _processNoiseVariances[0] = _pinData[0].varAngularAccNoise;
        break;
    case PinData::AngularAccVarianceUnit::deg2_s4:
        _processNoiseVariances[0] = deg2rad(_pinData[0].varAngularAccNoise);
        break;
    case PinData::AngularAccVarianceUnit::deg_s2:
        _processNoiseVariances[0] = deg2rad(_pinData[0].varAngularAccNoise).array().pow(2);
        break;
    case PinData::AngularAccVarianceUnit::rad_s2:
        _processNoiseVariances[0] = _pinData[0].varAngularAccNoise.array().pow(2);
        break;
    }

    // 𝜎_jerk Standard deviation of the noise on the jerk state [m/s³]
    switch (_pinData[0].varJerkNoiseUnit)
    {
    case PinData::JerkVarianceUnit::m2_s6:
        _processNoiseVariances[1] = _pinData[0].varJerkNoise;
        break;
    case PinData::JerkVarianceUnit::m_s3:
        _processNoiseVariances[1] = _pinData[0].varJerkNoise.array().pow(2);
        break;
    }

    for (size_t pinIndex = 0; pinIndex < _nInputPins - 1; ++pinIndex)
    {
        // 𝜎_biasAngRate Standard deviation of the bias on the angular rate state [rad/s²]
        switch (_pinData[1 + pinIndex].varBiasAngRateNoiseUnit)
        {
        case PinData::AngRateVarianceUnit::rad2_s2:
            _processNoiseVariances[2 + 2 * pinIndex] = _pinData[1 + pinIndex].varBiasAngRateNoise;
            break;
        case PinData::AngRateVarianceUnit::deg2_s2:
            _processNoiseVariances[2 + 2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].varBiasAngRateNoise);
            break;
        case PinData::AngRateVarianceUnit::deg_s:
            _processNoiseVariances[2 + 2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].varBiasAngRateNoise).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::rad_s:
            _processNoiseVariances[2 + 2 * pinIndex] = _pinData[1 + pinIndex].varBiasAngRateNoise.array().pow(2);
            break;
        }

        // 𝜎_biasAcceleration Standard deviation of the noise on the acceleration state [m/s³]
        switch (_pinData[pinIndex].varBiasAccelerationNoiseUnit)
        {
        case PinData::AccelerationVarianceUnit::m2_s4:
            _processNoiseVariances[3 + 2 * pinIndex] = _pinData[1 + pinIndex].varBiasAccelerationNoise;
            break;
        case PinData::AccelerationVarianceUnit::m_s2:
            _processNoiseVariances[3 + 2 * pinIndex] = _pinData[1 + pinIndex].varBiasAccelerationNoise.array().pow(2);
            break;
        }
    }

    // -------------------------------------------------- Measurement uncertainty matrix R -----------------------------------------------------
    _measurementNoiseVariances.resize(2 * _nInputPins);

    for (size_t pinIndex = 0; pinIndex < _nInputPins; ++pinIndex)
    {
        // Measurement uncertainty for the angular rate (Variance σ²) in [(rad/s)^2, (rad/s)^2, (rad/s)^2]
        switch (_pinData[pinIndex].measurementUncertaintyAngularRateUnit)
        {
        case PinData::AngRateVarianceUnit::rad_s:
            _measurementNoiseVariances[2 * pinIndex] = (_pinData[pinIndex].measurementUncertaintyAngularRate).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::deg_s:
            _measurementNoiseVariances[2 * pinIndex] = (deg2rad(_pinData[pinIndex].measurementUncertaintyAngularRate)).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::rad2_s2:
            _measurementNoiseVariances[2 * pinIndex] = _pinData[pinIndex].measurementUncertaintyAngularRate;
            break;
        case PinData::AngRateVarianceUnit::deg2_s2:
            _measurementNoiseVariances[2 * pinIndex] = deg2rad((_pinData[pinIndex].measurementUncertaintyAngularRate).cwiseSqrt()).array().pow(2);
            break;
        }

        // Measurement uncertainty for the acceleration (Variance σ²) in [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        switch (_pinData[pinIndex].measurementUncertaintyAccelerationUnit)
        {
        case PinData::AccelerationVarianceUnit::m2_s4:
            _measurementNoiseVariances[1 + 2 * pinIndex] = _pinData[pinIndex].measurementUncertaintyAcceleration;
            break;
        case PinData::AccelerationVarianceUnit::m_s2:
            _measurementNoiseVariances[1 + 2 * pinIndex] = (_pinData[pinIndex].measurementUncertaintyAcceleration).array().pow(2);
            break;
        }
    }

    auto dtInit = 1.0 / _imuFrequency;

    // --------------------------------------------------------- KF Initializations ------------------------------------------------------------
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(variance_angularRate, variance_angularAcceleration, variance_acceleration, variance_jerk);
    LOG_DATA("kalmanFilter.P =\n{}", _kalmanFilter.P);
    _kalmanFilter.Phi = initialStateTransitionMatrix_Phi(dtInit);
    LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
    processNoiseMatrix_Q(_kalmanFilter.Q, dtInit);
    LOG_DATA("kalmanFilter.Q =\n{}", _kalmanFilter.Q);
}

void NAV::ImuFusion::recvSignal(NAV::InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto imuObs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    if (imuObs->insTime.empty() && !imuObs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime/timeSinceStartup)", nameId());
        return;
    }

    if (_latestTimestamp.empty())
    {
        // Initial time step for KF prediction
        InsTime dt_init = InsTime{ 0, 0, 0, 0, 0, 1.0 / _imuFrequency };
        _latestTimestamp = InsTime{ 0, 0, 0, 0, 0, (imuObs->insTime - dt_init).count() };
    }

    // Predict states over the time difference between the latest signal and the one before
    auto dt = static_cast<double>((imuObs->insTime - _latestTimestamp).count());
    _latestTimestamp = imuObs->insTime;
    LOG_DATA("dt = {}", dt);

    stateTransitionMatrix_Phi(_kalmanFilter.Phi, dt);
    LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
    processNoiseMatrix_Q(_kalmanFilter.Q, dt);
    LOG_DATA("kalmanFilter.Q =\n{}", _kalmanFilter.Q);

    if (_checkKalmanMatricesRanks)
    {
        if (inputPins.at(_pinData.size() - 1).isPinLinked())
        {
            auto rank = _kalmanFilter.P.fullPivLu().rank();
            if (rank != _kalmanFilter.P.rows())
            {
                LOG_WARN("{}: P.rank = {}", nameId(), rank);
            }
        }
    }

    // Read sensor rotation info from 'imuObs'
    if (std::isnan(_imuRotations_accel[pinIdx](0, 0)))
    {
        // Rotation matrix of the accelerometer platform to body frame
        auto DCM_accel = imuObs->imuPos.b_quatAccel_p().toRotationMatrix();

        _imuRotations_accel[pinIdx] = DCM_accel;
    }
    if (std::isnan(_imuRotations_gyro[pinIdx](0, 0)))
    {
        // Rotation matrix of the gyro platform to body frame
        auto DCM_gyro = imuObs->imuPos.b_quatGyro_p().toRotationMatrix();

        _imuRotations_gyro[pinIdx] = DCM_gyro;
    }

    // Initialize H with mounting angles (DCM) of the sensor that provided the latest measurement
    auto DCM_accel = _imuRotations_accel.at(pinIdx);
    LOG_DATA("DCM_accel =\n{}", DCM_accel);
    auto DCM_gyro = _imuRotations_gyro.at(pinIdx);
    LOG_DATA("DCM_gyro =\n{}", DCM_gyro);

    // Initialize '_imuPos' of the combined solution - that of the reference sensor
    if (!_imuPosSet && pinIdx == 0)
    {
        this->_imuPos = imuObs->imuPos;
        _imuPosSet = true;
    }

    _kalmanFilter.H = designMatrix_H(DCM_accel, DCM_gyro, pinIdx);
    LOG_DATA("kalmanFilter.H =\n", _kalmanFilter.H);

    measurementNoiseMatrix_R(_kalmanFilter.R, pinIdx);
    LOG_DATA("{}: kalmanFilter.R =\n{}", nameId(), _kalmanFilter.R);

    if (_checkKalmanMatricesRanks)
    {
        auto rank = (_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R).fullPivLu().rank();
        if (rank != _kalmanFilter.H.rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }
    }

    combineSignals(imuObs);
}

void NAV::ImuFusion::combineSignals(const std::shared_ptr<const ImuObs>& imuObs)
{
    LOG_DATA("{}: called", nameId());

    auto imuObsFiltered = std::make_shared<ImuObs>(this->_imuPos);
    auto imuRelativeBiases = std::make_shared<LcKfInsGnssErrors>();

    LOG_DATA("Estimated state before prediction: x =\n{}", _kalmanFilter.x);

    _kalmanFilter.predict();

    _kalmanFilter.z.block<3, 1>(0, 0) = imuObs->gyroUncompXYZ.value();
    _kalmanFilter.z.block<3, 1>(3, 0) = imuObs->accelUncompXYZ.value();

    LOG_DATA("Measurements z =\n{}", _kalmanFilter.z);

    _kalmanFilter.correct();
    LOG_DATA("Estimated state after correction: x =\n{}", _kalmanFilter.x);

    if (_checkKalmanMatricesRanks)
    {
        auto rank = (_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R).fullPivLu().rank();
        if (rank != _kalmanFilter.H.rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }

        rank = _kalmanFilter.K.fullPivLu().rank();
        if (rank != _kalmanFilter.K.cols())
        {
            LOG_WARN("{}: K.rank = {}", nameId(), rank);
        }
    }
    if (_checkKalmanMatricesRanks)
    {
        if (inputPins.at(_pinData.size() - 1).isPinLinked())
        {
            auto rank = _kalmanFilter.P.fullPivLu().rank();
            if (rank != _kalmanFilter.P.rows())
            {
                LOG_WARN("{}: P.rank = {}", nameId(), rank);
                LOG_DATA("kalmanFilter.P =\n{}", _kalmanFilter.P);
            }
        }
    }

    // Construct imuObs
    imuObsFiltered->insTime = imuObs->insTime;
    imuObsFiltered->accelUncompXYZ.emplace(_kalmanFilter.x(6, 0), _kalmanFilter.x(7, 0), _kalmanFilter.x(8, 0));
    imuObsFiltered->gyroUncompXYZ.emplace(_kalmanFilter.x(0, 0), _kalmanFilter.x(1, 0), _kalmanFilter.x(2, 0));

    invokeCallbacks(OUTPUT_PORT_INDEX_COMBINED_SIGNAL, imuObsFiltered);

    imuRelativeBiases->insTime = imuObs->insTime;
    for (size_t OUTPUT_PORT_INDEX_BIAS = 1; OUTPUT_PORT_INDEX_BIAS < _nInputPins; ++OUTPUT_PORT_INDEX_BIAS)
    {
        auto biasIndex = _numStatesEst + static_cast<uint8_t>((OUTPUT_PORT_INDEX_BIAS - 1) * _numStatesPerPin);

        imuRelativeBiases->b_biasGyro = { _kalmanFilter.x(biasIndex, 0), _kalmanFilter.x(biasIndex + 1, 0), _kalmanFilter.x(biasIndex + 2, 0) };
        imuRelativeBiases->b_biasAccel = { _kalmanFilter.x(biasIndex + 3, 0), _kalmanFilter.x(biasIndex + 4, 0), _kalmanFilter.x(biasIndex + 5, 0) };

        invokeCallbacks(OUTPUT_PORT_INDEX_BIAS, imuRelativeBiases);
    }
}

// #########################################################################################################################################
//                                                         Kalman Filter Matrices
// #########################################################################################################################################

Eigen::MatrixXd NAV::ImuFusion::initialStateTransitionMatrix_Phi(double dt) const
{
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(_numStates, _numStates);

    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk

    return Phi;
}

void NAV::ImuFusion::stateTransitionMatrix_Phi(Eigen::MatrixXd& Phi, double dt)
{
    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk
}

Eigen::MatrixXd NAV::ImuFusion::initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& varAngRate,
                                                                const Eigen::Vector3d& varAngAcc,
                                                                const Eigen::Vector3d& varAcc,
                                                                const Eigen::Vector3d& varJerk) const
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(_numStates, _numStates);

    P.block<3, 3>(0, 0).diagonal() = varAngRate;
    P.block<3, 3>(3, 3).diagonal() = varAngAcc;
    P.block<3, 3>(6, 6).diagonal() = varAcc;
    P.block<3, 3>(9, 9).diagonal() = varJerk;

    for (uint32_t i = 12; i < _numStates - 1UL; i += 6)
    {
        size_t j = (i - 12) / 3 + 2; // access 2 bias variances for each sensor from the third element onwards
        P.block<3, 3>(i, i).diagonal() = _biasCovariances.at(j);
        P.block<3, 3>(i + 3, i + 3).diagonal() = _biasCovariances.at(j + 1);
    }

    return P;
}

void NAV::ImuFusion::processNoiseMatrix_Q(Eigen::MatrixXd& Q, double dt) const
{
    // Integrated Random Walk of the angular rate
    Q.block<3, 3>(0, 0).diagonal() = _processNoiseVariances.at(0) / 3. * std::pow(dt, 3);
    Q.block<3, 3>(0, 3).diagonal() = _processNoiseVariances.at(0) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(3, 0).diagonal() = _processNoiseVariances.at(0) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(3, 3).diagonal() = _processNoiseVariances.at(0) * dt;

    // Integrated Random Walk of the acceleration
    Q.block<3, 3>(6, 6).diagonal() = _processNoiseVariances.at(1) / 3. * std::pow(dt, 3);
    Q.block<3, 3>(6, 9).diagonal() = _processNoiseVariances.at(1) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(9, 6).diagonal() = _processNoiseVariances.at(1) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(9, 9).diagonal() = _processNoiseVariances.at(1) * dt;

    // Random Walk of the bias states
    for (uint32_t i = 12; i < _numStates; i += 6)
    {
        size_t j = (i - 12) / 3 + 2;                                                    // access 2 bias variances for each sensor from the third element onwards
        Q.block<3, 3>(i, i).diagonal() = _processNoiseVariances.at(j) * dt;             // variance for the process noise of the angular rate
        Q.block<3, 3>(i + 3, i + 3).diagonal() = _processNoiseVariances.at(j + 1) * dt; // variance for the process noise of the acceleration
    }
}

Eigen::MatrixXd NAV::ImuFusion::designMatrix_H(const Eigen::Matrix3d& DCM_accel, const Eigen::Matrix3d& DCM_gyro, size_t pinIndex) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(_numMeasurements, _numStates);

    // Mounting angles of sensor with latest measurement
    H.block<3, 3>(0, 0) = DCM_accel.transpose(); // Inverse rotation for angular rate
    H.block<3, 3>(3, 6) = DCM_gyro.transpose();  // Inverse rotation for acceleration

    // Mapping of bias states on sensor with the latest measurement
    if (pinIndex > 0)
    {
        auto stateIndex = static_cast<uint8_t>(_numStatesEst + _numStatesPerPin * (pinIndex - 1));
        LOG_DATA("stateIndex = {}", stateIndex);

        H.block<6, 6>(0, stateIndex) = Eigen::MatrixXd::Identity(6, 6);
    }

    LOG_DATA("H =\n{}", H);

    return H;
}

Eigen::MatrixXd NAV::ImuFusion::measurementNoiseMatrix_R_adaptive(double alpha, const Eigen::MatrixXd& R, const Eigen::VectorXd& e, const Eigen::MatrixXd& H, const Eigen::MatrixXd& P)
{
    return alpha * R + (1.0 - alpha) * (e * e.transpose() + H * P * H.transpose());
}

void NAV::ImuFusion::measurementNoiseMatrix_R(Eigen::MatrixXd& R, size_t pinIndex) const
{
    R.block<3, 3>(0, 0).diagonal() = _measurementNoiseVariances.at(2 * pinIndex);
    R.block<3, 3>(3, 3).diagonal() = _measurementNoiseVariances.at(2 * pinIndex + 1);
}