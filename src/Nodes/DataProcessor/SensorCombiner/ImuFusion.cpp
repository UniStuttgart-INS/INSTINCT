// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuFusion.hpp"

#include "util/Logger.hpp"

#include <numeric>
#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "Navigation/INS/SensorCombiner/IRWKF/IRWKF.hpp"
#include "Navigation/INS/SensorCombiner/BsplineKF/BsplineKF.hpp"
#include "Navigation/INS/SensorCombiner/BsplineKF/QuadraticBsplines.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"

#include <imgui_internal.h>
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
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
void to_json(json& j, const PinData& data)
{
    j = json{
        // ---------------------------------------- Initialization -------------------------------------------
        { "initAngularRateBias", data.initAngularRateBias },
        { "initAngularRateBiasUnit", data.initAngularRateBiasUnit },
        { "initAccelerationBias", data.initAccelerationBias },
        { "initAccelerationBiasUnit", data.initAccelerationBiasUnit },
        { "initCovarianceAngularRate", data.initCovarianceAngularRate },
        { "initCovarianceAngularRateUnit", data.initCovarianceAngularRateUnit },
        { "initCovarianceAcceleration", data.initCovarianceAcceleration },
        { "initCovarianceAccelerationUnit", data.initCovarianceAccelerationUnit },
        { "initCovarianceBiasAngRate", data.initCovarianceBiasAngRate },
        { "initCovarianceBiasAngRateUnit", data.initCovarianceBiasAngRateUnit },
        { "initCovarianceBiasAcc", data.initCovarianceBiasAcc },
        { "initCovarianceBiasAccUnit", data.initCovarianceBiasAccUnit },
        // ----------------------------------------- Process Noise -------------------------------------------
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
void from_json(const json& j, PinData& data)
{
    // ------------------------------------------ Initialization ---------------------------------------------
    if (j.contains("initAngularRateBias"))
    {
        j.at("initAngularRateBias").get_to(data.initAngularRateBias);
    }
    if (j.contains("initAngularRateBiasUnit"))
    {
        j.at("initAngularRateBiasUnit").get_to(data.initAngularRateBiasUnit);
    }
    if (j.contains("initAccelerationBias"))
    {
        j.at("initAccelerationBias").get_to(data.initAccelerationBias);
    }
    if (j.contains("initAccelerationBiasUnit"))
    {
        j.at("initAccelerationBiasUnit").get_to(data.initAccelerationBiasUnit);
    }
    if (j.contains("initCovarianceAngularRate"))
    {
        j.at("initCovarianceAngularRate").get_to(data.initCovarianceAngularRate);
    }
    if (j.contains("initCovarianceAngularRateUnit"))
    {
        j.at("initCovarianceAngularRateUnit").get_to(data.initCovarianceAngularRateUnit);
    }
    if (j.contains("initCovarianceAcceleration"))
    {
        j.at("initCovarianceAcceleration").get_to(data.initCovarianceAcceleration);
    }
    if (j.contains("initCovarianceAccelerationUnit"))
    {
        j.at("initCovarianceAccelerationUnit").get_to(data.initCovarianceAccelerationUnit);
    }
    if (j.contains("initCovarianceBiasAngRate"))
    {
        j.at("initCovarianceBiasAngRate").get_to(data.initCovarianceBiasAngRate);
    }
    if (j.contains("initCovarianceBiasAngRateUnit"))
    {
        j.at("initCovarianceBiasAngRateUnit").get_to(data.initCovarianceBiasAngRateUnit);
    }
    if (j.contains("initCovarianceBiasAcc"))
    {
        j.at("initCovarianceBiasAcc").get_to(data.initCovarianceBiasAcc);
    }
    if (j.contains("initCovarianceBiasAccUnit"))
    {
        j.at("initCovarianceBiasAccUnit").get_to(data.initCovarianceBiasAccUnit);
    }
    // ------------------------------------------- Process Noise ---------------------------------------------
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

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const PinDataIRWKF& data)
{
    j = json{
        // ---------------------------------------- Initialization -------------------------------------------
        { "initAngularRate", data.initAngularRate },
        { "initAngularRateUnit", data.initAngularRateUnit },
        { "initAcceleration", data.initAcceleration },
        { "initAccelerationUnit", data.initAccelerationUnit },
        { "initAngularAcc", data.initAngularAcc },
        { "initAngularAccUnit", data.initAngularAccUnit },
        { "initJerk", data.initJerk },
        { "initJerkUnit", data.initJerkUnit },
        { "initCovarianceAngularAcc", data.initCovarianceAngularAcc },
        { "initCovarianceAngularAccUnit", data.initCovarianceAngularAccUnit },
        { "initCovarianceJerk", data.initCovarianceJerk },
        { "initCovarianceJerkUnit", data.initCovarianceJerkUnit },

        // ----------------------------------------- Process Noise -------------------------------------------
        { "varAngularAccNoise", data.varAngularAccNoise },
        { "varAngularAccNoiseUnit", data.varAngularAccNoiseUnit },
        { "varJerkNoise", data.varJerkNoise },
        { "varJerkNoiseUnit", data.varJerkNoiseUnit },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, PinDataIRWKF& data)
{
    // ------------------------------------------ Initialization ---------------------------------------------
    if (j.contains("initAngularRate"))
    {
        j.at("initAngularRate").get_to(data.initAngularRate);
    }
    if (j.contains("initAngularRateUnit"))
    {
        j.at("initAngularRateUnit").get_to(data.initAngularRateUnit);
    }
    if (j.contains("initAcceleration"))
    {
        j.at("initAcceleration").get_to(data.initAcceleration);
    }
    if (j.contains("initAccelerationUnit"))
    {
        j.at("initAccelerationUnit").get_to(data.initAccelerationUnit);
    }
    if (j.contains("initAngularAcc"))
    {
        j.at("initAngularAcc").get_to(data.initAngularAcc);
    }
    if (j.contains("initAngularAccUnit"))
    {
        j.at("initAngularAccUnit").get_to(data.initAngularAccUnit);
    }
    if (j.contains("initJerk"))
    {
        j.at("initJerk").get_to(data.initJerk);
    }
    if (j.contains("initJerkUnit"))
    {
        j.at("initJerkUnit").get_to(data.initJerkUnit);
    }

    if (j.contains("initCovarianceAngularAcc"))
    {
        j.at("initCovarianceAngularAcc").get_to(data.initCovarianceAngularAcc);
    }
    if (j.contains("initCovarianceAngularAccUnit"))
    {
        j.at("initCovarianceAngularAccUnit").get_to(data.initCovarianceAngularAccUnit);
    }
    if (j.contains("initCovarianceJerk"))
    {
        j.at("initCovarianceJerk").get_to(data.initCovarianceJerk);
    }
    if (j.contains("initCovarianceJerkUnit"))
    {
        j.at("initCovarianceJerkUnit").get_to(data.initCovarianceJerkUnit);
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
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const PinDataBsplineKF& data)
{
    j = json{
        // ---------------------------------------- Initialization -------------------------------------------
        { "initAngularRate", data.initCoeffsAngRate },
        { "initCoeffsAngularRateUnit", data.initCoeffsAngularRateUnit },
        { "initCoeffsAccel", data.initCoeffsAccel },
        { "initCoeffsAccelUnit", data.initCoeffsAccelUnit },
        { "initCovarianceCoeffsAngRate", data.initCovarianceCoeffsAngRate },
        { "initCovarianceCoeffsAngRateUnit", data.initCovarianceCoeffsAngRateUnit },
        { "initCovarianceCoeffsAccel", data.initCovarianceCoeffsAccel },
        { "initCovarianceCoeffsAccelUnit", data.initCovarianceCoeffsAccelUnit },

        // ----------------------------------------- Process Noise -------------------------------------------
        { "varCoeffsAngRateNoise", data.varCoeffsAngRateNoise },
        { "varCoeffsAngRateUnit", data.varCoeffsAngRateUnit },
        { "varCoeffsAccelNoise", data.varCoeffsAccelNoise },
        { "varCoeffsAccelUnit", data.varCoeffsAccelUnit },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, PinDataBsplineKF& data)
{
    // ------------------------------------------ Initialization ---------------------------------------------

    if (j.contains("initCoeffsAngRate"))
    {
        j.at("initCoeffsAngRate").get_to(data.initCoeffsAngRate);
    }
    if (j.contains("initCoeffsAngularRateUnit"))
    {
        j.at("initCoeffsAngularRateUnit").get_to(data.initCoeffsAngularRateUnit);
    }
    if (j.contains("initCoeffsAccel"))
    {
        j.at("initCoeffsAccel").get_to(data.initCoeffsAccel);
    }
    if (j.contains("initCoeffsAccelUnit"))
    {
        j.at("initCoeffsAccelUnit").get_to(data.initCoeffsAccelUnit);
    }
    if (j.contains("initCovarianceCoeffsAngRate"))
    {
        j.at("initCovarianceCoeffsAngRate").get_to(data.initCovarianceCoeffsAngRate);
    }
    if (j.contains("initCovarianceCoeffsAngRateUnit"))
    {
        j.at("initCovarianceCoeffsAngRateUnit").get_to(data.initCovarianceCoeffsAngRateUnit);
    }
    if (j.contains("initCovarianceCoeffsAccel"))
    {
        j.at("initCovarianceCoeffsAccel").get_to(data.initCovarianceCoeffsAccel);
    }
    if (j.contains("initCovarianceCoeffsAccelUnit"))
    {
        j.at("initCovarianceCoeffsAccelUnit").get_to(data.initCovarianceCoeffsAccelUnit);
    }

    // ------------------------------------------- Process Noise ---------------------------------------------

    if (j.contains("varCoeffsAngRateNoise"))
    {
        j.at("varCoeffsAngRateNoise").get_to(data.varCoeffsAngRateNoise);
    }
    if (j.contains("varCoeffsAngRateUnit"))
    {
        j.at("varCoeffsAngRateUnit").get_to(data.varCoeffsAngRateUnit);
    }
    if (j.contains("varCoeffsAccelNoise"))
    {
        j.at("varCoeffsAccelNoise").get_to(data.varCoeffsAccelNoise);
    }
    if (j.contains("varCoeffsAccelUnit"))
    {
        j.at("varCoeffsAccelUnit").get_to(data.varCoeffsAccelUnit);
    }
}
} // namespace NAV

NAV::ImuFusion::ImuFusion()
    : Imu(typeStatic())
{
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

    float columnWidth = 130.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::Separator();

    // #######################################################################################################
    //                                               KF config
    // #######################################################################################################
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);

    ImGui::SetNextItemWidth(columnWidth);
    if (gui::widgets::EnumCombo(fmt::format("IMU fusion type##{}", size_t(id)).c_str(), _imuFusionType))
    {
        LOG_DEBUG("{}: imuFusionType changed to {}", nameId(), fmt::underlying(_imuFusionType));

        flow::ApplyChanges();
        doDeinitialize();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("IRWKF: Integrated-Random-Walk Kalman-Filter (estimates angular acceleration and jerk).\nB-spline KF: Kalman-Filter that estimates three equally spaced quadratic B-splines for angular rate and acceleration, respectively.");

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);

    ImGui::SetNextItemWidth(columnWidth);
    if (ImGui::InputDoubleL(fmt::format("Highest IMU sample rate in [Hz]##{}", size_t(id)).c_str(), &_imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.0f"))
    {
        LOG_DEBUG("{}: imuFrequency changed to {}", nameId(), _imuFrequency);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("The inverse of this rate is used as the initial 'dt' for the Kalman Filter Prediction (Phi and Q).");

    if (_imuFusionType == ImuFusionType::Bspline)
    {
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Spacing between the quadratic B-splines in [s]##{}", size_t(id)).c_str(), &_splineSpacing, 1e-3, 1.0, 0.0, 0.0, "%.3f"))
        {
            LOG_DEBUG("{}: splineSpacing changed to {}", nameId(), _splineSpacing);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Time difference between each quadratic B-spline, maximum: 1.0 second");
    }

    if (ImGui::Checkbox(fmt::format("Rank check for Kalman filter matrices##{}", size_t(id)).c_str(), &_checkKalmanMatricesRanks))
    {
        LOG_DEBUG("{}: checkKalmanMatricesRanks {}", nameId(), _checkKalmanMatricesRanks);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Computationally intensive - only recommended for debugging.");

    if (_imuFusionType != ImuFusionType::IRWKF)
    {
        ImGui::BeginDisabled();
    }
    if (ImGui::Checkbox(fmt::format("Auto-initialize Kalman filter##{}", size_t(id)).c_str(), &_autoInitKF))
    {
        LOG_DATA("{}: auto-initialize KF: {}", nameId(), _autoInitKF);
        flow::ApplyChanges();
    }
    if (_imuFusionType != ImuFusionType::IRWKF)
    {
        if (_autoInitKF)
        {
            _autoInitKF = false;
            LOG_INFO("{}: Auto-initialization for KF turned off. This is currently only available for the IRWKF.", nameId());
        }
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Initializes the KF by averaging the data over a specified time frame. Currently only available for the IRWKF fusion type.");
    if (ImGui::Checkbox(fmt::format("Characteristics of the multiple IMUs are identical##{}", size_t(id)).c_str(), &_imuCharacteristicsIdentical))
    {
        LOG_DATA("{}: imuCharacteristicsIdentical: {}", nameId(), _imuCharacteristicsIdentical);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("GUI input cells can be reduced considerably.");
    if (ImGui::Checkbox(fmt::format("Biases of the multiple IMUs are identical##{}", size_t(id)).c_str(), &_imuBiasesIdentical))
    {
        LOG_DATA("{}: imuBiasesIdentical: {}", nameId(), _imuBiasesIdentical);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("GUI input cells can be reduced considerably.");

    ImGui::Separator();

    // #######################################################################################################
    //                                           KF initialization
    // #######################################################################################################

    if (_autoInitKF)
    {
        ImGui::Text("Kalman Filter initialization (auto-init)");

        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Averaging time in [s]##{}", size_t(id)).c_str(), &_averageEndTime, 1e-3, 1e4, 0.0, 0.0, "%.0f"))
        {
            LOG_DEBUG("{}: averageEndTime changed to {}", nameId(), _averageEndTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Determines how long the data is averaged before the KF is auto-initialized");

        if (ImGui::Checkbox(fmt::format("Initialize Jerk variance to acceleration variance and angular acceleration variance to angular rate variance##{}", size_t(id)).c_str(), &_initJerkAngAcc))
        {
            LOG_DATA("{}: initJerkAngAcc: {}", nameId(), _initJerkAngAcc);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Otherwise zero");
    }
    else
    {
        ImGui::Text("Kalman Filter initialization (manual init)");

        // ---------------------------------------- State vector x0 -------------------------------------------
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("x - State vector##{}", size_t(id)).c_str()))
        {
            if (_imuFusionType == ImuFusionType::IRWKF)
            {
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate##{}", size_t(id)).c_str(),
                                                       configWidth, unitWidth, _pinDataIRWKF.initAngularRate.data(), reinterpret_cast<int*>(&_pinDataIRWKF.initAngularRateUnit), "deg/s\0"
                                                                                                                                                                                 "rad/s\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initAngularRate changed to {}", nameId(), _pinDataIRWKF.initAngularRate);
                    LOG_DATA("{}: AngularRateUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.initAngularRateUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration##{}", size_t(id)).c_str(),
                                                       configWidth, unitWidth, _pinDataIRWKF.initAcceleration.data(), reinterpret_cast<int*>(&_pinDataIRWKF.initAccelerationUnit), "m/s²\0\0", "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initAcceleration changed to {}", nameId(), _pinDataIRWKF.initAcceleration);
                    LOG_DATA("{}: initAccelerationUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.initAccelerationUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular Acceleration##{}", size_t(id)).c_str(),
                                                       configWidth, unitWidth, _pinDataIRWKF.initAngularAcc.data(), reinterpret_cast<int*>(&_pinDataIRWKF.initAngularAccUnit), "deg/s²\0"
                                                                                                                                                                               "rad/s^2\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initAngularAcc changed to {}", nameId(), _pinDataIRWKF.initAngularAcc);
                    LOG_DATA("{}: initAngularAccUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.initAngularAccUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk##{}", size_t(id)).c_str(),
                                                       configWidth, unitWidth, _pinDataIRWKF.initJerk.data(), reinterpret_cast<int*>(&_pinDataIRWKF.initJerkUnit), "m/s³\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initJerk changed to {}", nameId(), _pinDataIRWKF.initJerk);
                    LOG_DATA("{}: PinData::JerkVarianceUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.initJerkUnit));
                    flow::ApplyChanges();
                }
            }
            else // (_imuFusionType == ImuFusionType::Bspline)
            {
                if (gui::widgets::InputDouble3WithUnit(fmt::format("B-spline coefficients for the angular rate##{}", size_t(id)).c_str(),
                                                       configWidth, unitWidth, _initCoeffsAngRateTemp.data(), reinterpret_cast<int*>(&_pinDataBsplineKF.initCoeffsAngularRateUnit), "deg/s\0"
                                                                                                                                                                                    "rad/s\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCoeffsAngularRateUnit changed to {}", nameId(), fmt::underlying(_pinDataBsplineKF.initCoeffsAngularRateUnit));
                    flow::ApplyChanges();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("B-spline coefficients for the acceleration##{}", size_t(id)).c_str(),
                                                       configWidth, unitWidth, _initCoeffsAccelTemp.data(), reinterpret_cast<int*>(&_pinDataBsplineKF.initCoeffsAccelUnit), "m/s²\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCoeffsAccelUnit changed to {}", nameId(), fmt::underlying(_pinDataBsplineKF.initCoeffsAccelUnit));
                    flow::ApplyChanges();
                }
                for (uint8_t i = 0; i < _numBsplines; i += 3)
                {
                    _pinDataBsplineKF.initCoeffsAngRate.block<3, 1>(i, 0) = _initCoeffsAngRateTemp;
                    _pinDataBsplineKF.initCoeffsAccel.block<3, 1>(i, 0) = _initCoeffsAccelTemp;
                }
                LOG_DATA("{}: initCoeffsAngRate changed to {}", nameId(), _pinDataBsplineKF.initCoeffsAngRate);
                LOG_DATA("{}: initCoeffsAccel changed to {}", nameId(), _pinDataBsplineKF.initCoeffsAccel);
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate bias of sensor {}##{}", 2, size_t(id)).c_str(),
                                                   configWidth, unitWidth, _pinData[1].initAngularRateBias.data(), reinterpret_cast<int*>(&_pinData[1].initAngularRateBiasUnit), "deg/s\0rad/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration bias of sensor {}##{}", 2, size_t(id)).c_str(),
                                                   configWidth, unitWidth, _pinData[1].initAccelerationBias.data(), reinterpret_cast<int*>(&_pinData[1].initAccelerationBiasUnit), "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                flow::ApplyChanges();
            }
            if (!_imuBiasesIdentical)
            {
                for (size_t pinIndex = 2; pinIndex < _nInputPins; ++pinIndex)
                {
                    if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate bias of sensor {}##{}", pinIndex + 1, size_t(id)).c_str(),
                                                           configWidth, unitWidth, _pinData[pinIndex].initAngularRateBias.data(), reinterpret_cast<int*>(&_pinData[pinIndex].initAngularRateBiasUnit), "deg/s\0rad/s\0\0",
                                                           "%.2e", ImGuiInputTextFlags_CharsScientific))
                    {
                        flow::ApplyChanges();
                    }

                    if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration bias of sensor {}##{}", pinIndex + 1, size_t(id)).c_str(),
                                                           configWidth, unitWidth, _pinData[pinIndex].initAccelerationBias.data(), reinterpret_cast<int*>(&_pinData[pinIndex].initAccelerationBiasUnit), "m/s^2\0\0",
                                                           "%.2e", ImGuiInputTextFlags_CharsScientific))
                    {
                        flow::ApplyChanges();
                    }
                }
            }

            ImGui::TreePop();
        }

        // ----------------------------------- Error covariance matrix P0 -------------------------------------
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("P - Error covariance matrix##{}", size_t(id)).c_str()))
        {
            if (_imuFusionType == ImuFusionType::IRWKF)
            {
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate covariance ({})##{}",
                                                                   _pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                           || _pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(),
                                                       configWidth, unitWidth, _pinData[0].initCovarianceAngularRate.data(), reinterpret_cast<int*>(&_pinData[0].initCovarianceAngularRateUnit), "(rad/s)²\0"
                                                                                                                                                                                                 "rad/s\0"
                                                                                                                                                                                                 "(deg/s)²\0"
                                                                                                                                                                                                 "deg/s\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCovarianceAngularRate changed to {}", nameId(), _pinData[0].initCovarianceAngularRate);
                    LOG_DATA("{}: AngRateVarianceUnit changed to {}", nameId(), fmt::underlying(_pinData[0].initCovarianceAngularRateUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular acceleration covariance ({})##{}",
                                                                   _pinDataIRWKF.initCovarianceAngularAccUnit == PinDataIRWKF::AngularAccVarianceUnit::rad2_s4
                                                                           || _pinDataIRWKF.initCovarianceAngularAccUnit == PinDataIRWKF::AngularAccVarianceUnit::deg2_s4
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(),
                                                       configWidth, unitWidth, _pinDataIRWKF.initCovarianceAngularAcc.data(), reinterpret_cast<int*>(&_pinDataIRWKF.initCovarianceAngularAccUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                                                   "rad/s^2\0"
                                                                                                                                                                                                   "(deg^2)/(s^4)\0"
                                                                                                                                                                                                   "deg/s^2\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCovarianceAngularAcc changed to {}", nameId(), _pinDataIRWKF.initCovarianceAngularAcc);
                    LOG_DATA("{}: PinData::AngularAccVarianceUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.initCovarianceAngularAccUnit));
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
                    LOG_DATA("{}: initCovarianceAcceleration changed to {}", nameId(), _pinData[0].initCovarianceAcceleration);
                    LOG_DATA("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), fmt::underlying(_pinData[0].initCovarianceAccelerationUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk covariance ({})##{}",
                                                                   _pinDataIRWKF.initCovarianceJerkUnit == PinDataIRWKF::JerkVarianceUnit::m2_s6
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(),
                                                       configWidth, unitWidth, _pinDataIRWKF.initCovarianceJerk.data(), reinterpret_cast<int*>(&_pinDataIRWKF.initCovarianceJerkUnit), "(m^2)/(s^6)\0"
                                                                                                                                                                                       "m/s^3\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCovarianceJerk changed to {}", nameId(), _pinDataIRWKF.initCovarianceJerk);
                    LOG_DATA("{}: PinData::JerkVarianceUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.initCovarianceJerkUnit));
                    flow::ApplyChanges();
                }
            }
            else // (_imuFusionType == ImuFusionType::Bspline)
            {
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Covariance of the B-spline coefficients of the angular rate ({})##{}",
                                                                   _pinDataBsplineKF.initCovarianceCoeffsAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                           || _pinDataBsplineKF.initCovarianceCoeffsAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(),
                                                       configWidth, unitWidth, _initCovarianceCoeffsAngRateTemp.data(), reinterpret_cast<int*>(&_pinDataBsplineKF.initCovarianceCoeffsAngRateUnit), "(rad/s)²\0"
                                                                                                                                                                                                    "rad/s\0"
                                                                                                                                                                                                    "(deg/s)²\0"
                                                                                                                                                                                                    "deg/s\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCovarianceCoeffsAngRateUnit changed to {}", nameId(), fmt::underlying(_pinDataBsplineKF.initCovarianceCoeffsAngRateUnit));
                    flow::ApplyChanges();
                }
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Covariance of the B-spline coefficients of the acceleration ({})##{}",
                                                                   _pinDataBsplineKF.initCovarianceCoeffsAccelUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(),
                                                       configWidth, unitWidth, _initCovarianceCoeffsAccelTemp.data(), reinterpret_cast<int*>(&_pinDataBsplineKF.initCovarianceCoeffsAccelUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                "m/s^2\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: initCovarianceCoeffsAccelUnit changed to {}", nameId(), fmt::underlying(_pinDataBsplineKF.initCovarianceCoeffsAccelUnit));
                    flow::ApplyChanges();
                }
                for (uint8_t i = 0; i < _numBsplines; i += 3)
                {
                    _pinDataBsplineKF.initCovarianceCoeffsAngRate.block<3, 1>(i, 0) = _initCovarianceCoeffsAngRateTemp;
                    _pinDataBsplineKF.initCovarianceCoeffsAccel.block<3, 1>(i, 0) = _initCovarianceCoeffsAccelTemp;
                }
                LOG_DATA("{}: initCovarianceCoeffsAngRate changed to {}", nameId(), _pinDataBsplineKF.initCovarianceCoeffsAngRate);
                LOG_DATA("{}: initCovarianceCoeffsAccel changed to {}", nameId(), _pinDataBsplineKF.initCovarianceCoeffsAccel);
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate bias covariance of sensor {} ({})##{}", 2,
                                                               _pinData[1].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                       || _pinData[1].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinData[1].initCovarianceBiasAngRate.data(), reinterpret_cast<int*>(&_pinData[1].initCovarianceBiasAngRateUnit), "(rad^2)/(s^2)\0"
                                                                                                                                                                                             "rad/s\0"
                                                                                                                                                                                             "(deg^2)/(s^2)\0"
                                                                                                                                                                                             "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: initCovarianceBiasAngRate changed to {}", nameId(), _pinData[1].initCovarianceBiasAngRate);
                LOG_DATA("{}: PinData::AngRateVarianceUnit changed to {}", nameId(), fmt::underlying(_pinData[1].initCovarianceBiasAngRateUnit));
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration bias covariance of sensor {} ({})##{}", 2,
                                                               _pinData[1].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinData[1].initCovarianceBiasAcc.data(), reinterpret_cast<int*>(&_pinData[1].initCovarianceBiasAccUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                     "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: initCovarianceBiasAcc changed to {}", nameId(), _pinData[1].initCovarianceBiasAcc);
                LOG_DATA("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), fmt::underlying(_pinData[1].initCovarianceBiasAccUnit));
                flow::ApplyChanges();
            }
            if (!_imuCharacteristicsIdentical)
            {
                for (size_t pinIndex = 2; pinIndex < _nInputPins; ++pinIndex)
                {
                    if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate bias covariance of sensor {} ({})##{}", pinIndex + 1,
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
                        LOG_DATA("{}: initCovarianceBiasAngRate changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAngRate);
                        LOG_DATA("{}: PinData::AngRateVarianceUnit changed to {}", nameId(), fmt::underlying(_pinData[pinIndex].initCovarianceBiasAngRateUnit));
                        flow::ApplyChanges();
                    }

                    if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration bias covariance of sensor {} ({})##{}", pinIndex + 1,
                                                                       _pinData[pinIndex].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                           ? "Variance σ²"
                                                                           : "Standard deviation σ",
                                                                       size_t(id))
                                                               .c_str(),
                                                           configWidth, unitWidth, _pinData[pinIndex].initCovarianceBiasAcc.data(), reinterpret_cast<int*>(&_pinData[pinIndex].initCovarianceBiasAccUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                           "m/s^2\0\0",
                                                           "%.2e", ImGuiInputTextFlags_CharsScientific))
                    {
                        LOG_DATA("{}: initCovarianceBiasAcc changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAcc);
                        LOG_DATA("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), fmt::underlying(_pinData[pinIndex].initCovarianceBiasAccUnit));
                        flow::ApplyChanges();
                    }
                }
            }

            ImGui::TreePop();
        }
    }

    ImGui::Separator();

    // #######################################################################################################
    //                                           KF noise setting
    // #######################################################################################################
    ImGui::Text("Kalman Filter noise setting");

    // -------------------------------------- Process noise matrix Q -----------------------------------------
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Q - System/Process noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        if (_imuFusionType == ImuFusionType::IRWKF)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular acceleration ({})##{}",
                                                               _pinDataIRWKF.varAngularAccNoiseUnit == PinDataIRWKF::AngularAccVarianceUnit::rad2_s4
                                                                       || _pinDataIRWKF.varAngularAccNoiseUnit == PinDataIRWKF::AngularAccVarianceUnit::deg2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinDataIRWKF.varAngularAccNoise.data(), reinterpret_cast<int*>(&_pinDataIRWKF.varAngularAccNoiseUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                                   "rad/s^2\0"
                                                                                                                                                                                   "(deg^2)/(s^4)\0"
                                                                                                                                                                                   "deg/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: varAngularAccNoise changed to {}", nameId(), _pinDataIRWKF.varAngularAccNoise.transpose());
                LOG_DATA("{}: varAngularAccNoiseUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.varAngularAccNoiseUnit));
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk ({})##{}",
                                                               _pinDataIRWKF.varJerkNoiseUnit == PinDataIRWKF::JerkVarianceUnit::m2_s6
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _pinDataIRWKF.varJerkNoise.data(), reinterpret_cast<int*>(&_pinDataIRWKF.varJerkNoiseUnit), "(m^2)/(s^6)\0"
                                                                                                                                                                       "m/s^3\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: varJerkNoise changed to {}", nameId(), _pinDataIRWKF.varJerkNoise.transpose());
                LOG_DATA("{}: varJerkNoiseUnit changed to {}", nameId(), fmt::underlying(_pinDataIRWKF.varJerkNoiseUnit));
                flow::ApplyChanges();
            }
        }
        else // (_imuFusionType == ImuFusionType::Bspline)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate B-spline coefficients ({})##{}",
                                                               _pinDataBsplineKF.varCoeffsAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                       || _pinDataBsplineKF.varCoeffsAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _procNoiseCoeffsAngRateTemp.data(), reinterpret_cast<int*>(&_pinDataBsplineKF.varCoeffsAngRateUnit), "(rad^2)/(s^2)\0"
                                                                                                                                                                                "rad/s\0"
                                                                                                                                                                                "(deg^2)/(s^2)\0"
                                                                                                                                                                                "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: varCoeffsAngRateUnit changed to {}", nameId(), fmt::underlying(_pinDataBsplineKF.varCoeffsAngRateUnit));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration B-spline coefficients ({})##{}",
                                                               _pinDataBsplineKF.varCoeffsAccelUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _procNoiseCoeffsAccelTemp.data(), reinterpret_cast<int*>(&_pinDataBsplineKF.varCoeffsAccelUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                            "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: varCoeffsAccelUnit changed to {}", nameId(), fmt::underlying(_pinDataBsplineKF.varCoeffsAccelUnit));
                flow::ApplyChanges();
            }
            for (uint8_t i = 0; i < _numBsplines; i += 3)
            {
                _pinDataBsplineKF.varCoeffsAngRateNoise.block<3, 1>(i, 0) = _procNoiseCoeffsAngRateTemp;
                _pinDataBsplineKF.varCoeffsAccelNoise.block<3, 1>(i, 0) = _procNoiseCoeffsAccelTemp;
            }
            LOG_DATA("{}: varCoeffsAngRateNoise changed to {}", nameId(), _pinDataBsplineKF.varCoeffsAngRateNoise.transpose());
            LOG_DATA("{}: varCoeffsAccelNoise changed to {}", nameId(), _pinDataBsplineKF.varCoeffsAccelNoise.transpose());
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Bias of the angular rate of sensor {} ({})##{}", 2,
                                                           _pinData[1].varBiasAngRateNoiseUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                   || _pinData[1].varBiasAngRateNoiseUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                               configWidth, unitWidth, _pinData[1].varBiasAngRateNoise.data(), reinterpret_cast<int*>(&_pinData[1].varBiasAngRateNoiseUnit), "(rad/s)^2\0"
                                                                                                                                                                             "rad/s\0"
                                                                                                                                                                             "(deg/s)^2\0"
                                                                                                                                                                             "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DATA("{}: varBiasAngRateNoise changed to {}", nameId(), _pinData[1].varBiasAngRateNoise.transpose());
            LOG_DATA("{}: varBiasAngRateNoiseUnit changed to {}", nameId(), fmt::underlying(_pinData[1].varBiasAngRateNoiseUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Bias of the acceleration of sensor {} ({})##{}", 2,
                                                           _pinData[1].varBiasAccelerationNoiseUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                               configWidth, unitWidth, _pinData[1].varBiasAccelerationNoise.data(), reinterpret_cast<int*>(&_pinData[1].varBiasAccelerationNoiseUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                       "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DATA("{}: varBiasAccelerationNoise changed to {}", nameId(), _pinData[1].varBiasAccelerationNoise.transpose());
            LOG_DATA("{}: varBiasAccelerationNoiseUnit changed to {}", nameId(), fmt::underlying(_pinData[1].varBiasAccelerationNoiseUnit));
            flow::ApplyChanges();
        }
        if (!_imuCharacteristicsIdentical)
        {
            for (size_t pinIndex = 2; pinIndex < _nInputPins; ++pinIndex)
            {
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Bias of the angular rate of sensor {} ({})##{}", pinIndex + 1,
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
                    LOG_DATA("{}: varBiasAngRateNoise changed to {}", nameId(), _pinData[pinIndex].varBiasAngRateNoise.transpose());
                    LOG_DATA("{}: varBiasAngRateNoiseUnit changed to {}", nameId(), fmt::underlying(_pinData[pinIndex].varBiasAngRateNoiseUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Bias of the acceleration of sensor {} ({})##{}", pinIndex + 1,
                                                                   _pinData[pinIndex].varBiasAccelerationNoiseUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                                       configWidth, unitWidth, _pinData[pinIndex].varBiasAccelerationNoise.data(), reinterpret_cast<int*>(&_pinData[pinIndex].varBiasAccelerationNoiseUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                             "m/s^2\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: varBiasAccelerationNoise changed to {}", nameId(), _pinData[pinIndex].varBiasAccelerationNoise.transpose());
                    LOG_DATA("{}: varBiasAccelerationNoiseUnit changed to {}", nameId(), fmt::underlying(_pinData[pinIndex].varBiasAccelerationNoiseUnit));
                    flow::ApplyChanges();
                }
            }
        }

        ImGui::TreePop();
    }

    // ------------------------------------ Measurement noise matrix R ---------------------------------------
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate of sensor {} ({})##{}", 1,
                                                           _pinData[0].measurementUncertaintyAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2
                                                                   || _pinData[0].measurementUncertaintyAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].measurementUncertaintyAngularRate.data(), reinterpret_cast<int*>(&_pinData[0].measurementUncertaintyAngularRateUnit), "(rad/s)^2\0"
                                                                                                                                                                                                         "rad/s\0"
                                                                                                                                                                                                         "(deg/s)^2\0"
                                                                                                                                                                                                         "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DATA("{}: stdevAngularAcc changed to {}", nameId(), _pinData[0].measurementUncertaintyAngularRate.transpose());
            LOG_DATA("{}: stdevAngularAccUnit changed to {}", nameId(), fmt::underlying(_pinData[0].measurementUncertaintyAngularRateUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration of sensor {} ({})##{}", 1,
                                                           _pinData[0].measurementUncertaintyAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _pinData[0].measurementUncertaintyAcceleration.data(), reinterpret_cast<int*>(&_pinData[0].measurementUncertaintyAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                           "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DATA("{}: stdevJerk changed to {}", nameId(), _pinData[0].measurementUncertaintyAcceleration.transpose());
            LOG_DATA("{}: stdevJerkUnit changed to {}", nameId(), fmt::underlying(_pinData[0].measurementUncertaintyAccelerationUnit));
            flow::ApplyChanges();
        }
        if (!_imuCharacteristicsIdentical)
        {
            for (size_t pinIndex = 1; pinIndex < _nInputPins; ++pinIndex)
            {
                if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate of sensor {} ({})##{}", pinIndex + 1,
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
                    LOG_DATA("{}: stdevAngularAcc changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAngularRate.transpose());
                    LOG_DATA("{}: stdevAngularAccUnit changed to {}", nameId(), fmt::underlying(_pinData[pinIndex].measurementUncertaintyAngularRateUnit));
                    flow::ApplyChanges();
                }

                if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration of sensor {} ({})##{}", pinIndex + 1,
                                                                   _pinData[pinIndex].measurementUncertaintyAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4
                                                                       ? "Variance σ²"
                                                                       : "Standard deviation σ",
                                                                   size_t(id))
                                                           .c_str(),
                                                       configWidth, unitWidth, _pinData[pinIndex].measurementUncertaintyAcceleration.data(), reinterpret_cast<int*>(&_pinData[pinIndex].measurementUncertaintyAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                                                                 "m/s^2\0\0",
                                                       "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DATA("{}: stdevJerk changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAcceleration.transpose());
                    LOG_DATA("{}: stdevJerkUnit changed to {}", nameId(), fmt::underlying(_pinData[pinIndex].measurementUncertaintyAccelerationUnit));
                    flow::ApplyChanges();
                }
            }
        }

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::ImuFusion::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuFusionType"] = _imuFusionType;
    j["checkKalmanMatricesRanks"] = _checkKalmanMatricesRanks;
    j["nInputPins"] = _nInputPins;
    j["imuFrequency"] = _imuFrequency;
    j["numStates"] = _numStates;
    j["pinData"] = _pinData;
    j["pinDataIRWKF"] = _pinDataIRWKF;
    j["pinDataBsplineKF"] = _pinDataBsplineKF;
    j["initCoeffsAngRateTemp"] = _initCoeffsAngRateTemp;
    j["initCoeffsAccelTemp"] = _initCoeffsAccelTemp;
    j["initCovarianceCoeffsAngRateTemp"] = _initCovarianceCoeffsAngRateTemp;
    j["initCovarianceCoeffsAccelTemp"] = _initCovarianceCoeffsAccelTemp;
    j["procNoiseCoeffsAngRateTemp"] = _procNoiseCoeffsAngRateTemp;
    j["procNoiseCoeffsAccelTemp"] = _procNoiseCoeffsAccelTemp;
    j["autoInitKF"] = _autoInitKF;
    j["initJerkAngAcc"] = _initJerkAngAcc;
    j["kfInitialized"] = _kfInitialized;
    j["averageEndTime"] = _averageEndTime;
    j["splineSpacing"] = _splineSpacing;

    return j;
}

void NAV::ImuFusion::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuFusionType"))
    {
        j.at("imuFusionType").get_to(_imuFusionType);
    }
    if (j.contains("checkKalmanMatricesRanks"))
    {
        j.at("checkKalmanMatricesRanks").get_to(_checkKalmanMatricesRanks);
    }
    if (j.contains("nInputPins"))
    {
        j.at("nInputPins").get_to(_nInputPins);
        updateNumberOfInputPins();
    }
    if (j.contains("imuFrequency"))
    {
        j.at("imuFrequency").get_to(_imuFrequency);
    }
    if (j.contains("numStates"))
    {
        j.at("numStates").get_to(_numStates);
    }
    if (j.contains("pinData"))
    {
        j.at("pinData").get_to(_pinData);
    }
    if (j.contains("pinDataIRWKF"))
    {
        j.at("pinDataIRWKF").get_to(_pinDataIRWKF);
    }
    if (j.contains("pinDataBsplineKF"))
    {
        j.at("pinDataBsplineKF").get_to(_pinDataBsplineKF);
    }
    if (j.contains("initCoeffsAngRateTemp"))
    {
        j.at("initCoeffsAngRateTemp").get_to(_initCoeffsAngRateTemp);
    }
    if (j.contains("initCoeffsAccelTemp"))
    {
        j.at("initCoeffsAccelTemp").get_to(_initCoeffsAccelTemp);
    }
    if (j.contains("initCovarianceCoeffsAngRateTemp"))
    {
        j.at("initCovarianceCoeffsAngRateTemp").get_to(_initCovarianceCoeffsAngRateTemp);
    }
    if (j.contains("initCovarianceCoeffsAccelTemp"))
    {
        j.at("initCovarianceCoeffsAccelTemp").get_to(_initCovarianceCoeffsAccelTemp);
    }
    if (j.contains("procNoiseCoeffsAngRateTemp"))
    {
        j.at("procNoiseCoeffsAngRateTemp").get_to(_procNoiseCoeffsAngRateTemp);
    }
    if (j.contains("procNoiseCoeffsAccelTemp"))
    {
        j.at("procNoiseCoeffsAccelTemp").get_to(_procNoiseCoeffsAccelTemp);
    }
    if (j.contains("autoInitKF"))
    {
        j.at("autoInitKF").get_to(_autoInitKF);
    }
    if (j.contains("initJerkAngAcc"))
    {
        j.at("initJerkAngAcc").get_to(_initJerkAngAcc);
    }
    if (j.contains("_kfInitialized"))
    {
        j.at("_kfInitialized").get_to(_kfInitialized);
    }
    if (j.contains("averageEndTime"))
    {
        j.at("averageEndTime").get_to(_averageEndTime);
    }
    if (j.contains("splineSpacing"))
    {
        j.at("splineSpacing").get_to(_splineSpacing);
    }
}

bool NAV::ImuFusion::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _imuRotations_accel.clear();
    _imuRotations_gyro.clear();
    _biasCovariances.clear();
    _processNoiseVariances.clear();
    _measurementNoiseVariances.clear();

    _cumulatedImuObs.clear();
    _cumulatedPinIds.clear();
    _lastFiltObs.reset();
    _latestTimestamp = InsTime{};
    _firstTimestamp.reset();

    _imuPosSet = false;
    _kfInitialized = false;

    for (size_t pinIndex = 0; pinIndex < _pinData.size(); pinIndex++)
    {
        if (!inputPins.at(pinIndex).isPinLinked())
        {
            LOG_INFO("Fewer links than input pins - Consider deleting pins that are not connected to limit KF matrices to the necessary size.");
        }
    }

    _numStatesEst = _imuFusionType == ImuFusionType::IRWKF ? _numStatesEstIRWKF : _numStatesEstBsplineKF;

    _numStates = _numStatesEst + static_cast<uint8_t>((_nInputPins - 1) * _numStatesPerPin);

    _kalmanFilter = KalmanFilter{ _numStates, _numMeasurements };
    _kalmanFilter.setZero();

    initializeMountingAngles();

    // --------------------------------------------------------- KF Initializations ------------------------------------------------------------
    if (!_autoInitKF) // i.e. manual initialization thru inputs from the GUI
    {
        auto dtInit = 1.0 / _imuFrequency; // Initial state transition time in [s]

        if (_imuFusionType == ImuFusionType::IRWKF)
        {
            _kalmanFilter = IRWKF::initializeKalmanFilterManually(_nInputPins, _pinData, _pinDataIRWKF, _numStates, dtInit, _processNoiseVariances, _kalmanFilter, _imuCharacteristicsIdentical, _imuBiasesIdentical);
        }
        else // (_imuFusionType == ImuFusionType::BsplineKF)
        {
            _kalmanFilter = BsplineKF::initializeKalmanFilterManually(_nInputPins, _pinData, _pinDataBsplineKF, _numStates, dtInit, _processNoiseVariances, _kalmanFilter, _imuCharacteristicsIdentical, _imuBiasesIdentical);
            _latestKnot = 0.0;
        }

        LOG_DATA("{}: Initial kalmanFilter.x = {}", nameId(), _kalmanFilter.x.transpose());
        LOG_DATA("{}: Initial kalmanFilter.P =\n{}", nameId(), _kalmanFilter.P);
        LOG_DATA("{}: Initial kalmanFilter.Phi =\n{}", nameId(), _kalmanFilter.Phi);
        LOG_DATA("{}: Initial kalmanFilter.Q =\n{}", nameId(), _kalmanFilter.Q);
    }

    // -------------------------------------------------- Measurement uncertainty matrix R -----------------------------------------------------
    _measurementNoiseVariances.resize(2 * _nInputPins);

    size_t pinDataIdx = 0;
    for (size_t pinIndex = 0; pinIndex < _nInputPins; ++pinIndex)
    {
        if (!_imuCharacteristicsIdentical)
        {
            pinDataIdx = pinIndex;
        }

        // Measurement uncertainty for the angular rate (Variance σ²) in [(rad/s)^2, (rad/s)^2, (rad/s)^2]
        switch (_pinData[pinDataIdx].measurementUncertaintyAngularRateUnit)
        {
        case PinData::AngRateVarianceUnit::rad_s:
            _measurementNoiseVariances[2 * pinIndex] = (_pinData[pinDataIdx].measurementUncertaintyAngularRate).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::deg_s:
            _measurementNoiseVariances[2 * pinIndex] = (deg2rad(_pinData[pinDataIdx].measurementUncertaintyAngularRate)).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::rad2_s2:
            _measurementNoiseVariances[2 * pinIndex] = _pinData[pinDataIdx].measurementUncertaintyAngularRate;
            break;
        case PinData::AngRateVarianceUnit::deg2_s2:
            _measurementNoiseVariances[2 * pinIndex] = deg2rad((_pinData[pinDataIdx].measurementUncertaintyAngularRate).cwiseSqrt()).array().pow(2);
            break;
        }

        // Measurement uncertainty for the acceleration (Variance σ²) in [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        switch (_pinData[pinDataIdx].measurementUncertaintyAccelerationUnit)
        {
        case PinData::AccelerationVarianceUnit::m2_s4:
            _measurementNoiseVariances[1 + 2 * pinIndex] = _pinData[pinDataIdx].measurementUncertaintyAcceleration;
            break;
        case PinData::AccelerationVarianceUnit::m_s2:
            _measurementNoiseVariances[1 + 2 * pinIndex] = (_pinData[pinDataIdx].measurementUncertaintyAcceleration).array().pow(2);
            break;
        }
    }

    if (_imuCharacteristicsIdentical)
    {
        measurementNoiseMatrix_R(_kalmanFilter.R);
        LOG_DATA("{}: imuCharacteristicsIdentical - kalmanFilter.R =\n{}", nameId(), _kalmanFilter.R);
    }

    if (!_autoInitKF) { _kfInitialized = true; } // Auto-init initializes KF at 'avgEndTime' seconds --> see 'recvSignal'

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
            nm::CreateOutputPin(this, fmt::format("ImuBiases {}1", outputPins.size() + 1).c_str(), Pin::Type::Flow, { NAV::InsGnssLCKFSolution::type() });
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
        _firstTimestamp = imuObs->insTime;

        // Time until averaging ends and filtering starts (for auto-init of KF)
        _avgEndTime = imuObs->insTime + std::chrono::milliseconds(static_cast<int>(1e3 * _averageEndTime));
    }

    _timeSinceStartup = static_cast<double>((imuObs->insTime - _firstTimestamp).count());
    LOG_DATA("_timeSinceStartup = {}", _timeSinceStartup);

    // Predict states over the time difference between the latest signal and the one before
    auto dt = static_cast<double>((imuObs->insTime - _latestTimestamp).count());
    _latestTimestamp = imuObs->insTime;
    LOG_DATA("{}: dt = {}", nameId(), dt);

    if (_kfInitialized)
    {
        if (_imuFusionType == ImuFusionType::IRWKF)
        {
            IRWKF::stateTransitionMatrix_Phi(_kalmanFilter.Phi, dt);
            IRWKF::processNoiseMatrix_Q(_kalmanFilter.Q, dt, _processNoiseVariances, _numStates);
        }
        else // (_imuFusionType == ImuFusionType::Bspline)
        {
            BsplineKF::processNoiseMatrix_Q(_kalmanFilter.Q, dt, _processNoiseVariances, _numStates);

            if (_timeSinceStartup >= _latestKnot)
            {
                _latestKnot += _splineSpacing;

                BsplineKF::rotateCoeffStates(_kalmanFilter.x);
                LOG_DATA("{}: kalmanFilter.P before B-spline coeff rotation =\n{}", nameId(), _kalmanFilter.P.block<18, 18>(0, 0));
                BsplineKF::rotateErrorCovariances(_kalmanFilter.P, _numStates);
            }
        }

        LOG_DATA("{}: kalmanFilter.P (B-spline coeffs) =\n{}", nameId(), _kalmanFilter.P.block<18, 18>(0, 0));
        LOG_DATA("{}: kalmanFilter.Phi =\n{}", nameId(), _kalmanFilter.Phi);
        LOG_DATA("{}: kalmanFilter.Q =\n{}", nameId(), _kalmanFilter.Q);

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
    }

    // Read sensor rotation info from 'imuObs'
    if (std::isnan(_imuRotations_accel[pinIdx](0, 0)))
    {
        // Rotation matrix of the accelerometer platform to body frame
        _imuRotations_accel[pinIdx] = imuObs->imuPos.b_quatAccel_p().toRotationMatrix();
    }
    if (std::isnan(_imuRotations_gyro[pinIdx](0, 0)))
    {
        // Rotation matrix of the gyro platform to body frame
        _imuRotations_gyro[pinIdx] = imuObs->imuPos.b_quatGyro_p().toRotationMatrix();
    }

    // Initialize H with mounting angles (DCM) of the sensor that provided the latest measurement
    auto DCM_accel = _imuRotations_accel.at(pinIdx);
    LOG_DATA("DCM_accel =\n{}", DCM_accel);
    auto DCM_gyro = _imuRotations_gyro.at(pinIdx);
    LOG_DATA("{}: DCM_gyro =\n{}", nameId(), DCM_gyro);

    if (_imuFusionType == ImuFusionType::IRWKF)
    {
        _kalmanFilter.H = IRWKF::designMatrix_H(DCM_accel, DCM_gyro, pinIdx, _numMeasurements, _numStates, _numStatesEst, _numStatesPerPin);
        LOG_DATA("{}: Sensor (pinIdx): {}, kalmanFilter.H =\n{}", nameId(), _kalmanFilter.H, pinIdx);
    }
    else // (_imuFusionType == ImuFusionType::BsplineKF)
    {
        _kalmanFilter.H = BsplineKF::designMatrix_H(_timeSinceStartup, _splineSpacing, DCM_accel, DCM_gyro, pinIdx, _numMeasurements, _numStates, _numStatesEst, _numStatesPerPin);
        LOG_DATA("{}: timeSinceStartup: {}, Sensor (pinIdx): {}, kalmanFilter.H =\n{}", nameId(), _timeSinceStartup, pinIdx, _kalmanFilter.H);
    }

    if (!_imuCharacteristicsIdentical)
    {
        measurementNoiseMatrix_R(_kalmanFilter.R, pinIdx);
        LOG_DATA("{}: kalmanFilter.R =\n{}", nameId(), _kalmanFilter.R);
    }

    if (_checkKalmanMatricesRanks)
    {
        auto rank = (_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R).fullPivLu().rank();
        if (rank != _kalmanFilter.H.rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }
    }

    if (_autoInitKF && !_kfInitialized)
    {
        if (imuObs->insTime < _avgEndTime)
        {
            _cumulatedImuObs.push_back(imuObs);
            _cumulatedPinIds.push_back(pinIdx);
        }
        else // if (imuObs->insTime == _avgEndTime) // <-- do auto-init, once _avgEndTime is reached
        {
            double dtInit = 1.0 / _imuFrequency;
            _kalmanFilter = IRWKF::initializeKalmanFilterAuto(_nInputPins, _pinData, _pinDataIRWKF, _cumulatedPinIds, _cumulatedImuObs, _initJerkAngAcc, dtInit, _numStates, _numMeasurements, _processNoiseVariances, _kalmanFilter);
            _kfInitialized = true; // Start Kalman Filter
        }
    }
    if (_kfInitialized)
    {
        combineSignals(imuObs);
    }
}

void NAV::ImuFusion::combineSignals(const std::shared_ptr<const ImuObs>& imuObs)
{
    LOG_DATA("{}: called", nameId());

    auto imuObsFiltered = std::make_shared<ImuObs>(this->_imuPos);

    LOG_DATA("{}: Estimated state before prediction: x =\n{}", nameId(), _kalmanFilter.x);

    _kalmanFilter.predict();

    LOG_DATA("{}: kalmanFilter.P (B-spline coeffs) =\n{}", nameId(), _kalmanFilter.P.block<18, 18>(0, 0));

    LOG_DATA("{}: kalmanFilter.P (B-spline coeffs) =\n{}", nameId(), _kalmanFilter.P.block<18, 18>(0, 0));

    _kalmanFilter.z.block<3, 1>(0, 0) = imuObs->p_angularRate;
    _kalmanFilter.z.block<3, 1>(3, 0) = imuObs->p_acceleration;

    LOG_DATA("{}: Measurements z =\n{}", nameId(), _kalmanFilter.z);
    LOG_DATA("{}: coeff states x =\n{}", nameId(), _kalmanFilter.x.block<18, 1>(0, 0));
    LOG_DATA("{}: Innovation: z - H * x =\n{}", nameId(), _kalmanFilter.z - _kalmanFilter.H * _kalmanFilter.x);

    _kalmanFilter.correct();
    LOG_DATA("{}: Estimated state after correction: x =\n{}", nameId(), _kalmanFilter.x);

    if (_checkKalmanMatricesRanks)
    {
        auto rankH = (_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R).fullPivLu().rank();
        if (rankH != _kalmanFilter.H.rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rankH);
        }

        if (inputPins.at(_pinData.size() - 1).isPinLinked())
        {
            auto rankP = _kalmanFilter.P.fullPivLu().rank();
            if (rankP != _kalmanFilter.P.rows())
            {
                LOG_WARN("{}: P.rank = {}", nameId(), rankP);
                LOG_DATA("{}: kalmanFilter.P =\n{}", nameId(), _kalmanFilter.P);
            }
        }
    }

    // Construct imuObs
    imuObsFiltered->insTime = imuObs->insTime;
    if (_imuFusionType == ImuFusionType::IRWKF)
    {
        imuObsFiltered->p_acceleration = { _kalmanFilter.x(6, 0), _kalmanFilter.x(7, 0), _kalmanFilter.x(8, 0) };
        imuObsFiltered->p_angularRate = { _kalmanFilter.x(0, 0), _kalmanFilter.x(1, 0), _kalmanFilter.x(2, 0) };
    }
    else // (_imuFusionType == ImuFusionType::BsplineKF)
    {
        auto qBsplines = NAV::BsplineKF::quadraticBsplines(_timeSinceStartup, _splineSpacing);

        LOG_DATA("{}: timeSinceStartup: {}, qBsplines (stacked B-spline values, cumulatively = 1) = {}, {}, {}", nameId(), _timeSinceStartup, qBsplines.at(0), qBsplines.at(1), qBsplines.at(2));
        LOG_DATA("{}: timeSinceStartup: {}, Angular rate B-spline coefficient estimates:\n{}", nameId(), _timeSinceStartup, _kalmanFilter.x.block<9, 1>(0, 0));
        LOG_DATA("{}: timeSinceStartup: {}, Acceleration B-spline coefficient estimates:\n{}", nameId(), _timeSinceStartup, _kalmanFilter.x.block<9, 1>(9, 0));

        // Estimated angular rate: Cumulative sum of the three estimated B-spline coefficients for the angular rate
        auto angRateEst = _kalmanFilter.x.block<3, 1>(0, 0) * qBsplines.at(0)
                          + _kalmanFilter.x.block<3, 1>(3, 0) * qBsplines.at(1)
                          + _kalmanFilter.x.block<3, 1>(6, 0) * qBsplines.at(2);

        // Estimated acceleration: Cumulative sum of the three estimated B-spline coefficients for the acceleration
        auto accelEst = _kalmanFilter.x.block<3, 1>(9, 0) * qBsplines.at(0)
                        + _kalmanFilter.x.block<3, 1>(12, 0) * qBsplines.at(1)
                        + _kalmanFilter.x.block<3, 1>(15, 0) * qBsplines.at(2);

        LOG_DATA("{}: imuObs->insTime = {}, timeSinceStartup = {}, angRateEst = {}, accelEst = {}", nameId(), imuObs->insTime.toYMDHMS(), _timeSinceStartup, angRateEst.transpose(), accelEst.transpose());

        imuObsFiltered->p_acceleration = accelEst;
        imuObsFiltered->p_angularRate = angRateEst;
    }

    // Detect jumps back in time
    if (imuObsFiltered->insTime < _lastFiltObs)
    {
        LOG_ERROR("{}: imuObsFiltered->insTime < _lastFiltObs --> {}", nameId(), static_cast<double>((imuObsFiltered->insTime - _lastFiltObs).count()));
    }
    _lastFiltObs = imuObsFiltered->insTime;

    invokeCallbacks(OUTPUT_PORT_INDEX_COMBINED_SIGNAL, imuObsFiltered);

    for (size_t OUTPUT_PORT_INDEX_BIAS = 1; OUTPUT_PORT_INDEX_BIAS < _nInputPins; ++OUTPUT_PORT_INDEX_BIAS)
    {
        auto imuRelativeBiases = std::make_shared<InsGnssLCKFSolution>();
        imuRelativeBiases->insTime = imuObs->insTime;
        auto biasIndex = _numStatesEst + static_cast<uint8_t>((OUTPUT_PORT_INDEX_BIAS - 1) * _numStatesPerPin);
        LOG_DATA("{}: biasIndex = {}", nameId(), biasIndex);

        imuRelativeBiases->b_biasGyro = { _kalmanFilter.x(biasIndex, 0), _kalmanFilter.x(biasIndex + 1, 0), _kalmanFilter.x(biasIndex + 2, 0) };
        imuRelativeBiases->b_biasAccel = { _kalmanFilter.x(biasIndex + 3, 0), _kalmanFilter.x(biasIndex + 4, 0), _kalmanFilter.x(biasIndex + 5, 0) };

        LOG_DATA("{}: timeSinceStartup = {}, Relative bias {}1 Gyro:  {}", nameId(), _timeSinceStartup, OUTPUT_PORT_INDEX_BIAS + 1, imuRelativeBiases->b_biasGyro.transpose());
        LOG_DATA("{}: timeSinceStartup = {}, Relative bias {}1 Accel: {}", nameId(), _timeSinceStartup, OUTPUT_PORT_INDEX_BIAS + 1, imuRelativeBiases->b_biasAccel.transpose());

        invokeCallbacks(OUTPUT_PORT_INDEX_BIAS, imuRelativeBiases);
    }
}

// -------------------------------------- Measurement noise matrix R -----------------------------------------

Eigen::MatrixXd NAV::ImuFusion::measurementNoiseMatrix_R_adaptive(double alpha, const Eigen::MatrixXd& R, const Eigen::VectorXd& e, const Eigen::MatrixXd& H, const Eigen::MatrixXd& P)
{
    return alpha * R + (1.0 - alpha) * (e * e.transpose() + H * P * H.transpose());
}

void NAV::ImuFusion::measurementNoiseMatrix_R(Eigen::MatrixXd& R, size_t pinIndex) const
{
    R.block<3, 3>(0, 0).diagonal() = _measurementNoiseVariances.at(2 * pinIndex);
    R.block<3, 3>(3, 3).diagonal() = _measurementNoiseVariances.at(2 * pinIndex + 1);
}
