#include "ImuFusion.hpp"

#include "util/Logger.hpp"

#include <numeric>
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
        { "initAngularRate", data.initAngularRate },
        { "initAngularRateUnit", data.initAngularRateUnit },
        { "initAcceleration", data.initAcceleration },
        { "initAccelerationUnit", data.initAccelerationUnit },
        { "initAngularAcc", data.initAngularAcc },
        { "initAngularAccUnit", data.initAngularAccUnit },
        { "initJerk", data.initJerk },
        { "initJerkUnit", data.initJerkUnit },
        { "initAngularRateBias", data.initAngularRateBias },
        { "initAngularRateBiasUnit", data.initAngularRateBiasUnit },
        { "initAccelerationBias", data.initAccelerationBias },
        { "initAccelerationBiasUnit", data.initAccelerationBiasUnit },
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
                        nm::DeleteInputPin(inputPins.at(pinIndex).id);
                        nm::DeleteOutputPin(outputPins.at(pinIndex).id);
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
    if (ImGui::InputDoubleL(fmt::format("Highest IMU sample rate in [Hz]##{}", size_t(id)).c_str(), &_imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.0f"))
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

    if (ImGui::Checkbox(fmt::format("Auto-initialize Kalman filter##{}", size_t(id)).c_str(), &_autoInitKF))
    {
        LOG_DATA("{}: auto-initialize KF: {}", nameId(), _autoInitKF);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Initializes the KF by averaging the data over a specified time frame");

    ImGui::Separator();

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

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("x - State vector##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _pinData[0].initAngularRate.data(), reinterpret_cast<int*>(&_pinData[0].initAngularRateUnit), "deg/s\0"
                                                                                                                                                                         "rad/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: initAngularRate changed to {}", nameId(), _pinData[0].initAngularRate);
                LOG_DATA("{}: AngularRateUnit changed to {}", nameId(), _pinData[0].initAngularRateUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _pinData[0].initAcceleration.data(), reinterpret_cast<int*>(&_pinData[0].initAccelerationUnit), "m/s²\0\0", "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: initAcceleration changed to {}", nameId(), _pinData[0].initAcceleration);
                LOG_DATA("{}: initAccelerationUnit changed to {}", nameId(), _pinData[0].initAccelerationUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular Acceleration##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _pinData[0].initAngularAcc.data(), reinterpret_cast<int*>(&_pinData[0].initAngularAccUnit), "deg/s²\0"
                                                                                                                                                                       "rad/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: initAngularAcc changed to {}", nameId(), _pinData[0].initAngularAcc);
                LOG_DATA("{}: initAngularAccUnit changed to {}", nameId(), _pinData[0].initAngularAccUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _pinData[0].initJerk.data(), reinterpret_cast<int*>(&_pinData[0].initJerkUnit), "m/s³\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DATA("{}: initJerk changed to {}", nameId(), _pinData[0].initJerk);
                LOG_DATA("{}: PinData::JerkVarianceUnit changed to {}", nameId(), _pinData[0].initJerkUnit);
                flow::ApplyChanges();
            }

            for (size_t pinIndex = 1; pinIndex < _nInputPins; ++pinIndex)
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

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("P - Error covariance matrix##{}", size_t(id)).c_str()))
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
                LOG_DATA("{}: AngRateVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceAngularRateUnit);
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
                LOG_DATA("{}: initCovarianceAngularAcc changed to {}", nameId(), _pinData[0].initCovarianceAngularAcc);
                LOG_DATA("{}: PinData::AngularAccVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceAngularAccUnit);
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
                LOG_DATA("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceAccelerationUnit);
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
                LOG_DATA("{}: initCovarianceJerk changed to {}", nameId(), _pinData[0].initCovarianceJerk);
                LOG_DATA("{}: PinData::JerkVarianceUnit changed to {}", nameId(), _pinData[0].initCovarianceJerkUnit);
                flow::ApplyChanges();
            }

            for (size_t pinIndex = 1; pinIndex < _nInputPins; ++pinIndex)
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
                    LOG_DATA("{}: PinData::AngRateVarianceUnit changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAngRateUnit);
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
                    LOG_DATA("{}: PinData::AccelerationVarianceUnit changed to {}", nameId(), _pinData[pinIndex].initCovarianceBiasAccUnit);
                    flow::ApplyChanges();
                }
            }

            ImGui::TreePop();
        }
    }

    ImGui::Separator();

    ImGui::Text("Kalman Filter noise setting");

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
            LOG_DATA("{}: varAngularAccNoise changed to {}", nameId(), _pinData[0].varAngularAccNoise.transpose());
            LOG_DATA("{}: varAngularAccNoiseUnit changed to {}", nameId(), _pinData[0].varAngularAccNoiseUnit);
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
            LOG_DATA("{}: varJerkNoise changed to {}", nameId(), _pinData[0].varJerkNoise.transpose());
            LOG_DATA("{}: varJerkNoiseUnit changed to {}", nameId(), _pinData[0].varJerkNoiseUnit);
            flow::ApplyChanges();
        }

        for (size_t pinIndex = 1; pinIndex < _nInputPins; ++pinIndex)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the bias of the angular rate of sensor {} ({})##{}", pinIndex + 1,
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
                LOG_DATA("{}: varBiasAngRateNoiseUnit changed to {}", nameId(), _pinData[pinIndex].varBiasAngRateNoiseUnit);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the bias of the acceleration of sensor {} ({})##{}", pinIndex + 1,
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
                LOG_DATA("{}: varBiasAccelerationNoiseUnit changed to {}", nameId(), _pinData[pinIndex].varBiasAccelerationNoiseUnit);
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
                LOG_DATA("{}: stdevAngularAcc changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAngularRate.transpose());
                LOG_DATA("{}: stdevAngularAccUnit changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAngularRateUnit);
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
                LOG_DATA("{}: stdevJerk changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAcceleration.transpose());
                LOG_DATA("{}: stdevJerkUnit changed to {}", nameId(), _pinData[pinIndex].measurementUncertaintyAccelerationUnit);
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
    j["imuFrequency"] = _imuFrequency;
    j["designMatrixInitialized"] = _designMatrixInitialized;
    j["numStates"] = _numStates;
    j["numMeasurements"] = _numMeasurements;
    j["pinData"] = _pinData;
    j["autoInitKF"] = _autoInitKF;
    j["initJerkAngAcc"] = _initJerkAngAcc;
    j["kfInitialized"] = _kfInitialized;
    j["averageEndTime"] = _averageEndTime;

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
}

bool NAV::ImuFusion::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _numStates = _numStatesEst + static_cast<uint8_t>((_nInputPins - 1) * _numStatesPerPin);

    _kalmanFilter = KalmanFilter{ _numStates, _numMeasurements };

    _kalmanFilter.setZero();
    _imuRotations_accel.clear();
    _imuRotations_gyro.clear();
    _processNoiseVariances.clear();
    _measurementNoiseVariances.clear();
    _cumulatedImuObs.clear();
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
        nm::DeleteInputPin(inputPins.back().id);
        nm::DeleteOutputPin(outputPins.back().id);
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
    _kfInitialized = false;

    for (size_t pinIndex = 0; pinIndex < _pinData.size(); pinIndex++)
    {
        if (!nm::FindConnectedLinkToInputPin(inputPins.at(pinIndex).id))
        {
            LOG_INFO("Fewer links than input pins - Consider deleting pins that are not connected to limit KF matrices to the necessary size.");
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
    if (!_autoInitKF)
    {
        std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> initValues = initializeKalmanFilterManually();

        _kalmanFilter.x.block<3, 1>(0, 0) = initValues.first[0];
        _kalmanFilter.x.block<3, 1>(3, 0) = initValues.first[1];
        _kalmanFilter.x.block<3, 1>(6, 0) = initValues.first[2];
        _kalmanFilter.x.block<3, 1>(9, 0) = initValues.first[3];
        for (uint32_t pinIndex = 0; pinIndex < _nInputPins - 1UL; ++pinIndex)
        {
            auto containerIndex = 4 + 2 * pinIndex;
            _kalmanFilter.x.block<3, 1>(12 + 6 * pinIndex, 0) = initValues.first[containerIndex];
            _kalmanFilter.x.block<3, 1>(15 + 6 * pinIndex, 0) = initValues.first[1 + containerIndex];
        }

        // hard-coded - long time measurements (kept for reference) ################################################################################
        // Eigen::VectorXd bla = Eigen::VectorXd::Zero(24, 1);
        // bla << 0.1250, -0.1615, 0.0194, -0.1434, 0.2279, -2.5783, -0.0132, 0.0117, 0.0245, -0.2762, 0.4848, -1.5336, 0.0213, -0.0320, 0.0397, -0.1588, 0.2588, -2.9022, 0.0091, -0.0055, 0.0183, -0.2413, 0.0189, -3.6849; // entire time series
        // bla << 0.1250, -0.1616, 0.0192, -0.1447, 0.2291, -2.5865, -0.0130, 0.0117, 0.0237, -0.2725, 0.4880, -1.5728, 0.0223, -0.0330, 0.0394, -0.1571, 0.2600, -2.9525, 0.0098, -0.0060, 0.0185, -0.2440, 0.0256, -3.7012; // 1 second
        // _kalmanFilter.x.block<24, 1>(12, 0) = bla;
        LOG_DEBUG("kalmanFilter.x = {}", _kalmanFilter.x.transpose());
        // hard-coded - long time measurements (kept for reference) ################################################################################

        _kalmanFilter.P = initialErrorCovarianceMatrix_P0(initValues.second);
        LOG_DEBUG("kalmanFilter.P =\n{}", _kalmanFilter.P);
        _kalmanFilter.Phi = initialStateTransitionMatrix_Phi(dtInit);
        LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
        processNoiseMatrix_Q(_kalmanFilter.Q, dtInit);
        LOG_DATA("kalmanFilter.Q =\n{}", _kalmanFilter.Q);
    }
}

void NAV::ImuFusion::recvSignal(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    auto imuObs = std::static_pointer_cast<const ImuObs>(nodeData);

    if (!imuObs->insTime.has_value() && !imuObs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime/timeSinceStartup)", nameId());
        return;
    }

    if (_latestTimestamp.empty())
    {
        // Initial time step for KF prediction
        InsTime dt_init = InsTime{ 0, 0, 0, 0, 0, 1.0 / _imuFrequency };
        _latestTimestamp = InsTime{ 0, 0, 0, 0, 0, (imuObs->insTime.value() - dt_init).count() };

        // Time until averaging ends and filtering starts (for auto-init of KF)
        _avgEndTime = imuObs->insTime.value() + std::chrono::milliseconds(static_cast<int>(1e3 * _averageEndTime));
    }

    // Predict states over the time difference between the latest signal and the one before
    auto dt = static_cast<double>((imuObs->insTime.value() - _latestTimestamp).count());
    _latestTimestamp = imuObs->insTime.value();
    LOG_DATA("dt = {}", dt);

    stateTransitionMatrix_Phi(_kalmanFilter.Phi, dt);
    LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
    processNoiseMatrix_Q(_kalmanFilter.Q, dt);
    LOG_DATA("kalmanFilter.Q =\n{}", _kalmanFilter.Q);

    if (_checkKalmanMatricesRanks)
    {
        if (nm::FindConnectedLinkToInputPin(inputPins.at(_pinData.size() - 1).id))
        {
            auto rank = _kalmanFilter.P.fullPivLu().rank();
            if (rank != _kalmanFilter.P.rows())
            {
                LOG_WARN("{}: P.rank = {}", nameId(), rank);
            }
        }
    }

    if (Link* link = nm::FindLink(linkId))
    {
        size_t pinIndex = pinIndexFromId(link->endPinId);

        // Read sensor rotation info from 'imuObs'
        if (std::isnan(_imuRotations_accel[pinIndex](0, 0)))
        {
            // Rotation matrix of the accelerometer platform to body frame
            auto DCM_accel = imuObs->imuPos.b_quatAccel_p().toRotationMatrix();

            _imuRotations_accel[pinIndex] = DCM_accel;
        }
        if (std::isnan(_imuRotations_gyro[pinIndex](0, 0)))
        {
            // Rotation matrix of the gyro platform to body frame
            auto DCM_gyro = imuObs->imuPos.b_quatGyro_p().toRotationMatrix();

            _imuRotations_gyro[pinIndex] = DCM_gyro;
        }

        // Initialize H with mounting angles (DCM) of the sensor that provided the latest measurement
        auto DCM_accel = _imuRotations_accel.at(pinIndex);
        LOG_DATA("DCM_accel =\n{}", DCM_accel);
        auto DCM_gyro = _imuRotations_gyro.at(pinIndex);
        LOG_DATA("DCM_gyro =\n{}", DCM_gyro);

        // Initialize '_imuPos' of the combined solution - that of the reference sensor
        if (!_imuPosSet && pinIndex == 0)
        {
            this->_imuPos = imuObs->imuPos;
            _imuPosSet = true;
        }

        _kalmanFilter.H = designMatrix_H(DCM_accel, DCM_gyro, pinIndex);
        LOG_DATA("kalmanFilter.H =\n", _kalmanFilter.H);

        measurementNoiseMatrix_R(_kalmanFilter.R, pinIndex);
        LOG_DATA("{}: kalmanFilter.R =\n{}", nameId(), _kalmanFilter.R);

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
            if (imuObs->insTime.value() < _avgEndTime)
            {
                _cumulatedImuObs.push_back(imuObs);
                _cumulatedPinIds.push_back(pinIndex);
            }
            else
            {
                initializeKalmanFilterAuto();
            }
        }
        else
        {
            combineSignals(imuObs);
        }
    }
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
        if (nm::FindConnectedLinkToInputPin(inputPins.at(_pinData.size() - 1).id))
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

Eigen::MatrixXd NAV::ImuFusion::initialErrorCovarianceMatrix_P0(std::vector<Eigen::Vector3d>& initCovariances) const
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(_numStates, _numStates);

    P.block<3, 3>(0, 0).diagonal() = initCovariances[0]; // initial covariance of the angular rate
    P.block<3, 3>(3, 3).diagonal() = initCovariances[1]; // initial covariance of the angular acceleration
    P.block<3, 3>(6, 6).diagonal() = initCovariances[2]; // initial covariance of the acceleration
    P.block<3, 3>(9, 9).diagonal() = initCovariances[3]; // initial covariance of the jerk

    for (uint32_t i = 12; i < _numStates - 1UL; i += 3)
    {
        size_t j = 4 + (i - 12) / 3; // access bias variances for each sensor from the fifth element onwards
        P.block<3, 3>(i, i).diagonal() = initCovariances[j];
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

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> NAV::ImuFusion::initializeKalmanFilterManually()
{
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> initVectors; // contains init values for all state vectors
    initVectors.first.resize(6 + 2 * _nInputPins);

    // -------------------------------------------- State vector x -----------------------------------------------
    // Initial angular rate in [rad/s]
    if (_pinData[0].initAngularRateUnit == PinData::AngRateUnit::deg_s)
    {
        initVectors.first[0] = deg2rad(_pinData[0].initAngularRate);
    }
    if (_pinData[0].initAngularRateUnit == PinData::AngRateUnit::rad_s)
    {
        initVectors.first[0] = _pinData[0].initAngularRate;
    }

    // Initial acceleration in [m/s²]
    if (_pinData[0].initAccelerationUnit == PinData::AccelerationUnit::m_s2)
    {
        initVectors.first[1] = _pinData[0].initAcceleration;
    }

    // Initial angular acceleration in [rad/s²]
    if (_pinData[0].initAngularAccUnit == PinData::AngularAccUnit::deg_s2)
    {
        initVectors.first[2] = deg2rad(_pinData[0].initAngularAcc);
    }
    if (_pinData[0].initAngularAccUnit == PinData::AngularAccUnit::rad_s2)
    {
        initVectors.first[2] = _pinData[0].initAngularAcc;
    }

    // Initial jerk in [m/s³]
    if (_pinData[0].initJerkUnit == PinData::JerkUnit::m_s3)
    {
        initVectors.first[3] = _pinData[0].initJerk;
    }

    // Initial bias of the angular rate in [rad/s]
    for (size_t pinIndex = 0; pinIndex < _nInputPins - 1UL; ++pinIndex)
    {
        if (_pinData[pinIndex].initAngularRateBiasUnit == PinData::AngRateUnit::deg_s)
        {
            initVectors.first[4 + 2 * pinIndex] = _pinData[1 + pinIndex].initAngularRateBias.array();
        }
        else if (_pinData[pinIndex].initAngularRateBiasUnit == PinData::AngRateUnit::rad_s)
        {
            initVectors.first[4 + 2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].initAngularRateBias).array();
        }

        // Initial bias of the acceleration in [m/s²]
        if (_pinData[pinIndex].initAccelerationBiasUnit == PinData::AccelerationUnit::m_s2)
        {
            initVectors.first[5 + 2 * pinIndex] = _pinData[1 + pinIndex].initAccelerationBias.array();
        }
    }

    // ------------------------------------------------------ Error covariance matrix P --------------------------------------------------------
    initVectors.second.resize(6 + 2 * _nInputPins);

    // Initial Covariance of the angular rate in [rad²/s²]
    if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
    {
        initVectors.second[0] = _pinData[0].initCovarianceAngularRate;
    }
    else if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
    {
        initVectors.second[0] = deg2rad(_pinData[0].initCovarianceAngularRate);
    }
    else if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad_s)
    {
        initVectors.second[0] = _pinData[0].initCovarianceAngularRate.array().pow(2);
    }
    else if (_pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg_s)
    {
        initVectors.second[0] = deg2rad(_pinData[0].initCovarianceAngularRate).array().pow(2);
    }

    // Initial Covariance of the angular acceleration in [(rad^2)/(s^4)]
    if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::rad2_s4)
    {
        initVectors.second[1] = _pinData[0].initCovarianceAngularAcc;
    }
    else if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::deg2_s4)
    {
        initVectors.second[1] = deg2rad(_pinData[0].initCovarianceAngularAcc);
    }
    else if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::rad_s2)
    {
        initVectors.second[1] = _pinData[0].initCovarianceAngularAcc.array().pow(2);
    }
    else if (_pinData[0].initCovarianceAngularAccUnit == PinData::AngularAccVarianceUnit::deg_s2)
    {
        initVectors.second[1] = deg2rad(_pinData[0].initCovarianceAngularAcc).array().pow(2);
    }

    // Initial Covariance of the acceleration in [(m^2)/(s^4)]
    if (_pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4)
    {
        initVectors.second[2] = _pinData[0].initCovarianceAcceleration;
    }
    else if (_pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m_s2)
    {
        initVectors.second[2] = _pinData[0].initCovarianceAcceleration.array().pow(2);
    }

    // Initial Covariance of the jerk in [(m^2)/(s^6)]
    if (_pinData[0].initCovarianceJerkUnit == PinData::JerkVarianceUnit::m2_s6)
    {
        initVectors.second[3] = _pinData[0].initCovarianceJerk;
    }
    else if (_pinData[0].initCovarianceJerkUnit == PinData::JerkVarianceUnit::m_s3)
    {
        initVectors.second[3] = _pinData[0].initCovarianceJerk.array().pow(2);
    }

    // Initial Covariance of the bias of the angular acceleration in [(rad^2)/(s^4)]
    for (size_t pinIndex = 0; pinIndex < _nInputPins - 1UL; ++pinIndex)
    {
        if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
        {
            initVectors.second[4 + 2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAngRate;
        }
        else if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
        {
            initVectors.second[4 + 2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].initCovarianceBiasAngRate);
        }
        else if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad_s)
        {
            initVectors.second[4 + 2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAngRate.array().pow(2);
        }
        else if (_pinData[pinIndex].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg_s)
        {
            initVectors.second[4 + 2 * pinIndex] = deg2rad(_pinData[1 + pinIndex].initCovarianceBiasAngRate).array().pow(2);
        }

        // Initial Covariance of the bias of the jerk in [(m^2)/(s^6)]
        if (_pinData[pinIndex].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4)
        {
            initVectors.second[5 + 2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAcc;
        }
        else if (_pinData[pinIndex].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m_s2)
        {
            initVectors.second[5 + 2 * pinIndex] = _pinData[1 + pinIndex].initCovarianceBiasAcc.array().pow(2);
        }
    }

    return initVectors;
}

void NAV::ImuFusion::initializeKalmanFilterAuto()
{
    std::vector<std::vector<std::shared_ptr<const NAV::ImuObs>>> sensorMeasurements; // pinIndex / msgIndex(imuObs)
    sensorMeasurements.resize(_nInputPins);

    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> initVectors; // contains init values for all state vectors
    initVectors.first.resize(6 + 2 * _nInputPins);                                     // state vector x
    initVectors.second.resize(6 + 2 * _nInputPins);                                    // error covariance matrix P

    // Split cumulated imuObs into vectors for each sensor
    for (size_t msgIndex = 0; msgIndex < _cumulatedImuObs.size(); msgIndex++)
    {
        sensorMeasurements[_cumulatedPinIds[msgIndex]].push_back(_cumulatedImuObs[msgIndex]); // 'push_back' instead of 'resize()' and 'operator[]' since number of msgs of a certain pin are not known in advance
    }

    std::vector<std::vector<std::vector<double>>> sensorComponents; // pinIndex / axisIndex / msgIndex(double)
    sensorComponents.resize(_nInputPins);

    for (size_t pinIndex = 0; pinIndex < _nInputPins; pinIndex++) // loop thru connected sensors
    {
        sensorComponents[pinIndex].resize(_numMeasurements);
        for (size_t axisIndex = 0; axisIndex < _numMeasurements; axisIndex++) // loop thru the 6 measurements: AccX, GyroX, AccY, GyroY, AccZ, GyroZ
        {
            for (size_t msgIndex = 0; msgIndex < sensorMeasurements[pinIndex].size(); msgIndex++) // loop thru the msg of each measurement axis
            {
                if (axisIndex < 3) // Accelerations X/Y/Z
                {
                    sensorComponents[pinIndex][axisIndex].push_back(sensorMeasurements[pinIndex][msgIndex]->accelUncompXYZ.value()[static_cast<uint32_t>(axisIndex)]);
                }
                else // Gyro X/Y/Z
                {
                    sensorComponents[pinIndex][axisIndex].push_back(sensorMeasurements[pinIndex][msgIndex]->gyroUncompXYZ.value()[static_cast<uint32_t>(axisIndex - 3)]);
                }
            }
        }
    }

    // --------------------------- Averaging single measurements of each sensor ------------------------------
    // Accelerations X/Y/Z (pos. 6,7,8 in state vector) - init value is mean of reference sensor, i.e. pinIndex = 0
    initVectors.first[2] = mean(sensorComponents[0], 0);
    // Jerk X/Y/Z
    initVectors.first[3] = Eigen::Vector3d::Zero();
    // Angular Rate X/Y/Z (pos. 0,1,2 in state vector) - init value is mean of reference sensor, i.e. pinIndex = 0
    initVectors.first[0] = mean(sensorComponents[0], 3);
    // Angular Acceleration X/Y/Z
    initVectors.first[1] = Eigen::Vector3d::Zero();

    // Bias-inits
    for (size_t pinIndex = 0; pinIndex < _nInputPins - 1; pinIndex++) // _nInputPins - 1 since there are only relative biases
    {
        auto stateIndex = 4 + 2 * pinIndex;                                                                 // 4 states are Acceleration, Jerk, Angular Rate, Angular Acceleration (see above), plus 2 bias states per sensor
        initVectors.first[stateIndex + 1] = mean(sensorComponents[pinIndex + 1], 0) - initVectors.first[2]; // Acceleration biases
        initVectors.first[stateIndex] = mean(sensorComponents[pinIndex + 1], 3) - initVectors.first[0];     // Angular rate biases
    }

    // -------------------------------------- Variance of each sensor ----------------------------------------
    // Acceleration variances X/Y/Z (pos. 6,7,8 on diagonal of P matrix) - init value is variance of reference sensor, i.e. pinIndex = 0
    initVectors.second[2] = variance(sensorComponents[0], 0);
    // Angular Rate variances X/Y/Z (pos. 0,1,2 on diagonal of P matrix) - init value is variance of reference sensor, i.e. pinIndex = 0
    initVectors.second[0] = variance(sensorComponents[0], 3);

    if (_initJerkAngAcc)
    {
        // Jerk variances X/Y/Z
        initVectors.second[3] = initVectors.second[2];
        // Angular Acceleration variances X/Y/Z
        initVectors.second[1] = initVectors.second[0];
    }
    else
    {
        // Jerk variances X/Y/Z
        initVectors.second[3] = Eigen::Vector3d::Zero();
        // Angular Acceleration variances X/Y/Z
        initVectors.second[1] = Eigen::Vector3d::Zero();
    }

    // P-matrix bias inits
    for (size_t pinIndex = 0; pinIndex < _nInputPins - 1; pinIndex++) // _nInputPins - 1 since there are only relative biases
    {
        auto stateIndex = 4 + 2 * pinIndex;                                               // 4 states are Acceleration, Jerk, Angular Rate, Angular Acceleration (see above), plus 2 bias states per sensor
        initVectors.second[stateIndex + 1] = variance(sensorComponents[pinIndex + 1], 0); // Acceleration biases
        initVectors.second[stateIndex] = variance(sensorComponents[pinIndex + 1], 3);     // Angular rate biases

        // Choose the bigger one of the two variances, i.e. of sensor #'pinIndex' and the reference sensor #0 (since bias is the difference of these two sensors)
        for (int axisIndex = 0; axisIndex < 3; axisIndex++)
        {
            // Acceleration variance
            if (initVectors.second[stateIndex + 1](axisIndex) < initVectors.second[2](axisIndex))
            {
                initVectors.second[stateIndex + 1](axisIndex) = initVectors.second[2](axisIndex);
            }
            // Angular rate variance
            if (initVectors.second[stateIndex](axisIndex) < initVectors.second[0](axisIndex))
            {
                initVectors.second[stateIndex](axisIndex) = initVectors.second[0](axisIndex);
            }
        }
    }

    _kalmanFilter.x.block<3, 1>(0, 0) = initVectors.first[0];
    _kalmanFilter.x.block<3, 1>(3, 0) = initVectors.first[1];
    _kalmanFilter.x.block<3, 1>(6, 0) = initVectors.first[2];
    _kalmanFilter.x.block<3, 1>(9, 0) = initVectors.first[3];
    for (uint32_t pinIndex = 0; pinIndex < _nInputPins - 1UL; ++pinIndex)
    {
        auto containerIndex = 4 + 2 * pinIndex;
        _kalmanFilter.x.block<3, 1>(12 + 6 * pinIndex, 0) = initVectors.first[containerIndex];
        _kalmanFilter.x.block<3, 1>(15 + 6 * pinIndex, 0) = initVectors.first[1 + containerIndex];
    }

    LOG_DEBUG("kalmanFilter.x = {}", _kalmanFilter.x.transpose());
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(initVectors.second);
    LOG_DEBUG("kalmanFilter.P =\n{}", _kalmanFilter.P);
    _kalmanFilter.Phi = initialStateTransitionMatrix_Phi(1.0 / _imuFrequency);
    LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
    processNoiseMatrix_Q(_kalmanFilter.Q, 1.0 / _imuFrequency);
    LOG_DATA("kalmanFilter.Q =\n{}", _kalmanFilter.Q);

    // Start Kalman Filter
    _kfInitialized = true;
}

Eigen::Vector3d NAV::ImuFusion::mean(const std::vector<std::vector<double>>& sensorType, size_t containerPos)
{
    Eigen::Vector3d meanVector = Eigen::Vector3d::Zero();

    for (size_t axisIndex = 0; axisIndex < 3; axisIndex++)
    {
        meanVector(static_cast<int>(axisIndex)) = std::accumulate(sensorType[axisIndex + containerPos].begin(), sensorType[axisIndex + containerPos].end(), 0.) / static_cast<double>(sensorType[axisIndex + containerPos].size());
    }

    return meanVector;
}

Eigen::Vector3d NAV::ImuFusion::variance(const std::vector<std::vector<double>>& sensorType, size_t containerPos)
{
    Eigen::Vector3d varianceVector = Eigen::Vector3d::Zero();

    auto means = mean(sensorType, containerPos); // mean values for each axis

    for (size_t axisIndex = 0; axisIndex < 3; axisIndex++)
    {
        auto N = sensorType.at(axisIndex + containerPos).size(); // Number of msgs along the specific axis

        std::vector<double> absolSquared(N, 0.); // Inner part of the variance calculation (squared absolute values)

        for (size_t msgIndex = 0; msgIndex < N; msgIndex++)
        {
            absolSquared[msgIndex] = std::pow(std::abs(sensorType[axisIndex + containerPos][msgIndex] - means(static_cast<int>(axisIndex))), 2);
        }

        varianceVector(static_cast<int>(axisIndex)) = (1. / (static_cast<double>(N) - 1.)) * std::accumulate(absolSquared.begin(), absolSquared.end(), 0.);
    }

    return varianceVector;
}