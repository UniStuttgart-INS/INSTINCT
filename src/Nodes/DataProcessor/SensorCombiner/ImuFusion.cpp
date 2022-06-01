#include "ImuFusion.hpp"

#include "util/Logger.hpp"

#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include <imgui_internal.h>
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "util/Json.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

namespace NAV
{
/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const ImuFusion::PinData::SensorData& data)
{
    j = json{
        { "displayName", data.displayName },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, ImuFusion::PinData::SensorData& data)
{
    if (j.contains("displayName"))
    {
        j.at("displayName").get_to(data.displayName);
    }
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const ImuFusion::PinData& data)
{
    j = json{
        { "dataIdentifier", data.dataIdentifier },
        { "size", data.size },
        { "sensorData", data.sensorData },
        { "pinType", data.pinType },
        { "stride", data.stride },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, ImuFusion::PinData& data)
{
    if (j.contains("dataIdentifier"))
    {
        j.at("dataIdentifier").get_to(data.dataIdentifier);
    }
    if (j.contains("size"))
    {
        j.at("size").get_to(data.size);
    }
    if (j.contains("sensorData"))
    {
        j.at("sensorData").get_to(data.sensorData);
        for (auto& sensorData : data.sensorData)
        {
            sensorData.buffer = ScrollingBuffer<double>(static_cast<size_t>(data.size));
        }
    }
    if (j.contains("pinType"))
    {
        j.at("pinType").get_to(data.pinType);
    }
    if (j.contains("stride"))
    {
        j.at("stride").get_to(data.stride);
    }
}
} // namespace NAV

NAV::ImuFusion::ImuFusion()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 934, 586 };

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

            bool selectablePinDummy = false;
            ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, size_t(id)).c_str(), &selectablePinDummy);

            if (inputPins.size() > 2) // Minimum # of pins for the fusion to make sense is two
            {
                ImGui::TableNextColumn(); // Delete
                if (!(pinIndex == 0))     // Don't delete Pin 1, it's the reference for all other sensor (biases) that follow
                {
                    if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                    {
                        nm::DeleteInputPin(inputPins.at(pinIndex).id);
                        nm::DeleteOutputPin(outputPins.at(pinIndex).id);
                        _pinData.erase(_pinData.begin() + static_cast<int64_t>(pinIndex));
                        _initCovarianceBiasAngRate.erase(_initCovarianceBiasAngRate.begin() + static_cast<int64_t>(pinIndex - 1));
                        _initCovarianceBiasAngRateUnit.erase(_initCovarianceBiasAngRateUnit.begin() + static_cast<int64_t>(pinIndex - 1));
                        _initCovarianceBiasAcc.erase(_initCovarianceBiasAcc.begin() + static_cast<int64_t>(pinIndex - 1));
                        _initCovarianceBiasAccUnit.erase(_initCovarianceBiasAccUnit.begin() + static_cast<int64_t>(pinIndex - 1));
                        _varBiasAngRateNoise.erase(_varBiasAngRateNoise.begin() + static_cast<int64_t>(pinIndex - 1));
                        _varBiasAngRateNoiseUnit.erase(_varBiasAngRateNoiseUnit.begin() + static_cast<int64_t>(pinIndex - 1));
                        _varBiasAccelerationNoise.erase(_varBiasAccelerationNoise.begin() + static_cast<int64_t>(pinIndex - 1));
                        _varBiasAccelerationNoiseUnit.erase(_varBiasAccelerationNoiseUnit.begin() + static_cast<int64_t>(pinIndex - 1));
                        _measurementUncertaintyAngularRate.erase(_measurementUncertaintyAngularRate.begin() + static_cast<int64_t>(pinIndex - 1));
                        _measurementUncertaintyAngularRateUnit.erase(_measurementUncertaintyAngularRateUnit.begin() + static_cast<int64_t>(pinIndex - 1));
                        _measurementUncertaintyAcceleration.erase(_measurementUncertaintyAcceleration.begin() + static_cast<int64_t>(pinIndex - 1));
                        _measurementUncertaintyAccelerationUnit.erase(_measurementUncertaintyAccelerationUnit.begin() + static_cast<int64_t>(pinIndex - 1));
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
        ImGui::TableNextColumn();                                          // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str())) // TODO: increment only if Pin is connected
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

    constexpr float columnWidth{ 130.0F };

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
                                                           _initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::rad2_s2
                                                                   || _initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAngularRate.data(), reinterpret_cast<int*>(&_initCovarianceAngularRateUnit), "(rad/s)^2\0"
                                                                                                                                                                   "rad/s\0"
                                                                                                                                                                   "(deg/s)^2\0"
                                                                                                                                                                   "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAngularRate changed to {}", nameId(), _initCovarianceAngularRate);
            LOG_DEBUG("{}: InitCovarianceAngularRateUnit changed to {}", nameId(), _initCovarianceAngularRateUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular acceleration covariance ({})##{}",
                                                           _initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::rad2_s4
                                                                   || _initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::deg2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAngularAcc.data(), reinterpret_cast<int*>(&_initCovarianceAngularAccUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                 "rad/s^2\0"
                                                                                                                                                                 "(deg^2)/(s^4)\0"
                                                                                                                                                                 "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAngularAcc changed to {}", nameId(), _initCovarianceAngularAcc);
            LOG_DEBUG("{}: InitCovarianceAngularAccUnit changed to {}", nameId(), _initCovarianceAngularAccUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration covariance ({})##{}",
                                                           _initCovarianceAccelerationUnit == InitCovarianceAccelerationUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAcceleration.data(), reinterpret_cast<int*>(&_initCovarianceAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                     "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAcceleration changed to {}", nameId(), _initCovarianceAcceleration);
            LOG_DEBUG("{}: InitCovarianceAccelerationUnit changed to {}", nameId(), _initCovarianceAccelerationUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk covariance ({})##{}",
                                                           _initCovarianceJerkUnit == InitCovarianceJerkUnit::m2_s6
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceJerk.data(), reinterpret_cast<int*>(&_initCovarianceJerkUnit), "(m^2)/(s^6)\0"
                                                                                                                                                     "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceJerk changed to {}", nameId(), _initCovarianceJerk);
            LOG_DEBUG("{}: InitCovarianceJerkUnit changed to {}", nameId(), _initCovarianceJerkUnit);
            flow::ApplyChanges();
        }

        for (size_t i = 0; i < _nInputPins - 1; ++i)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate bias covariance of sensor {} ({})##{}", i + 2,
                                                               _initCovarianceBiasAngRateUnit[i] == InitCovarianceBiasAngRateUnit::rad2_s2
                                                                       || _initCovarianceBiasAngRateUnit[i] == InitCovarianceBiasAngRateUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceBiasAngRate.at(i).data(), reinterpret_cast<int*>(&_initCovarianceBiasAngRateUnit[i]), "(rad^2)/(s^2)\0"
                                                                                                                                                                                "rad/s\0"
                                                                                                                                                                                "(deg^2)/(s^2)\0"
                                                                                                                                                                                "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasAngRate changed to {}", nameId(), _initCovarianceBiasAngRate.at(i));
                LOG_DEBUG("{}: InitCovarianceBiasAngRateUnit changed to {}", nameId(), _initCovarianceBiasAngRateUnit[i]);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration bias covariance of sensor {} ({})##{}", i + 2,
                                                               _initCovarianceBiasAccUnit[i] == InitCovarianceBiasAccUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceBiasAcc.at(i).data(), reinterpret_cast<int*>(&_initCovarianceBiasAccUnit[i]), "(m^2)/(s^4)\0"
                                                                                                                                                                        "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasAcc changed to {}", nameId(), _initCovarianceBiasAcc.at(i));
                LOG_DEBUG("{}: InitCovarianceBiasAccUnit changed to {}", nameId(), _initCovarianceBiasAccUnit[i]);
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
                                                           _varAngularAccNoiseUnit == VarAngularAccNoiseUnit::rad2_s4
                                                                   || _varAngularAccNoiseUnit == VarAngularAccNoiseUnit::deg2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _varAngularAccNoise.data(), reinterpret_cast<int*>(&_varAngularAccNoiseUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                     "rad/s^2\0"
                                                                                                                                                     "(deg^2)/(s^4)\0"
                                                                                                                                                     "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varAngularAccNoise changed to {}", nameId(), _varAngularAccNoise.transpose());
            LOG_DEBUG("{}: varAngularAccNoiseUnit changed to {}", nameId(), _varAngularAccNoiseUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the jerk ({})##{}",
                                                           _varJerkNoiseUnit == VarJerkNoiseUnit::m2_s6
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _varJerkNoise.data(), reinterpret_cast<int*>(&_varJerkNoiseUnit), "(m^2)/(s^6)\0"
                                                                                                                                         "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varJerkNoise changed to {}", nameId(), _varJerkNoise.transpose());
            LOG_DEBUG("{}: varJerkNoiseUnit changed to {}", nameId(), _varJerkNoiseUnit);
            flow::ApplyChanges();
        }

        for (size_t i = 0; i < _nInputPins - 1; ++i)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the bias of the angular rate of sensor {} ({})##{}", i + 2,
                                                               _varBiasAngRateNoiseUnit[i] == VarBiasAngRateNoiseUnit::rad2_s2
                                                                       || _varBiasAngRateNoiseUnit[i] == VarBiasAngRateNoiseUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                                   configWidth, unitWidth, _varBiasAngRateNoise.at(i).data(), reinterpret_cast<int*>(&_varBiasAngRateNoiseUnit[i]), "(rad/s)^2\0"
                                                                                                                                                                    "rad/s\0"
                                                                                                                                                                    "(deg/s)^2\0"
                                                                                                                                                                    "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific)) // FIXME: make '_varBiasAngRateNoiseUnit' a container, s.t. user can choose different units for each sensor
            {
                LOG_DEBUG("{}: varBiasAngRateNoise changed to {}", nameId(), _varBiasAngRateNoise.at(i).transpose());
                LOG_DEBUG("{}: varBiasAngRateNoiseUnit changed to {}", nameId(), _varBiasAngRateNoiseUnit[i]);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Process noise of the bias of the acceleration of sensor {} ({})##{}", i + 2,
                                                               _varBiasAccelerationNoiseUnit[i] == VarBiasAccelerationNoiseUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(), // FIXME: adapt config window number of sensors (if pin 3 is deleted, keep 1,2,4 instead of re-counting to 1,2,3)
                                                   configWidth, unitWidth, _varBiasAccelerationNoise.at(i).data(), reinterpret_cast<int*>(&_varBiasAccelerationNoiseUnit[i]), "(m^2)/(s^4)\0"
                                                                                                                                                                              "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific)) // FIXME: make '_varBiasAccelerationNoiseUnit' a container, s.t. user can choose different units for each sensor
            {
                LOG_DEBUG("{}: varBiasAccelerationNoise changed to {}", nameId(), _varBiasAccelerationNoise.at(i).transpose());
                LOG_DEBUG("{}: varBiasAccelerationNoiseUnit changed to {}", nameId(), _varBiasAccelerationNoiseUnit[i]);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        for (size_t i = 0; i < _nInputPins; ++i)
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Measurement uncertainty of the angular rate of sensor {} ({})##{}", i + 1,
                                                               _measurementUncertaintyAngularRateUnit[i] == MeasurementUncertaintyAngularRateUnit::rad2_s2
                                                                       || _measurementUncertaintyAngularRateUnit[i] == MeasurementUncertaintyAngularRateUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _measurementUncertaintyAngularRate.at(i).data(), reinterpret_cast<int*>(&_measurementUncertaintyAngularRateUnit[i]), "(rad/s)^2\0"
                                                                                                                                                                                                "rad/s\0"
                                                                                                                                                                                                "(deg/s)^2\0"
                                                                                                                                                                                                "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdevAngularAcc changed to {}", nameId(), _measurementUncertaintyAngularRate.at(i).transpose());
                LOG_DEBUG("{}: stdevAngularAccUnit changed to {}", nameId(), _measurementUncertaintyAngularRateUnit[i]);
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Measurement uncertainty of the acceleration of sensor {} ({})##{}", i + 1,
                                                               _measurementUncertaintyAccelerationUnit[i] == MeasurementUncertaintyAccelerationUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _measurementUncertaintyAcceleration.at(i).data(), reinterpret_cast<int*>(&_measurementUncertaintyAccelerationUnit[i]), "(m^2)/(s^4)\0"
                                                                                                                                                                                                  "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdevJerk changed to {}", nameId(), _measurementUncertaintyAcceleration.at(i).transpose());
                LOG_DEBUG("{}: stdevJerkUnit changed to {}", nameId(), _measurementUncertaintyAccelerationUnit[i]);
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
    j["pinData"] = _pinData;
    j["numStates"] = _numStates;
    j["numMeasurements"] = _numMeasurements;

    j["initCovarianceAngularRateUnit"] = _initCovarianceAngularRateUnit;
    j["initCovarianceAngularRate"] = _initCovarianceAngularRate;
    j["initCovarianceAngularAccUnit"] = _initCovarianceAngularAccUnit;
    j["initCovarianceAngularAcc"] = _initCovarianceAngularAcc;
    j["initCovarianceAccelerationUnit"] = _initCovarianceAccelerationUnit;
    j["initCovarianceAcceleration"] = _initCovarianceAcceleration;
    j["initCovarianceJerkUnit"] = _initCovarianceJerkUnit;
    j["initCovarianceJerk"] = _initCovarianceJerk;
    j["initCovarianceBiasAngRateUnit"] = _initCovarianceBiasAngRateUnit;
    j["initCovarianceBiasAngRate"] = _initCovarianceBiasAngRate;
    j["initCovarianceBiasAccUnit"] = _initCovarianceBiasAccUnit;
    j["initCovarianceBiasAcc"] = _initCovarianceBiasAcc;
    j["varAngularAccNoiseUnit"] = _varAngularAccNoiseUnit;
    j["varAngularAccNoise"] = _varAngularAccNoise;
    j["varJerkNoiseUnit"] = _varJerkNoiseUnit;
    j["varJerkNoise"] = _varJerkNoise;
    j["varBiasAngRateNoiseUnit"] = _varBiasAngRateNoiseUnit;
    j["varBiasAngRateNoise"] = _varBiasAngRateNoise;
    j["varBiasAccelerationNoiseUnit"] = _varBiasAccelerationNoiseUnit;
    j["varBiasAccelerationNoise"] = _varBiasAccelerationNoise;
    j["measurementUncertaintyAngularRate"] = _measurementUncertaintyAngularRate;
    j["measurementUncertaintyAcceleration"] = _measurementUncertaintyAcceleration;
    j["measurementUncertaintyAngularRateUnit"] = _measurementUncertaintyAngularRateUnit;
    j["measurementUncertaintyAngularRate"] = _measurementUncertaintyAngularRate;
    j["measurementUncertaintyAccelerationUnit"] = _measurementUncertaintyAccelerationUnit;
    j["measurementUncertaintyAcceleration"] = _measurementUncertaintyAcceleration;

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

        for (size_t inputPinIndex = 0; inputPinIndex < inputPins.size(); inputPinIndex++) //NOLINT(modernize-loop-convert)
        {
            inputPins.at(inputPinIndex).notifyFunc.clear();
        }
    }
    // -------------------------------------- 𝐏 Error covariance matrix -----------------------------------------
    if (j.contains("initCovarianceAngularRateUnit"))
    {
        j.at("initCovarianceAngularRateUnit").get_to(_initCovarianceAngularRateUnit);
    }
    if (j.contains("initCovarianceAngularRate"))
    {
        j.at("initCovarianceAngularRate").get_to(_initCovarianceAngularRate);
    }
    if (j.contains("initCovarianceAngularAccUnit"))
    {
        j.at("initCovarianceAngularAccUnit").get_to(_initCovarianceAngularAccUnit);
    }
    if (j.contains("initCovarianceAngularAcc"))
    {
        j.at("initCovarianceAngularAcc").get_to(_initCovarianceAngularAcc);
    }
    if (j.contains("initCovarianceAccelerationUnit"))
    {
        j.at("initCovarianceAccelerationUnit").get_to(_initCovarianceAccelerationUnit);
    }
    if (j.contains("initCovarianceAcceleration"))
    {
        j.at("initCovarianceAcceleration").get_to(_initCovarianceAcceleration);
    }
    if (j.contains("initCovarianceJerkUnit"))
    {
        j.at("initCovarianceJerkUnit").get_to(_initCovarianceJerkUnit);
    }
    if (j.contains("initCovarianceJerk"))
    {
        j.at("initCovarianceJerk").get_to(_initCovarianceJerk);
    }
    if (j.contains("initCovarianceBiasAngRateUnit"))
    {
        j.at("initCovarianceBiasAngRateUnit").get_to(_initCovarianceBiasAngRateUnit);
    }
    if (j.contains("initCovarianceBiasAngRate"))
    {
        j.at("initCovarianceBiasAngRate").get_to(_initCovarianceBiasAngRate);
    }
    if (j.contains("initCovarianceBiasAccUnit"))
    {
        j.at("initCovarianceBiasAccUnit").get_to(_initCovarianceBiasAccUnit);
    }
    if (j.contains("initCovarianceBiasAcc"))
    {
        j.at("initCovarianceBiasAcc").get_to(_initCovarianceBiasAcc);
    }

    // ------------------------------- 𝐐 System/Process noise covariance matrix ---------------------------------
    if (j.contains("varAngularAccNoiseUnit"))
    {
        j.at("varAngularAccNoiseUnit").get_to(_varAngularAccNoiseUnit);
    }
    if (j.contains("varAngularAccNoise"))
    {
        j.at("varAngularAccNoise").get_to(_varAngularAccNoise);
    }
    if (j.contains("varJerkNoiseUnit"))
    {
        j.at("varJerkNoiseUnit").get_to(_varJerkNoiseUnit);
    }
    if (j.contains("varJerkNoise"))
    {
        j.at("varJerkNoise").get_to(_varJerkNoise);
    }
    if (j.contains("varBiasAngRateNoiseUnit"))
    {
        j.at("varBiasAngRateNoiseUnit").get_to(_varBiasAngRateNoiseUnit);
    }
    if (j.contains("varBiasAngRateNoise"))
    {
        j.at("varBiasAngRateNoise").get_to(_varBiasAngRateNoise);
    }
    if (j.contains("varBiasAccelerationNoiseUnit"))
    {
        j.at("varBiasAccelerationNoiseUnit").get_to(_varBiasAccelerationNoiseUnit);
    }
    if (j.contains("varBiasAccelerationNoise"))
    {
        j.at("varBiasAccelerationNoise").get_to(_varBiasAccelerationNoise);
    }

    // -------------------------------- 𝐑 Measurement noise covariance matrix -----------------------------------
    if (j.contains("measurementUncertaintyAngularRateUnit"))
    {
        j.at("measurementUncertaintyAngularRateUnit").get_to(_measurementUncertaintyAngularRateUnit);
    }
    if (j.contains("measurementUncertaintyAngularRate"))
    {
        j.at("measurementUncertaintyAngularRate").get_to(_measurementUncertaintyAngularRate);
    }
    if (j.contains("measurementUncertaintyAccelerationUnit"))
    {
        j.at("measurementUncertaintyAccelerationUnit").get_to(_measurementUncertaintyAccelerationUnit);
    }
    if (j.contains("measurementUncertaintyAcceleration"))
    {
        j.at("measurementUncertaintyAcceleration").get_to(_measurementUncertaintyAcceleration);
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

    _latestTimestamp = InsTime{};

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
            nm::CreateOutputPin(this, fmt::format("ImuBiases {}1", outputPins.size() + 1).c_str(), Pin::Type::Flow, { NAV::ImuBiases::type() });
        }
    }
    while (inputPins.size() > _nInputPins) // TODO: while loop still necessary here? guiConfig also deletes pins
    {
        nm::DeleteInputPin(inputPins.back().id);
        nm::DeleteOutputPin(outputPins.back().id);
        _pinData.pop_back();
    }
    _initCovarianceBiasAngRate.resize(_nInputPins - 1);
    _initCovarianceBiasAngRateUnit.resize(_nInputPins - 1);
    _initCovarianceBiasAcc.resize(_nInputPins - 1);
    _initCovarianceBiasAccUnit.resize(_nInputPins - 1);
    _varBiasAngRateNoise.resize(_nInputPins - 1);
    _varBiasAngRateNoiseUnit.resize(_nInputPins - 1);
    _varBiasAccelerationNoise.resize(_nInputPins - 1);
    _varBiasAccelerationNoiseUnit.resize(_nInputPins - 1);
    _measurementUncertaintyAngularRate.resize(_nInputPins);
    _measurementUncertaintyAngularRateUnit.resize(_nInputPins);
    _measurementUncertaintyAcceleration.resize(_nInputPins);
    _measurementUncertaintyAccelerationUnit.resize(_nInputPins);
}

void NAV::ImuFusion::initializeKalmanFilter()
{
    LOG_TRACE("{}: called", nameId());

    _designMatrixInitialized = false;

    for (size_t pinIndex = 0; pinIndex < _pinData.size(); pinIndex++)
    {
        if (!nm::FindConnectedLinkToInputPin(inputPins.at(pinIndex).id))
        {
            LOG_INFO("Fewer links than input pins - Consider deleting pins that are not connected to limit KF matrices to the necessary size.");
        }
    }

    // ------------------------------------------------------ Error covariance matrix P --------------------------------------------------------

    // Initial Covariance of the angular rate in [rad²/s²]
    Eigen::Vector3d variance_angularRate = Eigen::Vector3d::Zero();
    if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::rad2_s2)
    {
        variance_angularRate = _initCovarianceAngularRate;
    }
    else if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::deg2_s2)
    {
        variance_angularRate = trafo::deg2rad(_initCovarianceAngularRate);
    }
    else if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::rad_s)
    {
        variance_angularRate = _initCovarianceAngularRate.array().pow(2);
    }
    else if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::deg_s)
    {
        variance_angularRate = trafo::deg2rad(_initCovarianceAngularRate).array().pow(2);
    }

    // Initial Covariance of the angular acceleration in [(rad^2)/(s^4)]
    Eigen::Vector3d variance_angularAcceleration = Eigen::Vector3d::Zero();
    if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::rad2_s4)
    {
        variance_angularAcceleration = _initCovarianceAngularAcc;
    }
    else if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::deg2_s4)
    {
        variance_angularAcceleration = trafo::deg2rad(_initCovarianceAngularAcc);
    }
    else if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::rad_s2)
    {
        variance_angularAcceleration = _initCovarianceAngularAcc.array().pow(2);
    }
    else if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::deg_s2)
    {
        variance_angularAcceleration = trafo::deg2rad(_initCovarianceAngularAcc).array().pow(2);
    }

    // Initial Covariance of the acceleration in [(m^2)/(s^4)]
    Eigen::Vector3d variance_acceleration = Eigen::Vector3d::Zero();
    if (_initCovarianceAccelerationUnit == InitCovarianceAccelerationUnit::m2_s4)
    {
        variance_acceleration = _initCovarianceAcceleration;
    }
    else if (_initCovarianceAccelerationUnit == InitCovarianceAccelerationUnit::m_s2)
    {
        variance_acceleration = _initCovarianceAcceleration.array().pow(2);
    }

    // Initial Covariance of the jerk in [(m^2)/(s^6)]
    Eigen::Vector3d variance_jerk = Eigen::Vector3d::Zero();
    if (_initCovarianceJerkUnit == InitCovarianceJerkUnit::m2_s6)
    {
        variance_jerk = _initCovarianceJerk;
    }
    else if (_initCovarianceJerkUnit == InitCovarianceJerkUnit::m_s3)
    {
        variance_jerk = _initCovarianceJerk.array().pow(2);
    }

    // Initial Covariance of the bias of the angular acceleration in [(rad^2)/(s^4)]
    _biasCovariances.resize(2 * _nInputPins);
    for (size_t i = 0; i < _nInputPins - 1UL; ++i)
    {
        if (_initCovarianceBiasAngRateUnit[i] == InitCovarianceBiasAngRateUnit::rad2_s2)
        {
            _biasCovariances[2 * i] = _initCovarianceBiasAngRate.at(i);
        }
        else if (_initCovarianceBiasAngRateUnit[i] == InitCovarianceBiasAngRateUnit::deg2_s2)
        {
            _biasCovariances[2 * i] = trafo::deg2rad(_initCovarianceBiasAngRate.at(i));
        }
        else if (_initCovarianceBiasAngRateUnit[i] == InitCovarianceBiasAngRateUnit::rad_s)
        {
            _biasCovariances[2 * i] = _initCovarianceBiasAngRate.at(i).array().pow(2);
        }
        else if (_initCovarianceBiasAngRateUnit[i] == InitCovarianceBiasAngRateUnit::deg_s)
        {
            _biasCovariances[2 * i] = trafo::deg2rad(_initCovarianceBiasAngRate.at(i)).array().pow(2);
        }

        // Initial Covariance of the bias of the jerk in [(m^2)/(s^6)]
        if (_initCovarianceBiasAccUnit[i] == InitCovarianceBiasAccUnit::m2_s4)
        {
            _biasCovariances[1 + 2 * i] = _initCovarianceBiasAcc.at(i);
        }
        else if (_initCovarianceBiasAccUnit[i] == InitCovarianceBiasAccUnit::m_s2)
        {
            _biasCovariances[1 + 2 * i] = _initCovarianceBiasAcc.at(i).array().pow(2);
        }
    }

    // ------------------------------------------------------- Process noise matrix Q ----------------------------------------------------------
    _processNoiseVariances.resize(2 * _nInputPins);

    // 𝜎_AngAcc Standard deviation of the noise on the angular acceleration state [rad/s²]
    switch (_varAngularAccNoiseUnit)
    {
    case VarAngularAccNoiseUnit::rad2_s4:
        _processNoiseVariances[0] = _varAngularAccNoise;
        break;
    case VarAngularAccNoiseUnit::deg2_s4:
        _processNoiseVariances[0] = trafo::deg2rad(_varAngularAccNoise);
        break;
    case VarAngularAccNoiseUnit::deg_s2:
        _processNoiseVariances[0] = trafo::deg2rad(_varAngularAccNoise).array().pow(2);
        break;
    case VarAngularAccNoiseUnit::rad_s2:
        _processNoiseVariances[0] = _varAngularAccNoise.array().pow(2);
        break;
    }

    // 𝜎_jerk Standard deviation of the noise on the jerk state [m/s³]
    switch (_varJerkNoiseUnit)
    {
    case VarJerkNoiseUnit::m2_s6:
        _processNoiseVariances[1] = _varJerkNoise;
        break;
    case VarJerkNoiseUnit::m_s3:
        _processNoiseVariances[1] = _varJerkNoise.array().pow(2);
        break;
    }

    for (size_t i = 0; i < _nInputPins - 1; ++i)
    {
        // 𝜎_biasAngRate Standard deviation of the bias on the angular rate state [rad/s²]
        switch (_varBiasAngRateNoiseUnit[i])
        {
        case VarBiasAngRateNoiseUnit::rad2_s2:
            _processNoiseVariances[2 + 2 * i] = _varBiasAngRateNoise.at(i);
            break;
        case VarBiasAngRateNoiseUnit::deg2_s2:
            _processNoiseVariances[2 + 2 * i] = trafo::deg2rad(_varBiasAngRateNoise.at(i));
            break;
        case VarBiasAngRateNoiseUnit::deg_s:
            _processNoiseVariances[2 + 2 * i] = trafo::deg2rad(_varBiasAngRateNoise.at(i)).array().pow(2);
            break;
        case VarBiasAngRateNoiseUnit::rad_s:
            _processNoiseVariances[2 + 2 * i] = _varBiasAngRateNoise.at(i).array().pow(2);
            break;
        }

        // 𝜎_biasAcceleration Standard deviation of the noise on the acceleration state [m/s³]
        switch (_varBiasAccelerationNoiseUnit[i])
        {
        case VarBiasAccelerationNoiseUnit::m2_s4:
            _processNoiseVariances[3 + 2 * i] = _varBiasAccelerationNoise.at(i);
            break;
        case VarBiasAccelerationNoiseUnit::m_s2:
            _processNoiseVariances[3 + 2 * i] = _varBiasAccelerationNoise.at(i).array().pow(2);
            break;
        }
    }

    // -------------------------------------------------- Measurement uncertainty matrix R -----------------------------------------------------
    _measurementNoiseVariances.resize(2 * _nInputPins);

    for (size_t i = 0; i < _nInputPins; ++i)
    {
        // Measurement uncertainty for the angular rate (Variance σ²) in [(rad/s)^2, (rad/s)^2, (rad/s)^2]
        switch (_measurementUncertaintyAngularRateUnit[i])
        {
        case MeasurementUncertaintyAngularRateUnit::rad_s:
            _measurementNoiseVariances[2 * i] = (_measurementUncertaintyAngularRate.at(i)).array().pow(2);
            break;
        case MeasurementUncertaintyAngularRateUnit::deg_s:
            _measurementNoiseVariances[2 * i] = (trafo::deg2rad(_measurementUncertaintyAngularRate.at(i))).array().pow(2);
            break;
        case MeasurementUncertaintyAngularRateUnit::rad2_s2:
            _measurementNoiseVariances[2 * i] = _measurementUncertaintyAngularRate.at(i);
            break;
        case MeasurementUncertaintyAngularRateUnit::deg2_s2:
            _measurementNoiseVariances[2 * i] = trafo::deg2rad((_measurementUncertaintyAngularRate.at(i)).cwiseSqrt()).array().pow(2);
            break;
        }

        // Measurement uncertainty for the acceleration (Variance σ²) in [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
        switch (_measurementUncertaintyAccelerationUnit[i])
        {
        case MeasurementUncertaintyAccelerationUnit::m2_s4:
            _measurementNoiseVariances[1 + 2 * i] = _measurementUncertaintyAcceleration.at(i);
            break;
        case MeasurementUncertaintyAccelerationUnit::m_s2:
            _measurementNoiseVariances[1 + 2 * i] = (_measurementUncertaintyAcceleration.at(i)).array().pow(2);
            break;
        }
    }

    auto dtInit = 1.0 / _imuFrequency;

    // --------------------------------------------------------- KF Initializations ------------------------------------------------------------
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(variance_angularRate, variance_angularAcceleration, variance_acceleration, variance_jerk);
    LOG_DATA("kalmanFilter.P =\n{}", _kalmanFilter.P);
    _kalmanFilter.Phi = initialStateTransitionMatrix_Phi(dtInit);
    LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
    _kalmanFilter.Q = processNoiseMatrix_Q(_kalmanFilter.Q, dtInit);
    LOG_DATA("kalmanFilter.Q =\n{}", _kalmanFilter.Q);
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
    }

    // Predict states over the time difference between the latest signal and the one before
    auto dt = static_cast<double>((imuObs->insTime.value() - _latestTimestamp).count());
    _latestTimestamp = imuObs->insTime.value();
    LOG_DATA("dt = {}", dt);

    _kalmanFilter.Phi = stateTransitionMatrix_Phi(_kalmanFilter.Phi, dt);
    LOG_DATA("kalmanFilter.Phi =\n{}", _kalmanFilter.Phi);
    _kalmanFilter.Q = processNoiseMatrix_Q(_kalmanFilter.Q, dt);
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
        if (!_imuRotations_accel.contains(pinIndex))
        {
            // Do heavy calculations
            auto DCM_accel = imuObs->imuPos.b_quatAccel_p().toRotationMatrix();

            _imuRotations_accel.insert_or_assign(pinIndex, DCM_accel);
        }
        if (!_imuRotations_gyro.contains(pinIndex))
        {
            // Do heavy calculations
            auto DCM_gyro = imuObs->imuPos.b_quatGyro_p().toRotationMatrix();

            _imuRotations_gyro.insert_or_assign(pinIndex, DCM_gyro);
        }

        // Initialize H with mounting angles (DCM) of the sensor that provided the latest measurement
        auto DCM_accel = _imuRotations_accel.at(pinIndex);
        LOG_DATA("DCM_accel =\n{}", DCM_accel);
        auto DCM_gyro = _imuRotations_gyro.at(pinIndex);
        LOG_DATA("DCM_gyro =\n{}", DCM_gyro);

        _kalmanFilter.H = designMatrix_H(DCM_accel, DCM_gyro, pinIndex);
        LOG_DATA("kalmanFilter.H =\n", _kalmanFilter.H);

        _kalmanFilter.R = measurementNoiseMatrix_R(_kalmanFilter.R, pinIndex);
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
}

void NAV::ImuFusion::combineSignals(std::shared_ptr<const ImuObs>& imuObs)
{
    LOG_TRACE("{}: called", nameId());

    auto imuObsFiltered = std::make_shared<ImuObs>(this->_imuPos);
    auto imuRelativeBiases = std::make_shared<ImuBiases>();

    LOG_DATA("Estimated state before prediction: x =\n{}", _kalmanFilter.x);

    _kalmanFilter.predict();

    _kalmanFilter.z.block<3, 1>(0, 0) = *imuObs->gyroUncompXYZ;
    _kalmanFilter.z.block<3, 1>(3, 0) = *imuObs->accelUncompXYZ;

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

        imuRelativeBiases->b_biasGyro << _kalmanFilter.x(biasIndex, 0), _kalmanFilter.x(biasIndex + 1, 0), _kalmanFilter.x(biasIndex + 2, 0);
        imuRelativeBiases->b_biasAccel << _kalmanFilter.x(biasIndex + 3, 0), _kalmanFilter.x(biasIndex + 4, 0), _kalmanFilter.x(biasIndex + 5, 0);

        invokeCallbacks(OUTPUT_PORT_INDEX_BIAS, imuRelativeBiases);
    }
}

// #########################################################################################################################################
//                                                         Kalman Filter Matrices
// #########################################################################################################################################

Eigen::MatrixXd NAV::ImuFusion::initialStateTransitionMatrix_Phi(double& dt) const
{
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(_numStates, _numStates);

    Phi.diagonal().setOnes(); // constant part of states

    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk

    return Phi;
}

Eigen::MatrixXd NAV::ImuFusion::stateTransitionMatrix_Phi(Eigen::MatrixXd& Phi, double& dt)
{
    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk

    return Phi;
}

Eigen::MatrixXd NAV::ImuFusion::initialErrorCovarianceMatrix_P0(Eigen::Vector3d& varAngRate,
                                                                Eigen::Vector3d& varAngAcc,
                                                                Eigen::Vector3d& varAcc,
                                                                Eigen::Vector3d& varJerk) const
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

Eigen::MatrixXd NAV::ImuFusion::processNoiseMatrix_Q(Eigen::MatrixXd& Q, double& dt) const
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

    return Q;
}

Eigen::MatrixXd NAV::ImuFusion::designMatrix_H(Eigen::Matrix3d& DCM_accel, Eigen::Matrix3d& DCM_gyro, size_t pinIndex) const
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

Eigen::MatrixXd NAV::ImuFusion::measurementNoiseMatrix_R_adaptive(double alpha, Eigen::MatrixXd& R, Eigen::VectorXd& e, Eigen::MatrixXd& H, Eigen::MatrixXd& P)
{
    return alpha * R + (1.0 - alpha) * (e * e.transpose() + H * P * H.transpose());
}

Eigen::MatrixXd NAV::ImuFusion::measurementNoiseMatrix_R(Eigen::MatrixXd& R, size_t& pinIndex) const
{
    R.block<3, 3>(0, 0).diagonal() = _measurementNoiseVariances.at(2 * pinIndex);
    R.block<3, 3>(3, 3).diagonal() = _measurementNoiseVariances.at(2 * pinIndex + 1);

    return R;
}