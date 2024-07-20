// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "WiFiPositioning.hpp"

#include <algorithm>
#include <ranges>
#include <regex>

#include "util/Logger.hpp"
#include "util/Container/Vector.hpp"

#include "Navigation/Constants.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"
#include "NodeData/WiFi/WiFiPositioningSolution.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "Navigation/Math/LeastSquares.hpp"

NAV::WiFiPositioning::WiFiPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 575, 300 };

    updateNumberOfInputPins();

    nm::CreateOutputPin(this, NAV::WiFiPositioningSolution::type().c_str(), Pin::Type::Flow, { NAV::WiFiPositioningSolution::type() });
}

NAV::WiFiPositioning::~WiFiPositioning()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::WiFiPositioning::typeStatic()
{
    return "WiFiPositioning";
}

std::string NAV::WiFiPositioning::type() const
{
    return typeStatic();
}

std::string NAV::WiFiPositioning::category()
{
    return "Data Processor";
}

void NAV::WiFiPositioning::guiConfig()
{
    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();
    float configWidth = 380.0F * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 150.0F * gui::NodeEditorApplication::windowFontRatio();

    if (ImGui::Button(fmt::format("Add Input Pin##{}", size_t(id)).c_str()))
    {
        _nWifiInputPins++;
        LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nWifiInputPins);
        flow::ApplyChanges();
        updateNumberOfInputPins();
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("Delete Input Pin##{}", size_t(id)).c_str()))
    {
        _nWifiInputPins--;
        LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nWifiInputPins);
        flow::ApplyChanges();
        updateNumberOfInputPins();
    }

    ImGui::SetNextItemWidth(250 * gui::NodeEditorApplication::windowFontRatio());

    // ###########################################################################################################
    //                                        Frames
    // ###########################################################################################################
    if (_numOfDevices == 0)
    {
        if (ImGui::Combo(fmt::format("Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_frame), "ECEF\0LLA\0\0"))
        {
            switch (_frame)
            {
            case Frame::ECEF:
                LOG_DEBUG("{}: Frame changed to ECEF", nameId());
                break;
            case Frame::LLA:
                LOG_DEBUG("{}: Frame changed to LLA", nameId());
                break;
            }
        }

        flow::ApplyChanges();
    }

    if (ImGui::BeginTable("AccessPointInput", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        // Column headers
        ImGui::TableSetupColumn("MAC address", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        if (_frame == Frame::ECEF)
        {
            ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        }
        else if (_frame == Frame::LLA)
        {
            ImGui::TableSetupColumn("Latitude", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Longitude", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Altitude", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        }
        ImGui::TableSetupColumn("Bias", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        ImGui::TableSetupColumn("Scale", ImGuiTableColumnFlags_WidthFixed, columnWidth);

        // Automatic header row
        ImGui::TableHeadersRow();

        for (size_t rowIndex = 0; rowIndex < _numOfDevices; rowIndex++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();

            // MAC address validation
            std::regex macRegex("^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$");

            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputText(fmt::format("##Mac{}", rowIndex).c_str(), &_deviceMacAddresses.at(rowIndex), ImGuiInputTextFlags_None))
            {
                std::transform(_deviceMacAddresses.at(rowIndex).begin(), _deviceMacAddresses.at(rowIndex).end(), _deviceMacAddresses.at(rowIndex).begin(), ::toupper); // Convert to uppercase
                if (!std::regex_match(_deviceMacAddresses.at(rowIndex), macRegex))
                {
                    _deviceMacAddresses.at(rowIndex) = "00:00:00:00:00:00";
                    LOG_DEBUG("{}: Invalid MAC address", nameId());
                }
                else
                {
                    flow::ApplyChanges();
                }
            }
            if (_frame == Frame::ECEF)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputX{}", rowIndex).c_str(), &_devicePositions.at(rowIndex)[0], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputY{}", rowIndex).c_str(), &_devicePositions.at(rowIndex)[1], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputZ{}", rowIndex).c_str(), &_devicePositions.at(rowIndex)[2], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
            }
            else if (_frame == Frame::LLA)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDoubleL(fmt::format("##InputLat{}", rowIndex).c_str(), &_devicePositions.at(rowIndex)[0], -180, 180, 0.0, 0.0, "%.8f°"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDoubleL(fmt::format("##InputLon{}", rowIndex).c_str(), &_devicePositions.at(rowIndex)[1], -180, 180, 0.0, 0.0, "%.8f°"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputHeight{}", rowIndex).c_str(), &_devicePositions.at(rowIndex)[2], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##InputBias{}", rowIndex).c_str(), &_deviceBias.at(rowIndex), 0.0, 0.0, "%.4fm"))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##InputScale{}", rowIndex).c_str(), &_deviceScale.at(rowIndex), 0.0, 0.0, "%.4f"))
            {
                flow::ApplyChanges();
            }
        }
        ImGui::EndTable();
    }
    if (ImGui::Button(fmt::format("Add Device##{}", size_t(id)).c_str(), ImVec2(columnWidth * 2.1f, 0)))
    {
        _numOfDevices++;
        _deviceMacAddresses.push_back("00:00:00:00:00:00");
        _devicePositions.push_back(Eigen::Vector3d::Zero());
        _deviceBias.push_back(0.0);
        _deviceScale.push_back(0.0);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    if (ImGui::Button(fmt::format("Delete Device##{}", size_t(id)).c_str(), ImVec2(columnWidth * 2.1f, 0)))
    {
        if (_numOfDevices > 0)
        {
            _numOfDevices--;
            _deviceMacAddresses.pop_back();
            _devicePositions.pop_back();
            _deviceBias.pop_back();
            _deviceScale.pop_back();
            flow::ApplyChanges();
        }
    }
    ImGui::Separator();
    if (ImGui::Combo(fmt::format("Solution##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_solutionMode), "Least squares\0Kalman Filter\0\0"))
    {
        switch (_solutionMode)
        {
        case SolutionMode::LSQ:
            LOG_DEBUG("{}: Solution changed to Least squares 3D", nameId());
            break;
        case SolutionMode::KF:
            LOG_DEBUG("{}: Solution changed to Kalman Filter", nameId());
            break;
        }
    }
    flow::ApplyChanges();

    // ###########################################################################################################
    //                                        Least Squares
    // ###########################################################################################################
    if (_solutionMode == SolutionMode::LSQ)
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("x0 - Initial State##{}", size_t(id)).c_str()))
        {
            Eigen::Vector3d llaPos = trafo::ecef2lla_WGS84(_initialState.e_position);
            llaPos.block<2, 1>(0, 0) = rad2deg(llaPos.block<2, 1>(0, 0));

            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDouble3(fmt::format("Position ECEF (m)##{}", "m",
                                                size_t(id))
                                        .c_str(),
                                    _initialState.e_position.data(), "%.4f", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: e_position changed to {}", nameId(), _initialState.e_position);
                flow::ApplyChanges();
            }

            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDouble3(fmt::format("Position LLA (°,°,m)##{}", "(°,°,m)",
                                                size_t(id))
                                        .c_str(),
                                    llaPos.data(), "%.8f", ImGuiInputTextFlags_CharsScientific))
            {
                llaPos.block<2, 1>(0, 0) = deg2rad(llaPos.block<2, 1>(0, 0));
                _initialState.e_position = trafo::lla2ecef_WGS84(llaPos);
                LOG_DEBUG("{}: e_position changed to {}", nameId(), _initialState.e_position);
                flow::ApplyChanges();
            }

            if (_estimateBias)
            {
                ImGui::SetNextItemWidth(configWidth);
                if (ImGui::InputDouble(fmt::format("Bias (m)##{}", "m",
                                                   size_t(id))
                                           .c_str(),
                                       &_initialState.bias, 0, 0, "%.3e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: bias changed to {}", nameId(), _initialState.bias);
                    flow::ApplyChanges();
                }
            }

            ImGui::TreePop();
        }
    }
    // ###########################################################################################################
    //                                        Kalman Filter
    // ###########################################################################################################
    if (_solutionMode == SolutionMode::KF)
    {
        // ###########################################################################################################
        //                                        Measurement Uncertainties 𝐑
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("R - Measurement Noise ##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDoubleWithUnit(fmt::format("({})##{}",
                                                              _measurementNoiseUnit == MeasurementNoiseUnit::meter2
                                                                  ? "Variance σ²"
                                                                  : "Standard deviation σ",
                                                              size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, &_measurementNoise, reinterpret_cast<int*>(&_measurementNoiseUnit), "m^2, m^2, m^2\0"
                                                                                                                                              "m, m, m\0\0",
                                                  0, 0, "%.3e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: measurementNoise changed to {}", nameId(), _measurementNoise);
                LOG_DEBUG("{}: measurementNoiseUnit changed to {}", nameId(), fmt::underlying(_measurementNoiseUnit));
                flow::ApplyChanges();
            }
            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                        Process Noise Covariance 𝐐
        // ###########################################################################################################
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Q - Process Noise ##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDoubleWithUnit(fmt::format("({})##{}",
                                                              _processNoiseUnit == ProcessNoiseUnit::meter2
                                                                  ? "Variance σ²"
                                                                  : "Standard deviation σ",
                                                              size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, &_processNoise, reinterpret_cast<int*>(&_processNoiseUnit), "m^2, m^2, m^2\0"
                                                                                                                                      "m, m, m\0\0",
                                                  0, 0, "%.3e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: processNoise changed to {}", nameId(), _processNoise);
                LOG_DEBUG("{}: processNoiseUnit changed to {}", nameId(), fmt::underlying(_processNoiseUnit));
                flow::ApplyChanges();
            }
            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                        Initial State Estimate 𝐱0
        // ###########################################################################################################
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("x0 - Initial State##{}", size_t(id)).c_str()))
        {
            Eigen::Vector3d llaPos = trafo::ecef2lla_WGS84(_initialState.e_position);
            llaPos.block<2, 1>(0, 0) = rad2deg(llaPos.block<2, 1>(0, 0));

            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDouble3(fmt::format("Position ECEF (m)##{}", "m",
                                                size_t(id))
                                        .c_str(),
                                    _initialState.e_position.data(), "%.4f", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: e_position changed to {}", nameId(), _initialState.e_position);
                flow::ApplyChanges();
            }

            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDouble3(fmt::format("Position LLA ##{}", "m",
                                                size_t(id))
                                        .c_str(),
                                    llaPos.data(), "%.8f°", ImGuiInputTextFlags_CharsScientific))
            {
                llaPos.block<2, 1>(0, 0) = deg2rad(llaPos.block<2, 1>(0, 0));
                _initialState.e_position = trafo::lla2ecef_WGS84(llaPos);
                LOG_DEBUG("{}: e_position changed to {}", nameId(), _initialState.e_position);
                flow::ApplyChanges();
            }

            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDouble3(fmt::format("Velocity (m/s)##{}", "m",
                                                size_t(id))
                                        .c_str(),
                                    _state.e_velocity.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: e_position changed to {}", nameId(), _state.e_velocity);
                flow::ApplyChanges();
            }

            if (_estimateBias)
            {
                ImGui::SetNextItemWidth(configWidth);
                if (ImGui::InputDouble(fmt::format("Bias (m)##{}", "m",
                                                   size_t(id))
                                           .c_str(),
                                       &_state.bias, 0, 0, "%.3e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: bias changed to {}", nameId(), _state.bias);
                    flow::ApplyChanges();
                }
            }

            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                        𝐏 Error covariance matrix
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("P - Error covariance matrix (init)##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Position covariance ({})##{}",
                                                               _initCovariancePositionUnit == InitCovariancePositionUnit::meter2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovariancePosition.data(), reinterpret_cast<int*>(&_initCovariancePositionUnit), "m^2, m^2, m^2\0"
                                                                                                                                                                 "m, m, m\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovariancePosition changed to {}", nameId(), _initCovariancePosition);
                LOG_DEBUG("{}: initCovariancePositionUnit changed to {}", nameId(), fmt::underlying(_initCovariancePositionUnit));
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity covariance ({})##{}",
                                                               _initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceVelocity.data(), reinterpret_cast<int*>(&_initCovarianceVelocityUnit), "m^2/s^2\0"
                                                                                                                                                                 "m/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceVelocity changed to {}", nameId(), _initCovarianceVelocity);
                LOG_DEBUG("{}: initCovarianceVelocityUnit changed to {}", nameId(), fmt::underlying(_initCovarianceVelocityUnit));
                flow::ApplyChanges();
            }

            if (_estimateBias)
            {
                if (gui::widgets::InputDoubleWithUnit(fmt::format("Bias covariance ({})##{}",
                                                                  _initCovarianceBiasUnit == InitCovarianceBiasUnit::meter2
                                                                      ? "Variance σ²"
                                                                      : "Standard deviation σ",
                                                                  size_t(id))
                                                          .c_str(),
                                                      configWidth, unitWidth, &_initCovarianceBias, reinterpret_cast<int*>(&_initCovarianceBiasUnit), "m^2\0"
                                                                                                                                                      "m\0\0",
                                                      0, 0, "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: initCovarianceBias changed to {}", nameId(), _initCovarianceBias);
                    LOG_DEBUG("{}: initCovarianceBiasUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasUnit));
                    flow::ApplyChanges();
                }
            }

            ImGui::TreePop();
        }
    }
    ImGui::Separator();
    // ###########################################################################################################
    //                                        Estimate Bias
    // ###########################################################################################################
    if (ImGui::Checkbox(fmt::format("Estimate Bias##{}", size_t(id)).c_str(), &_estimateBias))
    {
        if (_estimateBias)
        {
            LOG_DEBUG("{}: Estimate Bias changed to Yes", nameId());
            _numStates = 7;
        }
        else
        {
            LOG_DEBUG("{}: Estimate Bias changed to No", nameId());
            _numStates = 6;
        }
    }
    flow::ApplyChanges();
    // ###########################################################################################################
    //                                        Weighted Solution
    // ###########################################################################################################
    if (ImGui::Checkbox(fmt::format("Weighted Solution##{}", size_t(id)).c_str(), &_weightedSolution))
    {
        if (_weightedSolution)
        {
            LOG_DEBUG("{}: Weighted Solution changed to Yes", nameId());
        }
        else
        {
            LOG_DEBUG("{}: Weighted Solution changed to No", nameId());
        }
    }
    flow::ApplyChanges();

    // ###########################################################################################################
    //                                        Use Initial Values
    // ###########################################################################################################
    if (_solutionMode == SolutionMode::LSQ)
    {
        if (ImGui::Checkbox(fmt::format("Use Initial Values##{}", size_t(id)).c_str(), &_useInitialValues))
        {
            if (_useInitialValues)
            {
                LOG_DEBUG("{}: Use Initial Values changed to Yes", nameId());
            }
            else
            {
                LOG_DEBUG("{}: Use Initial Values changed to No", nameId());
            }
        }
    }
    flow::ApplyChanges();
}

[[nodiscard]] json NAV::WiFiPositioning::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nWifiInputPins"] = _nWifiInputPins;
    j["numStates"] = _numStates;
    j["numMeasurements"] = _numMeasurements;
    j["frame"] = _frame;
    j["estimateBias"] = _estimateBias;
    j["weightedSolution"] = _weightedSolution;
    j["useInitialValues"] = _useInitialValues;
    j["deviceMacAddresses"] = _deviceMacAddresses;
    j["devicePositions"] = _devicePositions;
    j["deviceBias"] = _deviceBias;
    j["deviceScale"] = _deviceScale;
    j["numOfDevices"] = _numOfDevices;
    j["solutionMode"] = _solutionMode;
    j["e_position"] = _state.e_position;
    j["e_velocity"] = _state.e_velocity;
    j["bias"] = _state.bias;
    j["intialStatePosition"] = _initialState.e_position;
    j["initialStateVelocity"] = _initialState.e_velocity;
    j["initialStateBias"] = _initialState.bias;
    j["initCovariancePosition"] = _initCovariancePosition;
    j["initCovariancePositionUnit"] = _initCovariancePositionUnit;
    j["initCovarianceVelocity"] = _initCovarianceVelocity;
    j["initCovarianceVelocityUnit"] = _initCovarianceVelocityUnit;
    j["initCovarianceBias"] = _initCovarianceBias;
    j["initCovarianceBiasUnit"] = _initCovarianceBiasUnit;
    j["measurementNoise"] = _measurementNoise;
    j["measurementNoiseUnit"] = _measurementNoiseUnit;
    j["processNoise"] = _processNoise;
    j["processNoiseUnit"] = _processNoiseUnit;

    return j;
}

void NAV::WiFiPositioning::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nWifiInputPins"))
    {
        j.at("nWifiInputPins").get_to(_nWifiInputPins);
        updateNumberOfInputPins();
    }
    if (j.contains("numStates"))
    {
        j.at("numStates").get_to(_numStates);
    }
    if (j.contains("numMeasurements"))
    {
        j.at("numMeasurements").get_to(_numMeasurements);
    }
    if (j.contains("frame"))
    {
        j.at("frame").get_to(_frame);
    }
    if (j.contains("estimateBias"))
    {
        j.at("estimateBias").get_to(_estimateBias);
    }
    if (j.contains("weightedSolution"))
    {
        j.at("weightedSolution").get_to(_weightedSolution);
    }
    if (j.contains("useInitialValues"))
    {
        j.at("useInitialValues").get_to(_useInitialValues);
    }
    if (j.contains("deviceMacAddresses"))
    {
        j.at("deviceMacAddresses").get_to(_deviceMacAddresses);
    }
    if (j.contains("devicePositions"))
    {
        j.at("devicePositions").get_to(_devicePositions);
    }
    if (j.contains("deviceBias"))
    {
        j.at("deviceBias").get_to(_deviceBias);
    }
    if (j.contains("deviceScale"))
    {
        j.at("deviceScale").get_to(_deviceScale);
    }
    if (j.contains("numOfDevices"))
    {
        j.at("numOfDevices").get_to(_numOfDevices);
    }
    if (j.contains("solutionMode"))
    {
        j.at("solutionMode").get_to(_solutionMode);
    }
    if (j.contains("e_position"))
    {
        j.at("e_position").get_to(_state.e_position);
    }
    if (j.contains("e_velocity"))
    {
        j.at("e_velocity").get_to(_state.e_velocity);
    }
    if (j.contains("bias"))
    {
        j.at("bias").get_to(_state.bias);
    }
    if (j.contains("intialStatePosition"))
    {
        j.at("intialStatePosition").get_to(_initialState.e_position);
    }
    if (j.contains("initialStateVelocity"))
    {
        j.at("initialStateVelocity").get_to(_initialState.e_velocity);
    }
    if (j.contains("initialStateBias"))
    {
        j.at("initialStateBias").get_to(_initialState.bias);
    }
    if (j.contains("initCovariancePosition"))
    {
        j.at("initCovariancePosition").get_to(_initCovariancePosition);
    }
    if (j.contains("initCovariancePositionUnit"))
    {
        j.at("initCovariancePositionUnit").get_to(_initCovariancePositionUnit);
    }
    if (j.contains("initCovarianceVelocity"))
    {
        j.at("initCovarianceVelocity").get_to(_initCovarianceVelocity);
    }
    if (j.contains("initCovarianceVelocityUnit"))
    {
        j.at("initCovarianceVelocityUnit").get_to(_initCovarianceVelocityUnit);
    }
    if (j.contains("initCovarianceBias"))
    {
        j.at("initCovarianceBias").get_to(_initCovarianceBias);
    }
    if (j.contains("initCovarianceBiasUnit"))
    {
        j.at("initCovarianceBiasUnit").get_to(_initCovarianceBiasUnit);
    }
    if (j.contains("measurementNoise"))
    {
        j.at("measurementNoise").get_to(_measurementNoise);
    }
    if (j.contains("measurementNoiseUnit"))
    {
        j.at("measurementNoiseUnit").get_to(_measurementNoiseUnit);
    }
    if (j.contains("processNoise"))
    {
        j.at("processNoise").get_to(_processNoise);
    }
    if (j.contains("processNoiseUnit"))
    {
        j.at("processNoiseUnit").get_to(_processNoiseUnit);
    }
}

bool NAV::WiFiPositioning::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _kalmanFilter = KalmanFilter{ _numStates, _numMeasurements };

    // Initial state
    _state.e_position = _initialState.e_position;
    _state.e_velocity = _initialState.e_velocity;
    if (_estimateBias)
    {
        _state.bias = _initialState.bias;
    }

    // Initial Covariance of the velocity in [m²/s²]
    Eigen::Vector3d variance_vel = Eigen::Vector3d::Zero();
    if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2)
    {
        variance_vel = _initCovarianceVelocity;
    }
    else if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m_s)
    {
        variance_vel = _initCovarianceVelocity.array().pow(2);
    }

    // Initial Covariance of the position in [m²]
    Eigen::Vector3d variance_pos = Eigen::Vector3d::Zero();
    if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter2)
    {
        variance_pos = _initCovariancePosition;
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter)
    {
        variance_pos = _initCovariancePosition.array().pow(2);
    }

    // Initial Covariance of the bias in [m²]
    double variance_bias = 0.0;
    if (_initCovarianceBiasUnit == InitCovarianceBiasUnit::meter2)
    {
        variance_bias = _initCovarianceBias;
    }
    else if (_initCovarianceBiasUnit == InitCovarianceBiasUnit::meter)
    {
        variance_bias = std::pow(_initCovarianceBias, 2);
    }
    if (_estimateBias)
    {
        _kalmanFilter.P.diagonal() << variance_pos, variance_vel, variance_bias;
    }
    else
    {
        _kalmanFilter.P.diagonal() << variance_pos, variance_vel;
    }
    if (_estimateBias)
    {
        _kalmanFilter.x << _state.e_position, _state.e_velocity, _state.bias;
    }
    else
    {
        _kalmanFilter.x << _state.e_position, _state.e_velocity;
    }
    if (_measurementNoiseUnit == MeasurementNoiseUnit::meter2)
    {
        _kalmanFilter.R << _measurementNoise;
    }
    else if (_measurementNoiseUnit == MeasurementNoiseUnit::meter)
    {
        _kalmanFilter.R << std::pow(_measurementNoise, 2);
    }

    LOG_DEBUG("WiFiPositioning initialized");

    return true;
}

void NAV::WiFiPositioning::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::WiFiPositioning::updateNumberOfInputPins()
{
    while (inputPins.size() < _nWifiInputPins)
    {
        nm::CreateInputPin(this, NAV::WiFiObs::type().c_str(), Pin::Type::Flow, { NAV::WiFiObs::type() }, &WiFiPositioning::recvWiFiObs);
    }
    while (inputPins.size() > _nWifiInputPins)
    {
        nm::DeleteInputPin(inputPins.back());
    }
}

void NAV::WiFiPositioning::recvWiFiObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const WiFiObs>(queue.extract_front());
    auto it = std::find(_deviceMacAddresses.begin(), _deviceMacAddresses.end(), obs->macAddress);
    if (it != _deviceMacAddresses.end()) // Check if the MAC address is in the list
    {
        // Get the index of the found element
        size_t index = static_cast<size_t>(std::distance(_deviceMacAddresses.begin(), it));

        // Check if a device with the same position already exists and update the distance
        bool deviceExists = false;
        for (auto& device : _devices)
        {
            if (_frame == Frame::ECEF)
            {
                if (device.position == _devicePositions.at(index))
                {
                    deviceExists = true;
                    device.distance = obs->distance * _deviceScale.at(index) + _deviceBias.at(index);
                    device.distanceStd = obs->distanceStd * _deviceScale.at(index);
                    device.time = obs->insTime;
                    break;
                }
            }
            else if (_frame == Frame::LLA)
            {
                Eigen::Vector3d ecefPos = _devicePositions.at(index);
                ecefPos.block<2, 1>(0, 0) = deg2rad(ecefPos.block<2, 1>(0, 0));
                ecefPos = trafo::lla2ecef_WGS84(ecefPos);
                if (device.position == ecefPos)
                {
                    deviceExists = true;
                    device.distance = obs->distance * _deviceScale.at(index) + _deviceBias.at(index);
                    device.distanceStd = obs->distanceStd * _deviceScale.at(index);
                    device.time = obs->insTime;
                    break;
                }
            }
        }

        // If the device does not exist, add it to the list
        if (!deviceExists)
        {
            if (_frame == Frame::LLA)
            {
                Eigen::Vector3d llaPos = _devicePositions.at(index);
                llaPos.block<2, 1>(0, 0) = deg2rad(llaPos.block<2, 1>(0, 0));
                _devices.push_back({ trafo::lla2ecef_WGS84(llaPos), obs->insTime, obs->distance * _deviceScale.at(index) + _deviceBias.at(index), obs->distanceStd * _deviceScale.at(index) });
            }
            else if (_frame == Frame::ECEF)
            {
                _devices.push_back({ _devicePositions.at(index), obs->insTime, obs->distance * _deviceScale.at(index) + _deviceBias.at(index), obs->distanceStd * _deviceScale.at(index) });
            }
        }

        // Calculate the solution
        auto wifiPositioningSolution = std::make_shared<NAV::WiFiPositioningSolution>();
        wifiPositioningSolution->insTime = obs->insTime;
        // Least Squares
        if (_solutionMode == SolutionMode::LSQ)
        {
            if (_devices.size() == _numOfDevices)
            {
                LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsqSolution = WiFiPositioning::lsqSolution();
                wifiPositioningSolution->setPositionAndStdDev_e(lsqSolution.solution.block<3, 1>(0, 0), lsqSolution.variance.block<3, 3>(0, 0).cwiseSqrt());
                wifiPositioningSolution->setPosCovarianceMatrix_e(lsqSolution.variance.block<3, 3>(0, 0));
                if (_estimateBias)
                {
                    wifiPositioningSolution->bias = _state.bias;
                    wifiPositioningSolution->biasStdev = lsqSolution.variance(3, 3);
                }
                invokeCallbacks(OUTPUT_PORT_INDEX_WIFISOL, wifiPositioningSolution);
            }
        }
        // Kalman Filter
        else if (_solutionMode == SolutionMode::KF)
        {
            WiFiPositioning::kfSolution();
            wifiPositioningSolution->setPositionAndStdDev_e(_kalmanFilter.x.block<3, 1>(0, 0), _kalmanFilter.P.block<3, 3>(0, 0).cwiseSqrt());
            if (_estimateBias)
            {
                wifiPositioningSolution->bias = _kalmanFilter.x(6);
                wifiPositioningSolution->biasStdev = _kalmanFilter.P(6, 6);
            }
            wifiPositioningSolution->setVelocityAndStdDev_e(_kalmanFilter.x.block<3, 1>(3, 0), _kalmanFilter.P.block<3, 3>(3, 3).cwiseSqrt());
            wifiPositioningSolution->setPosVelCovarianceMatrix_e(_kalmanFilter.P);
            invokeCallbacks(OUTPUT_PORT_INDEX_WIFISOL, wifiPositioningSolution);
        }

        LOG_DEBUG("{}: Received distance to device {} at position {} with distance {}", nameId(), obs->macAddress, _devicePositions.at(index).transpose(), obs->distance);
    }
}

NAV::LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> NAV::WiFiPositioning::lsqSolution()
{
    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsq;
    int n = (_estimateBias) ? 4 : 3; // Number of unknowns

    // Check if the number of devices is sufficient to compute the position
    if ((_estimateBias && _devices.size() < 5) || (!_estimateBias && _devices.size() < 4))
    {
        LOG_DEBUG("{}: Received less than {} observations. Can't compute position", nameId(), (_estimateBias ? 5 : 4));
        return lsq;
    }
    else
    {
        LOG_DEBUG("{}: Received {} observations", nameId(), _devices.size());
    }

    Eigen::MatrixXd e_H = Eigen::MatrixXd::Zero(static_cast<int>(_devices.size()), n);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(static_cast<int>(_devices.size()), static_cast<int>(_devices.size()));
    Eigen::VectorXd ddist = Eigen::VectorXd::Zero(static_cast<int>(_devices.size()));
    size_t numMeasurements = _devices.size();

    // Check if the initial position is NaN
    if (std::isnan(_state.e_position(0)) || std::isnan(_state.e_position(1)) || std::isnan(_state.e_position(2)) || _useInitialValues)
    {
        _state.e_position << _initialState.e_position;
        if (_estimateBias)
        {
            _state.bias = _initialState.bias;
        }
    }

    // Iteratively solve the linear least squares problem
    for (size_t o = 0; o < 15; o++)
    {
        LOG_DATA("{}: Iteration {}", nameId(), o);
        for (size_t i = 0; i < numMeasurements; i++)
        {
            // Calculate the distance between the device and the estimated position
            double distEst = (_devices.at(i).position - _state.e_position).norm();
            if (_estimateBias)
            {
                distEst += _state.bias;
            }

            // Calculate the residual vector
            ddist(static_cast<int>(i)) = _devices.at(i).distance - distEst;

            Eigen::Vector3d e_lineOfSightUnitVector = Eigen::Vector3d::Zero();
            if ((_state.e_position - _devices.at(i).position).norm() != 0) // Check if it is the same position
            {
                e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(_state.e_position, _devices.at(i).position);
            }

            // Calculate the design matrix
            e_H.block<1, 3>(static_cast<int>(i), 0) = -e_lineOfSightUnitVector;
            if (_estimateBias)
            {
                e_H(static_cast<int>(i), 3) = 1;
            }

            // Calculate the weight matrix
            if (_weightedSolution)
            {
                W(static_cast<int>(i), static_cast<int>(i)) = 1 / std::pow(_devices.at(i).distanceStd, 2);
            }
        }
        // Solve the linear least squares problem
        lsq = solveWeightedLinearLeastSquaresUncertainties(e_H, W, ddist);

        if (_estimateBias)
        {
            LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2), lsq.solution(3));
            LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }
        else
        {
            LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2));
            LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());
        }

        // Update the estimated position
        _state.e_position += lsq.solution.block<3, 1>(0, 0);

        // Update the estimated bias
        if (_estimateBias)
        {
            _state.bias += lsq.solution(3);
        }

        bool solInaccurate = pow(lsq.solution.norm(), 2) > 1e-3;
        if (!solInaccurate) // Solution is accurate enough
        {
            lsq.solution.block<3, 1>(0, 0) = _state.e_position;
            break;
        }
        if (o == 14)
        {
            LOG_DEBUG("{}: Solution did not converge", nameId());
            lsq.solution.setConstant(std::numeric_limits<double>::quiet_NaN());
            lsq.variance.setConstant(std::numeric_limits<double>::quiet_NaN());
            if (_estimateBias)
            {
                _state.bias = std::numeric_limits<double>::quiet_NaN();
            }
            _state.e_position = _initialState.e_position;
            if (_estimateBias)
            {
                _state.bias = _initialState.bias;
            }
        }
    }

    _devices.clear();
    LOG_DEBUG("{}: Position: {}", nameId(), _state.e_position.transpose());

    return lsq;
}

void NAV::WiFiPositioning::kfSolution()
{
    double tau_i = !_lastPredictTime.empty()
                       ? static_cast<double>((_devices.at(0).time - _lastPredictTime).count())
                       : 0.0;

    // ###########################################################################################################
    // Prediction
    // ###########################################################################################################

    _lastPredictTime = _devices.at(0).time;
    if (tau_i > 0)
    {
        // Transition matrix
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(_numStates, _numStates);
        F(0, 3) = 1;
        F(1, 4) = 1;
        F(2, 5) = 1;
        _kalmanFilter.Phi = transitionMatrix_Phi_Taylor(F, tau_i, 1);

        // Process noise covariance matrix
        _kalmanFilter.Q.block(0, 0, 3, 3) = std::pow(tau_i, 3) / 3.0 * Eigen::Matrix3d::Identity();
        _kalmanFilter.Q.block(3, 0, 3, 3) = std::pow(tau_i, 2) / 2.0 * Eigen::Matrix3d::Identity();
        _kalmanFilter.Q.block(0, 3, 3, 3) = std::pow(tau_i, 2) / 2.0 * Eigen::Matrix3d::Identity();
        _kalmanFilter.Q.block(3, 3, 3, 3) = tau_i * Eigen::Matrix3d::Identity();
        if (_estimateBias)
        {
            _kalmanFilter.Q(6, 6) = tau_i;
        }
        if (_processNoiseUnit == ProcessNoiseUnit::meter2)
        {
            _kalmanFilter.Q *= _processNoise;
        }
        else if (_processNoiseUnit == ProcessNoiseUnit::meter)
        {
            _kalmanFilter.Q *= std::pow(_processNoise, 2);
        }
        // Predict
        _kalmanFilter.predict();
    }

    // ###########################################################################################################
    // Update
    // ###########################################################################################################

    // Measurement
    double estimatedDistance = (_devices.at(0).position - _kalmanFilter.x.block<3, 1>(0, 0)).norm();
    if (_estimateBias)
    {
        estimatedDistance += _kalmanFilter.x(6);
    }
    _kalmanFilter.z << _devices.at(0).distance - estimatedDistance;
    if (_weightedSolution)
    {
        _kalmanFilter.R << std::pow(_devices.at(0).distanceStd, 2);
    }

    // Design matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, _numStates);
    H.block<1, 3>(0, 0) = -e_calcLineOfSightUnitVector(_kalmanFilter.x.block<3, 1>(0, 0), _devices.at(0).position);
    if (_estimateBias)
    {
        H(0, 6) = 1;
    }
    _kalmanFilter.H << H;

    // Update
    _kalmanFilter.correctWithMeasurementInnovation();

    _devices.clear();
    LOG_DATA("{}: Position: {}", nameId(), _kalmanFilter.x.block<3, 1>(0, 0).transpose());
    LOG_DATA("{}: Velocity: {}", nameId(), _kalmanFilter.x.block<3, 1>(3, 0).transpose());
}