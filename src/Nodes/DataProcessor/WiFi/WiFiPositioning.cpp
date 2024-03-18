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

#include "Navigation/Math/LeastSquares.hpp"

NAV::WiFiPositioning::WiFiPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 575, 300 };

    updateNumberOfInputPins();

    // updateOutputPin();
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

    if (_numOfDevices == 0)
    {
        if (ImGui::Combo(fmt::format("Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_frame), "ENU\0NED\0ECEF\0LLA\0\0"))
        {
            switch (_frame)
            {
            case Frame::ENU:
                LOG_DEBUG("{}: Frame changed to ENU", nameId());
                break;
            case Frame::NED:
                LOG_DEBUG("{}: Frame changed to NED", nameId());
                break;
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

    if (ImGui::BeginTable("RouterInput", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        // Column headers
        ImGui::TableSetupColumn("MAC address", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        if (_frame == Frame::ENU)
        {
            ImGui::TableSetupColumn("East", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("North", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Up", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        }
        else if (_frame == Frame::NED)
        {
            ImGui::TableSetupColumn("North", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("East", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Down", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        }
        else if (_frame == Frame::ECEF)
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
            if (ImGui::InputText(fmt::format("##Mac{}", size_t(rowIndex)).c_str(), &_deviceMacAddresses.at(rowIndex), ImGuiInputTextFlags_None))
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
            if (_frame == Frame::ENU)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputEast{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[0], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputNorth{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[1], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputUp{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[2], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
            }
            else if (_frame == Frame::NED)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputNorth{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[0], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputEast{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[1], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputDown{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[2], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
            }
            else if (_frame == Frame::ECEF)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputX{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[0], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputY{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[1], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputZ{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[2], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
            }
            else if (_frame == Frame::LLA)
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDoubleL(fmt::format("##InputLat{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[0], -180, 180, 0.0, 0.0, "%.8f°"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDoubleL(fmt::format("##InputLon{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[1], -180, 180, 0.0, 0.0, "%.8f°"))
                {
                    flow::ApplyChanges();
                }
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("##InputHeight{}", size_t(rowIndex)).c_str(), &_devicePositions.at(rowIndex)[2], 0.0, 0.0, "%.4fm"))
                {
                    flow::ApplyChanges();
                }
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##InputBias{}", size_t(rowIndex)).c_str(), &_deviceBias.at(rowIndex), 0.0, 0.0, "%.4fm"))
            {
                flow::ApplyChanges();
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##InputScale{}", size_t(rowIndex)).c_str(), &_deviceScale.at(rowIndex), 0.0, 0.0, "%.4f"))
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
    // updateOutputPin();

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
            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDouble3(fmt::format("Position (m)##{}", "m",
                                                size_t(id))
                                        .c_str(),
                                    _state.e_position.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: e_position changed to {}", nameId(), _state.e_position);
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

            ImGui::TreePop();
        }
    }
}

[[nodiscard]] json NAV::WiFiPositioning::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nWifiInputPins"] = _nWifiInputPins;
    j["frame"] = _frame;
    j["deviceMacAddresses"] = _deviceMacAddresses;
    j["devicePositions"] = _devicePositions;
    j["deviceBias"] = _deviceBias;
    j["deviceScale"] = _deviceScale;
    j["numOfDevices"] = _numOfDevices;
    j["solutionMode"] = _solutionMode;
    j["e_position"] = _state.e_position;
    j["e_velocity"] = _state.e_velocity;
    j["initCovariancePosition"] = _initCovariancePosition;
    j["initCovariancePositionUnit"] = _initCovariancePositionUnit;
    j["initCovarianceVelocity"] = _initCovarianceVelocity;
    j["initCovarianceVelocityUnit"] = _initCovarianceVelocityUnit;
    j["measurementNoise"] = _measurementNoise;
    j["measurementNoiseUnit"] = _measurementNoiseUnit;
    j["processNoise"] = _processNoise;
    j["processNoiseUnit"] = _processNoiseUnit;

    return j;
}

void NAV::WiFiPositioning::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nNavInfoPins"))
    {
        j.at("nNavInfoPins").get_to(_nWifiInputPins);
        updateNumberOfInputPins();
    }
    if (j.contains("frame"))
    {
        j.at("frame").get_to(_frame);
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

    _kalmanFilter.P.diagonal() << variance_pos, variance_vel;
    _kalmanFilter.x << _state.e_position, _state.e_velocity;
    std::cout << _kalmanFilter.x << std::endl;
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
    auto wifiObs = std::static_pointer_cast<const WiFiObs>(queue.extract_front());
    for (auto const& obs : wifiObs->data)
    {
        auto it = std::find(_deviceMacAddresses.begin(), _deviceMacAddresses.end(), obs.macAddress);
        if (it != _deviceMacAddresses.end()) // Device exists
        {
            // Get the index of the found element
            size_t index = static_cast<size_t>(std::distance(_deviceMacAddresses.begin(), it));

            // Check if a device with the same position already exists and update the distance
            bool deviceExists = false;
            for (auto& device : _devices)
            {
                if (device.position == _devicePositions.at(index))
                {
                    deviceExists = true;
                    device.distance = obs.distance * _deviceScale.at(index) + _deviceBias.at(index);
                    device.time = obs.time;
                    break;
                }
            }

            // If the device does not exist, add it to the list
            if (!deviceExists)
            {
                if (_frame == Frame::LLA)
                {
                    _devices.push_back({ trafo::lla2ecef_WGS84(_devicePositions.at(index)), obs.time, obs.distance * _deviceScale.at(index) + _deviceBias.at(index) });
                }
                else if (_frame == Frame::ECEF)
                {
                    _devices.push_back({ _devicePositions.at(index), obs.time, obs.distance * _deviceScale.at(index) + _deviceBias.at(index) });
                }
                else if (_frame == Frame::ENU)
                {
                    _devices.push_back({ _devicePositions.at(index), obs.time, obs.distance * _deviceScale.at(index) + _deviceBias.at(index) });
                }
                else if (_frame == Frame::NED)
                {
                    _devices.push_back({ _devicePositions.at(index), obs.time, obs.distance * _deviceScale.at(index) + _deviceBias.at(index) });
                }
            }

            auto wifiPositioningSolution = std::make_shared<NAV::WiFiPositioningSolution>();
            wifiPositioningSolution->insTime = obs.time;
            if (_solutionMode == SolutionMode::LSQ)
            {
                if (_devices.size() == _numOfDevices)
                {
                    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsqSolution = WiFiPositioning::lsqSolution();
                    wifiPositioningSolution->setPositionAndStdDev_e(_state.e_position, lsqSolution.variance.cwiseSqrt());
                    std::cout << lsqSolution.solution << std::endl;
                    wifiPositioningSolution->setCovarianceMatrix(lsqSolution.variance);
                    invokeCallbacks(OUTPUT_PORT_INDEX_WIFISOL, wifiPositioningSolution);
                }
            }
            else if (_solutionMode == SolutionMode::KF)
            {
                WiFiPositioning::kfSolution();
                wifiPositioningSolution->setPositionAndStdDev_e(_kalmanFilter.x.block<3, 1>(0, 0), _kalmanFilter.P.block<3, 3>(0, 0).cwiseSqrt());
                wifiPositioningSolution->setVelocityAndStdDev_e(_kalmanFilter.x.block<3, 1>(3, 0), _kalmanFilter.P.block<3, 3>(3, 3).cwiseSqrt());
                wifiPositioningSolution->setCovarianceMatrix(_kalmanFilter.P);
                invokeCallbacks(OUTPUT_PORT_INDEX_WIFISOL, wifiPositioningSolution);
            }

            // print // TODO delete
            LOG_DEBUG("{}: Received distance to device {} at position {} with distance {}", nameId(), obs.macAddress, _devicePositions.at(index).transpose(), obs.distance);
        }
    }
}

NAV::LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> NAV::WiFiPositioning::lsqSolution()
{
    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsq;

    if (_devices.size() < 4)
    {
        LOG_DEBUG("{}: Received less than 4 observations. Can't compute position", nameId());
        return lsq;
    }
    else
    {
        LOG_DEBUG("{}: Received {} observations", nameId(), _devices.size());
    }
    _state.e_position = { 10.0, 10.0, 10.0 }; // TODO Initialwerte

    // calculate the centroid of device positions
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& device : _devices)
    {
        centroid += device.position;
    }
    centroid /= _devices.size();
    _state.e_position = centroid;

    Eigen::MatrixXd e_H = Eigen::MatrixXd::Zero(static_cast<int>(_devices.size()), static_cast<int>(3));
    Eigen::VectorXd ddist = Eigen::VectorXd::Zero(static_cast<int>(_devices.size()));
    size_t numMeasurements = _devices.size();

    for (size_t o = 0; o < 10; o++)
    {
        LOG_DATA("{}: Iteration {}", nameId(), o);
        for (size_t i = 0; i < numMeasurements; i++)
        {
            // calculate the distance between the device and the estimated position
            double distEst = (_devices.at(i).position - _state.e_position).norm();
            // calculate the residual vector
            ddist(static_cast<int>(i)) = _devices.at(i).distance - distEst;

            Eigen::Vector3d e_lineOfSightUnitVector = Eigen::Vector3d::Zero();
            if ((_state.e_position - _devices.at(i).position).norm() != 0) // Check if it is the same position
            {
                e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(_state.e_position, _devices.at(i).position);
            }
            // calculate the design matrix
            e_H.block<1, 3>(static_cast<int>(i), 0) = -e_lineOfSightUnitVector;
        }
        // solve the linear least squares problem
        lsq = solveLinearLeastSquaresUncertainties(e_H, ddist);
        LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2));
        LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());

        // update the estimated position
        _state.e_position += lsq.solution;

        bool solInaccurate = lsq.solution.norm() > 1e-3;
        if (!solInaccurate) // Solution is accurate enough
        {
            break;
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
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
        F(0, 3) = 1;
        F(1, 4) = 1;
        F(2, 5) = 1;
        _kalmanFilter.Phi = transitionMatrix_Phi_Taylor(F, tau_i, 1);
        std::cout << F << std::endl;
        std::cout << _kalmanFilter.Phi << std::endl;
        // Process noise covariance matrix
        Eigen::Matrix3d Q1 = Eigen::Matrix3d::Zero();
        Q1.diagonal() = Eigen::Vector3d(std::pow(tau_i, 3) / 3.0, std::pow(tau_i, 3) / 3.0, std::pow(tau_i, 3) / 3.0);
        Eigen::Matrix3d Q2 = Eigen::Matrix3d::Zero();
        Q2.diagonal() = Eigen::Vector3d(std::pow(tau_i, 2) / 2.0, std::pow(tau_i, 2) / 2.0, std::pow(tau_i, 2) / 2.0);
        Eigen::Matrix3d Q4 = Eigen::Matrix3d::Zero();
        Q4.diagonal() = Eigen::Vector3d(tau_i, tau_i, tau_i);
        _kalmanFilter.Q << Q1, Q2, Q2, Q4;
        if (_processNoiseUnit == ProcessNoiseUnit::meter2)
        {
            _kalmanFilter.Q *= _processNoise;
        }
        else if (_processNoiseUnit == ProcessNoiseUnit::meter)
        {
            _kalmanFilter.Q *= std::pow(_processNoise, 2);
        }
        // Predict
        std::cout << _kalmanFilter.Q << std::endl;
        std::cout << _kalmanFilter.x << std::endl;
        std::cout << _kalmanFilter.P << std::endl;
        _kalmanFilter.predict();
    }

    // ###########################################################################################################
    // Update
    // ###########################################################################################################
    // Measurement
    double estimatedDistance = (_devices.at(0).position - _kalmanFilter.x.block<3, 1>(0, 0)).norm();
    _kalmanFilter.z << _devices.at(0).distance - estimatedDistance;
    // Design matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
    H.block<1, 3>(0, 0) = -e_calcLineOfSightUnitVector(_kalmanFilter.x.block<3, 1>(0, 0), _devices.at(0).position);
    _kalmanFilter.H << H;
    // Correct
    std::cout << _kalmanFilter.Q << std::endl;
    std::cout << _kalmanFilter.x << std::endl;
    std::cout << _kalmanFilter.P << std::endl;
    std::cout << _kalmanFilter.z << std::endl;
    std::cout << _kalmanFilter.H << std::endl;
    _kalmanFilter.correctWithMeasurementInnovation();

    _devices.clear();
    LOG_DATA("{}: Position: {}", nameId(), _kalmanFilter.x.block<3, 1>(0, 0).transpose());
    LOG_DATA("{}: Velocity: {}", nameId(), _kalmanFilter.x.block<3, 1>(3, 0).transpose());
}