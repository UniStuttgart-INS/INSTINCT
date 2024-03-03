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

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "Navigation/Math/LeastSquares.hpp"

NAV::WiFiPositioning::WiFiPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 575, 300 };
    _outputInterval = 3000;

    updateNumberOfInputPins();

    nm::CreateOutputPin(this, NAV::Pos::type().c_str(), Pin::Type::Flow, { NAV::Pos::type() });
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
        if (ImGui::Combo(fmt::format("Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_frame), "ECEF\0LLA\0NED\0\0"))
        {
            switch (_frame)
            {
            case Frame::ECEF:
                LOG_DEBUG("{}: Frame changed to ECEF", nameId());
                break;
            case Frame::LLA:
                LOG_DEBUG("{}: Frame changed to LLA", nameId());
                break;
            case Frame::NED:
                LOG_DEBUG("{}: Frame changed to NED", nameId());
                break;
            }
        }

        flow::ApplyChanges();
    }

    if (ImGui::BeginTable("RouterInput", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
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
        else if (_frame == Frame::NED)
        {
            ImGui::TableSetupColumn("North", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("East", ImGuiTableColumnFlags_WidthFixed, columnWidth);
            ImGui::TableSetupColumn("Down", ImGuiTableColumnFlags_WidthFixed, columnWidth);
        }

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

            if (_frame == Frame::ECEF)
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
        }

        ImGui::EndTable();
    }
    if (ImGui::Button(fmt::format("Add Device##{}", size_t(id)).c_str(), ImVec2(columnWidth * 2.1f, 0)))
    {
        _numOfDevices++;
        _deviceMacAddresses.push_back("00:00:00:00:00:00");
        _devicePositions.push_back(Eigen::Vector3d::Zero());
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
            flow::ApplyChanges();
        }
    }
    ImGui::Separator();
    if (ImGui::Combo(fmt::format("Solution##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_solutionMode), "Least squares 2D\0Least squares 3D\0Kalman Filter\0\0"))
    {
        switch (_solutionMode)
        {
        case SolutionMode::LSQ2D:
            LOG_DEBUG("{}: Solution changed to Least squares 2D", nameId());
            break;
        case SolutionMode::LSQ3D:
            LOG_DEBUG("{}: Solution changed to Least squares 3D", nameId());
            break;
        case SolutionMode::KF:
            LOG_DEBUG("{}: Solution changed to Kalman Filter", nameId());
            break;
        }
    }
    flow::ApplyChanges();
    if (_solutionMode == SolutionMode::LSQ2D || _solutionMode == SolutionMode::LSQ3D)
    {
        ImGui::SameLine();
        gui::widgets::HelpMarker("The third coordinate of the devices is not utilized in a two-dimensional solution.");
        flow::ApplyChanges();
        if (ImGui::InputInt("Output interval [ms]", &_outputInterval))
        {
            LOG_DEBUG("{}: output interval changed to {}", nameId(), _outputInterval);
        }
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::WiFiPositioning::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nWifiInputPins"] = _nWifiInputPins;
    j["frame"] = static_cast<int>(_frame);
    j["deviceMacAddresses"] = _deviceMacAddresses;
    j["devicePositions"] = _devicePositions;
    j["numOfDevices"] = _numOfDevices;
    j["solutionMode"] = static_cast<int>(_solutionMode);
    j["outputInterval"] = _outputInterval;

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
    if (j.contains("numOfDevices"))
    {
        j.at("numOfDevices").get_to(_numOfDevices);
    }
    if (j.contains("solutionMode"))
    {
        j.at("solutionMode").get_to(_solutionMode);
    }
    if (j.contains("outputInterval"))
    {
        j.at("outputInterval").get_to(_outputInterval);
    }
}

bool NAV::WiFiPositioning::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _e_position = Eigen::Vector3d::Zero();

    LOG_DEBUG("WiFiPositioning initialized");

    _devices.clear();
    _timer.start(_outputInterval, lsqSolution3d, this);

    return true;
}

void NAV::WiFiPositioning::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (_timer.is_running())
    {
        _timer.stop();
    }
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
        if (it != _deviceMacAddresses.end()) // Device already exists
        {
            // Get the index of the found element
            size_t index = static_cast<size_t>(std::distance(_deviceMacAddresses.begin(), it));
            if (_frame == Frame::LLA)
            {
                _devices.push_back({ trafo::lla2ecef_WGS84(_devicePositions.at(index)), obs.time, obs.distance });
            }
            else if (_frame == Frame::ECEF)
            {
                _devices.push_back({ _devicePositions.at(index), obs.time, obs.distance });
            }
            else if (_frame == Frame::NED)
            {
                _devices.push_back({ _devicePositions.at(index), obs.time, obs.distance });
            }
            // aufruf kf update
        }
    }
}

void NAV::WiFiPositioning::lsqSolution2d(void* userData)
{
    auto* node = static_cast<WiFiPositioning*>(userData);
    if (node->_devices.size() < 3)
    {
        LOG_DEBUG("{}: Received less than 3 observations. Can't compute position", node->nameId());
        return;
    }
    else
    {
        LOG_DEBUG("{}: Received {} observations", node->nameId(), node->_devices.size());
    }
    node->_e_position = { 1, 1, 0 };

    Eigen::MatrixXd e_H = Eigen::MatrixXd::Zero(static_cast<int>(node->_devices.size()), static_cast<int>(2));
    Eigen::VectorXd ddist = Eigen::VectorXd::Zero(static_cast<int>(node->_devices.size()));
    size_t numMeasurements = node->_devices.size();
    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd>
        lsq;
    for (size_t o = 0; o < 10; o++)
    {
        LOG_DATA("{}: Iteration {}", nameId(), o);
        for (size_t i = 0; i < numMeasurements; i++)
        {
            double distEst = (node->_devices.at(i).position.head<2>() - node->_e_position.head<2>()).norm();
            ddist(static_cast<int>(i)) = node->_devices.at(i).distance - distEst;
            Eigen::Vector2d e_lineOfSightUnitVector = Eigen::Vector2d::Zero();
            if ((node->_e_position.head<2>() - node->_devices.at(i).position.head<2>()).norm() != 0) // Check if it is the same position
            {
                e_lineOfSightUnitVector = (node->_e_position.head<2>() - node->_devices.at(i).position.head<2>()) / (node->_e_position.head<2>() - node->_devices.at(i).position.head<2>()).norm();
            }
            e_H.block<1, 2>(static_cast<int>(i), 0) = -e_lineOfSightUnitVector;
        }
        // std::cout << "Matrix:\n"
        //           << e_H << "\n";
        Eigen::Vector2d dx = (e_H.transpose() * e_H).inverse() * e_H.transpose() * ddist;
        // LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2));
        // LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());

        node->_e_position.head<2>() += dx;
        bool solInaccurate = dx.norm() > 1e-3;

        if (!solInaccurate)
        {
            break;
        }
    }
    node->_devices.clear();
    auto pos = std::make_shared<NAV::Pos>();
    pos->setPosition_e(node->_e_position);
    LOG_DEBUG("{}: Position: {}", node->nameId(), node->_e_position.transpose());
    node->invokeCallbacks(OUTPUT_PORT_INDEX_POS, pos);
}

void NAV::WiFiPositioning::lsqSolution3d(void* userData)
{
    auto* node = static_cast<WiFiPositioning*>(userData);
    if (node->_devices.size() < 4)
    {
        LOG_DEBUG("{}: Received less than 4 observations. Can't compute position", node->nameId());
        return;
    }
    else
    {
        LOG_DEBUG("{}: Received {} observations", node->nameId(), node->_devices.size());
    }
    node->_e_position = { 1, 1, 1 };

    Eigen::MatrixXd e_H = Eigen::MatrixXd::Zero(static_cast<int>(node->_devices.size()), static_cast<int>(3));
    Eigen::VectorXd ddist = Eigen::VectorXd::Zero(static_cast<int>(node->_devices.size()));
    size_t numMeasurements = node->_devices.size();
    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd>
        lsq;
    for (size_t o = 0; o < 10; o++)
    {
        LOG_DATA("{}: Iteration {}", nameId(), o);
        for (size_t i = 0; i < numMeasurements; i++)
        {
            double distEst = (node->_devices.at(i).position - node->_e_position).norm();
            ddist(static_cast<int>(i)) = node->_devices.at(i).distance - distEst;
            Eigen::Vector3d e_lineOfSightUnitVector = Eigen::Vector3d::Zero();
            if ((node->_e_position - node->_devices.at(i).position).norm() != 0) // Check if it is the same position
            {
                e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(node->_e_position, node->_devices.at(i).position);
            }
            e_H.block<1, 3>(static_cast<int>(i), 0) = -e_lineOfSightUnitVector;
        }
        // std::cout << "Matrix:\n"
        //           << e_H << "\n";
        Eigen::Vector3d dx = (e_H.transpose() * e_H).inverse() * e_H.transpose() * ddist;
        // LOG_DATA("{}:     [{}] dx (lsq) {}, {}, {}", nameId(), o, lsq.solution(0), lsq.solution(1), lsq.solution(2));
        // LOG_DATA("{}:     [{}] stdev_dx (lsq)\n{}", nameId(), o, lsq.variance.cwiseSqrt());

        node->_e_position += dx;
        bool solInaccurate = dx.norm() > 1e-3;

        if (!solInaccurate)
        {
            break;
        }
    }
    node->_devices.clear();
    auto pos = std::make_shared<NAV::Pos>();
    pos->setPosition_e(node->_e_position);
    LOG_DEBUG("{}: Position: {}", node->nameId(), node->_e_position.transpose());
    node->invokeCallbacks(OUTPUT_PORT_INDEX_POS, pos);
}

// void NAV::WiFiPositioning::kfSolution()
// {
//     auto pos = std::make_shared<NAV::Pos>();
//     pos->setPosition_e(_e_position);
//     LOG_DEBUG("{}: Position: {}", nameId(), _e_position.transpose());
//     invokeCallbacks(OUTPUT_PORT_INDEX_POS, pos);
// }