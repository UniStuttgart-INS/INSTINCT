// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PosVelAttInitializer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "util/Time/TimeBase.hpp"
#include "util/Vendor/Ublox/UbloxTypes.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

#include <limits>

NAV::PosVelAttInitializer::PosVelAttInitializer()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 345, 342 };

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });

    updatePins();
}

NAV::PosVelAttInitializer::~PosVelAttInitializer()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PosVelAttInitializer::typeStatic()
{
    return "PosVelAttInitializer";
}

std::string NAV::PosVelAttInitializer::type() const
{
    return typeStatic();
}

std::string NAV::PosVelAttInitializer::category()
{
    return "State";
}

void NAV::PosVelAttInitializer::guiConfig()
{
    if (_inputPinIdxIMU >= 0 && inputPins.at(static_cast<size_t>(_inputPinIdxIMU)).isPinLinked()
        && !(_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2)))
    {
        ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::InputDouble(fmt::format("Initialization Duration Attitude##{}", size_t(id)).c_str(), &_initDuration, 0.0, 0.0, "%.3f s"))
        {
            flow::ApplyChanges();
        }
    }

    if (_inputPinIdxIMU >= 0 && inputPins.at(static_cast<size_t>(_inputPinIdxIMU)).isPinLinked()
        && _inputPinIdxGNSS >= 0 && inputPins.at(static_cast<size_t>(_inputPinIdxGNSS)).isPinLinked()
        && !(_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2)))
    {
        ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
        if (gui::widgets::EnumCombo(fmt::format("Attitude Init Source##{}", size_t(id)).c_str(), _attitudeMode))
        {
            LOG_DEBUG("{}: Attitude Init Source changed to {}", nameId(), to_string(_attitudeMode));
            flow::ApplyChanges();
        }
    }

    if (ImGui::BeginTable(fmt::format("Initialized State##{}", size_t(id)).c_str(),
                          4, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("Threshold");
        ImGui::TableSetupColumn("Accuracy");
        ImGui::TableHeadersRow();

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                                 Position                                                 */
        /* -------------------------------------------------------------------------------------------------------- */

        ImGui::TableNextColumn();
        ImGui::Text("Position");
        ImGui::TableNextColumn();
        float size = 7.0F;
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size / 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _posVelAttInitialized.at(0) || _overridePosition != PositionOverride::OFF
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determinePosition" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", _posVelAttInitialized.at(0) || _overridePosition != PositionOverride::OFF
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::DragDouble(("##positionAccuracyThreshold" + std::to_string(size_t(id))).c_str(),
                              &_positionAccuracyThreshold, 0.1F, 0.0, 1000.0, "%.1f cm"))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastPositionAccuracy.at(0)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", _lastPositionAccuracy.at(0));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastPositionAccuracy.at(1)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", _lastPositionAccuracy.at(1));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastPositionAccuracy.at(2)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", _lastPositionAccuracy.at(2));
        }

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                                 Velocity                                                 */
        /* -------------------------------------------------------------------------------------------------------- */

        ImGui::TableNextColumn();
        ImGui::Text("Velocity");
        ImGui::TableNextColumn();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size / 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _posVelAttInitialized.at(1) || _overrideVelocity != VelocityOverride::OFF
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determineVelocity" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", _posVelAttInitialized.at(1) || _overrideVelocity != VelocityOverride::OFF
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::DragDouble(("##velocityAccuracyThreshold" + std::to_string(size_t(id))).c_str(),
                              &_velocityAccuracyThreshold, 1.0F, 0.0, 1000.0, "%.0f cm/s"))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _lastVelocityAccuracy.at(0) <= _velocityAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastVelocityAccuracy.at(0)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", _lastVelocityAccuracy.at(0));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _lastVelocityAccuracy.at(1) <= _velocityAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastVelocityAccuracy.at(1)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", _lastVelocityAccuracy.at(1));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    _lastVelocityAccuracy.at(2) <= _velocityAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastVelocityAccuracy.at(2)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", _lastVelocityAccuracy.at(2));
        }

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                                 Attitude                                                 */
        /* -------------------------------------------------------------------------------------------------------- */

        ImGui::TableNextColumn();
        ImGui::Text("Attitude");
        ImGui::TableNextColumn();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size / 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.4F),
                                                    size,
                                                    _posVelAttInitialized.at(2) || (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determineAttitude" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", _posVelAttInitialized.at(2) || (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();

        ImGui::EndTable();
    }

    auto oldOverridePosition = _overridePosition;
    ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
    if (gui::widgets::EnumCombo(fmt::format("Override Position##{}", size_t(id)).c_str(), _overridePosition))
    {
        updatePins();
        flow::ApplyChanges();
        if (oldOverridePosition == PositionOverride::LLA && _overridePosition == PositionOverride::ECEF)
        {
            _overrideValuesPosition = trafo::lla2ecef_WGS84({ deg2rad(_overrideValuesPosition(0)),
                                                              deg2rad(_overrideValuesPosition(1)),
                                                              _overrideValuesPosition(2) });
        }
        else if (oldOverridePosition == PositionOverride::ECEF && _overridePosition == PositionOverride::LLA)
        {
            Eigen::Vector3d lla = trafo::ecef2lla_WGS84(_overrideValuesPosition);
            _overrideValuesPosition = Eigen::Vector3d(rad2deg(lla(0)), rad2deg(lla(1)), lla(2));
        }
    }
    if (_overridePosition != PositionOverride::OFF)
    {
        ImGui::Indent();

        bool lla = _overridePosition == PositionOverride::LLA;
        ImGui::SetNextItemWidth(140 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::DragDouble(fmt::format("{}##{}", lla ? "Latitude [deg]" : "ECEF X [m]", size_t(id)).c_str(), &_overrideValuesPosition(0),
                              1.0F, lla ? -90.0 : 0.0, lla ? 90.0 : 0.0, lla ? "%.9f" : "%.4f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(140 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::DragDouble(fmt::format("{}##{}", lla ? "Longitude [deg]" : "ECEF Y [m]", size_t(id)).c_str(), &_overrideValuesPosition(1),
                              1.0F, lla ? -180.0 : 0.0, lla ? 180.0 : 0.0, lla ? "%.9f" : "%.4f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(140 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::DragDouble(fmt::format("{}##{}", lla ? "Altitude [m]" : "ECEF Z [m]", size_t(id)).c_str(), &_overrideValuesPosition(2),
                              1.0F, 0.0, 0.0, "%.4f"))
        {
            flow::ApplyChanges();
        }

        ImGui::Unindent();
    }

    ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
    if (gui::widgets::EnumCombo(fmt::format("Override Velocity##{}", size_t(id)).c_str(), _overrideVelocity))
    {
        updatePins();
        flow::ApplyChanges();
    }
    if (_overrideVelocity != VelocityOverride::OFF)
    {
        ImGui::Indent();

        bool ned = _overrideVelocity == VelocityOverride::NED;
        ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::DragDouble(fmt::format("Velocity {} [m/s]##{}", ned ? "North" : "ECEF X", size_t(id)).c_str(), &_overrideValuesVelocity(0),
                              1.0F, 0.0, 0.0, "%.4f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::DragDouble(fmt::format("Velocity {} [m/s]##{}", ned ? "Eeast" : "ECEF Y", size_t(id)).c_str(), &_overrideValuesVelocity(1),
                              1.0F, 0.0, 0.0, "%.4f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100 * gui::NodeEditorApplication::windowFontRatio());
        if (ImGui::DragDouble(fmt::format("Velocity {} [m/s]##{}", ned ? "Down" : "ECEF Z", size_t(id)).c_str(), &_overrideValuesVelocity(2),
                              1.0F, 0.0, 0.0, "%.4f"))
        {
            flow::ApplyChanges();
        }

        ImGui::Unindent();
    }

    if (ImGui::BeginTable(("Overrides##" + std::to_string(size_t(id))).c_str(),
                          2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Roll##" + std::to_string(size_t(id))).c_str(), &_overrideRollPitchYaw.at(0)))
        {
            updatePins();
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (_overrideRollPitchYaw.at(0))
        {
            ImGui::SetNextItemWidth(80 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragDouble(("##overrideValuesRollPitchYaw.at(0)" + std::to_string(size_t(id))).c_str(),
                                  &_overrideValuesRollPitchYaw.at(0), 1.0F, -180.0, 180.0, "%.3f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Pitch##" + std::to_string(size_t(id))).c_str(), &_overrideRollPitchYaw.at(1)))
        {
            updatePins();
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (_overrideRollPitchYaw.at(1))
        {
            ImGui::SetNextItemWidth(80 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragDouble(("##overrideValuesRollPitchYaw.at(1)" + std::to_string(size_t(id))).c_str(),
                                  &_overrideValuesRollPitchYaw.at(1), 1.0F, -90.0, 90.0, "%.3f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Yaw##" + std::to_string(size_t(id))).c_str(), &_overrideRollPitchYaw.at(2)))
        {
            updatePins();
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (_overrideRollPitchYaw.at(2))
        {
            ImGui::SetNextItemWidth(80 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragDouble(("##overrideValuesRollPitchYaw.at(2)" + std::to_string(size_t(id))).c_str(),
                                  &_overrideValuesRollPitchYaw.at(2), 1.0, -180.0, 180.0, "%.3f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::EndTable();
    }

    if (_overridePosition != PositionOverride::OFF && _overrideVelocity != VelocityOverride::OFF
        && _overrideRollPitchYaw[0] && _overrideRollPitchYaw[1] && _overrideRollPitchYaw[2])
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Init Time##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::TimeEdit(fmt::format("Init Time Edit {}", size_t(id)).c_str(), _initTime, _initTimeEditFormat))
            {
                LOG_DEBUG("{}: initTime changed to {}", nameId(), _initTime);
                flow::ApplyChanges();
            }
            ImGui::TreePop();
        }
    }
}

[[nodiscard]] json NAV::PosVelAttInitializer::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["initDuration"] = _initDuration;
    j["attitudeMode"] = _attitudeMode;
    j["positionAccuracyThreshold"] = _positionAccuracyThreshold;
    j["velocityAccuracyThreshold"] = _velocityAccuracyThreshold;
    j["overridePosition"] = _overridePosition;
    j["overrideValuesPosition"] = _overrideValuesPosition;
    j["overrideVelocity"] = _overrideVelocity;
    j["overrideValuesVelocity"] = _overrideValuesVelocity;
    j["overrideRollPitchYaw"] = _overrideRollPitchYaw;
    j["overrideValuesRollPitchYaw"] = _overrideValuesRollPitchYaw;
    j["initTime"] = _initTime;
    j["initTimeEditFormat"] = _initTimeEditFormat;

    return j;
}

void NAV::PosVelAttInitializer::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("initDuration"))
    {
        j.at("initDuration").get_to(_initDuration);
    }
    if (j.contains("attitudeMode"))
    {
        j.at("attitudeMode").get_to(_attitudeMode);
    }
    if (j.contains("positionAccuracyThreshold"))
    {
        j.at("positionAccuracyThreshold").get_to(_positionAccuracyThreshold);
    }
    if (j.contains("velocityAccuracyThreshold"))
    {
        j.at("velocityAccuracyThreshold").get_to(_velocityAccuracyThreshold);
    }
    if (j.contains("overridePosition"))
    {
        j.at("overridePosition").get_to(_overridePosition);
    }
    if (j.contains("overrideValuesPosition"))
    {
        j.at("overrideValuesPosition").get_to(_overrideValuesPosition);
    }
    if (j.contains("overrideVelocity"))
    {
        j.at("overrideVelocity").get_to(_overrideVelocity);
    }
    if (j.contains("overrideValuesVelocity"))
    {
        j.at("overrideValuesVelocity").get_to(_overrideValuesVelocity);
    }
    if (j.contains("overrideRollPitchYaw"))
    {
        j.at("overrideRollPitchYaw").get_to(_overrideRollPitchYaw);
    }
    if (j.contains("overrideValuesRollPitchYaw"))
    {
        j.at("overrideValuesRollPitchYaw").get_to(_overrideValuesRollPitchYaw);
    }
    if (j.contains("initTime"))
    {
        j.at("initTime").get_to(_initTime);
    }
    if (j.contains("initTimeEditFormat"))
    {
        j.at("initTimeEditFormat").get_to(_initTimeEditFormat);
    }
    updatePins();
}

bool NAV::PosVelAttInitializer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _startTime = 0;

    _countAveragedAttitude = 0.0;

    _lastPositionAccuracy = { std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity() };
    _lastVelocityAccuracy = { std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity() };

    _posVelAttInitialized = { false, false, false, false };

    if (_overridePosition == PositionOverride::OFF || _overrideVelocity == VelocityOverride::OFF
        || !_overrideRollPitchYaw[0] || !_overrideRollPitchYaw[1] || !_overrideRollPitchYaw[2])
    {
        _initTime = InsTime(InsTime_GPSweekTow(0, 0, 0));
    }

    return true;
}

void NAV::PosVelAttInitializer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::PosVelAttInitializer::updatePins()
{
    if (_overrideRollPitchYaw[0] && _overrideRollPitchYaw[1] && _overrideRollPitchYaw[2] && _inputPinIdxIMU >= 0)
    {
        nm::DeleteInputPin(inputPins.at(static_cast<size_t>(_inputPinIdxIMU)));
        _inputPinIdxIMU = -1;
        _inputPinIdxGNSS--;
    }
    else if ((!_overrideRollPitchYaw[0] || !_overrideRollPitchYaw[1] || !_overrideRollPitchYaw[2]) && _inputPinIdxIMU < 0)
    {
        nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &PosVelAttInitializer::receiveImuObs, nullptr, 0, 0);
        _inputPinIdxIMU = 0;
        if (_inputPinIdxGNSS >= 0)
        {
            _inputPinIdxGNSS++;
        }
    }

    if (_overridePosition != PositionOverride::OFF
        && _overrideVelocity != VelocityOverride::OFF
        && _overrideRollPitchYaw[0] && _overrideRollPitchYaw[1] && _overrideRollPitchYaw[2] && _inputPinIdxGNSS >= 0)
    {
        nm::DeleteInputPin(inputPins.at(static_cast<size_t>(_inputPinIdxGNSS)));
        _inputPinIdxGNSS = -1;
    }
    else if ((_overridePosition == PositionOverride::OFF || _overrideVelocity == VelocityOverride::OFF
              || !_overrideRollPitchYaw[0] || !_overrideRollPitchYaw[1] || !_overrideRollPitchYaw[2])
             && _inputPinIdxGNSS < 0)
    {
        nm::CreateInputPin(this, "PosVelAttInit", Pin::Type::Flow,
                           { NAV::UbloxObs::type(), NAV::RtklibPosObs::type(), NAV::PosVelAtt::type(), NAV::PosVel::type(), NAV::Pos::type() },
                           &PosVelAttInitializer::receiveGnssObs, nullptr, 1);
        _inputPinIdxGNSS = static_cast<int>(inputPins.size()) - 1;
    }

    if (_inputPinIdxGNSS < 0 && _inputPinIdxIMU < 0)
    {
        outputPins[OUTPUT_PORT_INDEX_POS_VEL_ATT].data = static_cast<OutputPin::PollDataFunc>(&PosVelAttInitializer::pollPVASolution);
    }
    else
    {
        outputPins[OUTPUT_PORT_INDEX_POS_VEL_ATT].data = static_cast<void*>(nullptr);
    }
}

void NAV::PosVelAttInitializer::finalizeInit()
{
    if (!_posVelAttInitialized.at(3)
        && (_posVelAttInitialized.at(0) || _overridePosition != PositionOverride::OFF)
        && (_posVelAttInitialized.at(1) || _overrideVelocity != VelocityOverride::OFF)
        && (_posVelAttInitialized.at(2) || (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))))
    {
        for (auto& inputPin : inputPins)
        {
            inputPin.queueBlocked = true;
        }
        _posVelAttInitialized.at(3) = true;

        if (_overridePosition == PositionOverride::LLA)
        {
            _e_initPosition = trafo::lla2ecef_WGS84(Eigen::Vector3d(deg2rad(_overrideValuesPosition(0)),
                                                                    deg2rad(_overrideValuesPosition(1)),
                                                                    _overrideValuesPosition(2)));
        }
        else if (_overridePosition == PositionOverride::ECEF)
        {
            _e_initPosition = _overrideValuesPosition;
        }
        [[maybe_unused]] auto lla_position = trafo::ecef2lla_WGS84(_e_initPosition);
        LOG_INFO("{}: Position initialized to Lat {:3.12f} [°], Lon {:3.12f} [°], Alt {:4.3f} [m]", nameId(),
                 rad2deg(lla_position.x()),
                 rad2deg(lla_position.y()),
                 lla_position.z());

        if (_overrideVelocity == VelocityOverride::NED)
        {
            _n_initVelocity = _overrideValuesVelocity;
        }
        else if (_overrideVelocity == VelocityOverride::ECEF)
        {
            _n_initVelocity = trafo::n_Quat_e(lla_position(0), lla_position(1)) * _overrideValuesVelocity;
        }

        if (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))
        {
            _n_Quat_b_init = trafo::n_Quat_b(deg2rad(_overrideValuesRollPitchYaw.at(0)),
                                             deg2rad(_overrideValuesRollPitchYaw.at(1)),
                                             deg2rad(_overrideValuesRollPitchYaw.at(2)));
        }

        if (lla_position.z() < 0)
        {
            LOG_WARN("{}: Altitude has a value of {} which is below the ellipsoid height.", nameId(), lla_position.z());
        }

        LOG_INFO("{}: Velocity initialized to v_N {:3.5f} [m/s], v_E {:3.5f} [m/s], v_D {:3.5f} [m/s]", nameId(),
                 _n_initVelocity(0), _n_initVelocity(1), _n_initVelocity(2));

        [[maybe_unused]] auto rollPitchYaw = trafo::quat2eulerZYX(_n_Quat_b_init);
        LOG_INFO("{}: Attitude initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", nameId(),
                 rad2deg(rollPitchYaw.x()),
                 rad2deg(rollPitchYaw.y()),
                 rad2deg(rollPitchYaw.z()));

        auto posVelAtt = std::make_shared<PosVelAtt>();
        posVelAtt->insTime = _initTime;
        posVelAtt->setPosition_e(_e_initPosition);
        posVelAtt->setVelocity_n(_n_initVelocity);
        posVelAtt->setAttitude_n_Quat_b(_n_Quat_b_init);
        invokeCallbacks(OUTPUT_PORT_INDEX_POS_VEL_ATT, posVelAtt);
    }
    else if (std::all_of(inputPins.begin(), inputPins.end(), [](const InputPin& inputPin) {
                 if (auto* connectedPin = inputPin.link.getConnectedPin())
                 {
                     return connectedPin->mode == OutputPin::Mode::REAL_TIME;
                 }
                 return !inputPin.isPinLinked();
             }))
    {
        LOG_ERROR("{}: State Initialization failed. No more messages incoming to eventually receive a state. Please check which states got initialized and override the others.", nameId());
    }
}

void NAV::PosVelAttInitializer::receiveImuObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    if (_posVelAttInitialized.at(3)) { return; }
    LOG_DATA("{}: receiveImuObs at time [{}]", nameId(), obs->insTime.toYMDHMS());

    if (!obs->timeSinceStartup.has_value()) // TODO: Make this work with insTime
    {
        LOG_ERROR("{}: Can only process data with an insTime", nameId());
        return;
    }

    if (_startTime == 0)
    {
        _startTime = obs->timeSinceStartup.value();
    }

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = obs->imuPos;

    // Choose compenated data if available, otherwise uncompensated
    if (!_overrideRollPitchYaw.at(2) && !obs->magUncompXYZ.has_value())
    {
        LOG_ERROR("No magnetometer data available. Please override the Yaw angle.");
        return;
    }
    const Eigen::Vector3d& mag_p = obs->magCompXYZ.has_value() ? obs->magCompXYZ.value() : obs->magUncompXYZ.value();
    const Eigen::Vector3d& accel_p = obs->accelCompXYZ.has_value() ? obs->accelCompXYZ.value() : obs->accelUncompXYZ.value();

    // Calculate Magnetic Heading
    const Eigen::Vector3d b_mag = imuPosition.b_quatMag_p() * mag_p;
    auto magneticHeading = -std::atan2(b_mag.y(), b_mag.x());

    // Calculate Roll and Pitch from gravity vector direction (only valid under static conditions)
    const Eigen::Vector3d b_accel = imuPosition.b_quatAccel_p() * accel_p * -1;
    auto roll = calcRollFromStaticAcceleration(b_accel);
    auto pitch = calcPitchFromStaticAcceleration(b_accel);

    // TODO: Determine Velocity first and if vehicle not static, initialize the attitude from velocity

    // Average with previous attitude
    _countAveragedAttitude++;
    if (_countAveragedAttitude > 1)
    {
        _averagedAttitude.at(0) = (_averagedAttitude.at(0) * (_countAveragedAttitude - 1) + roll) / _countAveragedAttitude;
        _averagedAttitude.at(1) = (_averagedAttitude.at(1) * (_countAveragedAttitude - 1) + pitch) / _countAveragedAttitude;
        _averagedAttitude.at(2) = (_averagedAttitude.at(2) * (_countAveragedAttitude - 1) + magneticHeading) / _countAveragedAttitude;
    }
    else
    {
        _averagedAttitude.at(0) = roll;
        _averagedAttitude.at(1) = pitch;
        _averagedAttitude.at(2) = magneticHeading;
    }

    if ((_attitudeMode == AttitudeMode::BOTH || _attitudeMode == AttitudeMode::IMU
         || _inputPinIdxGNSS < 0 || !inputPins.at(static_cast<size_t>(_inputPinIdxGNSS)).isPinLinked())
        && (static_cast<double>(obs->timeSinceStartup.value() - _startTime) * 1e-9 >= _initDuration
            || (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))))
    {
        _n_Quat_b_init = trafo::n_Quat_b(_overrideRollPitchYaw.at(0) ? deg2rad(_overrideValuesRollPitchYaw.at(0)) : _averagedAttitude.at(0),
                                         _overrideRollPitchYaw.at(1) ? deg2rad(_overrideValuesRollPitchYaw.at(1)) : _averagedAttitude.at(1),
                                         _overrideRollPitchYaw.at(2) ? deg2rad(_overrideValuesRollPitchYaw.at(2)) : _averagedAttitude.at(2));

        _initTime = obs->insTime;
        _posVelAttInitialized.at(2) = true;
    }

    finalizeInit();
}

void NAV::PosVelAttInitializer::receiveGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto nodeData = queue.extract_front();

    if (_posVelAttInitialized.at(3)) { return; }
    LOG_DATA("{}: receiveGnssObs at time [{}]", nameId(), nodeData->insTime.toYMDHMS());

    const auto* sourcePin = inputPins[pinIdx].link.getConnectedPin();

    if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
    {
        receiveRtklibPosObs(std::static_pointer_cast<const RtklibPosObs>(nodeData));
    }
    else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
    {
        receiveUbloxObs(std::static_pointer_cast<const UbloxObs>(nodeData));
    }
    else if (sourcePin->dataIdentifier.front() == Pos::type())
    {
        receivePosObs(std::static_pointer_cast<const Pos>(nodeData));
    }
    else if (sourcePin->dataIdentifier.front() == PosVel::type()
             || sourcePin->dataIdentifier.front() == SppSolution::type())
    {
        receivePosVelObs(std::static_pointer_cast<const PosVel>(nodeData));
    }
    else if (sourcePin->dataIdentifier.front() == PosVelAtt::type())
    {
        receivePosVelAttObs(std::static_pointer_cast<const PosVelAtt>(nodeData));
    }

    finalizeInit();
}

void NAV::PosVelAttInitializer::receiveUbloxObs(const std::shared_ptr<const UbloxObs>& obs)
{
    if (obs->msgClass == vendor::ublox::UbxClass::UBX_CLASS_NAV)
    {
        auto msgId = static_cast<vendor::ublox::UbxNavMessages>(obs->msgId);
        if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_ATT)
        {
            LOG_DATA("{}: UBX_NAV_ATT: Roll {}, Pitch {}, Heading {} [deg]", nameId(),
                     std::get<vendor::ublox::UbxNavAtt>(obs->data).roll * 1e-5,
                     std::get<vendor::ublox::UbxNavAtt>(obs->data).pitch * 1e-5,
                     std::get<vendor::ublox::UbxNavAtt>(obs->data).heading * 1e-5);
        }
        else if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_POSECEF)
        {
            _lastPositionAccuracy.at(0) = static_cast<float>(std::get<vendor::ublox::UbxNavPosecef>(obs->data).pAcc);
            _lastPositionAccuracy.at(1) = static_cast<float>(std::get<vendor::ublox::UbxNavPosecef>(obs->data).pAcc);
            _lastPositionAccuracy.at(2) = static_cast<float>(std::get<vendor::ublox::UbxNavPosecef>(obs->data).pAcc);

            if (_lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold)
            {
                _e_initPosition = Eigen::Vector3d(std::get<vendor::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                                                  std::get<vendor::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                                                  std::get<vendor::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);
                _initTime = obs->insTime;

                _posVelAttInitialized.at(0) = true;
            }
        }
        else if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_POSLLH)
        {
            _lastPositionAccuracy.at(0) = static_cast<float>(std::get<vendor::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            _lastPositionAccuracy.at(1) = static_cast<float>(std::get<vendor::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            _lastPositionAccuracy.at(2) = static_cast<float>(std::get<vendor::ublox::UbxNavPosllh>(obs->data).vAcc * 1e-1);

            if (_lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold)
            {
                Eigen::Vector3d lla_position(deg2rad(std::get<vendor::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                             deg2rad(std::get<vendor::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                             std::get<vendor::ublox::UbxNavPosllh>(obs->data).height * 1e-3);

                _e_initPosition = trafo::lla2ecef_WGS84(lla_position);
                _initTime = obs->insTime;

                _posVelAttInitialized.at(0) = true;
            }
        }
        else if (msgId == vendor::ublox::UbxNavMessages::UBX_NAV_VELNED)
        {
            _lastVelocityAccuracy.at(0) = static_cast<float>(std::get<vendor::ublox::UbxNavVelned>(obs->data).sAcc);
            _lastVelocityAccuracy.at(1) = static_cast<float>(std::get<vendor::ublox::UbxNavVelned>(obs->data).sAcc);
            _lastVelocityAccuracy.at(2) = static_cast<float>(std::get<vendor::ublox::UbxNavVelned>(obs->data).sAcc);

            if (_lastVelocityAccuracy.at(0) <= _velocityAccuracyThreshold
                && _lastVelocityAccuracy.at(1) <= _velocityAccuracyThreshold
                && _lastVelocityAccuracy.at(2) <= _velocityAccuracyThreshold)
            {
                _n_initVelocity = Eigen::Vector3d(std::get<vendor::ublox::UbxNavVelned>(obs->data).velN * 1e-2,
                                                  std::get<vendor::ublox::UbxNavVelned>(obs->data).velE * 1e-2,
                                                  std::get<vendor::ublox::UbxNavVelned>(obs->data).velD * 1e-2);
                _initTime = obs->insTime;

                _posVelAttInitialized.at(1) = true;
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs)
{
    if (!std::isnan(obs->e_position().x()))
    {
        if (!std::isnan(obs->sdXYZ.x()))
        {
            _lastPositionAccuracy.at(0) = static_cast<float>(obs->sdXYZ.x() * 1e2);
            _lastPositionAccuracy.at(1) = static_cast<float>(obs->sdXYZ.y() * 1e2);
            _lastPositionAccuracy.at(2) = static_cast<float>(obs->sdXYZ.z() * 1e2);
        }
        else if (!std::isnan(obs->sdNED.x()))
        {
            _lastPositionAccuracy.at(0) = static_cast<float>(obs->sdNED.x() * 1e2);
            _lastPositionAccuracy.at(1) = static_cast<float>(obs->sdNED.y() * 1e2);
            _lastPositionAccuracy.at(2) = static_cast<float>(obs->sdNED.z() * 1e2);
        }

        if (_lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
            && _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
            && _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold)
        {
            _e_initPosition = obs->e_position();
            _initTime = obs->insTime;

            _posVelAttInitialized.at(0) = true;
        }
    }
}

void NAV::PosVelAttInitializer::receivePosObs(const std::shared_ptr<const Pos>& obs)
{
    _e_initPosition = obs->e_position();
    _initTime = obs->insTime;
    _posVelAttInitialized.at(0) = true;
}

void NAV::PosVelAttInitializer::receivePosVelObs(const std::shared_ptr<const PosVel>& obs)
{
    receivePosObs(obs);

    _n_initVelocity = obs->n_velocity();
    _initTime = obs->insTime;
    _posVelAttInitialized.at(1) = true;
}

void NAV::PosVelAttInitializer::receivePosVelAttObs(const std::shared_ptr<const PosVelAtt>& obs)
{
    receivePosVelObs(obs);

    if (_attitudeMode == AttitudeMode::BOTH || _attitudeMode == AttitudeMode::GNSS
        || _inputPinIdxIMU < 0 || !inputPins.at(static_cast<size_t>(_inputPinIdxIMU)).isPinLinked())
    {
        if (_overrideRollPitchYaw.at(0) || _overrideRollPitchYaw.at(1) || _overrideRollPitchYaw.at(2))
        {
            const Eigen::Vector3d rollPitchYaw = obs->rollPitchYaw();

            _n_Quat_b_init = trafo::n_Quat_b(_overrideRollPitchYaw.at(0) ? deg2rad(_overrideValuesRollPitchYaw.at(0)) : rollPitchYaw(0),
                                             _overrideRollPitchYaw.at(1) ? deg2rad(_overrideValuesRollPitchYaw.at(1)) : rollPitchYaw(1),
                                             _overrideRollPitchYaw.at(2) ? deg2rad(_overrideValuesRollPitchYaw.at(2)) : rollPitchYaw(2));
        }
        else
        {
            _n_Quat_b_init = obs->n_Quat_b();
        }
        _initTime = obs->insTime;

        _posVelAttInitialized.at(2) = true;
    }
}

std::shared_ptr<const NAV::NodeData> NAV::PosVelAttInitializer::pollPVASolution()
{
    if (_inputPinIdxIMU >= 0 || _inputPinIdxGNSS >= 0)
    {
        return nullptr;
    }

    int initCount = 0;
    if (_overridePosition == PositionOverride::LLA)
    {
        _e_initPosition = trafo::lla2ecef_WGS84(Eigen::Vector3d(deg2rad(_overrideValuesPosition(0)),
                                                                deg2rad(_overrideValuesPosition(1)),
                                                                _overrideValuesPosition(2)));
        ++initCount;
    }
    else if (_overridePosition == PositionOverride::ECEF)
    {
        _e_initPosition = _overrideValuesPosition;
        ++initCount;
    }
    if (_overrideVelocity == VelocityOverride::NED)
    {
        _n_initVelocity = _overrideValuesVelocity;
        ++initCount;
    }
    else if (_overrideVelocity == VelocityOverride::ECEF)
    {
        auto pos_lla = trafo::ecef2lla_WGS84(_e_initPosition);
        _n_initVelocity = trafo::n_Quat_e(pos_lla(0), pos_lla(1)) * _overrideValuesVelocity;
        ++initCount;
    }

    if (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))
    {
        _n_Quat_b_init = trafo::n_Quat_b(deg2rad(_overrideValuesRollPitchYaw.at(0)),
                                         deg2rad(_overrideValuesRollPitchYaw.at(1)),
                                         deg2rad(_overrideValuesRollPitchYaw.at(2)));
        ++initCount;
    }
    if (initCount == 3 && !_posVelAttInitialized.at(3))
    {
        auto posVelAtt = std::make_shared<PosVelAtt>();
        posVelAtt->insTime = _initTime;

        _posVelAttInitialized.at(3) = true;
        posVelAtt->setPosition_e(_e_initPosition);
        posVelAtt->setVelocity_n(_n_initVelocity);
        posVelAtt->setAttitude_n_Quat_b(_n_Quat_b_init);

        invokeCallbacks(OUTPUT_PORT_INDEX_POS_VEL_ATT, posVelAtt);
        return posVelAtt;
    }
    return nullptr;
}