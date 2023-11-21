// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuSimulator.hpp"

#include <ctime>

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/LocalNavFrame/Mechanization.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/State/PosVelAtt.hpp"

NAV::ImuSimulator::ImuSimulator()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 677, 580 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObsSimulated::type() }, &ImuSimulator::pollImuObs);
    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &ImuSimulator::pollPosVelAtt);
}

NAV::ImuSimulator::~ImuSimulator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuSimulator::typeStatic()
{
    return "ImuSimulator";
}

std::string NAV::ImuSimulator::type() const
{
    return typeStatic();
}

std::string NAV::ImuSimulator::category()
{
    return "Data Simulator";
}

void NAV::ImuSimulator::guiConfig()
{
    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();

    if (_trajectoryType != TrajectoryType::Csv && ImGui::TreeNode("Start Time"))
    {
        if (ImGui::RadioButton(fmt::format("Current Computer Time##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_startTimeSource), static_cast<int>(StartTimeSource::CurrentComputerTime)))
        {
            LOG_DEBUG("{}: startTimeSource changed to {}", nameId(), fmt::underlying(_startTimeSource));
            flow::ApplyChanges();
        }
        if (_startTimeSource == StartTimeSource::CurrentComputerTime)
        {
            ImGui::Indent();

            std::time_t t = std::time(nullptr);
            std::tm* now = std::localtime(&t); // NOLINT(concurrency-mt-unsafe)

            ImGui::Text("%d-%02d-%02d %02d:%02d:%02d", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);

            ImGui::Unindent();
        }

        if (ImGui::RadioButton(fmt::format("Custom Time##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_startTimeSource), static_cast<int>(StartTimeSource::CustomTime)))
        {
            LOG_DEBUG("{}: startTimeSource changed to {}", nameId(), fmt::underlying(_startTimeSource));
            flow::ApplyChanges();
        }
        if (_startTimeSource == StartTimeSource::CustomTime)
        {
            ImGui::Indent();
            if (gui::widgets::TimeEdit(fmt::format("{}", size_t(id)).c_str(), _startTime, _startTimeEditFormat))
            {
                LOG_DEBUG("{}: startTime changed to {}", nameId(), _startTime);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Output datarate"))
    {
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("IMU output rate##{}", size_t(id)).c_str(), &_imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.3f Hz"))
        {
            LOG_DEBUG("{}: imuFrequency changed to {}", nameId(), _imuFrequency);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDouble(fmt::format("Trajectory output rate##{}", size_t(id)).c_str(), &_gnssFrequency, 0.0, 0.0, "%.3f Hz"))
        {
            LOG_DEBUG("{}: gnssFrequency changed to {}", nameId(), _gnssFrequency);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Scenario"))
    {
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::BeginCombo(fmt::format("Trajectory##{}", size_t(id)).c_str(), to_string(_trajectoryType)))
        {
            for (size_t i = 0; i < static_cast<size_t>(TrajectoryType::COUNT); i++)
            {
                const bool is_selected = (static_cast<size_t>(_trajectoryType) == i);
                if (ImGui::Selectable(to_string(static_cast<TrajectoryType>(i)), is_selected))
                {
                    _trajectoryType = static_cast<TrajectoryType>(i);
                    LOG_DEBUG("{}: trajectoryType changed to {}", nameId(), fmt::underlying(_trajectoryType));

                    if (_trajectoryType == TrajectoryType::Csv && inputPins.empty())
                    {
                        nm::CreateInputPin(this, CsvData::type().c_str(), Pin::Type::Object, { CsvData::type() });
                    }
                    else if (_trajectoryType != TrajectoryType::Csv && !inputPins.empty())
                    {
                        nm::DeleteInputPin(inputPins.front());
                    }

                    flow::ApplyChanges();
                    doDeinitialize();
                }

                if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        if (_trajectoryType == TrajectoryType::Csv)
        {
            auto TextColoredIfExists = [this](int index, const char* text, const char* type) {
                ImGui::TableSetColumnIndex(index);
                auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_CSV);
                if (mutex) { mutex->lock(); }
                if (const auto* csvData = getInputValue<const CsvData>(INPUT_PORT_INDEX_CSV);
                    csvData && std::find(csvData->description.begin(), csvData->description.end(), text) != csvData->description.end())
                {
                    ImGui::TextUnformatted(text);
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(type);
                }
                else
                {
                    ImGui::TextDisabled("%s", text);
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", type);
                }
                if (mutex) { mutex->unlock(); }
            };

            if (ImGui::TreeNode(fmt::format("Format description##{}", size_t(id)).c_str()))
            {
                ImGui::TextUnformatted("Time information:");
                ImGui::SameLine();
                gui::widgets::HelpMarker("You can either provide the time in GPS time or in UTC time.");
                if (ImGui::BeginTable(fmt::format("##Time ({})", size_t(id)).c_str(), 5,
                                      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoHostExtendX))
                {
                    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableHeadersRow();

                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "GpsCycle", "uint");
                    TextColoredIfExists(3, "YearUTC", "uint");
                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "GpsWeek", "uint");
                    TextColoredIfExists(3, "MonthUTC", "uint");
                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "GpsTow [s]", "uint");
                    TextColoredIfExists(3, "DayUTC", "uint");
                    ImGui::TableNextRow();
                    TextColoredIfExists(3, "HourUTC", "uint");
                    ImGui::TableNextRow();
                    TextColoredIfExists(3, "MinUTC", "uint");
                    ImGui::TableNextRow();
                    TextColoredIfExists(3, "SecUTC", "double");

                    ImGui::EndTable();
                }

                ImGui::TextUnformatted("Position information:");
                ImGui::SameLine();
                gui::widgets::HelpMarker("You can either provide the position in ECEF coordinates\nor in latitude, longitude and altitude.");
                if (ImGui::BeginTable(fmt::format("##Position ({})", size_t(id)).c_str(), 5,
                                      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoHostExtendX))
                {
                    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableHeadersRow();

                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "Pos ECEF X [m]", "double");
                    TextColoredIfExists(3, "Latitude [deg]", "double");
                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "Pos ECEF Y [m]", "double");
                    TextColoredIfExists(3, "Longitude [deg]", "double");
                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "Pos ECEF Z [m]", "double");
                    TextColoredIfExists(3, "Altitude [m]", "double");

                    ImGui::EndTable();
                }

                ImGui::TextUnformatted("Attitude information:");
                ImGui::SameLine();
                gui::widgets::HelpMarker("This is optional. If not provided the simulator will calculate\nit using a rotation minimizing frame.\n\n"
                                         "You can either provide the attitude as an angle or quaternion.");
                if (ImGui::BeginTable(fmt::format("##Attitude ({})", size_t(id)).c_str(), 5,
                                      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoHostExtendX))
                {
                    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableHeadersRow();

                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "Roll [deg]", "double");
                    TextColoredIfExists(3, "n_Quat_b w", "double");
                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "Pitch [deg]", "double");
                    TextColoredIfExists(3, "n_Quat_b x", "double");
                    ImGui::TableNextRow();
                    TextColoredIfExists(0, "Yaw [deg]", "double");
                    TextColoredIfExists(3, "n_Quat_b y", "double");
                    ImGui::TableNextRow();
                    TextColoredIfExists(3, "n_Quat_b z", "double");

                    ImGui::EndTable();
                }

                ImGui::TreePop();
            }
        }
        else
        {
            const auto* txt = _trajectoryType == TrajectoryType::Fixed
                                  ? "Position"
                                  : (_trajectoryType == TrajectoryType::Linear
                                         ? "Start position"
                                         : (_trajectoryType == TrajectoryType::Circular
                                                ? "Center position"
                                                : (_trajectoryType == TrajectoryType::RoseFigure
                                                       ? "Center/Tangential position"
                                                       : "")));

            if (gui::widgets::PositionInput(fmt::format("{}##PosInput {}", txt, size_t(id)).c_str(), _startPosition, gui::widgets::PositionInputLayout::TWO_ROWS, columnWidth))
            {
                flow::ApplyChanges();
                doDeinitialize();
            }
        }

        if (_trajectoryType == TrajectoryType::Fixed)
        {
            ImGui::SetNextItemWidth(columnWidth);
            double roll = rad2deg(_fixedTrajectoryStartOrientation.x());
            if (ImGui::InputDoubleL(fmt::format("##Roll{}", size_t(id)).c_str(), &roll, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.x() = deg2rad(roll);
                LOG_DEBUG("{}: roll changed to {}", nameId(), roll);
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double pitch = rad2deg(_fixedTrajectoryStartOrientation.y());
            if (ImGui::InputDoubleL(fmt::format("##Pitch{}", size_t(id)).c_str(), &pitch, -90, 90, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.y() = deg2rad(pitch);
                LOG_DEBUG("{}: pitch changed to {}", nameId(), pitch);
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double yaw = rad2deg(_fixedTrajectoryStartOrientation.z());
            if (ImGui::InputDoubleL(fmt::format("##Yaw{}", size_t(id)).c_str(), &yaw, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.z() = deg2rad(yaw);
                LOG_DEBUG("{}: yaw changed to {}", nameId(), yaw);
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Orientation (Roll, Pitch, Yaw)");
        }
        else if (_trajectoryType == TrajectoryType::Linear)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##North velocity{}", size_t(id)).c_str(), &_n_linearTrajectoryStartVelocity.x(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: n_linearTrajectoryStartVelocity changed to {}", nameId(), _n_linearTrajectoryStartVelocity.transpose());
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##East velocity{}", size_t(id)).c_str(), &_n_linearTrajectoryStartVelocity.y(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: n_linearTrajectoryStartVelocity changed to {}", nameId(), _n_linearTrajectoryStartVelocity.transpose());
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##Down velocity{}", size_t(id)).c_str(), &_n_linearTrajectoryStartVelocity.z(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: n_linearTrajectoryStartVelocity changed to {}", nameId(), _n_linearTrajectoryStartVelocity.transpose());
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Velocity (North, East, Down)");
        }
        else if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::RoseFigure)
        {
            if (ImGui::BeginTable(fmt::format("CircularOrRoseTrajectory##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX))
            {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDoubleL(fmt::format("{}##{}", _trajectoryType == TrajectoryType::Circular ? "Radius" : "Radius/Amplitude", size_t(id)).c_str(), &_trajectoryRadius, 1e-3, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f m"))
                {
                    LOG_DEBUG("{}: circularTrajectoryRadius changed to {}", nameId(), _trajectoryRadius);
                    flow::ApplyChanges();
                    doDeinitialize();
                }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("Horizontal speed##{}", size_t(id)).c_str(), &_trajectoryHorizontalSpeed, 0.0, 0.0, "%.3f m/s"))
                {
                    LOG_DEBUG("{}: circularTrajectoryHorizontalSpeed changed to {}", nameId(), _trajectoryHorizontalSpeed);
                    flow::ApplyChanges();
                    doDeinitialize();
                }
                // ####################################################################################################
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                double originAngle = rad2deg(_trajectoryRotationAngle);
                if (ImGui::DragDouble(fmt::format("{}##{}", _trajectoryType == TrajectoryType::Circular ? "Origin angle" : "Rotation angle", size_t(id)).c_str(), &originAngle, 15.0, -360.0, 360.0, "%.3f°"))
                {
                    _trajectoryRotationAngle = deg2rad(originAngle);
                    LOG_DEBUG("{}: originAngle changed to {}", nameId(), originAngle);
                    flow::ApplyChanges();
                    doDeinitialize();
                }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("Vertical speed (Up)##{}", size_t(id)).c_str(), &_trajectoryVerticalSpeed, 0.0, 0.0, "%.3f m/s"))
                {
                    LOG_DEBUG("{}: circularTrajectoryVerticalSpeed changed to {}", nameId(), _trajectoryVerticalSpeed);
                    flow::ApplyChanges();
                    doDeinitialize();
                }
                // ####################################################################################################
                ImGui::TableNextColumn();
                auto tableStartX = ImGui::GetCursorPosX();
                ImGui::SetNextItemWidth(200 * gui::NodeEditorApplication::windowFontRatio());
                if (ImGui::BeginCombo(fmt::format("Motion##{}", size_t(id)).c_str(), to_string(_trajectoryDirection)))
                {
                    for (size_t i = 0; i < static_cast<size_t>(Direction::COUNT); i++)
                    {
                        const bool is_selected = (static_cast<size_t>(_trajectoryDirection) == i);
                        if (ImGui::Selectable(to_string(static_cast<Direction>(i)), is_selected))
                        {
                            _trajectoryDirection = static_cast<Direction>(i);
                            LOG_DEBUG("{}: circularTrajectoryDirection changed to {}", nameId(), fmt::underlying(_trajectoryDirection));
                            flow::ApplyChanges();
                            doDeinitialize();
                        }

                        if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::SetCursorPosX(tableStartX + columnWidth * 2.0F + ImGui::GetStyle().ItemSpacing.x * 1.0F);

                ImGui::TableNextColumn();
                // ####################################################################################################
                if (_trajectoryType == TrajectoryType::Circular)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(columnWidth);
                    if (ImGui::DragInt(fmt::format("Osc Frequency##{}", size_t(id)).c_str(), &_circularHarmonicFrequency, 1.0F, 0, 100, "%d [cycles/rev]"))
                    {
                        LOG_DEBUG("{}: circularHarmonicFrequency changed to {}", nameId(), _circularHarmonicFrequency);
                        flow::ApplyChanges();
                        doDeinitialize();
                    }
                    ImGui::SameLine();
                    gui::widgets::HelpMarker("This modulates a harmonic oscillation on the circular path.\n"
                                             "The frequency is in units [cycles per revolution].");

                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(columnWidth);
                    if (ImGui::DragDouble(fmt::format("Osc Amplitude Factor##{}", size_t(id)).c_str(), &_circularHarmonicAmplitudeFactor, 0.01F, 0.0, 10.0, "%.3f * r"))
                    {
                        LOG_DEBUG("{}: circularHarmonicAmplitudeFactor changed to {}", nameId(), _circularHarmonicAmplitudeFactor);
                        flow::ApplyChanges();
                        doDeinitialize();
                    }
                    ImGui::SameLine();
                    gui::widgets::HelpMarker("This modulates a harmonic oscillation on the circular path.\n"
                                             "This factor determines the amplitude of the oscillation\n"
                                             "with respect to the radius of the circle.");
                }
                else if (_trajectoryType == TrajectoryType::RoseFigure)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(columnWidth);
                    if (ImGui::InputIntL(fmt::format("n (angular freq.)##{}", size_t(id)).c_str(), &_rosePetNum, 1.0, std::numeric_limits<int>::max(), 1, 1))
                    {
                        LOG_DEBUG("{}: Rose figure numerator changed to {}", nameId(), _rosePetNum);
                        flow::ApplyChanges();
                        doDeinitialize();
                    }

                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(columnWidth);
                    if (ImGui::InputIntL(fmt::format("d (angular freq.)##{}", size_t(id)).c_str(), &_rosePetDenom, 1.0, std::numeric_limits<int>::max(), 1, 1))
                    {
                        LOG_DEBUG("{}: Rose figure denominator changed to {}", nameId(), _rosePetDenom);
                        flow::ApplyChanges();
                        doDeinitialize();
                    }
                    ImGui::SameLine();
                    if (gui::widgets::BeginHelpMarker())
                    {
                        ImGui::TextUnformatted("Angular frequency k=n/d, n,d = natural numbers. In case of d=1,\n"
                                               "for even k number of petals is 2*k, for odd k, number of petals is k.\n"
                                               "Adjusts the integration limits of the incomplete elliptical integral\n"
                                               "of the second kind. If k is rational (d >1), the correct limit needs \n"
                                               "to be choosen depending on the symmetry (petals) of the figure.");
                        float width = 500.0F;
                        ImGui::Image(gui::NodeEditorApplication::m_RoseFigure, ImVec2(width, width * 909.0F / 706.0F));
                        ImGui::TextUnformatted("Graphic by Jason Davies, CC BY-SA 3.0,\nhttps://commons.wikimedia.org/w/index.php?curid=30515231");
                        gui::widgets::EndHelpMarker();
                    }
                }
                // ####################################################################################################

                ImGui::EndTable();
            }
        }
        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (_trajectoryType != TrajectoryType::Csv && ImGui::TreeNode("Simulation Stop Condition"))
    {
        if (_trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDuration{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_simulationStopCondition), static_cast<int>(StopCondition::Duration)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), fmt::underlying(_simulationStopCondition));
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
        }

        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Duration##{}", size_t(id)).c_str(), &_simulationDuration, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f s"))
        {
            LOG_DEBUG("{}: simulationDuration changed to {}", nameId(), _simulationDuration);
            flow::ApplyChanges();
            doDeinitialize();
        }
        if (_trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDistanceOrCirclesOrRoses{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_simulationStopCondition), static_cast<int>(StopCondition::DistanceOrCirclesOrRoses)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), fmt::underlying(_simulationStopCondition));
                flow::ApplyChanges();
                doDeinitialize();
            }
            ImGui::SameLine();
        }

        if (_trajectoryType == TrajectoryType::Linear)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Distance to start##{}", size_t(id)).c_str(), &_linearTrajectoryDistanceForStop, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f m"))
            {
                LOG_DEBUG("{}: linearTrajectoryDistanceForStop changed to {}", nameId(), _linearTrajectoryDistanceForStop);
                flow::ApplyChanges();
                doDeinitialize();
            }
        }
        else if (_trajectoryType == TrajectoryType::Circular)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Amount of Circles##{}", size_t(id)).c_str(), &_circularTrajectoryCircleCountForStop, 0.0, std::numeric_limits<double>::max(), 1.0, 1.0, "%.3f"))
            {
                LOG_DEBUG("{}: circularTrajectoryCircleCountForStop changed to {}", nameId(), _circularTrajectoryCircleCountForStop);
                flow::ApplyChanges();
                doDeinitialize();
            }
        }
        else if (_trajectoryType == TrajectoryType::RoseFigure)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Amount of rose figures##{}", size_t(id)).c_str(), &_roseTrajectoryCountForStop, 0.0, std::numeric_limits<double>::max(), 1.0, 1.0, "%.3f"))
            {
                LOG_DEBUG("{}: RoseTrajectoryCountForStop changed to {}", nameId(), _roseTrajectoryCountForStop);
                flow::ApplyChanges();
                doDeinitialize();
            }
        }
        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Simulation models"))
    {
        if (_trajectoryType != TrajectoryType::Fixed && _trajectoryType != TrajectoryType::Csv && _trajectoryType != TrajectoryType::RoseFigure)
        {
            ImGui::TextUnformatted(fmt::format("Spline (current knots {})", _splines.x.size()).c_str());
            {
                ImGui::Indent();
                ImGui::SetNextItemWidth(columnWidth - ImGui::GetStyle().IndentSpacing);
                if (ImGui::InputDoubleL(fmt::format("Sample Interval##{}", size_t(id)).c_str(), &_splines.sampleInterval, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3e s"))
                {
                    LOG_DEBUG("{}: spline sample interval changed to {}", nameId(), _splines.sampleInterval);
                    flow::ApplyChanges();
                    doDeinitialize();
                }
                ImGui::Unindent();
            }
        }
        else if (_trajectoryType == TrajectoryType::RoseFigure)
        {
            ImGui::TextUnformatted(fmt::format("Spline (current knots {})", _splines.x.size()).c_str());
            {
                ImGui::Indent();
                ImGui::SetNextItemWidth(columnWidth - ImGui::GetStyle().IndentSpacing);
                if (ImGui::InputDoubleL(fmt::format("Sample Distance##{}", size_t(id)).c_str(), &_roseStepLengthMax, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3e m"))
                {
                    LOG_DEBUG("{}: Spline sample distance (rose figure) changed to {}", nameId(), _roseStepLengthMax);
                    flow::ApplyChanges();
                    doDeinitialize();
                }
                ImGui::Unindent();
            }
        }
        ImGui::TextUnformatted("Measured acceleration");
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(columnWidth - ImGui::GetStyle().IndentSpacing);

            if (ComboGravitationModel(fmt::format("Gravitation Model##{}", size_t(id)).c_str(), _gravitationModel))
            {
                LOG_DEBUG("{}: Gravitation Model changed to {}", nameId(), NAV::to_string(_gravitationModel));
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Coriolis acceleration##{}", size_t(id)).c_str(), &_coriolisAccelerationEnabled))
            {
                LOG_DEBUG("{}: coriolisAccelerationEnabled changed to {}", nameId(), _coriolisAccelerationEnabled);
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Centrifugal acceleration##{}", size_t(id)).c_str(), &_centrifgalAccelerationEnabled))
            {
                LOG_DEBUG("{}: centrifgalAccelerationEnabled changed to {}", nameId(), _centrifgalAccelerationEnabled);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TextUnformatted("Measured angular rates");
        {
            ImGui::Indent();
            if (ImGui::Checkbox(fmt::format("Earth rotation rate##{}", size_t(id)).c_str(), &_angularRateEarthRotationEnabled))
            {
                LOG_DEBUG("{}: angularRateEarthRotationEnabled changed to {}", nameId(), _angularRateEarthRotationEnabled);
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Transport rate##{}", size_t(id)).c_str(), &_angularRateTransportRateEnabled))
            {
                LOG_DEBUG("{}: angularRateTransportRateEnabled changed to {}", nameId(), _angularRateTransportRateEnabled);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }

        ImGui::TreePop();
    }

    Imu::guiConfig();
}

json NAV::ImuSimulator::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["startTimeSource"] = _startTimeSource;
    j["startTimeEditFormat"] = _startTimeEditFormat;
    j["startTime"] = _startTime;
    // ###########################################################################################################
    j["imuFrequency"] = _imuFrequency;
    j["gnssFrequency"] = _gnssFrequency;
    // ###########################################################################################################
    j["trajectoryType"] = _trajectoryType;
    j["startPosition"] = _startPosition;
    j["fixedTrajectoryStartOrientation"] = _fixedTrajectoryStartOrientation;
    j["n_linearTrajectoryStartVelocity"] = _n_linearTrajectoryStartVelocity;
    j["circularHarmonicFrequency"] = _circularHarmonicFrequency;
    j["circularHarmonicAmplitudeFactor"] = _circularHarmonicAmplitudeFactor;
    j["trajectoryHorizontalSpeed"] = _trajectoryHorizontalSpeed;
    j["trajectoryVerticalSpeed"] = _trajectoryVerticalSpeed;
    j["trajectoryRadius"] = _trajectoryRadius;
    j["trajectoryRotationAngle"] = _trajectoryRotationAngle;
    j["trajectoryDirection"] = _trajectoryDirection;
    j["rosePetNum"] = _rosePetNum;
    j["rosePetDenom"] = _rosePetDenom;
    //  ###########################################################################################################
    j["simulationStopCondition"] = _simulationStopCondition;
    j["simulationDuration"] = _simulationDuration;
    j["linearTrajectoryDistanceForStop"] = _linearTrajectoryDistanceForStop;
    j["circularTrajectoryCircleCountForStop"] = _circularTrajectoryCircleCountForStop;
    j["roseTrajectoryCountForStop"] = _roseTrajectoryCountForStop;
    // ###########################################################################################################
    j["splineSampleInterval"] = _splines.sampleInterval;
    j["roseStepLengthMax"] = _roseStepLengthMax;
    j["gravitationModel"] = _gravitationModel;
    j["coriolisAccelerationEnabled"] = _coriolisAccelerationEnabled;
    j["centrifgalAccelerationEnabled"] = _centrifgalAccelerationEnabled;
    j["angularRateEarthRotationEnabled"] = _angularRateEarthRotationEnabled;
    j["angularRateTransportRateEnabled"] = _angularRateTransportRateEnabled;
    // ###########################################################################################################
    j["Imu"] = Imu::save();

    return j;
}

void NAV::ImuSimulator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("startTimeSource"))
    {
        j.at("startTimeSource").get_to(_startTimeSource);
    }
    if (j.contains("startTimeEditFormat"))
    {
        j.at("startTimeEditFormat").get_to(_startTimeEditFormat);
    }
    if (j.contains("startTime"))
    {
        j.at("startTime").get_to(_startTime);
    }
    // ###########################################################################################################
    if (j.contains("imuFrequency"))
    {
        j.at("imuFrequency").get_to(_imuFrequency);
    }
    if (j.contains("gnssFrequency"))
    {
        j.at("gnssFrequency").get_to(_gnssFrequency);
    }
    // ###########################################################################################################
    if (j.contains("trajectoryType"))
    {
        j.at("trajectoryType").get_to(_trajectoryType);

        if (_trajectoryType == TrajectoryType::Csv && inputPins.empty())
        {
            nm::CreateInputPin(this, CsvData::type().c_str(), Pin::Type::Object, { CsvData::type() });
        }
        else if (_trajectoryType != TrajectoryType::Csv && !inputPins.empty())
        {
            nm::DeleteInputPin(inputPins.front());
        }
    }
    if (j.contains("startPosition"))
    {
        j.at("startPosition").get_to(_startPosition);
    }
    if (j.contains("fixedTrajectoryStartOrientation"))
    {
        j.at("fixedTrajectoryStartOrientation").get_to(_fixedTrajectoryStartOrientation);
    }
    if (j.contains("n_linearTrajectoryStartVelocity"))
    {
        j.at("n_linearTrajectoryStartVelocity").get_to(_n_linearTrajectoryStartVelocity);
    }
    if (j.contains("circularHarmonicFrequency"))
    {
        j.at("circularHarmonicFrequency").get_to(_circularHarmonicFrequency);
    }
    if (j.contains("circularHarmonicAmplitudeFactor"))
    {
        j.at("circularHarmonicAmplitudeFactor").get_to(_circularHarmonicAmplitudeFactor);
    }
    if (j.contains("trajectoryHorizontalSpeed"))
    {
        j.at("trajectoryHorizontalSpeed").get_to(_trajectoryHorizontalSpeed);
    }
    if (j.contains("trajectoryVerticalSpeed"))
    {
        j.at("trajectoryVerticalSpeed").get_to(_trajectoryVerticalSpeed);
    }
    if (j.contains("trajectoryRadius"))
    {
        j.at("trajectoryRadius").get_to(_trajectoryRadius);
    }
    if (j.contains("trajectoryRotationAngle"))
    {
        j.at("trajectoryRotationAngle").get_to(_trajectoryRotationAngle);
    }
    if (j.contains("trajectoryDirection"))
    {
        j.at("trajectoryDirection").get_to(_trajectoryDirection);
    }
    if (j.contains("rosePetNum"))
    {
        j.at("rosePetNum").get_to(_rosePetNum);
    }
    if (j.contains("rosePetDenom"))
    {
        j.at("rosePetDenom").get_to(_rosePetDenom);
    }
    //  ###########################################################################################################
    if (j.contains("simulationStopCondition"))
    {
        j.at("simulationStopCondition").get_to(_simulationStopCondition);
    }
    if (j.contains("simulationDuration"))
    {
        j.at("simulationDuration").get_to(_simulationDuration);
    }
    if (j.contains("linearTrajectoryDistanceForStop"))
    {
        j.at("linearTrajectoryDistanceForStop").get_to(_linearTrajectoryDistanceForStop);
    }
    if (j.contains("circularTrajectoryCircleCountForStop"))
    {
        j.at("circularTrajectoryCircleCountForStop").get_to(_circularTrajectoryCircleCountForStop);
    }
    if (j.contains("roseTrajectoryCountForStop"))
    {
        j.at("roseTrajectoryCountForStop").get_to(_roseTrajectoryCountForStop);
    }
    // ###########################################################################################################
    if (j.contains("splineSampleInterval"))
    {
        j.at("splineSampleInterval").get_to(_splines.sampleInterval);
    }
    if (j.contains("roseStepLengthMax"))
    {
        j.at("roseStepLengthMax").get_to(_roseStepLengthMax);
    }
    if (j.contains("gravitationModel"))
    {
        j.at("gravitationModel").get_to(_gravitationModel);
    }
    if (j.contains("coriolisAccelerationEnabled"))
    {
        j.at("coriolisAccelerationEnabled").get_to(_coriolisAccelerationEnabled);
    }
    if (j.contains("centrifgalAccelerationEnabled"))
    {
        j.at("centrifgalAccelerationEnabled").get_to(_centrifgalAccelerationEnabled);
    }
    if (j.contains("angularRateEarthRotationEnabled"))
    {
        j.at("angularRateEarthRotationEnabled").get_to(_angularRateEarthRotationEnabled);
    }
    if (j.contains("angularRateTransportRateEnabled"))
    {
        j.at("angularRateTransportRateEnabled").get_to(_angularRateTransportRateEnabled);
    }
    // ###########################################################################################################
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::ImuSimulator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return initializeSplines();
}

NAV::InsTime NAV::ImuSimulator::getTimeFromCsvLine(const CsvData::CsvLine& line, const std::vector<std::string>& description) const
{
    auto gpsCycleIter = std::find(description.begin(), description.end(), "GpsCycle");
    auto gpsWeekIter = std::find(description.begin(), description.end(), "GpsWeek");
    auto gpsTowIter = std::find(description.begin(), description.end(), "GpsTow [s]");
    if (gpsCycleIter != description.end() && gpsWeekIter != description.end() && gpsTowIter != description.end())
    {
        auto gpsCycle = static_cast<int32_t>(std::get<double>(line.at(static_cast<size_t>(gpsCycleIter - description.begin()))));
        auto gpsWeek = static_cast<int32_t>(std::get<double>(line.at(static_cast<size_t>(gpsWeekIter - description.begin()))));
        auto gpsTow = std::get<double>(line.at(static_cast<size_t>(gpsTowIter - description.begin())));
        return { gpsCycle, gpsWeek, gpsTow };
    }

    auto yearUTCIter = std::find(description.begin(), description.end(), "YearUTC");
    auto monthUTCIter = std::find(description.begin(), description.end(), "MonthUTC");
    auto dayUTCIter = std::find(description.begin(), description.end(), "DayUTC");
    auto hourUTCIter = std::find(description.begin(), description.end(), "HourUTC");
    auto minUTCIter = std::find(description.begin(), description.end(), "MinUTC");
    auto secUTCIter = std::find(description.begin(), description.end(), "SecUTC");
    if (yearUTCIter != description.end() && monthUTCIter != description.end() && dayUTCIter != description.end()
        && hourUTCIter != description.end() && minUTCIter != description.end() && secUTCIter != description.end())
    {
        auto yearUTC = static_cast<uint16_t>(std::get<double>(line.at(static_cast<size_t>(yearUTCIter - description.begin()))));
        auto monthUTC = static_cast<uint16_t>(std::get<double>(line.at(static_cast<size_t>(monthUTCIter - description.begin()))));
        auto dayUTC = static_cast<uint16_t>(std::get<double>(line.at(static_cast<size_t>(dayUTCIter - description.begin()))));
        auto hourUTC = static_cast<uint16_t>(std::get<double>(line.at(static_cast<size_t>(hourUTCIter - description.begin()))));
        auto minUTC = static_cast<uint16_t>(std::get<double>(line.at(static_cast<size_t>(minUTCIter - description.begin()))));
        auto secUTC = std::get<double>(line.at(static_cast<size_t>(secUTCIter - description.begin())));
        return { yearUTC, monthUTC, dayUTC, hourUTC, minUTC, secUTC, UTC };
    }

    LOG_ERROR("{}: Could not find the necessary columns in the CSV file to determine the time.", nameId());
    return {};
}

Eigen::Vector3d NAV::ImuSimulator::e_getPositionFromCsvLine(const CsvData::CsvLine& line, const std::vector<std::string>& description) const
{
    auto posXIter = std::find(description.begin(), description.end(), "Pos ECEF X [m]");
    auto posYIter = std::find(description.begin(), description.end(), "Pos ECEF Y [m]");
    auto posZIter = std::find(description.begin(), description.end(), "Pos ECEF Z [m]");
    if (posXIter != description.end() && posYIter != description.end() && posZIter != description.end())
    {
        auto posX = std::get<double>(line.at(static_cast<size_t>(posXIter - description.begin())));
        auto posY = std::get<double>(line.at(static_cast<size_t>(posYIter - description.begin())));
        auto posZ = std::get<double>(line.at(static_cast<size_t>(posZIter - description.begin())));
        return { posX, posY, posZ };
    }

    auto latIter = std::find(description.begin(), description.end(), "Latitude [deg]");
    auto lonIter = std::find(description.begin(), description.end(), "Longitude [deg]");
    auto altIter = std::find(description.begin(), description.end(), "Altitude [m]");
    if (latIter != description.end() && lonIter != description.end() && altIter != description.end())
    {
        auto lat = deg2rad(std::get<double>(line.at(static_cast<size_t>(latIter - description.begin()))));
        auto lon = deg2rad(std::get<double>(line.at(static_cast<size_t>(lonIter - description.begin()))));
        auto alt = std::get<double>(line.at(static_cast<size_t>(altIter - description.begin())));
        return trafo::lla2ecef_WGS84({ lat, lon, alt });
    }

    LOG_ERROR("{}: Could not find the necessary columns in the CSV file to determine the position.", nameId());
    return { std::nan(""), std::nan(""), std::nan("") };
}

Eigen::Quaterniond NAV::ImuSimulator::n_getAttitudeQuaternionFromCsvLine_b(const CsvData::CsvLine& line, const std::vector<std::string>& description)
{
    auto rollIter = std::find(description.begin(), description.end(), "Roll [deg]");
    auto pitchIter = std::find(description.begin(), description.end(), "Pitch [deg]");
    auto yawIter = std::find(description.begin(), description.end(), "Yaw [deg]");
    if (rollIter != description.end() && pitchIter != description.end() && yawIter != description.end())
    {
        auto roll = std::get<double>(line.at(static_cast<size_t>(rollIter - description.begin())));
        auto pitch = std::get<double>(line.at(static_cast<size_t>(pitchIter - description.begin())));
        auto yaw = std::get<double>(line.at(static_cast<size_t>(yawIter - description.begin())));
        return trafo::n_Quat_b(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
    }

    auto quatWIter = std::find(description.begin(), description.end(), "n_Quat_b w");
    auto quatXIter = std::find(description.begin(), description.end(), "n_Quat_b x");
    auto quatYIter = std::find(description.begin(), description.end(), "n_Quat_b y");
    auto quatZIter = std::find(description.begin(), description.end(), "n_Quat_b z");
    if (quatWIter != description.end() && quatXIter != description.end() && quatYIter != description.end() && quatZIter != description.end())
    {
        auto w = std::get<double>(line.at(static_cast<size_t>(quatWIter - description.begin())));
        auto x = std::get<double>(line.at(static_cast<size_t>(quatXIter - description.begin())));
        auto y = std::get<double>(line.at(static_cast<size_t>(quatYIter - description.begin())));
        auto z = std::get<double>(line.at(static_cast<size_t>(quatZIter - description.begin())));
        return { w, x, y, z };
    }

    return { std::nan(""), std::nan(""), std::nan(""), std::nan("") };
}

bool NAV::ImuSimulator::initializeSplines()
{
    std::vector<double> splineTime;

    auto unwrapAngle = [](double angle, double prevAngle, double rangeMax) {
        double x = angle - prevAngle;
        x = std::fmod(x + rangeMax, 2 * rangeMax);
        if (x < 0)
        {
            x += 2 * rangeMax;
        }
        x -= rangeMax;

        return prevAngle + x;
    };

    if (_trajectoryType == TrajectoryType::Fixed)
    {
        splineTime.push_back(-30.0); // 10 seconds in the past; simply to define the derivative at zero seconds
        splineTime.push_back(-20.0); // 10 seconds in the past; simply to define the derivative at zero seconds
        splineTime.push_back(-10.0); // 10 seconds in the past; simply to define the derivative at zero seconds
        splineTime.push_back(0.0);
        splineTime.push_back(_simulationDuration);
        splineTime.push_back(_simulationDuration + 10.0); // 10 seconds past simulation end; simply to define the derivative at end node
        splineTime.push_back(_simulationDuration + 20.0); // 10 seconds past simulation end; simply to define the derivative at end node
        splineTime.push_back(_simulationDuration + 30.0); // 10 seconds past simulation end; simply to define the derivative at end node

        const Eigen::Vector3d& e_startPosition = _startPosition.e_position;
        std::vector<double> X(splineTime.size(), e_startPosition[0]);
        std::vector<double> Y(splineTime.size(), e_startPosition[1]);
        std::vector<double> Z(splineTime.size(), e_startPosition[2]);
        std::vector<double> Roll(splineTime.size(), _fixedTrajectoryStartOrientation.x());
        std::vector<double> Pitch(splineTime.size(), _fixedTrajectoryStartOrientation.y());
        std::vector<double> Yaw(splineTime.size(), _fixedTrajectoryStartOrientation.z());

        _splines.x.setPoints(splineTime, X);
        _splines.y.setPoints(splineTime, Y);
        _splines.z.setPoints(splineTime, Z);

        _splines.roll.setPoints(splineTime, Roll);
        _splines.pitch.setPoints(splineTime, Pitch);
        _splines.yaw.setPoints(splineTime, Yaw);
    }
    else if (_trajectoryType == TrajectoryType::Linear)
    {
        double roll = 0.0;
        double pitch = _n_linearTrajectoryStartVelocity.head<2>().norm() > 1e-8 ? calcPitchFromVelocity(_n_linearTrajectoryStartVelocity) : 0;
        double yaw = _n_linearTrajectoryStartVelocity.head<2>().norm() > 1e-8 ? calcYawFromVelocity(_n_linearTrajectoryStartVelocity) : 0;
        const Eigen::Vector3d& e_startPosition = _startPosition.e_position;

        size_t nOverhead = static_cast<size_t>(std::round(1.0 / _splines.sampleInterval)) + 1;

        splineTime = std::vector<double>(nOverhead, 0.0);
        std::vector<double> splineX(nOverhead, e_startPosition[0]);
        std::vector<double> splineY(nOverhead, e_startPosition[1]);
        std::vector<double> splineZ(nOverhead, e_startPosition[2]);
        std::vector<double> splineRoll(nOverhead, roll);
        std::vector<double> splinePitch(nOverhead, pitch);
        std::vector<double> splineYaw(nOverhead, yaw);

        // @brief Calculates the derivative of the curvilinear position and velocity
        // @param[in] y [𝜙, λ, h, v_N, v_E, v_D]^T Latitude, longitude, altitude, velocity NED in [rad, rad, m, m/s, m/s, m/s]
        // @param[in] n_acceleration Acceleration in local-navigation frame coordinates [m/s^s]
        // @return The curvilinear position and velocity derivatives ∂/∂t [𝜙, λ, h, v_N, v_E, v_D]^T
        auto f = [](const Eigen::Vector<double, 6>& y, const Eigen::Vector3d& n_acceleration) {
            Eigen::Vector<double, 6> y_dot;
            y_dot << lla_calcTimeDerivativeForPosition(y.tail<3>(),              // Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
                                                       y(0),                     // 𝜙 Latitude in [rad]
                                                       y(2),                     // h Altitude in [m]
                                                       calcEarthRadius_N(y(0)),  // North/South (meridian) earth radius [m]
                                                       calcEarthRadius_E(y(0))), // East/West (prime vertical) earth radius [m]
                n_acceleration;

            return y_dot;
        };

        Eigen::Vector3d lla_lastPosition = _startPosition.latLonAlt();
        for (size_t i = 2; i <= nOverhead; i++) // Calculate one second backwards
        {
            Eigen::Vector<double, 6> y; // [𝜙, λ, h, v_N, v_E, v_D]^T
            y << lla_lastPosition,
                _n_linearTrajectoryStartVelocity;

            y = RungeKutta1(f, -_splines.sampleInterval, y, Eigen::Vector3d::Zero());
            lla_lastPosition = y.head<3>();

            Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_lastPosition);

            splineTime.at(nOverhead - i) = -_splines.sampleInterval * static_cast<double>(i - 1);
            splineX.at(nOverhead - i) = e_position(0);
            splineY.at(nOverhead - i) = e_position(1);
            splineZ.at(nOverhead - i) = e_position(2);
        }

        lla_lastPosition = _startPosition.latLonAlt();
        for (size_t i = 1;; i++)
        {
            Eigen::Vector<double, 6> y; // [𝜙, λ, h, v_N, v_E, v_D]^T
            y << lla_lastPosition,
                _n_linearTrajectoryStartVelocity;

            y = RungeKutta1(f, _splines.sampleInterval, y, Eigen::Vector3d::Zero());
            lla_lastPosition = y.head<3>();

            Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_lastPosition);

            double simTime = _splines.sampleInterval * static_cast<double>(i);
            splineTime.push_back(simTime);
            splineX.push_back(e_position(0));
            splineY.push_back(e_position(1));
            splineZ.push_back(e_position(2));
            splineRoll.push_back(roll);
            splinePitch.push_back(pitch);
            splineYaw.push_back(yaw);

            if (_simulationStopCondition == StopCondition::Duration
                && simTime > _simulationDuration + 1.0)
            {
                break;
            }
            if (_simulationStopCondition == StopCondition::DistanceOrCirclesOrRoses)
            {
                auto horizontalDistance = calcGeographicalDistance(_startPosition.latitude(), _startPosition.longitude(), lla_lastPosition(0), lla_lastPosition(1));
                auto distance = std::sqrt(std::pow(horizontalDistance, 2) + std::pow(_startPosition.altitude() - lla_lastPosition(2), 2))
                                - _n_linearTrajectoryStartVelocity.norm() * 1.0;
                if (distance > _linearTrajectoryDistanceForStop)
                {
                    break;
                }
            }
        }

        _splines.x.setPoints(splineTime, splineX);
        _splines.y.setPoints(splineTime, splineY);
        _splines.z.setPoints(splineTime, splineZ);

        _splines.roll.setPoints(splineTime, splineRoll);
        _splines.pitch.setPoints(splineTime, splinePitch);
        _splines.yaw.setPoints(splineTime, splineYaw);
    }
    else if (_trajectoryType == TrajectoryType::Circular)
    {
        double simDuration{};
        if (_simulationStopCondition == StopCondition::Duration)
        {
            simDuration = _simulationDuration;
        }
        else if (_simulationStopCondition == StopCondition::DistanceOrCirclesOrRoses)
        {
            double omega = _trajectoryHorizontalSpeed / _trajectoryRadius;
            simDuration = _circularTrajectoryCircleCountForStop * 2 * M_PI / omega;
        }

        for (size_t i = 0; i <= static_cast<size_t>(std::round((simDuration + 2.0) / _splines.sampleInterval)); i++)
        {
            splineTime.push_back(_splines.sampleInterval * static_cast<double>(i) - 1.0);
        }

        std::vector<double> splineX(splineTime.size());
        std::vector<double> splineY(splineTime.size());
        std::vector<double> splineZ(splineTime.size());

        const Eigen::Vector3d& e_origin = _startPosition.e_position;
        Eigen::Vector3d lla_origin = _startPosition.latLonAlt();

        Eigen::Quaterniond e_quatCenter_n = trafo::e_Quat_n(lla_origin(0), lla_origin(1));

        for (uint64_t i = 0; i < splineTime.size(); i++)
        {
            auto phi = _trajectoryHorizontalSpeed * splineTime[i] / _trajectoryRadius; // Angle of the current point on the circle
            phi *= _trajectoryDirection == Direction::CW ? -1 : 1;
            phi += _trajectoryRotationAngle;

            Eigen::Vector3d n_relativePosition{ _trajectoryRadius * std::sin(phi) * (1 + _circularHarmonicAmplitudeFactor * sin(phi * static_cast<double>(_circularHarmonicFrequency))), // [m]
                                                _trajectoryRadius * std::cos(phi) * (1 + _circularHarmonicAmplitudeFactor * sin(phi * static_cast<double>(_circularHarmonicFrequency))), // [m]
                                                -_trajectoryVerticalSpeed * splineTime[i] };                                                                                             // [m]

            Eigen::Vector3d e_relativePosition = e_quatCenter_n * n_relativePosition;

            Eigen::Vector3d e_position = e_origin + e_relativePosition;

            splineX[i] = e_position[0];
            splineY[i] = e_position[1];
            splineZ[i] = e_position[2];
        }

        _splines.x.setPoints(splineTime, splineX);
        _splines.y.setPoints(splineTime, splineY);
        _splines.z.setPoints(splineTime, splineZ);
    }
    else if (_trajectoryType == TrajectoryType::RoseFigure)
    {
        // Formulas and notation for the rose figure from https://en.wikipedia.org/wiki/Rose_(mathematics)

        // Adjusting the integration bounds needs a factor * PI
        // | d  \ n | 1   | 2   | 3   | 4   | 5   | 6   | 7   |
        // | ------ | --- | --- | --- | --- | --- | --- | --- |
        // | 1      | 1   | 2   | 1   | 2   | 1   | 2   | 1   |
        // | 2      | 4   | 1   | 4   | 2   | 4   | 1   | 4   |
        // | 3      | 3   | 6   | 1   | 6   | 3   | 2   | 3   |
        // | 4      | 8   | 4   | 8   | 1   | 8   | 4   | 8   |
        // | 5      | 5   | 10  | 5   | 10  | 1   | 10  | 5   |
        // | 6      | 12  | 3   | 4   | 6   | 12  | 1   | 12  |
        // | 7      | 7   | 14  | 7   | 14  | 7   | 14  | 1   |
        // | 8      | 16  | 8   | 16  | 4   | 16  | 8   | 16  |
        // | 9      | 9   | 18  | 3   | 18  | 9   | 6   | 9   |

        int n = _rosePetNum;
        int d = _rosePetDenom;

        for (int i = 2; i <= n; i++) // reduction of fraction ( 4/2 ==> 2/1 )
        {
            if (n % i == 0 && d % i == 0)
            {
                n /= i;
                d /= i;
                i--;
            }
        }

        auto isOdd = [](auto a) { return static_cast<int>(a) % 2 != 0; };
        auto isEven = [](auto a) { return static_cast<int>(a) % 2 == 0; };

        double integrationFactor = 0.0;
        if (isOdd(d))
        {
            if (isOdd(n)) { integrationFactor = static_cast<double>(d); }
            else { integrationFactor = 2.0 * static_cast<double>(d); }
        }
        else // if (isEven(d))
        {
            if (isEven(n)) { integrationFactor = static_cast<double>(d); }
            else { integrationFactor = 2.0 * static_cast<double>(d); }
        }

        constexpr size_t nVirtPoints = 10;
        splineTime.resize(nVirtPoints); // Preallocate points to make the spline start at the right point
        std::vector<double> splineX(splineTime.size());
        std::vector<double> splineY(splineTime.size());
        std::vector<double> splineZ(splineTime.size());

        Eigen::Vector3d e_origin = trafo::lla2ecef_WGS84(_startPosition.latLonAlt());

        Eigen::Quaterniond e_quatCenter_n = trafo::e_Quat_n(_startPosition.latLonAlt()(0), _startPosition.latLonAlt()(1));

        double lengthOld = -_roseStepLengthMax / 2.0;            // n-1 length
        double dPhi = 0.001;                                     // Angle step size
        double maxPhi = std::numeric_limits<double>::infinity(); // Interval for integration depending on selected stop criteria
        if (_simulationStopCondition == StopCondition::DistanceOrCirclesOrRoses)
        {
            maxPhi = _roseTrajectoryCountForStop * integrationFactor * M_PI;
        }

        // k = n/d
        double roseK = static_cast<double>(_rosePetNum) / static_cast<double>(_rosePetDenom);

        _roseSimDuration = 0.0;

        // We cannot input negative values or zero
        for (double phi = dPhi; phi <= maxPhi + 10 * dPhi; phi += dPhi) // NOLINT(clang-analyzer-security.FloatLoopCounter, cert-flp30-c)
        {
            double length = _trajectoryRadius / roseK * math::calcEllipticalIntegral(roseK * phi, 1.0 - std::pow(roseK, 2.0));
            double dL = length - lengthOld;

            if (dL > _roseStepLengthMax)
            {
                phi -= dPhi;
                dPhi /= 2.0;
                continue;
            }
            if (dL < _roseStepLengthMax / 3.0) // Avoid also too small steps
            {
                phi -= dPhi;
                dPhi *= 1.5;
                continue;
            }
            lengthOld = length;

            double time = length / _trajectoryHorizontalSpeed;
            splineTime.push_back(time);
            Eigen::Vector3d n_relativePosition{ _trajectoryRadius * std::cos(roseK * phi) * std::sin(phi * (_trajectoryDirection == Direction::CW ? -1.0 : 1.0) + _trajectoryRotationAngle), // [m]
                                                _trajectoryRadius * std::cos(roseK * phi) * std::cos(phi * (_trajectoryDirection == Direction::CW ? -1.0 : 1.0) + _trajectoryRotationAngle), // [m]
                                                -_trajectoryVerticalSpeed * time };                                                                                                          // [m]

            LOG_DATA("{}: t={:8.6}s | l={:8.6}m", nameId(), time, length);

            Eigen::Vector3d e_relativePosition = e_quatCenter_n * n_relativePosition;
            Eigen::Vector3d e_position = e_origin + e_relativePosition;

            splineX.push_back(e_position[0]);
            splineY.push_back(e_position[1]);
            splineZ.push_back(e_position[2]);

            if (_simulationStopCondition == StopCondition::DistanceOrCirclesOrRoses && std::abs(maxPhi - phi) < 1.0 * dPhi)
            {
                LOG_TRACE("{}: Rose figure simulation duration: {:8.6}s | l={:8.6}m", nameId(), time, length);
                _roseSimDuration = time;
            }
            else if (_simulationStopCondition == StopCondition::Duration && _roseSimDuration == 0.0 && time > _simulationDuration)
            {
                _roseSimDuration = _simulationDuration;
                maxPhi = phi;
            }
        }

        maxPhi = integrationFactor * M_PI;
        double endLength = _trajectoryRadius / roseK * math::calcEllipticalIntegral(roseK * maxPhi, 1.0 - std::pow(roseK, 2.0));
        for (size_t i = 0; i < nVirtPoints; i++)
        {
            double phi = maxPhi - static_cast<double>(i) * dPhi;
            double length = _trajectoryRadius / roseK * math::calcEllipticalIntegral(roseK * phi, 1.0 - std::pow(roseK, 2.0));
            double time = (length - endLength) / _trajectoryHorizontalSpeed;
            splineTime[nVirtPoints - i - 1] = time;

            Eigen::Vector3d n_relativePosition{ _trajectoryRadius * std::cos(roseK * phi) * std::sin(phi * (_trajectoryDirection == Direction::CW ? -1.0 : 1.0) + _trajectoryRotationAngle), // [m]
                                                _trajectoryRadius * std::cos(roseK * phi) * std::cos(phi * (_trajectoryDirection == Direction::CW ? -1.0 : 1.0) + _trajectoryRotationAngle), // [m]
                                                -_trajectoryVerticalSpeed * time };                                                                                                          // [m]

            LOG_DATA("{}: t={:8.6}s | l={:8.6}m", nameId(), time, length);

            Eigen::Vector3d e_relativePosition = e_quatCenter_n * n_relativePosition;
            Eigen::Vector3d e_position = e_origin + e_relativePosition;

            splineX[nVirtPoints - i - 1] = e_position[0];
            splineY[nVirtPoints - i - 1] = e_position[1];
            splineZ[nVirtPoints - i - 1] = e_position[2];
        }

        _splines.x.setPoints(splineTime, splineX);
        _splines.y.setPoints(splineTime, splineY);
        _splines.z.setPoints(splineTime, splineZ);
    }
    else if (_trajectoryType == TrajectoryType::Csv)
    {
        auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_CSV);
        if (mutex) { mutex->lock(); }
        if (const auto* csvData = getInputValue<const CsvData>(INPUT_PORT_INDEX_CSV);
            csvData && csvData->lines.size() >= 2)
        {
            _startTime = getTimeFromCsvLine(csvData->lines.front(), csvData->description);
            if (_startTime.empty()) { return false; }

            constexpr size_t nVirtPoints = 10;
            splineTime.resize(nVirtPoints); // Preallocate points to make the spline start at the right point
            std::vector<double> splineX(splineTime.size());
            std::vector<double> splineY(splineTime.size());
            std::vector<double> splineZ(splineTime.size());
            std::vector<double> splineRoll(splineTime.size());
            std::vector<double> splinePitch(splineTime.size());
            std::vector<double> splineYaw(splineTime.size());

            for (size_t i = 0; i < csvData->lines.size(); i++)
            {
                InsTime insTime = getTimeFromCsvLine(csvData->lines[i], csvData->description);
                if (insTime.empty()) { return false; }
                LOG_DATA("{}: Time {}", nameId(), insTime);
                double time = static_cast<double>((insTime - _startTime).count());

                Eigen::Vector3d e_pos = e_getPositionFromCsvLine(csvData->lines[i], csvData->description);
                if (std::isnan(e_pos.x())) { return false; }
                LOG_DATA("{}: e_pos {}", nameId(), e_pos);

                Eigen::Quaterniond n_Quat_b = n_getAttitudeQuaternionFromCsvLine_b(csvData->lines[i], csvData->description);
                if (std::isnan(n_Quat_b.w()))
                {
                    // TODO: Calculate with rotation minimizing frame instead of returning false
                    return false;
                }
                LOG_DATA("{}: n_Quat_b {}", nameId(), n_Quat_b);

                splineTime.push_back(time);
                splineX.push_back(e_pos.x());
                splineY.push_back(e_pos.y());
                splineZ.push_back(e_pos.z());

                auto rpy = trafo::quat2eulerZYX(n_Quat_b);
                LOG_DATA("{}: RPY {} [deg] (from CSV)", nameId(), rad2deg(rpy).transpose());
                splineRoll.push_back(i > 0 ? unwrapAngle(rpy(0), splineRoll.back(), M_PI) : rpy(0));
                splinePitch.push_back(i > 0 ? unwrapAngle(rpy(1), splinePitch.back(), M_PI_2) : rpy(1));
                splineYaw.push_back(i > 0 ? unwrapAngle(rpy(2), splineYaw.back(), M_PI) : rpy(2));
                LOG_DATA("{}: R {}, P {}, Y {} [deg] (in Spline)", nameId(), rad2deg(splineRoll.back()), rad2deg(splinePitch.back()), rad2deg(splineYaw.back()));
            }
            if (mutex) { mutex->unlock(); }
            _csvDuration = splineTime.back();

            double dt = splineTime[nVirtPoints + 1] - splineTime[nVirtPoints];
            for (size_t i = 0; i < nVirtPoints; i++)
            {
                double h = 0.001;
                splineTime[nVirtPoints - i - 1] = splineTime[nVirtPoints - i] - h;
                splineX[nVirtPoints - i - 1] = splineX[nVirtPoints - i] - h * (splineX[nVirtPoints + 1] - splineX[nVirtPoints]) / dt;
                splineY[nVirtPoints - i - 1] = splineY[nVirtPoints - i] - h * (splineY[nVirtPoints + 1] - splineY[nVirtPoints]) / dt;
                splineZ[nVirtPoints - i - 1] = splineZ[nVirtPoints - i] - h * (splineZ[nVirtPoints + 1] - splineZ[nVirtPoints]) / dt;
                splineRoll[nVirtPoints - i - 1] = splineRoll[nVirtPoints - i] - h * (splineRoll[nVirtPoints + 1] - splineRoll[nVirtPoints]) / dt;
                splinePitch[nVirtPoints - i - 1] = splinePitch[nVirtPoints - i] - h * (splinePitch[nVirtPoints + 1] - splinePitch[nVirtPoints]) / dt;
                splineYaw[nVirtPoints - i - 1] = splineYaw[nVirtPoints - i] - h * (splineYaw[nVirtPoints + 1] - splineYaw[nVirtPoints]) / dt;
                splineTime.push_back(splineTime[splineX.size() - 1] + h);
                splineX.push_back(splineX[splineX.size() - 1] + h * (splineX[splineX.size() - 1] - splineX[splineX.size() - 2]) / dt);
                splineY.push_back(splineY[splineY.size() - 1] + h * (splineY[splineY.size() - 1] - splineY[splineY.size() - 2]) / dt);
                splineZ.push_back(splineZ[splineZ.size() - 1] + h * (splineZ[splineZ.size() - 1] - splineZ[splineZ.size() - 2]) / dt);
                splineRoll.push_back(splineRoll[splineRoll.size() - 1] + h * (splineRoll[splineRoll.size() - 1] - splineRoll[splineRoll.size() - 2]) / dt);
                splinePitch.push_back(splinePitch[splinePitch.size() - 1] + h * (splinePitch[splinePitch.size() - 1] - splinePitch[splinePitch.size() - 2]) / dt);
                splineYaw.push_back(splineYaw[splineYaw.size() - 1] + h * (splineYaw[splineYaw.size() - 1] - splineYaw[splineYaw.size() - 2]) / dt);
            }

            _splines.x.setPoints(splineTime, splineX);
            _splines.y.setPoints(splineTime, splineY);
            _splines.z.setPoints(splineTime, splineZ);

            _splines.roll.setPoints(splineTime, splineRoll);
            _splines.pitch.setPoints(splineTime, splinePitch);
            _splines.yaw.setPoints(splineTime, splineYaw);
        }
        else
        {
            if (mutex) { mutex->unlock(); }
            LOG_ERROR("{}: Can't calculate the data without a connected CSV file with at least two datasets", nameId());
            return false;
        }
    }

    if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::RoseFigure)
    {
        std::vector<double> splineRoll(splineTime.size());
        std::vector<double> splinePitch(splineTime.size());
        std::vector<double> splineYaw(splineTime.size());

        for (uint64_t i = 0; i < splineTime.size(); i++)
        {
            Eigen::Vector3d e_pos{ _splines.x(splineTime[i]),
                                   _splines.y(splineTime[i]),
                                   _splines.z(splineTime[i]) };
            Eigen::Vector3d e_vel{ _splines.x.derivative(1, splineTime[i]),
                                   _splines.y.derivative(1, splineTime[i]),
                                   _splines.z.derivative(1, splineTime[i]) };

            Eigen::Vector3d lla_position = trafo::ecef2lla_WGS84(e_pos);
            Eigen::Vector3d n_velocity = trafo::n_Quat_e(lla_position(0), lla_position(1)) * e_vel;

            Eigen::Vector3d e_normalVectorCenterCircle{ std::cos(_startPosition.latLonAlt()(0)) * std::cos(_startPosition.latLonAlt()(1)),
                                                        std::cos(_startPosition.latLonAlt()(0)) * std::sin(_startPosition.latLonAlt()(1)),
                                                        std::sin(_startPosition.latLonAlt()(0)) };

            Eigen::Vector3d e_normalVectorCurrentPosition{ std::cos(lla_position(0)) * std::cos(lla_position(1)),
                                                           std::cos(lla_position(0)) * std::sin(lla_position(1)),
                                                           std::sin(lla_position(0)) };

            double yaw = calcYawFromVelocity(n_velocity);

            splineYaw[i] = i > 0 ? unwrapAngle(yaw, splineYaw[i - 1], M_PI) : yaw;
            splineRoll[i] = 0.0;
            splinePitch[i] = n_velocity.head<2>().norm() > 1e-8 ? calcPitchFromVelocity(n_velocity) : 0;
        }

        _splines.roll.setPoints(splineTime, splineRoll);
        _splines.pitch.setPoints(splineTime, splinePitch);
        _splines.yaw.setPoints(splineTime, splineYaw);
    }

    return true;
}

void NAV::ImuSimulator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::ImuSimulator::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    _imuUpdateCnt = 0;
    _gnssUpdateCnt = 0;

    _imuLastUpdateTime = 0.0;
    _gnssLastUpdateTime = 0.0;
    _lla_imuLastLinearPosition = _startPosition.latLonAlt();
    _lla_gnssLastLinearPosition = _startPosition.latLonAlt();

    if (_trajectoryType == TrajectoryType::Csv)
    {
        auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_CSV);
        if (mutex) { mutex->lock(); }
        if (const auto* csvData = getInputValue<const CsvData>(INPUT_PORT_INDEX_CSV);
            csvData && !csvData->lines.empty())
        {
            _startTime = getTimeFromCsvLine(csvData->lines.front(), csvData->description);
            if (mutex) { mutex->unlock(); }
            if (_startTime.empty())
            {
                return false;
            }
            LOG_DEBUG("{}: Start Time set to {}", nameId(), _startTime);
        }
        else
        {
            if (mutex) { mutex->unlock(); }
            LOG_ERROR("{}: Can't reset the ImuSimulator without a connected CSV file", nameId());
            return false;
        }
    }
    else if (_startTimeSource == StartTimeSource::CurrentComputerTime)
    {
        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t); // NOLINT(concurrency-mt-unsafe)

        _startTime = InsTime{ static_cast<uint16_t>(now->tm_year + 1900), static_cast<uint16_t>(now->tm_mon), static_cast<uint16_t>(now->tm_mday),
                              static_cast<uint16_t>(now->tm_hour), static_cast<uint16_t>(now->tm_min), static_cast<long double>(now->tm_sec) };
        LOG_DEBUG("{}: Start Time set to {}", nameId(), _startTime);
    }

    return true;
}

bool NAV::ImuSimulator::checkStopCondition(double time, const Eigen::Vector3d& lla_position)
{
    if (_trajectoryType == TrajectoryType::Csv)
    {
        return time > _csvDuration;
    }
    if (_simulationStopCondition == StopCondition::Duration
        || _trajectoryType == TrajectoryType::Fixed)
    {
        return time > _simulationDuration;
    }
    if (_simulationStopCondition == StopCondition::DistanceOrCirclesOrRoses)
    {
        if (_trajectoryType == TrajectoryType::Linear)
        {
            auto horizontalDistance = calcGeographicalDistance(_startPosition.latitude(), _startPosition.longitude(), lla_position(0), lla_position(1));
            auto distance = std::sqrt(std::pow(horizontalDistance, 2) + std::pow(_startPosition.altitude() - lla_position(2), 2));
            return distance > _linearTrajectoryDistanceForStop;
        }
        if (_trajectoryType == TrajectoryType::Circular)
        {
            double omega = _trajectoryHorizontalSpeed / _trajectoryRadius;
            double simDuration = _circularTrajectoryCircleCountForStop * 2 * M_PI / omega;
            return time > simDuration;
        }
        if (_trajectoryType == TrajectoryType::RoseFigure)
        {
            return time > _roseSimDuration;
        }
    }
    return false;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollImuObs(size_t /* pinIdx */, bool peek)
{
    double imuUpdateTime = static_cast<double>(_imuUpdateCnt) / _imuFrequency;

    Eigen::Vector3d lla_position = lla_calcPosition(imuUpdateTime);

    // -------------------------------------------------- Check if a stop condition is met -----------------------------------------------------
    if (checkStopCondition(imuUpdateTime, lla_position))
    {
        return nullptr;
    }

    // ------------------------------------- Early return in case of peeking to avoid heavy calculations ---------------------------------------
    if (peek)
    {
        auto obs = std::make_shared<NodeData>();
        obs->insTime = _startTime + std::chrono::duration<double>(imuUpdateTime);
        return obs;
    }

    auto obs = std::make_shared<ImuObsSimulated>(_imuPos);
    obs->timeSinceStartup = static_cast<uint64_t>(imuUpdateTime * 1e9);
    obs->insTime = _startTime + std::chrono::duration<double>(imuUpdateTime);
    LOG_DATA("{}: Simulating IMU data for time [{}]", nameId(), obs->insTime.toYMDHMS());

    // --------------------------------------------------------- Calculation of data -----------------------------------------------------------
    LOG_DATA("{}: [{:8.3f}] lla_position = {}°, {}°, {} m", nameId(), imuUpdateTime, rad2deg(lla_position(0)), rad2deg(lla_position(1)), lla_position(2));
    Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_position);
    auto n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));

    Eigen::Vector3d n_vel = n_calcVelocity(imuUpdateTime, n_Quat_e);
    LOG_DATA("{}: [{:8.3f}] n_vel = {} [m/s]", nameId(), imuUpdateTime, n_vel.transpose());

    auto [roll, pitch, yaw] = calcFlightAngles(imuUpdateTime);
    LOG_DATA("{}: [{:8.3f}] roll = {}°, pitch = {}°, yaw = {}°", nameId(), imuUpdateTime, rad2deg(roll), rad2deg(pitch), rad2deg(yaw));

    auto b_Quat_n = trafo::b_Quat_n(roll, pitch, yaw);

    Eigen::Vector3d n_omega_ie = n_Quat_e * InsConst::e_omega_ie;
    LOG_DATA("{}: [{:8.3f}] n_omega_ie = {} [rad/s]", nameId(), imuUpdateTime, n_omega_ie.transpose());
    double R_N = calcEarthRadius_N(lla_position(0));
    LOG_DATA("{}: [{:8.3f}] R_N = {} [m]", nameId(), imuUpdateTime, R_N);
    double R_E = calcEarthRadius_E(lla_position(0));
    LOG_DATA("{}: [{:8.3f}] R_E = {} [m]", nameId(), imuUpdateTime, R_E);
    Eigen::Vector3d n_omega_en = n_calcTransportRate(lla_position, n_vel, R_N, R_E);
    LOG_DATA("{}: [{:8.3f}] n_omega_en = {} [rad/s]", nameId(), imuUpdateTime, n_omega_en.transpose());

    // ------------------------------------------------------------ Accelerations --------------------------------------------------------------

    // Force to keep vehicle on track
    Eigen::Vector3d n_trajectoryAccel = n_calcTrajectoryAccel(imuUpdateTime, n_Quat_e, lla_position, n_vel);
    LOG_DATA("{}: [{:8.3f}] n_trajectoryAccel = {} [m/s^2]", nameId(), imuUpdateTime, n_trajectoryAccel.transpose());

    // Measured acceleration in local-navigation frame coordinates [m/s^2]
    Eigen::Vector3d n_accel = n_trajectoryAccel;
    if (_coriolisAccelerationEnabled) // Apply Coriolis Acceleration
    {
        Eigen::Vector3d n_coriolisAcceleration = n_calcCoriolisAcceleration(n_omega_ie, n_omega_en, n_vel);
        LOG_DATA("{}: [{:8.3f}] n_coriolisAcceleration = {} [m/s^2]", nameId(), imuUpdateTime, n_coriolisAcceleration.transpose());
        n_accel += n_coriolisAcceleration;
    }

    // Mass attraction of the Earth (gravitation)
    Eigen::Vector3d n_gravitation = n_calcGravitation(lla_position, _gravitationModel);
    LOG_DATA("{}: [{:8.3f}] n_gravitation = {} [m/s^2] ({})", nameId(), imuUpdateTime, n_gravitation.transpose(), NAV::to_string(_gravitationModel));
    n_accel -= n_gravitation; // Apply the local gravity vector

    if (_centrifgalAccelerationEnabled) // Centrifugal acceleration caused by the Earth's rotation
    {
        Eigen::Vector3d e_centrifugalAcceleration = e_calcCentrifugalAcceleration(e_position);
        LOG_DATA("{}: [{:8.3f}] e_centrifugalAcceleration = {} [m/s^2]", nameId(), imuUpdateTime, e_centrifugalAcceleration.transpose());
        Eigen::Vector3d n_centrifugalAcceleration = n_Quat_e * e_centrifugalAcceleration;
        LOG_DATA("{}: [{:8.3f}] n_centrifugalAcceleration = {} [m/s^2]", nameId(), imuUpdateTime, n_centrifugalAcceleration.transpose());
        n_accel += n_centrifugalAcceleration;
    }

    // Acceleration measured by the accelerometer in platform coordinates
    Eigen::Vector3d accel_p = _imuPos.p_quatAccel_b() * b_Quat_n * n_accel;
    LOG_DATA("{}: [{:8.3f}] accel_p = {} [m/s^2]", nameId(), imuUpdateTime, accel_p.transpose());

    // ------------------------------------------------------------ Angular rates --------------------------------------------------------------

    Eigen::Vector3d n_omega_ip = n_calcOmega_ip(imuUpdateTime, Eigen::Vector3d{ roll, pitch, yaw }, b_Quat_n.conjugate(), n_omega_ie, n_omega_en);

    // ω_ib_b = b_Quat_n * ω_ib_n
    //                            = 0
    // ω_ip_p = p_Quat_b * (ω_ib_b + ω_bp_b) = p_Quat_b * ω_ib_b
    Eigen::Vector3d p_omega_ip = _imuPos.p_quatGyro_b() * b_Quat_n * n_omega_ip;
    LOG_DATA("{}: [{:8.3f}] p_omega_ip = {} [rad/s]", nameId(), imuUpdateTime, p_omega_ip.transpose());

    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------

    obs->accelCompXYZ = accel_p;
    obs->accelUncompXYZ = accel_p;
    obs->gyroCompXYZ = p_omega_ip;
    obs->gyroUncompXYZ = p_omega_ip;
    // obs->magCompXYZ.emplace(0, 0, 0);
    // obs->magUncompXYZ.emplace(0, 0, 0);

    auto e_Quat_n = n_Quat_e.conjugate();

    obs->n_accelUncomp = n_accel;
    obs->n_gyroUncomp = n_omega_ip;
    obs->n_magUncomp.setZero();

    obs->e_accelUncomp = e_Quat_n * n_accel;
    obs->e_gyroUncomp = e_Quat_n * n_omega_ip;
    obs->e_magUncomp.setZero();

    _imuUpdateCnt++;
    invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);

    return obs;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollPosVelAtt(size_t /* pinIdx */, bool peek)
{
    double gnssUpdateTime = static_cast<double>(_gnssUpdateCnt) / _gnssFrequency;

    Eigen::Vector3d lla_position = lla_calcPosition(gnssUpdateTime);

    // -------------------------------------------------- Check if a stop condition is met -----------------------------------------------------
    if (checkStopCondition(gnssUpdateTime, lla_position))
    {
        return nullptr;
    }

    // ------------------------------------- Early return in case of peeking to avoid heavy calculations ---------------------------------------
    if (peek)
    {
        auto obs = std::make_shared<NodeData>();
        obs->insTime = _startTime + std::chrono::duration<double>(gnssUpdateTime);
        return obs;
    }
    auto obs = std::make_shared<PosVelAtt>();
    obs->insTime = _startTime + std::chrono::duration<double>(gnssUpdateTime);
    LOG_DATA("{}: Simulating GNSS data for time [{}]", nameId(), obs->insTime.toYMDHMS());

    // --------------------------------------------------------- Calculation of data -----------------------------------------------------------
    LOG_DATA("{}: [{:8.3f}] lla_position = {}°, {}°, {} m", nameId(), gnssUpdateTime, rad2deg(lla_position(0)), rad2deg(lla_position(1)), lla_position(2));
    auto n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));
    Eigen::Vector3d n_vel = n_calcVelocity(gnssUpdateTime, n_Quat_e);
    LOG_DATA("{}: [{:8.3f}] n_vel = {} [m/s]", nameId(), gnssUpdateTime, n_vel.transpose());
    auto [roll, pitch, yaw] = calcFlightAngles(gnssUpdateTime);
    LOG_DATA("{}: [{:8.3f}] roll = {}°, pitch = {}°, yaw = {}°", nameId(), gnssUpdateTime, rad2deg(roll), rad2deg(pitch), rad2deg(yaw));

    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------

    obs->setState_n(lla_position, n_vel, trafo::n_Quat_b(roll, pitch, yaw));

    _gnssUpdateCnt++;
    invokeCallbacks(OUTPUT_PORT_INDEX_POS_VEL_ATT, obs);

    return obs;
}

std::array<double, 3> NAV::ImuSimulator::calcFlightAngles(double time) const
{
    double roll = _splines.roll(time);
    double pitch = _splines.pitch(time);
    double yaw = _splines.yaw(time);
    return { roll, pitch, yaw };
}

Eigen::Vector3d NAV::ImuSimulator::lla_calcPosition(double time) const
{
    Eigen::Vector3d e_pos(_splines.x(time), _splines.y(time), _splines.z(time));
    return trafo::ecef2lla_WGS84(e_pos);
}

Eigen::Vector3d NAV::ImuSimulator::n_calcVelocity(double time, const Eigen::Quaterniond& n_Quat_e) const
{
    Eigen::Vector3d e_vel(_splines.x.derivative(1, time), _splines.y.derivative(1, time), _splines.z.derivative(1, time));
    return n_Quat_e * e_vel;
}

Eigen::Vector3d NAV::ImuSimulator::n_calcTrajectoryAccel(double time, const Eigen::Quaterniond& n_Quat_e, const Eigen::Vector3d& lla_position, const Eigen::Vector3d& n_velocity) const
{
    Eigen::Vector3d e_accel(_splines.x.derivative(2, time), _splines.y.derivative(2, time), _splines.z.derivative(2, time));
    Eigen::Quaterniond e_Quat_n = n_Quat_e.conjugate();
    Eigen::Vector3d e_vel = e_Quat_n * n_velocity;

    // Math: \dot{C}_n^e = C_n^e \cdot \Omega_{en}^n
    Eigen::Matrix3d n_DCM_dot_e = e_Quat_n.toRotationMatrix()
                                  * math::skewSymmetricMatrix(n_calcTransportRate(lla_position, n_velocity,
                                                                                  calcEarthRadius_N(lla_position(0)),
                                                                                  calcEarthRadius_E(lla_position(0))));

    // Math: \dot{C}_e^n = (\dot{C}_n^e)^T
    Eigen::Matrix3d e_DCM_dot_n = n_DCM_dot_e.transpose();

    // Math: a^n = \frac{\partial}{\partial t} \left( \dot{x}^n \right) = \frac{\partial}{\partial t} \left( C_e^n \cdot \dot{x}^e \right) = \dot{C}_e^n \cdot \dot{x}^e + C_e^n \cdot \ddot{x}^e
    return e_DCM_dot_n * e_vel + n_Quat_e * e_accel;
}

Eigen::Vector3d NAV::ImuSimulator::n_calcOmega_ip(double time,
                                                  const Eigen::Vector3d& rollPitchYaw,
                                                  const Eigen::Quaterniond& n_Quat_b,
                                                  const Eigen::Vector3d& n_omega_ie,
                                                  const Eigen::Vector3d& n_omega_en) const
{
    const auto& R = rollPitchYaw(0);
    const auto& P = rollPitchYaw(1);

    // #########################################################################################################################################

    double R_dot = _splines.roll.derivative(1, time);
    double Y_dot = _splines.yaw.derivative(1, time);
    double P_dot = _splines.pitch.derivative(1, time);

    auto C_3 = [](double R) {
        // Eigen::Matrix3d C;
        // C << 1,       0     ,      0     ,
        //      0,  std::cos(R), std::sin(R),
        //      0, -std::sin(R), std::cos(R);
        // return C;
        return Eigen::AngleAxisd{ -R, Eigen::Vector3d::UnitX() };
    };
    auto C_2 = [](double P) {
        // Eigen::Matrix3d C;
        // C << std::cos(P), 0 , -std::sin(P),
        //           0     , 1 ,       0     ,
        //      std::sin(P), 0 ,  std::cos(P);
        // return C;
        return Eigen::AngleAxisd{ -P, Eigen::Vector3d::UnitY() };
    };

    // ω_nb_b = [∂/∂t R] + C_3 [   0  ] + C_3 C_2 [   0  ]
    //          [   0  ]       [∂/∂t P]           [   0  ]
    //          [   0  ]       [   0  ]           [∂/∂t Y]
    Eigen::Vector3d b_omega_nb = Eigen::Vector3d{ R_dot, 0, 0 }
                                 + C_3(R) * Eigen::Vector3d{ 0, P_dot, 0 }
                                 + C_3(R) * C_2(P) * Eigen::Vector3d{ 0, 0, Y_dot };

    //  ω_ib_n = ω_in_n + ω_nb_n = (ω_ie_n + ω_en_n) + n_Quat_b * ω_nb_b
    Eigen::Vector3d n_omega_ib = n_Quat_b * b_omega_nb;
    if (_angularRateEarthRotationEnabled)
    {
        n_omega_ib += n_omega_ie;
    }
    if (_angularRateTransportRateEnabled)
    {
        n_omega_ib += n_omega_en;
    }

    return n_omega_ib;
}

const char* NAV::ImuSimulator::to_string(TrajectoryType value)
{
    switch (value)
    {
    case TrajectoryType::Fixed:
        return "Fixed";
    case TrajectoryType::Linear:
        return "Linear";
    case TrajectoryType::Circular:
        return "Circular";
    case TrajectoryType::Csv:
        return "CSV";
    case TrajectoryType::RoseFigure:
        return "Rose Figure";
    case TrajectoryType::COUNT:
        return "";
    }
    return "";
}

const char* NAV::ImuSimulator::to_string(Direction value)
{
    switch (value)
    {
    case Direction::CW:
        return "Clockwise (CW)";
    case Direction::CCW:
        return "Counterclockwise (CCW)";
    case Direction::COUNT:
        return "";
    }
    return "";
}