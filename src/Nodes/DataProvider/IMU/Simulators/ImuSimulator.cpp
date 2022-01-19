#include "ImuSimulator.hpp"

#include <ctime>

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/Mechanization.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"

NAV::ImuSimulator::ImuSimulator()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 677, 580 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuSimulator::pollImuObs);
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
    constexpr float columnWidth{ 130.0F };

    if (ImGui::TreeNode("Start Time"))
    {
        if (ImGui::RadioButton(fmt::format("Current Computer Time##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_startTimeSource), static_cast<int>(StartTimeSource::CurrentComputerTime)))
        {
            LOG_DEBUG("{}: startTimeSource changed to {}", nameId(), _startTimeSource);
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
            LOG_DEBUG("{}: startTimeSource changed to {}", nameId(), _startTimeSource);
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
        if (ImGui::InputDoubleL(fmt::format("IMU sample rate##{}", size_t(id)).c_str(), &_imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.3f Hz"))
        {
            LOG_DEBUG("{}: imuFrequency changed to {}", nameId(), _imuFrequency);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDouble(fmt::format("GNSS sample rate##{}", size_t(id)).c_str(), &_gnssFrequency, 0.0, 0.0, "%.3f Hz"))
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
                    LOG_DEBUG("{}: trajectoryType changed to {}", nameId(), _trajectoryType);
                    flow::ApplyChanges();
                }

                if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::SetNextItemWidth(columnWidth);
        double latitude = trafo::rad2deg(_startPosition_lla.x());
        if (ImGui::InputDoubleL(fmt::format("##Latitude{}", size_t(id)).c_str(), &latitude, -90, 90, 0.0, 0.0, "%.8f°"))
        {
            _startPosition_lla.x() = trafo::deg2rad(latitude);
            LOG_DEBUG("{}: latitude changed to {}", nameId(), latitude);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(columnWidth);
        double longitude = trafo::rad2deg(_startPosition_lla.y());
        if (ImGui::InputDoubleL(fmt::format("##Longitude{}", size_t(id)).c_str(), &longitude, -180, 180, 0.0, 0.0, "%.8f°"))
        {
            _startPosition_lla.y() = trafo::deg2rad(longitude);
            LOG_DEBUG("{}: longitude changed to {}", nameId(), longitude);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDouble(fmt::format("##Altitude (Ellipsoid){}", size_t(id)).c_str(), &_startPosition_lla.z(), 0.0, 0.0, "%.3f m"))
        {
            LOG_DEBUG("{}: altitude changed to {}", nameId(), _startPosition_lla.y());
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::TextUnformatted(_trajectoryType == TrajectoryType::Fixed
                                   ? "Position (Lat, Lon, Alt)"
                                   : (_trajectoryType == TrajectoryType::Linear
                                          ? "Start position (Lat, Lon, Alt)"
                                          : (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix
                                                 ? "Center position (Lat, Lon, Alt)"
                                                 : "")));

        if (_trajectoryType == TrajectoryType::Fixed)
        {
            ImGui::SetNextItemWidth(columnWidth);
            double roll = trafo::rad2deg(_fixedTrajectoryStartOrientation.x());
            if (ImGui::InputDoubleL(fmt::format("##Roll{}", size_t(id)).c_str(), &roll, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.x() = trafo::deg2rad(roll);
                LOG_DEBUG("{}: roll changed to {}", nameId(), roll);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double pitch = trafo::rad2deg(_fixedTrajectoryStartOrientation.y());
            if (ImGui::InputDoubleL(fmt::format("##Pitch{}", size_t(id)).c_str(), &pitch, -90, 90, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.y() = trafo::deg2rad(pitch);
                LOG_DEBUG("{}: pitch changed to {}", nameId(), pitch);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double yaw = trafo::rad2deg(_fixedTrajectoryStartOrientation.z());
            if (ImGui::InputDoubleL(fmt::format("##Yaw{}", size_t(id)).c_str(), &yaw, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.z() = trafo::deg2rad(yaw);
                LOG_DEBUG("{}: yaw changed to {}", nameId(), yaw);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("startOrientation (Roll, Pitch, Yaw)");
        }
        else if (_trajectoryType == TrajectoryType::Linear)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##North velocity{}", size_t(id)).c_str(), &_linearTrajectoryStartVelocity_n.x(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: linearTrajectoryStartVelocity_n changed to {}", nameId(), _linearTrajectoryStartVelocity_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##East velocity{}", size_t(id)).c_str(), &_linearTrajectoryStartVelocity_n.y(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: linearTrajectoryStartVelocity_n changed to {}", nameId(), _linearTrajectoryStartVelocity_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##Down velocity{}", size_t(id)).c_str(), &_linearTrajectoryStartVelocity_n.z(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: linearTrajectoryStartVelocity_n changed to {}", nameId(), _linearTrajectoryStartVelocity_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Start velocity (North, East, Down)");

            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##North acceleration{}", size_t(id)).c_str(), &_linearTrajectoryAcceleration_n.x(), 0.0, 0.0, "%.3f m/s^2"))
            {
                LOG_DEBUG("{}: linearTrajectoryAcceleration_n changed to {}", nameId(), _linearTrajectoryAcceleration_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##East acceleration{}", size_t(id)).c_str(), &_linearTrajectoryAcceleration_n.y(), 0.0, 0.0, "%.3f m/s^2"))
            {
                LOG_DEBUG("{}: linearTrajectoryAcceleration_n changed to {}", nameId(), _linearTrajectoryAcceleration_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##Down acceleration{}", size_t(id)).c_str(), &_linearTrajectoryAcceleration_n.z(), 0.0, 0.0, "%.3f m/s^2"))
            {
                LOG_DEBUG("{}: linearTrajectoryAcceleration_n changed to {}", nameId(), _linearTrajectoryAcceleration_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Acceleration (North, East, Down)");
        }
        else if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
        {
            if (ImGui::BeginTable(fmt::format("CircularTrajectory##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX))
            {
                ImGui::TableNextColumn();
                auto tableStartX = ImGui::GetCursorPosX();
                ImGui::SetNextItemWidth(200);
                if (ImGui::BeginCombo(fmt::format("Motion##{}", size_t(id)).c_str(), to_string(_circularTrajectoryDirection)))
                {
                    for (size_t i = 0; i < static_cast<size_t>(Direction::COUNT); i++)
                    {
                        const bool is_selected = (static_cast<size_t>(_circularTrajectoryDirection) == i);
                        if (ImGui::Selectable(to_string(static_cast<Direction>(i)), is_selected))
                        {
                            _circularTrajectoryDirection = static_cast<Direction>(i);
                            LOG_DEBUG("{}: circularTrajectoryDirection changed to {}", nameId(), _circularTrajectoryDirection);
                            flow::ApplyChanges();
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
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDoubleL(fmt::format("Radius##{}", size_t(id)).c_str(), &_circularTrajectoryRadius, 1e-3, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f m"))
                {
                    LOG_DEBUG("{}: circularTrajectoryRadius changed to {}", nameId(), _circularTrajectoryRadius);
                    flow::ApplyChanges();
                }
                // ####################################################################################################
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                double originAngle = trafo::rad2deg(_circularTrajectoryOriginAngle);
                if (ImGui::DragDouble(fmt::format("Origin Angle##{}", size_t(id)).c_str(), &originAngle, 15.0, -360.0, 360.0, "%.8f°"))
                {
                    _circularTrajectoryOriginAngle = trafo::deg2rad(originAngle);
                    LOG_DEBUG("{}: originAngle changed to {}", nameId(), originAngle);
                    flow::ApplyChanges();
                }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("Horizontal speed##{}", size_t(id)).c_str(), &_circularTrajectoryHorizontalSpeed, 0.0, 0.0, "%.3f m/s"))
                {
                    LOG_DEBUG("{}: circularTrajectoryHorizontalSpeed changed to {}", nameId(), _circularTrajectoryHorizontalSpeed);
                    flow::ApplyChanges();
                }
                // ####################################################################################################
                if (_trajectoryType == TrajectoryType::Helix)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(columnWidth);
                    if (ImGui::InputDouble(fmt::format("Vertical speed (Up)##{}", size_t(id)).c_str(), &_helicalTrajectoryVerticalSpeed, 0.0, 0.0, "%.3f m/s"))
                    {
                        LOG_DEBUG("{}: helicalTrajectoryVerticalSpeed changed to {}", nameId(), _helicalTrajectoryVerticalSpeed);
                        flow::ApplyChanges();
                    }
                }

                ImGui::EndTable();
            }
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode("Simulation Stop Condition"))
    {
        if (_trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDuration{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_simulationStopCondition), static_cast<int>(StopCondition::Duration)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), _simulationStopCondition);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
        }
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Duration##{}", size_t(id)).c_str(), &_simulationDuration, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f s"))
        {
            LOG_DEBUG("{}: simulationDuration changed to {}", nameId(), _simulationDuration);
            flow::ApplyChanges();
        }

        if (_trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDistanceOrCircles{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_simulationStopCondition), static_cast<int>(StopCondition::DistanceOrCircles)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), _simulationStopCondition);
                flow::ApplyChanges();
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
            }
        }
        else if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Amount of Circles##{}", size_t(id)).c_str(), &_circularTrajectoryCircleCountForStop, 0.0, std::numeric_limits<double>::max(), 1.0, 1.0, "%.3f"))
            {
                LOG_DEBUG("{}: circularTrajectoryCircleCountForStop changed to {}", nameId(), _circularTrajectoryCircleCountForStop);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Simulation models"))
    {
        ImGui::TextUnformatted("Measured acceleration");
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(230);
            if (ImGui::BeginCombo(fmt::format("Gravitation Model##{}", size_t(id)).c_str(), NAV::to_string(_gravityModel)))
            {
                for (size_t i = 0; i < static_cast<size_t>(GravityModel::COUNT); i++)
                {
                    const bool is_selected = (static_cast<size_t>(_gravityModel) == i);
                    if (ImGui::Selectable(NAV::to_string(static_cast<GravityModel>(i)), is_selected))
                    {
                        _gravityModel = static_cast<GravityModel>(i);
                        LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), NAV::to_string(_gravityModel));
                        flow::ApplyChanges();
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
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

[[nodiscard]] json NAV::ImuSimulator::save() const
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
    j["startPosition_lla"] = _startPosition_lla;
    j["fixedTrajectoryStartOrientation"] = _fixedTrajectoryStartOrientation;
    j["linearTrajectoryStartVelocity_n"] = _linearTrajectoryStartVelocity_n;
    j["linearTrajectoryAcceleration_n"] = _linearTrajectoryAcceleration_n;
    j["circularTrajectoryHorizontalSpeed"] = _circularTrajectoryHorizontalSpeed;
    j["helicalTrajectoryVerticalSpeed"] = _helicalTrajectoryVerticalSpeed;
    j["circularTrajectoryRadius"] = _circularTrajectoryRadius;
    j["circularTrajectoryOriginAngle"] = _circularTrajectoryOriginAngle;
    j["circularTrajectoryDirection"] = _circularTrajectoryDirection;
    // ###########################################################################################################
    j["simulationStopCondition"] = _simulationStopCondition;
    j["simulationDuration"] = _simulationDuration;
    j["linearTrajectoryDistanceForStop"] = _linearTrajectoryDistanceForStop;
    j["circularTrajectoryCircleCountForStop"] = _circularTrajectoryCircleCountForStop;
    // ###########################################################################################################
    j["gravityModel"] = _gravityModel;
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
    }
    if (j.contains("startPosition_lla"))
    {
        j.at("startPosition_lla").get_to(_startPosition_lla);
    }
    if (j.contains("fixedTrajectoryStartOrientation"))
    {
        j.at("fixedTrajectoryStartOrientation").get_to(_fixedTrajectoryStartOrientation);
    }
    if (j.contains("linearTrajectoryStartVelocity_n"))
    {
        j.at("linearTrajectoryStartVelocity_n").get_to(_linearTrajectoryStartVelocity_n);
    }
    if (j.contains("linearTrajectoryAcceleration_n"))
    {
        j.at("linearTrajectoryAcceleration_n").get_to(_linearTrajectoryAcceleration_n);
    }
    if (j.contains("circularTrajectoryHorizontalSpeed"))
    {
        j.at("circularTrajectoryHorizontalSpeed").get_to(_circularTrajectoryHorizontalSpeed);
    }
    if (j.contains("helicalTrajectoryVerticalSpeed"))
    {
        j.at("helicalTrajectoryVerticalSpeed").get_to(_helicalTrajectoryVerticalSpeed);
    }
    if (j.contains("circularTrajectoryRadius"))
    {
        j.at("circularTrajectoryRadius").get_to(_circularTrajectoryRadius);
    }
    if (j.contains("circularTrajectoryOriginAngle"))
    {
        j.at("circularTrajectoryOriginAngle").get_to(_circularTrajectoryOriginAngle);
    }
    if (j.contains("circularTrajectoryDirection"))
    {
        j.at("circularTrajectoryDirection").get_to(_circularTrajectoryDirection);
    }
    // ###########################################################################################################
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
    // ###########################################################################################################
    if (j.contains("gravityModel"))
    {
        j.at("gravityModel").get_to(_gravityModel);
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

    _startPosition_e = trafo::lla2ecef_WGS84(_startPosition_lla);

    return true;
}

void NAV::ImuSimulator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::ImuSimulator::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    _imuUpdateTime = 0.0;
    _gnssUpdateTime = 0.0;
    _stopConditionReached = false;

    if (_startTimeSource == StartTimeSource::CurrentComputerTime)
    {
        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t); // NOLINT(concurrency-mt-unsafe)

        _startTime = InsTime{ static_cast<uint16_t>(now->tm_year + 1900), static_cast<uint16_t>(now->tm_mon), static_cast<uint16_t>(now->tm_mday),
                              static_cast<uint16_t>(now->tm_hour), static_cast<uint16_t>(now->tm_min), static_cast<long double>(now->tm_sec) };
        LOG_DEBUG("{}: Start Time set to {}", nameId(), _startTime);
    }

    return true;
}

bool NAV::ImuSimulator::checkStopCondition(double time, const Eigen::Vector3d& position_lla)
{
    if (_stopConditionReached)
    {
        return true;
    }

    if ((_simulationStopCondition == StopCondition::Duration || _trajectoryType == TrajectoryType::Fixed)
        && _imuUpdateTime > _simulationDuration)
    {
        return true;
    }
    if (_simulationStopCondition == StopCondition::DistanceOrCircles && _trajectoryType == TrajectoryType::Linear)
    {
        auto horizontalDistance = calcGeographicalDistance(_startPosition_lla(0), _startPosition_lla(1), position_lla(0), position_lla(1));
        auto distance = std::sqrt(std::pow(horizontalDistance, 2) + std::pow(_startPosition_lla(2) - position_lla(2), 2));
        if (distance > _linearTrajectoryDistanceForStop)
        {
            return true;
        }
    }
    else if (_simulationStopCondition == StopCondition::DistanceOrCircles && (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix))
    {
        auto phi = _circularTrajectoryHorizontalSpeed * time / _circularTrajectoryRadius; // Angle of the current point on the circle
        auto circleCount = phi / (2 * M_PI);
        if (circleCount >= _circularTrajectoryCircleCountForStop)
        {
            return true;
        }
    }

    return false;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollImuObs(bool peek)
{
    Eigen::Vector3d position_lla = calcPosition_lla(_imuUpdateTime);
    LOG_DATA("{}: position_lla = {}°, {}°, {} m", nameId(), trafo::rad2deg(position_lla(0)), trafo::rad2deg(position_lla(1)), position_lla(2));
    Eigen::Vector3d position_e = trafo::lla2ecef_WGS84(position_lla);
    auto q_ne = trafo::quat_ne(position_lla(0), position_lla(1));

    Eigen::Vector3d vel_n = calcVelocity_n(_imuUpdateTime, q_ne);
    LOG_DATA("{}: vel_n = {} [m/s]", nameId(), vel_n.transpose());

    auto [roll, pitch, yaw] = calcFlightAngles(position_lla, vel_n);
    LOG_DATA("{}: roll = {}°, pitch = {}°, yaw = {}°", nameId(), trafo::rad2deg(roll), trafo::rad2deg(pitch), trafo::rad2deg(yaw));

    auto q_bn = trafo::quat_bn(roll, pitch, yaw);

    const Eigen::Vector3d omega_ie_n = q_ne * InsConst::omega_ie_e;
    LOG_DATA("{}: omega_ie_n = {} [rad/s]", nameId(), omega_ie_n.transpose());
    const double R_N = calcEarthRadius_N(position_lla(0));
    LOG_DATA("{}: R_N = {} [m]", nameId(), R_N);
    const double R_E = calcEarthRadius_E(position_lla(0));
    LOG_DATA("{}: R_E = {} [m]", nameId(), R_E);
    const Eigen::Vector3d omega_en_n = calcTransportRate_n(position_lla, vel_n, R_N, R_E);
    LOG_DATA("{}: omega_en_n = {} [rad/s]", nameId(), omega_en_n.transpose());

    // -------------------------------------------------- Check if a stop condition is met -----------------------------------------------------

    if (checkStopCondition(_imuUpdateTime, position_lla))
    {
        _stopConditionReached = true;
        return nullptr;
    }

    // ------------------------------------------------------------ Accelerations --------------------------------------------------------------

    // Force to keep vehicle on track
    const Eigen::Vector3d trajectoryAccel_n = calcTrajectoryAccel_n(_imuUpdateTime, q_ne);
    LOG_DATA("{}: trajectoryAccel_n = {} [m/s^2]", nameId(), trajectoryAccel_n.transpose());

    // Measured acceleration in local-navigation frame coordinates [m/s^2]
    Eigen::Vector3d accel_n = trajectoryAccel_n;
    if (_coriolisAccelerationEnabled) // Apply Coriolis Acceleration
    {
        const Eigen::Vector3d coriolisAcceleration_n = calcCoriolisAcceleration_n(omega_ie_n, omega_en_n, vel_n);
        LOG_DATA("{}: coriolisAcceleration_n = {} [m/s^2]", nameId(), coriolisAcceleration_n.transpose());
        accel_n += coriolisAcceleration_n;
    }

    // Mass attraction of the Earth (gravitation)
    const Eigen::Vector3d gravitation_n = calcGravitation_n(position_lla, _gravityModel);
    LOG_DATA("gravitation_n = {} [m/s^2] ({})", gravitation_n.transpose(), NAV::to_string(_gravityModel));
    accel_n -= gravitation_n; // Apply the local gravity vector

    if (_centrifgalAccelerationEnabled) // Centrifugal acceleration caused by the Earth's rotation
    {
        const Eigen::Vector3d centrifugalAcceleration_e = calcCentrifugalAcceleration_e(position_e);
        LOG_DATA("{}: centrifugalAcceleration_e = {} [m/s^2]", nameId(), centrifugalAcceleration_e.transpose());
        const Eigen::Vector3d centrifugalAcceleration_n = q_ne * centrifugalAcceleration_e;
        LOG_DATA("{}: centrifugalAcceleration_n = {} [m/s^2]", nameId(), centrifugalAcceleration_n.transpose());
        accel_n += centrifugalAcceleration_n;
    }

    // Acceleration measured by the accelerometer in platform coordinates
    Eigen::Vector3d accel_p = _imuPos.quatAccel_pb() * q_bn * accel_n;
    LOG_DATA("{}: accel_p = {} [m/s^2]", nameId(), accel_p.transpose());

    // ------------------------------------------------------------ Angular rates --------------------------------------------------------------

    const Eigen::Vector3d omega_ip_p = calcOmega_ip_p(position_lla, vel_n, trajectoryAccel_n, Eigen::Vector3d{ roll, pitch, yaw }, q_bn, omega_ie_n, omega_en_n);
    LOG_DATA("{}: omega_ip_p = {} [rad/s]", nameId(), omega_ip_p.transpose());

    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------
    auto obs = std::make_shared<ImuObs>(_imuPos);
    obs->timeSinceStartup = static_cast<uint64_t>(_imuUpdateTime * 1e9);
    obs->insTime = _startTime + std::chrono::nanoseconds(obs->timeSinceStartup.value());

    obs->accelCompXYZ = accel_p;
    obs->accelUncompXYZ = accel_p;
    obs->gyroCompXYZ = omega_ip_p;
    obs->gyroUncompXYZ = omega_ip_p;

    obs->magCompXYZ.emplace(0, 0, 0);
    obs->magUncompXYZ.emplace(0, 0, 0);

    // Calls all the callbacks
    if (!peek)
    {
        _imuUpdateTime += 1.0 / _imuFrequency;
        invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
    }

    return obs;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollPosVelAtt(bool peek)
{
    Eigen::Vector3d position_lla = calcPosition_lla(_gnssUpdateTime);
    LOG_DATA("{}: position_lla = {}°, {}°, {} m", nameId(), trafo::rad2deg(position_lla(0)), trafo::rad2deg(position_lla(1)), position_lla(2));
    auto q_ne = trafo::quat_ne(position_lla(0), position_lla(1));
    Eigen::Vector3d vel_n = calcVelocity_n(_gnssUpdateTime, q_ne);
    LOG_DATA("{}: vel_n = {} [m/s]", nameId(), vel_n.transpose());
    auto [roll, pitch, yaw] = calcFlightAngles(position_lla, vel_n);
    LOG_DATA("{}: roll = {}°, pitch = {}°, yaw = {}°", nameId(), trafo::rad2deg(roll), trafo::rad2deg(pitch), trafo::rad2deg(yaw));

    // -------------------------------------------------- Check if a stop condition is met -----------------------------------------------------

    if (checkStopCondition(_gnssUpdateTime, position_lla))
    {
        _stopConditionReached = true;
        return nullptr;
    }

    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------
    auto obs = std::make_shared<PosVelAtt>();
    obs->insTime = _startTime + std::chrono::nanoseconds(static_cast<uint64_t>(_gnssUpdateTime * 1e9));

    obs->setState_n(position_lla, vel_n, trafo::quat_nb(roll, pitch, yaw));

    // Calls all the callbacks
    if (!peek)
    {
        _gnssUpdateTime += 1.0 / _gnssFrequency;
        invokeCallbacks(OUTPUT_PORT_INDEX_POS_VEL_ATT, obs);
    }

    return obs;
}

std::array<double, 3> NAV::ImuSimulator::calcFlightAngles(const Eigen::Vector3d& position_lla, const Eigen::Vector3d& velocity_n)
{
    double roll = 0;
    double pitch = calcPitchFromVelocity(velocity_n);
    double yaw = calcYawFromVelocity(velocity_n);

    if (_trajectoryType == TrajectoryType::Fixed)
    {
        roll = _fixedTrajectoryStartOrientation.x();
        pitch = _fixedTrajectoryStartOrientation.y();
        yaw = _fixedTrajectoryStartOrientation.z();
    }
    else if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
    {
        // The normal vector of the center point of the circle to the ellipsoid expressed in Earth frame coordinates (see https://en.wikipedia.org/wiki/N-vector)
        const Eigen::Vector3d normalVectorCenterCircle_e{ std::cos(_startPosition_lla(0)) * std::cos(_startPosition_lla(1)),
                                                          std::cos(_startPosition_lla(0)) * std::sin(_startPosition_lla(1)),
                                                          std::sin(_startPosition_lla(0)) };
        // The normal vector of the current position to the ellipsoid expressed in Earth frame coordinates
        const Eigen::Vector3d normalVectorCurrentPosition_e{ std::cos(position_lla(0)) * std::cos(position_lla(1)),
                                                             std::cos(position_lla(0)) * std::sin(position_lla(1)),
                                                             std::sin(position_lla(0)) };

        roll = (_circularTrajectoryDirection == Direction::CCW ? -1.0 : 1.0) // CCW = Right wing facing outwards, roll angle measured downwards
               * std::acos(normalVectorCurrentPosition_e.dot(normalVectorCenterCircle_e) / (normalVectorCurrentPosition_e.norm() * normalVectorCenterCircle_e.norm()));
    }

    return { roll, pitch, yaw };
}

Eigen::Vector3d NAV::ImuSimulator::calcPosition_lla(double time)
{
    if (_trajectoryType == TrajectoryType::Fixed)
    {
        return _startPosition_lla;
    }
    if (_trajectoryType == TrajectoryType::Linear)
    {
        // @brief Calculates the derivative of the curvilinear position
        // @param[in] position_lla [𝜙, λ, h]^T Latitude, longitude, altitude in [rad, rad, m]
        // @param[in] velocity_n Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
        // @return The curvilinear position derivative ∂/∂t [𝜙, λ, h]^T
        auto f = [](const Eigen::Vector3d& position_lla, const Eigen::Vector3d& velocity_n) {
            return calcTimeDerivativeForPosition_lla(velocity_n,                          // v_n Velocity with respect to the Earth in local-navigation frame coordinates [m/s]
                                                     position_lla(0),                     // 𝜙 Latitude in [rad]
                                                     position_lla(2),                     // h Altitude in [m]
                                                     calcEarthRadius_N(position_lla(0)),  // North/South (meridian) earth radius [m]
                                                     calcEarthRadius_E(position_lla(0))); // East/West (prime vertical) earth radius [m]
        };

        return RungeKutta1(f, time, _startPosition_lla, _linearTrajectoryVelocity_n);
    }
    if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
    {
        auto phi = _circularTrajectoryHorizontalSpeed * time / _circularTrajectoryRadius; // Angle of the current point on the circle
        phi *= _circularTrajectoryDirection == Direction::CW ? -1 : 1;
        phi += _circularTrajectoryOriginAngle;

        Eigen::Vector3d dx_n{ _circularTrajectoryRadius * std::sin(phi),                                                 // [m]
                              _circularTrajectoryRadius * std::cos(phi),                                                 // [m]
                              _trajectoryType == TrajectoryType::Helix ? _helicalTrajectoryVerticalSpeed * time : 0.0 }; // [m]

        Eigen::Vector3d dx_e = trafo::quat_en(_startPosition_lla(0), _startPosition_lla(1)) * dx_n;

        Eigen::Vector3d x_e = trafo::lla2ecef_WGS84(_startPosition_lla) + dx_e;

        Eigen::Vector3d pos_lla = trafo::ecef2lla_WGS84(x_e);

        return pos_lla;
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d NAV::ImuSimulator::calcVelocity_n(double time, const Eigen::Quaterniond& q_ne)
{
    if (_trajectoryType == TrajectoryType::Fixed)
    {
        return Eigen::Vector3d::Zero();
    }
    if (_trajectoryType == TrajectoryType::Linear)
    {
        return _linearTrajectoryStartVelocity_n + _linearTrajectoryAcceleration_n * time;
    }
    if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
    {
        auto direction = _circularTrajectoryDirection == Direction::CW ? -1 : 1;

        auto phi = _circularTrajectoryHorizontalSpeed * time / _circularTrajectoryRadius; // Angle of the current point on the circle
        phi *= direction;
        phi += _circularTrajectoryOriginAngle;

        Eigen::Vector3d v_e = trafo::quat_en(_startPosition_lla(0), _startPosition_lla(1))
                              * Eigen::Vector3d{ _circularTrajectoryHorizontalSpeed * direction * std::cos(phi),                     // [m/s]
                                                 -_circularTrajectoryHorizontalSpeed * direction * std::sin(phi),                    // [m/s]
                                                 _trajectoryType == TrajectoryType::Helix ? _helicalTrajectoryVerticalSpeed : 0.0 }; // [m/s]

        return q_ne * v_e;
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d NAV::ImuSimulator::calcTrajectoryAccel_n(double time, const Eigen::Quaterniond& q_ne)
{
    if (_trajectoryType == TrajectoryType::Fixed)
    {
        return Eigen::Vector3d::Zero();
    }
    if (_trajectoryType == TrajectoryType::Linear)
    {
        return _linearTrajectoryAcceleration_n;
    }
    if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
    {
        auto direction = _circularTrajectoryDirection == Direction::CW ? -1 : 1;

        auto phi = _circularTrajectoryHorizontalSpeed * time / _circularTrajectoryRadius; // Angle of the current point on the circle
        phi *= direction;
        phi += _circularTrajectoryOriginAngle;

        Eigen::Vector3d a_e = trafo::quat_en(_startPosition_lla(0), _startPosition_lla(1))
                              * Eigen::Vector3d{ -std::pow(_circularTrajectoryHorizontalSpeed, 2) / _circularTrajectoryRadius * std::sin(phi), // [m/s^2]
                                                 -std::pow(_circularTrajectoryHorizontalSpeed, 2) / _circularTrajectoryRadius * std::cos(phi), // [m/s^2]
                                                 0.0 };                                                                                        // [m/s^2]

        return q_ne * a_e;
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d NAV::ImuSimulator::calcOmega_ip_p(const Eigen::Vector3d& position_lla,
                                                  const Eigen::Vector3d& velocity_n,
                                                  const Eigen::Vector3d& acceleration_n,
                                                  const Eigen::Vector3d& rollPitchYaw,
                                                  const Eigen::Quaterniond& q_bn,
                                                  const Eigen::Vector3d& omega_ie_n,
                                                  const Eigen::Vector3d& omega_en_n)
{
    const auto& phi = position_lla(0);
    const auto& lambda = position_lla(1);
    const auto& h = position_lla(2);

    const auto& v_N = velocity_n(0);
    const auto& v_E = velocity_n(1);
    const auto& v_D = velocity_n(2);
    const auto& v_N_2 = std::pow(v_N, 2);
    const auto& v_E_2 = std::pow(v_E, 2);
    const auto& v_D_2 = std::pow(v_D, 2);

    const auto& a_N = acceleration_n(0);
    const auto& a_E = acceleration_n(1);
    const auto& a_D = acceleration_n(2);

    const auto& R = rollPitchYaw(0);
    const auto& P = rollPitchYaw(1);

    const Eigen::Quaterniond q_nb = q_bn.conjugate();

    const double R_N = calcEarthRadius_N(phi);
    const double R_E = calcEarthRadius_E(phi);

    // #########################################################################################################################################

    double R_dot = 0;
    double Y_dot = 0;
    double P_dot = 0;
    if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
    {
        // The normal vector of the center point of the circle to the ellipsoid expressed in Earth frame coordinates (see https://en.wikipedia.org/wiki/N-vector)
        const Eigen::Vector3d normalVectorCenterCircle_e{ std::cos(_startPosition_lla(0)) * std::cos(_startPosition_lla(1)),
                                                          std::cos(_startPosition_lla(0)) * std::sin(_startPosition_lla(1)),
                                                          std::sin(_startPosition_lla(0)) };
        // The normal vector of the current position to the ellipsoid expressed in Earth frame coordinates
        const Eigen::Vector3d normalVectorCurrentPosition_e{ std::cos(phi) * std::cos(lambda),
                                                             std::cos(phi) * std::sin(lambda),
                                                             std::sin(phi) };

        R_dot = normalVectorCenterCircle_e.dot(Eigen::Vector3d{ -v_N / (R_N + h) * std::sin(phi) * std::cos(lambda) - v_E / (R_E + h) * std::sin(lambda),
                                                                -v_N / (R_N + h) * std::sin(phi) * std::sin(lambda) * v_E / (R_E + h) * std::cos(lambda),
                                                                v_N / (R_N + h) * std::cos(phi) })
                * -1 / std::sqrt(1 - std::pow(normalVectorCenterCircle_e.dot(normalVectorCurrentPosition_e), 2));

        Y_dot = (a_E * v_N - v_E * a_N)
                / (v_E_2 + v_N_2);
        P_dot = (-a_D * std::sqrt(v_N_2 + v_E_2) + v_D * (a_N * v_N + a_E * v_E) / std::sqrt(v_N_2 + v_E_2))
                / (v_D_2 + v_N_2 + v_E_2);
    }

    auto C_3 = [](double R) {
        // Eigen::Matrix3d C;
        // // clang-format off
        // C << 1,       0     ,      0     ,
        //      0,  std::cos(R), std::sin(R),
        //      0, -std::sin(R), std::cos(R);
        // // clang-format on
        // return C;
        return Eigen::AngleAxisd{ -R, Eigen::Vector3d::UnitX() };
    };
    auto C_2 = [](double P) {
        // Eigen::Matrix3d C;
        // // clang-format off
        // C << std::cos(P), 0 , -std::sin(P),
        //           0     , 1 ,       0     ,
        //      std::sin(P), 0 ,  std::cos(P);
        // // clang-format on
        // return C;
        return Eigen::AngleAxisd{ -P, Eigen::Vector3d::UnitY() };
    };

    // ω_nb_b = [∂/∂t R] + C_3 [   0  ] + C_3 C_2 [   0  ]
    //          [   0  ]       [∂/∂t P]           [   0  ]
    //          [   0  ]       [   0  ]           [∂/∂t Y]
    const Eigen::Vector3d omega_nb_b = Eigen::Vector3d{ R_dot, 0, 0 }
                                       + C_3(R) * Eigen::Vector3d{ 0, P_dot, 0 }
                                       + C_3(R) * C_2(P) * Eigen::Vector3d{ 0, 0, Y_dot };

    //  ω_ib_n = ω_in_n + ω_nb_n = (ω_ie_n + ω_en_n) + q_nb * ω_nb_b
    Eigen::Vector3d omega_ib_n = q_nb * omega_nb_b;
    if (_angularRateEarthRotationEnabled)
    {
        omega_ib_n += omega_ie_n;
    }
    if (_angularRateTransportRateEnabled)
    {
        omega_ib_n += omega_en_n;
    }

    // ω_ib_b = q_bn * ω_ib_n
    const Eigen::Vector3d omega_ib_b = q_bn * omega_ib_n;

    //                            = 0
    // ω_ip_p = q_pb * (ω_ib_b + ω_bp_b) = q_pb * ω_ib_b
    return _imuPos.quatGyro_pb() * omega_ib_b;
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
    case TrajectoryType::Helix:
        return "Helix";
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