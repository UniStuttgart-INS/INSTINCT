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
                    deinitializeNode();
                }

                if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::SetNextItemWidth(columnWidth);
        double latitude = trafo::rad2deg(_lla_startPosition.x());
        if (ImGui::InputDoubleL(fmt::format("##Latitude{}", size_t(id)).c_str(), &latitude, -90, 90, 0.0, 0.0, "%.8f°"))
        {
            _lla_startPosition.x() = trafo::deg2rad(latitude);
            LOG_DEBUG("{}: latitude changed to {}", nameId(), latitude);
            flow::ApplyChanges();
            deinitializeNode();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(columnWidth);
        double longitude = trafo::rad2deg(_lla_startPosition.y());
        if (ImGui::InputDoubleL(fmt::format("##Longitude{}", size_t(id)).c_str(), &longitude, -180, 180, 0.0, 0.0, "%.8f°"))
        {
            _lla_startPosition.y() = trafo::deg2rad(longitude);
            LOG_DEBUG("{}: longitude changed to {}", nameId(), longitude);
            flow::ApplyChanges();
            deinitializeNode();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDouble(fmt::format("##Altitude (Ellipsoid){}", size_t(id)).c_str(), &_lla_startPosition.z(), 0.0, 0.0, "%.3f m"))
        {
            LOG_DEBUG("{}: altitude changed to {}", nameId(), _lla_startPosition.y());
            flow::ApplyChanges();
            deinitializeNode();
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
                deinitializeNode();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double pitch = trafo::rad2deg(_fixedTrajectoryStartOrientation.y());
            if (ImGui::InputDoubleL(fmt::format("##Pitch{}", size_t(id)).c_str(), &pitch, -90, 90, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.y() = trafo::deg2rad(pitch);
                LOG_DEBUG("{}: pitch changed to {}", nameId(), pitch);
                flow::ApplyChanges();
                deinitializeNode();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double yaw = trafo::rad2deg(_fixedTrajectoryStartOrientation.z());
            if (ImGui::InputDoubleL(fmt::format("##Yaw{}", size_t(id)).c_str(), &yaw, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                _fixedTrajectoryStartOrientation.z() = trafo::deg2rad(yaw);
                LOG_DEBUG("{}: yaw changed to {}", nameId(), yaw);
                flow::ApplyChanges();
                deinitializeNode();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("startOrientation (Roll, Pitch, Yaw)");
        }
        else if (_trajectoryType == TrajectoryType::Linear)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##North velocity{}", size_t(id)).c_str(), &_n_linearTrajectoryStartVelocity.x(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: n_linearTrajectoryStartVelocity changed to {}", nameId(), _n_linearTrajectoryStartVelocity.transpose());
                flow::ApplyChanges();
                deinitializeNode();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##East velocity{}", size_t(id)).c_str(), &_n_linearTrajectoryStartVelocity.y(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: n_linearTrajectoryStartVelocity changed to {}", nameId(), _n_linearTrajectoryStartVelocity.transpose());
                flow::ApplyChanges();
                deinitializeNode();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##Down velocity{}", size_t(id)).c_str(), &_n_linearTrajectoryStartVelocity.z(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: n_linearTrajectoryStartVelocity changed to {}", nameId(), _n_linearTrajectoryStartVelocity.transpose());
                flow::ApplyChanges();
                deinitializeNode();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Start velocity (North, East, Down)");
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
                            deinitializeNode();
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
                    deinitializeNode();
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
                    deinitializeNode();
                }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("Horizontal speed##{}", size_t(id)).c_str(), &_circularTrajectoryHorizontalSpeed, 0.0, 0.0, "%.3f m/s"))
                {
                    LOG_DEBUG("{}: circularTrajectoryHorizontalSpeed changed to {}", nameId(), _circularTrajectoryHorizontalSpeed);
                    flow::ApplyChanges();
                    deinitializeNode();
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
                        deinitializeNode();
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
                deinitializeNode();
            }
            ImGui::SameLine();
        }
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Duration##{}", size_t(id)).c_str(), &_simulationDuration, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f s"))
        {
            LOG_DEBUG("{}: simulationDuration changed to {}", nameId(), _simulationDuration);
            flow::ApplyChanges();
            deinitializeNode();
        }

        if (_trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDistanceOrCircles{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_simulationStopCondition), static_cast<int>(StopCondition::DistanceOrCircles)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), _simulationStopCondition);
                flow::ApplyChanges();
                deinitializeNode();
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
                deinitializeNode();
            }
        }
        else if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Amount of Circles##{}", size_t(id)).c_str(), &_circularTrajectoryCircleCountForStop, 0.0, std::numeric_limits<double>::max(), 1.0, 1.0, "%.3f"))
            {
                LOG_DEBUG("{}: circularTrajectoryCircleCountForStop changed to {}", nameId(), _circularTrajectoryCircleCountForStop);
                flow::ApplyChanges();
                deinitializeNode();
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
            if (ImGui::BeginCombo(fmt::format("Gravitation Model##{}", size_t(id)).c_str(), NAV::to_string(_gravitationModel)))
            {
                for (size_t i = 0; i < static_cast<size_t>(GravitationModel::COUNT); i++)
                {
                    const bool is_selected = (static_cast<size_t>(_gravitationModel) == i);
                    if (ImGui::Selectable(NAV::to_string(static_cast<GravitationModel>(i)), is_selected))
                    {
                        _gravitationModel = static_cast<GravitationModel>(i);
                        LOG_DEBUG("{}: Gravitation Model changed to {}", nameId(), NAV::to_string(_gravitationModel));
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
    j["startPosition_lla"] = _lla_startPosition;
    j["fixedTrajectoryStartOrientation"] = _fixedTrajectoryStartOrientation;
    j["n_linearTrajectoryStartVelocity"] = _n_linearTrajectoryStartVelocity;
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
    }
    if (j.contains("startPosition_lla"))
    {
        j.at("startPosition_lla").get_to(_lla_startPosition);
    }
    if (j.contains("fixedTrajectoryStartOrientation"))
    {
        j.at("fixedTrajectoryStartOrientation").get_to(_fixedTrajectoryStartOrientation);
    }
    if (j.contains("n_linearTrajectoryStartVelocity"))
    {
        j.at("n_linearTrajectoryStartVelocity").get_to(_n_linearTrajectoryStartVelocity);
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

    initializeSplines();

    return true;
}

void NAV::ImuSimulator::initializeSplines()
{
    std::vector<double> splineTime;
    constexpr double splineSampleInterval = 0.1;

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

        Eigen::Vector3d e_startPosition = trafo::lla2ecef_WGS84(_lla_startPosition);
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
        Eigen::Vector3d e_startPosition = trafo::lla2ecef_WGS84(_lla_startPosition);

        splineTime = { 0.0 };
        std::vector<double> splineX = { e_startPosition[0] };
        std::vector<double> splineY = { e_startPosition[1] };
        std::vector<double> splineZ = { e_startPosition[2] };
        std::vector<double> splineRoll = { roll };
        std::vector<double> splinePitch = { pitch };
        std::vector<double> splineYaw = { yaw };

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

        Eigen::Vector3d lla_lastPosition = _lla_startPosition;
        for (size_t i = 1; i <= static_cast<size_t>(std::round(1.0 / splineSampleInterval)); i++) // Calculate one second backwards
        {
            Eigen::Vector<double, 6> y; // [𝜙, λ, h, v_N, v_E, v_D]^T
            y << lla_lastPosition,
                _n_linearTrajectoryStartVelocity;

            y = RungeKutta1(f, -splineSampleInterval, y, Eigen::Vector3d::Zero());
            lla_lastPosition = y.head<3>();

            Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_lastPosition);

            splineTime.insert(splineTime.begin(), -splineSampleInterval * static_cast<double>(i));
            splineX.insert(splineX.begin(), e_position(0));
            splineY.insert(splineY.begin(), e_position(1));
            splineZ.insert(splineZ.begin(), e_position(2));
            splineRoll.insert(splineRoll.begin(), roll);
            splinePitch.insert(splinePitch.begin(), pitch);
            splineYaw.insert(splineYaw.begin(), yaw);
        }

        lla_lastPosition = _lla_startPosition;
        for (size_t i = 1;; i++)
        {
            Eigen::Vector<double, 6> y; // [𝜙, λ, h, v_N, v_E, v_D]^T
            y << lla_lastPosition,
                _n_linearTrajectoryStartVelocity;

            y = RungeKutta1(f, splineSampleInterval, y, Eigen::Vector3d::Zero());
            lla_lastPosition = y.head<3>();

            Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_lastPosition);

            double simTime = splineSampleInterval * static_cast<double>(i);
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
            if (_simulationStopCondition == StopCondition::DistanceOrCircles)
            {
                auto horizontalDistance = calcGeographicalDistance(_lla_startPosition(0), _lla_startPosition(1), lla_lastPosition(0), lla_lastPosition(1));
                auto distance = std::sqrt(std::pow(horizontalDistance, 2) + std::pow(_lla_startPosition(2) - lla_lastPosition(2), 2))
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
    else if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
    {
        double simDuration{};
        if (_simulationStopCondition == StopCondition::Duration)
        {
            simDuration = _simulationDuration;
        }
        else if (_simulationStopCondition == StopCondition::DistanceOrCircles)
        {
            double omega = _circularTrajectoryHorizontalSpeed / _circularTrajectoryRadius;
            simDuration = _circularTrajectoryCircleCountForStop * 2 * M_PI / omega;
        }

        for (size_t i = 0; i <= static_cast<size_t>(std::round((simDuration + 2.0) / splineSampleInterval)); i++)
        {
            splineTime.push_back(splineSampleInterval * static_cast<double>(i) - 1.0);
        }

        std::vector<double> splineX(splineTime.size());
        std::vector<double> splineY(splineTime.size());
        std::vector<double> splineZ(splineTime.size());
        std::vector<double> splineRoll(splineTime.size());
        std::vector<double> splinePitch(splineTime.size());
        std::vector<double> splineYaw(splineTime.size());

        Eigen::Vector3d e_origin = trafo::lla2ecef_WGS84(_lla_startPosition);

        Eigen::Quaterniond e_quatCenter_n = trafo::e_Quat_n(_lla_startPosition(0), _lla_startPosition(1));

        for (uint64_t i = 0; i < splineTime.size(); i++)
        {
            auto phi = _circularTrajectoryHorizontalSpeed * splineTime[i] / _circularTrajectoryRadius; // Angle of the current point on the circle
            phi *= _circularTrajectoryDirection == Direction::CW ? -1 : 1;
            phi += _circularTrajectoryOriginAngle;

            Eigen::Vector3d n_relativePosition{ _circularTrajectoryRadius * std::sin(phi),                                                           // [m]
                                                _circularTrajectoryRadius * std::cos(phi),                                                           // [m]
                                                _trajectoryType == TrajectoryType::Helix ? -_helicalTrajectoryVerticalSpeed * splineTime[i] : 0.0 }; // [m]

            Eigen::Vector3d e_relativePosition = e_quatCenter_n * n_relativePosition;

            Eigen::Vector3d e_position = e_origin + e_relativePosition;

            splineX[i] = e_position[0];
            splineY[i] = e_position[1];
            splineZ[i] = e_position[2];
        }

        _splines.x.setPoints(splineTime, splineX);
        _splines.y.setPoints(splineTime, splineY);
        _splines.z.setPoints(splineTime, splineZ);

        for (uint64_t i = 0; i < splineTime.size(); i++)
        {
            Eigen::Vector3d lla_position = lla_calcPosition(splineTime[i]);
            auto n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));
            Eigen::Vector3d n_velocity = n_calcVelocity(splineTime[i], n_Quat_e);

            Eigen::Vector3d e_normalVectorCenterCircle{ std::cos(_lla_startPosition(0)) * std::cos(_lla_startPosition(1)),
                                                        std::cos(_lla_startPosition(0)) * std::sin(_lla_startPosition(1)),
                                                        std::sin(_lla_startPosition(0)) };

            Eigen::Vector3d e_normalVectorCurrentPosition{ std::cos(lla_position(0)) * std::cos(lla_position(1)),
                                                           std::cos(lla_position(0)) * std::sin(lla_position(1)),
                                                           std::sin(lla_position(0)) };

            double yaw = calcYawFromVelocity(n_velocity);

            if (i > 0)
            {
                double x = yaw - splineYaw[i - 1];
                x = fmod(x + M_PI, 2 * M_PI);
                if (x < 0)
                {
                    x += 2 * M_PI;
                }
                x -= M_PI;

                splineYaw[i] = splineYaw[i - 1] + x;
            }
            else
            {
                splineYaw[i] = yaw;
            }

            splineRoll[i] = (_circularTrajectoryDirection == Direction::CCW ? -1.0 : 1.0) // CCW = Right wing facing outwards, roll angle measured downwards
                            * std::acos(e_normalVectorCurrentPosition.dot(e_normalVectorCenterCircle) / (e_normalVectorCurrentPosition.norm() * e_normalVectorCenterCircle.norm()));
            splinePitch[i] = n_velocity.head<2>().norm() > 1e-8 ? calcPitchFromVelocity(n_velocity) : 0;
        }

        _splines.roll.setPoints(splineTime, splineRoll);
        _splines.pitch.setPoints(splineTime, splinePitch);
        _splines.yaw.setPoints(splineTime, splineYaw);
    }
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
    _lla_imuLastLinearPosition = _lla_startPosition;
    _lla_gnssLastLinearPosition = _lla_startPosition;

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

bool NAV::ImuSimulator::checkStopCondition(double time, const Eigen::Vector3d& lla_position)
{
    if (_simulationStopCondition == StopCondition::Duration
        || _trajectoryType == TrajectoryType::Fixed)
    {
        return time > _simulationDuration;
    }
    if (_simulationStopCondition == StopCondition::DistanceOrCircles)
    {
        if (_trajectoryType == TrajectoryType::Linear)
        {
            auto horizontalDistance = calcGeographicalDistance(_lla_startPosition(0), _lla_startPosition(1), lla_position(0), lla_position(1));
            auto distance = std::sqrt(std::pow(horizontalDistance, 2) + std::pow(_lla_startPosition(2) - lla_position(2), 2));
            return distance > _linearTrajectoryDistanceForStop;
        }
        if (_trajectoryType == TrajectoryType::Circular || _trajectoryType == TrajectoryType::Helix)
        {
            double omega = _circularTrajectoryHorizontalSpeed / _circularTrajectoryRadius;
            double simDuration = _circularTrajectoryCircleCountForStop * 2 * M_PI / omega;
            return time > simDuration;
        }
    }
    return false;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollImuObs(bool peek)
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
        auto obs = std::make_shared<InsObs>();
        obs->insTime = _startTime + std::chrono::duration<double>(imuUpdateTime - 1e-9);
        return obs;
    }

    auto obs = std::make_shared<ImuObs>(_imuPos);
    obs->timeSinceStartup = static_cast<uint64_t>(imuUpdateTime * 1e9);
    obs->insTime = _startTime + std::chrono::duration<double>(imuUpdateTime);
    LOG_DATA("{}: Simulating IMU data for time [{}]", nameId(), obs->insTime->toYMDHMS());

    // --------------------------------------------------------- Calculation of data -----------------------------------------------------------
    LOG_DATA("{}: [{:8.3f}] lla_position = {}°, {}°, {} m", nameId(), imuUpdateTime, trafo::rad2deg(lla_position(0)), trafo::rad2deg(lla_position(1)), lla_position(2));
    Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_position);
    auto n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));

    Eigen::Vector3d n_vel = n_calcVelocity(imuUpdateTime, n_Quat_e);
    LOG_DATA("{}: [{:8.3f}] n_vel = {} [m/s]", nameId(), imuUpdateTime, n_vel.transpose());

    auto [roll, pitch, yaw] = calcFlightAngles(imuUpdateTime);
    LOG_DATA("{}: [{:8.3f}] roll = {}°, pitch = {}°, yaw = {}°", nameId(), imuUpdateTime, trafo::rad2deg(roll), trafo::rad2deg(pitch), trafo::rad2deg(yaw));

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

    Eigen::Vector3d omega_ip_p = p_calcOmega_ip(imuUpdateTime, Eigen::Vector3d{ roll, pitch, yaw }, b_Quat_n, n_omega_ie, n_omega_en);
    LOG_DATA("{}: [{:8.3f}] omega_ip_p = {} [rad/s]", nameId(), imuUpdateTime, omega_ip_p.transpose());

    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------

    obs->accelCompXYZ = accel_p;
    obs->accelUncompXYZ = accel_p;
    obs->gyroCompXYZ = omega_ip_p;
    obs->gyroUncompXYZ = omega_ip_p;

    obs->magCompXYZ.emplace(0, 0, 0);
    obs->magUncompXYZ.emplace(0, 0, 0);

    _imuUpdateCnt++;
    invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);

    return obs;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollPosVelAtt(bool peek)
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
        auto obs = std::make_shared<InsObs>();
        obs->insTime = _startTime + std::chrono::duration<double>(gnssUpdateTime);
        return obs;
    }
    auto obs = std::make_shared<PosVelAtt>();
    obs->insTime = _startTime + std::chrono::duration<double>(gnssUpdateTime);
    LOG_DATA("{}: Simulating GNSS data for time [{}]", nameId(), obs->insTime->toYMDHMS());

    // --------------------------------------------------------- Calculation of data -----------------------------------------------------------
    LOG_DATA("{}: [{:8.3f}] lla_position = {}°, {}°, {} m", nameId(), gnssUpdateTime, trafo::rad2deg(lla_position(0)), trafo::rad2deg(lla_position(1)), lla_position(2));
    auto n_Quat_e = trafo::n_Quat_e(lla_position(0), lla_position(1));
    Eigen::Vector3d n_vel = n_calcVelocity(gnssUpdateTime, n_Quat_e);
    LOG_DATA("{}: [{:8.3f}] n_vel = {} [m/s]", nameId(), gnssUpdateTime, n_vel.transpose());
    auto [roll, pitch, yaw] = calcFlightAngles(gnssUpdateTime);
    LOG_DATA("{}: [{:8.3f}] roll = {}°, pitch = {}°, yaw = {}°", nameId(), gnssUpdateTime, trafo::rad2deg(roll), trafo::rad2deg(pitch), trafo::rad2deg(yaw));

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
                                  * skewSymmetricMatrix(n_calcTransportRate(lla_position, n_velocity,
                                                                            calcEarthRadius_N(lla_position(0)),
                                                                            calcEarthRadius_E(lla_position(0))));

    // Math: \dot{C}_e^n = (\dot{C}_n^e)^T
    Eigen::Matrix3d e_DCM_dot_n = n_DCM_dot_e.transpose();

    // Math: a^n = \frac{\partial}{\partial t} \left( \dot{x}^n \right) = \frac{\partial}{\partial t} \left( C_e^n \cdot \dot{x}^e \right) = \dot{C}_e^n \cdot \dot{x}^e + C_e^n \cdot \ddot{x}^e
    return e_DCM_dot_n * e_vel + n_Quat_e * e_accel;
}

Eigen::Vector3d NAV::ImuSimulator::p_calcOmega_ip(double time,
                                                  const Eigen::Vector3d& rollPitchYaw,
                                                  const Eigen::Quaterniond& b_Quat_n,
                                                  const Eigen::Vector3d& n_omega_ie,
                                                  const Eigen::Vector3d& n_omega_en) const
{
    const auto& R = rollPitchYaw(0);
    const auto& P = rollPitchYaw(1);

    Eigen::Quaterniond n_Quat_b = b_Quat_n.conjugate();

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

    // ω_ib_b = b_Quat_n * ω_ib_n
    Eigen::Vector3d b_omega_ib = b_Quat_n * n_omega_ib;

    //                            = 0
    // ω_ip_p = p_Quat_b * (ω_ib_b + ω_bp_b) = p_Quat_b * ω_ib_b
    return _imuPos.p_quatGyro_b() * b_omega_ib;
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