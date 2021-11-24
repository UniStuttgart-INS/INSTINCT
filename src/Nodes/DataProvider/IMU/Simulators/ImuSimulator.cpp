#include "ImuSimulator.hpp"

#include <ctime>

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"
#include "util/InsGravity.hpp"
#include "util/InsMechanization.hpp"
#include "util/NumericalIntegration.hpp"
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

    hasConfig = true;
    guiConfigDefaultWindowSize = { 648, 504 };

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
        if (ImGui::RadioButton(fmt::format("Current Computer Time##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&startTimeSource), static_cast<int>(StartTimeSource::CurrentComputerTime)))
        {
            LOG_DEBUG("{}: startTimeSource changed to {}", nameId(), startTimeSource);
            flow::ApplyChanges();
        }
        if (startTimeSource == StartTimeSource::CurrentComputerTime)
        {
            ImGui::Indent();

            std::time_t t = std::time(nullptr);
            std::tm* now = std::localtime(&t);

            ImGui::Text("%d-%02d-%02d %02d:%02d:%02d", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);

            ImGui::Unindent();
        }

        if (ImGui::RadioButton(fmt::format("Custom Time##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&startTimeSource), static_cast<int>(StartTimeSource::CustomTime)))
        {
            LOG_DEBUG("{}: startTimeSource changed to {}", nameId(), startTimeSource);
            flow::ApplyChanges();
        }
        if (startTimeSource == StartTimeSource::CustomTime)
        {
            ImGui::Indent();
            if (gui::widgets::TimeEdit(fmt::format("{}", size_t(id)).c_str(), startTime, startTimeEditFormat))
            {
                LOG_DEBUG("{}: startTime changed to {}", nameId(), startTime);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Output datarate"))
    {
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("IMU sample rate##{}", size_t(id)).c_str(), &imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.3f Hz"))
        {
            LOG_DEBUG("{}: imuFrequency changed to {}", nameId(), imuFrequency);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDouble(fmt::format("GNSS sample rate##{}", size_t(id)).c_str(), &gnssFrequency, 0.0, 0.0, "%.3f Hz"))
        {
            LOG_DEBUG("{}: gnssFrequency changed to {}", nameId(), gnssFrequency);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Scenario"))
    {
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::BeginCombo(fmt::format("Trajectory##{}", size_t(id)).c_str(), to_string(trajectoryType)))
        {
            for (size_t i = 0; i < static_cast<size_t>(TrajectoryType::COUNT); i++)
            {
                const bool is_selected = (static_cast<size_t>(trajectoryType) == i);
                if (ImGui::Selectable(to_string(static_cast<TrajectoryType>(i)), is_selected))
                {
                    trajectoryType = static_cast<TrajectoryType>(i);
                    LOG_DEBUG("{}: trajectoryType changed to {}", nameId(), trajectoryType);
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
        double latitude = trafo::rad2deg(startPosition_lla.x());
        if (ImGui::InputDoubleL(fmt::format("##Latitude{}", size_t(id)).c_str(), &latitude, -90, 90, 0.0, 0.0, "%.8f°"))
        {
            startPosition_lla.x() = trafo::deg2rad(latitude);
            LOG_DEBUG("{}: latitude changed to {}", nameId(), latitude);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(columnWidth);
        double longitude = trafo::rad2deg(startPosition_lla.y());
        if (ImGui::InputDoubleL(fmt::format("##Longitude{}", size_t(id)).c_str(), &longitude, -180, 180, 0.0, 0.0, "%.8f°"))
        {
            startPosition_lla.y() = trafo::deg2rad(longitude);
            LOG_DEBUG("{}: longitude changed to {}", nameId(), longitude);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDouble(fmt::format("##Altitude (Ellipsoid){}", size_t(id)).c_str(), &startPosition_lla.z(), 0.0, 0.0, "%.3f m"))
        {
            LOG_DEBUG("{}: altitude changed to {}", nameId(), startPosition_lla.y());
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::TextUnformatted(trajectoryType == TrajectoryType::Fixed
                                   ? "Position (Lat, Lon, Alt)"
                                   : (trajectoryType == TrajectoryType::Linear
                                          ? "Start position (Lat, Lon, Alt)"
                                          : (trajectoryType == TrajectoryType::Circular || trajectoryType == TrajectoryType::Helix
                                                 ? "Center position (Lat, Lon, Alt)"
                                                 : "")));

        if (trajectoryType == TrajectoryType::Fixed)
        {
            ImGui::SetNextItemWidth(columnWidth);
            double roll = trafo::rad2deg(startOrientation.x());
            if (ImGui::InputDoubleL(fmt::format("##Roll{}", size_t(id)).c_str(), &roll, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                startOrientation.x() = trafo::deg2rad(roll);
                LOG_DEBUG("{}: roll changed to {}", nameId(), roll);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double pitch = trafo::rad2deg(startOrientation.y());
            if (ImGui::InputDoubleL(fmt::format("##Pitch{}", size_t(id)).c_str(), &pitch, -90, 90, 0.0, 0.0, "%.3f°"))
            {
                startOrientation.y() = trafo::deg2rad(pitch);
                LOG_DEBUG("{}: pitch changed to {}", nameId(), pitch);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            double yaw = trafo::rad2deg(startOrientation.z());
            if (ImGui::InputDoubleL(fmt::format("##Yaw{}", size_t(id)).c_str(), &yaw, -180, 180, 0.0, 0.0, "%.3f°"))
            {
                startOrientation.z() = trafo::deg2rad(yaw);
                LOG_DEBUG("{}: yaw changed to {}", nameId(), yaw);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("startOrientation (Roll, Pitch, Yaw)");
        }
        else if (trajectoryType == TrajectoryType::Linear)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##North velocity{}", size_t(id)).c_str(), &velocity_n.x(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: velocity_n changed to {}", nameId(), velocity_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##East velocity{}", size_t(id)).c_str(), &velocity_n.y(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: velocity_n changed to {}", nameId(), velocity_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDouble(fmt::format("##Down velocity{}", size_t(id)).c_str(), &velocity_n.z(), 0.0, 0.0, "%.3f m/s"))
            {
                LOG_DEBUG("{}: velocity_n changed to {}", nameId(), velocity_n.transpose());
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::TextUnformatted("Velocity (North, East, Down)");
        }
        else if (trajectoryType == TrajectoryType::Circular || trajectoryType == TrajectoryType::Helix)
        {
            if (ImGui::BeginTable(fmt::format("CircularTrajectory##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX))
            {
                ImGui::TableNextColumn();
                auto tableStartX = ImGui::GetCursorPosX();
                ImGui::SetNextItemWidth(200);
                if (ImGui::BeginCombo(fmt::format("Motion##{}", size_t(id)).c_str(), to_string(circularTrajectoryDirection)))
                {
                    for (size_t i = 0; i < static_cast<size_t>(Direction::COUNT); i++)
                    {
                        const bool is_selected = (static_cast<size_t>(circularTrajectoryDirection) == i);
                        if (ImGui::Selectable(to_string(static_cast<Direction>(i)), is_selected))
                        {
                            circularTrajectoryDirection = static_cast<Direction>(i);
                            LOG_DEBUG("{}: circularTrajectoryDirection changed to {}", nameId(), circularTrajectoryDirection);
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
                if (ImGui::InputDoubleL(fmt::format("Radius##{}", size_t(id)).c_str(), &circularTrajectoryRadius, 1e-3, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f m"))
                {
                    LOG_DEBUG("{}: circularTrajectoryRadius changed to {}", nameId(), circularTrajectoryRadius);
                    flow::ApplyChanges();
                }
                // ####################################################################################################
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                double originAngle = trafo::rad2deg(circularTrajectoryOriginAngle);
                if (ImGui::DragDouble(fmt::format("Origin Angle##{}", size_t(id)).c_str(), &originAngle, 15.0, -360.0, 360.0, "%.8f°"))
                {
                    circularTrajectoryOriginAngle = trafo::deg2rad(originAngle);
                    LOG_DEBUG("{}: originAngle changed to {}", nameId(), originAngle);
                    flow::ApplyChanges();
                }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(columnWidth);
                if (ImGui::InputDouble(fmt::format("Horizontal speed##{}", size_t(id)).c_str(), &velocity_n.x(), 0.0, 0.0, "%.3f m/s"))
                {
                    LOG_DEBUG("{}: Horizontal speed changed to {}", nameId(), velocity_n.x());
                    flow::ApplyChanges();
                }
                // ####################################################################################################
                if (trajectoryType == TrajectoryType::Helix)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(columnWidth);
                    double vspeed = -velocity_n.z();
                    if (ImGui::InputDouble(fmt::format("Vertical speed (Up)##{}", size_t(id)).c_str(), &vspeed, 0.0, 0.0, "%.3f m/s"))
                    {
                        velocity_n.z() = -vspeed;
                        LOG_DEBUG("{}: Vertical speed changed to {}", nameId(), vspeed);
                        flow::ApplyChanges();
                    }
                }

                ImGui::EndTable();
            }
        }

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Simulation Stop Condition"))
    {
        if (trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDuration{}", size_t(id)).c_str(), reinterpret_cast<int*>(&simulationStopCondition), static_cast<int>(StopCondition::Duration)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), simulationStopCondition);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
        }
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Duration##{}", size_t(id)).c_str(), &simulationDuration, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f s"))
        {
            LOG_DEBUG("{}: simulationDuration changed to {}", nameId(), simulationDuration);
            flow::ApplyChanges();
        }

        if (trajectoryType != TrajectoryType::Fixed)
        {
            if (ImGui::RadioButton(fmt::format("##simulationStopConditionDistanceOrCircles{}", size_t(id)).c_str(), reinterpret_cast<int*>(&simulationStopCondition), static_cast<int>(StopCondition::DistanceOrCircles)))
            {
                LOG_DEBUG("{}: simulationStopCondition changed to {}", nameId(), simulationStopCondition);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
        }
        if (trajectoryType == TrajectoryType::Linear)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Distance to start##{}", size_t(id)).c_str(), &linearTrajectoryDistanceForStop, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f m"))
            {
                LOG_DEBUG("{}: linearTrajectoryDistanceForStop changed to {}", nameId(), linearTrajectoryDistanceForStop);
                flow::ApplyChanges();
            }
        }
        else if (trajectoryType == TrajectoryType::Circular || trajectoryType == TrajectoryType::Helix)
        {
            ImGui::SetNextItemWidth(columnWidth);
            if (ImGui::InputDoubleL(fmt::format("Amount of Circles##{}", size_t(id)).c_str(), &circularTrajectoryCircleCountForStop, 0.0, std::numeric_limits<double>::max(), 1.0, 1.0, "%.3f"))
            {
                LOG_DEBUG("{}: circularTrajectoryCircleCountForStop changed to {}", nameId(), circularTrajectoryCircleCountForStop);
                flow::ApplyChanges();
            }
        }

        ImGui::TreePop();
    }

    Imu::guiConfig();
}

[[nodiscard]] json NAV::ImuSimulator::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["startTimeSource"] = startTimeSource;
    j["startTimeEditFormat"] = startTimeEditFormat;
    j["startTime"] = startTime;
    // ###########################################################################################################
    j["imuFrequency"] = imuFrequency;
    j["gnssFrequency"] = gnssFrequency;
    // ###########################################################################################################
    j["trajectoryType"] = trajectoryType;
    j["startPosition_lla"] = startPosition_lla;
    j["startOrientation"] = startOrientation;
    j["velocity_n"] = velocity_n;
    j["circularTrajectoryRadius"] = circularTrajectoryRadius;
    j["circularTrajectoryOriginAngle"] = circularTrajectoryOriginAngle;
    j["circularTrajectoryDirection"] = circularTrajectoryDirection;
    // ###########################################################################################################
    j["simulationStopCondition"] = simulationStopCondition;
    j["simulationDuration"] = simulationDuration;
    j["linearTrajectoryDistanceForStop"] = linearTrajectoryDistanceForStop;
    j["circularTrajectoryCircleCountForStop"] = circularTrajectoryCircleCountForStop;
    // ###########################################################################################################
    j["Imu"] = Imu::save();

    return j;
}

void NAV::ImuSimulator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("startTimeSource"))
    {
        j.at("startTimeSource").get_to(startTimeSource);
    }
    if (j.contains("startTimeEditFormat"))
    {
        j.at("startTimeEditFormat").get_to(startTimeEditFormat);
    }
    if (j.contains("startTime"))
    {
        j.at("startTime").get_to(startTime);
    }
    // ###########################################################################################################
    if (j.contains("imuFrequency"))
    {
        j.at("imuFrequency").get_to(imuFrequency);
    }
    if (j.contains("gnssFrequency"))
    {
        j.at("gnssFrequency").get_to(gnssFrequency);
    }
    // ###########################################################################################################
    if (j.contains("trajectoryType"))
    {
        j.at("trajectoryType").get_to(trajectoryType);
    }
    if (j.contains("startPosition_lla"))
    {
        j.at("startPosition_lla").get_to(startPosition_lla);
    }
    if (j.contains("startOrientation"))
    {
        j.at("startOrientation").get_to(startOrientation);
    }
    if (j.contains("velocity_n"))
    {
        j.at("velocity_n").get_to(velocity_n);
    }
    if (j.contains("circularTrajectoryRadius"))
    {
        j.at("circularTrajectoryRadius").get_to(circularTrajectoryRadius);
    }
    if (j.contains("circularTrajectoryOriginAngle"))
    {
        j.at("circularTrajectoryOriginAngle").get_to(circularTrajectoryOriginAngle);
    }
    if (j.contains("circularTrajectoryDirection"))
    {
        j.at("circularTrajectoryDirection").get_to(circularTrajectoryDirection);
    }
    // ###########################################################################################################
    if (j.contains("simulationStopCondition"))
    {
        j.at("simulationStopCondition").get_to(simulationStopCondition);
    }
    if (j.contains("simulationDuration"))
    {
        j.at("simulationDuration").get_to(simulationDuration);
    }
    if (j.contains("linearTrajectoryDistanceForStop"))
    {
        j.at("linearTrajectoryDistanceForStop").get_to(linearTrajectoryDistanceForStop);
    }
    if (j.contains("circularTrajectoryCircleCountForStop"))
    {
        j.at("circularTrajectoryCircleCountForStop").get_to(circularTrajectoryCircleCountForStop);
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

    return true;
}

void NAV::ImuSimulator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::ImuSimulator::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    imuUpdateTime = 0.0;
    gnssUpdateTime = 0.0;

    if (startTimeSource == StartTimeSource::CurrentComputerTime)
    {
        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t);

        startTime = InsTime{ static_cast<uint16_t>(now->tm_year + 1900), static_cast<uint16_t>(now->tm_mon), static_cast<uint16_t>(now->tm_mday),
                             static_cast<uint16_t>(now->tm_hour), static_cast<uint16_t>(now->tm_min), static_cast<long double>(now->tm_sec) };
        LOG_DEBUG("{}: Start Time set to {}", nameId(), startTime);
    }

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollImuObs(bool peek)
{
    if (simulationStopCondition == StopCondition::Duration && imuUpdateTime > simulationDuration)
    {
        return nullptr;
    }
    // TODO: Check other abort conditions

    auto obs = std::make_shared<ImuObs>(imuPos);
    obs->timeSinceStartup = static_cast<uint64_t>(imuUpdateTime * 1e9);
    obs->insTime = startTime + std::chrono::nanoseconds(obs->timeSinceStartup.value());

    // TODO: Fill
    obs->accelCompXYZ.emplace(0, 0, 0);
    obs->accelUncompXYZ.emplace(0, 0, 0);
    obs->gyroCompXYZ.emplace(0, 0, 0);
    obs->gyroUncompXYZ.emplace(0, 0, 0);

    obs->magCompXYZ.emplace(0, 0, 0);
    obs->magUncompXYZ.emplace(0, 0, 0);

    // Calls all the callbacks
    if (!peek)
    {
        imuUpdateTime += 1.0 / imuFrequency;
        invokeCallbacks(OutputPortIndex_ImuObs, obs);
    }

    return obs;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuSimulator::pollPosVelAtt(bool peek)
{
    if (simulationStopCondition == StopCondition::Duration && gnssUpdateTime > simulationDuration)
    {
        return nullptr;
    }
    // else if (simulationStopCondition == StopCondition::DistanceOrCircles && trajectoryType == TrajectoryType::Linear)
    // {

    // }
    // TODO: Check other abort conditions

    auto obs = std::make_shared<PosVelAtt>();
    obs->insTime = startTime + std::chrono::nanoseconds(static_cast<uint64_t>(gnssUpdateTime * 1e9));

    // TODO: Fill
    if (!peek)
    {
        Eigen::Vector3d position_lla = calcPosition(gnssUpdateTime);
        Eigen::Vector3d vel_n = calcVelocity(gnssUpdateTime);

        double roll = 0;
        double pitch = std::atan(-vel_n(2) / std::sqrt(std::pow(vel_n(0), 2) + std::pow(vel_n(1), 2)));
        double yaw = std::atan2(vel_n(1), vel_n(0));

        if (trajectoryType == TrajectoryType::Fixed)
        {
            roll = startOrientation.x();
            pitch = startOrientation.y();
            yaw = startOrientation.z();
        }

        obs->setState_n(position_lla, vel_n, trafo::quat_nb(roll, pitch, yaw));
    }

    // Calls all the callbacks
    if (!peek)
    {
        gnssUpdateTime += 1.0 / gnssFrequency;
        invokeCallbacks(OutputPortIndex_PosVelAtt, obs);
    }

    return obs;
}

Eigen::Vector3d NAV::ImuSimulator::calcPosition(double time)
{
    if (trajectoryType == TrajectoryType::Fixed)
    {
        return startPosition_lla;
    }
    else if (trajectoryType == TrajectoryType::Linear)
    {
        Eigen::Matrix<double, 6, 1> y;
        y.block<3, 1>(0, 0) = startPosition_lla;
        y.block<3, 1>(3, 0) = velocity_n;

        y = Integration::RungeKutta1(curvilinearPositionDerivative, time, y, time);

        return y.block<3, 1>(0, 0);
    }
    else if (trajectoryType == TrajectoryType::Circular || trajectoryType == TrajectoryType::Helix)
    {
        auto phi = velocity_n(0) * time / circularTrajectoryRadius; // Angle of the current point on the circle
        phi *= circularTrajectoryDirection == Direction::CW ? -1 : 1;
        phi += circularTrajectoryOriginAngle;

        auto dEast = circularTrajectoryRadius * std::cos(phi);                             // [m]
        auto dNorth = circularTrajectoryRadius * std::sin(phi);                            // [m]
        auto dDown = trajectoryType == TrajectoryType::Helix ? velocity_n(2) * time : 0.0; // [m]

        auto pos_lla = trafo::ecef2lla_WGS84(trafo::ned2ecef(Eigen::Vector3d{ dNorth, dEast, dDown }, startPosition_lla));
        pos_lla(2) = startPosition_lla(2) - dDown;

        return pos_lla;
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d NAV::ImuSimulator::calcVelocity([[maybe_unused]] double time)
{
    if (trajectoryType == TrajectoryType::Fixed)
    {
        return Eigen::Vector3d::Zero();
    }
    else if (trajectoryType == TrajectoryType::Linear)
    {
        return velocity_n;
    }
    else if (trajectoryType == TrajectoryType::Circular || trajectoryType == TrajectoryType::Helix)
    {
        // TODO: Fill
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d NAV::ImuSimulator::calcAccel([[maybe_unused]] double time)
{
    if (trajectoryType == TrajectoryType::Fixed)
    {
        return Eigen::Vector3d::Zero();
    }
    else if (trajectoryType == TrajectoryType::Linear)
    {
        return Eigen::Vector3d::Zero();
    }
    else if (trajectoryType == TrajectoryType::Circular || trajectoryType == TrajectoryType::Helix)
    {
        // TODO: Fill
    }
    return Eigen::Vector3d::Zero();
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