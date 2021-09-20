#include "PosVelAttInitializer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "util/UartSensors/Ublox/UbloxTypes.hpp"
#include "util/InsTransformations.hpp"
#include "util/InsMath.hpp"

#include "NodeData/State/PosVelAtt.hpp"

#include <limits>

NAV::PosVelAttInitializer::PosVelAttInitializer()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 345, 342 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &PosVelAttInitializer::receiveImuObs);
    nm::CreateInputPin(this, "GnssObs", Pin::Type::Flow, { NAV::UbloxObs::type(), NAV::RtklibPosObs::type(), NAV::PosVelAtt::type() }, &PosVelAttInitializer::receiveGnssObs);

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
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
    if (nm::IsPinLinked(inputPins.at(InputPortIndex_ImuObs).id)
        && !(overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2)))
    {
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputDouble(fmt::format("Initialization Duration Attitude##{}", size_t(id)).c_str(), &initDuration, 0.0, 0.0, "%.3f s"))
        {
            flow::ApplyChanges();
        }
    }

    if (nm::IsPinLinked(inputPins.at(InputPortIndex_ImuObs).id) && nm::IsPinLinked(inputPins.at(InputPortIndex_GnssObs).id)
        && !(overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2)))
    {
        ImGui::SetNextItemWidth(100);
        if (ImGui::Combo(fmt::format("Attitude Init Source##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&attitudeMode), "Both\0IMU\0GNSS\0\0"))
        {
            LOG_DEBUG("{}: Attitude Init Source changed to {}", nameId(), attitudeMode == 0 ? "Both" : (attitudeMode == 1 ? "IMU" : "GNSS"));
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
                                                    posVelAttInitialized.at(0) || overridePosition
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determinePosition" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", posVelAttInitialized.at(0) || overridePosition
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::DragFloat(("##positionAccuracyThreshold" + std::to_string(size_t(id))).c_str(),
                             &positionAccuracyThreshold, 0.1F, 0.0F, 1000.0F, "%.1f cm"))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    lastPositionAccuracy.at(0) <= positionAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastPositionAccuracy.at(0)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", lastPositionAccuracy.at(0));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    lastPositionAccuracy.at(1) <= positionAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastPositionAccuracy.at(1)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", lastPositionAccuracy.at(1));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    lastPositionAccuracy.at(2) <= positionAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastPositionAccuracy.at(2)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", lastPositionAccuracy.at(2));
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
                                                    posVelAttInitialized.at(1) || overrideVelocity
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determineVelocity" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", posVelAttInitialized.at(1) || overrideVelocity
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::DragFloat(("##velocityAccuracyThreshold" + std::to_string(size_t(id))).c_str(),
                             &velocityAccuracyThreshold, 1.0F, 0.0F, 1000.0F, "%.0f cm/s"))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    lastVelocityAccuracy.at(0) <= velocityAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastVelocityAccuracy.at(0)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", lastVelocityAccuracy.at(0));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    lastVelocityAccuracy.at(1) <= velocityAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastVelocityAccuracy.at(1)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", lastVelocityAccuracy.at(1));
        }
        ImGui::SameLine();
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetCursorScreenPos().x + size * 1.2F,
                                                           ImGui::GetCursorScreenPos().y + size * 1.8F),
                                                    size,
                                                    lastVelocityAccuracy.at(2) <= velocityAccuracyThreshold
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##lastVelocityAccuracy.at(2)" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(2.0F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Last: %.3f cm", lastVelocityAccuracy.at(2));
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
                                                    posVelAttInitialized.at(2) || (overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2))
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determineAttitude" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", posVelAttInitialized.at(2) || (overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2))
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();

        ImGui::EndTable();
    }

    if (ImGui::Checkbox(fmt::format("Override Position##{}", size_t(id)).c_str(), &overridePosition))
    {
        flow::ApplyChanges();
    }
    if (overridePosition)
    {
        ImGui::Indent();

        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Latitude [deg]##{}", size_t(id)).c_str(), &overrideValuesPosition_lla.at(0), 1.0F, -90.0F, 90.0F, "%.6f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Longitude [deg]##{}", size_t(id)).c_str(), &overrideValuesPosition_lla.at(1), 1.0F, -180.0F, 180.0F, "%.6f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Altitude [m]##{}", size_t(id)).c_str(), &overrideValuesPosition_lla.at(2)))
        {
            flow::ApplyChanges();
        }

        ImGui::Unindent();
    }

    if (ImGui::Checkbox(fmt::format("Override Velocity##{}", size_t(id)).c_str(), &overrideVelocity))
    {
        flow::ApplyChanges();
    }
    if (overrideVelocity)
    {
        ImGui::Indent();

        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Velocity N [m/s]##{}", size_t(id)).c_str(), &overrideValuesVelocity_n.at(0)))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Velocity E [m/s]##{}", size_t(id)).c_str(), &overrideValuesVelocity_n.at(1)))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Velocity D [m/s]##{}", size_t(id)).c_str(), &overrideValuesVelocity_n.at(2)))
        {
            flow::ApplyChanges();
        }

        ImGui::Unindent();
    }

    if (ImGui::BeginTable(("Overrides##" + std::to_string(size_t(id))).c_str(),
                          2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Roll##" + std::to_string(size_t(id))).c_str(), &overrideRollPitchYaw.at(0)))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (overrideRollPitchYaw.at(0))
        {
            ImGui::SetNextItemWidth(60);
            if (ImGui::DragFloat(("##overrideValuesRollPitchYaw.at(0)" + std::to_string(size_t(id))).c_str(),
                                 &overrideValuesRollPitchYaw.at(0), 1.0F, -180.0F, 180.0F, "%.1f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Pitch##" + std::to_string(size_t(id))).c_str(), &overrideRollPitchYaw.at(1)))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (overrideRollPitchYaw.at(1))
        {
            ImGui::SetNextItemWidth(60);
            if (ImGui::DragFloat(("##overrideValuesRollPitchYaw.at(1)" + std::to_string(size_t(id))).c_str(),
                                 &overrideValuesRollPitchYaw.at(1), 1.0F, -90.0F, 90.0F, "%.1f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Yaw##" + std::to_string(size_t(id))).c_str(), &overrideRollPitchYaw.at(2)))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (overrideRollPitchYaw.at(2))
        {
            ImGui::SetNextItemWidth(60);
            if (ImGui::DragFloat(("##overrideValuesRollPitchYaw.at(2)" + std::to_string(size_t(id))).c_str(),
                                 &overrideValuesRollPitchYaw.at(2), 1.0F, -180.0F, 180.0F, "%.1f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::PosVelAttInitializer::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["initDuration"] = initDuration;
    j["attitudeMode"] = attitudeMode;
    j["positionAccuracyThreshold"] = positionAccuracyThreshold;
    j["velocityAccuracyThreshold"] = velocityAccuracyThreshold;
    j["overridePosition"] = overridePosition;
    j["overrideValuesPosition_lla"] = overrideValuesPosition_lla;
    j["overrideVelocity"] = overrideVelocity;
    j["overrideValuesVelocity_n"] = overrideValuesVelocity_n;
    j["overrideRollPitchYaw"] = overrideRollPitchYaw;
    j["overrideValuesRollPitchYaw"] = overrideValuesRollPitchYaw;

    return j;
}

void NAV::PosVelAttInitializer::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("initDuration"))
    {
        j.at("initDuration").get_to(initDuration);
    }
    if (j.contains("attitudeMode"))
    {
        attitudeMode = static_cast<AttitudeMode>(j.at("attitudeMode").get<int>());
    }
    if (j.contains("positionAccuracyThreshold"))
    {
        j.at("positionAccuracyThreshold").get_to(positionAccuracyThreshold);
    }
    if (j.contains("velocityAccuracyThreshold"))
    {
        j.at("velocityAccuracyThreshold").get_to(velocityAccuracyThreshold);
    }
    if (j.contains("overridePosition"))
    {
        j.at("overridePosition").get_to(overridePosition);
    }
    if (j.contains("overrideValuesPosition_lla"))
    {
        j.at("overrideValuesPosition_lla").get_to(overrideValuesPosition_lla);
    }
    if (j.contains("overrideVelocity"))
    {
        j.at("overrideVelocity").get_to(overrideVelocity);
    }
    if (j.contains("overrideValuesVelocity_n"))
    {
        j.at("overrideValuesVelocity_n").get_to(overrideValuesVelocity_n);
    }
    if (j.contains("overrideRollPitchYaw"))
    {
        j.at("overrideRollPitchYaw").get_to(overrideRollPitchYaw);
    }
    if (j.contains("overrideValuesRollPitchYaw"))
    {
        j.at("overrideValuesRollPitchYaw").get_to(overrideValuesRollPitchYaw);
    }
}

bool NAV::PosVelAttInitializer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    startTime = 0;

    countAveragedAttitude = 0.0;

    lastPositionAccuracy = { std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity() };
    lastVelocityAccuracy = { std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity() };

    posVelAttInitialized = { false, false, false, false };

    return true;
}

void NAV::PosVelAttInitializer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::PosVelAttInitializer::finalizeInit()
{
    if (!posVelAttInitialized.at(3)
        && (posVelAttInitialized.at(0) || overridePosition)
        && (posVelAttInitialized.at(1) || overrideVelocity)
        && (posVelAttInitialized.at(2) || (overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2))))
    {
        posVelAttInitialized.at(3) = true;

        if (overridePosition)
        {
            p_ecef_init = trafo::lla2ecef_WGS84(Eigen::Vector3d(trafo::deg2rad(overrideValuesPosition_lla.at(0)),
                                                                trafo::deg2rad(overrideValuesPosition_lla.at(1)),
                                                                overrideValuesPosition_lla.at(2)));
        }
        if (overrideVelocity)
        {
            v_n_init = Eigen::Vector3d(overrideValuesVelocity_n.at(0), overrideValuesVelocity_n.at(1), overrideValuesVelocity_n.at(2));
        }

        if (overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2))
        {
            q_nb_init = trafo::quat_nb(trafo::deg2rad(overrideValuesRollPitchYaw.at(0)),
                                       trafo::deg2rad(overrideValuesRollPitchYaw.at(1)),
                                       trafo::deg2rad(overrideValuesRollPitchYaw.at(2)));
        }

        [[maybe_unused]] auto latLonAlt = trafo::ecef2lla_WGS84(p_ecef_init);
        LOG_INFO("{}: Position initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", nameId(),
                 trafo::rad2deg(latLonAlt.x()),
                 trafo::rad2deg(latLonAlt.y()),
                 latLonAlt.z());

        if (latLonAlt.z() < 0)
        {
            LOG_WARN("{}: Altitude has a value of {} which is below the ellipsoid height.", nameId(), latLonAlt.z());
        }

        LOG_INFO("{}: Velocity initialized to v_N {:3.5f} [m/s], v_E {:3.5f} [m/s], v_D {:3.5f} [m/s]", nameId(),
                 v_n_init(0), v_n_init(1), v_n_init(2));

        [[maybe_unused]] auto rollPitchYaw = trafo::quat2eulerZYX(q_nb_init);
        LOG_INFO("{}: Attitude initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", nameId(),
                 trafo::rad2deg(rollPitchYaw.x()),
                 trafo::rad2deg(rollPitchYaw.y()),
                 trafo::rad2deg(rollPitchYaw.z()));

        auto posVelAtt = std::make_shared<PosVelAtt>();
        posVelAtt->position_ecef() = p_ecef_init;
        posVelAtt->velocity_n() = v_n_init;
        posVelAtt->quaternion_nb() = q_nb_init;
        invokeCallbacks(OutputPortIndex_PosVelAtt, posVelAtt);
    }
}

void NAV::PosVelAttInitializer::receiveImuObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    if (posVelAttInitialized.at(3))
    {
        return;
    }

    auto obs = std::dynamic_pointer_cast<const ImuObs>(nodeData);

    if (!obs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can only process data with an insTime", nameId());
        return;
    }

    if (startTime == 0)
    {
        startTime = obs->timeSinceStartup.value();
    }

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = obs->imuPos;

    // Choose compenated data if available, otherwise uncompensated
    if (!overrideRollPitchYaw.at(2) && !obs->magUncompXYZ.has_value())
    {
        LOG_ERROR("No magnetometer data available. Please override the Yaw angle.");
        return;
    }
    const Eigen::Vector3d& mag_p = obs->magCompXYZ.has_value() ? obs->magCompXYZ.value() : obs->magUncompXYZ.value();
    const Eigen::Vector3d& accel_p = obs->accelCompXYZ.has_value() ? obs->accelCompXYZ.value() : obs->accelUncompXYZ.value();

    // Calculate Magnetic Heading
    const Eigen::Vector3d mag_b = imuPosition.quatMag_bp() * mag_p;
    auto magneticHeading = -std::atan2(mag_b.y(), mag_b.x());

    // Calculate Roll and Pitch from gravity vector direction (only valid under static conditions)
    const Eigen::Vector3d accel_b = imuPosition.quatAccel_bp() * accel_p * -1;
    auto roll = rollFromStaticAccelerationObs(accel_b);
    auto pitch = pitchFromStaticAccelerationObs(accel_b);

    // TODO: Determine Velocity first and if vehicle not static, initialize the attitude from velocity

    // Average with previous attitude
    countAveragedAttitude++;
    if (countAveragedAttitude > 1)
    {
        averagedAttitude.at(0) = (averagedAttitude.at(0) * (countAveragedAttitude - 1) + roll) / countAveragedAttitude;
        averagedAttitude.at(1) = (averagedAttitude.at(1) * (countAveragedAttitude - 1) + pitch) / countAveragedAttitude;
        averagedAttitude.at(2) = (averagedAttitude.at(2) * (countAveragedAttitude - 1) + magneticHeading) / countAveragedAttitude;
    }
    else
    {
        averagedAttitude.at(0) = roll;
        averagedAttitude.at(1) = pitch;
        averagedAttitude.at(2) = magneticHeading;
    }

    if ((attitudeMode == AttitudeMode_BOTH || attitudeMode == AttitudeMode_IMU || !nm::IsPinLinked(inputPins.at(InputPortIndex_GnssObs).id))
        && (static_cast<double>(obs->timeSinceStartup.value() - startTime) * 1e-9 >= initDuration
            || (overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(2))))
    {
        q_nb_init = trafo::quat_nb(overrideRollPitchYaw.at(0) ? trafo::deg2rad(overrideValuesRollPitchYaw.at(0)) : averagedAttitude.at(0),
                                   overrideRollPitchYaw.at(1) ? trafo::deg2rad(overrideValuesRollPitchYaw.at(1)) : averagedAttitude.at(1),
                                   overrideRollPitchYaw.at(2) ? trafo::deg2rad(overrideValuesRollPitchYaw.at(2)) : averagedAttitude.at(2));

        posVelAttInitialized.at(2) = true;
    }

    finalizeInit();
}

void NAV::PosVelAttInitializer::receiveGnssObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (posVelAttInitialized.at(3))
    {
        return;
    }

    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                receiveRtklibPosObs(std::dynamic_pointer_cast<const RtklibPosObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                receiveUbloxObs(std::dynamic_pointer_cast<const UbloxObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == PosVelAtt::type())
            {
                receivePosVelAttObs(std::dynamic_pointer_cast<PosVelAtt>(nodeData));
            }
        }
    }

    finalizeInit();
}

void NAV::PosVelAttInitializer::receiveUbloxObs(const std::shared_ptr<const UbloxObs>& obs)
{
    if (obs->msgClass == sensors::ublox::UbxClass::UBX_CLASS_NAV)
    {
        auto msgId = static_cast<sensors::ublox::UbxNavMessages>(obs->msgId);
        if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_ATT)
        {
            LOG_DATA("{}: UBX_NAV_ATT: Roll {}, Pitch {}, Heading {} [deg]", nameId(),
                     std::get<sensors::ublox::UbxNavAtt>(obs->data).roll * 1e-5,
                     std::get<sensors::ublox::UbxNavAtt>(obs->data).pitch * 1e-5,
                     std::get<sensors::ublox::UbxNavAtt>(obs->data).heading * 1e-5);
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSECEF)
        {
            lastPositionAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);
            lastPositionAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);
            lastPositionAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);

            if (lastPositionAccuracy.at(0) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(1) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(2) <= positionAccuracyThreshold)
            {
                p_ecef_init = Eigen::Vector3d(std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                                              std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                                              std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);

                posVelAttInitialized.at(0) = true;
            }
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSLLH)
        {
            lastPositionAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            lastPositionAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            lastPositionAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).vAcc * 1e-1);

            if (lastPositionAccuracy.at(0) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(1) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(2) <= positionAccuracyThreshold)
            {
                Eigen::Vector3d latLonAlt(trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                          trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                          std::get<sensors::ublox::UbxNavPosllh>(obs->data).height * 1e-3);

                p_ecef_init = trafo::lla2ecef_WGS84(latLonAlt);

                posVelAttInitialized.at(0) = true;
            }
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_VELNED)
        {
            lastVelocityAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);
            lastVelocityAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);
            lastVelocityAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);

            if (lastVelocityAccuracy.at(0) <= velocityAccuracyThreshold
                && lastVelocityAccuracy.at(1) <= velocityAccuracyThreshold
                && lastVelocityAccuracy.at(2) <= velocityAccuracyThreshold)
            {
                v_n_init = Eigen::Vector3d(std::get<sensors::ublox::UbxNavVelned>(obs->data).velN * 1e-2,
                                           std::get<sensors::ublox::UbxNavVelned>(obs->data).velE * 1e-2,
                                           std::get<sensors::ublox::UbxNavVelned>(obs->data).velD * 1e-2);

                posVelAttInitialized.at(1) = true;
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs)
{
    if (obs->position_ecef.has_value())
    {
        if (obs->sdXYZ.has_value())
        {
            lastPositionAccuracy.at(0) = static_cast<float>(obs->sdXYZ->x() * 1e2);
            lastPositionAccuracy.at(1) = static_cast<float>(obs->sdXYZ->y() * 1e2);
            lastPositionAccuracy.at(2) = static_cast<float>(obs->sdXYZ->z() * 1e2);
        }
        else if (obs->sdNEU.has_value())
        {
            lastPositionAccuracy.at(0) = static_cast<float>(obs->sdNEU->x() * 1e2);
            lastPositionAccuracy.at(1) = static_cast<float>(obs->sdNEU->y() * 1e2);
            lastPositionAccuracy.at(2) = static_cast<float>(obs->sdNEU->z() * 1e2);
        }

        if (lastPositionAccuracy.at(0) <= positionAccuracyThreshold
            && lastPositionAccuracy.at(1) <= positionAccuracyThreshold
            && lastPositionAccuracy.at(2) <= positionAccuracyThreshold)
        {
            p_ecef_init = obs->position_ecef.value();

            posVelAttInitialized.at(0) = true;
        }
    }
}

void NAV::PosVelAttInitializer::receivePosVelAttObs(const std::shared_ptr<PosVelAtt>& obs)
{
    p_ecef_init = obs->position_ecef();
    posVelAttInitialized.at(0) = true;

    v_n_init = obs->velocity_n();
    posVelAttInitialized.at(1) = true;

    if (attitudeMode == AttitudeMode_BOTH || attitudeMode == AttitudeMode_GNSS || !nm::IsPinLinked(inputPins.at(InputPortIndex_ImuObs).id))
    {
        q_nb_init = obs->quaternion_nb();
        posVelAttInitialized.at(2) = true;
    }
}