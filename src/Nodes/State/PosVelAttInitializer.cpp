#include "PosVelAttInitializer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"

#include "util/UartSensors/Ublox/UbloxTypes.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "NodeData/State/PosVelAtt.hpp"

#include <limits>

NAV::PosVelAttInitializer::PosVelAttInitializer()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 345, 342 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &PosVelAttInitializer::receiveImuObs);
    nm::CreateInputPin(this, "PosVelAttInit", Pin::Type::Flow, { NAV::UbloxObs::type(), NAV::RtklibPosObs::type(), NAV::PosVelAtt::type() }, &PosVelAttInitializer::receiveGnssObs);

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
    if (nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_IMU_OBS).id)
        && !(_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2)))
    {
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputDouble(fmt::format("Initialization Duration Attitude##{}", size_t(id)).c_str(), &_initDuration, 0.0, 0.0, "%.3f s"))
        {
            flow::ApplyChanges();
        }
    }

    if (nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_IMU_OBS).id) && nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_GNSS_OBS).id)
        && !(_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2)))
    {
        ImGui::SetNextItemWidth(100);
        if (ImGui::BeginCombo(fmt::format("Attitude Init Source##{}", size_t(id)).c_str(), to_string(_attitudeMode)))
        {
            for (size_t i = 0; i < static_cast<size_t>(AttitudeMode::COUNT); i++)
            {
                const bool is_selected = (static_cast<size_t>(_attitudeMode) == i);
                if (ImGui::Selectable(to_string(static_cast<AttitudeMode>(i)), is_selected))
                {
                    _attitudeMode = static_cast<AttitudeMode>(i);
                    LOG_DEBUG("{}: Attitude Init Source changed to {}", nameId(), to_string(_attitudeMode));
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
                                                    _posVelAttInitialized.at(0) || _overridePosition
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determinePosition" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", _posVelAttInitialized.at(0) || _overridePosition
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::DragFloat(("##positionAccuracyThreshold" + std::to_string(size_t(id))).c_str(),
                             &_positionAccuracyThreshold, 0.1F, 0.0F, 1000.0F, "%.1f cm"))
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
                                                    _posVelAttInitialized.at(1) || _overrideVelocity
                                                        ? ImColor(0.0F, 255.0F, 0.0F)
                                                        : ImColor(255.0F, 0.0F, 0.0F));
        ImGui::Selectable(("##determineVelocity" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", _posVelAttInitialized.at(1) || _overrideVelocity
                                        ? "Successfully Initialized"
                                        : "To be initialized");
        }
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::DragFloat(("##velocityAccuracyThreshold" + std::to_string(size_t(id))).c_str(),
                             &_velocityAccuracyThreshold, 1.0F, 0.0F, 1000.0F, "%.0f cm/s"))
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

    if (ImGui::Checkbox(fmt::format("Override Position##{}", size_t(id)).c_str(), &_overridePosition))
    {
        flow::ApplyChanges();
    }
    if (_overridePosition)
    {
        ImGui::Indent();

        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Latitude [deg]##{}", size_t(id)).c_str(), &_overrideValuesPosition_lla.at(0), 1.0F, -90.0F, 90.0F, "%.6f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Longitude [deg]##{}", size_t(id)).c_str(), &_overrideValuesPosition_lla.at(1), 1.0F, -180.0F, 180.0F, "%.6f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Altitude [m]##{}", size_t(id)).c_str(), &_overrideValuesPosition_lla.at(2)))
        {
            flow::ApplyChanges();
        }

        ImGui::Unindent();
    }

    if (ImGui::Checkbox(fmt::format("Override Velocity##{}", size_t(id)).c_str(), &_overrideVelocity))
    {
        flow::ApplyChanges();
    }
    if (_overrideVelocity)
    {
        ImGui::Indent();

        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Velocity N [m/s]##{}", size_t(id)).c_str(), &_overrideValuesVelocity_n.at(0)))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Velocity E [m/s]##{}", size_t(id)).c_str(), &_overrideValuesVelocity_n.at(1)))
        {
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::DragFloat(fmt::format("Velocity D [m/s]##{}", size_t(id)).c_str(), &_overrideValuesVelocity_n.at(2)))
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
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (_overrideRollPitchYaw.at(0))
        {
            ImGui::SetNextItemWidth(60);
            if (ImGui::DragFloat(("##overrideValuesRollPitchYaw.at(0)" + std::to_string(size_t(id))).c_str(),
                                 &_overrideValuesRollPitchYaw.at(0), 1.0F, -180.0F, 180.0F, "%.1f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Pitch##" + std::to_string(size_t(id))).c_str(), &_overrideRollPitchYaw.at(1)))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (_overrideRollPitchYaw.at(1))
        {
            ImGui::SetNextItemWidth(60);
            if (ImGui::DragFloat(("##overrideValuesRollPitchYaw.at(1)" + std::to_string(size_t(id))).c_str(),
                                 &_overrideValuesRollPitchYaw.at(1), 1.0F, -90.0F, 90.0F, "%.1f °"))
            {
                flow::ApplyChanges();
            }
        }

        ImGui::TableNextColumn();
        if (ImGui::Checkbox(("Override Yaw##" + std::to_string(size_t(id))).c_str(), &_overrideRollPitchYaw.at(2)))
        {
            flow::ApplyChanges();
        }
        ImGui::TableNextColumn();
        if (_overrideRollPitchYaw.at(2))
        {
            ImGui::SetNextItemWidth(60);
            if (ImGui::DragFloat(("##overrideValuesRollPitchYaw.at(2)" + std::to_string(size_t(id))).c_str(),
                                 &_overrideValuesRollPitchYaw.at(2), 1.0F, -180.0F, 180.0F, "%.1f °"))
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

    j["initDuration"] = _initDuration;
    j["attitudeMode"] = _attitudeMode;
    j["positionAccuracyThreshold"] = _positionAccuracyThreshold;
    j["velocityAccuracyThreshold"] = _velocityAccuracyThreshold;
    j["overridePosition"] = _overridePosition;
    j["overrideValuesPosition_lla"] = _overrideValuesPosition_lla;
    j["overrideVelocity"] = _overrideVelocity;
    j["overrideValuesVelocity_n"] = _overrideValuesVelocity_n;
    j["overrideRollPitchYaw"] = _overrideRollPitchYaw;
    j["overrideValuesRollPitchYaw"] = _overrideValuesRollPitchYaw;

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
    if (j.contains("overrideValuesPosition_lla"))
    {
        j.at("overrideValuesPosition_lla").get_to(_overrideValuesPosition_lla);
    }
    if (j.contains("overrideVelocity"))
    {
        j.at("overrideVelocity").get_to(_overrideVelocity);
    }
    if (j.contains("overrideValuesVelocity_n"))
    {
        j.at("overrideValuesVelocity_n").get_to(_overrideValuesVelocity_n);
    }
    if (j.contains("overrideRollPitchYaw"))
    {
        j.at("overrideRollPitchYaw").get_to(_overrideRollPitchYaw);
    }
    if (j.contains("overrideValuesRollPitchYaw"))
    {
        j.at("overrideValuesRollPitchYaw").get_to(_overrideValuesRollPitchYaw);
    }
}

bool NAV::PosVelAttInitializer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _startTime = 0;

    _countAveragedAttitude = 0.0;

    _lastPositionAccuracy = { std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity() };
    _lastVelocityAccuracy = { std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::infinity() };

    _posVelAttInitialized = { false, false, false, false };

    return true;
}

void NAV::PosVelAttInitializer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::PosVelAttInitializer::finalizeInit()
{
    if (!_posVelAttInitialized.at(3)
        && (_posVelAttInitialized.at(0) || _overridePosition)
        && (_posVelAttInitialized.at(1) || _overrideVelocity)
        && (_posVelAttInitialized.at(2) || (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))))
    {
        _posVelAttInitialized.at(3) = true;

        if (_overridePosition)
        {
            _p_ecef_init = trafo::lla2ecef_WGS84(Eigen::Vector3d(trafo::deg2rad(_overrideValuesPosition_lla.at(0)),
                                                                 trafo::deg2rad(_overrideValuesPosition_lla.at(1)),
                                                                 _overrideValuesPosition_lla.at(2)));
        }
        if (_overrideVelocity)
        {
            _n_velocity_init = Eigen::Vector3d(_overrideValuesVelocity_n.at(0), _overrideValuesVelocity_n.at(1), _overrideValuesVelocity_n.at(2));
        }

        if (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))
        {
            _n_Quat_b_init = trafo::n_Quat_b(trafo::deg2rad(_overrideValuesRollPitchYaw.at(0)),
                                             trafo::deg2rad(_overrideValuesRollPitchYaw.at(1)),
                                             trafo::deg2rad(_overrideValuesRollPitchYaw.at(2)));
        }

        [[maybe_unused]] auto latLonAlt = trafo::ecef2lla_WGS84(_p_ecef_init);
        LOG_INFO("{}: Position initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", nameId(),
                 trafo::rad2deg(latLonAlt.x()),
                 trafo::rad2deg(latLonAlt.y()),
                 latLonAlt.z());

        if (latLonAlt.z() < 0)
        {
            LOG_WARN("{}: Altitude has a value of {} which is below the ellipsoid height.", nameId(), latLonAlt.z());
        }

        LOG_INFO("{}: Velocity initialized to v_N {:3.5f} [m/s], v_E {:3.5f} [m/s], v_D {:3.5f} [m/s]", nameId(),
                 _n_velocity_init(0), _n_velocity_init(1), _n_velocity_init(2));

        [[maybe_unused]] auto rollPitchYaw = trafo::quat2eulerZYX(_n_Quat_b_init);
        LOG_INFO("{}: Attitude initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", nameId(),
                 trafo::rad2deg(rollPitchYaw.x()),
                 trafo::rad2deg(rollPitchYaw.y()),
                 trafo::rad2deg(rollPitchYaw.z()));

        auto posVelAtt = std::make_shared<PosVelAtt>();
        posVelAtt->setPosition_e(_p_ecef_init);
        posVelAtt->setVelocity_n(_n_velocity_init);
        posVelAtt->setAttitude_nb(_n_Quat_b_init);
        invokeCallbacks(OUTPUT_PORT_INDEX_POS_VEL_ATT, posVelAtt);
    }
}

void NAV::PosVelAttInitializer::receiveImuObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    if (_posVelAttInitialized.at(3))
    {
        return;
    }

    auto obs = std::static_pointer_cast<const ImuObs>(nodeData);

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
    const Eigen::Vector3d mag_b = imuPosition.quatMag_bp() * mag_p;
    auto magneticHeading = -std::atan2(mag_b.y(), mag_b.x());

    // Calculate Roll and Pitch from gravity vector direction (only valid under static conditions)
    const Eigen::Vector3d accel_b = imuPosition.quatAccel_bp() * accel_p * -1;
    auto roll = calcRollFromStaticAcceleration(accel_b);
    auto pitch = calcPitchFromStaticAcceleration(accel_b);

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

    if ((_attitudeMode == AttitudeMode::BOTH || _attitudeMode == AttitudeMode::IMU || !nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_GNSS_OBS).id))
        && (static_cast<double>(obs->timeSinceStartup.value() - _startTime) * 1e-9 >= _initDuration
            || (_overrideRollPitchYaw.at(0) && _overrideRollPitchYaw.at(1) && _overrideRollPitchYaw.at(2))))
    {
        _n_Quat_b_init = trafo::n_Quat_b(_overrideRollPitchYaw.at(0) ? trafo::deg2rad(_overrideValuesRollPitchYaw.at(0)) : _averagedAttitude.at(0),
                                         _overrideRollPitchYaw.at(1) ? trafo::deg2rad(_overrideValuesRollPitchYaw.at(1)) : _averagedAttitude.at(1),
                                         _overrideRollPitchYaw.at(2) ? trafo::deg2rad(_overrideValuesRollPitchYaw.at(2)) : _averagedAttitude.at(2));

        _posVelAttInitialized.at(2) = true;
    }

    finalizeInit();
}

void NAV::PosVelAttInitializer::receiveGnssObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (_posVelAttInitialized.at(3))
    {
        return;
    }

    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                receiveRtklibPosObs(std::static_pointer_cast<const RtklibPosObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                receiveUbloxObs(std::static_pointer_cast<const UbloxObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == PosVelAtt::type())
            {
                receivePosVelAttObs(std::static_pointer_cast<const PosVelAtt>(nodeData));
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
            _lastPositionAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);
            _lastPositionAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);
            _lastPositionAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);

            if (_lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold)
            {
                _p_ecef_init = Eigen::Vector3d(std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                                               std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                                               std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);

                _posVelAttInitialized.at(0) = true;
            }
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSLLH)
        {
            _lastPositionAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            _lastPositionAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            _lastPositionAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).vAcc * 1e-1);

            if (_lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
                && _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold)
            {
                Eigen::Vector3d latLonAlt(trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                          trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                          std::get<sensors::ublox::UbxNavPosllh>(obs->data).height * 1e-3);

                _p_ecef_init = trafo::lla2ecef_WGS84(latLonAlt);

                _posVelAttInitialized.at(0) = true;
            }
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_VELNED)
        {
            _lastVelocityAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);
            _lastVelocityAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);
            _lastVelocityAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);

            if (_lastVelocityAccuracy.at(0) <= _velocityAccuracyThreshold
                && _lastVelocityAccuracy.at(1) <= _velocityAccuracyThreshold
                && _lastVelocityAccuracy.at(2) <= _velocityAccuracyThreshold)
            {
                _n_velocity_init = Eigen::Vector3d(std::get<sensors::ublox::UbxNavVelned>(obs->data).velN * 1e-2,
                                                   std::get<sensors::ublox::UbxNavVelned>(obs->data).velE * 1e-2,
                                                   std::get<sensors::ublox::UbxNavVelned>(obs->data).velD * 1e-2);

                _posVelAttInitialized.at(1) = true;
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
            _lastPositionAccuracy.at(0) = static_cast<float>(obs->sdXYZ->x() * 1e2);
            _lastPositionAccuracy.at(1) = static_cast<float>(obs->sdXYZ->y() * 1e2);
            _lastPositionAccuracy.at(2) = static_cast<float>(obs->sdXYZ->z() * 1e2);
        }
        else if (obs->sdNEU.has_value())
        {
            _lastPositionAccuracy.at(0) = static_cast<float>(obs->sdNEU->x() * 1e2);
            _lastPositionAccuracy.at(1) = static_cast<float>(obs->sdNEU->y() * 1e2);
            _lastPositionAccuracy.at(2) = static_cast<float>(obs->sdNEU->z() * 1e2);
        }

        if (_lastPositionAccuracy.at(0) <= _positionAccuracyThreshold
            && _lastPositionAccuracy.at(1) <= _positionAccuracyThreshold
            && _lastPositionAccuracy.at(2) <= _positionAccuracyThreshold)
        {
            _p_ecef_init = obs->position_ecef.value();

            _posVelAttInitialized.at(0) = true;
        }
    }
}

void NAV::PosVelAttInitializer::receivePosVelAttObs(const std::shared_ptr<const PosVelAtt>& obs)
{
    _p_ecef_init = obs->position_ecef();
    _posVelAttInitialized.at(0) = true;

    _n_velocity_init = obs->n_velocity();
    _posVelAttInitialized.at(1) = true;

    if (_attitudeMode == AttitudeMode::BOTH || _attitudeMode == AttitudeMode::GNSS || !nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_IMU_OBS).id))
    {
        if (_overrideRollPitchYaw.at(0) || _overrideRollPitchYaw.at(1) || _overrideRollPitchYaw.at(2))
        {
            const Eigen::Vector3d rollPitchYaw = obs->rollPitchYaw();

            _n_Quat_b_init = trafo::n_Quat_b(_overrideRollPitchYaw.at(0) ? trafo::deg2rad(_overrideValuesRollPitchYaw.at(0)) : rollPitchYaw(0),
                                             _overrideRollPitchYaw.at(1) ? trafo::deg2rad(_overrideValuesRollPitchYaw.at(1)) : rollPitchYaw(1),
                                             _overrideRollPitchYaw.at(2) ? trafo::deg2rad(_overrideValuesRollPitchYaw.at(2)) : rollPitchYaw(2));
        }
        else
        {
            _n_Quat_b_init = obs->n_Quat_b();
        }

        _posVelAttInitialized.at(2) = true;
    }
}

const char* NAV::PosVelAttInitializer::to_string(AttitudeMode attitudeMode)
{
    switch (attitudeMode)
    {
    case AttitudeMode::BOTH:
        return "Both";
    case AttitudeMode::IMU:
        return "IMU";
    case AttitudeMode::GNSS:
        return "GNSS";
    case AttitudeMode::COUNT:
        return "";
    }
    return "";
}