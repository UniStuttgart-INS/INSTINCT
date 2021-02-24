#include "PosVelAttInitializer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/UartSensors/Ublox/UbloxTypes.hpp"
#include "util/InsTransformations.hpp"
#include "util/InsMath.hpp"

#include <limits>

NAV::PosVelAttInitializer::PosVelAttInitializer()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &PosVelAttInitializer::receiveImuObs);
    nm::CreateInputPin(this, "GnssObs", Pin::Type::Flow, { NAV::UbloxObs::type(), NAV::RtklibPosObs::type() }, &PosVelAttInitializer::receiveGnssObs);
    nm::CreateInputPin(this, "Position ECEF", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Velocity NED", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Quaternion nb", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow);
    nm::CreateOutputPin(this, "GnssObs", Pin::Type::Flow);
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
    if (nm::IsPinLinked(inputPins.at(InputPortIndex_ImuObs).id))
    {
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputDouble("Initialization Duration Attitude", &initDuration, 0.0, 0.0, "%.3f s"))
        {
            flow::ApplyChanges();
        }
    }

    if (ImGui::BeginTable(("Initialized State##" + std::to_string(size_t(id))).c_str(),
                          4, ImGuiTableFlags_Borders | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
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
                                                    determinePosition == InitFlag::NOT_CONNECTED
                                                        ? ImColor(255.0F, 255.0F, 0.0F)
                                                        : (determinePosition == InitFlag::CONNECTED
                                                               ? ImColor(255.0F, 0.0F, 0.0F)
                                                               : ImColor(0.0F, 255.0F, 0.0F)));
        ImGui::Selectable(("##determinePosition" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", determinePosition == InitFlag::NOT_CONNECTED
                                        ? "Not connected, so won't be initialized"
                                        : (determinePosition == InitFlag::CONNECTED
                                               ? "To be initialized"
                                               : "Successfully Initialized"));
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
                                                    determineVelocity == InitFlag::NOT_CONNECTED
                                                        ? ImColor(255.0F, 255.0F, 0.0F)
                                                        : (determineVelocity == InitFlag::CONNECTED
                                                               ? ImColor(255.0F, 0.0F, 0.0F)
                                                               : ImColor(0.0F, 255.0F, 0.0F)));
        ImGui::Selectable(("##determineVelocity" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", determineVelocity == InitFlag::NOT_CONNECTED
                                        ? "Not connected, so won't be initialized"
                                        : (determineVelocity == InitFlag::CONNECTED
                                               ? "To be initialized"
                                               : "Successfully Initialized"));
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
                                                    determineAttitude == InitFlag::NOT_CONNECTED
                                                        ? ImColor(255.0F, 255.0F, 0.0F)
                                                        : (determineAttitude == InitFlag::CONNECTED
                                                               ? ImColor(255.0F, 0.0F, 0.0F)
                                                               : ImColor(0.0F, 255.0F, 0.0F)));
        ImGui::Selectable(("##determineAttitude" + std::to_string(size_t(id))).c_str(),
                          false, ImGuiSelectableFlags_Disabled, ImVec2(1.6F * size, 0.0F));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", determineAttitude == InitFlag::NOT_CONNECTED
                                        ? "Not connected, so won't be initialized"
                                        : (determineAttitude == InitFlag::CONNECTED
                                               ? "To be initialized"
                                               : "Successfully Initialized"));
        }
        ImGui::TableNextColumn();

        ImGui::EndTable();
    }

    if (ImGui::BeginTable(("Overrides##" + std::to_string(size_t(id))).c_str(),
                          2, ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
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
    j["positionAccuracyThreshold"] = positionAccuracyThreshold;
    j["velocityAccuracyThreshold"] = velocityAccuracyThreshold;
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
    if (j.contains("positionAccuracyThreshold"))
    {
        j.at("positionAccuracyThreshold").get_to(positionAccuracyThreshold);
    }
    if (j.contains("velocityAccuracyThreshold"))
    {
        j.at("velocityAccuracyThreshold").get_to(velocityAccuracyThreshold);
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

bool NAV::PosVelAttInitializer::onCreateLink(Pin* startPin, Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    bool canConnect = false;
    if (startPin && endPin)
    {
        if (endPin->parentNode->id != id) // Link on Output Port
        {
            return true;
        }
        size_t endPinIndex = pinIndexFromId(endPin->id);

        int64_t rows = 3;
        int64_t cols = 1;

        if (endPinIndex == InputPortIndex_Attitude)
        {
            rows = 4;
        }

        if (endPinIndex == InputPortIndex_Position
            || endPinIndex == InputPortIndex_Velocity
            || endPinIndex == InputPortIndex_Attitude)
        {
            if (startPin->dataIdentifier.front() == "Eigen::MatrixXd")
            {
                if (const auto* pval = std::get_if<void*>(&startPin->data))
                {
                    if (auto* mat = static_cast<Eigen::MatrixXd*>(*pval))
                    {
                        if (mat->rows() == rows && mat->cols() == cols)
                        {
                            canConnect = true;
                        }
                        else
                        {
                            LOG_ERROR("{}: The Matrix needs to have the size {}x{}", nameId(), rows, cols);
                        }
                    }
                }
            }
            else if (startPin->dataIdentifier.front() == "BlockMatrix")
            {
                if (const auto* pval = std::get_if<void*>(&startPin->data))
                {
                    if (auto* block = static_cast<BlockMatrix*>(*pval))
                    {
                        auto mat = (*block)();
                        if (mat.rows() == rows && mat.cols() == cols)
                        {
                            canConnect = true;
                        }
                        else
                        {
                            LOG_ERROR("{}: The Matrix needs to have the size {}x{}", nameId(), rows, cols);
                        }
                    }
                }
            }
        }
        else
        {
            canConnect = true;
            if (endPinIndex == InputPortIndex_ImuObs)
            {
                outputPins.at(OutputPortIndex_ImuObs).dataIdentifier = startPin->dataIdentifier;
            }
            else if (endPinIndex == InputPortIndex_GnssObs)
            {
                outputPins.at(OutputPortIndex_GnssObs).dataIdentifier = startPin->dataIdentifier;
            }
        }
    }

    if (canConnect)
    {
        if (endPin->id == inputPins.at(InputPortIndex_Position).id)
        {
            determinePosition = InitFlag::CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Velocity).id)
        {
            determineVelocity = InitFlag::CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Attitude).id)
        {
            determineAttitude = InitFlag::CONNECTED;
        }
    }

    return canConnect;
}

void NAV::PosVelAttInitializer::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    if (endPin)
    {
        if (endPin->id == inputPins.at(InputPortIndex_Position).id)
        {
            determinePosition = InitFlag::NOT_CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Velocity).id)
        {
            determineVelocity = InitFlag::NOT_CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Attitude).id)
        {
            determineAttitude = InitFlag::NOT_CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_ImuObs).id)
        {
            outputPins.at(OutputPortIndex_ImuObs).dataIdentifier.clear();
            auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_ImuObs).id);
            for (auto& connectedLink : connectedLinks)
            {
                nm::DeleteLink(connectedLink->id);
            }
        }
        else if (endPin->id == inputPins.at(InputPortIndex_GnssObs).id)
        {
            outputPins.at(OutputPortIndex_GnssObs).dataIdentifier.clear();
            auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_GnssObs).id);
            for (auto& connectedLink : connectedLinks)
            {
                nm::DeleteLink(connectedLink->id);
            }
        }
        deinitializeNode();
    }
}

bool NAV::PosVelAttInitializer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    startTime = 0;

    countAveragedAttitude = 0.0;

    if (determinePosition == InitFlag::INITIALIZED)
    {
        determinePosition = InitFlag::CONNECTED;
    }
    if (determineVelocity == InitFlag::INITIALIZED)
    {
        determineVelocity = InitFlag::CONNECTED;
    }
    if (determineAttitude == InitFlag::INITIALIZED)
    {
        determineAttitude = InitFlag::CONNECTED;
    }

    lastPositionAccuracy = { std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity() };
    lastVelocityAccuracy = { std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity() };

    return true;
}

void NAV::PosVelAttInitializer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::PosVelAttInitializer::finalizeInit()
{
    LOG_TRACE("{}: called", nameId());

    if (determinePosition != InitFlag::CONNECTED
        && determineVelocity != InitFlag::CONNECTED
        && determineAttitude != InitFlag::CONNECTED)
    {
        if (determinePosition == InitFlag::INITIALIZED)
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
                    {
                        [[maybe_unused]] auto latLonAlt = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix->data(), 3));
                        LOG_INFO("{}: Position initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", nameId(),
                                 trafo::rad2deg(latLonAlt.x()),
                                 trafo::rad2deg(latLonAlt.y()),
                                 latLonAlt.z());
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Position))
                    {
                        auto matrix = (*value)();

                        [[maybe_unused]] auto latLonAlt = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data(), 3));
                        LOG_INFO("{}: Position initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", nameId(),
                                 trafo::rad2deg(latLonAlt.x()), trafo::rad2deg(latLonAlt.y()), latLonAlt.z());
                    }
                }
                notifyInputValueChanged(InputPortIndex_Position);
            }
        }
        if (determineVelocity == InitFlag::INITIALIZED)
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Velocity).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if ([[maybe_unused]] auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Velocity))
                    {
                        LOG_INFO("{}: State initialized to v_N {:3.5f} [m/s], v_E {:3.5f} [m/s], v_D {:3.5f} [m/s]", nameId(),
                                 (*matrix)(0, 0), (*matrix)(1, 0), (*matrix)(2, 0));
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Velocity))
                    {
                        [[maybe_unused]] auto matrix = (*value)();
                        LOG_INFO("{}: State initialized to v_N {:3.5f} [m/s], v_E {:3.5f} [m/s], v_D {:3.5f} [m/s]", nameId(),
                                 matrix(0, 0), matrix(1, 0), matrix(2, 0));
                    }
                }
                notifyInputValueChanged(InputPortIndex_Velocity);
            }
        }
        if (determineAttitude == InitFlag::INITIALIZED)
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Attitude).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Attitude))
                    {
                        auto quat_nb = Eigen::Quaterniond((*matrix)(0, 0), (*matrix)(1, 0), (*matrix)(2, 0), (*matrix)(3, 0));
                        [[maybe_unused]] auto rollPitchYaw = trafo::quat2eulerZYX(quat_nb);
                        LOG_INFO("{}: State initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", nameId(),
                                 trafo::rad2deg(rollPitchYaw.x()),
                                 trafo::rad2deg(rollPitchYaw.y()),
                                 trafo::rad2deg(rollPitchYaw.z()));
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Attitude))
                    {
                        auto matrix = (*value)();
                        auto quat_nb = Eigen::Quaterniond(matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0));
                        [[maybe_unused]] auto rollPitchYaw = trafo::quat2eulerZYX(quat_nb);
                        LOG_INFO("{}: State initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", nameId(),
                                 trafo::rad2deg(rollPitchYaw.x()),
                                 trafo::rad2deg(rollPitchYaw.y()),
                                 trafo::rad2deg(rollPitchYaw.z()));
                    }
                }
                notifyInputValueChanged(InputPortIndex_Attitude);
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    if (determinePosition != InitFlag::CONNECTED
        && determineVelocity != InitFlag::CONNECTED
        && determineAttitude != InitFlag::CONNECTED)
    {
        invokeCallbacks(OutputPortIndex_ImuObs, nodeData);
        return;
    }

    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

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

    const Eigen::Vector3d magUncomp_b = imuPosition.quatMag_bp() * obs->magUncompXYZ.value();
    auto magneticHeading = -std::atan2(magUncomp_b.y(), magUncomp_b.x());

    const Eigen::Vector3d accelUncomp_b = imuPosition.quatAccel_bp() * obs->accelUncompXYZ.value() * -1;
    auto roll = rollFromStaticAccelerationObs(accelUncomp_b);
    auto pitch = pitchFromStaticAccelerationObs(accelUncomp_b);

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

    if (static_cast<double>(obs->timeSinceStartup.value() - startTime) * 1e-9 >= initDuration
        || (determineAttitude == InitFlag::CONNECTED
            && overrideRollPitchYaw.at(0) && overrideRollPitchYaw.at(1) && overrideRollPitchYaw.at(0)))
    {
        if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Attitude).id))
        {
            auto quat_nb = trafo::quat_nb(overrideRollPitchYaw.at(0) ? trafo::deg2rad(overrideValuesRollPitchYaw.at(0)) : averagedAttitude.at(0),
                                          overrideRollPitchYaw.at(1) ? trafo::deg2rad(overrideValuesRollPitchYaw.at(1)) : averagedAttitude.at(1),
                                          overrideRollPitchYaw.at(2) ? trafo::deg2rad(overrideValuesRollPitchYaw.at(2)) : averagedAttitude.at(2));
            if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
            {
                if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Attitude))
                {
                    (*matrix)(0, 0) = quat_nb.coeffs().w();
                    (*matrix)(1, 0) = quat_nb.coeffs().x();
                    (*matrix)(2, 0) = quat_nb.coeffs().y();
                    (*matrix)(3, 0) = quat_nb.coeffs().z();
                    determineAttitude = InitFlag::INITIALIZED;
                    finalizeInit();
                }
            }
            else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
            {
                if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Attitude))
                {
                    auto matrix = (*value)();
                    matrix(0, 0) = quat_nb.coeffs().w();
                    matrix(1, 0) = quat_nb.coeffs().x();
                    matrix(2, 0) = quat_nb.coeffs().y();
                    matrix(3, 0) = quat_nb.coeffs().z();
                    determineAttitude = InitFlag::INITIALIZED;
                    finalizeInit();
                }
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveGnssObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (determinePosition != InitFlag::CONNECTED
        && determineVelocity != InitFlag::CONNECTED
        && determineAttitude != InitFlag::CONNECTED)
    {
        invokeCallbacks(OutputPortIndex_GnssObs, nodeData);
        return;
    }

    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                receiveRtklibPosObs(std::static_pointer_cast<RtklibPosObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                receiveUbloxObs(std::static_pointer_cast<UbloxObs>(nodeData));
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveUbloxObs(const std::shared_ptr<UbloxObs>& obs)
{
    if (obs->msgClass == sensors::ublox::UbxClass::UBX_CLASS_NAV)
    {
        auto msgId = static_cast<sensors::ublox::UbxNavMessages>(obs->msgId);
        if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_ATT)
        // && determineAttitude != InitFlag::NOT_CONNECTED)
        {
            // LOG_DATA("{}: UBX_NAV_ATT: Roll {}, Pitch {}, Heading {} [deg]", nameId(),
            //           std::get<sensors::ublox::UbxNavAtt>(obs->data).roll * 1e-5,
            //           std::get<sensors::ublox::UbxNavAtt>(obs->data).pitch * 1e-5,
            //           std::get<sensors::ublox::UbxNavAtt>(obs->data).heading * 1e-5);
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSECEF
                 && determinePosition != InitFlag::NOT_CONNECTED)
        {
            lastPositionAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);
            lastPositionAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);
            lastPositionAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavPosecef>(obs->data).pAcc);

            if (lastPositionAccuracy.at(0) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(1) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(2) <= positionAccuracyThreshold)
            {
                if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
                {
                    Eigen::Vector3d position_ecef(std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                                                  std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                                                  std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);

                    if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                    {
                        if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
                        {
                            (*matrix)(0, 0) = position_ecef.x();
                            (*matrix)(1, 0) = position_ecef.y();
                            (*matrix)(2, 0) = position_ecef.z();
                            determinePosition = InitFlag::INITIALIZED;
                            finalizeInit();
                        }
                    }
                    else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                    {
                        if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Position))
                        {
                            auto matrix = (*value)();
                            matrix(0, 0) = position_ecef.x();
                            matrix(1, 0) = position_ecef.y();
                            matrix(2, 0) = position_ecef.z();
                            determinePosition = InitFlag::INITIALIZED;
                            finalizeInit();
                        }
                    }
                }
            }
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSLLH
                 && determinePosition != InitFlag::NOT_CONNECTED)
        {
            lastPositionAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            lastPositionAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).hAcc * 1e-1);
            lastPositionAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavPosllh>(obs->data).vAcc * 1e-1);

            if (lastPositionAccuracy.at(0) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(1) <= positionAccuracyThreshold
                && lastPositionAccuracy.at(2) <= positionAccuracyThreshold)
            {
                if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
                {
                    Eigen::Vector3d latLonAlt(trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                              trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                              std::get<sensors::ublox::UbxNavPosllh>(obs->data).height * 1e-3);

                    auto position_ecef = trafo::lla2ecef_WGS84(latLonAlt);

                    if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                    {
                        if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
                        {
                            (*matrix)(0, 0) = position_ecef.x();
                            (*matrix)(1, 0) = position_ecef.y();
                            (*matrix)(2, 0) = position_ecef.z();
                            determinePosition = InitFlag::INITIALIZED;
                            finalizeInit();
                        }
                    }
                    else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                    {
                        if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Position))
                        {
                            auto matrix = (*value)();
                            matrix(0, 0) = position_ecef.x();
                            matrix(1, 0) = position_ecef.y();
                            matrix(2, 0) = position_ecef.z();
                            determinePosition = InitFlag::INITIALIZED;
                            finalizeInit();
                        }
                    }
                }
            }
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_VELNED
                 && determineVelocity != InitFlag::NOT_CONNECTED)
        {
            lastVelocityAccuracy.at(0) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);
            lastVelocityAccuracy.at(1) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);
            lastVelocityAccuracy.at(2) = static_cast<float>(std::get<sensors::ublox::UbxNavVelned>(obs->data).sAcc);

            if (lastVelocityAccuracy.at(0) <= velocityAccuracyThreshold
                && lastVelocityAccuracy.at(1) <= velocityAccuracyThreshold
                && lastVelocityAccuracy.at(2) <= velocityAccuracyThreshold)
            {
                if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Velocity).id))
                {
                    Eigen::Vector3d velocity_n(std::get<sensors::ublox::UbxNavVelned>(obs->data).velN * 1e-2,
                                               std::get<sensors::ublox::UbxNavVelned>(obs->data).velE * 1e-2,
                                               std::get<sensors::ublox::UbxNavVelned>(obs->data).velD * 1e-2);

                    LOG_DATA("{}: UBX_NAV_VELNED: {}, {}, {} [m/s], {} [deg]", nameId(),
                             velocity_n.x(), velocity_n.y(), velocity_n.z(),
                             std::get<sensors::ublox::UbxNavVelned>(obs->data).heading * 1e-5);

                    if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                    {
                        if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Velocity))
                        {
                            (*matrix)(0, 0) = velocity_n.x();
                            (*matrix)(1, 0) = velocity_n.y();
                            (*matrix)(2, 0) = velocity_n.z();
                            determineVelocity = InitFlag::INITIALIZED;
                            finalizeInit();
                        }
                    }
                    else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                    {
                        if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Velocity))
                        {
                            auto matrix = (*value)();
                            matrix(0, 0) = velocity_n.x();
                            matrix(1, 0) = velocity_n.y();
                            matrix(2, 0) = velocity_n.z();
                            determineVelocity = InitFlag::INITIALIZED;
                            finalizeInit();
                        }
                    }
                }
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveRtklibPosObs(const std::shared_ptr<RtklibPosObs>& obs)
{
    if (determinePosition != InitFlag::NOT_CONNECTED && obs->position_ecef.has_value())
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
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
                    {
                        (*matrix)(0, 0) = obs->position_ecef->x();
                        (*matrix)(1, 0) = obs->position_ecef->y();
                        (*matrix)(2, 0) = obs->position_ecef->z();
                        determinePosition = InitFlag::INITIALIZED;
                        finalizeInit();
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Position))
                    {
                        auto matrix = (*value)();
                        matrix(0, 0) = obs->position_ecef->x();
                        matrix(1, 0) = obs->position_ecef->y();
                        matrix(2, 0) = obs->position_ecef->z();
                        determinePosition = InitFlag::INITIALIZED;
                        finalizeInit();
                    }
                }
            }
        }
    }
}
