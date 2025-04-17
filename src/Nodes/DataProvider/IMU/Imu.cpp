// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Imu.hpp"
#include <imgui.h>

#include "internal/FlowManager.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/Matrix.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace
{

void TrafoHelperMarker(const Eigen::Quaterniond& q)
{
    if (NAV::gui::widgets::BeginHelpMarker())
    {
        Eigen::Matrix3d dcm = q.toRotationMatrix();

        ImGui::BeginHorizontal("Tooltip Horizontal");

        ImGui::BeginVertical("Tooltip Vertical 1");
        ImGui::Spring();
        ImGui::TextUnformatted("(X Y Z)_b = ");
        ImGui::Spring();
        ImGui::EndVertical();

        NAV::gui::widgets::MatrixView("quaternionAccel_bp", &dcm, GuiMatrixViewFlags_None, ImGuiTableFlags_Borders | ImGuiTableFlags_NoHostExtendX, "% -.1f");

        ImGui::BeginVertical("Tooltip Vertical 2");
        ImGui::Spring();
        ImGui::TextUnformatted(" * (X Y Z)_p");
        ImGui::Spring();
        ImGui::EndVertical();

        ImGui::EndHorizontal();

        NAV::gui::widgets::EndHelpMarker();
    }
}

} // namespace

NAV::Imu::Imu(std::string name)
    : Node(std::move(name)) {}

void NAV::Imu::guiConfig()
{
    ImGui::SetNextItemOpen(false, ImGuiCond_Appearing);
    if (ImGui::TreeNode(fmt::format("Imu Position & Rotation##{}", size_t(id)).c_str()))
    {
        ImGui::BeginDisabled(); // FIXME Not properly simulated and accounted for in the algorithms
        std::array<float, 3> imuPos = { static_cast<float>(_imuPos.b_positionIMU_p().x()), static_cast<float>(_imuPos.b_positionIMU_p().y()), static_cast<float>(_imuPos.b_positionIMU_p().z()) };
        if (ImGui::InputFloat3(fmt::format("Position [m]##{}", size_t(id)).c_str(), imuPos.data()))
        {
            flow::ApplyChanges();
            _imuPos._b_positionIMU_p = Eigen::Vector3d(imuPos.at(0), imuPos.at(1), imuPos.at(2));
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        gui::widgets::HelpMarker("Position of the IMU sensor relative to the vehicle center of mass in the body coordinate frame.");

        Eigen::Vector3d eulerAngelsIMU = rad2deg(trafo::quat2eulerZYX(_imuPos.p_quat_b()));
        std::array<float, 3> imuRot = { static_cast<float>(eulerAngelsIMU.x()), static_cast<float>(eulerAngelsIMU.y()), static_cast<float>(eulerAngelsIMU.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation [deg]##{}", size_t(id)).c_str(), imuRot.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            imuRot.at(0) = std::max(imuRot.at(0), -179.9999F);
            imuRot.at(0) = std::min(imuRot.at(0), 180.0F);
            imuRot.at(1) = std::max(imuRot.at(1), -89.9999F);
            imuRot.at(1) = std::min(imuRot.at(1), 90.0F);
            imuRot.at(2) = std::max(imuRot.at(2), -179.9999F);
            imuRot.at(2) = std::min(imuRot.at(2), 180.0F);

            flow::ApplyChanges();
            _imuPos._b_quat_p = trafo::b_Quat_p(deg2rad(imuRot.at(0)), deg2rad(imuRot.at(1)), deg2rad(imuRot.at(2)));
        }
        ImGui::SameLine();
        TrafoHelperMarker(_imuPos.b_quat_p());

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::Imu::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuPos"] = _imuPos;

    return j;
}

void NAV::Imu::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuPos"))
    {
        j.at("imuPos").get_to(_imuPos);
    }
}