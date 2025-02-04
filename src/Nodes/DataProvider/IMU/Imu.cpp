// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Imu.hpp"

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
        std::array<float, 3> imuPosAccel = { static_cast<float>(_imuPos.b_positionAccel().x()), static_cast<float>(_imuPos.b_positionAccel().y()), static_cast<float>(_imuPos.b_positionAccel().z()) };
        if (ImGui::InputFloat3(fmt::format("Lever Accel [m]##{}", size_t(id)).c_str(), imuPosAccel.data()))
        {
            flow::ApplyChanges();
            _imuPos._b_positionAccel = Eigen::Vector3d(imuPosAccel.at(0), imuPosAccel.at(1), imuPosAccel.at(2));
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Position of the accelerometer sensor relative to the vehicle center of mass in the body coordinate frame.");

        std::array<float, 3> imuPosGyro = { static_cast<float>(_imuPos.b_positionGyro().x()), static_cast<float>(_imuPos.b_positionGyro().y()), static_cast<float>(_imuPos.b_positionGyro().z()) };
        if (ImGui::InputFloat3(fmt::format("Lever Gyro [m]##{}", size_t(id)).c_str(), imuPosGyro.data()))
        {
            flow::ApplyChanges();
            _imuPos._b_positionGyro = Eigen::Vector3d(imuPosGyro.at(0), imuPosGyro.at(1), imuPosGyro.at(2));
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Position of the gyroscope sensor relative to the vehicle center of mass in the body coordinate frame.");

        std::array<float, 3> imuPosMag = { static_cast<float>(_imuPos.b_positionMag().x()), static_cast<float>(_imuPos.b_positionMag().y()), static_cast<float>(_imuPos.b_positionMag().z()) };
        if (ImGui::InputFloat3(fmt::format("Lever Mag [m]##{}", size_t(id)).c_str(), imuPosMag.data()))
        {
            flow::ApplyChanges();
            _imuPos._b_positionMag = Eigen::Vector3d(imuPosMag.at(0), imuPosMag.at(1), imuPosMag.at(2));
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Position of the magnetometer sensor relative to the vehicle center of mass in the body coordinate frame.");

        Eigen::Vector3d eulerAccel = rad2deg(trafo::quat2eulerZYX(_imuPos.p_quatAccel_b()));
        std::array<float, 3> imuRotAccel = { static_cast<float>(eulerAccel.x()), static_cast<float>(eulerAccel.y()), static_cast<float>(eulerAccel.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation Accel [deg]##{}", size_t(id)).c_str(), imuRotAccel.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            imuRotAccel.at(0) = std::max(imuRotAccel.at(0), -179.9999F);
            imuRotAccel.at(0) = std::min(imuRotAccel.at(0), 180.0F);
            imuRotAccel.at(1) = std::max(imuRotAccel.at(1), -89.9999F);
            imuRotAccel.at(1) = std::min(imuRotAccel.at(1), 90.0F);
            imuRotAccel.at(2) = std::max(imuRotAccel.at(2), -179.9999F);
            imuRotAccel.at(2) = std::min(imuRotAccel.at(2), 180.0F);

            flow::ApplyChanges();
            _imuPos._b_quatAccel_p = trafo::b_Quat_p(deg2rad(imuRotAccel.at(0)), deg2rad(imuRotAccel.at(1)), deg2rad(imuRotAccel.at(2)));
        }
        ImGui::SameLine();
        TrafoHelperMarker(_imuPos.b_quatAccel_p());

        Eigen::Vector3d eulerGyro = rad2deg(trafo::quat2eulerZYX(_imuPos.p_quatGyro_b()));
        std::array<float, 3> imuRotGyro = { static_cast<float>(eulerGyro.x()), static_cast<float>(eulerGyro.y()), static_cast<float>(eulerGyro.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation Gyro [deg]##{}", size_t(id)).c_str(), imuRotGyro.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            imuRotGyro.at(0) = std::max(imuRotGyro.at(0), -179.9999F);
            imuRotGyro.at(0) = std::min(imuRotGyro.at(0), 180.0F);
            imuRotGyro.at(1) = std::max(imuRotGyro.at(1), -89.9999F);
            imuRotGyro.at(1) = std::min(imuRotGyro.at(1), 90.0F);
            imuRotGyro.at(2) = std::max(imuRotGyro.at(2), -179.9999F);
            imuRotGyro.at(2) = std::min(imuRotGyro.at(2), 180.0F);

            flow::ApplyChanges();
            _imuPos._b_quatGyro_p = trafo::b_Quat_p(deg2rad(imuRotGyro.at(0)), deg2rad(imuRotGyro.at(1)), deg2rad(imuRotGyro.at(2)));
        }
        ImGui::SameLine();
        TrafoHelperMarker(_imuPos.b_quatGyro_p());

        Eigen::Vector3d eulerMag = rad2deg(trafo::quat2eulerZYX(_imuPos.p_quatMag_b()));
        std::array<float, 3> imuRotMag = { static_cast<float>(eulerMag.x()), static_cast<float>(eulerMag.y()), static_cast<float>(eulerMag.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation Mag [deg]##{}", size_t(id)).c_str(), imuRotMag.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            imuRotMag.at(0) = std::max(imuRotMag.at(0), -179.9999F);
            imuRotMag.at(0) = std::min(imuRotMag.at(0), 180.0F);
            imuRotMag.at(1) = std::max(imuRotMag.at(1), -89.9999F);
            imuRotMag.at(1) = std::min(imuRotMag.at(1), 90.0F);
            imuRotMag.at(2) = std::max(imuRotMag.at(2), -179.9999F);
            imuRotMag.at(2) = std::min(imuRotMag.at(2), 180.0F);

            flow::ApplyChanges();
            _imuPos._b_quatMag_p = trafo::b_Quat_p(deg2rad(imuRotMag.at(0)), deg2rad(imuRotMag.at(1)), deg2rad(imuRotMag.at(2)));
        }
        ImGui::SameLine();
        TrafoHelperMarker(_imuPos.b_quatMag_p());

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