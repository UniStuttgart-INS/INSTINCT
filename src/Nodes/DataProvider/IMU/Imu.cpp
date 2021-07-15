#include "Imu.hpp"

#include "internal/FlowManager.hpp"

#include "util/InsTransformations.hpp"

void NAV::Imu::guiConfig()
{
    ImGui::SetNextItemOpen(false, ImGuiCond_Appearing);
    if (ImGui::TreeNode(fmt::format("Imu Position & Rotation##{}", size_t(id)).c_str()))
    {
        std::array<float, 3> imuPosAccel = { static_cast<float>(imuPos.positionAccel_b.x()), static_cast<float>(imuPos.positionAccel_b.y()), static_cast<float>(imuPos.positionAccel_b.z()) };
        if (ImGui::InputFloat3(fmt::format("Lever Accel [m]##{}", size_t(id)).c_str(), imuPosAccel.data()))
        {
            flow::ApplyChanges();
            imuPos.positionAccel_b = Eigen::Vector3d(imuPosAccel.at(0), imuPosAccel.at(1), imuPosAccel.at(2));
        }
        std::array<float, 3> imuPosGyro = { static_cast<float>(imuPos.positionGyro_b.x()), static_cast<float>(imuPos.positionGyro_b.y()), static_cast<float>(imuPos.positionGyro_b.z()) };
        if (ImGui::InputFloat3(fmt::format("Lever Gyro [m]##{}", size_t(id)).c_str(), imuPosGyro.data()))
        {
            flow::ApplyChanges();
            imuPos.positionGyro_b = Eigen::Vector3d(imuPosGyro.at(0), imuPosGyro.at(1), imuPosGyro.at(2));
        }
        std::array<float, 3> imuPosMag = { static_cast<float>(imuPos.positionMag_b.x()), static_cast<float>(imuPos.positionMag_b.y()), static_cast<float>(imuPos.positionMag_b.z()) };
        if (ImGui::InputFloat3(fmt::format("Lever Mag [m]##{}", size_t(id)).c_str(), imuPosMag.data()))
        {
            flow::ApplyChanges();
            imuPos.positionMag_b = Eigen::Vector3d(imuPosMag.at(0), imuPosMag.at(1), imuPosMag.at(2));
        }

        Eigen::Vector3d eulerAccel = trafo::rad2deg3(trafo::quat2eulerZYX(imuPos.quaternionAccel_bp));
        std::array<float, 3> imuRotAccel = { static_cast<float>(eulerAccel.x()), static_cast<float>(eulerAccel.y()), static_cast<float>(eulerAccel.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation Accel [deg]##{}", size_t(id)).c_str(), imuRotAccel.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            if (imuRotAccel.at(0) < -179.9999F)
            {
                imuRotAccel.at(0) = -179.9999F;
            }
            if (imuRotAccel.at(0) > 180)
            {
                imuRotAccel.at(0) = 180;
            }
            if (imuRotAccel.at(1) < -89.9999F)
            {
                imuRotAccel.at(1) = -89.9999F;
            }
            if (imuRotAccel.at(1) > 90)
            {
                imuRotAccel.at(1) = 90;
            }
            if (imuRotAccel.at(2) < -179.9999F)
            {
                imuRotAccel.at(2) = -179.9999F;
            }
            if (imuRotAccel.at(2) > 180)
            {
                imuRotAccel.at(2) = 180;
            }

            flow::ApplyChanges();
            imuPos.quaternionAccel_bp = trafo::quat_bp(trafo::deg2rad(imuRotAccel.at(0)), trafo::deg2rad(imuRotAccel.at(1)), trafo::deg2rad(imuRotAccel.at(2)));
        }

        Eigen::Vector3d eulerGyro = trafo::rad2deg3(trafo::quat2eulerZYX(imuPos.quaternionGyro_bp));
        std::array<float, 3> imuRotGyro = { static_cast<float>(eulerGyro.x()), static_cast<float>(eulerGyro.y()), static_cast<float>(eulerGyro.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation Gyro [deg]##{}", size_t(id)).c_str(), imuRotGyro.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            if (imuRotGyro.at(0) < -179.9999F)
            {
                imuRotGyro.at(0) = -179.9999F;
            }
            if (imuRotGyro.at(0) > 180)
            {
                imuRotGyro.at(0) = 180;
            }
            if (imuRotGyro.at(1) < -89.9999F)
            {
                imuRotGyro.at(1) = -89.9999F;
            }
            if (imuRotGyro.at(1) > 90)
            {
                imuRotGyro.at(1) = 90;
            }
            if (imuRotGyro.at(2) < -179.9999F)
            {
                imuRotGyro.at(2) = -179.9999F;
            }
            if (imuRotGyro.at(2) > 180)
            {
                imuRotGyro.at(2) = 180;
            }

            flow::ApplyChanges();
            imuPos.quaternionGyro_bp = trafo::quat_bp(trafo::deg2rad(imuRotGyro.at(0)), trafo::deg2rad(imuRotGyro.at(1)), trafo::deg2rad(imuRotGyro.at(2)));
        }

        Eigen::Vector3d eulerMag = trafo::rad2deg3(trafo::quat2eulerZYX(imuPos.quaternionMag_bp));
        std::array<float, 3> imuRotMag = { static_cast<float>(eulerMag.x()), static_cast<float>(eulerMag.y()), static_cast<float>(eulerMag.z()) };
        if (ImGui::InputFloat3(fmt::format("Rotation Mag [deg]##{}", size_t(id)).c_str(), imuRotMag.data()))
        {
            // (-180:180] x (-90:90] x (-180:180]
            if (imuRotMag.at(0) < -179.9999F)
            {
                imuRotMag.at(0) = -179.9999F;
            }
            if (imuRotMag.at(0) > 180)
            {
                imuRotMag.at(0) = 180;
            }
            if (imuRotMag.at(1) < -89.9999F)
            {
                imuRotMag.at(1) = -89.9999F;
            }
            if (imuRotMag.at(1) > 90)
            {
                imuRotMag.at(1) = 90;
            }
            if (imuRotMag.at(2) < -179.9999F)
            {
                imuRotMag.at(2) = -179.9999F;
            }
            if (imuRotMag.at(2) > 180)
            {
                imuRotMag.at(2) = 180;
            }

            flow::ApplyChanges();
            imuPos.quaternionMag_bp = trafo::quat_bp(trafo::deg2rad(imuRotMag.at(0)), trafo::deg2rad(imuRotMag.at(1)), trafo::deg2rad(imuRotMag.at(2)));
        }

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::Imu::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuPos"] = imuPos;

    return j;
}

void NAV::Imu::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuPos"))
    {
        j.at("imuPos").get_to(imuPos);
    }
}