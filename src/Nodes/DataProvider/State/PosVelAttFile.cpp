// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PosVelAttFile.hpp"

#include "util/Logger.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/PosVelAtt.hpp"

NAV::PosVelAttFile::PosVelAttFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 488, 248 };

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &PosVelAttFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, { "std::vector<std::string>" }, &_headerColumns);
}

NAV::PosVelAttFile::~PosVelAttFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PosVelAttFile::typeStatic()
{
    return "PosVelAttFile";
}

std::string NAV::PosVelAttFile::type() const
{
    return typeStatic();
}

std::string NAV::PosVelAttFile::category()
{
    return "Data Provider";
}

void NAV::PosVelAttFile::guiConfig()
{
    if (auto res = FileReader::guiConfig(".csv,.*", { ".csv" }, size_t(id), nameId()))
    {
        LOG_DEBUG("{}: Path changed to {}", nameId(), _path);
        flow::ApplyChanges();
        if (res == FileReader::PATH_CHANGED)
        {
            doReinitialize();
        }
        else
        {
            doDeinitialize();
        }
    }

    // Header info
    if (ImGui::BeginTable(fmt::format("##PvaHeaders ({})", id.AsPointer()).c_str(), 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Velocity", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Attitude", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        auto TextColoredIfExists = [this](int index, const char* text) {
            ImGui::TableSetColumnIndex(index);
            if (std::find(_headerColumns.begin(), _headerColumns.end(), text) != _headerColumns.end())
            {
                ImGui::TextUnformatted(text);
            }
            else
            {
                ImGui::TextDisabled("%s", text);
            }
        };

        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsCycle");
        TextColoredIfExists(1, "Pos ECEF X [m]");
        TextColoredIfExists(2, "Vel ECEF X [m/s]");
        TextColoredIfExists(3, "n_Quat_b w");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsWeek");
        TextColoredIfExists(1, "Pos ECEF Y [m]");
        TextColoredIfExists(2, "Vel ECEF Y [m/s]");
        TextColoredIfExists(3, "n_Quat_b x");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsToW [s]");
        TextColoredIfExists(1, "Pos ECEF Z [m]");
        TextColoredIfExists(2, "Vel ECEF Z [m/s]");
        TextColoredIfExists(3, "n_Quat_b y");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Latitude [deg]");
        TextColoredIfExists(2, "Vel N [m/s]");
        TextColoredIfExists(3, "n_Quat_b z");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Longitude [deg]");
        TextColoredIfExists(2, "Vel E [m/s]");
        TextColoredIfExists(3, "Roll [deg]");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Altitude [m]");
        TextColoredIfExists(2, "Vel D [m/s]");
        TextColoredIfExists(3, "Pitch [deg]");
        ImGui::TableNextRow();
        TextColoredIfExists(3, "Yaw [deg]");

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::PosVelAttFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::PosVelAttFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::PosVelAttFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::PosVelAttFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::PosVelAttFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::PosVelAttFile::pollData()
{
    auto obs = std::make_shared<PosVelAtt>();

    // Read line
    std::string line;
    getline(line);
    // Remove any starting non text characters
    line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

    if (line.empty())
    {
        return nullptr;
    }

    // Convert line into stream
    std::stringstream lineStream(line);
    std::string cell;

    std::optional<uint16_t> gpsCycle = 0;
    std::optional<uint16_t> gpsWeek;
    std::optional<long double> gpsToW;
    std::optional<double> e_position_x;
    std::optional<double> e_position_y;
    std::optional<double> e_position_z;
    std::optional<double> lla_position_x;
    std::optional<double> lla_position_y;
    std::optional<double> lla_position_z;
    std::optional<double> e_velocity_x;
    std::optional<double> e_velocity_y;
    std::optional<double> e_velocity_z;
    std::optional<double> n_velocity_x;
    std::optional<double> n_velocity_y;
    std::optional<double> n_velocity_z;
    std::optional<double> n_Quat_b_w;
    std::optional<double> n_Quat_b_x;
    std::optional<double> n_Quat_b_y;
    std::optional<double> n_Quat_b_z;
    std::optional<double> roll;
    std::optional<double> pitch;
    std::optional<double> yaw;

    // Split line at comma
    for (const auto& column : _headerColumns)
    {
        if (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
            if (cell.empty())
            {
                continue;
            }

            if (column == "GpsCycle") { gpsCycle = static_cast<uint16_t>(std::stoul(cell)); }
            else if (column == "GpsWeek") { gpsWeek = static_cast<uint16_t>(std::stoul(cell)); }
            else if (column == "GpsToW [s]") { gpsToW = std::stold(cell); }
            else if (column == "Pos ECEF X [m]") { e_position_x = std::stod(cell); }
            else if (column == "Pos ECEF Y [m]") { e_position_y = std::stod(cell); }
            else if (column == "Pos ECEF Z [m]") { e_position_z = std::stod(cell); }
            else if (column == "Latitude [deg]") { lla_position_x = deg2rad(std::stod(cell)); }
            else if (column == "Longitude [deg]") { lla_position_y = deg2rad(std::stod(cell)); }
            else if (column == "Altitude [m]") { lla_position_z = std::stod(cell); }
            else if (column == "Vel ECEF X [m/s]") { e_velocity_x = std::stod(cell); }
            else if (column == "Vel ECEF Y [m/s]") { e_velocity_y = std::stod(cell); }
            else if (column == "Vel ECEF Z [m/s]") { e_velocity_z = std::stod(cell); }
            else if (column == "Vel N [m/s]") { n_velocity_x = std::stod(cell); }
            else if (column == "Vel E [m/s]") { n_velocity_y = std::stod(cell); }
            else if (column == "Vel D [m/s]") { n_velocity_z = std::stod(cell); }
            else if (column == "n_Quat_b w") { n_Quat_b_w = std::stod(cell); }
            else if (column == "n_Quat_b x") { n_Quat_b_x = std::stod(cell); }
            else if (column == "n_Quat_b y") { n_Quat_b_y = std::stod(cell); }
            else if (column == "n_Quat_b z") { n_Quat_b_z = std::stod(cell); }
            else if (column == "Roll [deg]") { roll = deg2rad(std::stod(cell)); }
            else if (column == "Pitch [deg]") { pitch = deg2rad(std::stod(cell)); }
            else if (column == "Yaw [deg]") { yaw = deg2rad(std::stod(cell)); }
        }
    }

    if (gpsCycle.has_value() && gpsWeek.has_value() && gpsToW.has_value())
    {
        obs->insTime = InsTime(gpsCycle.value(), gpsWeek.value(), gpsToW.value());
    }
    else
    {
        LOG_WARN("{}: A PosVelAtt File needs a time.", nameId());
        return nullptr;
    }

    if (e_position_x.has_value() && e_position_y.has_value() && e_position_z.has_value())
    {
        obs->setPosition_e({ e_position_x.value(), e_position_y.value(), e_position_z.value() });
    }
    else if (lla_position_x.has_value() && lla_position_y.has_value() && lla_position_z.has_value())
    {
        obs->setPosition_lla({ lla_position_x.value(), lla_position_y.value(), lla_position_z.value() });
    }
    else
    {
        LOG_WARN("{}: A PosVelAtt File needs a position.", nameId());
        return nullptr;
    }

    if (e_velocity_x.has_value() && e_velocity_y.has_value() && e_velocity_z.has_value())
    {
        obs->setVelocity_e(Eigen::Vector3d{ e_velocity_x.value(), e_velocity_y.value(), e_velocity_z.value() });
    }
    else if (n_velocity_x.has_value() && n_velocity_y.has_value() && n_velocity_z.has_value())
    {
        obs->setVelocity_n(Eigen::Vector3d{ n_velocity_x.value(), n_velocity_y.value(), n_velocity_z.value() });
    }

    if (n_Quat_b_w.has_value() && n_Quat_b_x.has_value() && n_Quat_b_y.has_value() && n_Quat_b_z.has_value())
    {
        obs->setAttitude_n_Quat_b(Eigen::Quaterniond{ n_Quat_b_w.value(), n_Quat_b_x.value(), n_Quat_b_y.value(), n_Quat_b_z.value() });
    }
    else if (roll.has_value() && pitch.has_value() && yaw.has_value())
    {
        obs->setAttitude_n_Quat_b(trafo::n_Quat_b(roll.value(), pitch.value(), yaw.value()));
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_PVA, obs);
    return obs;
}