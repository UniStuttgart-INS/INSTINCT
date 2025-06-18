// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PosVelAttFile.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "internal/NodeManager.hpp"
#include <Eigen/src/Core/DiagonalMatrix.h>
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/PosVelAtt.hpp"

NAV::PosVelAttFile::PosVelAttFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 488, 248 };

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { Pos::type(), PosVel::type(), PosVelAtt::type() }, &PosVelAttFile::pollData);
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
            if (std::ranges::find(_headerColumns, text) != _headerColumns.end())
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
        TextColoredIfExists(1, "X-ECEF [m]");
        TextColoredIfExists(2, "X velocity ECEF [m/s]");
        TextColoredIfExists(3, "n_Quat_b w");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsWeek");
        TextColoredIfExists(1, "Y-ECEF [m]");
        TextColoredIfExists(2, "Y velocity ECEF [m/s]");
        TextColoredIfExists(3, "n_Quat_b x");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsToW [s]");
        TextColoredIfExists(1, "Z-ECEF [m]");
        TextColoredIfExists(2, "Z velocity ECEF [m/s]");
        TextColoredIfExists(3, "n_Quat_b y");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Latitude [deg]");
        TextColoredIfExists(2, "North velocity [m/s]");
        TextColoredIfExists(3, "n_Quat_b z");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Longitude [deg]");
        TextColoredIfExists(2, "East velocity [m/s]");
        TextColoredIfExists(3, "Roll [deg]");
        ImGui::TableNextRow();
        TextColoredIfExists(1, "Altitude [m]");
        TextColoredIfExists(2, "Down velocity [m/s]");
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
    j["pinType"] = outputPins[OUTPUT_PORT_INDEX_PVA].dataIdentifier;

    return j;
}

void NAV::PosVelAttFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("pinType"))
    {
        j.at("pinType").get_to(outputPins[OUTPUT_PORT_INDEX_PVA].dataIdentifier);
    }
}

bool NAV::PosVelAttFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (FileReader::initialize())
    {
        for (auto& col : _headerColumns)
        {
            str::replace(col, "GpsTow [s]", "GpsToW [s]");
        }

        auto hasCol = [&](const char* text) {
            return std::ranges::find(_headerColumns, text) != _headerColumns.end();
        };

        if (!hasCol("GpsCycle") || !hasCol("GpsWeek") || !hasCol("GpsToW [s]"))
        {
            LOG_ERROR("{}: A PosVelAtt File needs a time. Please add columns 'GpsCycle', 'GpsWeek' and 'GpsToW [s]'.", nameId());
            return false;
        }
        if (!(hasCol("X-ECEF [m]") && hasCol("Y-ECEF [m]") && hasCol("Z-ECEF [m]"))
            && !(hasCol("Latitude [deg]") && hasCol("Longitude [deg]") && hasCol("Altitude [m]")))
        {
            LOG_ERROR("{}: A PosVelAtt File needs a position. Please provide"
                      " either 'X-ECEF [m]', 'Y-ECEF [m]', 'Z-ECEF [m]'"
                      " or 'Latitude [deg]', 'Longitude [deg]', 'Altitude [m]'.",
                      nameId());
            return false;
        }
        _fileContent = FileContent::Pos;
        outputPins[OUTPUT_PORT_INDEX_PVA].dataIdentifier = std::vector{ Pos::type() };

        if ((hasCol("X velocity ECEF [m/s]") && hasCol("Y velocity ECEF [m/s]") && hasCol("Z velocity ECEF [m/s]"))
            || (hasCol("North velocity [m/s]") && hasCol("East velocity [m/s]") && hasCol("Down velocity [m/s]"))
            || (hasCol("X velocity ECEF [m/s]") && hasCol("Y velocity ECEF [m/s]") && hasCol("Z velocity ECEF [m/s]")))
        {
            _fileContent = FileContent::PosVel;
            outputPins[OUTPUT_PORT_INDEX_PVA].dataIdentifier = std::vector{ PosVel::type() };

            if ((hasCol("n_Quat_b w") && hasCol("n_Quat_b x") && hasCol("n_Quat_b y") && hasCol("n_Quat_b z"))
                || (hasCol("Roll [deg]") && hasCol("Pitch [deg]") && hasCol("Yaw [deg]")))
            {
                _fileContent = FileContent::PosVelAtt;
                outputPins[OUTPUT_PORT_INDEX_PVA].dataIdentifier = std::vector{ PosVelAtt::type() };
            }
        }

        for (auto& link : outputPins[OUTPUT_PORT_INDEX_PVA].links)
        {
            if (auto* pin = link.getConnectedPin())
            {
                outputPins[OUTPUT_PORT_INDEX_PVA].recreateLink(*pin);
            }
        }

        return true;
    }

    outputPins[OUTPUT_PORT_INDEX_PVA].dataIdentifier = { Pos::type(), PosVel::type(), PosVelAtt::type() };
    return false;
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
    std::shared_ptr<Pos> obs;
    switch (_fileContent)
    {
    case FileContent::Pos:
        obs = std::make_shared<Pos>();
        break;
    case FileContent::PosVel:
        obs = std::make_shared<PosVel>();
        break;
    case FileContent::PosVelAtt:
        obs = std::make_shared<PosVelAtt>();
        break;
    }

    // Read line
    std::string line;
    getline(line);
    // Remove any starting non text characters
    line.erase(line.begin(), std::ranges::find_if(line, [](int ch) { return std::isgraph(ch); }));

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
    std::optional<double> e_positionStdDev_x;
    std::optional<double> e_positionStdDev_y;
    std::optional<double> e_positionStdDev_z;
    std::optional<double> lla_position_x;
    std::optional<double> lla_position_y;
    std::optional<double> lla_position_z;
    std::optional<double> n_positionStdDev_n;
    std::optional<double> n_positionStdDev_e;
    std::optional<double> n_positionStdDev_d;
    std::optional<double> e_velocity_x;
    std::optional<double> e_velocity_y;
    std::optional<double> e_velocity_z;
    std::optional<double> e_velocityStdDev_x;
    std::optional<double> e_velocityStdDev_y;
    std::optional<double> e_velocityStdDev_z;
    std::optional<double> n_velocity_n;
    std::optional<double> n_velocity_e;
    std::optional<double> n_velocity_d;
    std::optional<double> n_velocityStdDev_n;
    std::optional<double> n_velocityStdDev_e;
    std::optional<double> n_velocityStdDev_d;
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
            cell.erase(std::ranges::find_if(cell, [](int ch) { return std::iscntrl(ch); }), cell.end());
            if (cell.empty())
            {
                continue;
            }

            if (column == "GpsCycle") { gpsCycle = static_cast<uint16_t>(std::stoul(cell)); }
            else if (column == "GpsWeek") { gpsWeek = static_cast<uint16_t>(std::stoul(cell)); }
            else if (column == "GpsToW [s]") { gpsToW = std::stold(cell); }

            else if (column == "X-ECEF [m]") { e_position_x = std::stod(cell); }
            else if (column == "Y-ECEF [m]") { e_position_y = std::stod(cell); }
            else if (column == "Z-ECEF [m]") { e_position_z = std::stod(cell); }
            else if (column == "X-ECEF StDev [m]") { e_positionStdDev_x = std::stod(cell); }
            else if (column == "Y-ECEF StDev [m]") { e_positionStdDev_y = std::stod(cell); }
            else if (column == "Z-ECEF StDev [m]") { e_positionStdDev_z = std::stod(cell); }

            else if (column == "Latitude [deg]") { lla_position_x = deg2rad(std::stod(cell)); }
            else if (column == "Longitude [deg]") { lla_position_y = deg2rad(std::stod(cell)); }
            else if (column == "Altitude [m]") { lla_position_z = std::stod(cell); }
            else if (column == "North StDev [m]") { n_positionStdDev_n = deg2rad(std::stod(cell)); }
            else if (column == "East StDev [m]") { n_positionStdDev_e = deg2rad(std::stod(cell)); }
            else if (column == "Down StDev [m]") { n_positionStdDev_d = std::stod(cell); }

            else if (column == "X velocity ECEF [m/s]") { e_velocity_x = std::stod(cell); }
            else if (column == "Y velocity ECEF [m/s]") { e_velocity_y = std::stod(cell); }
            else if (column == "Z velocity ECEF [m/s]") { e_velocity_z = std::stod(cell); }
            else if (column == "X velocity ECEF StdDev [m/s]") { e_velocityStdDev_x = std::stod(cell); }
            else if (column == "Y velocity ECEF StdDev [m/s]") { e_velocityStdDev_y = std::stod(cell); }
            else if (column == "Z velocity ECEF StdDev [m/s]") { e_velocityStdDev_z = std::stod(cell); }

            else if (column == "North velocity [m/s]") { n_velocity_n = std::stod(cell); }
            else if (column == "East velocity [m/s]") { n_velocity_e = std::stod(cell); }
            else if (column == "Down velocity [m/s]") { n_velocity_d = std::stod(cell); }
            else if (column == "North velocity StDev [m/s]") { n_velocityStdDev_n = std::stod(cell); }
            else if (column == "East velocity StDev [m/s]") { n_velocityStdDev_e = std::stod(cell); }
            else if (column == "Down velocity StDev [m/s]") { n_velocityStdDev_d = std::stod(cell); }

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

    if (_fileContent == FileContent::PosVel || _fileContent == FileContent::PosVelAtt)
    {
        if (e_velocity_x.has_value() && e_velocity_y.has_value() && e_velocity_z.has_value())
        {
            auto lla_pos = lla_position_x.has_value() && lla_position_y.has_value() && lla_position_z.has_value()
                               ? Eigen::Vector3d(*lla_position_x, *lla_position_y, *lla_position_z)
                               : trafo::ecef2lla_WGS84(Eigen::Vector3d(*e_position_x, *e_position_y, *e_position_z));
            auto n_vel = trafo::n_Quat_e(lla_pos.x(), lla_pos.y()) * Eigen::Vector3d(*e_velocity_x, *e_velocity_y, *e_velocity_z);
            n_velocity_n = n_vel.x();
            n_velocity_e = n_vel.y();
            n_velocity_d = n_vel.z();
        }
        else if (n_velocity_n.has_value() && n_velocity_e.has_value() && n_velocity_d.has_value())
        {
            auto lla_pos = lla_position_x.has_value() && lla_position_y.has_value() && lla_position_z.has_value()
                               ? Eigen::Vector3d(*lla_position_x, *lla_position_y, *lla_position_z)
                               : trafo::ecef2lla_WGS84(Eigen::Vector3d(*e_position_x, *e_position_y, *e_position_z));
            auto e_vel = trafo::e_Quat_n(lla_pos.x(), lla_pos.y()) * Eigen::Vector3d(*e_velocity_x, *e_velocity_y, *e_velocity_z);
            e_velocity_x = e_vel.x();
            e_velocity_y = e_vel.y();
            e_velocity_z = e_vel.z();
        }

        if (e_position_x.has_value() && e_position_y.has_value() && e_position_z.has_value()
            && e_velocity_x.has_value() && e_velocity_y.has_value() && e_velocity_z.has_value())
        {
            if (auto posVel = std::reinterpret_pointer_cast<PosVel>(obs))
            {
                if (e_positionStdDev_x.has_value() && e_positionStdDev_y.has_value() && e_positionStdDev_z.has_value()
                    && e_velocityStdDev_x.has_value() && e_velocityStdDev_y.has_value() && e_velocityStdDev_z.has_value())
                {
                    ;
                    posVel->setPosVelAndCov_e(Eigen::Vector3d{ e_position_x.value(), e_position_y.value(), e_position_z.value() },
                                              Eigen::Vector3d{ e_velocity_x.value(), e_velocity_y.value(), e_velocity_z.value() },
                                              (Eigen::Vector6d() << e_positionStdDev_x.value(), e_positionStdDev_y.value(), e_positionStdDev_z.value(),
                                               e_velocityStdDev_x.value(), e_velocityStdDev_y.value(), e_velocityStdDev_z.value())
                                                  .finished()
                                                  .asDiagonal()
                                                  .toDenseMatrix());
                }
                else
                {
                    posVel->setPosition_e(Eigen::Vector3d{ e_position_x.value(), e_position_y.value(), e_position_z.value() });
                    posVel->setVelocity_e(Eigen::Vector3d{ e_velocity_x.value(), e_velocity_y.value(), e_velocity_z.value() });
                }
            }
        }
        else if (lla_position_x.has_value() && lla_position_y.has_value() && lla_position_z.has_value()
                 && n_velocity_n.has_value() && n_velocity_e.has_value() && n_velocity_d.has_value())
        {
            if (auto posVel = std::reinterpret_pointer_cast<PosVel>(obs))
            {
                if (n_positionStdDev_n.has_value() && n_positionStdDev_e.has_value() && n_positionStdDev_d.has_value()
                    && n_velocityStdDev_n.has_value() && n_velocityStdDev_e.has_value() && n_velocityStdDev_d.has_value())
                {
                    posVel->setPosVelAndCov_n(Eigen::Vector3d{ lla_position_x.value(), lla_position_y.value(), lla_position_z.value() },
                                              Eigen::Vector3d{ n_velocity_n.value(), n_velocity_e.value(), n_velocity_d.value() },
                                              (Eigen::Vector6d() << n_positionStdDev_n.value(), n_positionStdDev_e.value(), n_positionStdDev_d.value(),
                                               n_velocityStdDev_n.value(), n_velocityStdDev_e.value(), n_velocityStdDev_d.value())
                                                  .finished()
                                                  .asDiagonal()
                                                  .toDenseMatrix());
                }
                else
                {
                    posVel->setPosition_lla(Eigen::Vector3d{ lla_position_x.value(), lla_position_y.value(), lla_position_z.value() });
                    posVel->setVelocity_n(Eigen::Vector3d{ n_velocity_n.value(), n_velocity_e.value(), n_velocity_d.value() });
                }
            }
        }
        else
        {
            LOG_WARN("{}: A PosVel/PosVelAtt file needs a position and velocity.", nameId());
            return nullptr;
        }
    }
    else if (_fileContent == FileContent::Pos)
    {
        if (e_position_x.has_value() && e_position_y.has_value() && e_position_z.has_value())
        {
            if (e_positionStdDev_x.has_value() && e_positionStdDev_y.has_value() && e_positionStdDev_z.has_value())
            {
                obs->setPositionAndCov_e(Eigen::Vector3d{ e_position_x.value(), e_position_y.value(), e_position_z.value() },
                                         Eigen::DiagonalMatrix<double, 3>{ e_positionStdDev_x.value(), e_positionStdDev_y.value(), e_positionStdDev_z.value() }.toDenseMatrix());
            }
            else
            {
                obs->setPosition_e(Eigen::Vector3d{ e_position_x.value(), e_position_y.value(), e_position_z.value() });
            }
        }
        else if (lla_position_x.has_value() && lla_position_y.has_value() && lla_position_z.has_value())
        {
            if (n_positionStdDev_n.has_value() && n_positionStdDev_e.has_value() && n_positionStdDev_d.has_value())
            {
                obs->setPositionAndCov_n(Eigen::Vector3d{ lla_position_x.value(), lla_position_y.value(), lla_position_z.value() },
                                         Eigen::DiagonalMatrix<double, 3>{ n_positionStdDev_n.value(), n_positionStdDev_e.value(), n_positionStdDev_d.value() }.toDenseMatrix());
            }
            else
            {
                obs->setPosition_lla(Eigen::Vector3d{ lla_position_x.value(), lla_position_y.value(), lla_position_z.value() });
            }
        }
        else
        {
            LOG_WARN("{}: A Pos file needs a position.", nameId());
            return nullptr;
        }
    }

    if (_fileContent == FileContent::PosVelAtt)
    {
        if (n_Quat_b_w.has_value() && n_Quat_b_x.has_value() && n_Quat_b_y.has_value() && n_Quat_b_z.has_value())
        {
            if (auto posVelAtt = std::reinterpret_pointer_cast<PosVelAtt>(obs))
            {
                posVelAtt->setAttitude_n_Quat_b(Eigen::Quaterniond{ n_Quat_b_w.value(), n_Quat_b_x.value(), n_Quat_b_y.value(), n_Quat_b_z.value() });
            }
        }
        else if (roll.has_value() && pitch.has_value() && yaw.has_value())
        {
            if (auto posVelAtt = std::reinterpret_pointer_cast<PosVelAtt>(obs))
            {
                posVelAtt->setAttitude_n_Quat_b(trafo::n_Quat_b(roll.value(), pitch.value(), yaw.value()));
            }
        }
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_PVA, obs);
    return obs;
}