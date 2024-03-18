// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "WiFiObsFile.hpp"

#include "util/Logger.hpp"

#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"

NAV::WiFiObsFile::WiFiObsFile()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 488, 248 };

    nm::CreateOutputPin(this, "WiFiObs", Pin::Type::Flow, { NAV::WiFiObs::type() }, &WiFiObsFile::pollData);
    // nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, { "std::vector<std::string>" }, &_headerColumns);
}

NAV::WiFiObsFile::~WiFiObsFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::WiFiObsFile::typeStatic()
{
    return "WiFiObsFile";
}

std::string NAV::WiFiObsFile::type() const
{
    return typeStatic();
}

std::string NAV::WiFiObsFile::category()
{
    return "Data Provider";
}

void NAV::WiFiObsFile::guiConfig()
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

    // // Header info
    // if (ImGui::BeginTable(fmt::format("##PvaHeaders ({})", id.AsPointer()).c_str(), 4,
    //                       ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    // {
    //     ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
    //     ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthFixed);
    //     ImGui::TableSetupColumn("Velocity", ImGuiTableColumnFlags_WidthFixed);
    //     ImGui::TableSetupColumn("Attitude", ImGuiTableColumnFlags_WidthFixed);
    //     ImGui::TableHeadersRow();

    //     auto TextColoredIfExists = [this](int index, const char* text) {
    //         ImGui::TableSetColumnIndex(index);
    //         if (std::find(_headerColumns.begin(), _headerColumns.end(), text) != _headerColumns.end())
    //         {
    //             ImGui::TextUnformatted(text);
    //         }
    //         else
    //         {
    //             ImGui::TextDisabled("%s", text);
    //         }
    //     };

    //     ImGui::TableNextRow();
    //     TextColoredIfExists(0, "GpsCycle");
    //     TextColoredIfExists(1, "Pos ECEF X [m]");
    //     TextColoredIfExists(2, "Vel ECEF X [m/s]");
    //     TextColoredIfExists(3, "n_Quat_b w");
    //     ImGui::TableNextRow();
    //     TextColoredIfExists(0, "GpsWeek");
    //     TextColoredIfExists(1, "Pos ECEF Y [m]");
    //     TextColoredIfExists(2, "Vel ECEF Y [m/s]");
    //     TextColoredIfExists(3, "n_Quat_b x");
    //     ImGui::TableNextRow();
    //     TextColoredIfExists(0, "GpsToW [s]");
    //     TextColoredIfExists(1, "Pos ECEF Z [m]");
    //     TextColoredIfExists(2, "Vel ECEF Z [m/s]");
    //     TextColoredIfExists(3, "n_Quat_b y");
    //     ImGui::TableNextRow();
    //     TextColoredIfExists(1, "Latitude [deg]");
    //     TextColoredIfExists(2, "Vel N [m/s]");
    //     TextColoredIfExists(3, "n_Quat_b z");
    //     ImGui::TableNextRow();
    //     TextColoredIfExists(1, "Longitude [deg]");
    //     TextColoredIfExists(2, "Vel E [m/s]");
    //     TextColoredIfExists(3, "Roll [deg]");
    //     ImGui::TableNextRow();
    //     TextColoredIfExists(1, "Altitude [m]");
    //     TextColoredIfExists(2, "Vel D [m/s]");
    //     TextColoredIfExists(3, "Pitch [deg]");
    //     ImGui::TableNextRow();
    //     TextColoredIfExists(3, "Yaw [deg]");

    //     ImGui::EndTable();
    // }
}

[[nodiscard]] json NAV::WiFiObsFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::WiFiObsFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::WiFiObsFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::WiFiObsFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::WiFiObsFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::WiFiObsFile::pollData()
{
    auto obs = std::make_shared<WiFiObs>();

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

    uint16_t gpsCycle = 0;
    uint16_t gpsWeek;
    long double gpsToW;
    InsTime time;
    std::string macAddress;
    double distance;

    bool gpsCycleSet = false;
    bool gpsWeekSet = false;
    bool gpsToWSet = false;
    bool macAddressSet = false;
    bool distanceSet = false;

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

            if (column == "GpsCycle")
            {
                gpsCycle = static_cast<uint16_t>(std::stoul(cell));
                gpsCycleSet = true;
            }
            else if (column == "GpsWeek")
            {
                gpsWeek = static_cast<uint16_t>(std::stoul(cell));
                gpsWeekSet = true;
            }
            else if (column == "GpsToW [s]")
            {
                gpsToW = std::stold(cell);
                gpsToWSet = true;
            }
            else if (column == "MacAddress")
            {
                macAddress = cell;
                macAddressSet = true;
            }
            else if (column == "Distance [m]")
            {
                distance = std::stod(cell);
                distanceSet = true;
            }
        }
    }

    if (!gpsCycleSet || !gpsWeekSet || !gpsToWSet || !macAddressSet || !distanceSet)
    {
        LOG_ERROR("{}: Not all columns are set", nameId());
        return nullptr;
    }
    else
    {
        time = InsTime(gpsCycle, gpsWeek, gpsToW);
    }
    obs->data.push_back({ macAddress, time, distance }); // TODO: Add timeOutputs
    obs->insTime = time;

    invokeCallbacks(OUTPUT_PORT_INDEX_WiFiObs_OBS, obs);
    return obs;
}