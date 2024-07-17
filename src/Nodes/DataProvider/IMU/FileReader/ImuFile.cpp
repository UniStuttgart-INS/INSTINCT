// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuFile.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

NAV::ImuFile::ImuFile()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 377, 201 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() }, &ImuFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, { "std::vector<std::string>" }, &_headerColumns);
}

NAV::ImuFile::~ImuFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuFile::typeStatic()
{
    return "ImuFile";
}

std::string NAV::ImuFile::type() const
{
    return typeStatic();
}

std::string NAV::ImuFile::category()
{
    return "Data Provider";
}

void NAV::ImuFile::guiConfig()
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

    Imu::guiConfig();

    // Header info
    if (ImGui::BeginTable(fmt::format("##ImuHeaders ({})", id.AsPointer()).c_str(), 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Delta IMU", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        auto TextColoredIfExists = [this](int index, const char* displayText, const char* searchText, bool alwaysNormal = false) {
            ImGui::TableSetColumnIndex(index);
            if (alwaysNormal
                || std::find_if(_headerColumns.begin(), _headerColumns.end(), [&searchText](const std::string& header) {
                       return header.starts_with(searchText);
                   }) != _headerColumns.end())
            {
                ImGui::TextUnformatted(displayText);
            }
            else
            {
                ImGui::TextDisabled("%s", displayText);
            }
        };

        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsCycle", "GpsCycle");
        TextColoredIfExists(1, "Mag", "MagX");
        TextColoredIfExists(2, "DeltaTime", "DeltaTime");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsWeek", "GpsWeek");
        TextColoredIfExists(1, "Acc", "AccX");
        TextColoredIfExists(2, "DeltaTheta", "DeltaThetaX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsToW", "GpsToW");
        TextColoredIfExists(1, "Gyro", "GyroX");
        TextColoredIfExists(2, "DeltaVel", "DeltaVelX");
        ImGui::TableNextRow();
        TextColoredIfExists(0, "TimeStartup", "TimeStartup");
        TextColoredIfExists(1, "Temperature", "Temperature");

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::ImuFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::ImuFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::ImuFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (FileReader::initialize())
    {
        for (auto& col : _headerColumns)
        {
            str::replace(col, "GpsTow", "GpsToW");
        }

        size_t nDelta = 0;
        for (const auto& col : _headerColumns)
        {
            if (col.starts_with("DeltaTime")
                || col.starts_with("DeltaThetaX") || col.starts_with("DeltaThetaY") || col.starts_with("DeltaThetaZ")
                || col.starts_with("DeltaVelX") || col.starts_with("DeltaVelY") || col.starts_with("DeltaVelZ"))
            {
                nDelta++;
            }
        }

        _withDelta = nDelta == 7;

        outputPins[OUTPUT_PORT_INDEX_IMU_OBS].dataIdentifier = _withDelta ? std::vector{ NAV::ImuObsWDelta::type() } : std::vector{ NAV::ImuObs::type() };
        return true;
    }

    outputPins[OUTPUT_PORT_INDEX_IMU_OBS].dataIdentifier = { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() };

    for (auto& link : outputPins[OUTPUT_PORT_INDEX_IMU_OBS].links)
    {
        if (auto* pin = link.getConnectedPin())
        {
            outputPins[OUTPUT_PORT_INDEX_IMU_OBS].recreateLink(*pin);
        }
    }

    return false;
}

void NAV::ImuFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::ImuFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::ImuFile::pollData()
{
    std::shared_ptr<ImuObs> obs;
    if (_withDelta) { obs = std::make_shared<ImuObsWDelta>(_imuPos); }
    else { obs = std::make_shared<ImuObs>(_imuPos); }

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
    std::optional<double> magX;
    std::optional<double> magY;
    std::optional<double> magZ;
    std::optional<double> accelX;
    std::optional<double> accelY;
    std::optional<double> accelZ;
    std::optional<double> gyroX;
    std::optional<double> gyroY;
    std::optional<double> gyroZ;

    std::optional<double> deltaTime;
    std::optional<double> deltaThetaX;
    std::optional<double> deltaThetaY;
    std::optional<double> deltaThetaZ;
    std::optional<double> deltaVelX;
    std::optional<double> deltaVelY;
    std::optional<double> deltaVelZ;

    // Split line at comma
    for (const auto& column : _headerColumns)
    {
        if (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());

            if (cell.empty()) { continue; }

            if (column.starts_with("GpsCycle"))
            {
                gpsCycle = static_cast<uint16_t>(std::stoul(cell));
            }
            else if (column.starts_with("GpsWeek"))
            {
                gpsWeek = static_cast<uint16_t>(std::stoul(cell));
            }
            else if (column.starts_with("GpsToW"))
            {
                gpsToW = std::stold(cell);
            }
            else if (column.starts_with("TimeStartup"))
            {
                obs->timeSinceStartup.emplace(std::stoull(cell));
            }
            else if (column.starts_with("MagX"))
            {
                magX = std::stod(cell);
            }
            else if (column.starts_with("MagY"))
            {
                magY = std::stod(cell);
            }
            else if (column.starts_with("MagZ"))
            {
                magZ = std::stod(cell);
            }
            else if (column.starts_with("AccX"))
            {
                accelX = std::stod(cell);
            }
            else if (column.starts_with("AccY"))
            {
                accelY = std::stod(cell);
            }
            else if (column.starts_with("AccZ"))
            {
                accelZ = std::stod(cell);
            }
            else if (column.starts_with("GyroX"))
            {
                gyroX = std::stod(cell);
            }
            else if (column.starts_with("GyroY"))
            {
                gyroY = std::stod(cell);
            }
            else if (column.starts_with("GyroZ"))
            {
                gyroZ = std::stod(cell);
            }
            else if (column.starts_with("Temperature"))
            {
                obs->temperature.emplace(std::stod(cell));
            }
            else if (column.starts_with("DeltaTime"))
            {
                deltaTime = std::stod(cell);
            }
            else if (column.starts_with("DeltaThetaX"))
            {
                deltaThetaX = std::stod(cell);
            }
            else if (column.starts_with("DeltaThetaY"))
            {
                deltaThetaY = std::stod(cell);
            }
            else if (column.starts_with("DeltaThetaZ"))
            {
                deltaThetaZ = std::stod(cell);
            }
            else if (column.starts_with("DeltaVelX"))
            {
                deltaVelX = std::stod(cell);
            }
            else if (column.starts_with("DeltaVelY"))
            {
                deltaVelY = std::stod(cell);
            }
            else if (column.starts_with("DeltaVelZ"))
            {
                deltaVelZ = std::stod(cell);
            }
        }
    }

    if (_withDelta)
    {
        if (deltaTime && deltaThetaX && deltaThetaY && deltaThetaZ && deltaVelX && deltaVelY && deltaVelZ)
        {
            if (auto obsWDelta = std::reinterpret_pointer_cast<ImuObsWDelta>(obs))
            {
                obsWDelta->dtime = deltaTime.value();
                obsWDelta->dtheta = { deltaThetaX.value(), deltaThetaY.value(), deltaThetaZ.value() };
                obsWDelta->dvel = { deltaVelX.value(), deltaVelY.value(), deltaVelZ.value() };
            }
        }
        else
        {
            LOG_ERROR("{}: Columns 'DeltaTime', 'DeltaThetaX', 'DeltaThetaY', 'DeltaThetaZ', 'DeltaVelX', 'DeltaVelY', 'DeltaVelZ' are needed.", nameId());
            return nullptr;
        }
    }

    if (!gpsCycle || !gpsWeek || !gpsToW)
    {
        LOG_ERROR("{}: Fields 'GpsCycle', 'GpsWeek', 'GpsToW' are needed.", nameId());
        return nullptr;
    }
    if (!accelX || !accelY || !accelZ)
    {
        LOG_ERROR("{}: Fields 'AccX', 'AccY', 'AccZ' are needed.", nameId());
        return nullptr;
    }
    if (!gyroX || !gyroY || !gyroZ)
    {
        LOG_ERROR("{}: Fields 'GyroX', 'GyroY', 'GyroZ' are needed.", nameId());
        return nullptr;
    }

    obs->insTime = InsTime(gpsCycle.value(), gpsWeek.value(), gpsToW.value());
    obs->p_acceleration = { accelX.value(), accelY.value(), accelZ.value() };
    obs->p_angularRate = { gyroX.value(), gyroY.value(), gyroZ.value() };

    if (magX && magY && magZ)
    {
        obs->p_magneticField.emplace(magX.value(), magY.value(), magZ.value());
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
    return obs;
}