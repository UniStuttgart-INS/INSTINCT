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

#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

NAV::ImuFile::ImuFile()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 377, 201 };

    CreateOutputPin("ImuObs", Pin::Type::Flow, { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() }, &ImuFile::pollData);
    CreateOutputPin("Header Columns", Pin::Type::Object, { "std::vector<std::string>" }, &_headerColumns);
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

        auto TextColoredIfExists = [this](int index, const char* displayText, std::vector<const char*> searchText, bool alwaysNormal = false) {
            ImGui::TableSetColumnIndex(index);
            if (alwaysNormal
                || std::ranges::find_if(_headerColumns, [&searchText](const std::string& header) {
                       auto headerStartsWithSearchText = false;
                       for (const char* searchIdx : searchText)
                       {
                           if (header.starts_with(searchIdx))
                           {
                               headerStartsWithSearchText = true;
                               continue;
                           }
                       }
                       return headerStartsWithSearchText;
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
        TextColoredIfExists(0, "GpsCycle", { "GpsCycle" });
        TextColoredIfExists(1, "Magnetometer", { "MagX", "Mag X" });
        TextColoredIfExists(2, "DeltaTime", { "DeltaTime" });
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsWeek", { "GpsWeek" });
        TextColoredIfExists(1, "Accelerometer", { "AccX", "Accel X" });
        TextColoredIfExists(2, "DeltaTheta", { "DeltaThetaX" });
        ImGui::TableNextRow();
        TextColoredIfExists(0, "GpsToW", { "GpsToW" });
        TextColoredIfExists(1, "Gyroscope", { "GyroX", "Gyro X" });
        TextColoredIfExists(2, "DeltaVel", { "DeltaVelX" });
        ImGui::TableNextRow();
        TextColoredIfExists(0, "TimeStartup", { "TimeStartup" });
        TextColoredIfExists(1, "Temperature", { "Temperature" });

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

    bool success = false;
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
        success = true;
    }
    else
    {
        outputPins[OUTPUT_PORT_INDEX_IMU_OBS].dataIdentifier = { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() };
    }

    for (auto& link : outputPins[OUTPUT_PORT_INDEX_IMU_OBS].links)
    {
        if (auto* pin = link.getConnectedPin())
        {
            outputPins[OUTPUT_PORT_INDEX_IMU_OBS].recreateLink(*pin);
        }
    }

    return success;
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
            cell.erase(std::ranges::find_if(cell, [](int ch) { return std::iscntrl(ch); }), cell.end());

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
            else if (column.starts_with("MagX") || column.starts_with("Mag X"))
            {
                magX = std::stod(cell);
            }
            else if (column.starts_with("MagY") || column.starts_with("Mag Y"))
            {
                magY = std::stod(cell);
            }
            else if (column.starts_with("MagZ") || column.starts_with("Mag Z"))
            {
                magZ = std::stod(cell);
            }
            else if (column.starts_with("AccX") || column.starts_with("Accel X"))
            {
                accelX = std::stod(cell);
            }
            else if (column.starts_with("AccY") || column.starts_with("Accel Y"))
            {
                accelY = std::stod(cell);
            }
            else if (column.starts_with("AccZ") || column.starts_with("Accel Z"))
            {
                accelZ = std::stod(cell);
            }
            else if (column.starts_with("GyroX") || column.starts_with("Gyro X"))
            {
                gyroX = std::stod(cell);
            }
            else if (column.starts_with("GyroY") || column.starts_with("Gyro Y"))
            {
                gyroY = std::stod(cell);
            }
            else if (column.starts_with("GyroZ") || column.starts_with("Gyro Z"))
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
        obs->p_magneticField = { magX.value(), magY.value(), magZ.value() };
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
    return obs;
}