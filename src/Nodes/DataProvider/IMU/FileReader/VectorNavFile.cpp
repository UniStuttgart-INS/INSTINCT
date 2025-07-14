// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "VectorNavFile.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "util/Vendor/VectorNav/VectorNavTypes.hpp"
#include <vn/types.h>

#include "util/Logger.hpp"
#include "util/Assert.h"
#include "util/StringUtil.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"

NAV::VectorNavFile::VectorNavFile()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 630, 466 };

    nm::CreateOutputPin(this, "Binary Output", Pin::Type::Flow, { NAV::VectorNavBinaryOutput::type() }, &VectorNavFile::pollData);
}

NAV::VectorNavFile::~VectorNavFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavFile::typeStatic()
{
    return "VectorNavFile";
}

std::string NAV::VectorNavFile::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavFile::category()
{
    return "Data Provider";
}

void NAV::VectorNavFile::guiConfig()
{
    if (auto res = FileReader::guiConfig("Supported types (*.csv *.vnb){.csv,.vnb},.*", { ".csv", ".vnb" }, size_t(id), nameId()))
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
    if (ImGui::BeginTable(fmt::format("##VectorNavHeaders ({})", id.AsPointer()).c_str(), 6,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit))
    {
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("GNSS1", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Attitude", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("INS", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("GNSS2", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        auto TextColored = [](int index, const char* label, bool enabled) {
            ImGui::TableSetColumnIndex(index);
            if (enabled)
            {
                ImGui::TextUnformatted(label);
            }
            else
            {
                ImGui::TextDisabled("%s", label);
            }
        };

        for (size_t i = 0; i < 16; i++)
        {
            if (i < std::max({ /* VectorNavSensor::_binaryGroupCommon.size(), */ VectorNavSensor::_binaryGroupTime.size(), VectorNavSensor::_binaryGroupIMU.size(),
                               VectorNavSensor::_binaryGroupGNSS.size(), VectorNavSensor::_binaryGroupAttitude.size(), VectorNavSensor::_binaryGroupINS.size() }))
            {
                ImGui::TableNextRow();
            }
            if (i < VectorNavSensor::_binaryGroupTime.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::_binaryGroupTime.at(i);
                TextColored(0, binaryGroupItem.name, _binaryOutputRegister.timeField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::_binaryGroupIMU.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::_binaryGroupIMU.at(i);
                TextColored(1, binaryGroupItem.name, _binaryOutputRegister.imuField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::_binaryGroupGNSS.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::_binaryGroupGNSS.at(i);
                TextColored(2, binaryGroupItem.name, _binaryOutputRegister.gpsField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::_binaryGroupAttitude.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::_binaryGroupAttitude.at(i);
                TextColored(3, binaryGroupItem.name, _binaryOutputRegister.attitudeField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::_binaryGroupINS.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::_binaryGroupINS.at(i);
                TextColored(4, binaryGroupItem.name, _binaryOutputRegister.insField & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < VectorNavSensor::_binaryGroupGNSS.size())
            {
                const auto& binaryGroupItem = VectorNavSensor::_binaryGroupGNSS.at(i);
                TextColored(5, binaryGroupItem.name, _binaryOutputRegister.gps2Field & binaryGroupItem.flagsValue);
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::VectorNavFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::VectorNavFile::restore(json const& j)
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

bool NAV::VectorNavFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::VectorNavFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::VectorNavFile::resetNode()
{
    FileReader::resetReader();

    _messageCount = 0;

    return true;
}

NAV::FileReader::FileType NAV::VectorNavFile::determineFileType()
{
    LOG_TRACE("called");

    std::filesystem::path filepath = getFilepath();

    auto filestreamHeader = std::ifstream(filepath);
    if (good())
    {
        std::array<char, std::string_view("GpsCycle,GpsWeek,GpsTow").length()> buffer{};
        filestreamHeader.read(buffer.data(), buffer.size());
        filestreamHeader.close();

        if (std::string(buffer.data(), buffer.size()).starts_with("Time [s]"))
        {
            _hasTimeColumn = true;
            return FileType::ASCII;
        }
        if (std::string(buffer.data(), buffer.size()).starts_with("GpsCycle,GpsWeek,GpsTow"))
        {
            _hasTimeColumn = false;
            return FileType::ASCII;
        }

        return FileType::BINARY;
    }

    LOG_ERROR("Could not open file {}", filepath);
    return FileType::NONE;
}

void NAV::VectorNavFile::readHeader()
{
    if (_fileType == FileType::ASCII)
    {
        _binaryOutputRegister.timeField = vn::protocol::uart::TimeGroup::TIMEGROUP_NONE;
        _binaryOutputRegister.imuField = vn::protocol::uart::ImuGroup::IMUGROUP_NONE;
        _binaryOutputRegister.gpsField = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;
        _binaryOutputRegister.attitudeField = vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE;
        _binaryOutputRegister.insField = vn::protocol::uart::InsGroup::INSGROUP_NONE;
        _binaryOutputRegister.gps2Field = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;

        // Read header line
        std::string line;
        getline(line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::ranges::find_if(line, [](int ch) { return std::isalnum(ch); }));
        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;

        int column = 0;
        // Split line at comma
        LOG_DATA("{}: Reading CSV header", nameId());
        while (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::ranges::find_if(cell, [](int ch) { return std::iscntrl(ch); }), cell.end());

            LOG_DATA("{}:   {}", nameId(), cell);
            _headerColumns.push_back(cell);
            if (column++ > (_hasTimeColumn ? 3 : 2))
            {
                std::string group = cell.substr(0, cell.find("::")); // Extract the group (Time::TimeUTC::year -> 'Time')

                cell = cell.substr(cell.find("::") + 2); // Remove the group -> 'TimeUTC::year'
                if (cell.find("::") != std::string::npos)
                {
                    cell = cell.substr(0, cell.find("::")); // Remove subgroups ('TimeUTC::year' -> 'TimeUTC')
                }
                if (cell.find(' ') != std::string::npos)
                {
                    cell = cell.substr(0, cell.find(' ')); // Remove everything after a blank, which is the unit ('TimeStartup [ns]' -> 'TimeStartup')
                }

                bool identified = false;
                if (group == "Time")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::_binaryGroupTime)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            _binaryOutputRegister.timeField |= static_cast<vn::protocol::uart::TimeGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "IMU")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::_binaryGroupIMU)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            _binaryOutputRegister.imuField |= static_cast<vn::protocol::uart::ImuGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                        if (cell == "DeltaTime")
                        {
                            _binaryOutputRegister.imuField |= vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA;
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "GNSS1")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::_binaryGroupGNSS)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            _binaryOutputRegister.gpsField |= static_cast<vn::protocol::uart::GpsGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "Att")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::_binaryGroupAttitude)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            _binaryOutputRegister.attitudeField |= static_cast<vn::protocol::uart::AttitudeGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "INS")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::_binaryGroupINS)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            _binaryOutputRegister.insField |= static_cast<vn::protocol::uart::InsGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else if (group == "GNSS2")
                {
                    for (const auto& binaryGroupItem : VectorNavSensor::_binaryGroupGNSS)
                    {
                        if (cell == binaryGroupItem.name)
                        {
                            _binaryOutputRegister.gps2Field |= static_cast<vn::protocol::uart::GpsGroup>(binaryGroupItem.flagsValue);
                            identified = true;
                            break;
                        }
                    }
                }
                else
                {
                    LOG_ERROR("{}: Could not identify the group in CSV header - {}::{}", nameId(), group, cell);
                    doDeinitialize();
                    break;
                }

                if (!identified)
                {
                    LOG_ERROR("{}: Could not identify the field in CSV header - {}::{}", nameId(), group, cell);
                    doDeinitialize();
                    break;
                }
            }
        }
    }
    else // if (fileType == FileType::BINARY)
    {
        read(reinterpret_cast<char*>(&_binaryOutputRegister.timeField), sizeof(vn::protocol::uart::TimeGroup));
        read(reinterpret_cast<char*>(&_binaryOutputRegister.imuField), sizeof(vn::protocol::uart::ImuGroup));
        read(reinterpret_cast<char*>(&_binaryOutputRegister.gpsField), sizeof(vn::protocol::uart::GpsGroup));
        read(reinterpret_cast<char*>(&_binaryOutputRegister.attitudeField), sizeof(vn::protocol::uart::AttitudeGroup));
        read(reinterpret_cast<char*>(&_binaryOutputRegister.insField), sizeof(vn::protocol::uart::InsGroup));
        read(reinterpret_cast<char*>(&_binaryOutputRegister.gps2Field), sizeof(vn::protocol::uart::GpsGroup));
    }
}

std::shared_ptr<const NAV::NodeData> NAV::VectorNavFile::pollData()
{
    auto obs = std::make_shared<VectorNavBinaryOutput>(_imuPos);

    if (_fileType == FileType::ASCII)
    {
        // Read line
        std::string line;
        getline(line);
        _messageCount++;
        // Remove any starting non text characters
        line.erase(line.begin(), std::ranges::find_if(line, [](int ch) { return std::isgraph(ch); }));
        LOG_DATA("{}: Reading line {}: {}", nameId(), _messageCount + 1, line);

        if (line.empty())
        {
            LOG_DEBUG("{}: End of file reached after {} lines", nameId(), _messageCount);
            return nullptr;
        }

        // Convert line into stream
        std::stringstream lineStream(line);

        size_t col = 0;
        auto extractCell = [&lineStream, &col]() {
            if (lineStream.eof())
            {
                throw std::runtime_error("End of file");
            }
            col++;
            if (std::string cell; std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::ranges::find_if(cell, [](int ch) { return std::iscntrl(ch); }), cell.end());

                // LOG_DEBUG("  extractCell: {}", cell);
                return cell;
            }
            return std::string("");
        };
        auto extractRemoveTillDelimiter = [](std::string& str, const std::string& delimiter) {
            std::string extract;
            if (size_t pos = str.find(delimiter);
                pos != std::string::npos)
            {
                extract = str.substr(0, pos);
                str = str.substr(pos + 1);
            }

            return extract;
        };

        auto extractSingleValue = [&](auto& field, auto flag, auto& out) {
            static_assert(std::is_same_v<bool&, decltype(out)> || std::is_same_v<uint8_t&, decltype(out)> || std::is_same_v<uint16_t&, decltype(out)> || std::is_same_v<uint32_t&, decltype(out)> || std::is_same_v<uint64_t&, decltype(out)>
                          || std::is_same_v<int8_t&, decltype(out)> || std::is_same_v<int16_t&, decltype(out)> || std::is_same_v<int32_t&, decltype(out)>
                          || std::is_same_v<float&, decltype(out)> || std::is_same_v<double&, decltype(out)>
                          || std::is_same_v<std::string&, decltype(out)>);

            auto cell = extractCell();
            LOG_DATA("{}: Extracting {}: {}", nameId(), vn::protocol::uart::to_string(flag), cell);

            if (cell.empty()) { return; }

            field |= flag; // set the flag
            if constexpr (std::is_same_v<bool&, decltype(out)>
                          || std::is_same_v<uint8_t&, decltype(out)>
                          || std::is_same_v<uint16_t&, decltype(out)>
                          || std::is_same_v<uint32_t&, decltype(out)>)
            {
                out = static_cast<std::remove_reference_t<decltype(out)>>(std::stoul(cell));
            }
            else if constexpr (std::is_same_v<uint64_t&, decltype(out)>)
            {
                out = static_cast<std::remove_reference_t<decltype(out)>>(std::stoull(cell));
            }
            else if constexpr (std::is_same_v<int8_t&, decltype(out)>
                               || std::is_same_v<int16_t&, decltype(out)>
                               || std::is_same_v<int32_t&, decltype(out)>)
            {
                out = static_cast<std::remove_reference_t<decltype(out)>>(std::stoi(cell));
            }
            else if constexpr (std::is_same_v<float&, decltype(out)>)
            {
                out = static_cast<std::remove_reference_t<decltype(out)>>(std::stof(cell));
            }
            else if constexpr (std::is_same_v<double&, decltype(out)>)
            {
                out = static_cast<std::remove_reference_t<decltype(out)>>(std::stod(cell));
            }
            else if constexpr (std::is_same_v<std::string&, decltype(out)>)
            {
                out = cell;
            }
        };

        auto extractValue = [&](auto& field, auto flag, auto&... out) {
            (extractSingleValue(field, flag, out), ...);
        };

        try
        {
            if (_hasTimeColumn) { extractCell(); } // Time [s]
            std::string gpsCycle = extractCell();
            std::string gpsWeek = extractCell();
            std::string gpsTow = extractCell();
            if (!gpsCycle.empty() && !gpsWeek.empty() && !gpsTow.empty())
            {
                obs->insTime = InsTime(std::stoi(gpsCycle), std::stoi(gpsWeek), std::stold(gpsTow));
            }
            else
            {
                LOG_DATA("{}:   Skipping message {}, as no InsTime set.", nameId(), _messageCount + 1);
                return obs;
            }

            for (; col < _headerColumns.size();)
            {
                const auto& column = _headerColumns.at(col);
                // Group 2 (Time)
                if (_binaryOutputRegister.timeField != vn::protocol::uart::TimeGroup::TIMEGROUP_NONE)
                {
                    if (!obs->timeOutputs) { obs->timeOutputs = std::make_shared<NAV::vendor::vectornav::TimeOutputs>(); }

                    if (column == "Time::TimeStartup [ns]")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP, obs->timeOutputs->timeStartup);
                        continue;
                    }
                    if (column == "Time::TimeGps [ns]")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS, obs->timeOutputs->timeGps);
                        continue;
                    }
                    if (column == "Time::GpsTow [ns]")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW, obs->timeOutputs->gpsTow);
                        continue;
                    }
                    if (column == "Time::GpsWeek")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK, obs->timeOutputs->gpsWeek);
                        continue;
                    }
                    if (column == "Time::TimeSyncIn [ns]")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN, obs->timeOutputs->timeSyncIn);
                        continue;
                    }
                    if (column == "Time::TimeGpsPps [ns]")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS, obs->timeOutputs->timePPS);
                        continue;
                    }
                    if (column == "Time::TimeUTC::year")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC,
                                     obs->timeOutputs->timeUtc.year, obs->timeOutputs->timeUtc.month, obs->timeOutputs->timeUtc.day,
                                     obs->timeOutputs->timeUtc.hour, obs->timeOutputs->timeUtc.min, obs->timeOutputs->timeUtc.sec, obs->timeOutputs->timeUtc.ms);
                        continue;
                    }
                    if (column == "Time::SyncInCnt")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT, obs->timeOutputs->syncInCnt);
                        continue;
                    }
                    if (column == "Time::SyncOutCnt")
                    {
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT, obs->timeOutputs->syncOutCnt);
                        continue;
                    }
                    if (column == "Time::TimeStatus::timeOk") // "Time::TimeStatus::dateOk", "Time::TimeStatus::utcTimeValid"
                    {
                        uint8_t timeOk{};
                        uint8_t dateOk{};
                        uint8_t utcTimeValid{};
                        extractValue(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS, timeOk, dateOk, utcTimeValid);
                        obs->timeOutputs->timeStatus = static_cast<uint8_t>(timeOk << 0U | dateOk << 1U | utcTimeValid << 2U);
                        continue;
                    }
                }
                // Group 3 (IMU)
                if (_binaryOutputRegister.imuField != vn::protocol::uart::ImuGroup::IMUGROUP_NONE)
                {
                    if (!obs->imuOutputs) { obs->imuOutputs = std::make_shared<NAV::vendor::vectornav::ImuOutputs>(); }

                    if (column == "IMU::ImuStatus")
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS, obs->imuOutputs->imuStatus);
                        continue;
                    }
                    if (column.starts_with("IMU::UncompMag::X")) // "IMU::UncompMag::Y [Gauss]", "IMU::UncompMag::Z [Gauss]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG,
                                     obs->imuOutputs->uncompMag.x(), obs->imuOutputs->uncompMag.y(), obs->imuOutputs->uncompMag.z());
                        continue;
                    }
                    if (column.starts_with("IMU::UncompAccel::X")) // "IMU::UncompAccel::Y [m/s^2]", "IMU::UncompAccel::Z [m/s^2]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL,
                                     obs->imuOutputs->uncompAccel.x(), obs->imuOutputs->uncompAccel.y(), obs->imuOutputs->uncompAccel.z());
                        continue;
                    }
                    if (column.starts_with("IMU::UncompGyro::X")) // "IMU::UncompGyro::Y [rad/s]", "IMU::UncompGyro::Z [rad/s]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO,
                                     obs->imuOutputs->uncompGyro.x(), obs->imuOutputs->uncompGyro.y(), obs->imuOutputs->uncompGyro.z());
                        continue;
                    }
                    if (column.starts_with("IMU::Temp"))
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_TEMP, obs->imuOutputs->temp);
                        continue;
                    }
                    if (column.starts_with("IMU::Pres"))
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_PRES, obs->imuOutputs->pres);
                        continue;
                    }
                    if (column.starts_with("IMU::DeltaTime")) // "IMU::DeltaTheta::X [deg]", "IMU::DeltaTheta::Y [deg]", "IMU::DeltaTheta::Z [deg]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA,
                                     obs->imuOutputs->deltaTime, obs->imuOutputs->deltaTheta.x(), obs->imuOutputs->deltaTheta.y(), obs->imuOutputs->deltaTheta.z());
                        continue;
                    }
                    if (column.starts_with("IMU::DeltaVel::X")) // "IMU::DeltaVel::Y [m/s]", "IMU::DeltaVel::Z [m/s]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL,
                                     obs->imuOutputs->deltaV.x(), obs->imuOutputs->deltaV.y(), obs->imuOutputs->deltaV.z());
                        continue;
                    }
                    if (column.starts_with("IMU::Mag::X")) // "IMU::Mag::Y [Gauss]", "IMU::Mag::Z [Gauss]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_MAG,
                                     obs->imuOutputs->mag.x(), obs->imuOutputs->mag.y(), obs->imuOutputs->mag.z());
                        continue;
                    }
                    if (column.starts_with("IMU::Accel::X")) // "IMU::Accel::Y [m/s^2]", "IMU::Accel::Z [m/s^2]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL,
                                     obs->imuOutputs->accel.x(), obs->imuOutputs->accel.y(), obs->imuOutputs->accel.z());
                        continue;
                    }
                    if (column.starts_with("IMU::AngularRate::X")) // "IMU::AngularRate::Y [rad/s]", "IMU::AngularRate::Z [rad/s]"
                    {
                        extractValue(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE,
                                     obs->imuOutputs->angularRate.x(), obs->imuOutputs->angularRate.y(), obs->imuOutputs->angularRate.z());
                        continue;
                    }
                }
                // Group 4 (GNSS1)
                if (_binaryOutputRegister.gpsField != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
                {
                    if (!obs->gnss1Outputs) { obs->gnss1Outputs = std::make_shared<NAV::vendor::vectornav::GnssOutputs>(); }

                    if (column == "GNSS1::UTC::year") // "GNSS1::UTC::month", "GNSS1::UTC::day", "GNSS1::UTC::hour", "GNSS1::UTC::min", "GNSS1::UTC::sec", "GNSS1::UTC::ms"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_UTC,
                                     obs->gnss1Outputs->timeUtc.year, obs->gnss1Outputs->timeUtc.month, obs->gnss1Outputs->timeUtc.day,
                                     obs->gnss1Outputs->timeUtc.hour, obs->gnss1Outputs->timeUtc.min, obs->gnss1Outputs->timeUtc.sec, obs->gnss1Outputs->timeUtc.ms);
                        continue;
                    }
                    if (column == "GNSS1::Tow [ns]")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW, obs->gnss1Outputs->tow);
                        continue;
                    }
                    if (column == "GNSS1::Week")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK, obs->gnss1Outputs->week);
                        continue;
                    }
                    if (column == "GNSS1::NumSats")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS, obs->gnss1Outputs->numSats);
                        continue;
                    }
                    if (column == "GNSS1::Fix")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX, obs->gnss1Outputs->fix);
                        continue;
                    }
                    if (column == "GNSS1::PosLla::latitude [deg]") // "GNSS1::PosLla::longitude [deg]", "GNSS1::PosLla::altitude [m]"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA,
                                     obs->gnss1Outputs->posLla.x(), obs->gnss1Outputs->posLla.y(), obs->gnss1Outputs->posLla.z());
                        continue;
                    }
                    if (column == "GNSS1::PosEcef::X [m]") // "GNSS1::PosEcef::Y [m]", "GNSS1::PosEcef::Z [m]"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF,
                                     obs->gnss1Outputs->posEcef.x(), obs->gnss1Outputs->posEcef.y(), obs->gnss1Outputs->posEcef.z());
                        continue;
                    }
                    if (column == "GNSS1::VelNed::N [m/s]") // "GNSS1::VelNed::E [m/s]", "GNSS1::VelNed::D [m/s]"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELNED,
                                     obs->gnss1Outputs->velNed.x(), obs->gnss1Outputs->velNed.y(), obs->gnss1Outputs->velNed.z());
                        continue;
                    }
                    if (column == "GNSS1::VelEcef::X [m/s]") // "GNSS1::VelEcef::Y [m/s]", "GNSS1::VelEcef::Z [m/s]"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF,
                                     obs->gnss1Outputs->velEcef.x(), obs->gnss1Outputs->velEcef.y(), obs->gnss1Outputs->velEcef.z());
                        continue;
                    }
                    if (column == "GNSS1::PosU::N [m]") // "GNSS1::PosU::E [m]", "GNSS1::PosU::D [m]"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSU,
                                     obs->gnss1Outputs->posU.x(), obs->gnss1Outputs->posU.y(), obs->gnss1Outputs->posU.z());
                        continue;
                    }
                    if (column == "GNSS1::VelU [m/s]")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELU, obs->gnss1Outputs->velU);
                        continue;
                    }
                    if (column == "GNSS1::TimeU [s]")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU, obs->gnss1Outputs->timeU);
                        continue;
                    }
                    if (column == "GNSS1::TimeInfo::Status::timeOk") // "GNSS1::TimeInfo::Status::dateOk", "GNSS1::TimeInfo::Status::utcTimeValid", "GNSS1::TimeInfo::LeapSeconds"
                    {
                        uint8_t timeOk{};
                        uint8_t dateOk{};
                        uint8_t utcTimeValid{};
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO, timeOk, dateOk, utcTimeValid, obs->gnss1Outputs->timeInfo.leapSeconds);
                        obs->gnss1Outputs->timeInfo.status = static_cast<uint8_t>(timeOk << 0U | dateOk << 1U | utcTimeValid << 2U);
                        continue;
                    }
                    if (column == "GNSS1::DOP::g") // "GNSS1::DOP::p","GNSS1::DOP::t","GNSS1::DOP::v","GNSS1::DOP::h","GNSS1::DOP::n","GNSS1::DOP::e"
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_DOP,
                                     obs->gnss1Outputs->dop.gDop, obs->gnss1Outputs->dop.pDop, obs->gnss1Outputs->dop.tDop, obs->gnss1Outputs->dop.vDop,
                                     obs->gnss1Outputs->dop.hDop, obs->gnss1Outputs->dop.nDop, obs->gnss1Outputs->dop.eDop);
                        continue;
                    }
                    if (column == "GNSS1::SatInfo::NumSats")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO, obs->gnss1Outputs->satInfo.numSats);
                        continue;
                    }
                    if (column == "GNSS1::SatInfo::Satellites")
                    {
                        std::string satellites;
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO, satellites);

                        for (size_t i = 0; i < obs->gnss1Outputs->satInfo.numSats; i++)
                        {
                            satellites = satellites.substr(1); // Remove leading '['
                            auto sys = static_cast<int8_t>(std::stoi(extractRemoveTillDelimiter(satellites, "|")));
                            auto svId = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagHealthy = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagAlmanac = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagEphemeris = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagDifferentialCorrection = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagUsedForNavigation = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagAzimuthElevationValid = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagUsedForRTK = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flags = static_cast<uint8_t>(flagHealthy << 0U
                                                              | flagAlmanac << 1U
                                                              | flagEphemeris << 2U
                                                              | flagDifferentialCorrection << 3U
                                                              | flagUsedForNavigation << 4U
                                                              | flagAzimuthElevationValid << 5U
                                                              | flagUsedForRTK << 6U);
                            auto cno = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto qi = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto el = static_cast<int8_t>(std::stoi(extractRemoveTillDelimiter(satellites, "|")));
                            auto az = static_cast<int16_t>(std::stoi(extractRemoveTillDelimiter(satellites, "]")));
                            obs->gnss1Outputs->satInfo.satellites.emplace_back(sys, svId, flags, cno, qi, el, az);
                        }
                        continue;
                    }
                    if (column.starts_with("GNSS1::SatInfo::") && column.ends_with(" - flag Healthy"))
                    {
                        std::string columnText = column.substr(16);
                        SatId satId(extractRemoveTillDelimiter(columnText, " -"));
                        LOG_DATA("{}:  SatInfo {}", nameId(), satId);
                        bool flagHealthy{};
                        bool flagAlmanac{};
                        bool flagEphemeris{};
                        bool flagDifferentialCorrection{};
                        bool flagUsedForNavigation{};
                        bool flagAzimuthElevationValid{};
                        bool flagUsedForRTK{};
                        auto cno = std::numeric_limits<uint8_t>::max();
                        uint8_t qi{};
                        int8_t el{};
                        int16_t az{};
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO,
                                     flagHealthy, flagAlmanac, flagEphemeris, flagDifferentialCorrection,
                                     flagUsedForNavigation, flagAzimuthElevationValid, flagUsedForRTK,
                                     cno, qi, el, az);
                        if (cno == std::numeric_limits<uint8_t>::max())
                        {
                            LOG_DATA("{}:   Skipping {} as empty", nameId(), satId);
                            continue;
                        }
                        auto flags = static_cast<uint8_t>(flagHealthy << 0U
                                                          | flagAlmanac << 1U
                                                          | flagEphemeris << 2U
                                                          | flagDifferentialCorrection << 3U
                                                          | flagUsedForNavigation << 4U
                                                          | flagAzimuthElevationValid << 5U
                                                          | flagUsedForRTK << 6U);
                        obs->gnss1Outputs->satInfo.satellites.emplace_back(static_cast<uint8_t>(vendor::vectornav::fromSatelliteSystem(satId.satSys)),
                                                                           static_cast<uint8_t>(satId.satNum), flags, cno, qi, el, az);
                        continue;
                    }
                    if (column == "GNSS1::RawMeas::Tow [s]")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, obs->gnss1Outputs->raw.tow);
                        continue;
                    }
                    if (column == "GNSS1::RawMeas::Week")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, obs->gnss1Outputs->raw.week);
                        continue;
                    }
                    if (column == "GNSS1::RawMeas::NumSats")
                    {
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, obs->gnss1Outputs->raw.numSats);
                        continue;
                    }
                    if (column == "GNSS1::RawMeas::Satellites")
                    {
                        std::string satellites;
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, satellites);

                        for (size_t i = 0; i < obs->gnss1Outputs->raw.numSats; i++)
                        {
                            satellites = satellites.substr(1); // Remove leading '['
                            auto sys = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto svId = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto freq = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto chan = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto slot = static_cast<int8_t>(std::stoi(extractRemoveTillDelimiter(satellites, "|")));
                            auto cno = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagSearching = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagTracking = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagTimeValid = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagCodeLock = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseLock = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseHalfAmbiguity = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseHalfSub = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseSlip = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPseudorangeSmoothed = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flags = static_cast<uint16_t>(flagSearching << 0U
                                                               | flagTracking << 1U
                                                               | flagTimeValid << 2U
                                                               | flagCodeLock << 3U
                                                               | flagPhaseLock << 4U
                                                               | flagPhaseHalfAmbiguity << 5U
                                                               | flagPhaseHalfSub << 6U
                                                               | flagPhaseSlip << 7U
                                                               | flagPseudorangeSmoothed << 8U);
                            auto pr = std::stod(extractRemoveTillDelimiter(satellites, "|"));
                            auto cp = std::stod(extractRemoveTillDelimiter(satellites, "|"));
                            auto dp = std::stof(extractRemoveTillDelimiter(satellites, "]"));
                            obs->gnss1Outputs->raw.satellites.emplace_back(sys, svId, freq, chan, slot, cno, flags, pr, cp, dp);
                        }
                        continue;
                    }
                    if (column.starts_with("GNSS1::RawMeas::") && column.ends_with(" - freq"))
                    {
                        std::string columnText = column.substr(16);
                        auto identifier = extractRemoveTillDelimiter(columnText, " - freq");
                        LOG_DATA("{}:  RawMeas {}", nameId(), identifier);
                        SatelliteSystem satSys;
                        uint16_t satNum{};
                        if (identifier.length() == 6)
                        {
                            SatSigId satSigId(identifier);
                            satSys = satSigId.toSatId().satSys;
                            satNum = satSigId.satNum;
                        }
                        else
                        {
                            SatId satId(identifier);
                            satSys = satId.satSys;
                            satNum = satId.satNum;
                        }
                        auto freq = std::numeric_limits<uint8_t>::max();
                        uint8_t chan{};
                        int8_t slot{};
                        uint8_t cno{};
                        uint8_t flagSearching{};
                        uint8_t flagTracking{};
                        uint8_t flagTimeValid{};
                        uint8_t flagCodeLock{};
                        uint8_t flagPhaseLock{};
                        uint8_t flagPhaseHalfAmbiguity{};
                        uint8_t flagPhaseHalfSub{};
                        uint8_t flagPhaseSlip{};
                        uint8_t flagPseudorangeSmoothed{};
                        double pr{};
                        double cp{};
                        float dp{};
                        extractValue(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS,
                                     freq, chan, slot, cno,
                                     flagSearching, flagTracking, flagTimeValid, flagCodeLock, flagPhaseLock,
                                     flagPhaseHalfAmbiguity, flagPhaseHalfSub, flagPhaseSlip, flagPseudorangeSmoothed,
                                     pr, cp, dp);
                        if (freq == std::numeric_limits<uint8_t>::max())
                        {
                            LOG_DATA("{}:   Skipping {} as empty", nameId(), identifier);
                            continue;
                        }
                        auto flags = static_cast<uint16_t>(flagSearching << 0U
                                                           | flagTracking << 1U
                                                           | flagTimeValid << 2U
                                                           | flagCodeLock << 3U
                                                           | flagPhaseLock << 4U
                                                           | flagPhaseHalfAmbiguity << 5U
                                                           | flagPhaseHalfSub << 6U
                                                           | flagPhaseSlip << 7U
                                                           | flagPseudorangeSmoothed << 8U);
                        obs->gnss1Outputs->raw.satellites.emplace_back(static_cast<uint8_t>(vendor::vectornav::fromSatelliteSystem(satSys)),
                                                                       static_cast<uint8_t>(satNum), freq, chan, slot, cno, flags, pr, cp, dp);
                        if (obs->gnss1Outputs->raw.satellites.back().toCode() == Code::None)
                        {
                            LOG_ERROR("{}: Line {}: Could not convert to valid Code. Skipping item. (sys {}, freq {}, chan {})", nameId(), _messageCount + 1,
                                      obs->gnss1Outputs->raw.satellites.back().sys, obs->gnss1Outputs->raw.satellites.back().freq, obs->gnss1Outputs->raw.satellites.back().chan);
                            obs->gnss1Outputs->raw.satellites.pop_back();
                        }
                        if (identifier.length() == 6 && SatSigId(identifier) != obs->gnss1Outputs->raw.satellites.back().toSatSigId())
                        {
                            LOG_ERROR("{}: Line {}: Could not convert to valid SatSigId. Skipping item. (sys {}, freq {}, chan {} = [{}], column was [{}])", nameId(), _messageCount + 1,
                                      obs->gnss1Outputs->raw.satellites.back().sys, obs->gnss1Outputs->raw.satellites.back().freq, obs->gnss1Outputs->raw.satellites.back().chan,
                                      obs->gnss1Outputs->raw.satellites.back().toSatSigId(), identifier);
                            obs->gnss1Outputs->raw.satellites.pop_back();
                        }
                        continue;
                    }
                }
                // Group 5 (Attitude)
                if (_binaryOutputRegister.attitudeField != vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE)
                {
                    if (!obs->attitudeOutputs) { obs->attitudeOutputs = std::make_shared<NAV::vendor::vectornav::AttitudeOutputs>(); }

                    if (column == "Att::VpeStatus::AttitudeQuality") // "Att::VpeStatus::GyroSaturation", "Att::VpeStatus::GyroSaturationRecovery", "Att::VpeStatus::MagDisturbance", "Att::VpeStatus::MagSaturation", "Att::VpeStatus::AccDisturbance", "Att::VpeStatus::AccSaturation", "Att::VpeStatus::KnownMagDisturbance", "Att::VpeStatus::KnownAccelDisturbance"
                    {
                        uint8_t attitudeQuality{};        // return ((_status & (1U << 0U | 1U << 1U)) >> 0U);
                        uint8_t gyroSaturation{};         // return ((_status & (1U << 2U)) >> 2U);
                        uint8_t gyroSaturationRecovery{}; // return ((_status & (1U << 3U)) >> 3U);
                        uint8_t magDisturbance{};         // return ((_status & (1U << 4U | 1U << 5U)) >> 4U);
                        uint8_t magSaturation{};          // return ((_status & (1U << 6U)) >> 6U);
                        uint8_t accDisturbance{};         // return ((_status & (1U << 7U | 1U << 8U)) >> 7U);
                        uint8_t accSaturation{};          // return ((_status & (1U << 9U)) >> 9U);
                        uint8_t knownMagDisturbance{};    // return ((_status & (1U << 11U)) >> 11U);
                        uint8_t knownAccelDisturbance{};  // return ((_status & (1U << 12U)) >> 12U);
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS,
                                     attitudeQuality, gyroSaturation, gyroSaturationRecovery, magDisturbance, magSaturation, accDisturbance, accSaturation, knownMagDisturbance, knownAccelDisturbance);
                        obs->attitudeOutputs->vpeStatus = static_cast<uint16_t>(attitudeQuality << 0U
                                                                                | gyroSaturation << 2U
                                                                                | gyroSaturationRecovery << 3U
                                                                                | magDisturbance << 4U
                                                                                | magSaturation << 6U
                                                                                | accDisturbance << 7U
                                                                                | accSaturation << 9U
                                                                                | knownMagDisturbance << 11U
                                                                                | knownAccelDisturbance << 12U);
                        continue;
                    }
                    if (column == "Att::YawPitchRoll::Y [deg]") // "Att::YawPitchRoll::P [deg]", "Att::YawPitchRoll::R [deg]"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL,
                                     obs->attitudeOutputs->ypr.x(), obs->attitudeOutputs->ypr.y(), obs->attitudeOutputs->ypr.z());
                        continue;
                    }
                    if (column == "Att::Quaternion::w") // "Att::Quaternion::x", "Att::Quaternion::y", "Att::Quaternion::z"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION,
                                     obs->attitudeOutputs->qtn.w(), obs->attitudeOutputs->qtn.x(), obs->attitudeOutputs->qtn.y(), obs->attitudeOutputs->qtn.z());
                        continue;
                    }
                    if (column == "Att::DCM::0-0") // "Att::DCM::0-1", "Att::DCM::0-2", "Att::DCM::1-0", "Att::DCM::1-1", "Att::DCM::1-2", "Att::DCM::2-0", "Att::DCM::2-1", "Att::DCM::2-2"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM,
                                     obs->attitudeOutputs->dcm(0, 0), obs->attitudeOutputs->dcm(0, 1), obs->attitudeOutputs->dcm(0, 2),
                                     obs->attitudeOutputs->dcm(1, 0), obs->attitudeOutputs->dcm(1, 1), obs->attitudeOutputs->dcm(1, 2),
                                     obs->attitudeOutputs->dcm(2, 0), obs->attitudeOutputs->dcm(2, 1), obs->attitudeOutputs->dcm(2, 2));
                        continue;
                    }
                    if (column == "Att::MagNed::N [Gauss]") // "Att::MagNed::E [Gauss]", "Att::MagNed::D [Gauss]"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED,
                                     obs->attitudeOutputs->magNed.x(), obs->attitudeOutputs->magNed.y(), obs->attitudeOutputs->magNed.z());
                        continue;
                    }
                    if (column == "Att::AccelNed::N [m/s^2]") // "Att::AccelNed::E [m/s^2]", "Att::AccelNed::D [m/s^2]"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED,
                                     obs->attitudeOutputs->accelNed.x(), obs->attitudeOutputs->accelNed.y(), obs->attitudeOutputs->accelNed.z());
                        continue;
                    }
                    if (column == "Att::LinearAccelBody::X [m/s^2]") // "Att::LinearAccelBody::Y [m/s^2]", "Att::LinearAccelBody::Z [m/s^2]"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
                                     obs->attitudeOutputs->linearAccelBody.x(), obs->attitudeOutputs->linearAccelBody.y(), obs->attitudeOutputs->linearAccelBody.z());
                        continue;
                    }
                    if (column == "Att::LinearAccelNed::N [m/s^2]") // "Att::LinearAccelNed::E [m/s^2]", "Att::LinearAccelNed::D [m/s^2]"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED,
                                     obs->attitudeOutputs->linearAccelNed.x(), obs->attitudeOutputs->linearAccelNed.y(), obs->attitudeOutputs->linearAccelNed.z());
                        continue;
                    }
                    if (column == "Att::YprU::Y [deg]") // "Att::YprU::P [deg]", "Att::YprU::R [deg]"
                    {
                        extractValue(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU,
                                     obs->attitudeOutputs->yprU.x(), obs->attitudeOutputs->yprU.y(), obs->attitudeOutputs->yprU.z());
                        continue;
                    }
                }
                // Group 6 (INS)
                if (_binaryOutputRegister.insField != vn::protocol::uart::InsGroup::INSGROUP_NONE)
                {
                    if (!obs->insOutputs) { obs->insOutputs = std::make_shared<NAV::vendor::vectornav::InsOutputs>(); }

                    if (column == "INS::InsStatus::Mode") // "INS::InsStatus::GpsFix", "INS::InsStatus::Error::IMU", "INS::InsStatus::Error::MagPres", "INS::InsStatus::Error::GNSS", "INS::InsStatus::GpsHeadingIns", "INS::InsStatus::GpsCompass"
                    {
                        uint8_t mode{};
                        uint8_t gpsFix{};
                        uint8_t errorImu{};
                        uint8_t errorMagPres{};
                        uint8_t errorGnss{};
                        uint8_t gpsHeadingIns{};
                        uint8_t gpsCompass{};
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS,
                                     mode, gpsFix, errorImu, errorMagPres, errorGnss, gpsHeadingIns, gpsCompass);
                        obs->insOutputs->insStatus.status() = static_cast<uint16_t>(mode << 0U | gpsFix << 2U
                                                                                    | errorImu << 4U | errorMagPres << 5U | errorGnss << 6U
                                                                                    | gpsHeadingIns << 8U | gpsCompass << 9U);
                        continue;
                    }
                    if (column == "INS::PosLla::latitude [deg]") // "INS::PosLla::longitude [deg]", "INS::PosLla::altitude [m]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSLLA,
                                     obs->insOutputs->posLla.x(), obs->insOutputs->posLla.y(), obs->insOutputs->posLla.z());
                        continue;
                    }
                    if (column == "INS::PosEcef::X [m]") // "INS::PosEcef::Y [m]", "INS::PosEcef::Z [m]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSECEF,
                                     obs->insOutputs->posEcef.x(), obs->insOutputs->posEcef.y(), obs->insOutputs->posEcef.z());
                        continue;
                    }
                    if (column == "INS::VelBody::X [m/s]") // "INS::VelBody::Y [m/s]", "INS::VelBody::Z [m/s]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELBODY,
                                     obs->insOutputs->velBody.x(), obs->insOutputs->velBody.y(), obs->insOutputs->velBody.z());
                        continue;
                    }
                    if (column == "INS::VelNed::N [m/s]") // "INS::VelNed::E [m/s]", "INS::VelNed::D [m/s]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELNED,
                                     obs->insOutputs->velNed.x(), obs->insOutputs->velNed.y(), obs->insOutputs->velNed.z());
                        continue;
                    }
                    if (column == "INS::VelEcef::X [m/s]") // "INS::VelEcef::Y [m/s]", "INS::VelEcef::Z [m/s]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELECEF,
                                     obs->insOutputs->velEcef.x(), obs->insOutputs->velEcef.y(), obs->insOutputs->velEcef.z());
                        continue;
                    }
                    if (column == "INS::MagEcef::X [Gauss]") // "INS::MagEcef::Y [Gauss]", "INS::MagEcef::Z [Gauss]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_MAGECEF,
                                     obs->insOutputs->magEcef.x(), obs->insOutputs->magEcef.y(), obs->insOutputs->magEcef.z());
                        continue;
                    }
                    if (column == "INS::AccelEcef::X [m/s^2]") // "INS::AccelEcef::Y [m/s^2]", "INS::AccelEcef::Z [m/s^2]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF,
                                     obs->insOutputs->accelEcef.x(), obs->insOutputs->accelEcef.y(), obs->insOutputs->accelEcef.z());
                        continue;
                    }
                    if (column == "INS::LinearAccelEcef::X [m/s^2]") // "INS::LinearAccelEcef::Y [m/s^2]", "INS::LinearAccelEcef::Z [m/s^2]"
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF,
                                     obs->insOutputs->linearAccelEcef.x(), obs->insOutputs->linearAccelEcef.y(), obs->insOutputs->linearAccelEcef.z());
                        continue;
                    }
                    if (column == "INS::PosU [m]")
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSU, obs->insOutputs->posU);
                        continue;
                    }
                    if (column == "INS::VelU [m/s]")
                    {
                        extractValue(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELU, obs->insOutputs->velU);
                        continue;
                    }
                }
                // Group 7 (GNSS2)
                if (_binaryOutputRegister.gps2Field != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
                {
                    if (!obs->gnss2Outputs) { obs->gnss2Outputs = std::make_shared<NAV::vendor::vectornav::GnssOutputs>(); }

                    if (column == "GNSS2::UTC::year") // "GNSS2::UTC::month", "GNSS2::UTC::day", "GNSS2::UTC::hour", "GNSS2::UTC::min", "GNSS2::UTC::sec", "GNSS2::UTC::ms"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_UTC,
                                     obs->gnss2Outputs->timeUtc.year, obs->gnss2Outputs->timeUtc.month, obs->gnss2Outputs->timeUtc.day,
                                     obs->gnss2Outputs->timeUtc.hour, obs->gnss2Outputs->timeUtc.min, obs->gnss2Outputs->timeUtc.sec, obs->gnss2Outputs->timeUtc.ms);
                        continue;
                    }
                    if (column == "GNSS2::Tow [ns]")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW, obs->gnss2Outputs->tow);
                        continue;
                    }
                    if (column == "GNSS2::Week")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK, obs->gnss2Outputs->week);
                        continue;
                    }
                    if (column == "GNSS2::NumSats")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS, obs->gnss2Outputs->numSats);
                        continue;
                    }
                    if (column == "GNSS2::Fix")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX, obs->gnss2Outputs->fix);
                        continue;
                    }
                    if (column == "GNSS2::PosLla::latitude [deg]") // "GNSS2::PosLla::longitude [deg]", "GNSS2::PosLla::altitude [m]"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA,
                                     obs->gnss2Outputs->posLla.x(), obs->gnss2Outputs->posLla.y(), obs->gnss2Outputs->posLla.z());
                        continue;
                    }
                    if (column == "GNSS2::PosEcef::X [m]") // "GNSS2::PosEcef::Y [m]", "GNSS2::PosEcef::Z [m]"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF,
                                     obs->gnss2Outputs->posEcef.x(), obs->gnss2Outputs->posEcef.y(), obs->gnss2Outputs->posEcef.z());
                        continue;
                    }
                    if (column == "GNSS2::VelNed::N [m/s]") // "GNSS2::VelNed::E [m/s]", "GNSS2::VelNed::D [m/s]"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELNED,
                                     obs->gnss2Outputs->velNed.x(), obs->gnss2Outputs->velNed.y(), obs->gnss2Outputs->velNed.z());
                        continue;
                    }
                    if (column == "GNSS2::VelEcef::X [m/s]") // "GNSS2::VelEcef::Y [m/s]", "GNSS2::VelEcef::Z [m/s]"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF,
                                     obs->gnss2Outputs->velEcef.x(), obs->gnss2Outputs->velEcef.y(), obs->gnss2Outputs->velEcef.z());
                        continue;
                    }
                    if (column == "GNSS2::PosU::N [m]") // "GNSS2::PosU::E [m]", "GNSS2::PosU::D [m]"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSU,
                                     obs->gnss2Outputs->posU.x(), obs->gnss2Outputs->posU.y(), obs->gnss2Outputs->posU.z());
                        continue;
                    }
                    if (column == "GNSS2::VelU [m/s]")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELU, obs->gnss2Outputs->velU);
                        continue;
                    }
                    if (column == "GNSS2::TimeU [s]")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU, obs->gnss2Outputs->timeU);
                        continue;
                    }
                    if (column == "GNSS2::TimeInfo::Status::timeOk") // "GNSS2::TimeInfo::Status::dateOk", "GNSS2::TimeInfo::Status::utcTimeValid", "GNSS2::TimeInfo::LeapSeconds"
                    {
                        uint8_t timeOk{};
                        uint8_t dateOk{};
                        uint8_t utcTimeValid{};
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO, timeOk, dateOk, utcTimeValid, obs->gnss2Outputs->timeInfo.leapSeconds);
                        obs->gnss2Outputs->timeInfo.status = static_cast<uint8_t>(timeOk << 0U | dateOk << 1U | utcTimeValid << 2U);
                        continue;
                    }
                    if (column == "GNSS2::DOP::g") // "GNSS2::DOP::p","GNSS2::DOP::t","GNSS2::DOP::v","GNSS2::DOP::h","GNSS2::DOP::n","GNSS2::DOP::e"
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_DOP,
                                     obs->gnss2Outputs->dop.gDop, obs->gnss2Outputs->dop.pDop, obs->gnss2Outputs->dop.tDop, obs->gnss2Outputs->dop.vDop,
                                     obs->gnss2Outputs->dop.hDop, obs->gnss2Outputs->dop.nDop, obs->gnss2Outputs->dop.eDop);
                        continue;
                    }
                    if (column == "GNSS2::SatInfo::NumSats")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO, obs->gnss2Outputs->satInfo.numSats);
                        continue;
                    }
                    if (column == "GNSS2::SatInfo::Satellites")
                    {
                        std::string satellites;
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO, satellites);

                        for (size_t i = 0; i < obs->gnss2Outputs->satInfo.numSats; i++)
                        {
                            satellites = satellites.substr(1); // Remove leading '['
                            auto sys = static_cast<int8_t>(std::stoi(extractRemoveTillDelimiter(satellites, "|")));
                            auto svId = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagHealthy = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagAlmanac = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagEphemeris = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagDifferentialCorrection = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagUsedForNavigation = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagAzimuthElevationValid = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagUsedForRTK = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flags = static_cast<uint8_t>(flagHealthy << 0U
                                                              | flagAlmanac << 1U
                                                              | flagEphemeris << 2U
                                                              | flagDifferentialCorrection << 3U
                                                              | flagUsedForNavigation << 4U
                                                              | flagAzimuthElevationValid << 5U
                                                              | flagUsedForRTK << 6U);
                            auto cno = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto qi = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto el = static_cast<int8_t>(std::stoi(extractRemoveTillDelimiter(satellites, "|")));
                            auto az = static_cast<int16_t>(std::stoi(extractRemoveTillDelimiter(satellites, "]")));
                            obs->gnss2Outputs->satInfo.satellites.emplace_back(sys, svId, flags, cno, qi, el, az);
                        }
                        continue;
                    }
                    if (column.starts_with("GNSS2::SatInfo::") && column.ends_with(" - flag Healthy"))
                    {
                        std::string columnText = column.substr(16);
                        SatId satId(extractRemoveTillDelimiter(columnText, " -"));
                        LOG_DATA("{}:  SatInfo {}", nameId(), satId);
                        bool flagHealthy{};
                        bool flagAlmanac{};
                        bool flagEphemeris{};
                        bool flagDifferentialCorrection{};
                        bool flagUsedForNavigation{};
                        bool flagAzimuthElevationValid{};
                        bool flagUsedForRTK{};
                        auto cno = std::numeric_limits<uint8_t>::max();
                        uint8_t qi{};
                        int8_t el{};
                        int16_t az{};
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO,
                                     flagHealthy, flagAlmanac, flagEphemeris, flagDifferentialCorrection,
                                     flagUsedForNavigation, flagAzimuthElevationValid, flagUsedForRTK,
                                     cno, qi, el, az);
                        if (cno == std::numeric_limits<uint8_t>::max())
                        {
                            LOG_DATA("{}:   Skipping {} as empty", nameId(), satId);
                            continue;
                        }
                        auto flags = static_cast<uint8_t>(flagHealthy << 0U
                                                          | flagAlmanac << 1U
                                                          | flagEphemeris << 2U
                                                          | flagDifferentialCorrection << 3U
                                                          | flagUsedForNavigation << 4U
                                                          | flagAzimuthElevationValid << 5U
                                                          | flagUsedForRTK << 6U);
                        obs->gnss2Outputs->satInfo.satellites.emplace_back(static_cast<uint8_t>(vendor::vectornav::fromSatelliteSystem(satId.satSys)),
                                                                           static_cast<uint8_t>(satId.satNum), flags, cno, qi, el, az);
                        continue;
                    }
                    if (column == "GNSS2::RawMeas::Tow [s]")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, obs->gnss2Outputs->raw.tow);
                        continue;
                    }
                    if (column == "GNSS2::RawMeas::Week")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, obs->gnss2Outputs->raw.week);
                        continue;
                    }
                    if (column == "GNSS2::RawMeas::NumSats")
                    {
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, obs->gnss2Outputs->raw.numSats);
                        LOG_DATA("{}: GNSS2::NumSats: {}", nameId(), obs->gnss2Outputs->raw.numSats); // HACK
                        continue;
                    }
                    if (column == "GNSS2::RawMeas::Satellites")
                    {
                        std::string satellites;
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, satellites);
                        LOG_DATA("{}: numSats: {}", nameId(), obs->gnss2Outputs->raw.numSats); // HACK
                        LOG_DATA("{}: satellites: {}", nameId(), satellites);                  // HACK

                        for (size_t i = 0; i < obs->gnss2Outputs->raw.numSats; i++)
                        {
                            satellites = satellites.substr(1); // Remove leading '['
                            auto sys = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto svId = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto freq = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto chan = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto slot = static_cast<int8_t>(std::stoi(extractRemoveTillDelimiter(satellites, "|")));
                            auto cno = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagSearching = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagTracking = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagTimeValid = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagCodeLock = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseLock = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseHalfAmbiguity = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseHalfSub = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPhaseSlip = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flagPseudorangeSmoothed = static_cast<uint8_t>(std::stoul(extractRemoveTillDelimiter(satellites, "|")));
                            auto flags = static_cast<uint16_t>(flagSearching << 0U
                                                               | flagTracking << 1U
                                                               | flagTimeValid << 2U
                                                               | flagCodeLock << 3U
                                                               | flagPhaseLock << 4U
                                                               | flagPhaseHalfAmbiguity << 5U
                                                               | flagPhaseHalfSub << 6U
                                                               | flagPhaseSlip << 7U
                                                               | flagPseudorangeSmoothed << 8U);
                            auto pr = std::stod(extractRemoveTillDelimiter(satellites, "|"));
                            auto cp = std::stod(extractRemoveTillDelimiter(satellites, "|"));
                            auto dp = std::stof(extractRemoveTillDelimiter(satellites, "]"));
                            obs->gnss2Outputs->raw.satellites.emplace_back(sys, svId, freq, chan, slot, cno, flags, pr, cp, dp);
                        }
                        continue;
                    }
                    if (column.starts_with("GNSS2::RawMeas::") && column.ends_with(" - freq"))
                    {
                        std::string columnText = column.substr(16);
                        auto identifier = extractRemoveTillDelimiter(columnText, " - freq");
                        LOG_DATA("{}:  RawMeas {}", nameId(), identifier);
                        SatelliteSystem satSys;
                        uint16_t satNum{};
                        if (identifier.length() == 6)
                        {
                            SatSigId satSigId(identifier);
                            satSys = satSigId.toSatId().satSys;
                            satNum = satSigId.satNum;
                        }
                        else
                        {
                            SatId satId(identifier);
                            satSys = satId.satSys;
                            satNum = satId.satNum;
                        }
                        auto freq = std::numeric_limits<uint8_t>::max();
                        uint8_t chan{};
                        int8_t slot{};
                        uint8_t cno{};
                        uint8_t flagSearching{};
                        uint8_t flagTracking{};
                        uint8_t flagTimeValid{};
                        uint8_t flagCodeLock{};
                        uint8_t flagPhaseLock{};
                        uint8_t flagPhaseHalfAmbiguity{};
                        uint8_t flagPhaseHalfSub{};
                        uint8_t flagPhaseSlip{};
                        uint8_t flagPseudorangeSmoothed{};
                        double pr{};
                        double cp{};
                        float dp{};
                        extractValue(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS,
                                     freq, chan, slot, cno,
                                     flagSearching, flagTracking, flagTimeValid, flagCodeLock, flagPhaseLock,
                                     flagPhaseHalfAmbiguity, flagPhaseHalfSub, flagPhaseSlip, flagPseudorangeSmoothed,
                                     pr, cp, dp);
                        if (freq == std::numeric_limits<uint8_t>::max())
                        {
                            LOG_DATA("{}:   Skipping {} as empty", nameId(), identifier);
                            continue;
                        }
                        auto flags = static_cast<uint16_t>(flagSearching << 0U
                                                           | flagTracking << 1U
                                                           | flagTimeValid << 2U
                                                           | flagCodeLock << 3U
                                                           | flagPhaseLock << 4U
                                                           | flagPhaseHalfAmbiguity << 5U
                                                           | flagPhaseHalfSub << 6U
                                                           | flagPhaseSlip << 7U
                                                           | flagPseudorangeSmoothed << 8U);
                        obs->gnss2Outputs->raw.satellites.emplace_back(static_cast<uint8_t>(vendor::vectornav::fromSatelliteSystem(satSys)),
                                                                       static_cast<uint8_t>(satNum), freq, chan, slot, cno, flags, pr, cp, dp);
                        if (obs->gnss2Outputs->raw.satellites.back().toCode() == Code::None)
                        {
                            LOG_ERROR("{}: Line {}: Could not convert to valid Code. Skipping item. (sys {}, freq {}, chan {})", nameId(), _messageCount + 1,
                                      obs->gnss2Outputs->raw.satellites.back().sys, obs->gnss2Outputs->raw.satellites.back().freq, obs->gnss2Outputs->raw.satellites.back().chan);
                            obs->gnss2Outputs->raw.satellites.pop_back();
                        }
                        if (identifier.length() == 6 && SatSigId(identifier) != obs->gnss2Outputs->raw.satellites.back().toSatSigId())
                        {
                            LOG_ERROR("{}: Line {}: Could not convert to valid SatSigId. Skipping item. (sys {}, freq {}, chan {} = [{}], column was [{}])", nameId(), _messageCount + 1,
                                      obs->gnss2Outputs->raw.satellites.back().sys, obs->gnss2Outputs->raw.satellites.back().freq, obs->gnss2Outputs->raw.satellites.back().chan,
                                      obs->gnss2Outputs->raw.satellites.back().toSatSigId(), identifier);
                            obs->gnss2Outputs->raw.satellites.pop_back();
                        }
                        continue;
                    }
                }

                extractCell();
            }
            if (obs->gnss1Outputs) { obs->gnss1Outputs->raw.numSats = static_cast<uint8_t>(obs->gnss1Outputs->raw.satellites.size()); }
            if (obs->gnss2Outputs) { obs->gnss2Outputs->raw.numSats = static_cast<uint8_t>(obs->gnss2Outputs->raw.satellites.size()); }
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("{}: Could not read line {} completely: {}", nameId(), _messageCount + 1, e.what());
            return nullptr;
        }
    }
    else // if (fileType == FileType::BINARY)
    {
        auto readFromFilestream = [&, this](char* __s, std::streamsize __n) {
            read(__s, __n);
            if (!good())
            {
                throw std::runtime_error("End of file reached");
            }
        };

        try
        {
            _messageCount++;
            LOG_DATA("{}: Reading message {}", nameId(), _messageCount);
            int32_t gpsCycle = 0;
            int32_t gpsWeek = 0;
            double tow = 0.0;
            readFromFilestream(reinterpret_cast<char*>(&gpsCycle), sizeof(gpsCycle));
            readFromFilestream(reinterpret_cast<char*>(&gpsWeek), sizeof(gpsWeek));
            readFromFilestream(reinterpret_cast<char*>(&tow), sizeof(tow));
            if (gpsCycle || gpsWeek)
            {
                obs->insTime = InsTime(gpsCycle, gpsWeek, tow);
            }

            // Group 2 (Time)
            if (_binaryOutputRegister.timeField != vn::protocol::uart::TimeGroup::TIMEGROUP_NONE)
            {
                if (!obs->timeOutputs)
                {
                    obs->timeOutputs = std::make_shared<NAV::vendor::vectornav::TimeOutputs>();
                    obs->timeOutputs->timeField |= _binaryOutputRegister.timeField;
                }

                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeStartup), sizeof(obs->timeOutputs->timeStartup));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeGps), sizeof(obs->timeOutputs->timeGps));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->gpsTow), sizeof(obs->timeOutputs->gpsTow));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->gpsWeek), sizeof(obs->timeOutputs->gpsWeek));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeSyncIn), sizeof(obs->timeOutputs->timeSyncIn));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timePPS), sizeof(obs->timeOutputs->timePPS));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.year), sizeof(obs->timeOutputs->timeUtc.year));
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.month), sizeof(obs->timeOutputs->timeUtc.month));
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.day), sizeof(obs->timeOutputs->timeUtc.day));
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.hour), sizeof(obs->timeOutputs->timeUtc.hour));
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.min), sizeof(obs->timeOutputs->timeUtc.min));
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.sec), sizeof(obs->timeOutputs->timeUtc.sec));
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeUtc.ms), sizeof(obs->timeOutputs->timeUtc.ms));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->syncInCnt), sizeof(obs->timeOutputs->syncInCnt));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->syncOutCnt), sizeof(obs->timeOutputs->syncOutCnt));
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
                {
                    read(reinterpret_cast<char*>(&obs->timeOutputs->timeStatus.status()), sizeof(obs->timeOutputs->timeStatus.status()));
                }
            }
            // Group 3 (IMU)
            if (_binaryOutputRegister.imuField != vn::protocol::uart::ImuGroup::IMUGROUP_NONE)
            {
                if (!obs->imuOutputs)
                {
                    obs->imuOutputs = std::make_shared<NAV::vendor::vectornav::ImuOutputs>();
                    obs->imuOutputs->imuField |= _binaryOutputRegister.imuField;
                }

                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
                {
                    read(reinterpret_cast<char*>(&obs->imuOutputs->imuStatus), sizeof(obs->imuOutputs->imuStatus));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->uncompMag.data()), sizeof(obs->imuOutputs->uncompMag));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->uncompAccel.data()), sizeof(obs->imuOutputs->uncompAccel));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->uncompGyro.data()), sizeof(obs->imuOutputs->uncompGyro));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
                {
                    read(reinterpret_cast<char*>(&obs->imuOutputs->temp), sizeof(obs->imuOutputs->temp));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
                {
                    read(reinterpret_cast<char*>(&obs->imuOutputs->pres), sizeof(obs->imuOutputs->pres));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
                {
                    read(reinterpret_cast<char*>(&obs->imuOutputs->deltaTime), sizeof(obs->imuOutputs->deltaTime));
                    read(reinterpret_cast<char*>(obs->imuOutputs->deltaTheta.data()), sizeof(obs->imuOutputs->deltaTheta));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->deltaV.data()), sizeof(obs->imuOutputs->deltaV));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->mag.data()), sizeof(obs->imuOutputs->mag));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->accel.data()), sizeof(obs->imuOutputs->accel));
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
                {
                    read(reinterpret_cast<char*>(obs->imuOutputs->angularRate.data()), sizeof(obs->imuOutputs->angularRate));
                }
            }
            // Group 4 (GNSS1)
            if (_binaryOutputRegister.gpsField != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
            {
                if (!obs->gnss1Outputs)
                {
                    obs->gnss1Outputs = std::make_shared<NAV::vendor::vectornav::GnssOutputs>();
                    obs->gnss1Outputs->gnssField |= _binaryOutputRegister.gpsField;
                }

                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.year), sizeof(obs->gnss1Outputs->timeUtc.year));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.month), sizeof(obs->gnss1Outputs->timeUtc.month));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.day), sizeof(obs->gnss1Outputs->timeUtc.day));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.hour), sizeof(obs->gnss1Outputs->timeUtc.hour));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.min), sizeof(obs->gnss1Outputs->timeUtc.min));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.sec), sizeof(obs->gnss1Outputs->timeUtc.sec));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeUtc.ms), sizeof(obs->gnss1Outputs->timeUtc.ms));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->tow), sizeof(obs->gnss1Outputs->tow));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->week), sizeof(obs->gnss1Outputs->week));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->numSats), sizeof(obs->gnss1Outputs->numSats));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->fix), sizeof(obs->gnss1Outputs->fix));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    read(reinterpret_cast<char*>(obs->gnss1Outputs->posLla.data()), sizeof(obs->gnss1Outputs->posLla));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    read(reinterpret_cast<char*>(obs->gnss1Outputs->posEcef.data()), sizeof(obs->gnss1Outputs->posEcef));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    read(reinterpret_cast<char*>(obs->gnss1Outputs->velNed.data()), sizeof(obs->gnss1Outputs->velNed));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    read(reinterpret_cast<char*>(obs->gnss1Outputs->velEcef.data()), sizeof(obs->gnss1Outputs->velEcef));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    read(reinterpret_cast<char*>(obs->gnss1Outputs->posU.data()), sizeof(obs->gnss1Outputs->posU));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->velU), sizeof(obs->gnss1Outputs->velU));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeU), sizeof(obs->gnss1Outputs->timeU));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeInfo.status.status()), sizeof(obs->gnss1Outputs->timeInfo.status.status()));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->timeInfo.leapSeconds), sizeof(obs->gnss1Outputs->timeInfo.leapSeconds));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.gDop), sizeof(obs->gnss1Outputs->dop.gDop));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.pDop), sizeof(obs->gnss1Outputs->dop.pDop));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.tDop), sizeof(obs->gnss1Outputs->dop.tDop));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.vDop), sizeof(obs->gnss1Outputs->dop.vDop));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.hDop), sizeof(obs->gnss1Outputs->dop.hDop));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.nDop), sizeof(obs->gnss1Outputs->dop.nDop));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->dop.eDop), sizeof(obs->gnss1Outputs->dop.eDop));
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->satInfo.numSats), sizeof(obs->gnss1Outputs->satInfo.numSats));
                    obs->gnss1Outputs->satInfo.satellites.resize(obs->gnss1Outputs->satInfo.numSats);

                    for (auto& satellite : obs->gnss1Outputs->satInfo.satellites)
                    {
                        read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        read(reinterpret_cast<char*>(&satellite.qi), sizeof(satellite.qi));
                        read(reinterpret_cast<char*>(&satellite.el), sizeof(satellite.el));
                        read(reinterpret_cast<char*>(&satellite.az), sizeof(satellite.az));
                    }
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->raw.tow), sizeof(obs->gnss1Outputs->raw.tow));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->raw.week), sizeof(obs->gnss1Outputs->raw.week));
                    read(reinterpret_cast<char*>(&obs->gnss1Outputs->raw.numSats), sizeof(obs->gnss1Outputs->raw.numSats));
                    obs->gnss1Outputs->raw.satellites.resize(obs->gnss1Outputs->raw.numSats);

                    for (auto& satellite : obs->gnss1Outputs->raw.satellites)
                    {
                        read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        read(reinterpret_cast<char*>(&satellite.freq), sizeof(satellite.freq));
                        read(reinterpret_cast<char*>(&satellite.chan), sizeof(satellite.chan));
                        read(reinterpret_cast<char*>(&satellite.slot), sizeof(satellite.slot));
                        read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        read(reinterpret_cast<char*>(&satellite.pr), sizeof(satellite.pr));
                        read(reinterpret_cast<char*>(&satellite.cp), sizeof(satellite.cp));
                        read(reinterpret_cast<char*>(&satellite.dp), sizeof(satellite.dp));
                    }
                }
            }
            // Group 5 (Attitude)
            if (_binaryOutputRegister.attitudeField != vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE)
            {
                if (!obs->attitudeOutputs)
                {
                    obs->attitudeOutputs = std::make_shared<NAV::vendor::vectornav::AttitudeOutputs>();
                    obs->attitudeOutputs->attitudeField |= _binaryOutputRegister.attitudeField;
                }

                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
                {
                    read(reinterpret_cast<char*>(&obs->attitudeOutputs->vpeStatus.status()), sizeof(obs->attitudeOutputs->vpeStatus.status()));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->ypr.data()), sizeof(obs->attitudeOutputs->ypr));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->qtn.coeffs().data()), sizeof(obs->attitudeOutputs->qtn));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->dcm.data()), sizeof(obs->attitudeOutputs->dcm));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->magNed.data()), sizeof(obs->attitudeOutputs->magNed));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->accelNed.data()), sizeof(obs->attitudeOutputs->accelNed));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->linearAccelBody.data()), sizeof(obs->attitudeOutputs->linearAccelBody));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->linearAccelNed.data()), sizeof(obs->attitudeOutputs->linearAccelNed));
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
                {
                    read(reinterpret_cast<char*>(obs->attitudeOutputs->yprU.data()), sizeof(obs->attitudeOutputs->yprU));
                }
            }
            // Group 6 (INS)
            if (_binaryOutputRegister.insField != vn::protocol::uart::InsGroup::INSGROUP_NONE)
            {
                if (!obs->insOutputs)
                {
                    obs->insOutputs = std::make_shared<NAV::vendor::vectornav::InsOutputs>();
                    obs->insOutputs->insField |= _binaryOutputRegister.insField;
                }

                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
                {
                    read(reinterpret_cast<char*>(&obs->insOutputs->insStatus.status()), sizeof(obs->insOutputs->insStatus.status()));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->posLla.data()), sizeof(obs->insOutputs->posLla));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->posEcef.data()), sizeof(obs->insOutputs->posEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->velBody.data()), sizeof(obs->insOutputs->velBody));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->velNed.data()), sizeof(obs->insOutputs->velNed));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->velEcef.data()), sizeof(obs->insOutputs->velEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->magEcef.data()), sizeof(obs->insOutputs->magEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->accelEcef.data()), sizeof(obs->insOutputs->accelEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
                {
                    read(reinterpret_cast<char*>(obs->insOutputs->linearAccelEcef.data()), sizeof(obs->insOutputs->linearAccelEcef));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
                {
                    read(reinterpret_cast<char*>(&obs->insOutputs->posU), sizeof(obs->insOutputs->posU));
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
                {
                    read(reinterpret_cast<char*>(&obs->insOutputs->velU), sizeof(obs->insOutputs->velU));
                }
            }
            // Group 7 (GNSS2)
            if (_binaryOutputRegister.gps2Field != vn::protocol::uart::GpsGroup::GPSGROUP_NONE)
            {
                if (!obs->gnss2Outputs)
                {
                    obs->gnss2Outputs = std::make_shared<NAV::vendor::vectornav::GnssOutputs>();
                    obs->gnss2Outputs->gnssField |= _binaryOutputRegister.gps2Field;
                }

                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.year), sizeof(obs->gnss2Outputs->timeUtc.year));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.month), sizeof(obs->gnss2Outputs->timeUtc.month));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.day), sizeof(obs->gnss2Outputs->timeUtc.day));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.hour), sizeof(obs->gnss2Outputs->timeUtc.hour));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.min), sizeof(obs->gnss2Outputs->timeUtc.min));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.sec), sizeof(obs->gnss2Outputs->timeUtc.sec));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeUtc.ms), sizeof(obs->gnss2Outputs->timeUtc.ms));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->tow), sizeof(obs->gnss2Outputs->tow));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->week), sizeof(obs->gnss2Outputs->week));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->numSats), sizeof(obs->gnss2Outputs->numSats));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->fix), sizeof(obs->gnss2Outputs->fix));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    read(reinterpret_cast<char*>(obs->gnss2Outputs->posLla.data()), sizeof(obs->gnss2Outputs->posLla));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    read(reinterpret_cast<char*>(obs->gnss2Outputs->posEcef.data()), sizeof(obs->gnss2Outputs->posEcef));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    read(reinterpret_cast<char*>(obs->gnss2Outputs->velNed.data()), sizeof(obs->gnss2Outputs->velNed));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    read(reinterpret_cast<char*>(obs->gnss2Outputs->velEcef.data()), sizeof(obs->gnss2Outputs->velEcef));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    read(reinterpret_cast<char*>(obs->gnss2Outputs->posU.data()), sizeof(obs->gnss2Outputs->posU));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->velU), sizeof(obs->gnss2Outputs->velU));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeU), sizeof(obs->gnss2Outputs->timeU));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeInfo.status.status()), sizeof(obs->gnss2Outputs->timeInfo.status.status()));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->timeInfo.leapSeconds), sizeof(obs->gnss2Outputs->timeInfo.leapSeconds));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.gDop), sizeof(obs->gnss2Outputs->dop.gDop));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.pDop), sizeof(obs->gnss2Outputs->dop.pDop));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.tDop), sizeof(obs->gnss2Outputs->dop.tDop));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.vDop), sizeof(obs->gnss2Outputs->dop.vDop));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.hDop), sizeof(obs->gnss2Outputs->dop.hDop));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.nDop), sizeof(obs->gnss2Outputs->dop.nDop));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->dop.eDop), sizeof(obs->gnss2Outputs->dop.eDop));
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->satInfo.numSats), sizeof(obs->gnss2Outputs->satInfo.numSats));
                    obs->gnss2Outputs->satInfo.satellites.resize(obs->gnss2Outputs->satInfo.numSats);

                    for (auto& satellite : obs->gnss2Outputs->satInfo.satellites)
                    {
                        read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        read(reinterpret_cast<char*>(&satellite.qi), sizeof(satellite.qi));
                        read(reinterpret_cast<char*>(&satellite.el), sizeof(satellite.el));
                        read(reinterpret_cast<char*>(&satellite.az), sizeof(satellite.az));
                    }
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->raw.tow), sizeof(obs->gnss2Outputs->raw.tow));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->raw.week), sizeof(obs->gnss2Outputs->raw.week));
                    read(reinterpret_cast<char*>(&obs->gnss2Outputs->raw.numSats), sizeof(obs->gnss2Outputs->raw.numSats));
                    obs->gnss2Outputs->raw.satellites.resize(obs->gnss2Outputs->raw.numSats);

                    for (auto& satellite : obs->gnss2Outputs->raw.satellites)
                    {
                        read(reinterpret_cast<char*>(&satellite.sys), sizeof(satellite.sys));
                        read(reinterpret_cast<char*>(&satellite.svId), sizeof(satellite.svId));
                        read(reinterpret_cast<char*>(&satellite.freq), sizeof(satellite.freq));
                        read(reinterpret_cast<char*>(&satellite.chan), sizeof(satellite.chan));
                        read(reinterpret_cast<char*>(&satellite.slot), sizeof(satellite.slot));
                        read(reinterpret_cast<char*>(&satellite.cno), sizeof(satellite.cno));
                        read(reinterpret_cast<char*>(&satellite.flags), sizeof(satellite.flags));
                        read(reinterpret_cast<char*>(&satellite.pr), sizeof(satellite.pr));
                        read(reinterpret_cast<char*>(&satellite.cp), sizeof(satellite.cp));
                        read(reinterpret_cast<char*>(&satellite.dp), sizeof(satellite.dp));
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_DEBUG("{}: {} after {} messages", nameId(), e.what(), _messageCount);
            return nullptr;
        }
    }

    if (!obs->timeOutputs && !obs->imuOutputs && !obs->gnss1Outputs && !obs->attitudeOutputs && !obs->insOutputs && !obs->gnss2Outputs)
    {
        LOG_DATA("{}: Nothing read at message {}. Ending reading.", nameId(), _messageCount);
        return nullptr;
    }
    if (obs->insTime.empty())
    {
        LOG_DATA("{}: Skipping message {}, as no InsTime set.", nameId(), _messageCount);
        return obs;
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_VECTORNAV_BINARY_OUTPUT, obs);
    return obs;
}