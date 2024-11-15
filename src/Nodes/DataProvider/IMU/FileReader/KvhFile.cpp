// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KvhFile.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/Vendor/KVH/KvhUtilities.hpp"

#include "NodeData/IMU/KvhObs.hpp"

NAV::KvhFile::KvhFile()
    : Imu(typeStatic()), _sensor(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateOutputPin(this, "KvhObs", Pin::Type::Flow, { NAV::KvhObs::type() }, &KvhFile::pollData);
    nm::CreateOutputPin(this, "Header Columns", Pin::Type::Object, { "std::vector<std::string>" }, &_headerColumns);
}

NAV::KvhFile::~KvhFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::KvhFile::typeStatic()
{
    return "KvhFile";
}

std::string NAV::KvhFile::type() const
{
    return typeStatic();
}

std::string NAV::KvhFile::category()
{
    return "Data Provider";
}

void NAV::KvhFile::guiConfig()
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

    if (_fileType == FileType::ASCII)
    {
        // Header info
        if (ImGui::BeginTable(fmt::format("##VectorNavHeaders ({})", id.AsPointer()).c_str(), 2,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableSetupColumn("Basic", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableHeadersRow();

            auto TextColoredIfExists = [this](int index, const char* displayText, const char* searchText, bool alwaysNormal = false) {
                ImGui::TableSetColumnIndex(index);
                if (alwaysNormal || std::ranges::find(_headerColumns, searchText) != _headerColumns.end())
                {
                    ImGui::TextUnformatted(displayText);
                }
                else
                {
                    ImGui::TextDisabled("%s", displayText);
                }
            };

            ImGui::TableNextRow();
            TextColoredIfExists(0, "GpsTime", "GpsToW [s]");
            TextColoredIfExists(1, "UnCompMag", "UnCompMagX [Gauss]");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "TimeStartup", "TimeStartup [ns]");
            TextColoredIfExists(1, "UnCompAcc", "UnCompAccX [m/s^2]");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "Temperature", "Temperature [Celsius]");
            TextColoredIfExists(1, "UnCompGyro", "UnCompGyroX [rad/s]");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "Status", "Status");
            ImGui::TableNextRow();
            TextColoredIfExists(0, "SequenceNumber", "SequenceNumber");

            ImGui::EndTable();
        }
    }
    else if (_fileType == FileType::BINARY)
    {
        ImGui::TextUnformatted("Binary file");
    }
}

[[nodiscard]] json NAV::KvhFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::KvhFile::restore(json const& j)
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

bool NAV::KvhFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::KvhFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::KvhFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<const NAV::NodeData> NAV::KvhFile::pollData()
{
    std::shared_ptr<KvhObs> obs = nullptr;

    if (_fileType == FileType::BINARY)
    {
        uint8_t i = 0;
        std::unique_ptr<uart::protocol::Packet> packet = nullptr;
        while (!eof() && read(reinterpret_cast<char*>(&i), 1))
        {
            packet = _sensor.findPacket(i);

            if (packet != nullptr)
            {
                break;
            }
        }

        if (!packet)
        {
            return nullptr;
        }

        obs = std::make_shared<KvhObs>(_imuPos, *packet);

        // Check if package is empty
        if (obs->raw.getRawDataLength() == 0)
        {
            return nullptr;
        }

        vendor::kvh::decryptKvhObs(obs);
    }
    else if (_fileType == FileType::ASCII)
    {
        obs = std::make_shared<KvhObs>(_imuPos);

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

                if (column == "GpsCycle")
                {
                    gpsCycle = static_cast<uint16_t>(std::stoul(cell));
                }
                else if (column == "GpsWeek")
                {
                    gpsWeek = static_cast<uint16_t>(std::stoul(cell));
                }
                else if (column == "GpsToW [s]")
                {
                    gpsToW = std::stold(cell);
                }
                else if (column == "TimeStartup [ns]")
                {
                    obs->timeSinceStartup.emplace(std::stoull(cell));
                }
                else if (column == "MagX [Gauss]")
                {
                    magX = std::stod(cell);
                }
                else if (column == "MagY [Gauss]")
                {
                    magY = std::stod(cell);
                }
                else if (column == "MagZ [Gauss]")
                {
                    magZ = std::stod(cell);
                }
                else if (column == "AccX [m/s^2]")
                {
                    accelX = std::stod(cell);
                }
                else if (column == "AccY [m/s^2]")
                {
                    accelY = std::stod(cell);
                }
                else if (column == "AccZ [m/s^2]")
                {
                    accelZ = std::stod(cell);
                }
                else if (column == "GyroX [rad/s]")
                {
                    gyroX = std::stod(cell);
                }
                else if (column == "GyroY [rad/s]")
                {
                    gyroY = std::stod(cell);
                }
                else if (column == "GyroZ [rad/s]")
                {
                    gyroZ = std::stod(cell);
                }
                else if (column == "Temperature [Celsius]")
                {
                    obs->temperature.emplace(std::stod(cell));
                }
                else if (column == "Status")
                {
                    obs->status = std::bitset<8>{ cell };
                }
                else if (column == "SequenceNumber")
                {
                    obs->sequenceNumber = static_cast<uint8_t>(std::stoul(cell));
                }
            }
        }

        if (!gpsCycle || !gpsWeek || !gpsToW)
        {
            LOG_ERROR("{}: Fields 'GpsCycle', 'GpsWeek', 'GpsToW [s]' are needed.", nameId());
            return nullptr;
        }
        if (!accelX || !accelY || !accelZ)
        {
            LOG_ERROR("{}: Fields 'AccX [m/s^2]', 'AccY [m/s^2]', 'AccZ [m/s^2]' are needed.", nameId());
            return nullptr;
        }
        if (!gyroX || !gyroY || !gyroZ)
        {
            LOG_ERROR("{}: Fields 'GyroX [rad/s]', 'GyroY [rad/s]', 'GyroZ [rad/s]' are needed.", nameId());
            return nullptr;
        }
        obs->insTime = InsTime(gpsCycle.value(), gpsWeek.value(), gpsToW.value());
        obs->p_acceleration = { accelX.value(), accelY.value(), accelZ.value() };
        obs->p_angularRate = { gyroX.value(), gyroY.value(), gyroZ.value() };

        if (magX && magY && magZ)
        {
            obs->p_magneticField.emplace(magX.value(), magY.value(), magZ.value());
        }
    }

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}", name, obs->sequenceNumber, obs->temperature.value(),
             obs->p_acceleration.x(), obs->p_acceleration.y(), obs->p_acceleration.z());

    // Check if a packet was skipped
    if (callbacksEnabled)
    {
        if (_prevSequenceNumber == UINT8_MAX)
        {
            _prevSequenceNumber = obs->sequenceNumber;
        }
        if (obs->sequenceNumber != 0 && (obs->sequenceNumber < _prevSequenceNumber || obs->sequenceNumber > _prevSequenceNumber + 2))
        {
            LOG_WARN("{}: Sequence Number changed from {} to {}", name, _prevSequenceNumber, obs->sequenceNumber);
        }
        _prevSequenceNumber = obs->sequenceNumber;
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_KVH_OBS, obs);
    return obs;
}

NAV::FileReader::FileType NAV::KvhFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    auto filestream = std::ifstream(getFilepath());
    if (filestream.good())
    {
        union
        {
            std::array<char, 4> buffer;
            uint32_t ui32;
        } un{};

        if (filestream.readsome(un.buffer.data(), sizeof(uint32_t)) == sizeof(uint32_t))
        {
            un.ui32 = uart::stoh(un.ui32, vendor::kvh::KvhUartSensor::ENDIANNESS);
            if (un.ui32 == vendor::kvh::KvhUartSensor::HEADER_FMT_A
                || un.ui32 == vendor::kvh::KvhUartSensor::HEADER_FMT_B
                || un.ui32 == vendor::kvh::KvhUartSensor::HEADER_FMT_C
                || un.ui32 == vendor::kvh::KvhUartSensor::HEADER_FMT_XBIT
                || un.ui32 == vendor::kvh::KvhUartSensor::HEADER_FMT_XBIT2)
            {
                return FileType::BINARY;
            }
        }

        filestream.seekg(0, std::ios_base::beg);
        std::string line;
        std::getline(filestream, line);
        filestream.close();

        auto n = std::ranges::count(line, ',');

        if (n >= 3)
        {
            return FileType::ASCII;
        }

        LOG_ERROR("{} could not determine file type", name);
        return FileType::NONE;
    }

    LOG_ERROR("{} could not open file {}", name, getFilepath());
    return FileType::NONE;
}