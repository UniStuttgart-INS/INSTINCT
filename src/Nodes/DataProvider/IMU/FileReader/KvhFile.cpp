#include "KvhFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <algorithm>

#include "util/UartSensors/KVH/KvhUtilities.hpp"

NAV::KvhFile::KvhFile(const std::string& name, const std::map<std::string, std::string>& options)
    : FileReader(name, options), Imu(name, options), sensor(name)
{
    LOG_TRACE("called for {}", name);

    fileType = determineFileType();

    readHeader();

    dataStart = filestream.tellg();

    if (fileType == FileType::ASCII)
    {
        LOG_DEBUG("{}-ASCII-File successfully initialized", name);
    }
    else
    {
        LOG_DEBUG("{}-Binary-File successfully initialized", name);
    }
}

void NAV::KvhFile::resetNode()
{
    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}

std::shared_ptr<NAV::KvhObs> NAV::KvhFile::pollData(bool peek)
{
    std::shared_ptr<KvhObs> obs = nullptr;

    // Get current position
    auto pos = filestream.tellg();

    if (fileType == FileType::BINARY)
    {
        uint8_t i = 0;
        std::unique_ptr<uart::protocol::Packet> packet = nullptr;
        while (filestream.readsome(reinterpret_cast<char*>(&i), 1))
        {
            packet = sensor.findPacket(i);

            if (packet != nullptr)
            {
                break;
            }
        }

        if (!packet)
        {
            return nullptr;
        }

        obs = std::make_shared<KvhObs>(*packet);

        // Check if package is empty
        if (obs->raw.getRawDataLength() == 0)
        {
            return nullptr;
        }

        sensors::kvh::decryptKvhObs(obs);
    }
    else if (fileType == FileType::ASCII)
    {
        obs = std::make_shared<KvhObs>();

        // Read line
        std::string line;
        std::getline(filestream, line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

        if (line.empty())
        {
            return nullptr;
        }

        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;

        std::optional<uint16_t> gpsCycle;
        std::optional<uint16_t> gpsWeek;
        std::optional<long double> gpsToW;

        // Split line at comma
        for (const auto& column : columns)
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

                    if (!obs->insTime.has_value() && gpsCycle.has_value() && gpsWeek.has_value() && gpsToW.has_value())
                    {
                        obs->insTime.emplace(gpsWeek.value(), gpsToW.value(), gpsCycle.value());
                    }
                }
                else if (column == "GpsWeek")
                {
                    gpsWeek = static_cast<uint16_t>(std::stoul(cell));

                    if (!obs->insTime.has_value() && gpsCycle.has_value() && gpsWeek.has_value() && gpsToW.has_value())
                    {
                        obs->insTime.emplace(gpsWeek.value(), gpsToW.value(), gpsCycle.value());
                    }
                }
                else if (column == "GpsToW")
                {
                    gpsToW = std::stold(cell);

                    if (!obs->insTime.has_value() && gpsCycle.has_value() && gpsWeek.has_value() && gpsToW.has_value())
                    {
                        obs->insTime.emplace(gpsWeek.value(), gpsToW.value(), gpsCycle.value());
                    }
                }
                else if (column == "TimeStartup")
                {
                    obs->timeSinceStartup.emplace(std::stoull(cell));
                }
                else if (column == "UnCompMagX")
                {
                    if (!obs->magUncompXYZ.has_value())
                    {
                        obs->magUncompXYZ = Eigen::Vector3d();
                    }
                    obs->magUncompXYZ.value().x() = std::stod(cell);
                }
                else if (column == "UnCompMagY")
                {
                    if (!obs->magUncompXYZ.has_value())
                    {
                        obs->magUncompXYZ = Eigen::Vector3d();
                    }
                    obs->magUncompXYZ.value().y() = std::stod(cell);
                }
                else if (column == "UnCompMagZ")
                {
                    if (!obs->magUncompXYZ.has_value())
                    {
                        obs->magUncompXYZ = Eigen::Vector3d();
                    }
                    obs->magUncompXYZ.value().z() = std::stod(cell);
                }
                else if (column == "UnCompAccX")
                {
                    if (!obs->accelUncompXYZ.has_value())
                    {
                        obs->accelUncompXYZ = Eigen::Vector3d();
                    }
                    obs->accelUncompXYZ.value().x() = std::stod(cell);
                }
                else if (column == "UnCompAccY")
                {
                    if (!obs->accelUncompXYZ.has_value())
                    {
                        obs->accelUncompXYZ = Eigen::Vector3d();
                    }
                    obs->accelUncompXYZ.value().y() = std::stod(cell);
                }
                else if (column == "UnCompAccZ")
                {
                    if (!obs->accelUncompXYZ.has_value())
                    {
                        obs->accelUncompXYZ = Eigen::Vector3d();
                    }
                    obs->accelUncompXYZ.value().z() = std::stod(cell);
                }
                else if (column == "UnCompGyroX")
                {
                    if (!obs->gyroUncompXYZ.has_value())
                    {
                        obs->gyroUncompXYZ = Eigen::Vector3d();
                    }
                    obs->gyroUncompXYZ.value().x() = std::stod(cell);
                }
                else if (column == "UnCompGyroY")
                {
                    if (!obs->gyroUncompXYZ.has_value())
                    {
                        obs->gyroUncompXYZ = Eigen::Vector3d();
                    }
                    obs->gyroUncompXYZ.value().y() = std::stod(cell);
                }
                else if (column == "UnCompGyroZ")
                {
                    if (!obs->gyroUncompXYZ.has_value())
                    {
                        obs->gyroUncompXYZ = Eigen::Vector3d();
                    }
                    obs->gyroUncompXYZ.value().z() = std::stod(cell);
                }
                else if (column == "Temperature")
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
    }

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
             name, obs->sequenceNumber, obs->temperature.value(),
             obs->accelUncompXYZ.value().x(), obs->accelUncompXYZ.value().y(), obs->accelUncompXYZ.value().z());

    // Check if a packet was skipped
    if (!peek && callbacksEnabled)
    {
        if (prevSequenceNumber == UINT8_MAX)
        {
            prevSequenceNumber = obs->sequenceNumber;
        }
        if (obs->sequenceNumber != 0 && (obs->sequenceNumber < prevSequenceNumber || obs->sequenceNumber > prevSequenceNumber + 1))
        {
            LOG_WARN("{}: Sequence Number changed from {} to {}", name, prevSequenceNumber, obs->sequenceNumber);
        }
        prevSequenceNumber = obs->sequenceNumber;
    }

    // Calls all the callbacks
    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(pos, std::ios_base::beg);
    }
    else
    {
        // Calls all the callbacks
        invokeCallbacks(obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::KvhFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    auto filestream = std::ifstream(path);
    if (filestream.good())
    {
        union
        {
            std::array<char, 4> buffer;
            uint32_t ui32;
        } un{};

        if (filestream.readsome(un.buffer.data(), sizeof(uint32_t)) == sizeof(uint32_t))
        {
            un.ui32 = uart::stoh(un.ui32, sensors::kvh::KvhUartSensor::endianness);
            if (un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_A
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_B
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_C
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_XBIT
                || un.ui32 == sensors::kvh::KvhUartSensor::HEADER_FMT_XBIT2)
            {
                return FileType::BINARY;
            }
        }

        filestream.seekg(0, std::ios_base::beg);
        std::string line;
        std::getline(filestream, line);
        filestream.close();

        auto n = std::count(line.begin(), line.end(), ',');

        if (n >= 3)
        {
            return FileType::ASCII;
        }

        LOG_CRITICAL("{} could not determine file type", name);
    }

    LOG_CRITICAL("{} could not open file {}", name, path);
    return FileType::NONE;
}

void NAV::KvhFile::readHeader()
{
    if (fileType == FileType::ASCII)
    {
        // Read header line
        std::string line;
        std::getline(filestream, line);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isalnum(ch); }));
        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;
        // Split line at comma
        while (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
            columns.push_back(cell);
        }
    }
}