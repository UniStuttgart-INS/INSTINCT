#include "ImuFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <algorithm>

NAV::ImuFile::ImuFile(const std::string& name, const std::map<std::string, std::string>& options)
    : FileReader(name, options), Imu(name, options)
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

void NAV::ImuFile::resetNode()
{
    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}

std::shared_ptr<NAV::ImuObs> NAV::ImuFile::pollData(bool peek)
{
    auto obs = std::make_shared<ImuObs>();

    if (fileType == FileType::BINARY)
    {
        // TODO: Implement ImuFile Binary reading
        LOG_CRITICAL("Binary ImuFile pollData is not implemented yet.");
    }
    // Ascii

    // Read line
    std::string line;
    // Get current position
    auto len = filestream.tellg();
    std::getline(filestream, line);
    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(len, std::ios_base::beg);
    }
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
        }
    }

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
             name, obs->timeSinceStartup.value(), obs->temperature.value(),
             obs->accelUncompXYZ.value().x(), obs->accelUncompXYZ.value().y(), obs->accelUncompXYZ.value().z());

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::ImuFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    auto filestream = std::ifstream(path);
    if (filestream.good())
    {
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

void NAV::ImuFile::readHeader()
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