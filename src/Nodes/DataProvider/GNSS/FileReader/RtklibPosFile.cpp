#include "RtklibPosFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <array>

NAV::RtklibPosFile::RtklibPosFile(const std::string& name, std::deque<std::string>& options)
    : FileReader(options), Gnss(name, options)
{
    LOG_TRACE("called for {}", name);

    fileType = determineFileType();

    filestream = std::ifstream(path);

    if (filestream.good())
    {
        // Read header line
        std::string line;
        do
        {
            std::getline(filestream, line);
            // Remove any starting non text characters
            line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                                  std::ptr_fun<int, int>(std::isgraph)));
        } while (!line.empty() && line.find("%  ") == std::string::npos);

        // Convert line into stream
        std::istringstream lineStream(line);

        for (std::string cell; lineStream >> cell;)
        {
            if (cell != "%")
            {
                if (cell == "GPST")
                {
                    columns.emplace_back("GpsWeek");
                    columns.emplace_back("GpsToW");
                }
                else
                {
                    columns.push_back(cell);
                }
            }
        }

        dataStart = filestream.tellg();

        LOG_DEBUG("{} successfully initialized", name);
    }
    else
    {
        LOG_CRITICAL("{} could not open file {}", name, path);
    }
}

NAV::RtklibPosFile::~RtklibPosFile()
{
    LOG_TRACE("called for {}", name);

    if (filestream.is_open())
    {
        filestream.close();
    }
}

void NAV::RtklibPosFile::resetNode()
{
    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}

std::shared_ptr<NAV::RtklibPosObs> NAV::RtklibPosFile::pollData(bool peek)
{
    auto obs = std::make_shared<RtklibPosObs>();
    // Get current position
    auto pos = filestream.tellg();

    // Read line
    std::string line;
    std::getline(filestream, line);
    // Remove any starting non text characters
    line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                          std::ptr_fun<int, int>(std::isgraph)));

    if (line.empty())
    {
        return nullptr;
    }

    std::istringstream lineStream(line);
    std::string cell;

    std::optional<uint16_t> gpsWeek;
    std::optional<long double> gpsToW;

    for (const auto& column : columns)
    {
        if (lineStream >> cell)
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(),
                                    std::ptr_fun<int, int>(std::iscntrl)),
                       cell.end());
            if (cell.empty())
            {
                continue;
            }

            if (column == "GpsWeek")
            {
                gpsWeek = static_cast<uint16_t>(std::stoul(cell));

                if (!obs->insTime.has_value() && gpsWeek.has_value() && gpsToW.has_value())
                {
                    obs->insTime.emplace(gpsWeek.value(), gpsToW.value(), 0);
                }
            }
            else if (column == "GpsToW")
            {
                gpsToW = std::stold(cell);

                if (!obs->insTime.has_value() && gpsWeek.has_value() && gpsToW.has_value())
                {
                    obs->insTime.emplace(gpsWeek.value(), gpsToW.value(), 0);
                }
            }
            else if (column == "x-ecef(m)")
            {
                if (!obs->positionXYZ.has_value())
                {
                    obs->positionXYZ = Eigen::Vector3d();
                }
                obs->positionXYZ.value().x() = std::stod(cell);
            }
            else if (column == "y-ecef(m)")
            {
                if (!obs->positionXYZ.has_value())
                {
                    obs->positionXYZ = Eigen::Vector3d();
                }
                obs->positionXYZ.value().y() = std::stod(cell);
            }
            else if (column == "z-ecef(m)")
            {
                if (!obs->positionXYZ.has_value())
                {
                    obs->positionXYZ = Eigen::Vector3d();
                }
                obs->positionXYZ.value().z() = std::stod(cell);
            }
            else if (column == "latitude(deg)")
            {
                if (!obs->positionLLH.has_value())
                {
                    obs->positionLLH = Eigen::Array3d();
                }
                obs->positionLLH.value()(0) = std::stod(cell);
            }
            else if (column == "longitude(deg)")
            {
                if (!obs->positionLLH.has_value())
                {
                    obs->positionLLH = Eigen::Array3d();
                }
                obs->positionLLH.value()(1) = std::stod(cell);
            }
            else if (column == "height(m)")
            {
                if (!obs->positionLLH.has_value())
                {
                    obs->positionLLH = Eigen::Array3d();
                }
                obs->positionLLH.value()(2) = std::stod(cell);
            }
            else if (column == "Q")
            {
                obs->Q = static_cast<uint8_t>(std::stoul(cell));
            }
            else if (column == "ns")
            {
                obs->ns = static_cast<uint8_t>(std::stoul(cell));
            }
            else if (column == "sdx(m)")
            {
                if (!obs->sdXYZ.has_value())
                {
                    obs->sdXYZ = Eigen::Vector3d();
                }
                obs->sdXYZ.value().x() = std::stod(cell);
            }
            else if (column == "sdy(m)")
            {
                if (!obs->sdXYZ.has_value())
                {
                    obs->sdXYZ = Eigen::Vector3d();
                }
                obs->sdXYZ.value().y() = std::stod(cell);
            }
            else if (column == "sdz(m)")
            {
                if (!obs->sdXYZ.has_value())
                {
                    obs->sdXYZ = Eigen::Vector3d();
                }
                obs->sdXYZ.value().z() = std::stod(cell);
            }
            else if (column == "sdn(m)")
            {
                if (!obs->sdNEU.has_value())
                {
                    obs->sdNEU = Eigen::Vector3d();
                }
                obs->sdNEU.value()(0) = std::stod(cell);
            }
            else if (column == "sde(m)")
            {
                if (!obs->sdNEU.has_value())
                {
                    obs->sdNEU = Eigen::Vector3d();
                }
                obs->sdNEU.value()(1) = std::stod(cell);
            }
            else if (column == "sdu(m)")
            {
                if (!obs->sdNEU.has_value())
                {
                    obs->sdNEU = Eigen::Vector3d();
                }
                obs->sdNEU.value()(2) = std::stod(cell);
            }
            else if (column == "sdxy(m)")
            {
                obs->sdxy = std::stod(cell);
            }
            else if (column == "sdyz(m)")
            {
                obs->sdyz = std::stod(cell);
            }
            else if (column == "sdzx(m)")
            {
                obs->sdzx = std::stod(cell);
            }
            else if (column == "sdne(m)")
            {
                obs->sdne = std::stod(cell);
            }
            else if (column == "sdeu(m)")
            {
                obs->sdeu = std::stod(cell);
            }
            else if (column == "sdun(m)")
            {
                obs->sdun = std::stod(cell);
            }
            else if (column == "age(s)")
            {
                obs->age = std::stod(cell);
            }
            else if (column == "ratio")
            {
                obs->ratio = std::stod(cell);
            }
        }
    }

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

NAV::FileReader::FileType NAV::RtklibPosFile::determineFileType()
{
    return FileReader::FileType::ASCII;
}