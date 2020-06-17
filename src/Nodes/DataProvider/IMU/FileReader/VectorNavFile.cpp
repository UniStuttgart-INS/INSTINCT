#include "VectorNavFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>

NAV::VectorNavFile::VectorNavFile(const std::string& name, const std::map<std::string, std::string>& options)
    : FileReader(options), Imu(name, options)
{
    LOG_TRACE("called for {}", name);

    fileType = determineFileType();

    if (fileType == FileType::BINARY)
    {
        filestream = std::ifstream(path, std::ios_base::in | std::ios_base::binary);
    }
    else
    {
        filestream = std::ifstream(path);
    }

    if (filestream.good())
    {
        if (fileType != FileType::BINARY)
        {
            // Read header line
            std::string line;
            std::getline(filestream, line);
            // Remove any starting non text characters
            line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                                  std::ptr_fun<int, int>(std::isalnum)));
            // Convert line into stream
            std::stringstream lineStream(line);
            std::string cell;
            // Split line at comma
            while (std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(),
                                        std::ptr_fun<int, int>(std::iscntrl)),
                           cell.end());
                columns.push_back(cell);
            }

            dataStart = filestream.tellg();

            LOG_DEBUG("{}-ASCII-File successfully initialized", name);
        }
        else
        {
            LOG_DEBUG("{}-Binary-File successfully initialized", name);
        }
    }
    else
    {
        LOG_CRITICAL("{} could not open file {}", name, path);
    }
}

NAV::VectorNavFile::~VectorNavFile()
{
    LOG_TRACE("called for {}", name);

    // removeAllCallbacks();
    columns.clear();
    if (filestream.is_open())
    {
        filestream.close();
    }
}

void NAV::VectorNavFile::resetNode()
{
    // Return to position
    filestream.clear();
    filestream.seekg(dataStart, std::ios_base::beg);
}

std::shared_ptr<NAV::VectorNavObs> NAV::VectorNavFile::pollData(bool peek)
{
    auto obs = std::make_shared<VectorNavObs>();

    if (fileType == FileType::BINARY)
    {
        // TODO: Implement VectorNavFile Binary reading
        LOG_CRITICAL("Binary VectorNavFile pollData is not implemented yet.");
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
    line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                          std::ptr_fun<int, int>(std::isgraph)));

    if (line.empty())
    {
        return nullptr;
    }

    // Convert line into stream
    std::stringstream lineStream(line);
    std::string cell;

    Eigen::Matrix3d dcm = Eigen::Matrix3d::Zero();

    std::optional<uint16_t> gpsCycle;
    std::optional<uint16_t> gpsWeek;
    std::optional<long double> gpsToW;

    // Split line at comma
    for (const auto& column : columns)
    {
        if (std::getline(lineStream, cell, ','))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(),
                                    std::ptr_fun<int, int>(std::iscntrl)),
                       cell.end());
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
            else if (column == "TimeSyncIn")
            {
                obs->timeSinceSyncIn.emplace(std::stoull(cell));
            }
            else if (column == "SyncInCnt")
            {
                obs->syncInCnt.emplace(std::stoul(cell));
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
            else if (column == "Pressure")
            {
                obs->pressure.emplace(std::stod(cell));
            }
            else if (column == "DeltaTime")
            {
                obs->dtime.emplace(std::stod(cell));
            }
            else if (column == "DeltaThetaX")
            {
                if (!obs->dtheta.has_value())
                {
                    obs->dtheta = Eigen::Array3d();
                }
                obs->dtheta.value().x() = std::stod(cell);
            }
            else if (column == "DeltaThetaY")
            {
                if (!obs->dtheta.has_value())
                {
                    obs->dtheta = Eigen::Array3d();
                }
                obs->dtheta.value().y() = std::stod(cell);
            }
            else if (column == "DeltaThetaZ")
            {
                if (!obs->dtheta.has_value())
                {
                    obs->dtheta = Eigen::Array3d();
                }
                obs->dtheta.value().z() = std::stod(cell);
            }
            else if (column == "DeltaVelX")
            {
                if (!obs->dvel.has_value())
                {
                    obs->dvel = Eigen::Vector3d();
                }
                obs->dvel.value().x() = std::stod(cell);
            }
            else if (column == "DeltaVelY")
            {
                if (!obs->dvel.has_value())
                {
                    obs->dvel = Eigen::Vector3d();
                }
                obs->dvel.value().y() = std::stod(cell);
            }
            else if (column == "DeltaVelZ")
            {
                if (!obs->dvel.has_value())
                {
                    obs->dvel = Eigen::Vector3d();
                }
                obs->dvel.value().z() = std::stod(cell);
            }
            else if (column == "MagX")
            {
                if (!obs->magCompXYZ.has_value())
                {
                    obs->magCompXYZ = Eigen::Vector3d();
                }
                obs->magCompXYZ.value().x() = std::stod(cell);
            }
            else if (column == "MagY")
            {
                if (!obs->magCompXYZ.has_value())
                {
                    obs->magCompXYZ = Eigen::Vector3d();
                }
                obs->magCompXYZ.value().y() = std::stod(cell);
            }
            else if (column == "MagZ")
            {
                if (!obs->magCompXYZ.has_value())
                {
                    obs->magCompXYZ = Eigen::Vector3d();
                }
                obs->magCompXYZ.value().z() = std::stod(cell);
            }
            else if (column == "AccX")
            {
                if (!obs->accelCompXYZ.has_value())
                {
                    obs->accelCompXYZ = Eigen::Vector3d();
                }
                obs->accelCompXYZ.value().x() = std::stod(cell);
            }
            else if (column == "AccY")
            {
                if (!obs->accelCompXYZ.has_value())
                {
                    obs->accelCompXYZ = Eigen::Vector3d();
                }
                obs->accelCompXYZ.value().y() = std::stod(cell);
            }
            else if (column == "AccZ")
            {
                if (!obs->accelCompXYZ.has_value())
                {
                    obs->accelCompXYZ = Eigen::Vector3d();
                }
                obs->accelCompXYZ.value().z() = std::stod(cell);
            }
            else if (column == "GyroX")
            {
                if (!obs->gyroCompXYZ.has_value())
                {
                    obs->gyroCompXYZ = Eigen::Vector3d();
                }
                obs->gyroCompXYZ.value().x() = std::stod(cell);
            }
            else if (column == "GyroY")
            {
                if (!obs->gyroCompXYZ.has_value())
                {
                    obs->gyroCompXYZ = Eigen::Vector3d();
                }
                obs->gyroCompXYZ.value().y() = std::stod(cell);
            }
            else if (column == "GyroZ")
            {
                if (!obs->gyroCompXYZ.has_value())
                {
                    obs->gyroCompXYZ = Eigen::Vector3d();
                }
                obs->gyroCompXYZ.value().z() = std::stod(cell);
            }
            else if (column == "AhrsStatus")
            {
                obs->vpeStatus.emplace(std::stoul(cell));
            }
            else if (column == "Yaw")
            {
                obs->yawPitchRoll.value()(0) = std::stod(cell);
            }
            else if (column == "Pitch")
            {
                obs->yawPitchRoll.value()(1) = std::stod(cell);
            }
            else if (column == "Roll")
            {
                obs->yawPitchRoll.value()(2) = std::stod(cell);
            }
            else if (column == "Quat[0]")
            {
                if (!obs->quaternion.has_value())
                {
                    obs->quaternion = Eigen::Quaterniond();
                }
                obs->quaternion.value().w() = std::stod(cell);
            }
            else if (column == "Quat[1]")
            {
                if (!obs->quaternion.has_value())
                {
                    obs->quaternion = Eigen::Quaterniond();
                }
                obs->quaternion.value().x() = std::stod(cell);
            }
            else if (column == "Quat[2]")
            {
                if (!obs->quaternion.has_value())
                {
                    obs->quaternion = Eigen::Quaterniond();
                }
                obs->quaternion.value().y() = std::stod(cell);
            }
            else if (column == "Quat[3]")
            {
                if (!obs->quaternion.has_value())
                {
                    obs->quaternion = Eigen::Quaterniond();
                }
                obs->quaternion.value().z() = std::stod(cell);
            }
            else if (column == "C[0.0]")
            {
                dcm(0, 0) = std::stod(cell);
            }
            else if (column == "C[0.1]")
            {
                dcm(0, 1) = std::stod(cell);
            }
            else if (column == "C[0.2]")
            {
                dcm(0, 2) = std::stod(cell);
            }
            else if (column == "C[1.0]")
            {
                dcm(1, 0) = std::stod(cell);
            }
            else if (column == "C[1.1]")
            {
                dcm(1, 1) = std::stod(cell);
            }
            else if (column == "C[1.2]")
            {
                dcm(1, 2) = std::stod(cell);
            }
            else if (column == "C[2.0]")
            {
                dcm(2, 0) = std::stod(cell);
            }
            else if (column == "C[2.1]")
            {
                dcm(2, 1) = std::stod(cell);
            }
            else if (column == "C[2.2]")
            {
                dcm(2, 2) = std::stod(cell);
            }
            else if (column == "MagN")
            {
                if (!obs->magCompNED.has_value())
                {
                    obs->magCompNED = Eigen::Vector3d();
                }
                obs->magCompNED.value()(0) = std::stod(cell);
            }
            else if (column == "MagE")
            {
                if (!obs->magCompNED.has_value())
                {
                    obs->magCompNED = Eigen::Vector3d();
                }
                obs->magCompNED.value()(1) = std::stod(cell);
            }
            else if (column == "MagD")
            {
                if (!obs->magCompNED.has_value())
                {
                    obs->magCompNED = Eigen::Vector3d();
                }
                obs->magCompNED.value()(2) = std::stod(cell);
            }
            else if (column == "AccN")
            {
                if (!obs->accelCompNED.has_value())
                {
                    obs->accelCompNED = Eigen::Vector3d();
                }
                obs->accelCompNED.value()(0) = std::stod(cell);
            }
            else if (column == "AccE")
            {
                if (!obs->accelCompNED.has_value())
                {
                    obs->accelCompNED = Eigen::Vector3d();
                }
                obs->accelCompNED.value()(1) = std::stod(cell);
            }
            else if (column == "AccD")
            {
                if (!obs->accelCompNED.has_value())
                {
                    obs->accelCompNED = Eigen::Vector3d();
                }
                obs->accelCompNED.value()(2) = std::stod(cell);
            }
            else if (column == "LinAccX")
            {
                if (!obs->linearAccelXYZ.has_value())
                {
                    obs->linearAccelXYZ = Eigen::Vector3d();
                }
                obs->linearAccelXYZ.value().x() = std::stod(cell);
            }
            else if (column == "LinAccY")
            {
                if (!obs->linearAccelXYZ.has_value())
                {
                    obs->linearAccelXYZ = Eigen::Vector3d();
                }
                obs->linearAccelXYZ.value().y() = std::stod(cell);
            }
            else if (column == "LinAccZ")
            {
                if (!obs->linearAccelXYZ.has_value())
                {
                    obs->linearAccelXYZ = Eigen::Vector3d();
                }
                obs->linearAccelXYZ.value().z() = std::stod(cell);
            }
            else if (column == "LinAccN")
            {
                if (!obs->linearAccelNED.has_value())
                {
                    obs->linearAccelNED = Eigen::Vector3d();
                }
                obs->linearAccelNED.value()(0) = std::stod(cell);
            }
            else if (column == "LinAccE")
            {
                if (!obs->linearAccelNED.has_value())
                {
                    obs->linearAccelNED = Eigen::Vector3d();
                }
                obs->linearAccelNED.value()(1) = std::stod(cell);
            }
            else if (column == "LinAccD")
            {
                if (!obs->linearAccelNED.has_value())
                {
                    obs->linearAccelNED = Eigen::Vector3d();
                }
                obs->linearAccelNED.value()(2) = std::stod(cell);
            }
            else if (column == "YawU")
            {
                if (!obs->yawPitchRollUncertainty.has_value())
                {
                    obs->yawPitchRollUncertainty = Eigen::Array3d();
                }
                obs->yawPitchRollUncertainty.value()(0) = std::stod(cell);
            }
            else if (column == "PitchU")
            {
                if (!obs->yawPitchRollUncertainty.has_value())
                {
                    obs->yawPitchRollUncertainty = Eigen::Array3d();
                }
                obs->yawPitchRollUncertainty.value()(1) = std::stod(cell);
            }
            else if (column == "RollU")
            {
                if (!obs->yawPitchRollUncertainty.has_value())
                {
                    obs->yawPitchRollUncertainty = Eigen::Array3d();
                }
                obs->yawPitchRollUncertainty.value()(2) = std::stod(cell);
            }
            else if (column == "YawRate")
            {
                if (!obs->gyroCompNED.has_value())
                {
                    obs->gyroCompNED = Eigen::Vector3d();
                }
                obs->gyroCompNED.value()(0) = std::stod(cell);
            }
            else if (column == "PitchRate")
            {
                if (!obs->gyroCompNED.has_value())
                {
                    obs->gyroCompNED = Eigen::Vector3d();
                }
                obs->gyroCompNED.value()(1) = std::stod(cell);
            }
            else if (column == "RollRate")
            {
                if (!obs->gyroCompNED.has_value())
                {
                    obs->gyroCompNED = Eigen::Vector3d();
                }
                obs->gyroCompNED.value()(2) = std::stod(cell);
            }
        }
    }

    if (!obs->quaternion.has_value())
    {
        if (!dcm.isZero())
        {
            obs->quaternion = dcm;
        }
        else if (!obs->yawPitchRoll.value().isZero())
        {
            constexpr double deg2rad = M_PI / 180.0;
            dcm = Eigen::AngleAxisd(obs->yawPitchRoll.value()(0) * deg2rad, Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(obs->yawPitchRoll.value()(1) * deg2rad, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(obs->yawPitchRoll.value()(2) * deg2rad, Eigen::Vector3d::UnitZ());
            obs->quaternion = dcm;
        }
    }

    LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
             name, obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
             obs->vpeStatus.value().status, obs->temperature.value());

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::VectorNavFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    constexpr uint8_t BinaryStartChar = 0xFA;

    filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 256;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if (std::strstr(buffer.data(), "TimeStartup"))
        {
            filestream.close();
            LOG_DEBUG("{} has the file type: ASCII", name);
            return FileType::ASCII;
        }
        if (memmem(buffer.data(), BUFFER_SIZE, "Control Center", sizeof("Control Center")) != nullptr
            || static_cast<unsigned char>(buffer.at(0)) == BinaryStartChar)
        {
            filestream.close();
            LOG_DEBUG("{} has the file type: Binary", name);
            return FileType::BINARY;
        }
        filestream.close();
        LOG_CRITICAL("{} could not determine file type", name);
    }

    LOG_CRITICAL("{} could not open file {}", name, path);
    return FileType::NONE;
}