#include "VectorNavFile.hpp"

#include "NodeInterface.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"
#include "util/Logger.hpp"
#include <ios>

NAV::VectorNavFile::VectorNavFile(std::string name, std::deque<std::string>& options)
    : FileReader(options), Imu(name, options)
{
    LOG_TRACE("called for {}", name);

    if (NavStatus result = determineFileType();
        result != NavStatus::NAV_OK)
        LOG_CRITICAL("{} could not determine file type", name);

    if (isBinary)
        filestream = std::ifstream(path, std::ios_base::in | std::ios_base::binary);
    else
        filestream = std::ifstream(path);

    if (filestream.good())
    {
        if (!isBinary)
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

            LOG_DEBUG("{}-ASCII-File successfully initialized", name);
        }
        else
        {
            LOG_DEBUG("{}-Binary-File successfully initialized", name);
        }
    }
    else
        LOG_CRITICAL("{} could not open file {}", name, path);
}

NAV::VectorNavFile::~VectorNavFile()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacks();
    callbacksEnabled = false;
    columns.clear();
    if (filestream.is_open())
        filestream.close();

    LOG_DEBUG("{} successfully deinitialized", name);
}

bool NAV::VectorNavFile::isFileReader()
{
    return true;
}

std::shared_ptr<NAV::NodeData> NAV::VectorNavFile::pollData()
{
    LOG_TRACE("called for {}", name);

    auto obs = std::make_shared<VectorNavObs>();

    if (isBinary)
    {
        LOG_CRITICAL("Binary VectorNavFile pollData is not implemented yet."); // TODO: implement

        return nullptr;
    }
    else
    {
        // Read line
        std::string line;
        std::getline(filestream, line);

        if (line.empty())
            return nullptr;

        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;

        Eigen::Array3d ypr = Eigen::Array3d::Zero();
        Eigen::Matrix3d dcm = Eigen::Matrix3d::Zero();

        short unsigned int gpsCycle = USHRT_MAX, gpsWeek = USHRT_MAX;
        double gpsToW = NAN;

        // Split line at comma
        for (size_t i = 0; i < columns.size(); i++)
        {
            if (std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(),
                                        std::ptr_fun<int, int>(std::iscntrl)),
                           cell.end());
                if (cell.empty())
                    continue;

                if (columns[i] == "GpsCycle")
                    gpsCycle = static_cast<short unsigned int>(std::stoul(cell));
                else if (columns[i] == "GpsWeek")
                    gpsWeek = static_cast<short unsigned int>(std::stoul(cell));
                else if (columns[i] == "GpsToW")
                    gpsToW = std::stod(cell);
                else if (columns[i] == "TimeStartup")
                    obs->timeSinceStartup = std::stoull(cell);
                else if (columns[i] == "TimeSyncIn")
                    obs->timeSinceSyncIn = std::stoull(cell);
                else if (columns[i] == "SyncInCnt")
                    obs->syncInCnt = std::stoul(cell);
                else if (columns[i] == "UnCompMagX")
                {
                    if (!obs->magUncompXYZ.has_value())
                        obs->magUncompXYZ = Eigen::Vector3d();
                    obs->magUncompXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "UnCompMagY")
                {
                    if (!obs->magUncompXYZ.has_value())
                        obs->magUncompXYZ = Eigen::Vector3d();
                    obs->magUncompXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "UnCompMagZ")
                {
                    if (!obs->magUncompXYZ.has_value())
                        obs->magUncompXYZ = Eigen::Vector3d();
                    obs->magUncompXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "UnCompAccX")
                {
                    if (!obs->accelUncompXYZ.has_value())
                        obs->accelUncompXYZ = Eigen::Vector3d();
                    obs->accelUncompXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "UnCompAccY")
                {
                    if (!obs->accelUncompXYZ.has_value())
                        obs->accelUncompXYZ = Eigen::Vector3d();
                    obs->accelUncompXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "UnCompAccZ")
                {
                    if (!obs->accelUncompXYZ.has_value())
                        obs->accelUncompXYZ = Eigen::Vector3d();
                    obs->accelUncompXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "UnCompGyroX")
                {
                    if (!obs->gyroUncompXYZ.has_value())
                        obs->gyroUncompXYZ = Eigen::Vector3d();
                    obs->gyroUncompXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "UnCompGyroY")
                {
                    if (!obs->gyroUncompXYZ.has_value())
                        obs->gyroUncompXYZ = Eigen::Vector3d();
                    obs->gyroUncompXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "UnCompGyroZ")
                {
                    if (!obs->gyroUncompXYZ.has_value())
                        obs->gyroUncompXYZ = Eigen::Vector3d();
                    obs->gyroUncompXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "Temperature")
                    obs->temperature = std::stof(cell);
                else if (columns[i] == "Pressure")
                    obs->pressure = std::stof(cell);
                else if (columns[i] == "DeltaTime")
                    obs->dtime = std::stof(cell);
                else if (columns[i] == "DeltaThetaX")
                {
                    if (!obs->dtheta.has_value())
                        obs->dtheta = Eigen::Array3d();
                    obs->dtheta.value().x() = std::stof(cell);
                }
                else if (columns[i] == "DeltaThetaY")
                {
                    if (!obs->dtheta.has_value())
                        obs->dtheta = Eigen::Array3d();
                    obs->dtheta.value().y() = std::stof(cell);
                }
                else if (columns[i] == "DeltaThetaZ")
                {
                    if (!obs->dtheta.has_value())
                        obs->dtheta = Eigen::Array3d();
                    obs->dtheta.value().z() = std::stof(cell);
                }
                else if (columns[i] == "DeltaVelX")
                {
                    if (!obs->dvel.has_value())
                        obs->dvel = Eigen::Vector3d();
                    obs->dvel.value().x() = std::stof(cell);
                }
                else if (columns[i] == "DeltaVelY")
                {
                    if (!obs->dvel.has_value())
                        obs->dvel = Eigen::Vector3d();
                    obs->dvel.value().y() = std::stof(cell);
                }
                else if (columns[i] == "DeltaVelZ")
                {
                    if (!obs->dvel.has_value())
                        obs->dvel = Eigen::Vector3d();
                    obs->dvel.value().z() = std::stof(cell);
                }
                else if (columns[i] == "MagX")
                {
                    if (!obs->magCompXYZ.has_value())
                        obs->magCompXYZ = Eigen::Vector3d();
                    obs->magCompXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "MagY")
                {
                    if (!obs->magCompXYZ.has_value())
                        obs->magCompXYZ = Eigen::Vector3d();
                    obs->magCompXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "MagZ")
                {
                    if (!obs->magCompXYZ.has_value())
                        obs->magCompXYZ = Eigen::Vector3d();
                    obs->magCompXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "AccX")
                {
                    if (!obs->accelCompXYZ.has_value())
                        obs->accelCompXYZ = Eigen::Vector3d();
                    obs->accelCompXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "AccY")
                {
                    if (!obs->accelCompXYZ.has_value())
                        obs->accelCompXYZ = Eigen::Vector3d();
                    obs->accelCompXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "AccZ")
                {
                    if (!obs->accelCompXYZ.has_value())
                        obs->accelCompXYZ = Eigen::Vector3d();
                    obs->accelCompXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "GyroX")
                {
                    if (!obs->gyroCompXYZ.has_value())
                        obs->gyroCompXYZ = Eigen::Vector3d();
                    obs->gyroCompXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "GyroY")
                {
                    if (!obs->gyroCompXYZ.has_value())
                        obs->gyroCompXYZ = Eigen::Vector3d();
                    obs->gyroCompXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "GyroZ")
                {
                    if (!obs->gyroCompXYZ.has_value())
                        obs->gyroCompXYZ = Eigen::Vector3d();
                    obs->gyroCompXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "AhrsStatus")
                    obs->vpeStatus = std::stoul(cell);
                else if (columns[i] == "Yaw")
                    ypr(0) = std::stof(cell);
                else if (columns[i] == "Pitch")
                    ypr(1) = std::stof(cell);
                else if (columns[i] == "Roll")
                    ypr(2) = std::stof(cell);
                else if (columns[i] == "Quat[0]")
                {
                    if (!obs->quaternion.has_value())
                        obs->quaternion = Eigen::Quaterniond();
                    obs->quaternion.value().w() = std::stof(cell);
                }
                else if (columns[i] == "Quat[1]")
                {
                    if (!obs->quaternion.has_value())
                        obs->quaternion = Eigen::Quaterniond();
                    obs->quaternion.value().x() = std::stof(cell);
                }
                else if (columns[i] == "Quat[2]")
                {
                    if (!obs->quaternion.has_value())
                        obs->quaternion = Eigen::Quaterniond();
                    obs->quaternion.value().y() = std::stof(cell);
                }
                else if (columns[i] == "Quat[3]")
                {
                    if (!obs->quaternion.has_value())
                        obs->quaternion = Eigen::Quaterniond();
                    obs->quaternion.value().z() = std::stof(cell);
                }
                else if (columns[i] == "C[0.0]")
                    dcm(0, 0) = std::stof(cell);
                else if (columns[i] == "C[0.1]")
                    dcm(0, 1) = std::stof(cell);
                else if (columns[i] == "C[0.2]")
                    dcm(0, 2) = std::stof(cell);
                else if (columns[i] == "C[1.0]")
                    dcm(1, 0) = std::stof(cell);
                else if (columns[i] == "C[1.1]")
                    dcm(1, 1) = std::stof(cell);
                else if (columns[i] == "C[1.2]")
                    dcm(1, 2) = std::stof(cell);
                else if (columns[i] == "C[2.0]")
                    dcm(2, 0) = std::stof(cell);
                else if (columns[i] == "C[2.1]")
                    dcm(2, 1) = std::stof(cell);
                else if (columns[i] == "C[2.2]")
                    dcm(2, 2) = std::stof(cell);
                else if (columns[i] == "MagN")
                {
                    if (!obs->magCompNED.has_value())
                        obs->magCompNED = Eigen::Vector3d();
                    obs->magCompNED.value()(0) = std::stof(cell);
                }
                else if (columns[i] == "MagE")
                {
                    if (!obs->magCompNED.has_value())
                        obs->magCompNED = Eigen::Vector3d();
                    obs->magCompNED.value()(1) = std::stof(cell);
                }
                else if (columns[i] == "MagD")
                {
                    if (!obs->magCompNED.has_value())
                        obs->magCompNED = Eigen::Vector3d();
                    obs->magCompNED.value()(2) = std::stof(cell);
                }
                else if (columns[i] == "AccN")
                {
                    if (!obs->accelCompNED.has_value())
                        obs->accelCompNED = Eigen::Vector3d();
                    obs->accelCompNED.value()(0) = std::stof(cell);
                }
                else if (columns[i] == "AccE")
                {
                    if (!obs->accelCompNED.has_value())
                        obs->accelCompNED = Eigen::Vector3d();
                    obs->accelCompNED.value()(1) = std::stof(cell);
                }
                else if (columns[i] == "AccD")
                {
                    if (!obs->accelCompNED.has_value())
                        obs->accelCompNED = Eigen::Vector3d();
                    obs->accelCompNED.value()(2) = std::stof(cell);
                }
                else if (columns[i] == "LinAccX")
                {
                    if (!obs->linearAccelXYZ.has_value())
                        obs->linearAccelXYZ = Eigen::Vector3d();
                    obs->linearAccelXYZ.value().x() = std::stof(cell);
                }
                else if (columns[i] == "LinAccY")
                {
                    if (!obs->linearAccelXYZ.has_value())
                        obs->linearAccelXYZ = Eigen::Vector3d();
                    obs->linearAccelXYZ.value().y() = std::stof(cell);
                }
                else if (columns[i] == "LinAccZ")
                {
                    if (!obs->linearAccelXYZ.has_value())
                        obs->linearAccelXYZ = Eigen::Vector3d();
                    obs->linearAccelXYZ.value().z() = std::stof(cell);
                }
                else if (columns[i] == "LinAccN")
                {
                    if (!obs->linearAccelNED.has_value())
                        obs->linearAccelNED = Eigen::Vector3d();
                    obs->linearAccelNED.value()(0) = std::stof(cell);
                }
                else if (columns[i] == "LinAccE")
                {
                    if (!obs->linearAccelNED.has_value())
                        obs->linearAccelNED = Eigen::Vector3d();
                    obs->linearAccelNED.value()(1) = std::stof(cell);
                }
                else if (columns[i] == "LinAccD")
                {
                    if (!obs->linearAccelNED.has_value())
                        obs->linearAccelNED = Eigen::Vector3d();
                    obs->linearAccelNED.value()(2) = std::stof(cell);
                }
                else if (columns[i] == "YawU")
                {
                    if (!obs->yawPitchRollUncertainty.has_value())
                        obs->yawPitchRollUncertainty = Eigen::Array3d();
                    obs->yawPitchRollUncertainty.value()(0) = std::stof(cell);
                }
                else if (columns[i] == "PitchU")
                {
                    if (!obs->yawPitchRollUncertainty.has_value())
                        obs->yawPitchRollUncertainty = Eigen::Array3d();
                    obs->yawPitchRollUncertainty.value()(1) = std::stof(cell);
                }
                else if (columns[i] == "RollU")
                {
                    if (!obs->yawPitchRollUncertainty.has_value())
                        obs->yawPitchRollUncertainty = Eigen::Array3d();
                    obs->yawPitchRollUncertainty.value()(2) = std::stof(cell);
                }
                else if (columns[i] == "YawRate")
                {
                    if (!obs->gyroCompNED.has_value())
                        obs->gyroCompNED = Eigen::Vector3d();
                    obs->gyroCompNED.value()(0) = std::stof(cell);
                }
                else if (columns[i] == "PitchRate")
                {
                    if (!obs->gyroCompNED.has_value())
                        obs->gyroCompNED = Eigen::Vector3d();
                    obs->gyroCompNED.value()(1) = std::stof(cell);
                }
                else if (columns[i] == "RollRate")
                {
                    if (!obs->gyroCompNED.has_value())
                        obs->gyroCompNED = Eigen::Vector3d();
                    obs->gyroCompNED.value()(2) = std::stof(cell);
                }

                if (!obs->insTime.has_value() && gpsCycle != USHRT_MAX && gpsWeek != USHRT_MAX && !std::isnan(gpsToW))
                    obs->insTime = std::make_optional(InsTime(gpsWeek, gpsToW, gpsCycle));
            }
            else
                LOG_WARN("{}-ASCII-File more column entries than headers", name);
        }

        if (!obs->quaternion.has_value())
        {
            if (!dcm.isZero())
                obs->quaternion = dcm;
            else if (!ypr.isZero())
            {
                dcm = Eigen::AngleAxisd(ypr(0) * M_PI / 180, Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(ypr(1) * M_PI / 180, Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(ypr(2) * M_PI / 180, Eigen::Vector3d::UnitZ());
                obs->quaternion = dcm;
            }
        }

        LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
                 name, obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
                 obs->vpeStatus.value(), obs->temperature.value());

        // Calls all the callbacks
        invokeCallbacks(NodeInterface::getCallbackPort("VectorNavFile", "VectorNavObs"), obs);

        return obs;
    }
}

std::optional<NAV::InsTime> NAV::VectorNavFile::peekNextUpdateTime()
{
    LOG_TRACE("called for {}", name);

    if (isBinary)
    {
        LOG_CRITICAL("Binary VectorNavFile Logging is not implemented yet."); // TODO: implement

        return std::nullopt;
    }
    else
    {
        // Read line
        std::string line;
        // Get current position
        auto len = filestream.tellg();
        std::getline(filestream, line);
        // Return to position before "Read line".
        filestream.seekg(len, std::ios_base::beg);
        // Remove any starting non text characters
        line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                              std::ptr_fun<int, int>(std::isalnum)));

        if (line.empty())
            return std::nullopt;

        // Convert line into stream
        std::stringstream lineStream(line);
        std::string cell;

        short unsigned int gpsCycle = USHRT_MAX, gpsWeek = USHRT_MAX;
        double gpsToW = NAN;
        // Split line at comma
        for (size_t i = 0; i < columns.size(); i++)
        {
            if (std::getline(lineStream, cell, ','))
            {
                if (columns[i] == "GpsCycle")
                    gpsCycle = static_cast<short unsigned int>(std::stoul(cell));
                else if (columns[i] == "GpsWeek")
                    gpsWeek = static_cast<short unsigned int>(std::stoul(cell));
                else if (columns[i] == "GpsToW")
                    gpsToW = std::stod(cell);

                if (gpsCycle != USHRT_MAX && gpsWeek != USHRT_MAX && !std::isnan(gpsToW))
                    return std::make_optional(InsTime(gpsWeek, gpsToW, gpsCycle));
            }
            else
                LOG_WARN("{}-ASCII-File more column entries than headers", name);
        }

        return std::nullopt;
    }
}

NAV::NavStatus NAV::VectorNavFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

    const uint8_t BinaryStartChar = 0xFA;

    filestream = std::ifstream(path);
    char buffer[256];
    if (filestream.good())
    {
        filestream.read(buffer, 256);

        if (std::strstr(buffer, "TimeStartup"))
        {
            isBinary = false;
            filestream.close();
            return NavStatus::NAV_OK;
        }
        else if (memmem(buffer, 256, "Control Center", sizeof("Control Center")) != NULL
                 || buffer[0] == BinaryStartChar)
        {
            isBinary = true;
            filestream.close();
            return NavStatus::NAV_OK;
        }
        else
        {
            filestream.close();
            return NavStatus::NAV_ERROR_DETERMINATION_FILE_TYPE;
        }
    }
    else
    {
        LOG_ERROR("{} could not open file {}", name, path);
        return NAV_ERROR_COULD_NOT_OPEN_FILE;
    }
}