#include "ImuDataLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

NAV::ImuDataLogger::ImuDataLogger(const std::string& name, const std::map<std::string, std::string>& options)
    : DataLogger(name, options)
{
    LOG_TRACE("called for {}", name);

    if (fileType == FileType::ASCII)
    {
        filestream << "GpsCycle,GpsWeek,GpsToW,TimeStartup,"
                   << "UnCompMagX,UnCompMagY,UnCompMagZ,UnCompAccX,UnCompAccY,UnCompAccZ,UnCompGyroX,UnCompGyroY,UnCompGyroZ,"
                   << "Temperature" << std::endl;
    }
}

NAV::ImuDataLogger::~ImuDataLogger()
{
    LOG_TRACE("called for {}", name);
}

void NAV::ImuDataLogger::writeObservation(std::shared_ptr<NAV::ImuObs>& obs)
{
    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    if (fileType == FileType::ASCII)
    {
        if (obs->insTime.has_value())
        {
            filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.value().GetGPSTime().gpsCycle;
        }
        filestream << ",";
        if (obs->insTime.has_value())
        {
            filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().GetGPSTime().gpsWeek;
        }
        filestream << ",";
        if (obs->insTime.has_value())
        {
            filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().GetGPSTime().tow;
        }
        filestream << ",";
        if (obs->timeSinceStartup.has_value())
        {
            filestream << std::setprecision(valuePrecision) << obs->timeSinceStartup.value();
        }
        filestream << ",";
        if (obs->magUncompXYZ.has_value())
        {
            filestream << obs->magUncompXYZ.value().x();
        }
        filestream << ",";
        if (obs->magUncompXYZ.has_value())
        {
            filestream << obs->magUncompXYZ.value().y();
        }
        filestream << ",";
        if (obs->magUncompXYZ.has_value())
        {
            filestream << obs->magUncompXYZ.value().z();
        }
        filestream << ",";
        if (obs->accelUncompXYZ.has_value())
        {
            filestream << obs->accelUncompXYZ.value().x();
        }
        filestream << ",";
        if (obs->accelUncompXYZ.has_value())
        {
            filestream << obs->accelUncompXYZ.value().y();
        }
        filestream << ",";
        if (obs->accelUncompXYZ.has_value())
        {
            filestream << obs->accelUncompXYZ.value().z();
        }
        filestream << ",";
        if (obs->gyroUncompXYZ.has_value())
        {
            filestream << obs->gyroUncompXYZ.value().x();
        }
        filestream << ",";
        if (obs->gyroUncompXYZ.has_value())
        {
            filestream << obs->gyroUncompXYZ.value().y();
        }
        filestream << ",";
        if (obs->gyroUncompXYZ.has_value())
        {
            filestream << obs->gyroUncompXYZ.value().z();
        }
        filestream << ",";
        if (obs->temperature.has_value())
        {
            filestream << obs->temperature.value();
        }
        filestream << '\n';
    }

    invokeCallbacks(obs);
}