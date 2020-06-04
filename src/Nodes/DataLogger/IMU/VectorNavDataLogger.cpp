#include "VectorNavDataLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

NAV::VectorNavDataLogger::VectorNavDataLogger(const std::string& name, const std::map<std::string, std::string>& options)
    : DataLogger(name, options)
{
    LOG_TRACE("called for {}", name);

    if (fileType == FileType::ASCII)
    {
        filestream << "GpsCycle,GpsWeek,GpsToW,TimeStartup,TimeSyncIn,SyncInCnt,"
                   << "UnCompMagX,UnCompMagY,UnCompMagZ,UnCompAccX,UnCompAccY,UnCompAccZ,UnCompGyroX,UnCompGyroY,UnCompGyroZ,"
                   << "Temperature,Pressure,DeltaTime,DeltaThetaX,DeltaThetaY,DeltaThetaZ,DeltaVelX,DeltaVelY,DeltaVelZ,"
                   << "MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AhrsStatus,Quat[0],Quat[1],Quat[2],Quat[3],"
                   << "MagN,MagE,MagD,AccN,AccE,AccD,LinAccX,LinAccY,LinAccZ,LinAccN,LinAccE,LinAccD,"
                   << "YawU,PitchU,RollU,YawRate,PitchRate,RollRate" << std::endl;
    }
}

NAV::VectorNavDataLogger::~VectorNavDataLogger()
{
    LOG_TRACE("called for {}", name);
}

void NAV::VectorNavDataLogger::writeObservation(std::shared_ptr<NAV::VectorNavObs>& obs)
{
    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    if (fileType == FileType::BINARY)
    {
        // TODO: Implement Binary Logging for VectorNavObs
        LOG_CRITICAL("Binary Logging of VectorNavObs is not implemented yet");
    }
    else
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
            filestream << obs->timeSinceStartup.value();
        }
        filestream << ",";
        if (obs->timeSinceSyncIn.has_value())
        {
            filestream << obs->timeSinceSyncIn.value();
        }
        filestream << ",";
        if (obs->syncInCnt.has_value())
        {
            filestream << obs->syncInCnt.value();
        }
        filestream << ",";
        if (obs->magUncompXYZ.has_value())
        {
            filestream << std::setprecision(valuePrecision) << obs->magUncompXYZ.value().x();
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
        filestream << ",";
        if (obs->pressure.has_value())
        {
            filestream << obs->pressure.value();
        }
        filestream << ",";
        if (obs->dtime.has_value())
        {
            filestream << obs->dtime.value();
        }
        filestream << ",";
        if (obs->dtheta.has_value())
        {
            filestream << obs->dtheta.value().x();
        }
        filestream << ",";
        if (obs->dtheta.has_value())
        {
            filestream << obs->dtheta.value().y();
        }
        filestream << ",";
        if (obs->dtheta.has_value())
        {
            filestream << obs->dtheta.value().z();
        }
        filestream << ",";
        if (obs->dvel.has_value())
        {
            filestream << obs->dvel.value().x();
        }
        filestream << ",";
        if (obs->dvel.has_value())
        {
            filestream << obs->dvel.value().y();
        }
        filestream << ",";
        if (obs->dvel.has_value())
        {
            filestream << obs->dvel.value().z();
        }
        filestream << ",";
        if (obs->magCompXYZ.has_value())
        {
            filestream << obs->magCompXYZ.value().x();
        }
        filestream << ",";
        if (obs->magCompXYZ.has_value())
        {
            filestream << obs->magCompXYZ.value().y();
        }
        filestream << ",";
        if (obs->magCompXYZ.has_value())
        {
            filestream << obs->magCompXYZ.value().z();
        }
        filestream << ",";
        if (obs->accelCompXYZ.has_value())
        {
            filestream << obs->accelCompXYZ.value().x();
        }
        filestream << ",";
        if (obs->accelCompXYZ.has_value())
        {
            filestream << obs->accelCompXYZ.value().y();
        }
        filestream << ",";
        if (obs->accelCompXYZ.has_value())
        {
            filestream << obs->accelCompXYZ.value().z();
        }
        filestream << ",";
        if (obs->gyroCompXYZ.has_value())
        {
            filestream << obs->gyroCompXYZ.value().x();
        }
        filestream << ",";
        if (obs->gyroCompXYZ.has_value())
        {
            filestream << obs->gyroCompXYZ.value().y();
        }
        filestream << ",";
        if (obs->gyroCompXYZ.has_value())
        {
            filestream << obs->gyroCompXYZ.value().z();
        }
        filestream << ",";
        if (obs->vpeStatus.has_value())
        {
            filestream << obs->vpeStatus.value().status;
        }
        filestream << ",";
        if (obs->quaternion.has_value())
        {
            filestream << obs->quaternion.value().w();
        }
        filestream << ",";
        if (obs->quaternion.has_value())
        {
            filestream << obs->quaternion.value().x();
        }
        filestream << ",";
        if (obs->quaternion.has_value())
        {
            filestream << obs->quaternion.value().y();
        }
        filestream << ",";
        if (obs->quaternion.has_value())
        {
            filestream << obs->quaternion.value().z();
        }
        filestream << ",";
        if (obs->magCompNED.has_value())
        {
            filestream << obs->magCompNED.value().x();
        }
        filestream << ",";
        if (obs->magCompNED.has_value())
        {
            filestream << obs->magCompNED.value().y();
        }
        filestream << ",";
        if (obs->magCompNED.has_value())
        {
            filestream << obs->magCompNED.value().z();
        }
        filestream << ",";
        if (obs->accelCompNED.has_value())
        {
            filestream << obs->accelCompNED.value().x();
        }
        filestream << ",";
        if (obs->accelCompNED.has_value())
        {
            filestream << obs->accelCompNED.value().y();
        }
        filestream << ",";
        if (obs->accelCompNED.has_value())
        {
            filestream << obs->accelCompNED.value().z();
        }
        filestream << ",";
        if (obs->linearAccelXYZ.has_value())
        {
            filestream << obs->linearAccelXYZ.value().x();
        }
        filestream << ",";
        if (obs->linearAccelXYZ.has_value())
        {
            filestream << obs->linearAccelXYZ.value().y();
        }
        filestream << ",";
        if (obs->linearAccelXYZ.has_value())
        {
            filestream << obs->linearAccelXYZ.value().z();
        }
        filestream << ",";
        if (obs->linearAccelNED.has_value())
        {
            filestream << obs->linearAccelNED.value().x();
        }
        filestream << ",";
        if (obs->linearAccelNED.has_value())
        {
            filestream << obs->linearAccelNED.value().y();
        }
        filestream << ",";
        if (obs->linearAccelNED.has_value())
        {
            filestream << obs->linearAccelNED.value().z();
        }
        filestream << ",";
        if (obs->yawPitchRollUncertainty.has_value())
        {
            filestream << obs->yawPitchRollUncertainty.value().x();
        }
        filestream << ",";
        if (obs->yawPitchRollUncertainty.has_value())
        {
            filestream << obs->yawPitchRollUncertainty.value().y();
        }
        filestream << ",";
        if (obs->yawPitchRollUncertainty.has_value())
        {
            filestream << obs->yawPitchRollUncertainty.value().z();
        }
        filestream << ",";
        if (obs->gyroCompNED.has_value())
        {
            filestream << obs->gyroCompNED.value().x();
        }
        filestream << ",";
        if (obs->gyroCompNED.has_value())
        {
            filestream << obs->gyroCompNED.value().y();
        }
        filestream << ",";
        if (obs->gyroCompNED.has_value())
        {
            filestream << obs->gyroCompNED.value().z();
        }
        filestream << '\n';
    }

    return invokeCallbacks(obs);
}