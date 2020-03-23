#include "VectorNavDataLogger.hpp"

#include "util/Logger.hpp"
#include "DataProvider/IMU/Observations/VectorNavObs.hpp"

#include <iomanip> // std::setprecision

NAV::VectorNavDataLogger::VectorNavDataLogger(std::string name, std::string path, bool isBinary)
    : DataLogger(name, path, isBinary) {}

NAV::VectorNavDataLogger::~VectorNavDataLogger()
{
    deinitialize();
}

NAV::NavStatus NAV::VectorNavDataLogger::initialize()
{
    LOG_TRACE("called for {}", name);

    // Initialize base class
    if (NavStatus result = DataLogger::initialize();
        result != NavStatus::NAV_OK)
        return result;

    if (!isBinary)
        filestream << "GpsToW,GpsWeek,TimeStartup,TimeSyncIn,SyncInCnt,"
                   << "UnCompMagX,UnCompMagY,UnCompMagZ,UnCompAccX,UnCompAccY,UnCompAccZ,UnCompGyroX,UnCompGyroY,UnCompGyroZ,"
                   << "Temperature,Pressure,DeltaTime,DeltaThetaX,DeltaThetaY,DeltaThetaZ,DeltaVelX,DeltaVelY,DeltaVelZ,"
                   << "MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AhrsStatus,Quat[0],Quat[1],Quat[2],Quat[3],"
                   << "MagN,MagE,MagD,AccN,AccE,AccD,LinAccX,LinAccY,LinAccZ,LinAccN,LinAccE,LinAccD,"
                   << "YawU,PitchU,RollU,YawRate,PitchRate,RollRate" << std::endl;

    LOG_DEBUG("{} successfully initialized", name);

    initialized = true;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::VectorNavDataLogger::deinitialize()
{
    LOG_TRACE("called for {}", name);

    // Deinitialize base class after
    if (NavStatus result = DataLogger::deinitialize();
        result != NavStatus::NAV_OK)
        return result;

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::VectorNavDataLogger::writeObservation(std::shared_ptr<void> observation, std::shared_ptr<void> userData)
{
    auto vnObs = std::static_pointer_cast<VectorNavObs>(observation);
    auto logger = std::static_pointer_cast<VectorNavDataLogger>(userData);

    LOG_TRACE("called for {}", logger->name);

    if (logger->isBinary)
    {
        // TODO: Implement this
        LOG_ERROR("Binary Logging of VectorNavObs is not implemented yet");
    }
    else
    {
        if (vnObs->gpsTimeOfWeek.has_value())
            logger->filestream << std::fixed << std::setprecision(3) << vnObs->gpsTimeOfWeek.value();
        logger->filestream << ",";
        if (vnObs->gpsWeek.has_value())
            logger->filestream << std::defaultfloat << std::setprecision(12) << vnObs->gpsWeek.value();
        logger->filestream << ",";
        if (vnObs->timeSinceStartup.has_value())
            logger->filestream << vnObs->timeSinceStartup.value();
        logger->filestream << ",";
        if (vnObs->timeSinceSyncIn.has_value())
            logger->filestream << vnObs->timeSinceSyncIn.value();
        logger->filestream << ",";
        if (vnObs->syncInCnt.has_value())
            logger->filestream << vnObs->syncInCnt.value();
        logger->filestream << ",";
        if (vnObs->magUncompXYZ.has_value())
            logger->filestream << std::setprecision(9) << vnObs->magUncompXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->magUncompXYZ.has_value())
            logger->filestream << vnObs->magUncompXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->magUncompXYZ.has_value())
            logger->filestream << vnObs->magUncompXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->accelUncompXYZ.has_value())
            logger->filestream << vnObs->accelUncompXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->accelUncompXYZ.has_value())
            logger->filestream << vnObs->accelUncompXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->accelUncompXYZ.has_value())
            logger->filestream << vnObs->accelUncompXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->gyroUncompXYZ.has_value())
            logger->filestream << vnObs->gyroUncompXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->gyroUncompXYZ.has_value())
            logger->filestream << vnObs->gyroUncompXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->gyroUncompXYZ.has_value())
            logger->filestream << vnObs->gyroUncompXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->temperature.has_value())
            logger->filestream << vnObs->temperature.value();
        logger->filestream << ",";
        if (vnObs->pressure.has_value())
            logger->filestream << vnObs->pressure.value();
        logger->filestream << ",";
        if (vnObs->dtime.has_value())
            logger->filestream << vnObs->dtime.value();
        logger->filestream << ",";
        if (vnObs->dtheta.has_value())
            logger->filestream << vnObs->dtheta.value().x();
        logger->filestream << ",";
        if (vnObs->dtheta.has_value())
            logger->filestream << vnObs->dtheta.value().y();
        logger->filestream << ",";
        if (vnObs->dtheta.has_value())
            logger->filestream << vnObs->dtheta.value().z();
        logger->filestream << ",";
        if (vnObs->dvel.has_value())
            logger->filestream << vnObs->dvel.value().x();
        logger->filestream << ",";
        if (vnObs->dvel.has_value())
            logger->filestream << vnObs->dvel.value().y();
        logger->filestream << ",";
        if (vnObs->dvel.has_value())
            logger->filestream << vnObs->dvel.value().z();
        logger->filestream << ",";
        if (vnObs->magCompXYZ.has_value())
            logger->filestream << vnObs->magCompXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->magCompXYZ.has_value())
            logger->filestream << vnObs->magCompXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->magCompXYZ.has_value())
            logger->filestream << vnObs->magCompXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->accelCompXYZ.has_value())
            logger->filestream << vnObs->accelCompXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->accelCompXYZ.has_value())
            logger->filestream << vnObs->accelCompXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->accelCompXYZ.has_value())
            logger->filestream << vnObs->accelCompXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->gyroCompXYZ.has_value())
            logger->filestream << vnObs->gyroCompXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->gyroCompXYZ.has_value())
            logger->filestream << vnObs->gyroCompXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->gyroCompXYZ.has_value())
            logger->filestream << vnObs->gyroCompXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->vpeStatus.has_value())
            logger->filestream << vnObs->vpeStatus.value();
        logger->filestream << ",";
        if (vnObs->quaternion.has_value())
            logger->filestream << vnObs->quaternion.value().w();
        logger->filestream << ",";
        if (vnObs->quaternion.has_value())
            logger->filestream << vnObs->quaternion.value().x();
        logger->filestream << ",";
        if (vnObs->quaternion.has_value())
            logger->filestream << vnObs->quaternion.value().y();
        logger->filestream << ",";
        if (vnObs->quaternion.has_value())
            logger->filestream << vnObs->quaternion.value().z();
        logger->filestream << ",";
        if (vnObs->magCompNED.has_value())
            logger->filestream << vnObs->magCompNED.value().x();
        logger->filestream << ",";
        if (vnObs->magCompNED.has_value())
            logger->filestream << vnObs->magCompNED.value().y();
        logger->filestream << ",";
        if (vnObs->magCompNED.has_value())
            logger->filestream << vnObs->magCompNED.value().z();
        logger->filestream << ",";
        if (vnObs->accelCompNED.has_value())
            logger->filestream << vnObs->accelCompNED.value().x();
        logger->filestream << ",";
        if (vnObs->accelCompNED.has_value())
            logger->filestream << vnObs->accelCompNED.value().y();
        logger->filestream << ",";
        if (vnObs->accelCompNED.has_value())
            logger->filestream << vnObs->accelCompNED.value().z();
        logger->filestream << ",";
        if (vnObs->linearAccelXYZ.has_value())
            logger->filestream << vnObs->linearAccelXYZ.value().x();
        logger->filestream << ",";
        if (vnObs->linearAccelXYZ.has_value())
            logger->filestream << vnObs->linearAccelXYZ.value().y();
        logger->filestream << ",";
        if (vnObs->linearAccelXYZ.has_value())
            logger->filestream << vnObs->linearAccelXYZ.value().z();
        logger->filestream << ",";
        if (vnObs->linearAccelNED.has_value())
            logger->filestream << vnObs->linearAccelNED.value().x();
        logger->filestream << ",";
        if (vnObs->linearAccelNED.has_value())
            logger->filestream << vnObs->linearAccelNED.value().y();
        logger->filestream << ",";
        if (vnObs->linearAccelNED.has_value())
            logger->filestream << vnObs->linearAccelNED.value().z();
        logger->filestream << ",";
        if (vnObs->yawPitchRollUncertainty.has_value())
            logger->filestream << vnObs->yawPitchRollUncertainty.value().x();
        logger->filestream << ",";
        if (vnObs->yawPitchRollUncertainty.has_value())
            logger->filestream << vnObs->yawPitchRollUncertainty.value().y();
        logger->filestream << ",";
        if (vnObs->yawPitchRollUncertainty.has_value())
            logger->filestream << vnObs->yawPitchRollUncertainty.value().z();
        logger->filestream << ",";
        if (vnObs->gyroCompNED.has_value())
            logger->filestream << vnObs->gyroCompNED.value().x();
        logger->filestream << ",";
        if (vnObs->gyroCompNED.has_value())
            logger->filestream << vnObs->gyroCompNED.value().y();
        logger->filestream << ",";
        if (vnObs->gyroCompNED.has_value())
            logger->filestream << vnObs->gyroCompNED.value().z();
        logger->filestream << std::endl;
    }

    return NavStatus::NAV_OK;
}