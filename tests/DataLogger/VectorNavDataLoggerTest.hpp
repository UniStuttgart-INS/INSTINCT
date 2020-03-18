#pragma once

#include <gtest/gtest.h>

#include <memory>

#include "DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "DataProcessor/DataLogger/IMU/VectorNavDataLogger.hpp"
#include "DataProvider/IMU/Observations/VectorNavObs.hpp"

namespace NAV
{
TEST(VectorNavDataLogger, UnitTest)
{
    NAV::VectorNavFile vn100File("VN-100-File", "tests/data/vectornav.csv", {});
    auto vn100logger = std::make_shared<NAV::VectorNavDataLogger>("VN-100-Logger", "tests/logs/vn100.csv", false);
    if (vn100File.initialize() == NAV::NavStatus::NAV_OK
        && vn100logger->initialize() == NAV::NavStatus::NAV_OK)
    {
        vn100File.addCallback(vn100logger->writeObservation, vn100logger);

        vn100File.callbacksEnabled = true;

        while (vn100File.pollObservation() != nullptr)
            ;

        vn100logger->deinitialize();
        vn100File.deinitialize();
    }

    NAV::VectorNavFile vn100File2("VN-100-File2", "tests/logs/vn100.csv", {});
    auto vn100logger2 = std::make_shared<NAV::VectorNavDataLogger>("VN-100-Logger2", "tests/logs/vn100-2.csv", false);
    if (vn100File2.initialize() == NAV::NavStatus::NAV_OK
        && vn100logger2->initialize() == NAV::NavStatus::NAV_OK)
    {
        vn100File2.addCallback(vn100logger2->writeObservation, vn100logger2);

        vn100File2.callbacksEnabled = true;

        while (vn100File2.pollObservation() != nullptr)
            ;

        vn100logger2->deinitialize();
        vn100File2.deinitialize();
    }

    NAV::VectorNavFile vn100File3("VN-100-File3", "tests/logs/vn100-2.csv", {});
    if (vn100File.initialize() == NAV::NavStatus::NAV_OK
        && vn100File3.initialize() == NAV::NavStatus::NAV_OK)
    {
        std::shared_ptr<NAV::VectorNavObs> obs1, obs2;
        do
        {
            obs1 = std::static_pointer_cast<NAV::VectorNavObs>(vn100File.pollObservation());
            obs2 = std::static_pointer_cast<NAV::VectorNavObs>(vn100File3.pollObservation());

            if (obs1->timeSinceStartup.has_value())
                EXPECT_FLOAT_EQ(obs1->timeSinceStartup.value(), obs2->timeSinceStartup.value());
            if (obs1->timeSinceSyncIn.has_value())
                EXPECT_FLOAT_EQ(obs1->timeSinceSyncIn.value(), obs2->timeSinceSyncIn.value());
            if (obs1->syncInCnt.has_value())
                EXPECT_FLOAT_EQ(obs1->syncInCnt.value(), obs2->syncInCnt.value());
            if (obs1->temperature.has_value())
                EXPECT_FLOAT_EQ(obs1->temperature.value(), obs2->temperature.value());
            if (obs1->vpeStatus.has_value())
                EXPECT_FLOAT_EQ(obs1->vpeStatus.value(), obs2->vpeStatus.value());
            if (obs1->pressure.has_value())
                EXPECT_FLOAT_EQ(obs1->pressure.value(), obs2->pressure.value());
            if (obs1->gpsTimeOfWeek.has_value())
                EXPECT_FLOAT_EQ(obs1->gpsTimeOfWeek.value(), obs2->gpsTimeOfWeek.value());
            if (obs1->gpsWeek.has_value())
                EXPECT_FLOAT_EQ(obs1->gpsWeek.value(), obs2->gpsWeek.value());
            if (obs1->yawPitchRollUncertainty.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->yawPitchRollUncertainty.value().x, obs2->yawPitchRollUncertainty.value().x);
                EXPECT_FLOAT_EQ(obs1->yawPitchRollUncertainty.value().y, obs2->yawPitchRollUncertainty.value().y);
                EXPECT_FLOAT_EQ(obs1->yawPitchRollUncertainty.value().z, obs2->yawPitchRollUncertainty.value().z);
            }
            if (obs1->quaternion.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->quaternion.value()(0), obs2->quaternion.value()(0));
                EXPECT_FLOAT_EQ(obs1->quaternion.value()(1), obs2->quaternion.value()(1));
                EXPECT_FLOAT_EQ(obs1->quaternion.value()(2), obs2->quaternion.value()(2));
                EXPECT_FLOAT_EQ(obs1->quaternion.value()(3), obs2->quaternion.value()(3));
            }
            if (obs1->accelCompNED.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->accelCompNED.value()(0), obs2->accelCompNED.value()(0));
                EXPECT_FLOAT_EQ(obs1->accelCompNED.value()(1), obs2->accelCompNED.value()(1));
                EXPECT_FLOAT_EQ(obs1->accelCompNED.value()(2), obs2->accelCompNED.value()(2));
            }
            if (obs1->accelCompXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->accelCompXYZ.value()(0), obs2->accelCompXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->accelCompXYZ.value()(1), obs2->accelCompXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->accelCompXYZ.value()(2), obs2->accelCompXYZ.value()(2));
            }
            if (obs1->accelUncompXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->accelUncompXYZ.value()(0), obs2->accelUncompXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->accelUncompXYZ.value()(1), obs2->accelUncompXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->accelUncompXYZ.value()(2), obs2->accelUncompXYZ.value()(2));
            }
            if (obs1->dtheta.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->dtheta.value()(0), obs2->dtheta.value()(0));
                EXPECT_FLOAT_EQ(obs1->dtheta.value()(1), obs2->dtheta.value()(1));
                EXPECT_FLOAT_EQ(obs1->dtheta.value()(2), obs2->dtheta.value()(2));
            }
            if (obs1->dtime.has_value())
                EXPECT_FLOAT_EQ(obs1->dtime.value(), obs2->dtime.value());
            if (obs1->dvel.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->dvel.value()(0), obs2->dvel.value()(0));
                EXPECT_FLOAT_EQ(obs1->dvel.value()(1), obs2->dvel.value()(1));
                EXPECT_FLOAT_EQ(obs1->dvel.value()(2), obs2->dvel.value()(2));
            }
            if (obs1->gyroCompNED.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->gyroCompNED.value()(0), obs2->gyroCompNED.value()(0));
                EXPECT_FLOAT_EQ(obs1->gyroCompNED.value()(1), obs2->gyroCompNED.value()(1));
                EXPECT_FLOAT_EQ(obs1->gyroCompNED.value()(2), obs2->gyroCompNED.value()(2));
            }
            if (obs1->gyroCompXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->gyroCompXYZ.value()(0), obs2->gyroCompXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->gyroCompXYZ.value()(1), obs2->gyroCompXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->gyroCompXYZ.value()(2), obs2->gyroCompXYZ.value()(2));
            }
            if (obs1->gyroUncompXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->gyroUncompXYZ.value()(0), obs2->gyroUncompXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->gyroUncompXYZ.value()(1), obs2->gyroUncompXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->gyroUncompXYZ.value()(2), obs2->gyroUncompXYZ.value()(2));
            }
            if (obs1->linearAccelNED.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->linearAccelNED.value()(0), obs2->linearAccelNED.value()(0));
                EXPECT_FLOAT_EQ(obs1->linearAccelNED.value()(1), obs2->linearAccelNED.value()(1));
                EXPECT_FLOAT_EQ(obs1->linearAccelNED.value()(2), obs2->linearAccelNED.value()(2));
            }
            if (obs1->linearAccelXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->linearAccelXYZ.value()(0), obs2->linearAccelXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->linearAccelXYZ.value()(1), obs2->linearAccelXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->linearAccelXYZ.value()(2), obs2->linearAccelXYZ.value()(2));
            }
            if (obs1->magCompNED.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->magCompNED.value()(0), obs2->magCompNED.value()(0));
                EXPECT_FLOAT_EQ(obs1->magCompNED.value()(1), obs2->magCompNED.value()(1));
                EXPECT_FLOAT_EQ(obs1->magCompNED.value()(2), obs2->magCompNED.value()(2));
            }
            if (obs1->magCompXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->magCompXYZ.value()(0), obs2->magCompXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->magCompXYZ.value()(1), obs2->magCompXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->magCompXYZ.value()(2), obs2->magCompXYZ.value()(2));
            }
            if (obs1->magUncompXYZ.has_value())
            {
                EXPECT_FLOAT_EQ(obs1->magUncompXYZ.value()(0), obs2->magUncompXYZ.value()(0));
                EXPECT_FLOAT_EQ(obs1->magUncompXYZ.value()(1), obs2->magUncompXYZ.value()(1));
                EXPECT_FLOAT_EQ(obs1->magUncompXYZ.value()(2), obs2->magUncompXYZ.value()(2));
            }
        } while (obs1 != nullptr);
    }
}

} // namespace NAV