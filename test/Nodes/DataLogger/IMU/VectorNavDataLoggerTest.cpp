#include <catch2/catch.hpp>

#include <memory>
#include <deque>

#include "util/Logger.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"

#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"

namespace NAV
{
TEST_CASE("VectorNavDataLogger: Read file and pass data to logger. Then read the logged file and compare data", "[VectorNavDataLogger]")
{
    Logger logger;

    system("pwd"); // NOLINT

    // Create VectorNavFile Node
    std::deque<std::string> optionsFile = { "../../../test/data/vectornav.csv" };
    auto vnFile = std::make_shared<VectorNavFile>("VN-File", optionsFile);

    // Create Logger
    std::deque<std::string> optionsLogger = { "../../../test/logs/vectornav.csv", "ascii" };
    auto vnLogger = std::make_shared<VectorNavDataLogger>("VN-DataLogger", optionsLogger);
    auto target = std::static_pointer_cast<Node>(vnLogger);

    constexpr int sourcePortIndex = 0;
    constexpr int targetPortIndex = 0;

    // Configure callbacks
    vnFile->addCallback<VectorNavObs>(target, targetPortIndex);
    vnLogger->incomingLinks.emplace(targetPortIndex, std::make_pair(vnFile, sourcePortIndex));
    vnFile->callbacksEnabled = true;

    while (vnFile->requestOutputData(sourcePortIndex) != nullptr) {}

    // Delete the nodes which should cause a flush
    target = nullptr;
    vnLogger = nullptr;
    vnFile = nullptr;

    // Create VectorNavFile Node
    std::deque<std::string> optionsFileOrig = { "../../../test/data/vectornav.csv" };
    auto vnFileOriginal = std::make_shared<VectorNavFile>("VN-File-Orig", optionsFileOrig);
    // Create VectorNavFile Node
    std::deque<std::string> optionsFileNew = { "../../../test/logs/vectornav.csv" };
    auto vnFileNew = std::make_shared<VectorNavFile>("VN-File-New", optionsFileNew);

    while (true)
    {
        auto obsNew = std::static_pointer_cast<VectorNavObs>(vnFileNew->requestOutputData(sourcePortIndex));
        auto obsOrig = std::static_pointer_cast<VectorNavObs>(vnFileOriginal->requestOutputData(sourcePortIndex));

        if (obsOrig == nullptr || obsNew == nullptr)
        {
            break;
        }

        if (obsOrig->timeSinceStartup.has_value())
        {
            REQUIRE(obsOrig->timeSinceStartup.value() == obsNew->timeSinceStartup.value());
        }
        if (obsOrig->timeSinceSyncIn.has_value())
        {
            REQUIRE(obsOrig->timeSinceSyncIn.value() == obsNew->timeSinceSyncIn.value());
        }
        if (obsOrig->syncInCnt.has_value())
        {
            REQUIRE(obsOrig->syncInCnt.value() == Approx(obsNew->syncInCnt.value()));
        }
        if (obsOrig->temperature.has_value())
        {
            REQUIRE(obsOrig->temperature.value() == Approx(obsNew->temperature.value()));
        }
        if (obsOrig->vpeStatus.has_value())
        {
            REQUIRE(obsOrig->vpeStatus.value().status == obsNew->vpeStatus.value().status);
        }
        if (obsOrig->pressure.has_value())
        {
            REQUIRE(obsOrig->pressure.value() == Approx(obsNew->pressure.value()));
        }
        if (obsOrig->insTime.has_value())
        {
            REQUIRE(obsOrig->insTime.value() == obsNew->insTime.value());
        }
        if (obsOrig->yawPitchRollUncertainty.has_value())
        {
            REQUIRE(obsOrig->yawPitchRollUncertainty.value().x() == Approx(obsNew->yawPitchRollUncertainty.value().x()));
            REQUIRE(obsOrig->yawPitchRollUncertainty.value().y() == Approx(obsNew->yawPitchRollUncertainty.value().y()));
            REQUIRE(obsOrig->yawPitchRollUncertainty.value().z() == Approx(obsNew->yawPitchRollUncertainty.value().z()));
        }
        if (obsOrig->quaternion.has_value())
        {
            REQUIRE(obsOrig->quaternion.value().w() == Approx(obsNew->quaternion.value().w()));
            REQUIRE(obsOrig->quaternion.value().x() == Approx(obsNew->quaternion.value().x()));
            REQUIRE(obsOrig->quaternion.value().y() == Approx(obsNew->quaternion.value().y()));
            REQUIRE(obsOrig->quaternion.value().z() == Approx(obsNew->quaternion.value().z()));
        }
        if (obsOrig->accelCompNED.has_value())
        {
            REQUIRE(obsOrig->accelCompNED.value()(0) == Approx(obsNew->accelCompNED.value()(0)));
            REQUIRE(obsOrig->accelCompNED.value()(1) == Approx(obsNew->accelCompNED.value()(1)));
            REQUIRE(obsOrig->accelCompNED.value()(2) == Approx(obsNew->accelCompNED.value()(2)));
        }
        if (obsOrig->accelCompXYZ.has_value())
        {
            REQUIRE(obsOrig->accelCompXYZ.value()(0) == Approx(obsNew->accelCompXYZ.value()(0)));
            REQUIRE(obsOrig->accelCompXYZ.value()(1) == Approx(obsNew->accelCompXYZ.value()(1)));
            REQUIRE(obsOrig->accelCompXYZ.value()(2) == Approx(obsNew->accelCompXYZ.value()(2)));
        }
        if (obsOrig->accelUncompXYZ.has_value())
        {
            REQUIRE(obsOrig->accelUncompXYZ.value()(0) == Approx(obsNew->accelUncompXYZ.value()(0)));
            REQUIRE(obsOrig->accelUncompXYZ.value()(1) == Approx(obsNew->accelUncompXYZ.value()(1)));
            REQUIRE(obsOrig->accelUncompXYZ.value()(2) == Approx(obsNew->accelUncompXYZ.value()(2)));
        }
        if (obsOrig->dtheta.has_value())
        {
            REQUIRE(obsOrig->dtheta.value()(0) == Approx(obsNew->dtheta.value()(0)));
            REQUIRE(obsOrig->dtheta.value()(1) == Approx(obsNew->dtheta.value()(1)));
            REQUIRE(obsOrig->dtheta.value()(2) == Approx(obsNew->dtheta.value()(2)));
        }
        if (obsOrig->dtime.has_value())
        {
            REQUIRE(obsOrig->dtime.value() == Approx(obsNew->dtime.value()));
        }
        if (obsOrig->dvel.has_value())
        {
            REQUIRE(obsOrig->dvel.value()(0) == Approx(obsNew->dvel.value()(0)));
            REQUIRE(obsOrig->dvel.value()(1) == Approx(obsNew->dvel.value()(1)));
            REQUIRE(obsOrig->dvel.value()(2) == Approx(obsNew->dvel.value()(2)));
        }
        if (obsOrig->gyroCompNED.has_value())
        {
            REQUIRE(obsOrig->gyroCompNED.value()(0) == Approx(obsNew->gyroCompNED.value()(0)));
            REQUIRE(obsOrig->gyroCompNED.value()(1) == Approx(obsNew->gyroCompNED.value()(1)));
            REQUIRE(obsOrig->gyroCompNED.value()(2) == Approx(obsNew->gyroCompNED.value()(2)));
        }
        if (obsOrig->gyroCompXYZ.has_value())
        {
            REQUIRE(obsOrig->gyroCompXYZ.value()(0) == Approx(obsNew->gyroCompXYZ.value()(0)));
            REQUIRE(obsOrig->gyroCompXYZ.value()(1) == Approx(obsNew->gyroCompXYZ.value()(1)));
            REQUIRE(obsOrig->gyroCompXYZ.value()(2) == Approx(obsNew->gyroCompXYZ.value()(2)));
        }
        if (obsOrig->gyroUncompXYZ.has_value())
        {
            REQUIRE(obsOrig->gyroUncompXYZ.value()(0) == Approx(obsNew->gyroUncompXYZ.value()(0)));
            REQUIRE(obsOrig->gyroUncompXYZ.value()(1) == Approx(obsNew->gyroUncompXYZ.value()(1)));
            REQUIRE(obsOrig->gyroUncompXYZ.value()(2) == Approx(obsNew->gyroUncompXYZ.value()(2)));
        }
        if (obsOrig->linearAccelNED.has_value())
        {
            REQUIRE(obsOrig->linearAccelNED.value()(0) == Approx(obsNew->linearAccelNED.value()(0)));
            REQUIRE(obsOrig->linearAccelNED.value()(1) == Approx(obsNew->linearAccelNED.value()(1)));
            REQUIRE(obsOrig->linearAccelNED.value()(2) == Approx(obsNew->linearAccelNED.value()(2)));
        }
        if (obsOrig->linearAccelXYZ.has_value())
        {
            REQUIRE(obsOrig->linearAccelXYZ.value()(0) == Approx(obsNew->linearAccelXYZ.value()(0)));
            REQUIRE(obsOrig->linearAccelXYZ.value()(1) == Approx(obsNew->linearAccelXYZ.value()(1)));
            REQUIRE(obsOrig->linearAccelXYZ.value()(2) == Approx(obsNew->linearAccelXYZ.value()(2)));
        }
        if (obsOrig->magCompNED.has_value())
        {
            REQUIRE(obsOrig->magCompNED.value()(0) == Approx(obsNew->magCompNED.value()(0)));
            REQUIRE(obsOrig->magCompNED.value()(1) == Approx(obsNew->magCompNED.value()(1)));
            REQUIRE(obsOrig->magCompNED.value()(2) == Approx(obsNew->magCompNED.value()(2)));
        }
        if (obsOrig->magCompXYZ.has_value())
        {
            REQUIRE(obsOrig->magCompXYZ.value()(0) == Approx(obsNew->magCompXYZ.value()(0)));
            REQUIRE(obsOrig->magCompXYZ.value()(1) == Approx(obsNew->magCompXYZ.value()(1)));
            REQUIRE(obsOrig->magCompXYZ.value()(2) == Approx(obsNew->magCompXYZ.value()(2)));
        }
        if (obsOrig->magUncompXYZ.has_value())
        {
            REQUIRE(obsOrig->magUncompXYZ.value()(0) == Approx(obsNew->magUncompXYZ.value()(0)));
            REQUIRE(obsOrig->magUncompXYZ.value()(1) == Approx(obsNew->magUncompXYZ.value()(1)));
            REQUIRE(obsOrig->magUncompXYZ.value()(2) == Approx(obsNew->magUncompXYZ.value()(2)));
        }
    }
}

} // namespace NAV