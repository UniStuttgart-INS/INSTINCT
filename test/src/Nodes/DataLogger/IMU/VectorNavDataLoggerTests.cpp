#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include "Nodes/FlowTester.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

namespace NAV::TEST::VectorNavDataLogger
{
constexpr int MESSAGE_COUNT_IMU = 18;  ///< Amount of messages expected in the Imu files
constexpr int MESSAGE_COUNT_GNSS = 12; ///< Amount of messages expected in the Gnss files

int messageCounterImuDataCsv = 0;  ///< Message Counter for the Imu data csv file
int messageCounterImuLogCsv = 0;   ///< Message Counter for the Imu log csv file
int messageCounterImuLogVnb = 0;   ///< Message Counter for the Imu log vnb file
int messageCounterGnssDataCsv = 0; ///< Message Counter for the Gnss data csv fil
int messageCounterGnssLogCsv = 0;  ///< Message Counter for the Gnss log csv file
int messageCounterGnssLogVnb = 0;  ///< Message Counter for the Gnss log vnb file

// TEST_CASE("[VectorNavDataReader] Read csv file and compare content with hardcoded values", "[VectorNavDataReader]")
// {
// TODO: Implement this test
// }

TEST_CASE("[VectorNavDataLogger] Read and log files and compare content", "[VectorNavDataLogger]")
{
    messageCounterImuDataCsv = 0;
    messageCounterImuLogCsv = 0;
    messageCounterImuLogVnb = 0;
    messageCounterGnssDataCsv = 0;
    messageCounterGnssLogCsv = 0;
    messageCounterGnssLogVnb = 0;

    Logger logger;

    // ###########################################################################################################
    //                                         VectorNavDataLogger.flow
    // ###########################################################################################################
    //
    //                                     / VectorNavDataLogger("logs/vn310-imu.csv")
    // VectorNavFile("data/vn310-imu.csv")
    //                                     \ VectorNavDataLogger("logs/vn310-imu.vnb")
    //
    //                                     / VectorNavDataLogger("logs/vn310-gnss.csv")
    // VectorNavFile("data/vn310-gnss.csv")
    //                                     \ VectorNavDataLogger("logs/vn310-gnss.vnb")
    //
    // ###########################################################################################################

    testFlow("test/flow/VectorNavDataLogger.flow");

    // ###########################################################################################################
    //                                         VectorNavDataReader.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/vn310-imu.csv")
    // VectorNavFile("logs/vn310-imu.csv")
    // VectorNavFile("logs/vn310-imu.vnb")
    // VectorNavFile("data/vn310-gnss.csv")
    // VectorNavFile("logs/vn310-gnss.csv")
    // VectorNavFile("logs/vn310-gnss.vnb")
    //
    // ###########################################################################################################

    // -------------------------------------------------- IMU ----------------------------------------------------

    nm::RegisterWatcherCallbackToOutputPin(1, [](const std::shared_ptr<NAV::NodeData>& data) mutable { // test/data/vn310-imu.csv
        messageCounterImuDataCsv++;

        [[maybe_unused]] auto obs = std::dynamic_pointer_cast<NAV::VectorNavBinaryOutput>(data);

        // TODO: Compare Data with other files
    });

    nm::RegisterWatcherCallbackToOutputPin(29, [](const std::shared_ptr<NAV::NodeData>& data) { // test/logs/vn310-imu.csv
        messageCounterImuLogCsv++;

        [[maybe_unused]] auto obs = std::dynamic_pointer_cast<NAV::VectorNavBinaryOutput>(data);

        // TODO: Compare Data with other files
    });

    nm::RegisterWatcherCallbackToOutputPin(32, [](const std::shared_ptr<NAV::NodeData>& data) { // test/logs/vn310-imu.vnb
        messageCounterImuLogVnb++;

        [[maybe_unused]] auto obs = std::dynamic_pointer_cast<NAV::VectorNavBinaryOutput>(data);

        // TODO: Compare Data with other files
    });

    // ------------------------------------------------- GNSS ----------------------------------------------------

    nm::RegisterWatcherCallbackToOutputPin(7, [](const std::shared_ptr<NAV::NodeData>& data) { // test/data/vn310-gnss.csv
        messageCounterGnssDataCsv++;

        [[maybe_unused]] auto obs = std::dynamic_pointer_cast<NAV::VectorNavBinaryOutput>(data);

        // TODO: Compare Data with other files
    });

    nm::RegisterWatcherCallbackToOutputPin(35, [](const std::shared_ptr<NAV::NodeData>& data) { // test/logs/vn310-gnss.csv
        messageCounterGnssLogCsv++;

        [[maybe_unused]] auto obs = std::dynamic_pointer_cast<NAV::VectorNavBinaryOutput>(data);

        // TODO: Compare Data with other files
    });

    nm::RegisterWatcherCallbackToOutputPin(38, [](const std::shared_ptr<NAV::NodeData>& data) { // test/logs/vn310-gnss.vnb
        messageCounterGnssLogVnb++;

        [[maybe_unused]] auto obs = std::dynamic_pointer_cast<NAV::VectorNavBinaryOutput>(data);

        // TODO: Compare Data with other files
    });

    testFlow("test/flow/VectorNavDataReader.flow");

    CHECK(messageCounterImuDataCsv == MESSAGE_COUNT_IMU);
    CHECK(messageCounterImuLogCsv == MESSAGE_COUNT_IMU);
    CHECK(messageCounterImuLogVnb == MESSAGE_COUNT_IMU);
    CHECK(messageCounterGnssDataCsv == MESSAGE_COUNT_GNSS);
    CHECK(messageCounterGnssLogCsv == MESSAGE_COUNT_GNSS);
    CHECK(messageCounterGnssLogVnb == MESSAGE_COUNT_GNSS);
}

} // namespace NAV::TEST::VectorNavDataLogger
