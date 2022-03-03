#include <catch2/catch.hpp>
#include <limits>

#include "Nodes/FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

#include "util/Logger.hpp"

namespace NAV::TEST::LooselyCoupledKF
{
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

size_t messageCounterVectorNavBinaryConverterIMU2ImuIntegrator = 0;
size_t messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer = 0;
size_t messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef = 0;
size_t messageCounterVectorNavBinaryConverterGNSS2PosVelAttInitializer = 0;

std::shared_ptr<const NAV::ImuObsWDelta> vnBinImu2ImuIntegrator = nullptr;
std::shared_ptr<const NAV::ImuObsWDelta> vnBinImu2PosVelAttInitializer = nullptr;
std::shared_ptr<const NAV::ImuObsWDelta> vnBinImu2ImuIntegratorRef = nullptr;

size_t messageCounterIntegratorResult = 0;
size_t messageCounterIntegratorResultRef = 0;

std::shared_ptr<const NAV::InertialNavSol> integratorResult = nullptr;
std::shared_ptr<const NAV::InertialNavSol> integratorResultRef = nullptr;

void compareVectorNavBinaryImu()
{
    if (!vnBinImu2ImuIntegrator || !vnBinImu2PosVelAttInitializer || !vnBinImu2ImuIntegratorRef)
    {
        return;
    }

    // ---------------------------------------- Check if same pointers -------------------------------------------
    REQUIRE(vnBinImu2ImuIntegrator == vnBinImu2PosVelAttInitializer);
    REQUIRE(vnBinImu2ImuIntegrator == vnBinImu2ImuIntegratorRef);

    // ------------------------------------------------- Reset ---------------------------------------------------
    vnBinImu2ImuIntegrator.reset();
    vnBinImu2PosVelAttInitializer.reset();
    vnBinImu2ImuIntegratorRef.reset();
}

void compareIntegratorResults()
{
    if (!integratorResult || !integratorResultRef)
    {
        return;
    }

    // ------------------------------------------------ InsTime --------------------------------------------------

    REQUIRE(integratorResult->insTime.has_value());
    REQUIRE(integratorResultRef->insTime.has_value());

    REQUIRE(integratorResult->insTime->toGPSweekTow().gpsCycle == integratorResultRef->insTime->toGPSweekTow().gpsCycle);
    REQUIRE(integratorResult->insTime->toGPSweekTow().gpsWeek == integratorResultRef->insTime->toGPSweekTow().gpsWeek);
    REQUIRE(integratorResult->insTime->toGPSweekTow().tow == Approx(integratorResultRef->insTime->toGPSweekTow().tow).margin(EPSILON));

    // ------------------------------------------------ Values ---------------------------------------------------

    REQUIRE(integratorResult->latitude() == Approx(integratorResultRef->latitude()).margin(EPSILON));
    REQUIRE(integratorResult->longitude() == Approx(integratorResultRef->longitude()).margin(EPSILON));
    REQUIRE(integratorResult->altitude() == Approx(integratorResultRef->altitude()).margin(EPSILON));
    REQUIRE(integratorResult->n_velocity()(0) == Approx(integratorResultRef->n_velocity()(0)).margin(EPSILON));
    REQUIRE(integratorResult->n_velocity()(1) == Approx(integratorResultRef->n_velocity()(1)).margin(EPSILON));
    REQUIRE(integratorResult->n_velocity()(2) == Approx(integratorResultRef->n_velocity()(2)).margin(EPSILON));
    REQUIRE(integratorResult->n_Quat_b().w() == Approx(integratorResultRef->n_Quat_b().w()).margin(EPSILON));
    REQUIRE(integratorResult->n_Quat_b().x() == Approx(integratorResultRef->n_Quat_b().x()).margin(EPSILON));
    REQUIRE(integratorResult->n_Quat_b().y() == Approx(integratorResultRef->n_Quat_b().y()).margin(EPSILON));
    REQUIRE(integratorResult->n_Quat_b().z() == Approx(integratorResultRef->n_Quat_b().z()).margin(EPSILON));

    // ------------------------------------------------- Reset ---------------------------------------------------
    integratorResult.reset();
    integratorResultRef.reset();
}

TEST_CASE("[CallbackManager] Flow order", "[CallbackManager]")
{
    messageCounterVectorNavBinaryConverterIMU2ImuIntegrator = 0;
    messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer = 0;
    messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef = 0;
    messageCounterVectorNavBinaryConverterGNSS2PosVelAttInitializer = 0;
    messageCounterIntegratorResult = 0;
    messageCounterIntegratorResultRef = 0;

    Logger logger;

    // ###########################################################################################################
    //                                      VectorNavBinaryConverter - IMU
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToLink(991, [](const std::shared_ptr<const NAV::NodeData>& data) { // "VectorNavBinaryConverter - IMU" === ImuObsWDelta ==> "ImuIntegrator"
        messageCounterVectorNavBinaryConverterIMU2ImuIntegrator++;

        LOG_TRACE("msgCntVNBinConvIMU2ImuIntegrator = {}, msgCntVNBinConvIMU2PosVelAttInitializer = {}, msgCntVNBinConvIMU2ImuIntegratorRef = {}",
                  messageCounterVectorNavBinaryConverterIMU2ImuIntegrator, messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer, messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef);

        REQUIRE(vnBinImu2ImuIntegrator == nullptr); // Check if messages are missing in the other nodes
        vnBinImu2ImuIntegrator = std::static_pointer_cast<const NAV::ImuObsWDelta>(data);

        compareVectorNavBinaryImu();
    });

    nm::RegisterWatcherCallbackToLink(548, [](const std::shared_ptr<const NAV::NodeData>& data) { // "VectorNavBinaryConverter - IMU" === ImuObsWDelta ==> "PosVelAttInitializer"
        messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer++;

        LOG_TRACE("msgCntVNBinConvIMU2ImuIntegrator = {}, msgCntVNBinConvIMU2PosVelAttInitializer = {}, msgCntVNBinConvIMU2ImuIntegratorRef = {}",
                  messageCounterVectorNavBinaryConverterIMU2ImuIntegrator, messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer, messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef);

        REQUIRE(vnBinImu2PosVelAttInitializer == nullptr); // Check if messages are missing in the other nodes
        vnBinImu2PosVelAttInitializer = std::static_pointer_cast<const NAV::ImuObsWDelta>(data);

        compareVectorNavBinaryImu();
    });

    nm::RegisterWatcherCallbackToLink(1200, [](const std::shared_ptr<const NAV::NodeData>& data) { // "VectorNavBinaryConverter - IMU" === ImuObsWDelta ==> "ImuIntegrator - Ref"
        messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef++;

        LOG_TRACE("msgCntVNBinConvIMU2ImuIntegrator = {}, msgCntVNBinConvIMU2PosVelAttInitializer = {}, msgCntVNBinConvIMU2ImuIntegratorRef = {}",
                  messageCounterVectorNavBinaryConverterIMU2ImuIntegrator, messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer, messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef);

        REQUIRE(vnBinImu2ImuIntegratorRef == nullptr); // Check if messages are missing in the other nodes
        vnBinImu2ImuIntegratorRef = std::static_pointer_cast<const NAV::ImuObsWDelta>(data);

        compareVectorNavBinaryImu();
    });

    // ###########################################################################################################
    //                                     VectorNavBinaryConverter - GNSS
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToLink(498, [](const std::shared_ptr<const NAV::NodeData>& /*data*/) { // "VectorNavBinaryConverter - GNSS" === PosVelAtt ==> "PosVelAttInitializer"
        messageCounterVectorNavBinaryConverterGNSS2PosVelAttInitializer++;

        LOG_TRACE("messageCounterVectorNavBinaryConverterGNSS2PosVelAttInitializer = {}", messageCounterVectorNavBinaryConverterGNSS2PosVelAttInitializer);
    });

    // ###########################################################################################################
    //                                               ImuIntegrator
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToLink(1206, [](const std::shared_ptr<const NAV::NodeData>& data) { // "ImuIntegrator - Ref" === InertialNavSol ==> "Combiner (821)"
        messageCounterIntegratorResultRef++;

        LOG_TRACE("messageCounterIntegratorResult = {}, messageCounterIntegratorResultRef = {}", messageCounterIntegratorResult, messageCounterIntegratorResultRef);

        REQUIRE(integratorResultRef == nullptr); // Check if messages are missing in the other nodes
        integratorResultRef = std::static_pointer_cast<const NAV::InertialNavSol>(data);

        CHECK(messageCounterVectorNavBinaryConverterIMU2ImuIntegrator == messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer);
        CHECK(messageCounterVectorNavBinaryConverterIMU2ImuIntegrator == messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef);

        compareIntegratorResults();
    });

    nm::RegisterWatcherCallbackToLink(785, [](const std::shared_ptr<const NAV::NodeData>& data) { // "ImuIntegrator" === InertialNavSol ==> "Combiner (507)"
        messageCounterIntegratorResult++;

        LOG_TRACE("messageCounterIntegratorResult = {}, messageCounterIntegratorResultRef = {}", messageCounterIntegratorResult, messageCounterIntegratorResultRef);

        REQUIRE(integratorResult == nullptr); // Check if messages are missing in the other nodes
        integratorResult = std::static_pointer_cast<const NAV::InertialNavSol>(data);

        CHECK(messageCounterVectorNavBinaryConverterIMU2ImuIntegrator == messageCounterVectorNavBinaryConverterIMU2PosVelAttInitializer);
        CHECK(messageCounterVectorNavBinaryConverterIMU2ImuIntegrator == messageCounterVectorNavBinaryConverterIMU2ImuIntegratorRef);

        compareIntegratorResults();
    });

    REQUIRE(testFlow("test/flow/internal/CallbackManager.flow"));
}
} // namespace NAV::TEST::LooselyCoupledKF