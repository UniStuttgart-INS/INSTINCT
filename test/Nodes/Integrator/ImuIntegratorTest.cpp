#include <catch2/catch.hpp>

#include <memory>
#include <map>
#include <string>
#include <thread>
#include <chrono>

#include "fmt/core.h"

#include "util/InsGravity.hpp"

#include "Nodes/NodeManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/StateData.hpp"

#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/Integrator/ImuIntegrator.hpp"
#include "Nodes/State/State.hpp"
#include "Nodes/GnuPlot/GnuPlot.hpp"

#include "util/Sleep.hpp"
#include "util/Logger.hpp"
#include "util/InsMath.hpp"

#include <limits>

namespace NAV
{
constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

TEST_CASE("[ImuIntegrator] Integrate Observation NED", "[ImuIntegrator]")
{
    NodeManager::appContext = Node::NodeContext::POST_PROCESSING;

    Eigen::Vector3d mountingAngles(0.0, 0.0, 0.0);

    Eigen::Vector3d initLatLonHeight(0, 0.0, 500);
    Eigen::Vector3d initRollPitchYaw(0.0, 0.0, 0.0);
    Eigen::Vector3d initVelocityNED(0.0, 0.0, 0.0);

    std::map<std::string, std::string> optionsImuFile = { { "Path", "../../../test/data/vectornav.csv" },
                                                          { "Accel pos", "0;0;0" },
                                                          { "Accel rot", fmt::format("{:2.2f};{:2.2f};{:2.2f}", mountingAngles(0), mountingAngles(1), mountingAngles(2)) },
                                                          { "Gyro pos", "0;0;0" },
                                                          { "Gyro rot", fmt::format("{:2.2f};{:2.2f};{:2.2f}", mountingAngles(0), mountingAngles(1), mountingAngles(2)) },
                                                          { "Mag pos", "0;0;0" },
                                                          { "Mag rot", fmt::format("{:2.2f};{:2.2f};{:2.2f}", mountingAngles(0), mountingAngles(1), mountingAngles(2)) } };
    auto imuFile = std::make_shared<VectorNavFile>("ImuFile", optionsImuFile);

    std::map<std::string, std::string> optionsIntegrator = { { "Integration Frame", "NED" } };
    auto imuIntegrator = std::make_shared<ImuIntegrator>("ImuIntegrator", optionsIntegrator);

    std::map<std::string, std::string> optionsState = { { "Init LatLonAlt", fmt::format("{:2.8f};{:1.9f};{:3.4f}", initLatLonHeight(0), initLatLonHeight(1), initLatLonHeight(2)) },
                                                        { "Init RollPitchYaw", fmt::format("{:2.2f};{:2.2f};{:2.2f}", initRollPitchYaw(0), initRollPitchYaw(1), initRollPitchYaw(2)) },
                                                        { "Init Velocity", fmt::format("{:2.2f};{:2.2f};{:2.2f}", initVelocityNED(0), initVelocityNED(1), initVelocityNED(2)) } };
    auto state = std::make_shared<State>("State", optionsState);

    // std::map<std::string, std::string> optionsGnuPlotLatLon = { { "Start", "set autoscale xy\n"
    //                                                                        "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                        "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                        "set xlabel \"East [m]\"\n"
    //                                                                        "set ylabel \"North [m]\"\n" },
    //                                                             { "Input Ports", "1" },
    //                                                             { "1-Port Type", "StateData" },
    //                                                             { "1-Data to plot", "East [m];North [m]" },
    //                                                             { "1-Update", "plot [~1,2~] using 1:2 with lines title 'StateData'" } };
    // auto gnuPlotLatLon = std::make_shared<GnuPlot>("GnuPlot LatLon", optionsGnuPlotLatLon);

    // std::map<std::string, std::string> optionsGnuPlotLat = { { "Start", "set autoscale xy\n"
    //                                                                     "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                     "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                     "set xlabel \"Time [s]\"\n"
    //                                                                     "set ylabel \"Latitude [deg]\"\n" },
    //                                                          { "Input Ports", "1" },
    //                                                          { "1-Port Type", "StateData" },
    //                                                          { "1-Data to plot", "Time;Latitude" },
    //                                                          { "1-Update", "plot [~1,2~] using 1:2 with lines title 'StateData'" } };
    // auto gnuPlotLat = std::make_shared<GnuPlot>("GnuPlot Lat", optionsGnuPlotLat);

    // std::map<std::string, std::string> optionsGnuPlotLon = { { "Start", "set autoscale xy\n"
    //                                                                     "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                     "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                     "set xlabel \"Time [s]\"\n"
    //                                                                     "set ylabel \"Longitude [deg]\"\n" },
    //                                                          { "Input Ports", "1" },
    //                                                          { "1-Port Type", "StateData" },
    //                                                          { "1-Data to plot", "Time;Longitude" },
    //                                                          { "1-Update", "plot [~1,2~] using 1:2 with lines title 'StateData'" } };
    // auto gnuPlotLon = std::make_shared<GnuPlot>("GnuPlot Lon", optionsGnuPlotLon);

    // std::map<std::string, std::string> optionsGnuPlotHeight = { { "Start", "set autoscale xy\n"
    //                                                                        "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                        "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                        "set xlabel \"Time [s]\"\n"
    //                                                                        "set ylabel \"Height [m]\"\n" },
    //                                                             { "Input Ports", "1" },
    //                                                             { "1-Port Type", "StateData" },
    //                                                             { "1-Data to plot", "Time;Height" },
    //                                                             { "1-Update", "plot [~1,2~] using 1:2 with lines title 'StateData'" } };
    // auto gnuPlotHeight = std::make_shared<GnuPlot>("GnuPlot Height", optionsGnuPlotHeight);

    // std::map<std::string, std::string> optionsGnuPlotVelocity = { { "Start", "set autoscale xy\n"
    //                                                                          "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                          "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                          "set xlabel \"Time [s]\"\n"
    //                                                                          "set ylabel \"Velocity NED [m/s]\"\n" },
    //                                                               { "Input Ports", "1" },
    //                                                               { "1-Port Type", "StateData" },
    //                                                               { "1-Data to plot", "Time;Velocity North;Velocity East;Velocity Down" },
    //                                                               { "1-Update", "plot [~1,2,3,4~] using 1:2 with lines title 'North'\n"
    //                                                                             "plot [~1,2,3,4~] using 1:3 with lines title 'East'\n"
    //                                                                             "plot [~1,2,3,4~] using 1:4 with lines title 'Down'" } };
    // auto gnuPlotVelocity = std::make_shared<GnuPlot>("GnuPlot Velocity", optionsGnuPlotVelocity);

    // std::map<std::string, std::string> optionsGnuPlotAngles = { { "Start", "set autoscale xy\n"
    //                                                                        "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                        "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
    //                                                                        "set xlabel \"Time [s]\"\n"
    //                                                                        "set ylabel \"Flight Angles [deg]\"\n" },
    //                                                             { "Input Ports", "1" },
    //                                                             { "1-Port Type", "StateData" },
    //                                                             { "1-Data to plot", "Time;Roll;Pitch;Yaw" },
    //                                                             { "1-Update", "plot [~1,2,3,4~] using 1:2 with lines title 'Roll'\n"
    //                                                                           "plot [~1,2,3,4~] using 1:3 with lines title 'Pitch'\n"
    //                                                                           "plot [~1,2,3,4~] using 1:4 with lines title 'Yaw'" } };
    // auto gnuPlotAngles = std::make_shared<GnuPlot>("GnuPlot Angles", optionsGnuPlotAngles);

    // Configure callbacks
    auto imuIntegratorAsNode = std::static_pointer_cast<Node>(imuIntegrator);
    auto stateAsNode = std::static_pointer_cast<Node>(state);
    // auto gnuPlotLatAsNode = std::static_pointer_cast<Node>(gnuPlotLat);
    // auto gnuPlotLonAsNode = std::static_pointer_cast<Node>(gnuPlotLon);
    // auto gnuPlotLatLonAsNode = std::static_pointer_cast<Node>(gnuPlotLatLon);
    // auto gnuPlotHeightAsNode = std::static_pointer_cast<Node>(gnuPlotHeight);
    // auto gnuPlotVelocityAsNode = std::static_pointer_cast<Node>(gnuPlotVelocity);
    // auto gnuPlotAnglesAsNode = std::static_pointer_cast<Node>(gnuPlotAngles);

    imuFile->addCallback<VectorNavObs>(imuIntegratorAsNode, 0);
    imuIntegrator->incomingLinks.emplace(0, std::make_pair(imuFile, 0));

    state->addCallback<StateData>(imuIntegratorAsNode, 1);
    imuIntegrator->incomingLinks.emplace(1, std::make_pair(state, 0));

    imuIntegrator->addCallback<StateData>(stateAsNode, 0);
    state->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));

    std::static_pointer_cast<Node>(imuFile)->initialize();
    std::static_pointer_cast<Node>(imuIntegrator)->initialize();
    std::static_pointer_cast<Node>(state)->initialize();
    // std::static_pointer_cast<Node>(gnuPlotLatLon)->initialize();
    // std::static_pointer_cast<Node>(gnuPlotLat)->initialize();
    // std::static_pointer_cast<Node>(gnuPlotLon)->initialize();
    // std::static_pointer_cast<Node>(gnuPlotHeight)->initialize();
    // std::static_pointer_cast<Node>(gnuPlotVelocity)->initialize();
    // std::static_pointer_cast<Node>(gnuPlotAngles)->initialize();

    // imuIntegrator->addCallback<StateData>(gnuPlotLatLonAsNode, 0);
    // gnuPlotLatLon->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    // imuIntegrator->addCallback<StateData>(gnuPlotLatAsNode, 0);
    // gnuPlotLat->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    // imuIntegrator->addCallback<StateData>(gnuPlotLonAsNode, 0);
    // gnuPlotLon->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    // imuIntegrator->addCallback<StateData>(gnuPlotHeightAsNode, 0);
    // gnuPlotHeight->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    // imuIntegrator->addCallback<StateData>(gnuPlotVelocityAsNode, 0);
    // gnuPlotVelocity->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    // imuIntegrator->addCallback<StateData>(gnuPlotAnglesAsNode, 0);
    // gnuPlotAngles->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));

    imuIntegrator->callbacksEnabled = true;
    state->callbacksEnabled = true;

    auto currentState = std::static_pointer_cast<StateData>(state->requestOutputData(0));
    auto quat_pb = trafo::quat_pb(mountingAngles.x(), mountingAngles.y(), mountingAngles.z());

    long double dt = 0.1L;
    long double seconds = 10.0L;
    for (size_t i = 0; i < static_cast<size_t>(seconds / dt) + 1; i++)
    {
        double gravityNorm = gravity::gravityMagnitude_Gleason(currentState->latitude());

        Vector3d<Navigation> gravity_n{ 0, 0, -gravityNorm };
        Vector3d<Platform> gravity_p = quat_pb * currentState->quaternion_bn() * gravity_n;

        /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
        Vector3d<Platform> angularVelocity_ie_p = quat_pb * currentState->quaternion_be() * InsConst::angularVelocity_ie_e;

        Vector3d<Navigation> accel_n(0, 0, 0);
        Vector3d<Body> accel_b = Vector3d<Body>(0, 0, 0) + currentState->quaternion_bn() * accel_n;
        Vector3d<Platform> accel_p = Vector3d<Platform>(0, 0, 0) + quat_pb * accel_b;

        auto obs = std::make_shared<ImuObs>(imuFile->imuPosition());
        obs->timeSinceStartup = static_cast<uint64_t>(dt * i * 10e9L);
        obs->insTime = InsTime(2, 72, 211996.0L + dt * i);
        obs->accelUncompXYZ = accel_p + gravity_p;
        obs->gyroUncompXYZ = Vector3d<Platform>(0, 0, 0) + angularVelocity_ie_p;
        obs->magUncompXYZ = Vector3d<Platform>(0, 0, 0);

        imuIntegrator->handleInputData(0, obs);
        currentState = std::static_pointer_cast<StateData>(state->requestOutputData(0));
    }

    // REQUIRE(gnuPlotLatLon->update() == true);
    // REQUIRE(gnuPlotLat->update() == true);
    // REQUIRE(gnuPlotLon->update() == true);
    // REQUIRE(gnuPlotHeight->update() == true);
    // REQUIRE(gnuPlotVelocity->update() == true);
    // REQUIRE(gnuPlotAngles->update() == true);

    double distanceTotal = measureDistance(currentState->latitude(), currentState->longitude(),
                                           trafo::deg2rad(initLatLonHeight(0)), trafo::deg2rad(initLatLonHeight(1)));
    double distanceEast = measureDistance(currentState->latitude(), currentState->longitude(),
                                          currentState->latitude(), trafo::deg2rad(initLatLonHeight(1)));
    double distanceNorth = measureDistance(currentState->latitude(), currentState->longitude(),
                                           trafo::deg2rad(initLatLonHeight(0)), currentState->longitude());
    double distanceHeight = initLatLonHeight(2) - currentState->altitude();

    CHECK(distanceTotal == Approx(0).margin(EPSILON));
    CHECK(distanceNorth == Approx(0).margin(EPSILON));
    CHECK(distanceEast == Approx(0).margin(EPSILON));
    CHECK(distanceHeight == Approx(0).margin(1e-10));

    // LOG_INFO("Distance Total : {} [m]", distanceTotal);
    // LOG_INFO("Distance North : {} [m]", distanceNorth);
    // LOG_INFO("Distance East  : {} [m]", distanceEast);
    // LOG_INFO("Distance Height: {} [m]", distanceHeight);

    // CHECK(true == false);

    // Sleep::waitForSignal();
}

} // namespace NAV