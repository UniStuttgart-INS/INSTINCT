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

namespace NAV
{
TEST_CASE("[ImuIntegrator] Integrate Observation NED", "[ImuIntegrator]")
{
    NodeManager::appContext = Node::NodeContext::POST_PROCESSING;

    Eigen::Vector3d mountingAngles(0.0, 0.0, 0.0);
    std::map<std::string, std::string> optionsImuFile = { { "Path", "../../../test/data/vectornav.csv" },
                                                          { "Accel pos", "0;0;0" },
                                                          { "Accel rot", fmt::format("{:2.2f};{:2.2f};{:2.2f}", mountingAngles(0), mountingAngles(1), mountingAngles(2)) },
                                                          { "Gyro pos", "0;0;0" },
                                                          { "Gyro rot", fmt::format("{:2.2f};{:2.2f};{:2.2f}", mountingAngles(0), mountingAngles(1), mountingAngles(2)) },
                                                          { "Mag pos", "0;0;0" },
                                                          { "Mag rot", fmt::format("{:2.2f};{:2.2f};{:2.2f}", mountingAngles(0), mountingAngles(1), mountingAngles(2)) } };
    auto imuFile = std::make_shared<VectorNavFile>("ImuFile", optionsImuFile);
    std::static_pointer_cast<Node>(imuFile)->initialize();

    std::map<std::string, std::string> optionsIntegrator = { { "Integration Frame", "NED" } };
    auto imuIntegrator = std::make_shared<ImuIntegrator>("ImuIntegrator", optionsIntegrator);
    std::static_pointer_cast<Node>(imuIntegrator)->initialize();

    Eigen::Vector3d initLatLonHeight(48.05202354, 9.361408376, 604.2897);
    Eigen::Vector3d initRollPitchYaw(0.0, 0.0, 0.0);
    Eigen::Vector3d initVelocityNED(0.0, 0.0, 0.0);

    std::map<std::string, std::string> optionsState = { { "Init LatLonAlt", fmt::format("{:2.8f};{:1.9f};{:3.4f}", initLatLonHeight(0), initLatLonHeight(1), initLatLonHeight(2)) },
                                                        { "Init RollPitchYaw", fmt::format("{:2.2f};{:2.2f};{:2.2f}", initRollPitchYaw(0), initRollPitchYaw(1), initRollPitchYaw(2)) },
                                                        { "Init Velocity", fmt::format("{:2.2f};{:2.2f};{:2.2f}", initVelocityNED(0), initVelocityNED(1), initVelocityNED(2)) } };
    auto state = std::make_shared<State>("State", optionsState);
    std::static_pointer_cast<Node>(state)->initialize();

    std::map<std::string, std::string> optionsGnuPlotLatLon = { { "Start", "set autoscale xy\n"
                                                                           "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                           "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                           "set xlabel \"Longitude [deg]\"\n"
                                                                           "set ylabel \"Latitude [deg]\"\n" },
                                                                { "Input Ports", "1" },
                                                                { "1-Port Type", "StateData" },
                                                                { "1-Data to plot", "Longitude;Latitude" },
                                                                { "1-Update", "plot [~1,2~] using 1:2 with lines title 'StateData'" } };
    auto gnuPlotLatLon = std::make_shared<GnuPlot>("GnuPlot LatLon", optionsGnuPlotLatLon);
    std::static_pointer_cast<Node>(gnuPlotLatLon)->initialize();

    std::map<std::string, std::string> optionsGnuPlotHeight = { { "Start", "set autoscale xy\n"
                                                                           "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                           "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                           "set xlabel \"Time [s]\"\n"
                                                                           "set ylabel \"Height [m]\"\n" },
                                                                { "Input Ports", "1" },
                                                                { "1-Port Type", "StateData" },
                                                                { "1-Data to plot", "Time;Height" },
                                                                { "1-Update", "plot [~1,2~] using 1:2 with lines title 'StateData'" } };
    auto gnuPlotHeight = std::make_shared<GnuPlot>("GnuPlot Height", optionsGnuPlotHeight);
    std::static_pointer_cast<Node>(gnuPlotHeight)->initialize();

    std::map<std::string, std::string> optionsGnuPlotVelocity = { { "Start", "set autoscale xy\n"
                                                                             "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                             "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                             "set xlabel \"Time [s]\"\n"
                                                                             "set ylabel \"Velocity NED [m/s]\"\n" },
                                                                  { "Input Ports", "1" },
                                                                  { "1-Port Type", "StateData" },
                                                                  { "1-Data to plot", "Time;Velocity North;Velocity East;Velocity Down" },
                                                                  { "1-Update", "plot [~1,2,3,4~] using 1:2 with lines title 'North'\n"
                                                                                "plot [~1,2,3,4~] using 1:3 with lines title 'East'\n"
                                                                                "plot [~1,2,3,4~] using 1:4 with lines title 'Down'" } };
    auto gnuPlotVelocity = std::make_shared<GnuPlot>("GnuPlot Velocity", optionsGnuPlotVelocity);
    std::static_pointer_cast<Node>(gnuPlotVelocity)->initialize();

    std::map<std::string, std::string> optionsGnuPlotAngles = { { "Start", "set autoscale xy\n"
                                                                           "set grid ytics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                           "set grid xtics lc rgb \"[hash]bbbbbb\" lw 1 lt 0\n"
                                                                           "set xlabel \"Time [s]\"\n"
                                                                           "set ylabel \"Flight Angles [deg]\"\n" },
                                                                { "Input Ports", "1" },
                                                                { "1-Port Type", "StateData" },
                                                                { "1-Data to plot", "Time;Roll;Pitch;Yaw" },
                                                                { "1-Update", "plot [~1,2,3,4~] using 1:2 with lines title 'Roll'\n"
                                                                              "plot [~1,2,3,4~] using 1:3 with lines title 'Pitch'\n"
                                                                              "plot [~1,2,3,4~] using 1:4 with lines title 'Yaw'" } };
    auto gnuPlotAngles = std::make_shared<GnuPlot>("GnuPlot Angles", optionsGnuPlotAngles);
    std::static_pointer_cast<Node>(gnuPlotAngles)->initialize();

    // Configure callbacks
    auto imuIntegratorAsNode = std::static_pointer_cast<Node>(imuIntegrator);
    auto stateAsNode = std::static_pointer_cast<Node>(state);
    auto gnuPlotLatLonAsNode = std::static_pointer_cast<Node>(gnuPlotLatLon);
    auto gnuPlotHeightAsNode = std::static_pointer_cast<Node>(gnuPlotHeight);
    auto gnuPlotVelocityAsNode = std::static_pointer_cast<Node>(gnuPlotVelocity);
    auto gnuPlotAnglesAsNode = std::static_pointer_cast<Node>(gnuPlotAngles);

    imuFile->addCallback<VectorNavObs>(imuIntegratorAsNode, 0);
    imuIntegrator->incomingLinks.emplace(0, std::make_pair(imuFile, 0));

    imuFile->addCallback<ImuPos>(imuIntegratorAsNode, 1);
    imuIntegrator->incomingLinks.emplace(1, std::make_pair(imuFile, 1));

    state->addCallback<StateData>(imuIntegratorAsNode, 2);
    imuIntegrator->incomingLinks.emplace(2, std::make_pair(state, 0));

    imuIntegrator->addCallback<StateData>(stateAsNode, 0);
    state->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));

    imuIntegrator->addCallback<StateData>(gnuPlotLatLonAsNode, 0);
    gnuPlotLatLon->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    imuIntegrator->addCallback<StateData>(gnuPlotHeightAsNode, 0);
    gnuPlotHeight->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    imuIntegrator->addCallback<StateData>(gnuPlotVelocityAsNode, 0);
    gnuPlotVelocity->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));
    imuIntegrator->addCallback<StateData>(gnuPlotAnglesAsNode, 0);
    gnuPlotAngles->incomingLinks.emplace(0, std::make_pair(imuIntegrator, 0));

    imuIntegrator->callbacksEnabled = true;
    state->callbacksEnabled = true;

    long double dt = 0.1L;
    auto currentState = std::static_pointer_cast<StateData>(state->requestOutputData(0));
    auto quat_b2p = trafo::quat_p2b(mountingAngles.x(), mountingAngles.y(), mountingAngles.z()).conjugate();

    for (size_t i = 0; i < 11; i++)
    {
        double gravityNorm = gravity::gravityMagnitude_Gleason(currentState->latitude());

        auto gravity_n = Eigen::Vector3d(0, 0, -gravityNorm);
        auto gravity_p = quat_b2p * currentState->quaternion_n2b() * gravity_n;

        auto accel_b = Eigen::Vector3d(0, 0, 0);
        auto accel_p = Eigen::Vector3d(0, 0, 0) + quat_b2p * accel_b;

        auto obs = std::make_shared<ImuObs>();
        obs->timeSinceStartup = static_cast<uint64_t>(dt * i * 10e9L);
        obs->insTime = InsTime(2, 72, 211996.0L + dt * i);
        obs->accelUncompXYZ = accel_p + gravity_p;
        obs->gyroUncompXYZ = Eigen::Vector3d(0, 0, 0);
        obs->magUncompXYZ = Eigen::Vector3d(0, 0, 0);
        obs->temperature = 24;

        imuIntegrator->handleInputData(0, obs);
        currentState = std::static_pointer_cast<StateData>(state->requestOutputData(0));
    }

    REQUIRE(gnuPlotLatLon->update() == true);
    REQUIRE(gnuPlotHeight->update() == true);
    REQUIRE(gnuPlotVelocity->update() == true);
    REQUIRE(gnuPlotAngles->update() == true);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

} // namespace NAV