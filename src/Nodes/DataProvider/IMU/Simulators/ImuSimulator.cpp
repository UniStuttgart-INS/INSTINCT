#include "ImuSimulator.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

NAV::ImuSimulator::ImuSimulator(const std::string& name, const std::map<std::string, std::string>& options)
    : Imu(name, options)
{
    if (options.count("Duration"))
    {
        duration = std::stod(options.at("Duration"));
    }
    if (options.count("Frequency"))
    {
        frequency = std::stod(options.at("Frequency"));
    }
}

std::shared_ptr<NAV::ImuObs> NAV::ImuSimulator::pollData(bool peek)
{
    if (currentSimTime > duration)
    {
        return nullptr;
    }

    // Get the current state data
    const auto& stateNode = incomingLinks[0].first.lock();
    auto& statePortIndex = incomingLinks[0].second;
    /// State Data at the time tₖ₋₁
    auto stateData = std::static_pointer_cast<StateData>(stateNode->requestOutputData(statePortIndex));

    auto obs = std::make_shared<ImuObs>();
    obs->timeSinceStartup = currentSimTime;
    currentSimTime += 1.0 / frequency;

    auto accel_n = Eigen::Vector3d(0, 0, 0);
    auto accel_b = Eigen::Vector3d(0, 0, 0);
    auto accel_p = Eigen::Vector3d(0, 0, 0);

    obs->accelUncompXYZ = accel_p + imuPos->quatAccel_pb() * (accel_b + stateData->quaternion_bn() * accel_n);
    obs->gyroUncompXYZ = Eigen::Vector3d(0, 0, 0);
    obs->magUncompXYZ = Eigen::Vector3d(0, 0, 0);
    obs->temperature = 0.0;

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(obs);
    }

    return obs;
}