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
    if (options.count("Accel n"))
    {
        std::stringstream lineStream(options.at("Accel n"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        accel_n = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Accel b"))
    {
        std::stringstream lineStream(options.at("Accel b"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        accel_b = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Accel p"))
    {
        std::stringstream lineStream(options.at("Accel p"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        accel_p = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Gyro n"))
    {
        std::stringstream lineStream(options.at("Gyro n"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        gyro_n = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Gyro b"))
    {
        std::stringstream lineStream(options.at("Gyro b"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        gyro_b = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Gyro p"))
    {
        std::stringstream lineStream(options.at("Gyro p"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        gyro_p = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Mag n"))
    {
        std::stringstream lineStream(options.at("Mag n"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        mag_n = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Mag b"))
    {
        std::stringstream lineStream(options.at("Mag b"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        mag_b = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Mag p"))
    {
        std::stringstream lineStream(options.at("Mag p"));
        std::string value;
        double X{};
        double Y{};
        double Z{};
        if (std::getline(lineStream, value, ';'))
        {
            X = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                Y = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    Z = std::stod(value);
                }
            }
        }
        mag_p = Eigen::Vector3d(X, Y, Z);
    }
    if (options.count("Temperature"))
    {
        temperature = std::stod(options.at("Temperature"));
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

    Eigen::Quaterniond quat_bn = Eigen::Quaterniond::Identity();
    if (stateData)
    {
        quat_bn = stateData->quaternion_bn();
    }

    auto obs = std::make_shared<ImuObs>();
    obs->timeSinceStartup = currentSimTime;
    currentSimTime += 1.0 / frequency;

    obs->accelUncompXYZ = accel_p + imuPos->quatAccel_pb() * (accel_b + quat_bn * accel_n);
    obs->gyroUncompXYZ = gyro_p + imuPos->quatGyro_pb() * (gyro_b + quat_bn * gyro_n);
    obs->magUncompXYZ = mag_p + imuPos->quatMag_pb() * (mag_b + quat_bn * mag_n);
    obs->temperature = temperature;

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(obs);
    }

    return obs;
}