#ifndef DISABLE_SENSORS

    #include "Navio2Sensor.hpp"

    #include "util/Debug.hpp"
    #include "util/Logger.hpp"
    #include "util/Constants.hpp"

    #include "navio/Common/MPU9250.h"
    #include "navio/Navio2/LSM9DS1.h"
    #include "navio/Common/Util.h"

    #include <chrono>

NAV::Navio2Sensor::Navio2Sensor(const std::string& name, const std::map<std::string, std::string>& options)
    : Imu(name, options)
{
    LOG_TRACE("called for {}", name);

    if (options.count("Frequency"))
    {
        outputFrequency = static_cast<uint16_t>(std::stoul(options.at("Frequency")));
    }
    if (options.count("Imu"))
    {
        if (options.at("Imu") == "MPU9250")
        {
            imuType = ImuType::MPU;
        }
        else if (options.at("Imu") == "LSM9DS1")
        {
            imuType = ImuType::LSM;
        }
        else
        {
            LOG_CRITICAL("{}: Unknown IMU Type {}", name, options.at("Imu"));
        }
    }

    if (imuType == ImuType::MPU)
    {
        sensor = std::make_unique<MPU9250>();
    }
    else // ImuType::LSM
    {
        sensor = std::make_unique<LSM9DS1>();
    }

    if (!sensor->probe())
    {
        LOG_CRITICAL("{} ({}): Sensor not enabled", name, options.at("Imu"));
    }
    sensor->initialize();

    int outputInterval = static_cast<int>(1.0 / static_cast<double>(outputFrequency) * 1000.0);
    startTime = std::chrono::high_resolution_clock::now();
    timer.start(outputInterval, readImuThread, this);

    LOG_DEBUG("{} successfully initialized {}", name, outputInterval);
}

NAV::Navio2Sensor::~Navio2Sensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<ImuObs>();
    callbacksEnabled = false;
    if (timer.is_running())
    {
        timer.stop();
    }
}

// void NAV::Navio2Sensor::readImuThread()
void NAV::Navio2Sensor::readImuThread(void* userData)
{
    auto* navio = static_cast<Navio2Sensor*>(userData);
    auto obs = std::make_shared<ImuObs>();

    auto currentTime = std::chrono::high_resolution_clock::now();
    navio->sensor->update();

    navio->sensor->read_accelerometer(&navio->ax, &navio->ay, &navio->az);
    navio->sensor->read_gyroscope(&navio->gx, &navio->gy, &navio->gz);
    navio->sensor->read_magnetometer(&navio->mx, &navio->my, &navio->mz);

    obs->temperature = navio->sensor->read_temperature();

    obs->accelUncompXYZ.emplace(navio->ax, navio->ay, navio->az);
    obs->gyroUncompXYZ.emplace(navio->gx, navio->gy, navio->gz);

    if (navio->imuType == ImuType::LSM)
    {
        obs->magUncompXYZ.emplace(navio->mx, navio->my, navio->mz);
        // constexpr double uT2Gauss = 1.0 / 100.0;
        // obs->magUncompXYZ.value() *= uT2Gauss;
    }

    std::chrono::nanoseconds diff = currentTime - navio->startTime;
    obs->timeSinceStartup = diff.count();

    LOG_DATA("DATA({}): {}, {}°C, a=({}, {}, {})", navio->name, obs->timeSinceStartup.value(), obs->temperature.value(),
             navio->ax, navio->ay, navio->az);

    navio->invokeCallbacks(obs);
}

#endif