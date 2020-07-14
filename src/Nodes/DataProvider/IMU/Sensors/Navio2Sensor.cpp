#ifndef DISABLE_SENSORS

    #include "Navio2Sensor.hpp"

    #include "util/Debug.hpp"
    #include "util/Logger.hpp"

    #include "navio/Common/MPU9250.h"
    #include "navio/Navio2/LSM9DS1.h"
    #include "navio/Common/Util.h"

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

    float ax{};
    float ay{};
    float az{};
    float gx{};
    float gy{};
    float gz{};
    float mx{};
    float my{};
    float mz{};
    sensor->update();
    sensor->read_accelerometer(&ax, &ay, &az);
    sensor->read_gyroscope(&gx, &gy, &gz);
    sensor->read_magnetometer(&mx, &my, &mz);
    LOG_INFO("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
    LOG_INFO("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
    LOG_INFO("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);

    LOG_DEBUG("{} successfully initialized", name);
}

NAV::Navio2Sensor::~Navio2Sensor()
{
    LOG_TRACE("called for {}", name);

    removeAllCallbacksOfType<ImuObs>();
    callbacksEnabled = false;
}

#endif