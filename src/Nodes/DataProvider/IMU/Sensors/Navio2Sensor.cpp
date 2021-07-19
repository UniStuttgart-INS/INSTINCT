#include "Navio2Sensor.hpp"

#include "util/Debug.hpp"
#include "util/Logger.hpp"

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    #include "navio/Common/MPU9250.h"
    #include "navio/Navio2/LSM9DS1.h"
    #include "navio/Common/Util.h"
#endif

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Time/TimeBase.hpp"

NAV::Navio2Sensor::Navio2Sensor()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 295, 92 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, NAV::ImuObs::type());
}

NAV::Navio2Sensor::~Navio2Sensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Navio2Sensor::typeStatic()
{
    return "Navio2Sensor";
}

std::string NAV::Navio2Sensor::type() const
{
    return typeStatic();
}

std::string NAV::Navio2Sensor::category()
{
    return "Data Provider";
}

void NAV::Navio2Sensor::guiConfig()
{
    if (ImGui::Combo("IMU", reinterpret_cast<int*>(&imuType), "MPU9250\0LSM9DS1\0\0"))
    {
        LOG_DEBUG("{}: IMU changed to {}", nameId(), imuType ? "LSM9DS1" : "MPU9250");
        flow::ApplyChanges();
        deinitializeNode();
    }

    if (ImGui::SliderInt("Frequency", &outputFrequency, 1, 200, "%d Hz"))
    {
        LOG_DEBUG("{}: Frequency changed to {}", nameId(), outputFrequency);
        flow::ApplyChanges();
        deinitializeNode();
    }

    Imu::guiConfig();
}

[[nodiscard]] json NAV::Navio2Sensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["Frequency"] = outputFrequency;
    j["Imu"] = Imu::save();

    return j;
}

void NAV::Navio2Sensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("Frequency"))
    {
        j.at("Frequency").get_to(outputFrequency);
    }
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::Navio2Sensor::resetNode()
{
    return true;
}

bool NAV::Navio2Sensor::initialize()
{
    LOG_TRACE("{} ({}): called", nameId(), imuType ? "LSM9DS1" : "MPU9250");

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
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
        LOG_ERROR("{} ({}): Sensor not enabled", nameId(), imuType ? "LSM9DS1" : "MPU9250");
        return false;
    }
    sensor->initialize();
#else
    LOG_ERROR("{} ({}): MacOS is not supported by the Navio2 Node", nameId(), imuType ? "LSM9DS1" : "MPU9250");
    return false;
#endif

    int outputInterval = static_cast<int>(1.0 / static_cast<double>(outputFrequency) * 1000.0);
    startTime = std::chrono::steady_clock::now();
    timer.start(outputInterval, readImuThread, this);

    return true;
}

void NAV::Navio2Sensor::deinitialize()
{
    LOG_TRACE("{} ({}): called", nameId(), imuType ? "LSM9DS1" : "MPU9250");

    if (timer.is_running())
    {
        timer.stop();
    }

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    sensor.reset();
#endif
}

// void NAV::Navio2Sensor::readImuThread()
void NAV::Navio2Sensor::readImuThread(void* userData)
{
    auto* navio = static_cast<Navio2Sensor*>(userData);
    auto obs = std::make_shared<ImuObs>(navio->imuPos);

    auto currentTime = std::chrono::steady_clock::now();
#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    navio->sensor->update();

    navio->sensor->read_accelerometer(&navio->ax, &navio->ay, &navio->az);
    navio->sensor->read_gyroscope(&navio->gx, &navio->gy, &navio->gz);
    navio->sensor->read_magnetometer(&navio->mx, &navio->my, &navio->mz);

    obs->temperature = navio->sensor->read_temperature();
#endif

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

    if (InsTime currentTime = util::time::GetCurrentInsTime();
        !currentTime.empty())
    {
        obs->insTime = currentTime;
    }
    navio->invokeCallbacks(OutputPortIndex_ImuObs, obs);
}