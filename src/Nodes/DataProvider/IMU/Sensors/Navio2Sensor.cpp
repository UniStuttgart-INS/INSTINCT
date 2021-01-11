#include "Navio2Sensor.hpp"

#include "util/Debug.hpp"
#include "util/Logger.hpp"

#if !__APPLE__
    #include "navio/Common/MPU9250.h"
    #include "navio/Navio2/LSM9DS1.h"
    #include "navio/Common/Util.h"
#endif

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

NAV::Navio2Sensor::Navio2Sensor()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, "Navio2Sensor", this);

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
        deinitialize();
    }

    if (ImGui::SliderInt("Frequency", &outputFrequency, 1, 200, "%d Hz"))
    {
        LOG_DEBUG("{}: Frequency changed to {}", nameId(), outputFrequency);
        flow::ApplyChanges();
        deinitialize();
    }
}

[[nodiscard]] json NAV::Navio2Sensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["Frequency"] = outputFrequency;

    return j;
}

void NAV::Navio2Sensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("Frequency"))
    {
        j.at("Frequency").get_to(outputFrequency);
    }
}

bool NAV::Navio2Sensor::initialize()
{
    deinitialize();

    LOG_TRACE("{} ({}): called", nameId(), imuType ? "LSM9DS1" : "MPU9250");

    if (!Node::initialize())
    {
        return false;
    }

#if !__APPLE__
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
    startTime = std::chrono::high_resolution_clock::now();
    timer.start(outputInterval, readImuThread, this);

    return isInitialized = true;
}

void NAV::Navio2Sensor::deinitialize()
{
    LOG_TRACE("{} ({}): called", nameId(), imuType ? "LSM9DS1" : "MPU9250");

    if (timer.is_running())
    {
        timer.stop();
    }

#if !__APPLE__
    sensor.reset();
#endif

    Node::deinitialize();
}

// void NAV::Navio2Sensor::readImuThread()
void NAV::Navio2Sensor::readImuThread(void* userData)
{
    auto* navio = static_cast<Navio2Sensor*>(userData);
    auto obs = std::make_shared<ImuObs>(navio->imuPos);

    auto currentTime = std::chrono::high_resolution_clock::now();
#if !__APPLE__
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

    navio->invokeCallbacks(OutputPortIndex_ImuObs, obs);
}