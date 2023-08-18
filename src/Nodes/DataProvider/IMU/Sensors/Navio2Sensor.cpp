// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Navio2Sensor.hpp"

#include "util/Logger.hpp"

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    #include "Navio/Common/MPU9250.h"
    #include "Navio/Navio2/LSM9DS1.h"
    #include "Navio/Common/Util.h"
#endif

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Time/TimeBase.hpp"

NAV::Navio2Sensor::Navio2Sensor()
    : Imu(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = true;
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 295, 92 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
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
    if (ImGui::Combo("IMU", reinterpret_cast<int*>(&_imuType), "MPU9250\0LSM9DS1\0\0"))
    {
        LOG_DEBUG("{}: IMU changed to {}", nameId(), _imuType ? "LSM9DS1" : "MPU9250");
        flow::ApplyChanges();
        doDeinitialize();
    }

    if (ImGui::SliderInt("Frequency", &_outputFrequency, 1, 200, "%d Hz"))
    {
        LOG_DEBUG("{}: Frequency changed to {}", nameId(), _outputFrequency);
        flow::ApplyChanges();
        doDeinitialize();
    }

    Imu::guiConfig();
}

[[nodiscard]] json NAV::Navio2Sensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["Frequency"] = _outputFrequency;
    j["Imu"] = Imu::save();

    return j;
}

void NAV::Navio2Sensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("Frequency"))
    {
        j.at("Frequency").get_to(_outputFrequency);
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
    LOG_TRACE("{} ({}): called", nameId(), _imuType ? "LSM9DS1" : "MPU9250");

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    if (_imuType == ImuType::MPU)
    {
        _sensor = std::make_unique<MPU9250>();
    }
    else // ImuType::LSM
    {
        _sensor = std::make_unique<LSM9DS1>();
    }

    if (!_sensor->probe())
    {
        LOG_ERROR("{} ({}): Sensor not enabled", nameId(), _imuType ? "LSM9DS1" : "MPU9250");
        return false;
    }
    _sensor->initialize();
#else
    LOG_ERROR("{} ({}): MacOS is not supported by the Navio2 Node", nameId(), _imuType ? "LSM9DS1" : "MPU9250");
    return false;
#endif

    int outputInterval = static_cast<int>(1.0 / static_cast<double>(_outputFrequency) * 1000.0);
    _startTime = std::chrono::steady_clock::now();
    _timer.start(outputInterval, readImuThread, this);

    return true;
}

void NAV::Navio2Sensor::deinitialize()
{
    LOG_TRACE("{} ({}): called", nameId(), _imuType ? "LSM9DS1" : "MPU9250");

    if (_timer.is_running())
    {
        _timer.stop();
    }

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    _sensor.reset();
#endif
}

// void NAV::Navio2Sensor::readImuThread()
void NAV::Navio2Sensor::readImuThread(void* userData)
{
    auto* navio = static_cast<Navio2Sensor*>(userData);
    auto obs = std::make_shared<ImuObs>(navio->_imuPos);

    auto currentTime = std::chrono::steady_clock::now();
#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
    navio->_sensor->update();

    navio->_sensor->read_accelerometer(&navio->_ax, &navio->_ay, &navio->_az);
    navio->_sensor->read_gyroscope(&navio->_gx, &navio->_gy, &navio->_gz);
    navio->_sensor->read_magnetometer(&navio->_mx, &navio->_my, &navio->_mz);

    obs->temperature = navio->_sensor->read_temperature();
#endif

    obs->accelUncompXYZ.emplace(navio->_ax, navio->_ay, navio->_az);
    obs->gyroUncompXYZ.emplace(navio->_gx, navio->_gy, navio->_gz);

    if (navio->_imuType == ImuType::LSM)
    {
        obs->magUncompXYZ.emplace(navio->_mx, navio->_my, navio->_mz);
        // constexpr double uT2Gauss = 1.0 / 100.0;
        // obs->magUncompXYZ.value() *= uT2Gauss;
    }

    std::chrono::nanoseconds diff = currentTime - navio->_startTime;
    obs->timeSinceStartup = diff.count();

    LOG_DATA("DATA({}): {}, {}°C, a=({}, {}, {})", navio->name, obs->timeSinceStartup.value(), obs->temperature.value(),
             navio->_ax, navio->_ay, navio->_az);

    if (InsTime currentTime = util::time::GetCurrentInsTime();
        !currentTime.empty())
    {
        obs->insTime = currentTime;
    }
    navio->invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
}