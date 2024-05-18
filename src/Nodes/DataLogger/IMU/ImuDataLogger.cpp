// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuDataLogger.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/IMU/ImuObsSimulated.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "NodeRegistry.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::ImuDataLogger::ImuDataLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { NAV::ImuObs::type(), NAV::ImuObsWDelta::type(), NAV::ImuObsSimulated::type() }, &ImuDataLogger::writeObservation);
}

NAV::ImuDataLogger::~ImuDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuDataLogger::typeStatic()
{
    return "ImuDataLogger";
}

std::string NAV::ImuDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::ImuDataLogger::category()
{
    return "Data Logger";
}

void NAV::ImuDataLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }
}

[[nodiscard]] json NAV::ImuDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::ImuDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::ImuDataLogger::afterCreateLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    doDeinitialize();
}

void NAV::ImuDataLogger::flush()
{
    _filestream.flush();
}

bool NAV::ImuDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _filestream << "Time [s],GpsCycle,GpsWeek,GpsToW [s],TimeStartup [ns],"
                << "MagX [Gauss],MagY [Gauss],MagZ [Gauss],"
                << "AccX [m/s^2],AccY [m/s^2],AccZ [m/s^2],"
                << "GyroX [rad/s],GyroY [rad/s],GyroZ [rad/s],"
                << "Temperature [Celsius]";
    if (auto* sourcePin = inputPins[INPUT_PORT_INDEX_IMU_OBS].link.getConnectedPin())
    {
        if (NodeRegistry::NodeDataTypeAnyIsChildOf(sourcePin->dataIdentifier, { ImuObsWDelta::type() }))
        {
            _filestream << ","
                        << "DeltaTime [s],"
                        << "DeltaThetaX [deg],DeltaThetaY [deg],DeltaThetaZ [deg],"
                        << "DeltaVelX [m/s],DeltaVelY [m/s],DeltaVelZ [m/s]";
        }
        if (NodeRegistry::NodeDataTypeAnyIsChildOf(sourcePin->dataIdentifier, { ImuObsSimulated::type() }))
        {
            _filestream << ","
                        << "AccelDynamicsN [m/s^2],AccelDynamicsE [m/s^2],AccelDynamicsD [m/s^2],"
                        << "AngularRateN (ω_nb_n) [rad/s],AngularRateE (ω_nb_n) [rad/s],AngularRateD (ω_nb_n) [rad/s],"
                        << "AccelDynamicsX ECEF [m/s^2],AccelDynamicsY ECEF [m/s^2],AccelDynamicsZ ECEF [m/s^2],"
                        << "AngularRateX ECEF (ω_nb_e) [rad/s],AngularRateY ECEF (ω_nb_e) [rad/s],AngularRateZ ECEF (ω_nb_e) [rad/s]";
        }
    }
    _filestream << std::endl; // NOLINT(performance-avoid-endl)

    return true;
}

void NAV::ImuDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::ImuDataLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    if (!obs->insTime.empty())
    {
        _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(obs->insTime) * 1e9) / 1e9;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.toGPSweekTow().gpsCycle;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().gpsWeek;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().tow;
    }
    _filestream << ",";
    if (obs->timeSinceStartup)
    {
        _filestream << std::setprecision(valuePrecision) << obs->timeSinceStartup.value();
    }
    _filestream << ",";
    if (obs->p_magneticField)
    {
        _filestream << obs->p_magneticField.value().x();
    }
    _filestream << ",";
    if (obs->p_magneticField)
    {
        _filestream << obs->p_magneticField.value().y();
    }
    _filestream << ",";
    if (obs->p_magneticField)
    {
        _filestream << obs->p_magneticField.value().z();
    }
    _filestream << "," << obs->p_acceleration.x();
    _filestream << "," << obs->p_acceleration.y();
    _filestream << "," << obs->p_acceleration.z();
    _filestream << "," << obs->p_angularRate.x();
    _filestream << "," << obs->p_angularRate.y();
    _filestream << "," << obs->p_angularRate.z();
    _filestream << ",";
    if (obs->temperature)
    {
        _filestream << obs->temperature.value();
    }

    if (auto* sourcePin = inputPins[INPUT_PORT_INDEX_IMU_OBS].link.getConnectedPin())
    {
        if (NodeRegistry::NodeDataTypeAnyIsChildOf(sourcePin->dataIdentifier, { ImuObsWDelta::type() }))
        {
            auto simObs = std::static_pointer_cast<const ImuObsWDelta>(obs);

            _filestream << "," << simObs->dtime;
            _filestream << "," << simObs->dtheta.x() << "," << simObs->dtheta.y() << "," << simObs->dtheta.z();
            _filestream << "," << simObs->dvel.x() << "," << simObs->dvel.y() << "," << simObs->dvel.z();
        }
        if (NodeRegistry::NodeDataTypeAnyIsChildOf(sourcePin->dataIdentifier, { ImuObsSimulated::type() }))
        {
            auto simObs = std::static_pointer_cast<const ImuObsSimulated>(obs);

            _filestream << "," << simObs->n_accelDynamics.x() << "," << simObs->n_accelDynamics.y() << "," << simObs->n_accelDynamics.z();
            _filestream << "," << simObs->n_angularRateDynamics.x() << "," << simObs->n_angularRateDynamics.y() << "," << simObs->n_angularRateDynamics.z();
            _filestream << "," << simObs->e_accelDynamics.x() << "," << simObs->e_accelDynamics.y() << "," << simObs->e_accelDynamics.z();
            _filestream << "," << simObs->e_angularRateDynamics.x() << "," << simObs->e_angularRateDynamics.y() << "," << simObs->e_angularRateDynamics.z();
        }
    }

    _filestream << '\n';
}