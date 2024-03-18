// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "WiFiObsLogger.hpp"

#include "NodeData/WiFi/WiFiObs.hpp"

#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::WiFiObsLogger::WiFiObsLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { WiFiObs::type() }, &WiFiObsLogger::writeObservation);
}

NAV::WiFiObsLogger::~WiFiObsLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::WiFiObsLogger::typeStatic()
{
    return "WiFiObsLogger";
}

std::string NAV::WiFiObsLogger::type() const
{
    return typeStatic();
}

std::string NAV::WiFiObsLogger::category()
{
    return "Data Logger";
}

void NAV::WiFiObsLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }
}

[[nodiscard]] json NAV::WiFiObsLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::WiFiObsLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::WiFiObsLogger::flush()
{
    _filestream.flush();
}

bool NAV::WiFiObsLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _filestream << "Time [s],GpsCycle,GpsWeek,GpsToW [s],"
                << "MacAddress,"
                << "Distance [m]" << std::endl;

    return true;
}

void NAV::WiFiObsLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::WiFiObsLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    if (auto* sourcePin = inputPins[pinIdx].link.getConnectedPin())
    {
        constexpr int gpsCyclePrecision = 3;
        constexpr int gpsTimePrecision = 12;
        constexpr int valuePrecision = 15;

        auto nodeData = queue.extract_front();

        // -------------------------------------------------------- Time -----------------------------------------------------------
        auto wifiObs = std::static_pointer_cast<const WiFiObs>(nodeData);
        for (auto const& obs : wifiObs->data)
        {
            if (!obs.time.empty())
            {
                _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(obs.time) * 1e9) / 1e9;
            }
            _filestream << ",";
            if (!obs.time.empty())
            {
                _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs.time.toGPSweekTow().gpsCycle;
            }
            _filestream << ",";
            if (!obs.time.empty())
            {
                _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs.time.toGPSweekTow().gpsWeek;
            }
            _filestream << ",";
            if (!obs.time.empty())
            {
                _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs.time.toGPSweekTow().tow;
            }
            _filestream << "," << std::setprecision(valuePrecision);

            // {
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.year), sizeof(obs->timeOutputs->timeUtc.year));
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.month), sizeof(obs->timeOutputs->timeUtc.month));
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.day), sizeof(obs->timeOutputs->timeUtc.day));
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.hour), sizeof(obs->timeOutputs->timeUtc.hour));
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.min), sizeof(obs->timeOutputs->timeUtc.min));
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.sec), sizeof(obs->timeOutputs->timeUtc.sec));
            //     _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.ms), sizeof(obs->timeOutputs->timeUtc.ms));
            // }

            // ------------------------------------------------------ MacAddress ----------------------------------------------------------
            if (!obs.macAddress.empty())
            {
                _filestream << obs.macAddress;
            }
            else
            {
                _filestream << ",";
            }
            _filestream << ",";

            // ------------------------------------------------------- Distance -----------------------------------------------------------
            if (!(obs.distance < 0.0))
            {
                _filestream << obs.distance;
            }
            else
            {
                _filestream << ",";
            }
        }
        _filestream << std::endl;
    }
}
