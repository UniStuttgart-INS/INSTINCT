// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "WiFiPositioningSolutionLogger.hpp"

#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/WiFi/WiFiPositioningSolution.hpp"

NAV::WiFiPositioningSolutionLogger::WiFiPositioningSolutionLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { WiFiPositioningSolution::type() }, &WiFiPositioningSolutionLogger::writeObservation);
}

NAV::WiFiPositioningSolutionLogger::~WiFiPositioningSolutionLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::WiFiPositioningSolutionLogger::typeStatic()
{
    return "WiFiPositioningSolutionLogger";
}

std::string NAV::WiFiPositioningSolutionLogger::type() const
{
    return typeStatic();
}

std::string NAV::WiFiPositioningSolutionLogger::category()
{
    return "Data Logger";
}

void NAV::WiFiPositioningSolutionLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }

    if (ImGui::Checkbox(fmt::format("Log NaN values##{}", nameId()).c_str(), &_logNanValues))
    {
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::WiFiPositioningSolutionLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();
    j["logNanValues"] = _logNanValues;

    return j;
}

void NAV::WiFiPositioningSolutionLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
    if (j.contains("logNanValues"))
    {
        _logNanValues = j.at("logNanValues");
    }
}

void NAV::WiFiPositioningSolutionLogger::flush()
{
    _filestream.flush();
}

bool NAV::WiFiPositioningSolutionLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _filestream << "Time [s],GpsCycle,GpsWeek,GpsTow [s],"
                << "Pos ECEF X [m],Pos ECEF Y [m],Pos ECEF Z [m],Latitude [deg],Longitude [deg],Altitude [m],"
                << "North/South [m],East/West [m],"
                << "Bias [m],Bias StDev [m],"
                << "Vel ECEF X [m/s],Vel ECEF Y [m/s],Vel ECEF Z [m/s],Vel N [m/s],Vel E [m/s],Vel D [m/s],"
                << "X-ECEF StDev [m],Y-ECEF StDev [m],Z-ECEF StDev [m],"
                // << "XY-ECEF StDev [m],XZ-ECEF StDev [m],YZ-ECEF StDev [m],"
                << "North StDev [m],East StDev [m],Down StDev [m],"
                // << "NE-ECEF StDev [m],ND-ECEF StDev [m],ED-ECEF StDev [m],"
                << "X velocity ECEF StDev [m/s],Y velocity ECEF StDev [m/s],Z velocity ECEF StDev [m/s],"
                // << "XY velocity StDev [m],XZ velocity StDev [m],YZ velocity StDev [m],"
                << "North velocity StDev [m/s],East velocity StDev [m/s],Down velocity StDev [m/s],"
                // << "NE velocity StDev [m],ND velocity StDev [m],ED velocity StDev [m],"
                << std::endl;

    return true;
}

void NAV::WiFiPositioningSolutionLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::WiFiPositioningSolutionLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const WiFiPositioningSolution>(queue.extract_front());

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 12;

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
    _filestream << "," << std::setprecision(valuePrecision);

    if (_logNanValues)
    {
        // -------------------------------------------------------- Position -----------------------------------------------------------

        _filestream << obs->e_position().x(); // Pos ECEF X [m]
        _filestream << ",";
        _filestream << obs->e_position().y(); // Pos ECEF Y [m]
        _filestream << ",";
        _filestream << obs->e_position().z(); // Pos ECEF Z [m]
        _filestream << ",";
        _filestream << rad2deg(obs->lla_position().x()); // Latitude [deg]
        _filestream << ",";
        _filestream << rad2deg(obs->lla_position().y()); // Longitude [deg]
        _filestream << ",";
        _filestream << obs->lla_position().z(); // Altitude [m]
        _filestream << ",";
        auto localPosition = calcLocalPosition(obs->lla_position());
        _filestream << localPosition.northSouth << ","; // North/South [m]
        _filestream << localPosition.eastWest << ",";   // East/West [m]

        // -------------------------------------------------------- Bias -----------------------------------------------------------
        _filestream << obs->bias(); // Bias [m]
        _filestream << ",";
        _filestream << obs->biasStdev(); // Bias StDev [m]
        _filestream << ",";
        // -------------------------------------------------------- Velocity -----------------------------------------------------------

        _filestream << obs->e_velocity().x(); // Vel ECEF X [m/s]
        _filestream << ",";
        _filestream << obs->e_velocity().y(); // Vel ECEF Y [m/s]
        _filestream << ",";
        _filestream << obs->e_velocity().z(); // Vel ECEF Z [m/s]
        _filestream << ",";
        _filestream << obs->n_velocity().x(); // Vel N [m/s]
        _filestream << ",";
        _filestream << obs->n_velocity().y(); // Vel E [m/s]
        _filestream << ",";
        _filestream << obs->n_velocity().z(); // Vel D [m/s]
        _filestream << ",";
        // -------------------------------------------------------- Standard Deviation -----------------------------------------------------------
        _filestream << obs->e_positionStdev()(0); // X-ECEF StDev [m]
        _filestream << ",";
        _filestream << obs->e_positionStdev()(1); // Y-ECEF StDev [m]
        _filestream << ",";
        _filestream << obs->e_positionStdev()(2); // Z-ECEF StDev [m]
        _filestream << ",";
        // _filestream << obs->e_positionStdev()(0, 1); // XY-ECEF StDev [m]
        // _filestream << ",";
        // _filestream << obs->e_positionStdev()(0, 2); // XZ-ECEF StDev [m]
        // _filestream << ",";
        // _filestream << obs->e_positionStdev()(1, 2); // YZ-ECEF StDev [m]
        // _filestream << ",";
        _filestream << obs->n_positionStdev()(0); // North StDev [m]
        _filestream << ",";
        _filestream << obs->n_positionStdev()(1); // East StDev [m]
        _filestream << ",";
        _filestream << obs->n_positionStdev()(2); // Down StDev [m]
        _filestream << ",";
        // _filestream << obs->n_positionStdev()(0, 1); // NE-ECEF StDev [m]
        // _filestream << ",";
        // _filestream << obs->n_positionStdev()(0, 2); // ND-ECEF StDev [m]
        // _filestream << ",";
        // _filestream << obs->n_positionStdev()(1, 2); // ED-ECEF StDev [m]
        // _filestream << ",";
        _filestream << obs->e_velocityStdev()(0); // X velocity ECEF StDev [m/s]
        _filestream << ",";
        _filestream << obs->e_velocityStdev()(1); // Y velocity ECEF StDev [m/s]
        _filestream << ",";
        _filestream << obs->e_velocityStdev()(2); // Z velocity ECEF StDev [m/s]
        _filestream << ",";
        // _filestream << obs->e_velocityStdev()(0, 1); // XY velocity StDev [m]
        // _filestream << ",";
        // _filestream << obs->e_velocityStdev()(0, 2); // XZ velocity StDev [m]
        // _filestream << ",";
        // _filestream << obs->e_velocityStdev()(1, 2); // YZ velocity StDev [m]
        // _filestream << ",";
        _filestream << obs->n_velocityStdev()(0); // North velocity StDev [m/s]
        _filestream << ",";
        _filestream << obs->n_velocityStdev()(1); // East velocity StDev [m/s]
        _filestream << ",";
        _filestream << obs->n_velocityStdev()(2); // Down velocity StDev [m/s]
        _filestream << ",";
        // _filestream << obs->n_velocityStdev()(0, 1); // NE velocity StDev [m]
        // _filestream << ",";
        // _filestream << obs->n_velocityStdev()(0, 2); // ND velocity StDev [m]
        // _filestream << ",";
        // _filestream << obs->n_velocityStdev()(1, 2); // ED velocity StDev [m]
        // _filestream << ",";
        _filestream << '\n';
    }
    else
    {
        // -------------------------------------------------------- Position -----------------------------------------------------------

        if (!std::isnan(obs->e_position().x())) { _filestream << obs->e_position().x(); } // Pos ECEF X [m]
        _filestream << ",";
        if (!std::isnan(obs->e_position().y())) { _filestream << obs->e_position().y(); } // Pos ECEF Y [m]
        _filestream << ",";
        if (!std::isnan(obs->e_position().z())) { _filestream << obs->e_position().z(); } // Pos ECEF Z [m]
        _filestream << ",";
        if (!std::isnan(obs->lla_position().x())) { _filestream << rad2deg(obs->lla_position().x()); } // Latitude [deg]
        _filestream << ",";
        if (!std::isnan(obs->lla_position().y())) { _filestream << rad2deg(obs->lla_position().y()); } // Longitude [deg]
        _filestream << ",";
        if (!std::isnan(obs->lla_position().z())) { _filestream << obs->lla_position().z(); } // Altitude [m]
        _filestream << ",";
        if (!std::isnan(obs->lla_position().x()) && !std::isnan(obs->lla_position().y()))
        {
            auto localPosition = calcLocalPosition(obs->lla_position());
            _filestream << localPosition.northSouth << ","; // North/South [m]
            _filestream << localPosition.eastWest << ",";   // East/West [m]
        }
        else
        {
            _filestream << ",,";
        }

        // -------------------------------------------------------- Bias -----------------------------------------------------------
        if (!std::isnan(obs->bias())) { _filestream << obs->bias(); } // Bias [m]
        _filestream << ",";
        if (!std::isnan(obs->biasStdev())) { _filestream << obs->biasStdev(); } // Bias StDev [m]
        _filestream << ",";

        // -------------------------------------------------------- Velocity -----------------------------------------------------------

        if (!std::isnan(obs->e_velocity().x())) { _filestream << obs->e_velocity().x(); } // Vel ECEF X [m/s]
        _filestream << ",";
        if (!std::isnan(obs->e_velocity().y())) { _filestream << obs->e_velocity().y(); } // Vel ECEF Y [m/s]
        _filestream << ",";
        if (!std::isnan(obs->e_velocity().z())) { _filestream << obs->e_velocity().z(); } // Vel ECEF Z [m/s]
        _filestream << ",";
        if (!std::isnan(obs->n_velocity().x())) { _filestream << obs->n_velocity().x(); } // Vel N [m/s]
        _filestream << ",";
        if (!std::isnan(obs->n_velocity().y())) { _filestream << obs->n_velocity().y(); } // Vel E [m/s]
        _filestream << ",";
        if (!std::isnan(obs->n_velocity().z())) { _filestream << obs->n_velocity().z(); } // Vel D [m/s]
        _filestream << ",";

        // -------------------------------------------------------- Standard Deviation -----------------------------------------------------------
        if (!std::isnan(obs->e_positionStdev()(0))) { _filestream << obs->e_positionStdev()(0); }; // X-ECEF StDev [m]
        _filestream << ",";
        if (!std::isnan(obs->e_positionStdev()(1))) { _filestream << obs->e_positionStdev()(1); }; // Y-ECEF StDev [m]
        _filestream << ",";
        if (!std::isnan(obs->e_positionStdev()(2))) { _filestream << obs->e_positionStdev()(2); }; // Z-ECEF StDev [m]
        _filestream << ",";
        // if (!std::isnan(obs->e_positionStdev()(0, 1))) { _filestream << obs->e_positionStdev()(0, 1); }; // XY-ECEF StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->e_positionStdev()(0, 2))) { _filestream << obs->e_positionStdev()(0, 2); }; // XZ-ECEF StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->e_positionStdev()(1, 2))) { _filestream << obs->e_positionStdev()(1, 2); }; // YZ-ECEF StDev [m]
        // _filestream << ",";
        if (!std::isnan(obs->n_positionStdev()(0))) { _filestream << obs->n_positionStdev()(0); }; // North StDev [m]
        _filestream << ",";
        if (!std::isnan(obs->n_positionStdev()(1))) { _filestream << obs->n_positionStdev()(1); }; // East StDev [m]
        _filestream << ",";
        if (!std::isnan(obs->n_positionStdev()(2))) { _filestream << obs->n_positionStdev()(2); }; // Down StDev [m]
        _filestream << ",";
        // if (!std::isnan(obs->n_positionStdev()(0, 1))) { _filestream << obs->n_positionStdev()(0, 1); }; // NE-ECEF StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->n_positionStdev()(0, 2))) { _filestream << obs->n_positionStdev()(0, 2); }; // ND-ECEF StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->n_positionStdev()(1, 2))) { _filestream << obs->n_positionStdev()(1, 2); }; // ED-ECEF StDev [m]
        // _filestream << ",";
        if (!std::isnan(obs->e_velocityStdev()(0))) { _filestream << obs->e_velocityStdev()(0); }; // X velocity ECEF StDev [m/s]
        _filestream << ",";
        if (!std::isnan(obs->e_velocityStdev()(1))) { _filestream << obs->e_velocityStdev()(1); }; // Y velocity ECEF StDev [m/s]
        _filestream << ",";
        if (!std::isnan(obs->e_velocityStdev()(2))) { _filestream << obs->e_velocityStdev()(2); }; // Z velocity ECEF StDev [m/s]
        _filestream << ",";
        // if (!std::isnan(obs->e_velocityStdev()(0, 1))) { _filestream << obs->e_velocityStdev()(0, 1); }; // XY velocity StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->e_velocityStdev()(0, 2))) { _filestream << obs->e_velocityStdev()(0, 2); }; // XZ velocity StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->e_velocityStdev()(1, 2))) { _filestream << obs->e_velocityStdev()(1, 2); }; // YZ velocity StDev [m]
        // _filestream << ",";
        if (!std::isnan(obs->n_velocityStdev()(0))) { _filestream << obs->n_velocityStdev()(0); }; // North velocity StDev [m/s]
        _filestream << ",";
        if (!std::isnan(obs->n_velocityStdev()(1))) { _filestream << obs->n_velocityStdev()(1); }; // East velocity StDev [m/s]
        _filestream << ",";
        if (!std::isnan(obs->n_velocityStdev()(2))) { _filestream << obs->n_velocityStdev()(2); }; // Down velocity StDev [m/s]
        _filestream << ",";
        // if (!std::isnan(obs->n_velocityStdev()(0, 1))) { _filestream << obs->n_velocityStdev()(0, 1); }; // NE velocity StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->n_velocityStdev()(0, 2))) { _filestream << obs->n_velocityStdev()(0, 2); }; // ND velocity StDev [m]
        // _filestream << ",";
        // if (!std::isnan(obs->n_velocityStdev()(1, 2))) { _filestream << obs->n_velocityStdev()(1, 2); }; // ED velocity StDev [m]
        // _filestream << ",";

        _filestream << '\n';
    }
}