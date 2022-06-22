#include "SppSolutionLogger.hpp"

#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/SppSolution.hpp"

NAV::SppSolutionLogger::SppSolutionLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::CSV;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { SppSolution::type() }, &SppSolutionLogger::writeObservation);
}

NAV::SppSolutionLogger::~SppSolutionLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SppSolutionLogger::typeStatic()
{
    return "SppSolutionLogger";
}

std::string NAV::SppSolutionLogger::type() const
{
    return typeStatic();
}

std::string NAV::SppSolutionLogger::category()
{
    return "Data Logger";
}

void NAV::SppSolutionLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::SppSolutionLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::SppSolutionLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::SppSolutionLogger::flush()
{
    _filestream.flush();
}

bool NAV::SppSolutionLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    _filestream << "GpsCycle,GpsWeek,GpsTow [s],"
                << "Pos ECEF X [m],Pos ECEF Y [m],Pos ECEF Z [m],Latitude [deg],Longitude [deg],Altitude [m],"
                << "Vel ECEF X [m/s],Vel ECEF Y [m/s],Vel ECEF Z [m/s],Vel N [m/s],Vel E [m/s],Vel D [m/s],"
                << "Number satellites (pos),Number satellites (vel),"
                << "Receiver clock bias [s],Receiver clock drift [s/s],"
                << "X-ECEF StDev [m],Y-ECEF StDev [m],Z-ECEF StDev [m],"
                << "XY-ECEF StDev [m],XZ-ECEF StDev [m],YZ-ECEF StDev [m],"
                << "North StDev [m],East StDev [m],Down StDev [m],"
                << "NE-ECEF StDev [m],ND-ECEF StDev [m],ED-ECEF StDev [m],"
                << "X velocity ECEF StDev [m/s],Y velocity ECEF StDev [m/s],Z velocity ECEF StDev [m/s],"
                << "XY velocity StDev [m],XZ velocity StDev [m],YZ velocity StDev [m],"
                << "North velocity StDev [m/s],East velocity StDev [m/s],Down velocity StDev [m/s],"
                << "NE velocity StDev [m],ND velocity StDev [m],ED velocity StDev [m],"
                << "Receiver clock bias StDev [s],Receiver clock drift StDev [s/s]"
                << std::endl;

    return true;
}

void NAV::SppSolutionLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::SppSolutionLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto obs = std::static_pointer_cast<const SppSolution>(nodeData);

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 12;

    if (obs->insTime.has_value())
    {
        _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.value().toGPSweekTow().gpsCycle;
    }
    _filestream << ",";
    if (obs->insTime.has_value())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().gpsWeek;
    }
    _filestream << ",";
    if (obs->insTime.has_value())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().tow;
    }
    _filestream << "," << std::setprecision(valuePrecision);

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

    _filestream << obs->nSatellitesPosition << ",";                 // Number satellites (pos)
    _filestream << obs->nSatellitesVelocity << ",";                 // Number satellites (vel)
    if (!std::isnan(obs->clkBias)) { _filestream << obs->clkBias; } // Receiver clock bias [s]
    _filestream << ",";
    if (!std::isnan(obs->clkDrift)) { _filestream << obs->clkDrift; } // Receiver clock drift [s/s]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(0, 0))) { _filestream << obs->e_positionStdev()(0, 0); }; // X-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(1, 1))) { _filestream << obs->e_positionStdev()(1, 1); }; // Y-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(2, 2))) { _filestream << obs->e_positionStdev()(2, 2); }; // Z-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(0, 1))) { _filestream << obs->e_positionStdev()(0, 1); }; // XY-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(0, 2))) { _filestream << obs->e_positionStdev()(0, 2); }; // XZ-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_positionStdev()(1, 2))) { _filestream << obs->e_positionStdev()(1, 2); }; // YZ-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(0, 0))) { _filestream << obs->n_positionStdev()(0, 0); }; // North StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(1, 1))) { _filestream << obs->n_positionStdev()(1, 1); }; // East StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(2, 2))) { _filestream << obs->n_positionStdev()(2, 2); }; // Down StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(0, 1))) { _filestream << obs->n_positionStdev()(0, 1); }; // NE-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(0, 2))) { _filestream << obs->n_positionStdev()(0, 2); }; // ND-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_positionStdev()(1, 2))) { _filestream << obs->n_positionStdev()(1, 2); }; // ED-ECEF StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(0, 0))) { _filestream << obs->e_velocityStdev()(0, 0); }; // X velocity ECEF StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(1, 1))) { _filestream << obs->e_velocityStdev()(1, 1); }; // Y velocity ECEF StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(2, 2))) { _filestream << obs->e_velocityStdev()(2, 2); }; // Z velocity ECEF StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(0, 1))) { _filestream << obs->e_velocityStdev()(0, 1); }; // XY velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(0, 2))) { _filestream << obs->e_velocityStdev()(0, 2); }; // XZ velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->e_velocityStdev()(1, 2))) { _filestream << obs->e_velocityStdev()(1, 2); }; // YZ velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(0, 0))) { _filestream << obs->n_velocityStdev()(0, 0); }; // North velocity StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(1, 1))) { _filestream << obs->n_velocityStdev()(1, 1); }; // East velocity StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(2, 2))) { _filestream << obs->n_velocityStdev()(2, 2); }; // Down velocity StDev [m/s]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(0, 1))) { _filestream << obs->n_velocityStdev()(0, 1); }; // NE velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(0, 2))) { _filestream << obs->n_velocityStdev()(0, 2); }; // ND velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->n_velocityStdev()(1, 2))) { _filestream << obs->n_velocityStdev()(1, 2); }; // ED velocity StDev [m]
    _filestream << ",";
    if (!std::isnan(obs->clkBiasStdev)) { _filestream << obs->clkBiasStdev; } // Receiver clock bias StDev [s]
    _filestream << ",";
    if (!std::isnan(obs->clkDriftStdev)) { _filestream << obs->clkDriftStdev; } // Receiver clock drift StDev [s/s]
    _filestream << '\n';
}