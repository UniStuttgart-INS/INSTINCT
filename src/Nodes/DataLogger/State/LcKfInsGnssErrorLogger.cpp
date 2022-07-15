#include "LcKfInsGnssErrorLogger.hpp"

#include "NodeData/State/LcKfInsGnssErrors.hpp"

#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::LcKfInsGnssErrorLogger::LcKfInsGnssErrorLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::CSV;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { LcKfInsGnssErrors::type() }, &LcKfInsGnssErrorLogger::writeObservation);
}

NAV::LcKfInsGnssErrorLogger::~LcKfInsGnssErrorLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LcKfInsGnssErrorLogger::typeStatic()
{
    return "LcKfInsGnssErrorLogger";
}

std::string NAV::LcKfInsGnssErrorLogger::type() const
{
    return typeStatic();
}

std::string NAV::LcKfInsGnssErrorLogger::category()
{
    return "Data Logger";
}

void NAV::LcKfInsGnssErrorLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::LcKfInsGnssErrorLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::LcKfInsGnssErrorLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::LcKfInsGnssErrorLogger::flush()
{
    _filestream.flush();
}

bool NAV::LcKfInsGnssErrorLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _filestream << "Time [s],GpsCycle,GpsWeek,GpsTow [s],"
                // PVAError
                << "Roll error [deg],Pitch error [deg],Yaw error [deg],"
                << "North velocity error [m/s],East velocity error [m/s],Down velocity error [m/s],"
                << "Latitude error [deg],Longitude error [deg],Altitude error [m],"
                << "Alpha_eb [deg],Beta_eb [deg],Gamma_eb [deg],"
                << "ECEF X velocity error [m/s],ECEF Y velocity error [m/s],ECEF Z velocity error [m/s],"
                << "ECEF X error [m],ECEF Y error [m],ECEF Z error [m],"
                // ImuBiases
                << "Accelerometer bias b_X accumulated [m/s^2],Accelerometer bias b_Y accumulated [m/s^2],Accelerometer bias b_Z accumulated [m/s^2],"
                << "Gyroscope bias b_X accumulated [rad/s],Gyroscope bias b_Y accumulated [rad/s],Gyroscope bias b_Z accumulated [rad/s]"
                << std::endl;

    return true;
}

void NAV::LcKfInsGnssErrorLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::LcKfInsGnssErrorLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    auto obs = std::static_pointer_cast<const LcKfInsGnssErrors>(nodeData);
    if (obs->insTime.has_value())
    {
        _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(obs->insTime.value()) * 1e9) / 1e9;
    }
    _filestream << ",";
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

    // ----------------------------------------------- PVAError --------------------------------------------------

    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->attitudeError(0))) { _filestream << rad2deg(obs->attitudeError(0)); }; // Roll error [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->attitudeError(1))) { _filestream << rad2deg(obs->attitudeError(1)); }; // Pitch error [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->attitudeError(2))) { _filestream << rad2deg(obs->attitudeError(2)); }; // Yaw error [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->velocityError(0))) { _filestream << obs->velocityError(0); }; // North velocity error [m/s]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->velocityError(1))) { _filestream << obs->velocityError(1); }; // East velocity error [m/s]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->velocityError(2))) { _filestream << obs->velocityError(2); }; // Down velocity error [m/s]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->positionError(0))) { _filestream << rad2deg(obs->positionError(0)); }; // Latitude error [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->positionError(1))) { _filestream << rad2deg(obs->positionError(1)); }; // Longitude error [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::NED && !std::isnan(obs->positionError(2))) { _filestream << obs->positionError(2); }; // Altitude error [m]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->attitudeError(0))) { _filestream << rad2deg(obs->attitudeError(0)); }; // Alpha_eb [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->attitudeError(1))) { _filestream << rad2deg(obs->attitudeError(1)); }; // Beta_eb [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->attitudeError(2))) { _filestream << rad2deg(obs->attitudeError(2)); }; // Gamma_eb [deg]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->velocityError(0))) { _filestream << obs->velocityError(0); }; // ECEF X velocity error [m/s]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->velocityError(1))) { _filestream << obs->velocityError(1); }; // ECEF Y velocity error [m/s]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->velocityError(2))) { _filestream << obs->velocityError(2); }; // ECEF Z velocity error [m/s]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->positionError(0))) { _filestream << obs->positionError(0); }; // ECEF X error [m]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->positionError(1))) { _filestream << obs->positionError(1); }; // ECEF Y error [m]
    _filestream << ",";
    if (obs->frame == LcKfInsGnssErrors::Frame::ECEF && !std::isnan(obs->positionError(2))) { _filestream << obs->positionError(2); }; // ECEF Z error [m]
    _filestream << ",";

    // ----------------------------------------------- ImuBiases -------------------------------------------------

    if (!std::isnan(obs->b_biasAccel(0))) { _filestream << obs->b_biasAccel(0); }; // Accelerometer bias b_X accumulated [m/s^2]
    _filestream << ",";
    if (!std::isnan(obs->b_biasAccel(1))) { _filestream << obs->b_biasAccel(1); }; // Accelerometer bias b_Y accumulated [m/s^2]
    _filestream << ",";
    if (!std::isnan(obs->b_biasAccel(2))) { _filestream << obs->b_biasAccel(2); }; // Accelerometer bias b_Z accumulated [m/s^2]
    _filestream << ",";
    if (!std::isnan(obs->b_biasGyro(0))) { _filestream << obs->b_biasGyro(0); }; // Gyroscope bias b_X accumulated [rad/s]
    _filestream << ",";
    if (!std::isnan(obs->b_biasGyro(1))) { _filestream << obs->b_biasGyro(1); }; // Gyroscope bias b_Y accumulated [rad/s]
    _filestream << ",";
    if (!std::isnan(obs->b_biasGyro(2))) { _filestream << obs->b_biasGyro(2); }; // Gyroscope bias b_Z accumulated [rad/s]
    _filestream << '\n';
}