#include "PosVelAttLogger.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InertialNavSol.hpp"

#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::PosVelAttLogger::PosVelAttLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::CSV;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { Pos::type(), PosVel::type(), PosVelAtt::type() }, &PosVelAttLogger::writeObservation);
}

NAV::PosVelAttLogger::~PosVelAttLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PosVelAttLogger::typeStatic()
{
    return "PosVelAttLogger";
}

std::string NAV::PosVelAttLogger::type() const
{
    return typeStatic();
}

std::string NAV::PosVelAttLogger::category()
{
    return "Data Logger";
}

void NAV::PosVelAttLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }
}

[[nodiscard]] json NAV::PosVelAttLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::PosVelAttLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::PosVelAttLogger::flush()
{
    _filestream.flush();
}

bool NAV::PosVelAttLogger::initialize()
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
                << "Vel ECEF X [m/s],Vel ECEF Y [m/s],Vel ECEF Z [m/s],Vel N [m/s],Vel E [m/s],Vel D [m/s],"
                << "n_Quat_b w,n_Quat_b x,n_Quat_b y,n_Quat_b z,Roll [deg],Pitch [deg],Yaw [deg]" << std::endl;

    return true;
}

void NAV::PosVelAttLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::PosVelAttLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        if (auto* sourcePin = nm::FindOutputPin(link->startPinId))
        {
            constexpr int gpsCyclePrecision = 3;
            constexpr int gpsTimePrecision = 12;
            constexpr int valuePrecision = 15;

            {
                auto obs = std::static_pointer_cast<const Pos>(nodeData);
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

                // -------------------------------------------------------- Position -----------------------------------------------------------

                if (!std::isnan(obs->e_position().x()))
                {
                    _filestream << obs->e_position().x();
                }
                _filestream << ",";
                if (!std::isnan(obs->e_position().y()))
                {
                    _filestream << obs->e_position().y();
                }
                _filestream << ",";
                if (!std::isnan(obs->e_position().z()))
                {
                    _filestream << obs->e_position().z();
                }
                _filestream << ",";
                if (!std::isnan(obs->lla_position().x()))
                {
                    _filestream << rad2deg(obs->lla_position().x());
                }
                _filestream << ",";
                if (!std::isnan(obs->lla_position().y()))
                {
                    _filestream << rad2deg(obs->lla_position().y());
                }
                _filestream << ",";
                if (!std::isnan(obs->lla_position().z()))
                {
                    _filestream << obs->lla_position().z();
                }
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
            }
            // -------------------------------------------------------- Velocity -----------------------------------------------------------
            if (sourcePin->dataIdentifier.front() == PosVelAtt::type()
                || sourcePin->dataIdentifier.front() == PosVel::type()
                || sourcePin->dataIdentifier.front() == InertialNavSol::type())
            {
                auto obs = std::static_pointer_cast<const PosVel>(nodeData);

                if (!std::isnan(obs->e_velocity().x()))
                {
                    _filestream << obs->e_velocity().x();
                }
                _filestream << ",";
                if (!std::isnan(obs->e_velocity().y()))
                {
                    _filestream << obs->e_velocity().y();
                }
                _filestream << ",";
                if (!std::isnan(obs->e_velocity().z()))
                {
                    _filestream << obs->e_velocity().z();
                }
                _filestream << ",";
                if (!std::isnan(obs->n_velocity().x()))
                {
                    _filestream << obs->n_velocity().x();
                }
                _filestream << ",";
                if (!std::isnan(obs->n_velocity().y()))
                {
                    _filestream << obs->n_velocity().y();
                }
                _filestream << ",";
                if (!std::isnan(obs->n_velocity().z()))
                {
                    _filestream << obs->n_velocity().z();
                }
                _filestream << ",";
            }
            else
            {
                _filestream << ",,,,,,";
            }
            // -------------------------------------------------------- Attitude -----------------------------------------------------------
            if (sourcePin->dataIdentifier.front() == PosVelAtt::type()
                || sourcePin->dataIdentifier.front() == InertialNavSol::type())
            {
                auto obs = std::static_pointer_cast<const PosVelAtt>(nodeData);
                if (!obs->n_Quat_b().coeffs().isZero())
                {
                    _filestream << obs->n_Quat_b().w();
                    _filestream << ",";
                    _filestream << obs->n_Quat_b().x();
                    _filestream << ",";
                    _filestream << obs->n_Quat_b().y();
                    _filestream << ",";
                    _filestream << obs->n_Quat_b().z();
                    _filestream << ",";

                    Eigen::Vector3d rpy = rad2deg(obs->rollPitchYaw());
                    _filestream << rpy.x();
                    _filestream << ",";
                    _filestream << rpy.y();
                    _filestream << ",";
                    _filestream << rpy.z();
                }
                else
                {
                    _filestream << ",,,,,,";
                }
            }
            else
            {
                _filestream << ",,,,,,";
            }

            _filestream << '\n';
        }
    }
}