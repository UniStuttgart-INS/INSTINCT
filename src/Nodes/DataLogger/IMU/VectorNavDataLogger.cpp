#include "VectorNavDataLogger.hpp"

#include "NodeData/VectorNavBinaryOutput.hpp"

#include "util/Logger.hpp"

#include "gui/widgets/FileDialog.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::VectorNavDataLogger::VectorNavDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::ASCII;

    hasConfig = true;
    guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "BinaryOutput", Pin::Type::Flow, { NAV::VectorNavBinaryOutput::type() }, &VectorNavDataLogger::writeObservation);
}

NAV::VectorNavDataLogger::~VectorNavDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavDataLogger::typeStatic()
{
    return "VectorNavDataLogger";
}

std::string NAV::VectorNavDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavDataLogger::category()
{
    return "Data Logger";
}

void NAV::VectorNavDataLogger::guiConfig()
{
    if (gui::widgets::FileDialogSave(path, "Save File", ".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::VectorNavDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::VectorNavDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

bool NAV::VectorNavDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    headerWritten = false;

    return true;
}

void NAV::VectorNavDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::VectorNavDataLogger::writeObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<VectorNavBinaryOutput>(nodeData);

    if (!headerWritten) // TODO: This solution does not work when the config changes while data is sent out
    {
        filestream << "GpsCycle,GpsWeek,GpsToW";

        // Group 2 (Time)
        if (obs->timeOutputs)
        {
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
            {
                filestream << ",TimeStartup";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
            {
                filestream << ",TimeGps";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
            {
                filestream << ",GpsTow";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
            {
                filestream << ",GpsWeek";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
            {
                filestream << ",TimeSyncIn";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
            {
                filestream << ",TimeGpsPps";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
            {
                filestream << ",UTCyear,UTCmonth,UTCday,UTChour,UTCmin,UTCsec,UTCms";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
            {
                filestream << ",SyncInCnt";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
            {
                filestream << ",SyncOutCnt";
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
            {
                filestream << ",TimeStatus";
            }
        }
        // Group 3 (IMU)
        if (obs->imuOutputs)
        {
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
            {
                filestream << ",ImuStatus";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
            {
                filestream << ",UncompMagX,UncompMagY,UncompMagZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                filestream << ",UncompAccelX,UncompAccelY,UncompAccelZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                filestream << ",UncompGyroX,UncompGyroY,UncompGyroZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                filestream << ",Temp";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                filestream << ",Pres";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                filestream << ",DeltaTime,DeltaThetaX,DeltaThetaY,DeltaThetaZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                filestream << ",DeltaVelX,DeltaVelY,DeltaVelZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                filestream << ",MagX,MagY,MagZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                filestream << ",AccelX,AccelY,AccelZ";
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                filestream << ",AngularRateX,AngularRateY,AngularRateZ";
            }
        }
        // Group 4 (GNSS1)
        if (obs->gnss1Outputs)
        {
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                filestream << ",gnss1-UTCyear,gnss1-UTCmonth,gnss1-UTCday,gnss1-UTChour,gnss1-UTCmin,gnss1-UTCsec,gnss1-UTCms";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                filestream << ",gnss1-Tow";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                filestream << ",gnss1-Week";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                filestream << ",gnss1-NumSats";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                filestream << ",gnss1-Fix";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                filestream << ",gnss1-PosLat,gnss1-PosLon,gnss1-PosAlt";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                filestream << ",gnss1-PosEcefX,gnss1-PosEcefY,gnss1-PosEcefZ";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                filestream << ",gnss1-VelN,gnss1-VelE,gnss1-VelD";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                filestream << ",gnss1-VelEcefX,gnss1-VelEcefY,gnss1-VelEcefZ";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                filestream << ",gnss1-PosU-N,gnss1-PosU-E,gnss1-PosU-D";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                filestream << ",gnss1-VelU";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                filestream << ",gnss1-TimeU";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                filestream << ",gnss1-TimeInfo,gnss1-LeapSeconds";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                filestream << ",gnss1-gDOP,gnss1-pDOP,gnss1-tDOP,gnss1-vDOP,gnss1-hDOP,gnss1-nDOP,gnss1-eDOP";
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                filestream << ","; // TODO
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                filestream << ","; // TODO
            }
        }
        // Group 5 (Attitude)
        if (obs->attitudeOutputs)
        {
            // TODO
            // if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::)
            // {
            //     filestream << ",";
            // }
        }
        // Group 6 (INS)
        if (obs->insOutputs)
        {
            // TODO
            // if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::)
            // {
            //     filestream << ",";
            // }
        }
        // Group 7 (GNSS2)
        if (obs->gnss2Outputs)
        {
            // TODO
            // if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::)
            // {
            //     filestream << ",";
            // }
        }

        filestream << std::endl;
        headerWritten = true;
    }

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    // constexpr int valuePrecision = 9;

    if (obs->insTime.has_value())
    {
        filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.value().toGPSweekTow().gpsCycle;
    }
    filestream << ',';
    if (obs->insTime.has_value())
    {
        filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().gpsWeek;
    }
    filestream << ',';
    if (obs->insTime.has_value())
    {
        filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().tow;
    }
    // Group 2 (Time)
    if (obs->timeOutputs)
    {
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
        {
            filestream << "," << obs->timeOutputs->timeStartup;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
        {
            filestream << "," << obs->timeOutputs->timeGps;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
        {
            filestream << "," << obs->timeOutputs->gpsTow;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
        {
            filestream << "," << obs->timeOutputs->gpsWeek;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
        {
            filestream << "," << obs->timeOutputs->timeSyncIn;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
        {
            filestream << "," << obs->timeOutputs->timePPS;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
        {
            filestream << "," << obs->timeOutputs->timeUtc.year
                       << "," << obs->timeOutputs->timeUtc.month
                       << "," << obs->timeOutputs->timeUtc.day
                       << "," << obs->timeOutputs->timeUtc.hour
                       << "," << obs->timeOutputs->timeUtc.min
                       << "," << obs->timeOutputs->timeUtc.sec
                       << "," << obs->timeOutputs->timeUtc.ms;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
        {
            filestream << "," << obs->timeOutputs->syncInCnt;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
        {
            filestream << "," << obs->timeOutputs->syncOutCnt;
        }
        if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
        {
            filestream << "," << obs->timeOutputs->timeStatus;
        }
    }
    // Group 3 (IMU)
    if (obs->imuOutputs)
    {
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
        {
            filestream << "," << obs->imuOutputs->imuStatus;
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
        {
            filestream << "," << obs->imuOutputs->uncompMag(0)
                       << "," << obs->imuOutputs->uncompMag(1)
                       << "," << obs->imuOutputs->uncompMag(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
        {
            filestream << "," << obs->imuOutputs->uncompAccel(0)
                       << "," << obs->imuOutputs->uncompAccel(1)
                       << "," << obs->imuOutputs->uncompAccel(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
        {
            filestream << "," << obs->imuOutputs->uncompGyro(0)
                       << "," << obs->imuOutputs->uncompGyro(1)
                       << "," << obs->imuOutputs->uncompGyro(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
        {
            filestream << "," << obs->imuOutputs->temp;
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
        {
            filestream << "," << obs->imuOutputs->pres;
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
        {
            filestream << "," << obs->imuOutputs->deltaTime
                       << "," << obs->imuOutputs->deltaTheta(0)
                       << "," << obs->imuOutputs->deltaTheta(1)
                       << "," << obs->imuOutputs->deltaTheta(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
        {
            filestream << "," << obs->imuOutputs->deltaV(0)
                       << "," << obs->imuOutputs->deltaV(1)
                       << "," << obs->imuOutputs->deltaV(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
        {
            filestream << "," << obs->imuOutputs->mag(0)
                       << "," << obs->imuOutputs->mag(1)
                       << "," << obs->imuOutputs->mag(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
        {
            filestream << "," << obs->imuOutputs->accel(0)
                       << "," << obs->imuOutputs->accel(1)
                       << "," << obs->imuOutputs->accel(2);
        }
        if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
        {
            filestream << "," << obs->imuOutputs->angularRate(0)
                       << "," << obs->imuOutputs->angularRate(1)
                       << "," << obs->imuOutputs->angularRate(2);
        }
    }
    // Group 4 (GNSS1)
    if (obs->gnss1Outputs)
    {
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
        {
            filestream << "," << obs->gnss1Outputs->timeUtc.year
                       << "," << obs->gnss1Outputs->timeUtc.month
                       << "," << obs->gnss1Outputs->timeUtc.day
                       << "," << obs->gnss1Outputs->timeUtc.hour
                       << "," << obs->gnss1Outputs->timeUtc.min
                       << "," << obs->gnss1Outputs->timeUtc.sec
                       << "," << obs->gnss1Outputs->timeUtc.ms;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
        {
            filestream << "," << obs->gnss1Outputs->tow;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
        {
            filestream << "," << obs->gnss1Outputs->week;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
        {
            filestream << "," << obs->gnss1Outputs->numSats;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
        {
            filestream << "," << obs->gnss1Outputs->fix;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
        {
            filestream << "," << obs->gnss1Outputs->posLla(0)
                       << "," << obs->gnss1Outputs->posLla(1)
                       << "," << obs->gnss1Outputs->posLla(2);
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
        {
            filestream << "," << obs->gnss1Outputs->posEcef(0)
                       << "," << obs->gnss1Outputs->posEcef(1)
                       << "," << obs->gnss1Outputs->posEcef(2);
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
        {
            filestream << "," << obs->gnss1Outputs->velNed(0)
                       << "," << obs->gnss1Outputs->velNed(1)
                       << "," << obs->gnss1Outputs->velNed(2);
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
        {
            filestream << "," << obs->gnss1Outputs->velEcef(0)
                       << "," << obs->gnss1Outputs->velEcef(1)
                       << "," << obs->gnss1Outputs->velEcef(2);
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
        {
            filestream << "," << obs->gnss1Outputs->posU(0)
                       << "," << obs->gnss1Outputs->posU(1)
                       << "," << obs->gnss1Outputs->posU(2);
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
        {
            filestream << "," << obs->gnss1Outputs->velU;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
        {
            filestream << "," << obs->gnss1Outputs->timeU;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
        {
            filestream << "," << obs->gnss1Outputs->timeInfo.status
                       << "," << obs->gnss1Outputs->timeInfo.leapSeconds;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
        {
            filestream << ",gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP";
            filestream << "," << obs->gnss1Outputs->dop.gDop
                       << "," << obs->gnss1Outputs->dop.pDop
                       << "," << obs->gnss1Outputs->dop.tDop
                       << "," << obs->gnss1Outputs->dop.vDop
                       << "," << obs->gnss1Outputs->dop.hDop
                       << "," << obs->gnss1Outputs->dop.nDop
                       << "," << obs->gnss1Outputs->dop.eDop;
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
        {
            filestream << ","; // TODO
        }
        if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
        {
            filestream << ","; // TODO
        }
    }
    // Group 5 (Attitude)
    if (obs->attitudeOutputs)
    {
        // TODO
        // if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::)
        // {
        //     filestream << ",";
        // }
    }
    // Group 6 (INS)
    if (obs->insOutputs)
    {
        // TODO
        // if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::)
        // {
        //     filestream << ",";
        // }
    }
    // Group 7 (GNSS2)
    if (obs->gnss2Outputs)
    {
        // TODO
        // if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::)
        // {
        //     filestream << ",";
        // }
    }

    filestream << std::endl; // FIXME: Replace with \n
}