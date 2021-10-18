#include "VectorNavDataLogger.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

#include "internal/gui/widgets/FileDialog.hpp"

#include <iomanip> // std::setprecision
#include <limits>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::VectorNavDataLogger::VectorNavDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::BINARY;

    hasConfig = true;
    guiConfigDefaultWindowSize = { 380, 92 };

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
    if (gui::widgets::FileDialogSave(path, "Save File", fileType == FileType::CSV ? ".csv" : ".vnb", { fileType == FileType::CSV ? ".csv" : ".vnb" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }

    static constexpr std::array<FileType, 2> fileTypes = {
        { FileType::CSV,
          FileType::BINARY }
    };
    if (ImGui::BeginCombo(fmt::format("Mode##{}", size_t(id)).c_str(), FileWriter::str(fileType).c_str()))
    {
        for (const auto& type : fileTypes)
        {
            const bool isSelected = (fileType == type);
            if (ImGui::Selectable(str(type).c_str(), isSelected))
            {
                fileType = type;
                LOG_DEBUG("{}: fileType changed to {}", nameId(), FileWriter::str(fileType));
                str::replace(path, fileType == FileType::CSV ? ".vnb" : ".csv", fileType == FileType::CSV ? ".csv" : ".vnb");
                flow::ApplyChanges();
                if (isInitialized())
                {
                    deinitialize();
                    initialize();
                }
            }

            if (isSelected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
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

bool NAV::VectorNavDataLogger::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    if (isInitialized())
    {
        deinitialize();
        initialize();
    }

    return true;
}

void NAV::VectorNavDataLogger::flush()
{
    filestream.flush();
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

void NAV::VectorNavDataLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::dynamic_pointer_cast<const VectorNavBinaryOutput>(nodeData);

    if (fileType == FileType::CSV)
    {
        if (!headerWritten)
        {
            filestream << "GpsCycle,GpsWeek,GpsTow";

            // Group 2 (Time)
            if (obs->timeOutputs)
            {
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
                {
                    filestream << ",Time::TimeStartup";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
                {
                    filestream << ",Time::TimeGps";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
                {
                    filestream << ",Time::GpsTow";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
                {
                    filestream << ",Time::GpsWeek";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
                {
                    filestream << ",Time::TimeSyncIn";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
                {
                    filestream << ",Time::TimeGpsPps";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
                {
                    filestream << ",Time::TimeUTC::year,Time::TimeUTC::month,Time::TimeUTC::day,Time::TimeUTC::hour,Time::TimeUTC::min,Time::TimeUTC::sec,Time::TimeUTC::ms";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
                {
                    filestream << ",Time::SyncInCnt";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
                {
                    filestream << ",Time::SyncOutCnt";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
                {
                    filestream << ",Time::TimeStatus::timeOk,Time::TimeStatus::dateOk,Time::TimeStatus::utcTimeValid";
                }
            }
            // Group 3 (IMU)
            if (obs->imuOutputs)
            {
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
                {
                    filestream << ",IMU::ImuStatus";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
                {
                    filestream << ",IMU::UncompMag::X,IMU::UncompMag::Y,IMU::UncompMag::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
                {
                    filestream << ",IMU::UncompAccel::X,IMU::UncompAccel::Y,IMU::UncompAccel::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
                {
                    filestream << ",IMU::UncompGyro::X,IMU::UncompGyro::Y,IMU::UncompGyro::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
                {
                    filestream << ",IMU::Temp";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
                {
                    filestream << ",IMU::Pres";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
                {
                    filestream << ",IMU::DeltaTime,IMU::DeltaTheta::X,IMU::DeltaTheta::Y,IMU::DeltaTheta::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
                {
                    filestream << ",IMU::DeltaVel::X,IMU::DeltaVel::Y,IMU::DeltaVel::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
                {
                    filestream << ",IMU::Mag::X,IMU::Mag::Y,IMU::Mag::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
                {
                    filestream << ",IMU::Accel::X,IMU::Accel::Y,IMU::Accel::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
                {
                    filestream << ",IMU::AngularRate::X,IMU::AngularRate::Y,IMU::AngularRate::Z";
                }
            }
            // Group 4 (GNSS1)
            if (obs->gnss1Outputs)
            {
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    filestream << ",GNSS1::UTC::year,GNSS1::UTC::month,GNSS1::UTC::day,GNSS1::UTC::hour,GNSS1::UTC::min,GNSS1::UTC::sec,GNSS1::UTC::ms";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    filestream << ",GNSS1::Tow";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    filestream << ",GNSS1::Week";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    filestream << ",GNSS1::NumSats";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    filestream << ",GNSS1::Fix";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    filestream << ",GNSS1::PosLla::latitude,GNSS1::PosLla::longitude,GNSS1::PosLla::altitude";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    filestream << ",GNSS1::PosEcef::X,GNSS1::PosEcef::Y,GNSS1::PosEcef::Z";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    filestream << ",GNSS1::VelNed::N,GNSS1::VelNed::E,GNSS1::VelNed::D";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    filestream << ",GNSS1::VelEcef::X,GNSS1::VelEcef::Y,GNSS1::VelEcef::Z";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    filestream << ",GNSS1::PosU::N,GNSS1::PosU::E,GNSS1::PosU::D";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    filestream << ",GNSS1::VelU";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    filestream << ",GNSS1::TimeU";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    filestream << ",GNSS1::TimeInfo::Status::timeOk,GNSS1::TimeInfo::Status::dateOk,GNSS1::TimeInfo::Status::utcTimeValid,GNSS1::TimeInfo::LeapSeconds";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    filestream << ",GNSS1::DOP::g,GNSS1::DOP::p,GNSS1::DOP::t,GNSS1::DOP::v,GNSS1::DOP::h,GNSS1::DOP::n,GNSS1::DOP::e";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    filestream << ",GNSS1::SatInfo::NumSats,GNSS1::SatInfo::Satellites";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    filestream << ",GNSS1::RawMeas::Tow,GNSS1::RawMeas::Week,GNSS1::RawMeas::NumSats,GNSS1::RawMeas::Satellites";
                }
            }
            // Group 5 (Attitude)
            if (obs->attitudeOutputs)
            {
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
                {
                    filestream << ",Att::VpeStatus::AttitudeQuality"
                                  ",Att::VpeStatus::GyroSaturation"
                                  ",Att::VpeStatus::GyroSaturationRecovery"
                                  ",Att::VpeStatus::MagDisturbance"
                                  ",Att::VpeStatus::MagSaturation"
                                  ",Att::VpeStatus::AccDisturbance"
                                  ",Att::VpeStatus::AccSaturation"
                                  ",Att::VpeStatus::KnownMagDisturbance"
                                  ",Att::VpeStatus::KnownAccelDisturbance";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
                {
                    filestream << ",Att::YawPitchRoll::Y,Att::YawPitchRoll::P,Att::YawPitchRoll::R";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
                {
                    filestream << ",Att::Quaternion::w,Att::Quaternion::x,Att::Quaternion::y,Att::Quaternion::z";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
                {
                    filestream << ",Att::DCM::0-0,Att::DCM::0-1,Att::DCM::0-2"
                                  ",Att::DCM::1-0,Att::DCM::1-1,Att::DCM::1-2"
                                  ",Att::DCM::2-0,Att::DCM::2-1,Att::DCM::2-2";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
                {
                    filestream << ",Att::MagNed::N,Att::MagNed::E,Att::MagNed::D";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
                {
                    filestream << ",Att::AccelNed::N,Att::AccelNed::E,Att::AccelNed::D";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
                {
                    filestream << ",Att::LinearAccelBody::X,Att::LinearAccelBody::Y,Att::LinearAccelBody::Z";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
                {
                    filestream << ",Att::LinearAccelNed::N,Att::LinearAccelNed::E,Att::LinearAccelNed::D";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
                {
                    filestream << ",Att::YprU::Y,Att::YprU::P,Att::YprU::R";
                }
            }
            // Group 6 (INS)
            if (obs->insOutputs)
            {
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
                {
                    filestream << ",INS::InsStatus::Mode"
                                  ",INS::InsStatus::GpsFix"
                                  ",INS::InsStatus::Error::IMU"
                                  ",INS::InsStatus::Error::MagPres"
                                  ",INS::InsStatus::Error::GNSS"
                                  ",INS::InsStatus::GpsHeadingIns"
                                  ",INS::InsStatus::GpsCompass";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
                {
                    filestream << ",INS::PosLla::latitude,INS::PosLla::longitude,INS::PosLla::altitude";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
                {
                    filestream << ",INS::PosEcef::X,INS::PosEcef::Y,INS::PosEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                {
                    filestream << ",INS::VelBody::X,INS::VelBody::Y,INS::VelBody::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
                {
                    filestream << ",INS::VelNed::N,INS::VelNed::E,INS::VelNed::D";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                {
                    filestream << ",INS::VelEcef::X,INS::VelEcef::Y,INS::VelEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
                {
                    filestream << ",INS::MagEcef::X,INS::MagEcef::Y,INS::MagEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
                {
                    filestream << ",INS::AccelEcef::X,INS::AccelEcef::Y,INS::AccelEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
                {
                    filestream << ",INS::LinearAccelEcef::X,INS::LinearAccelEcef::Y,INS::LinearAccelEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
                {
                    filestream << ",INS::PosU";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
                {
                    filestream << ",INS::VelU";
                }
            }
            // Group 7 (GNSS2)
            if (obs->gnss2Outputs)
            {
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    filestream << ",GNSS2::UTC::year,GNSS2::UTC::month,GNSS2::UTC::day,GNSS2::UTC::hour,GNSS2::UTC::min,GNSS2::UTC::sec,GNSS2::UTC::ms";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    filestream << ",GNSS2::Tow";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    filestream << ",GNSS2::Week";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    filestream << ",GNSS2::NumSats";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    filestream << ",GNSS2::Fix";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    filestream << ",GNSS2::PosLla::latitude,GNSS2::PosLla::longitude,GNSS2::PosLla::altitude";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    filestream << ",GNSS2::PosEcef::X,GNSS2::PosEcef::Y,GNSS2::PosEcef::Z";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    filestream << ",GNSS2::VelNed::N,GNSS2::VelNed::E,GNSS2::VelNed::D";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    filestream << ",GNSS2::VelEcef::X,GNSS2::VelEcef::Y,GNSS2::VelEcef::Z";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    filestream << ",GNSS2::PosU::N,GNSS2::PosU::E,GNSS2::PosU::D";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    filestream << ",GNSS2::VelU";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    filestream << ",GNSS2::TimeU";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    filestream << ",GNSS2::TimeInfo::Status::timeOk,GNSS2::TimeInfo::Status::dateOk,GNSS2::TimeInfo::Status::utcTimeValid,GNSS2::TimeInfo::LeapSeconds";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    filestream << ",GNSS2::DOP::g,GNSS2::DOP::p,GNSS2::DOP::t,GNSS2::DOP::v,GNSS2::DOP::h,GNSS2::DOP::n,GNSS2::DOP::e";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    filestream << ",GNSS2::SatInfo::NumSats,GNSS2::SatInfo::Satellites";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    filestream << ",GNSS2::RawMeas::Tow,GNSS2::RawMeas::Week,GNSS2::RawMeas::NumSats,GNSS2::RawMeas::Satellites";
                }
            }

            filestream << std::endl;
            headerWritten = true;
        }

        constexpr int gpsCyclePrecision = 3;
        constexpr int gpsTimePrecision = 12;
        constexpr int floatPrecision = std::numeric_limits<float>::digits10 + 2;
        constexpr int doublePrecision = std::numeric_limits<double>::digits10 + 2;

        if (obs->insTime.has_value())
        {
            filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime->toGPSweekTow().gpsCycle;
        }
        filestream << ',';
        if (obs->insTime.has_value())
        {
            filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime->toGPSweekTow().gpsWeek;
        }
        filestream << ',';
        if (obs->insTime.has_value())
        {
            filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime->toGPSweekTow().tow;
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
                filestream << "," << static_cast<int>(obs->timeOutputs->timeUtc.year)
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.month)
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.day)
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.hour)
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.min)
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.sec)
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
                filestream << "," << static_cast<unsigned int>(obs->timeOutputs->timeStatus.timeOk())
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeStatus.dateOk())
                           << "," << static_cast<unsigned int>(obs->timeOutputs->timeStatus.utcTimeValid());
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
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->uncompMag(0)
                           << "," << obs->imuOutputs->uncompMag(1)
                           << "," << obs->imuOutputs->uncompMag(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->uncompAccel(0)
                           << "," << obs->imuOutputs->uncompAccel(1)
                           << "," << obs->imuOutputs->uncompAccel(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->uncompGyro(0)
                           << "," << obs->imuOutputs->uncompGyro(1)
                           << "," << obs->imuOutputs->uncompGyro(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                filestream << "," << std::setprecision(floatPrecision) << obs->imuOutputs->temp;
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                filestream << "," << std::setprecision(floatPrecision) << obs->imuOutputs->pres;
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->deltaTime
                           << "," << obs->imuOutputs->deltaTheta(0)
                           << "," << obs->imuOutputs->deltaTheta(1)
                           << "," << obs->imuOutputs->deltaTheta(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->deltaV(0)
                           << "," << obs->imuOutputs->deltaV(1)
                           << "," << obs->imuOutputs->deltaV(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->mag(0)
                           << "," << obs->imuOutputs->mag(1)
                           << "," << obs->imuOutputs->mag(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->imuOutputs->accel(0)
                           << "," << obs->imuOutputs->accel(1)
                           << "," << obs->imuOutputs->accel(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                filestream << std::setprecision(floatPrecision);
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
                filestream << "," << static_cast<int>(obs->gnss1Outputs->timeUtc.year)
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.month)
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.day)
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.hour)
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.min)
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.sec)
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
                filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->numSats);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->fix);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                filestream << std::setprecision(doublePrecision);
                filestream << "," << obs->gnss1Outputs->posLla(0)
                           << "," << obs->gnss1Outputs->posLla(1)
                           << "," << obs->gnss1Outputs->posLla(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                filestream << std::setprecision(doublePrecision);
                filestream << "," << obs->gnss1Outputs->posEcef(0)
                           << "," << obs->gnss1Outputs->posEcef(1)
                           << "," << obs->gnss1Outputs->posEcef(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss1Outputs->velNed(0)
                           << "," << obs->gnss1Outputs->velNed(1)
                           << "," << obs->gnss1Outputs->velNed(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss1Outputs->velEcef(0)
                           << "," << obs->gnss1Outputs->velEcef(1)
                           << "," << obs->gnss1Outputs->velEcef(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss1Outputs->posU(0)
                           << "," << obs->gnss1Outputs->posU(1)
                           << "," << obs->gnss1Outputs->posU(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                filestream << "," << std::setprecision(floatPrecision) << obs->gnss1Outputs->velU;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                filestream << "," << std::setprecision(floatPrecision) << obs->gnss1Outputs->timeU;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeInfo.status.timeOk())
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeInfo.status.dateOk())
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeInfo.status.utcTimeValid())
                           << "," << static_cast<int>(obs->gnss1Outputs->timeInfo.leapSeconds);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                filestream << std::setprecision(floatPrecision);
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
                filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->satInfo.numSats)
                           << ",";
                for (auto& satellite : obs->gnss1Outputs->satInfo.satellites)
                {
                    filestream << "["
                               << static_cast<int>(satellite.sys) << "|"
                               << static_cast<unsigned int>(satellite.svId) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::Healthy) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::Almanac) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) << "|"
                               << static_cast<unsigned int>(satellite.cno) << "|"
                               << static_cast<unsigned int>(satellite.qi) << "|"
                               << static_cast<int>(satellite.el) << "|"
                               << satellite.az << "]";
                }
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                filestream << "," << obs->gnss1Outputs->raw.tow
                           << "," << obs->gnss1Outputs->raw.week
                           << "," << static_cast<unsigned int>(obs->gnss1Outputs->raw.numSats)
                           << ",";
                for (auto& satellite : obs->gnss1Outputs->raw.satellites)
                {
                    filestream << "["
                               << static_cast<int>(satellite.sys) << "|"
                               << static_cast<unsigned int>(satellite.svId) << "|"
                               << static_cast<unsigned int>(satellite.freq) << "|"
                               << static_cast<unsigned int>(satellite.chan) << "|"
                               << static_cast<int>(satellite.slot) << "|"
                               << static_cast<unsigned int>(satellite.cno) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::Searching) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::Tracking) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::TimeValid) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::CodeLock) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) << "|"
                               << std::setprecision(doublePrecision) << satellite.pr << "|"
                               << std::setprecision(doublePrecision) << satellite.cp << "|"
                               << std::setprecision(floatPrecision) << satellite.dp << "]";
                }
            }
        }
        // Group 5 (Attitude)
        if (obs->attitudeOutputs)
        {
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
            {
                filestream << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.attitudeQuality())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.gyroSaturation())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.gyroSaturationRecovery())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.magDisturbance())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.magSaturation())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.accDisturbance())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.accSaturation())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.knownMagDisturbance())
                           << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.knownAccelDisturbance());
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << std::setprecision(floatPrecision) << obs->attitudeOutputs->ypr(0)
                           << "," << obs->attitudeOutputs->ypr(1)
                           << "," << obs->attitudeOutputs->ypr(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->qtn.w()
                           << "," << obs->attitudeOutputs->qtn.x()
                           << "," << obs->attitudeOutputs->qtn.y()
                           << "," << obs->attitudeOutputs->qtn.z();
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->dcm(0, 0)
                           << "," << obs->attitudeOutputs->dcm(0, 1)
                           << "," << obs->attitudeOutputs->dcm(0, 2)
                           << "," << obs->attitudeOutputs->dcm(1, 0)
                           << "," << obs->attitudeOutputs->dcm(1, 1)
                           << "," << obs->attitudeOutputs->dcm(1, 2)
                           << "," << obs->attitudeOutputs->dcm(2, 0)
                           << "," << obs->attitudeOutputs->dcm(2, 1)
                           << "," << obs->attitudeOutputs->dcm(2, 2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->magNed(0)
                           << "," << obs->attitudeOutputs->magNed(1)
                           << "," << obs->attitudeOutputs->magNed(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->accelNed(0)
                           << "," << obs->attitudeOutputs->accelNed(1)
                           << "," << obs->attitudeOutputs->accelNed(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->linearAccelBody(0)
                           << "," << obs->attitudeOutputs->linearAccelBody(1)
                           << "," << obs->attitudeOutputs->linearAccelBody(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->linearAccelNed(0)
                           << "," << obs->attitudeOutputs->linearAccelNed(1)
                           << "," << obs->attitudeOutputs->linearAccelNed(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->attitudeOutputs->yprU(0)
                           << "," << obs->attitudeOutputs->yprU(1)
                           << "," << obs->attitudeOutputs->yprU(2);
            }
        }
        // Group 6 (INS)
        if (obs->insOutputs)
        {
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
            {
                filestream << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.mode())
                           << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.gpsFix())
                           << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.errorIMU())
                           << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.errorMagPres())
                           << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.errorGnss())
                           << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.gpsHeadingIns())
                           << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.gpsCompass());
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
            {
                filestream << std::setprecision(doublePrecision);
                filestream << "," << obs->insOutputs->posLla(0)
                           << "," << obs->insOutputs->posLla(1)
                           << "," << obs->insOutputs->posLla(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
            {
                filestream << std::setprecision(doublePrecision);
                filestream << "," << obs->insOutputs->posEcef(0)
                           << "," << obs->insOutputs->posEcef(1)
                           << "," << obs->insOutputs->posEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->velBody(0)
                           << "," << obs->insOutputs->velBody(1)
                           << "," << obs->insOutputs->velBody(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->velNed(0)
                           << "," << obs->insOutputs->velNed(1)
                           << "," << obs->insOutputs->velNed(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->velEcef(0)
                           << "," << obs->insOutputs->velEcef(1)
                           << "," << obs->insOutputs->velEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->magEcef(0)
                           << "," << obs->insOutputs->magEcef(1)
                           << "," << obs->insOutputs->magEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->accelEcef(0)
                           << "," << obs->insOutputs->accelEcef(1)
                           << "," << obs->insOutputs->accelEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->linearAccelEcef(0)
                           << "," << obs->insOutputs->linearAccelEcef(1)
                           << "," << obs->insOutputs->linearAccelEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->posU;
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->insOutputs->velU;
            }
        }
        // Group 7 (GNSS2)
        if (obs->gnss2Outputs)
        {
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                filestream << "," << static_cast<int>(obs->gnss2Outputs->timeUtc.year)
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.month)
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.day)
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.hour)
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.min)
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.sec)
                           << "," << obs->gnss2Outputs->timeUtc.ms;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                filestream << "," << obs->gnss2Outputs->tow;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                filestream << "," << obs->gnss2Outputs->week;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->numSats);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->fix);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                filestream << std::setprecision(doublePrecision);
                filestream << "," << obs->gnss2Outputs->posLla(0)
                           << "," << obs->gnss2Outputs->posLla(1)
                           << "," << obs->gnss2Outputs->posLla(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                filestream << std::setprecision(doublePrecision);
                filestream << "," << obs->gnss2Outputs->posEcef(0)
                           << "," << obs->gnss2Outputs->posEcef(1)
                           << "," << obs->gnss2Outputs->posEcef(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss2Outputs->velNed(0)
                           << "," << obs->gnss2Outputs->velNed(1)
                           << "," << obs->gnss2Outputs->velNed(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss2Outputs->velEcef(0)
                           << "," << obs->gnss2Outputs->velEcef(1)
                           << "," << obs->gnss2Outputs->velEcef(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss2Outputs->posU(0)
                           << "," << obs->gnss2Outputs->posU(1)
                           << "," << obs->gnss2Outputs->posU(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                filestream << "," << std::setprecision(floatPrecision) << obs->gnss2Outputs->velU;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                filestream << "," << std::setprecision(floatPrecision) << obs->gnss2Outputs->timeU;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeInfo.status.timeOk())
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeInfo.status.dateOk())
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeInfo.status.utcTimeValid())
                           << "," << static_cast<int>(obs->gnss2Outputs->timeInfo.leapSeconds);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                filestream << std::setprecision(floatPrecision);
                filestream << "," << obs->gnss2Outputs->dop.gDop
                           << "," << obs->gnss2Outputs->dop.pDop
                           << "," << obs->gnss2Outputs->dop.tDop
                           << "," << obs->gnss2Outputs->dop.vDop
                           << "," << obs->gnss2Outputs->dop.hDop
                           << "," << obs->gnss2Outputs->dop.nDop
                           << "," << obs->gnss2Outputs->dop.eDop;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->satInfo.numSats)
                           << ",";
                for (auto& satellite : obs->gnss2Outputs->satInfo.satellites)
                {
                    filestream << "["
                               << static_cast<int>(satellite.sys) << "|"
                               << static_cast<unsigned int>(satellite.svId) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::Healthy) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::Almanac) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) << "|"
                               << static_cast<unsigned int>(satellite.cno) << "|"
                               << static_cast<unsigned int>(satellite.qi) << "|"
                               << static_cast<int>(satellite.el) << "|"
                               << satellite.az << "]";
                }
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                filestream << "," << obs->gnss2Outputs->raw.tow
                           << "," << obs->gnss2Outputs->raw.week
                           << "," << static_cast<unsigned int>(obs->gnss2Outputs->raw.numSats)
                           << ",";
                for (auto& satellite : obs->gnss2Outputs->raw.satellites)
                {
                    filestream << "["
                               << static_cast<int>(satellite.sys) << "|"
                               << static_cast<unsigned int>(satellite.svId) << "|"
                               << static_cast<unsigned int>(satellite.freq) << "|"
                               << static_cast<unsigned int>(satellite.chan) << "|"
                               << static_cast<int>(satellite.slot) << "|"
                               << static_cast<unsigned int>(satellite.cno) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::Searching) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::Tracking) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::TimeValid) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::CodeLock) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) << "|"
                               << static_cast<unsigned int>(satellite.flags & NAV::sensors::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) << "|"
                               << std::setprecision(doublePrecision) << satellite.pr << "|"
                               << std::setprecision(doublePrecision) << satellite.cp << "|"
                               << std::setprecision(floatPrecision) << satellite.dp << "]";
                }
            }
        }

        filestream << '\n';
    }
    else // if (fileType == FileType::BINARY)
    {
        std::array<const char, 8> zeroData{};
        if (!headerWritten)
        {
            filestream.write(obs->timeOutputs ? reinterpret_cast<const char*>(&obs->timeOutputs->timeField) : zeroData.data(), sizeof(vn::protocol::uart::TimeGroup));
            filestream.write(obs->imuOutputs ? reinterpret_cast<const char*>(&obs->imuOutputs->imuField) : zeroData.data(), sizeof(vn::protocol::uart::ImuGroup));
            filestream.write(obs->gnss1Outputs ? reinterpret_cast<const char*>(&obs->gnss1Outputs->gnssField) : zeroData.data(), sizeof(vn::protocol::uart::GpsGroup));
            filestream.write(obs->attitudeOutputs ? reinterpret_cast<const char*>(&obs->attitudeOutputs->attitudeField) : zeroData.data(), sizeof(vn::protocol::uart::AttitudeGroup));
            filestream.write(obs->insOutputs ? reinterpret_cast<const char*>(&obs->insOutputs->insField) : zeroData.data(), sizeof(vn::protocol::uart::InsGroup));
            filestream.write(obs->gnss2Outputs ? reinterpret_cast<const char*>(&obs->gnss2Outputs->gnssField) : zeroData.data(), sizeof(vn::protocol::uart::GpsGroup));
            headerWritten = true;
        }

        if (obs->insTime.has_value())
        {
            auto insTimeGPS = obs->insTime->toGPSweekTow();
            filestream.write(reinterpret_cast<const char*>(&insTimeGPS.gpsCycle), sizeof(insTimeGPS.gpsCycle));
            filestream.write(reinterpret_cast<const char*>(&insTimeGPS.gpsWeek), sizeof(insTimeGPS.gpsWeek));
            filestream.write(reinterpret_cast<const char*>(&insTimeGPS.tow), sizeof(insTimeGPS.tow));
        }
        else
        {
            filestream.write(zeroData.data(), sizeof(int32_t));
            filestream.write(zeroData.data(), sizeof(int32_t));
            filestream.write(zeroData.data(), sizeof(long double));
        }

        // Group 2 (Time)
        if (obs->timeOutputs)
        {
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeStartup), sizeof(obs->timeOutputs->timeStartup));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeGps), sizeof(obs->timeOutputs->timeGps));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->gpsTow), sizeof(obs->timeOutputs->gpsTow));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->gpsWeek), sizeof(obs->timeOutputs->gpsWeek));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeSyncIn), sizeof(obs->timeOutputs->timeSyncIn));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timePPS), sizeof(obs->timeOutputs->timePPS));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.year), sizeof(obs->timeOutputs->timeUtc.year));
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.month), sizeof(obs->timeOutputs->timeUtc.month));
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.day), sizeof(obs->timeOutputs->timeUtc.day));
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.hour), sizeof(obs->timeOutputs->timeUtc.hour));
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.min), sizeof(obs->timeOutputs->timeUtc.min));
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.sec), sizeof(obs->timeOutputs->timeUtc.sec));
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.ms), sizeof(obs->timeOutputs->timeUtc.ms));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->syncInCnt), sizeof(obs->timeOutputs->syncInCnt));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->syncOutCnt), sizeof(obs->timeOutputs->syncOutCnt));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeStatus.status()), sizeof(obs->timeOutputs->timeStatus.status()));
            }
        }
        // Group 3 (IMU)
        if (obs->imuOutputs)
        {
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->imuStatus), sizeof(obs->imuOutputs->imuStatus));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->uncompMag.data()), sizeof(obs->imuOutputs->uncompMag));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->uncompAccel.data()), sizeof(obs->imuOutputs->uncompAccel));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->uncompGyro.data()), sizeof(obs->imuOutputs->uncompGyro));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->temp), sizeof(obs->imuOutputs->temp));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->pres), sizeof(obs->imuOutputs->pres));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->deltaTime), sizeof(obs->imuOutputs->deltaTime));
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->deltaTheta.data()), sizeof(obs->imuOutputs->deltaTheta));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->deltaV.data()), sizeof(obs->imuOutputs->deltaV));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->mag.data()), sizeof(obs->imuOutputs->mag));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->accel.data()), sizeof(obs->imuOutputs->accel));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->angularRate.data()), sizeof(obs->imuOutputs->angularRate));
            }
        }
        // Group 4 (GNSS1)
        if (obs->gnss1Outputs)
        {
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.year), sizeof(obs->gnss1Outputs->timeUtc.year));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.month), sizeof(obs->gnss1Outputs->timeUtc.month));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.day), sizeof(obs->gnss1Outputs->timeUtc.day));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.hour), sizeof(obs->gnss1Outputs->timeUtc.hour));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.min), sizeof(obs->gnss1Outputs->timeUtc.min));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.sec), sizeof(obs->gnss1Outputs->timeUtc.sec));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.ms), sizeof(obs->gnss1Outputs->timeUtc.ms));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->tow), sizeof(obs->gnss1Outputs->tow));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->week), sizeof(obs->gnss1Outputs->week));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->numSats), sizeof(obs->gnss1Outputs->numSats));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->fix), sizeof(obs->gnss1Outputs->fix));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->posLla.data()), sizeof(obs->gnss1Outputs->posLla));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->posEcef.data()), sizeof(obs->gnss1Outputs->posEcef));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->velNed.data()), sizeof(obs->gnss1Outputs->velNed));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->velEcef.data()), sizeof(obs->gnss1Outputs->velEcef));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->posU.data()), sizeof(obs->gnss1Outputs->posU));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->velU), sizeof(obs->gnss1Outputs->velU));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeU), sizeof(obs->gnss1Outputs->timeU));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeInfo.status.status()), sizeof(obs->gnss1Outputs->timeInfo.status.status()));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeInfo.leapSeconds), sizeof(obs->gnss1Outputs->timeInfo.leapSeconds));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.gDop), sizeof(obs->gnss1Outputs->dop.gDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.pDop), sizeof(obs->gnss1Outputs->dop.pDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.tDop), sizeof(obs->gnss1Outputs->dop.tDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.vDop), sizeof(obs->gnss1Outputs->dop.vDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.hDop), sizeof(obs->gnss1Outputs->dop.hDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.nDop), sizeof(obs->gnss1Outputs->dop.nDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.eDop), sizeof(obs->gnss1Outputs->dop.eDop));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->satInfo.numSats), sizeof(obs->gnss1Outputs->satInfo.numSats));

                for (auto& satellite : obs->gnss1Outputs->satInfo.satellites)
                {
                    filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    filestream.write(reinterpret_cast<const char*>(&satellite.qi), sizeof(satellite.qi));
                    filestream.write(reinterpret_cast<const char*>(&satellite.el), sizeof(satellite.el));
                    filestream.write(reinterpret_cast<const char*>(&satellite.az), sizeof(satellite.az));
                }
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->raw.tow), sizeof(obs->gnss1Outputs->raw.tow));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->raw.week), sizeof(obs->gnss1Outputs->raw.week));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->raw.numSats), sizeof(obs->gnss1Outputs->raw.numSats));

                for (auto& satellite : obs->gnss1Outputs->raw.satellites)
                {
                    filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    filestream.write(reinterpret_cast<const char*>(&satellite.freq), sizeof(satellite.freq));
                    filestream.write(reinterpret_cast<const char*>(&satellite.chan), sizeof(satellite.chan));
                    filestream.write(reinterpret_cast<const char*>(&satellite.slot), sizeof(satellite.slot));
                    filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    filestream.write(reinterpret_cast<const char*>(&satellite.pr), sizeof(satellite.pr));
                    filestream.write(reinterpret_cast<const char*>(&satellite.cp), sizeof(satellite.cp));
                    filestream.write(reinterpret_cast<const char*>(&satellite.dp), sizeof(satellite.dp));
                }
            }
        }
        // Group 5 (Attitude)
        if (obs->attitudeOutputs)
        {
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->attitudeOutputs->vpeStatus.status()), sizeof(obs->attitudeOutputs->vpeStatus.status()));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->ypr.data()), sizeof(obs->attitudeOutputs->ypr));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->qtn.coeffs().data()), sizeof(obs->attitudeOutputs->qtn));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->dcm.data()), sizeof(obs->attitudeOutputs->dcm));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->magNed.data()), sizeof(obs->attitudeOutputs->magNed));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->accelNed.data()), sizeof(obs->attitudeOutputs->accelNed));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->linearAccelBody.data()), sizeof(obs->attitudeOutputs->linearAccelBody));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->linearAccelNed.data()), sizeof(obs->attitudeOutputs->linearAccelNed));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
            {
                filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->yprU.data()), sizeof(obs->attitudeOutputs->yprU));
            }
        }
        // Group 6 (INS)
        if (obs->insOutputs)
        {
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->insOutputs->insStatus.status()), sizeof(obs->insOutputs->insStatus.status()));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->posLla.data()), sizeof(obs->insOutputs->posLla));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->posEcef.data()), sizeof(obs->insOutputs->posEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->velBody.data()), sizeof(obs->insOutputs->velBody));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->velNed.data()), sizeof(obs->insOutputs->velNed));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->velEcef.data()), sizeof(obs->insOutputs->velEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->magEcef.data()), sizeof(obs->insOutputs->magEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->accelEcef.data()), sizeof(obs->insOutputs->accelEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->insOutputs->linearAccelEcef.data()), sizeof(obs->insOutputs->linearAccelEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->insOutputs->posU), sizeof(obs->insOutputs->posU));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->insOutputs->velU), sizeof(obs->insOutputs->velU));
            }
        }
        // Group 7 (GNSS2)
        if (obs->gnss2Outputs)
        {
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.year), sizeof(obs->gnss2Outputs->timeUtc.year));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.month), sizeof(obs->gnss2Outputs->timeUtc.month));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.day), sizeof(obs->gnss2Outputs->timeUtc.day));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.hour), sizeof(obs->gnss2Outputs->timeUtc.hour));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.min), sizeof(obs->gnss2Outputs->timeUtc.min));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.sec), sizeof(obs->gnss2Outputs->timeUtc.sec));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.ms), sizeof(obs->gnss2Outputs->timeUtc.ms));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->tow), sizeof(obs->gnss2Outputs->tow));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->week), sizeof(obs->gnss2Outputs->week));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->numSats), sizeof(obs->gnss2Outputs->numSats));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->fix), sizeof(obs->gnss2Outputs->fix));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->posLla.data()), sizeof(obs->gnss2Outputs->posLla));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->posEcef.data()), sizeof(obs->gnss2Outputs->posEcef));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->velNed.data()), sizeof(obs->gnss2Outputs->velNed));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->velEcef.data()), sizeof(obs->gnss2Outputs->velEcef));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->posU.data()), sizeof(obs->gnss2Outputs->posU));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->velU), sizeof(obs->gnss2Outputs->velU));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeU), sizeof(obs->gnss2Outputs->timeU));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeInfo.status.status()), sizeof(obs->gnss2Outputs->timeInfo.status.status()));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeInfo.leapSeconds), sizeof(obs->gnss2Outputs->timeInfo.leapSeconds));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.gDop), sizeof(obs->gnss2Outputs->dop.gDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.pDop), sizeof(obs->gnss2Outputs->dop.pDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.tDop), sizeof(obs->gnss2Outputs->dop.tDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.vDop), sizeof(obs->gnss2Outputs->dop.vDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.hDop), sizeof(obs->gnss2Outputs->dop.hDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.nDop), sizeof(obs->gnss2Outputs->dop.nDop));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.eDop), sizeof(obs->gnss2Outputs->dop.eDop));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->satInfo.numSats), sizeof(obs->gnss2Outputs->satInfo.numSats));

                for (auto& satellite : obs->gnss2Outputs->satInfo.satellites)
                {
                    filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    filestream.write(reinterpret_cast<const char*>(&satellite.qi), sizeof(satellite.qi));
                    filestream.write(reinterpret_cast<const char*>(&satellite.el), sizeof(satellite.el));
                    filestream.write(reinterpret_cast<const char*>(&satellite.az), sizeof(satellite.az));
                }
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->raw.tow), sizeof(obs->gnss2Outputs->raw.tow));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->raw.week), sizeof(obs->gnss2Outputs->raw.week));
                filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->raw.numSats), sizeof(obs->gnss2Outputs->raw.numSats));

                for (auto& satellite : obs->gnss2Outputs->raw.satellites)
                {
                    filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    filestream.write(reinterpret_cast<const char*>(&satellite.freq), sizeof(satellite.freq));
                    filestream.write(reinterpret_cast<const char*>(&satellite.chan), sizeof(satellite.chan));
                    filestream.write(reinterpret_cast<const char*>(&satellite.slot), sizeof(satellite.slot));
                    filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    filestream.write(reinterpret_cast<const char*>(&satellite.pr), sizeof(satellite.pr));
                    filestream.write(reinterpret_cast<const char*>(&satellite.cp), sizeof(satellite.cp));
                    filestream.write(reinterpret_cast<const char*>(&satellite.dp), sizeof(satellite.dp));
                }
            }
        }
    }

    LOG_DATA("{}: Message logged", nameId());
}