#include "VectorNavDataLogger.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

#include <iomanip> // std::setprecision
#include <limits>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include <imgui_internal.h>

NAV::VectorNavDataLogger::VectorNavDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::BINARY;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 444, 92 };

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
    if (FileWriter::guiConfig(_fileType == FileType::CSV ? ".csv" : ".vnb", { _fileType == FileType::CSV ? ".csv" : ".vnb" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        nm::DeinitializeNode(*this);
    }

    static constexpr std::array<FileType, 2> fileTypes = {
        { FileType::CSV,
          FileType::BINARY }
    };
    if (ImGui::BeginCombo(fmt::format("Mode##{}", size_t(id)).c_str(), FileWriter::to_string(_fileType)))
    {
        for (const auto& type : fileTypes)
        {
            const bool isSelected = (_fileType == type);
            if (ImGui::Selectable(to_string(type), isSelected))
            {
                _fileType = type;
                LOG_DEBUG("{}: _fileType changed to {}", nameId(), FileWriter::to_string(_fileType));
                str::replace(_path, _fileType == FileType::CSV ? ".vnb" : ".csv", _fileType == FileType::CSV ? ".csv" : ".vnb");
                flow::ApplyChanges();
                if (getState() == State::Initialized)
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

    if (getState() == State::Initialized)
    {
        deinitialize();
        initialize();
    }

    return true;
}

void NAV::VectorNavDataLogger::flush()
{
    _filestream.flush();
}

bool NAV::VectorNavDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _headerWritten = false;

    return true;
}

void NAV::VectorNavDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::VectorNavDataLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<const VectorNavBinaryOutput>(nodeData);

    if (_fileType == FileType::CSV)
    {
        if (!_headerWritten)
        {
            _filestream << "Time [s],GpsCycle,GpsWeek,GpsTow";

            // Group 2 (Time)
            if (obs->timeOutputs)
            {
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
                {
                    _filestream << ",Time::TimeStartup";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
                {
                    _filestream << ",Time::TimeGps";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
                {
                    _filestream << ",Time::GpsTow";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
                {
                    _filestream << ",Time::GpsWeek";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
                {
                    _filestream << ",Time::TimeSyncIn";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
                {
                    _filestream << ",Time::TimeGpsPps";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
                {
                    _filestream << ",Time::TimeUTC::year,Time::TimeUTC::month,Time::TimeUTC::day,Time::TimeUTC::hour,Time::TimeUTC::min,Time::TimeUTC::sec,Time::TimeUTC::ms";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
                {
                    _filestream << ",Time::SyncInCnt";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
                {
                    _filestream << ",Time::SyncOutCnt";
                }
                if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
                {
                    _filestream << ",Time::TimeStatus::timeOk,Time::TimeStatus::dateOk,Time::TimeStatus::utcTimeValid";
                }
            }
            // Group 3 (IMU)
            if (obs->imuOutputs)
            {
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
                {
                    _filestream << ",IMU::ImuStatus";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
                {
                    _filestream << ",IMU::UncompMag::X,IMU::UncompMag::Y,IMU::UncompMag::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
                {
                    _filestream << ",IMU::UncompAccel::X,IMU::UncompAccel::Y,IMU::UncompAccel::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
                {
                    _filestream << ",IMU::UncompGyro::X,IMU::UncompGyro::Y,IMU::UncompGyro::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
                {
                    _filestream << ",IMU::Temp";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
                {
                    _filestream << ",IMU::Pres";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
                {
                    _filestream << ",IMU::DeltaTime,IMU::DeltaTheta::X,IMU::DeltaTheta::Y,IMU::DeltaTheta::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
                {
                    _filestream << ",IMU::DeltaVel::X,IMU::DeltaVel::Y,IMU::DeltaVel::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
                {
                    _filestream << ",IMU::Mag::X,IMU::Mag::Y,IMU::Mag::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
                {
                    _filestream << ",IMU::Accel::X,IMU::Accel::Y,IMU::Accel::Z";
                }
                if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
                {
                    _filestream << ",IMU::AngularRate::X,IMU::AngularRate::Y,IMU::AngularRate::Z";
                }
            }
            // Group 4 (GNSS1)
            if (obs->gnss1Outputs)
            {
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    _filestream << ",GNSS1::UTC::year,GNSS1::UTC::month,GNSS1::UTC::day,GNSS1::UTC::hour,GNSS1::UTC::min,GNSS1::UTC::sec,GNSS1::UTC::ms";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    _filestream << ",GNSS1::Tow";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    _filestream << ",GNSS1::Week";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    _filestream << ",GNSS1::NumSats";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    _filestream << ",GNSS1::Fix";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    _filestream << ",GNSS1::PosLla::latitude,GNSS1::PosLla::longitude,GNSS1::PosLla::altitude";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    _filestream << ",GNSS1::PosEcef::X,GNSS1::PosEcef::Y,GNSS1::PosEcef::Z";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    _filestream << ",GNSS1::VelNed::N,GNSS1::VelNed::E,GNSS1::VelNed::D";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    _filestream << ",GNSS1::VelEcef::X,GNSS1::VelEcef::Y,GNSS1::VelEcef::Z";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    _filestream << ",GNSS1::PosU::N,GNSS1::PosU::E,GNSS1::PosU::D";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    _filestream << ",GNSS1::VelU";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    _filestream << ",GNSS1::TimeU";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    _filestream << ",GNSS1::TimeInfo::Status::timeOk,GNSS1::TimeInfo::Status::dateOk,GNSS1::TimeInfo::Status::utcTimeValid,GNSS1::TimeInfo::LeapSeconds";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    _filestream << ",GNSS1::DOP::g,GNSS1::DOP::p,GNSS1::DOP::t,GNSS1::DOP::v,GNSS1::DOP::h,GNSS1::DOP::n,GNSS1::DOP::e";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    _filestream << ",GNSS1::SatInfo::NumSats,GNSS1::SatInfo::Satellites";
                }
                if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    _filestream << ",GNSS1::RawMeas::Tow,GNSS1::RawMeas::Week,GNSS1::RawMeas::NumSats,GNSS1::RawMeas::Satellites";
                }
            }
            // Group 5 (Attitude)
            if (obs->attitudeOutputs)
            {
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
                {
                    _filestream << ",Att::VpeStatus::AttitudeQuality"
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
                    _filestream << ",Att::YawPitchRoll::Y,Att::YawPitchRoll::P,Att::YawPitchRoll::R";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
                {
                    _filestream << ",Att::Quaternion::w,Att::Quaternion::x,Att::Quaternion::y,Att::Quaternion::z";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
                {
                    _filestream << ",Att::DCM::0-0,Att::DCM::0-1,Att::DCM::0-2"
                                   ",Att::DCM::1-0,Att::DCM::1-1,Att::DCM::1-2"
                                   ",Att::DCM::2-0,Att::DCM::2-1,Att::DCM::2-2";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
                {
                    _filestream << ",Att::MagNed::N,Att::MagNed::E,Att::MagNed::D";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
                {
                    _filestream << ",Att::AccelNed::N,Att::AccelNed::E,Att::AccelNed::D";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
                {
                    _filestream << ",Att::LinearAccelBody::X,Att::LinearAccelBody::Y,Att::LinearAccelBody::Z";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
                {
                    _filestream << ",Att::LinearAccelNed::N,Att::LinearAccelNed::E,Att::LinearAccelNed::D";
                }
                if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
                {
                    _filestream << ",Att::YprU::Y,Att::YprU::P,Att::YprU::R";
                }
            }
            // Group 6 (INS)
            if (obs->insOutputs)
            {
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
                {
                    _filestream << ",INS::InsStatus::Mode"
                                   ",INS::InsStatus::GpsFix"
                                   ",INS::InsStatus::Error::IMU"
                                   ",INS::InsStatus::Error::MagPres"
                                   ",INS::InsStatus::Error::GNSS"
                                   ",INS::InsStatus::GpsHeadingIns"
                                   ",INS::InsStatus::GpsCompass";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
                {
                    _filestream << ",INS::PosLla::latitude,INS::PosLla::longitude,INS::PosLla::altitude";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
                {
                    _filestream << ",INS::PosEcef::X,INS::PosEcef::Y,INS::PosEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                {
                    _filestream << ",INS::VelBody::X,INS::VelBody::Y,INS::VelBody::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
                {
                    _filestream << ",INS::VelNed::N,INS::VelNed::E,INS::VelNed::D";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                {
                    _filestream << ",INS::VelEcef::X,INS::VelEcef::Y,INS::VelEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
                {
                    _filestream << ",INS::MagEcef::X,INS::MagEcef::Y,INS::MagEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
                {
                    _filestream << ",INS::AccelEcef::X,INS::AccelEcef::Y,INS::AccelEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
                {
                    _filestream << ",INS::LinearAccelEcef::X,INS::LinearAccelEcef::Y,INS::LinearAccelEcef::Z";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
                {
                    _filestream << ",INS::PosU";
                }
                if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
                {
                    _filestream << ",INS::VelU";
                }
            }
            // Group 7 (GNSS2)
            if (obs->gnss2Outputs)
            {
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
                {
                    _filestream << ",GNSS2::UTC::year,GNSS2::UTC::month,GNSS2::UTC::day,GNSS2::UTC::hour,GNSS2::UTC::min,GNSS2::UTC::sec,GNSS2::UTC::ms";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
                {
                    _filestream << ",GNSS2::Tow";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
                {
                    _filestream << ",GNSS2::Week";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
                {
                    _filestream << ",GNSS2::NumSats";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
                {
                    _filestream << ",GNSS2::Fix";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
                {
                    _filestream << ",GNSS2::PosLla::latitude,GNSS2::PosLla::longitude,GNSS2::PosLla::altitude";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
                {
                    _filestream << ",GNSS2::PosEcef::X,GNSS2::PosEcef::Y,GNSS2::PosEcef::Z";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
                {
                    _filestream << ",GNSS2::VelNed::N,GNSS2::VelNed::E,GNSS2::VelNed::D";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                {
                    _filestream << ",GNSS2::VelEcef::X,GNSS2::VelEcef::Y,GNSS2::VelEcef::Z";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
                {
                    _filestream << ",GNSS2::PosU::N,GNSS2::PosU::E,GNSS2::PosU::D";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
                {
                    _filestream << ",GNSS2::VelU";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
                {
                    _filestream << ",GNSS2::TimeU";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
                {
                    _filestream << ",GNSS2::TimeInfo::Status::timeOk,GNSS2::TimeInfo::Status::dateOk,GNSS2::TimeInfo::Status::utcTimeValid,GNSS2::TimeInfo::LeapSeconds";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
                {
                    _filestream << ",GNSS2::DOP::g,GNSS2::DOP::p,GNSS2::DOP::t,GNSS2::DOP::v,GNSS2::DOP::h,GNSS2::DOP::n,GNSS2::DOP::e";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
                {
                    _filestream << ",GNSS2::SatInfo::NumSats,GNSS2::SatInfo::Satellites";
                }
                if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
                {
                    _filestream << ",GNSS2::RawMeas::Tow,GNSS2::RawMeas::Week,GNSS2::RawMeas::NumSats,GNSS2::RawMeas::Satellites";
                }
            }

            _filestream << std::endl;
            _headerWritten = true;
        }

        constexpr int gpsCyclePrecision = 3;
        constexpr int gpsTimePrecision = 12;
        constexpr int floatPrecision = std::numeric_limits<float>::digits10 + 2;
        constexpr int doublePrecision = std::numeric_limits<double>::digits10 + 2;

        if (obs->insTime.has_value())
        {
            _filestream << std::setprecision(doublePrecision) << std::round(calcTimeIntoRun(obs->insTime.value()) * 1e9) / 1e9;
        }
        _filestream << ",";
        if (obs->insTime.has_value())
        {
            _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime->toGPSweekTow().gpsCycle;
        }
        _filestream << ',';
        if (obs->insTime.has_value())
        {
            _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime->toGPSweekTow().gpsWeek;
        }
        _filestream << ',';
        if (obs->insTime.has_value())
        {
            _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime->toGPSweekTow().tow;
        }
        // Group 2 (Time)
        if (obs->timeOutputs)
        {
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
            {
                _filestream << "," << obs->timeOutputs->timeStartup;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
            {
                _filestream << "," << obs->timeOutputs->timeGps;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
            {
                _filestream << "," << obs->timeOutputs->gpsTow;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
            {
                _filestream << "," << obs->timeOutputs->gpsWeek;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
            {
                _filestream << "," << obs->timeOutputs->timeSyncIn;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
            {
                _filestream << "," << obs->timeOutputs->timePPS;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
            {
                _filestream << "," << static_cast<int>(obs->timeOutputs->timeUtc.year)
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.month)
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.day)
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.hour)
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.min)
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeUtc.sec)
                            << "," << obs->timeOutputs->timeUtc.ms;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
            {
                _filestream << "," << obs->timeOutputs->syncInCnt;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
            {
                _filestream << "," << obs->timeOutputs->syncOutCnt;
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
            {
                _filestream << "," << static_cast<unsigned int>(obs->timeOutputs->timeStatus.timeOk())
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeStatus.dateOk())
                            << "," << static_cast<unsigned int>(obs->timeOutputs->timeStatus.utcTimeValid());
            }
        }
        // Group 3 (IMU)
        if (obs->imuOutputs)
        {
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
            {
                _filestream << "," << obs->imuOutputs->imuStatus;
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->uncompMag(0)
                            << "," << obs->imuOutputs->uncompMag(1)
                            << "," << obs->imuOutputs->uncompMag(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->uncompAccel(0)
                            << "," << obs->imuOutputs->uncompAccel(1)
                            << "," << obs->imuOutputs->uncompAccel(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->uncompGyro(0)
                            << "," << obs->imuOutputs->uncompGyro(1)
                            << "," << obs->imuOutputs->uncompGyro(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                _filestream << "," << std::setprecision(floatPrecision) << obs->imuOutputs->temp;
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                _filestream << "," << std::setprecision(floatPrecision) << obs->imuOutputs->pres;
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->deltaTime
                            << "," << obs->imuOutputs->deltaTheta(0)
                            << "," << obs->imuOutputs->deltaTheta(1)
                            << "," << obs->imuOutputs->deltaTheta(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->deltaV(0)
                            << "," << obs->imuOutputs->deltaV(1)
                            << "," << obs->imuOutputs->deltaV(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->mag(0)
                            << "," << obs->imuOutputs->mag(1)
                            << "," << obs->imuOutputs->mag(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->accel(0)
                            << "," << obs->imuOutputs->accel(1)
                            << "," << obs->imuOutputs->accel(2);
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->imuOutputs->angularRate(0)
                            << "," << obs->imuOutputs->angularRate(1)
                            << "," << obs->imuOutputs->angularRate(2);
            }
        }
        // Group 4 (GNSS1)
        if (obs->gnss1Outputs)
        {
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                _filestream << "," << static_cast<int>(obs->gnss1Outputs->timeUtc.year)
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.month)
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.day)
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.hour)
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.min)
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeUtc.sec)
                            << "," << obs->gnss1Outputs->timeUtc.ms;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                _filestream << "," << obs->gnss1Outputs->tow;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                _filestream << "," << obs->gnss1Outputs->week;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->numSats);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->fix);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                _filestream << std::setprecision(doublePrecision);
                _filestream << "," << obs->gnss1Outputs->posLla(0)
                            << "," << obs->gnss1Outputs->posLla(1)
                            << "," << obs->gnss1Outputs->posLla(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                _filestream << std::setprecision(doublePrecision);
                _filestream << "," << obs->gnss1Outputs->posEcef(0)
                            << "," << obs->gnss1Outputs->posEcef(1)
                            << "," << obs->gnss1Outputs->posEcef(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss1Outputs->velNed(0)
                            << "," << obs->gnss1Outputs->velNed(1)
                            << "," << obs->gnss1Outputs->velNed(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss1Outputs->velEcef(0)
                            << "," << obs->gnss1Outputs->velEcef(1)
                            << "," << obs->gnss1Outputs->velEcef(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss1Outputs->posU(0)
                            << "," << obs->gnss1Outputs->posU(1)
                            << "," << obs->gnss1Outputs->posU(2);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                _filestream << "," << std::setprecision(floatPrecision) << obs->gnss1Outputs->velU;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                _filestream << "," << std::setprecision(floatPrecision) << obs->gnss1Outputs->timeU;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeInfo.status.timeOk())
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeInfo.status.dateOk())
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->timeInfo.status.utcTimeValid())
                            << "," << static_cast<int>(obs->gnss1Outputs->timeInfo.leapSeconds);
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss1Outputs->dop.gDop
                            << "," << obs->gnss1Outputs->dop.pDop
                            << "," << obs->gnss1Outputs->dop.tDop
                            << "," << obs->gnss1Outputs->dop.vDop
                            << "," << obs->gnss1Outputs->dop.hDop
                            << "," << obs->gnss1Outputs->dop.nDop
                            << "," << obs->gnss1Outputs->dop.eDop;
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss1Outputs->satInfo.numSats)
                            << ",";
                for (auto& satellite : obs->gnss1Outputs->satInfo.satellites)
                {
                    _filestream << "["
                                << static_cast<int>(satellite.sys) << "|"
                                << static_cast<unsigned int>(satellite.svId) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Healthy) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Almanac) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) ? 1 : 0) << "|"
                                << static_cast<unsigned int>(satellite.cno) << "|"
                                << static_cast<unsigned int>(satellite.qi) << "|"
                                << static_cast<int>(satellite.el) << "|"
                                << satellite.az << "]";
                }
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                _filestream << std::setprecision(gpsTimePrecision);
                _filestream << "," << obs->gnss1Outputs->raw.tow
                            << "," << obs->gnss1Outputs->raw.week
                            << "," << static_cast<unsigned int>(obs->gnss1Outputs->raw.numSats)
                            << ",";
                for (auto& satellite : obs->gnss1Outputs->raw.satellites)
                {
                    _filestream << "["
                                << static_cast<int>(satellite.sys) << "|"
                                << static_cast<unsigned int>(satellite.svId) << "|"
                                << static_cast<unsigned int>(satellite.freq) << "|"
                                << static_cast<unsigned int>(satellite.chan) << "|"
                                << static_cast<int>(satellite.slot) << "|"
                                << static_cast<unsigned int>(satellite.cno) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Searching) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Tracking) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::TimeValid) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::CodeLock) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) ? 1 : 0) << "|"
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
                _filestream << "," << static_cast<unsigned int>(obs->attitudeOutputs->vpeStatus.attitudeQuality())
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
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << std::setprecision(floatPrecision) << obs->attitudeOutputs->ypr(0)
                            << "," << obs->attitudeOutputs->ypr(1)
                            << "," << obs->attitudeOutputs->ypr(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->qtn.w()
                            << "," << obs->attitudeOutputs->qtn.x()
                            << "," << obs->attitudeOutputs->qtn.y()
                            << "," << obs->attitudeOutputs->qtn.z();
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->dcm(0, 0)
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
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->magNed(0)
                            << "," << obs->attitudeOutputs->magNed(1)
                            << "," << obs->attitudeOutputs->magNed(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->accelNed(0)
                            << "," << obs->attitudeOutputs->accelNed(1)
                            << "," << obs->attitudeOutputs->accelNed(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->linearAccelBody(0)
                            << "," << obs->attitudeOutputs->linearAccelBody(1)
                            << "," << obs->attitudeOutputs->linearAccelBody(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->linearAccelNed(0)
                            << "," << obs->attitudeOutputs->linearAccelNed(1)
                            << "," << obs->attitudeOutputs->linearAccelNed(2);
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->attitudeOutputs->yprU(0)
                            << "," << obs->attitudeOutputs->yprU(1)
                            << "," << obs->attitudeOutputs->yprU(2);
            }
        }
        // Group 6 (INS)
        if (obs->insOutputs)
        {
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
            {
                _filestream << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.mode())
                            << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.gpsFix())
                            << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.errorIMU())
                            << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.errorMagPres())
                            << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.errorGnss())
                            << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.gpsHeadingIns())
                            << "," << static_cast<unsigned int>(obs->insOutputs->insStatus.gpsCompass());
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
            {
                _filestream << std::setprecision(doublePrecision);
                _filestream << "," << obs->insOutputs->posLla(0)
                            << "," << obs->insOutputs->posLla(1)
                            << "," << obs->insOutputs->posLla(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
            {
                _filestream << std::setprecision(doublePrecision);
                _filestream << "," << obs->insOutputs->posEcef(0)
                            << "," << obs->insOutputs->posEcef(1)
                            << "," << obs->insOutputs->posEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->velBody(0)
                            << "," << obs->insOutputs->velBody(1)
                            << "," << obs->insOutputs->velBody(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->velNed(0)
                            << "," << obs->insOutputs->velNed(1)
                            << "," << obs->insOutputs->velNed(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->velEcef(0)
                            << "," << obs->insOutputs->velEcef(1)
                            << "," << obs->insOutputs->velEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->magEcef(0)
                            << "," << obs->insOutputs->magEcef(1)
                            << "," << obs->insOutputs->magEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->accelEcef(0)
                            << "," << obs->insOutputs->accelEcef(1)
                            << "," << obs->insOutputs->accelEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->linearAccelEcef(0)
                            << "," << obs->insOutputs->linearAccelEcef(1)
                            << "," << obs->insOutputs->linearAccelEcef(2);
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->posU;
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->insOutputs->velU;
            }
        }
        // Group 7 (GNSS2)
        if (obs->gnss2Outputs)
        {
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                _filestream << "," << static_cast<int>(obs->gnss2Outputs->timeUtc.year)
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.month)
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.day)
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.hour)
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.min)
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeUtc.sec)
                            << "," << obs->gnss2Outputs->timeUtc.ms;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                _filestream << "," << obs->gnss2Outputs->tow;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                _filestream << "," << obs->gnss2Outputs->week;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->numSats);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->fix);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                _filestream << std::setprecision(doublePrecision);
                _filestream << "," << obs->gnss2Outputs->posLla(0)
                            << "," << obs->gnss2Outputs->posLla(1)
                            << "," << obs->gnss2Outputs->posLla(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                _filestream << std::setprecision(doublePrecision);
                _filestream << "," << obs->gnss2Outputs->posEcef(0)
                            << "," << obs->gnss2Outputs->posEcef(1)
                            << "," << obs->gnss2Outputs->posEcef(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss2Outputs->velNed(0)
                            << "," << obs->gnss2Outputs->velNed(1)
                            << "," << obs->gnss2Outputs->velNed(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss2Outputs->velEcef(0)
                            << "," << obs->gnss2Outputs->velEcef(1)
                            << "," << obs->gnss2Outputs->velEcef(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss2Outputs->posU(0)
                            << "," << obs->gnss2Outputs->posU(1)
                            << "," << obs->gnss2Outputs->posU(2);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                _filestream << "," << std::setprecision(floatPrecision) << obs->gnss2Outputs->velU;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                _filestream << "," << std::setprecision(floatPrecision) << obs->gnss2Outputs->timeU;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeInfo.status.timeOk())
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeInfo.status.dateOk())
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->timeInfo.status.utcTimeValid())
                            << "," << static_cast<int>(obs->gnss2Outputs->timeInfo.leapSeconds);
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                _filestream << std::setprecision(floatPrecision);
                _filestream << "," << obs->gnss2Outputs->dop.gDop
                            << "," << obs->gnss2Outputs->dop.pDop
                            << "," << obs->gnss2Outputs->dop.tDop
                            << "," << obs->gnss2Outputs->dop.vDop
                            << "," << obs->gnss2Outputs->dop.hDop
                            << "," << obs->gnss2Outputs->dop.nDop
                            << "," << obs->gnss2Outputs->dop.eDop;
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                _filestream << "," << static_cast<unsigned int>(obs->gnss2Outputs->satInfo.numSats)
                            << ",";
                for (auto& satellite : obs->gnss2Outputs->satInfo.satellites)
                {
                    _filestream << "["
                                << static_cast<int>(satellite.sys) << "|"
                                << static_cast<unsigned int>(satellite.svId) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Healthy) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Almanac) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) ? 1 : 0) << "|"
                                << static_cast<unsigned int>(satellite.cno) << "|"
                                << static_cast<unsigned int>(satellite.qi) << "|"
                                << static_cast<int>(satellite.el) << "|"
                                << satellite.az << "]";
                }
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                _filestream << std::setprecision(gpsTimePrecision);
                _filestream << "," << obs->gnss2Outputs->raw.tow
                            << "," << obs->gnss2Outputs->raw.week
                            << "," << static_cast<unsigned int>(obs->gnss2Outputs->raw.numSats)
                            << ",";
                for (auto& satellite : obs->gnss2Outputs->raw.satellites)
                {
                    _filestream << "["
                                << static_cast<int>(satellite.sys) << "|"
                                << static_cast<unsigned int>(satellite.svId) << "|"
                                << static_cast<unsigned int>(satellite.freq) << "|"
                                << static_cast<unsigned int>(satellite.chan) << "|"
                                << static_cast<int>(satellite.slot) << "|"
                                << static_cast<unsigned int>(satellite.cno) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Searching) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Tracking) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::TimeValid) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::CodeLock) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) ? 1 : 0) << "|"
                                << (static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) ? 1 : 0) << "|"
                                << std::setprecision(doublePrecision) << satellite.pr << "|"
                                << std::setprecision(doublePrecision) << satellite.cp << "|"
                                << std::setprecision(floatPrecision) << satellite.dp << "]";
                }
            }
        }

        _filestream << '\n';
    }
    else // if (_fileType == FileType::BINARY)
    {
        std::array<const char, 8> zeroData{};
        if (!_headerWritten)
        {
            _filestream.write(obs->timeOutputs ? reinterpret_cast<const char*>(&obs->timeOutputs->timeField) : zeroData.data(), sizeof(vn::protocol::uart::TimeGroup));
            _filestream.write(obs->imuOutputs ? reinterpret_cast<const char*>(&obs->imuOutputs->imuField) : zeroData.data(), sizeof(vn::protocol::uart::ImuGroup));
            _filestream.write(obs->gnss1Outputs ? reinterpret_cast<const char*>(&obs->gnss1Outputs->gnssField) : zeroData.data(), sizeof(vn::protocol::uart::GpsGroup));
            _filestream.write(obs->attitudeOutputs ? reinterpret_cast<const char*>(&obs->attitudeOutputs->attitudeField) : zeroData.data(), sizeof(vn::protocol::uart::AttitudeGroup));
            _filestream.write(obs->insOutputs ? reinterpret_cast<const char*>(&obs->insOutputs->insField) : zeroData.data(), sizeof(vn::protocol::uart::InsGroup));
            _filestream.write(obs->gnss2Outputs ? reinterpret_cast<const char*>(&obs->gnss2Outputs->gnssField) : zeroData.data(), sizeof(vn::protocol::uart::GpsGroup));
            _headerWritten = true;
        }

        if (obs->insTime.has_value())
        {
            auto insTimeGPS = obs->insTime->toGPSweekTow();
            auto tow = static_cast<double>(insTimeGPS.tow);
            _filestream.write(reinterpret_cast<const char*>(&insTimeGPS.gpsCycle), sizeof(insTimeGPS.gpsCycle));
            _filestream.write(reinterpret_cast<const char*>(&insTimeGPS.gpsWeek), sizeof(insTimeGPS.gpsWeek));
            _filestream.write(reinterpret_cast<const char*>(&tow), sizeof(tow));
        }
        else
        {
            _filestream.write(zeroData.data(), sizeof(int32_t));
            _filestream.write(zeroData.data(), sizeof(int32_t));
            _filestream.write(zeroData.data(), sizeof(double));
        }

        // Group 2 (Time)
        if (obs->timeOutputs)
        {
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeStartup), sizeof(obs->timeOutputs->timeStartup));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeGps), sizeof(obs->timeOutputs->timeGps));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->gpsTow), sizeof(obs->timeOutputs->gpsTow));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->gpsWeek), sizeof(obs->timeOutputs->gpsWeek));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeSyncIn), sizeof(obs->timeOutputs->timeSyncIn));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timePPS), sizeof(obs->timeOutputs->timePPS));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.year), sizeof(obs->timeOutputs->timeUtc.year));
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.month), sizeof(obs->timeOutputs->timeUtc.month));
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.day), sizeof(obs->timeOutputs->timeUtc.day));
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.hour), sizeof(obs->timeOutputs->timeUtc.hour));
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.min), sizeof(obs->timeOutputs->timeUtc.min));
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.sec), sizeof(obs->timeOutputs->timeUtc.sec));
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeUtc.ms), sizeof(obs->timeOutputs->timeUtc.ms));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->syncInCnt), sizeof(obs->timeOutputs->syncInCnt));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->syncOutCnt), sizeof(obs->timeOutputs->syncOutCnt));
            }
            if (obs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->timeOutputs->timeStatus.status()), sizeof(obs->timeOutputs->timeStatus.status()));
            }
        }
        // Group 3 (IMU)
        if (obs->imuOutputs)
        {
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->imuStatus), sizeof(obs->imuOutputs->imuStatus));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->uncompMag.data()), sizeof(obs->imuOutputs->uncompMag));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->uncompAccel.data()), sizeof(obs->imuOutputs->uncompAccel));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->uncompGyro.data()), sizeof(obs->imuOutputs->uncompGyro));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->temp), sizeof(obs->imuOutputs->temp));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->pres), sizeof(obs->imuOutputs->pres));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->imuOutputs->deltaTime), sizeof(obs->imuOutputs->deltaTime));
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->deltaTheta.data()), sizeof(obs->imuOutputs->deltaTheta));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->deltaV.data()), sizeof(obs->imuOutputs->deltaV));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->mag.data()), sizeof(obs->imuOutputs->mag));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->accel.data()), sizeof(obs->imuOutputs->accel));
            }
            if (obs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->imuOutputs->angularRate.data()), sizeof(obs->imuOutputs->angularRate));
            }
        }
        // Group 4 (GNSS1)
        if (obs->gnss1Outputs)
        {
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.year), sizeof(obs->gnss1Outputs->timeUtc.year));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.month), sizeof(obs->gnss1Outputs->timeUtc.month));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.day), sizeof(obs->gnss1Outputs->timeUtc.day));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.hour), sizeof(obs->gnss1Outputs->timeUtc.hour));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.min), sizeof(obs->gnss1Outputs->timeUtc.min));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.sec), sizeof(obs->gnss1Outputs->timeUtc.sec));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeUtc.ms), sizeof(obs->gnss1Outputs->timeUtc.ms));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->tow), sizeof(obs->gnss1Outputs->tow));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->week), sizeof(obs->gnss1Outputs->week));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->numSats), sizeof(obs->gnss1Outputs->numSats));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->fix), sizeof(obs->gnss1Outputs->fix));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->posLla.data()), sizeof(obs->gnss1Outputs->posLla));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->posEcef.data()), sizeof(obs->gnss1Outputs->posEcef));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->velNed.data()), sizeof(obs->gnss1Outputs->velNed));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->velEcef.data()), sizeof(obs->gnss1Outputs->velEcef));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss1Outputs->posU.data()), sizeof(obs->gnss1Outputs->posU));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->velU), sizeof(obs->gnss1Outputs->velU));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeU), sizeof(obs->gnss1Outputs->timeU));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeInfo.status.status()), sizeof(obs->gnss1Outputs->timeInfo.status.status()));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->timeInfo.leapSeconds), sizeof(obs->gnss1Outputs->timeInfo.leapSeconds));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.gDop), sizeof(obs->gnss1Outputs->dop.gDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.pDop), sizeof(obs->gnss1Outputs->dop.pDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.tDop), sizeof(obs->gnss1Outputs->dop.tDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.vDop), sizeof(obs->gnss1Outputs->dop.vDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.hDop), sizeof(obs->gnss1Outputs->dop.hDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.nDop), sizeof(obs->gnss1Outputs->dop.nDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->dop.eDop), sizeof(obs->gnss1Outputs->dop.eDop));
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->satInfo.numSats), sizeof(obs->gnss1Outputs->satInfo.numSats));

                for (auto& satellite : obs->gnss1Outputs->satInfo.satellites)
                {
                    _filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.qi), sizeof(satellite.qi));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.el), sizeof(satellite.el));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.az), sizeof(satellite.az));
                }
            }
            if (obs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->raw.tow), sizeof(obs->gnss1Outputs->raw.tow));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->raw.week), sizeof(obs->gnss1Outputs->raw.week));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss1Outputs->raw.numSats), sizeof(obs->gnss1Outputs->raw.numSats));

                for (auto& satellite : obs->gnss1Outputs->raw.satellites)
                {
                    _filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.freq), sizeof(satellite.freq));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.chan), sizeof(satellite.chan));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.slot), sizeof(satellite.slot));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.pr), sizeof(satellite.pr));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.cp), sizeof(satellite.cp));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.dp), sizeof(satellite.dp));
                }
            }
        }
        // Group 5 (Attitude)
        if (obs->attitudeOutputs)
        {
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->attitudeOutputs->vpeStatus.status()), sizeof(obs->attitudeOutputs->vpeStatus.status()));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->ypr.data()), sizeof(obs->attitudeOutputs->ypr));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->qtn.coeffs().data()), sizeof(obs->attitudeOutputs->qtn));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->dcm.data()), sizeof(obs->attitudeOutputs->dcm));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->magNed.data()), sizeof(obs->attitudeOutputs->magNed));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->accelNed.data()), sizeof(obs->attitudeOutputs->accelNed));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->linearAccelBody.data()), sizeof(obs->attitudeOutputs->linearAccelBody));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->linearAccelNed.data()), sizeof(obs->attitudeOutputs->linearAccelNed));
            }
            if (obs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->attitudeOutputs->yprU.data()), sizeof(obs->attitudeOutputs->yprU));
            }
        }
        // Group 6 (INS)
        if (obs->insOutputs)
        {
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->insOutputs->insStatus.status()), sizeof(obs->insOutputs->insStatus.status()));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->posLla.data()), sizeof(obs->insOutputs->posLla));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->posEcef.data()), sizeof(obs->insOutputs->posEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->velBody.data()), sizeof(obs->insOutputs->velBody));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->velNed.data()), sizeof(obs->insOutputs->velNed));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->velEcef.data()), sizeof(obs->insOutputs->velEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->magEcef.data()), sizeof(obs->insOutputs->magEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->accelEcef.data()), sizeof(obs->insOutputs->accelEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->insOutputs->linearAccelEcef.data()), sizeof(obs->insOutputs->linearAccelEcef));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->insOutputs->posU), sizeof(obs->insOutputs->posU));
            }
            if (obs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->insOutputs->velU), sizeof(obs->insOutputs->velU));
            }
        }
        // Group 7 (GNSS2)
        if (obs->gnss2Outputs)
        {
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.year), sizeof(obs->gnss2Outputs->timeUtc.year));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.month), sizeof(obs->gnss2Outputs->timeUtc.month));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.day), sizeof(obs->gnss2Outputs->timeUtc.day));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.hour), sizeof(obs->gnss2Outputs->timeUtc.hour));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.min), sizeof(obs->gnss2Outputs->timeUtc.min));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.sec), sizeof(obs->gnss2Outputs->timeUtc.sec));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeUtc.ms), sizeof(obs->gnss2Outputs->timeUtc.ms));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->tow), sizeof(obs->gnss2Outputs->tow));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->week), sizeof(obs->gnss2Outputs->week));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->numSats), sizeof(obs->gnss2Outputs->numSats));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->fix), sizeof(obs->gnss2Outputs->fix));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->posLla.data()), sizeof(obs->gnss2Outputs->posLla));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->posEcef.data()), sizeof(obs->gnss2Outputs->posEcef));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->velNed.data()), sizeof(obs->gnss2Outputs->velNed));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->velEcef.data()), sizeof(obs->gnss2Outputs->velEcef));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                _filestream.write(reinterpret_cast<const char*>(obs->gnss2Outputs->posU.data()), sizeof(obs->gnss2Outputs->posU));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->velU), sizeof(obs->gnss2Outputs->velU));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeU), sizeof(obs->gnss2Outputs->timeU));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeInfo.status.status()), sizeof(obs->gnss2Outputs->timeInfo.status.status()));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->timeInfo.leapSeconds), sizeof(obs->gnss2Outputs->timeInfo.leapSeconds));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.gDop), sizeof(obs->gnss2Outputs->dop.gDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.pDop), sizeof(obs->gnss2Outputs->dop.pDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.tDop), sizeof(obs->gnss2Outputs->dop.tDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.vDop), sizeof(obs->gnss2Outputs->dop.vDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.hDop), sizeof(obs->gnss2Outputs->dop.hDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.nDop), sizeof(obs->gnss2Outputs->dop.nDop));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->dop.eDop), sizeof(obs->gnss2Outputs->dop.eDop));
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->satInfo.numSats), sizeof(obs->gnss2Outputs->satInfo.numSats));

                for (auto& satellite : obs->gnss2Outputs->satInfo.satellites)
                {
                    _filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.qi), sizeof(satellite.qi));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.el), sizeof(satellite.el));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.az), sizeof(satellite.az));
                }
            }
            if (obs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->raw.tow), sizeof(obs->gnss2Outputs->raw.tow));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->raw.week), sizeof(obs->gnss2Outputs->raw.week));
                _filestream.write(reinterpret_cast<const char*>(&obs->gnss2Outputs->raw.numSats), sizeof(obs->gnss2Outputs->raw.numSats));

                for (auto& satellite : obs->gnss2Outputs->raw.satellites)
                {
                    _filestream.write(reinterpret_cast<const char*>(&satellite.sys), sizeof(satellite.sys));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.svId), sizeof(satellite.svId));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.freq), sizeof(satellite.freq));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.chan), sizeof(satellite.chan));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.slot), sizeof(satellite.slot));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.cno), sizeof(satellite.cno));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.flags), sizeof(satellite.flags));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.pr), sizeof(satellite.pr));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.cp), sizeof(satellite.cp));
                    _filestream.write(reinterpret_cast<const char*>(&satellite.dp), sizeof(satellite.dp));
                }
            }
        }
    }

    LOG_DATA("{}: Message logged", nameId());
}