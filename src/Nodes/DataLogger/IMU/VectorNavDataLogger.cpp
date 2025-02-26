// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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
    : Node(typeStatic())
{
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
    if (FileWriter::guiConfig(".vnb", { ".vnb" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }

    if (CommonLog::ShowOriginInput(nameId().c_str()))
    {
        flow::ApplyChanges();
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

bool NAV::VectorNavDataLogger::onCreateLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (isInitialized())
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

void NAV::VectorNavDataLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const VectorNavBinaryOutput>(queue.extract_front());

    if (_fileType == FileType::BINARY)
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

        if (!obs->insTime.empty())
        {
            auto insTimeGPS = obs->insTime.toGPSweekTow();
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
    else
    {
        LOG_INFO("{}: If you want to log csv instead of binary, use the 'CsvLogger' instead.", nameId());
    }

    LOG_DATA("{}: Message logged", nameId());
}