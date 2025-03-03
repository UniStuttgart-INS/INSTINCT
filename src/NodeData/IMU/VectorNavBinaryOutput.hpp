// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file VectorNavBinaryOutput.hpp
/// @brief Binary Outputs from VectorNav Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <cmath>
#include <memory>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "NodeData/NodeData.hpp"
#include "NodeData/IMU/ImuPos.hpp"
#include "util/Eigen.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

#include "util/Vendor/VectorNav/BinaryOutputs/TimeOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/ImuOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/GnssOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/AttitudeOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/InsOutputs.hpp"
#include "util/Vendor/VectorNav/VectorNavTypes.hpp"
#include <fmt/core.h>
#include <sys/types.h>
#include <vn/types.h>

namespace NAV
{
/// IMU Observation storage class
class VectorNavBinaryOutput : public NodeData
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit VectorNavBinaryOutput(const ImuPos& imuPos)
        : imuPos(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "VectorNavBinaryOutput";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        return {};
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 0; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
            // Group 2 (Time)
        case 0: // Time::TimeStartup [ns]
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)) { return static_cast<double>(timeOutputs->timeStartup); }
            break;
        case 1: // Time::TimeGps [ns]
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)) { return static_cast<double>(timeOutputs->timeGps); }
            break;
        case 2: // Time::GpsTow [ns]
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)) { return static_cast<double>(timeOutputs->gpsTow); }
            break;
        case 3: // Time::GpsWeek
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)) { return static_cast<double>(timeOutputs->gpsWeek); }
            break;
        case 4: // Time::TimeSyncIn [ns]
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)) { return static_cast<double>(timeOutputs->timeSyncIn); }
            break;
        case 5: // Time::TimeGpsPps [ns]
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)) { return static_cast<double>(timeOutputs->timePPS); }
            break;
        case 6: // Time::TimeUTC::year
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.year); }
            break;
        case 7: // Time::TimeUTC::month
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.month); }
            break;
        case 8: // Time::TimeUTC::day
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.day); }
            break;
        case 9: // Time::TimeUTC::hour
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.hour); }
            break;
        case 10: // Time::TimeUTC::min
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.min); }
            break;
        case 11: // Time::TimeUTC::sec
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.sec); }
            break;
        case 12: // Time::TimeUTC::ms
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)) { return static_cast<double>(timeOutputs->timeUtc.ms); }
            break;
        case 13: // Time::SyncInCnt
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)) { return static_cast<double>(timeOutputs->syncInCnt); }
            break;
        case 14: // Time::SyncOutCnt
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)) { return static_cast<double>(timeOutputs->syncOutCnt); }
            break;
        case 15: // Time::TimeStatus::timeOk
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)) { return static_cast<double>(timeOutputs->timeStatus.timeOk()); }
            break;
        case 16: // Time::TimeStatus::dateOk
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)) { return static_cast<double>(timeOutputs->timeStatus.dateOk()); }
            break;
        case 17: // Time::TimeStatus::utcTimeValid
            if (timeOutputs && (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)) { return static_cast<double>(timeOutputs->timeStatus.utcTimeValid()); }
            break;
        // Group 3 (IMU)
        case 18: // IMU::ImuStatus
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)) { return static_cast<double>(imuOutputs->imuStatus); }
            break;
        case 19: // IMU::UncompMag::X [Gauss]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)) { return static_cast<double>(imuOutputs->uncompMag(0)); }
            break;
        case 20: // IMU::UncompMag::Y [Gauss]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)) { return static_cast<double>(imuOutputs->uncompMag(1)); }
            break;
        case 21: // IMU::UncompMag::Z [Gauss]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)) { return static_cast<double>(imuOutputs->uncompMag(2)); }
            break;
        case 22: // IMU::UncompAccel::X [m/s^2]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)) { return static_cast<double>(imuOutputs->uncompAccel(0)); }
            break;
        case 23: // IMU::UncompAccel::Y [m/s^2]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)) { return static_cast<double>(imuOutputs->uncompAccel(1)); }
            break;
        case 24: // IMU::UncompAccel::Z [m/s^2]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)) { return static_cast<double>(imuOutputs->uncompAccel(2)); }
            break;
        case 25: // IMU::UncompGyro::X [rad/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)) { return static_cast<double>(imuOutputs->uncompGyro(0)); }
            break;
        case 26: // IMU::UncompGyro::Y [rad/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)) { return static_cast<double>(imuOutputs->uncompGyro(1)); }
            break;
        case 27: // IMU::UncompGyro::Z [rad/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)) { return static_cast<double>(imuOutputs->uncompGyro(2)); }
            break;
        case 28: // IMU::Temp [Celsius]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)) { return static_cast<double>(imuOutputs->temp); }
            break;
        case 29: // IMU::Pres [kPa]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)) { return static_cast<double>(imuOutputs->pres); }
            break;
        case 30: // IMU::DeltaTime [s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)) { return static_cast<double>(imuOutputs->deltaTime); }
            break;
        case 31: // IMU::DeltaTheta::X [deg]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)) { return static_cast<double>(imuOutputs->deltaTheta(0)); }
            break;
        case 32: // IMU::DeltaTheta::Y [deg]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)) { return static_cast<double>(imuOutputs->deltaTheta(1)); }
            break;
        case 33: // IMU::DeltaTheta::Z [deg]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)) { return static_cast<double>(imuOutputs->deltaTheta(2)); }
            break;
        case 34: // IMU::DeltaVel::X [m/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)) { return static_cast<double>(imuOutputs->deltaV(0)); }
            break;
        case 35: // IMU::DeltaVel::Y [m/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)) { return static_cast<double>(imuOutputs->deltaV(1)); }
            break;
        case 36: // IMU::DeltaVel::Z [m/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)) { return static_cast<double>(imuOutputs->deltaV(2)); }
            break;
        case 37: // IMU::Mag::X [Gauss]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)) { return static_cast<double>(imuOutputs->mag(0)); }
            break;
        case 38: // IMU::Mag::Y [Gauss]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)) { return static_cast<double>(imuOutputs->mag(1)); }
            break;
        case 39: // IMU::Mag::Z [Gauss]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)) { return static_cast<double>(imuOutputs->mag(2)); }
            break;
        case 40: // IMU::Accel::X [m/s^2]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)) { return static_cast<double>(imuOutputs->accel(0)); }
            break;
        case 41: // IMU::Accel::Y [m/s^2]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)) { return static_cast<double>(imuOutputs->accel(1)); }
            break;
        case 42: // IMU::Accel::Z [m/s^2]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)) { return static_cast<double>(imuOutputs->accel(2)); }
            break;
        case 43: // IMU::AngularRate::X [rad/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)) { return static_cast<double>(imuOutputs->angularRate(0)); }
            break;
        case 44: // IMU::AngularRate::Y [rad/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)) { return static_cast<double>(imuOutputs->angularRate(1)); }
            break;
        case 45: // IMU::AngularRate::Z [rad/s]
            if (imuOutputs && (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)) { return static_cast<double>(imuOutputs->angularRate(2)); }
            break;
        // Group 4 (GNSS1)
        case 46: // GNSS1::UTC::year
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.year); }
            break;
        case 47: // GNSS1::UTC::month
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.month); }
            break;
        case 48: // GNSS1::UTC::day
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.day); }
            break;
        case 49: // GNSS1::UTC::hour
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.hour); }
            break;
        case 50: // GNSS1::UTC::min
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.min); }
            break;
        case 51: // GNSS1::UTC::sec
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.sec); }
            break;
        case 52: // GNSS1::UTC::ms
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss1Outputs->timeUtc.ms); }
            break;
        case 53: // GNSS1::Tow [ns]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)) { return static_cast<double>(gnss1Outputs->tow); }
            break;
        case 54: // GNSS1::Week
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)) { return static_cast<double>(gnss1Outputs->week); }
            break;
        case 55: // GNSS1::NumSats
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)) { return static_cast<double>(gnss1Outputs->numSats); }
            break;
        case 56: // GNSS1::Fix
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)) { return static_cast<double>(gnss1Outputs->fix); }
            break;
        case 57: // GNSS1::PosLla::latitude [deg]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)) { return gnss1Outputs->posLla(0); }
            break;
        case 58: // GNSS1::PosLla::longitude [deg]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)) { return gnss1Outputs->posLla(1); }
            break;
        case 59: // GNSS1::PosLla::altitude [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)) { return gnss1Outputs->posLla(2); }
            break;
        case 60: // GNSS1::PosEcef::X [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)) { return gnss1Outputs->posEcef(0); }
            break;
        case 61: // GNSS1::PosEcef::Y [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)) { return gnss1Outputs->posEcef(1); }
            break;
        case 62: // GNSS1::PosEcef::Z [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)) { return gnss1Outputs->posEcef(2); }
            break;
        case 63: // GNSS1::VelNed::N [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)) { return static_cast<double>(gnss1Outputs->velNed(0)); }
            break;
        case 64: // GNSS1::VelNed::E [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)) { return static_cast<double>(gnss1Outputs->velNed(1)); }
            break;
        case 65: // GNSS1::VelNed::D [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)) { return static_cast<double>(gnss1Outputs->velNed(2)); }
            break;
        case 66: // GNSS1::VelEcef::X [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)) { return static_cast<double>(gnss1Outputs->velEcef(0)); }
            break;
        case 67: // GNSS1::VelEcef::Y [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)) { return static_cast<double>(gnss1Outputs->velEcef(1)); }
            break;
        case 68: // GNSS1::VelEcef::Z [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)) { return static_cast<double>(gnss1Outputs->velEcef(2)); }
            break;
        case 69: // GNSS1::PosU::N [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)) { return static_cast<double>(gnss1Outputs->posU(0)); }
            break;
        case 70: // GNSS1::PosU::E [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)) { return static_cast<double>(gnss1Outputs->posU(1)); }
            break;
        case 71: // GNSS1::PosU::D [m]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)) { return static_cast<double>(gnss1Outputs->posU(2)); }
            break;
        case 72: // GNSS1::VelU [m/s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)) { return static_cast<double>(gnss1Outputs->velU); }
            break;
        case 73: // GNSS1::TimeU [s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)) { return static_cast<double>(gnss1Outputs->timeU); }
            break;
        case 74: // GNSS1::TimeInfo::Status::timeOk
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss1Outputs->timeInfo.status.timeOk()); }
            break;
        case 75: // GNSS1::TimeInfo::Status::dateOk
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss1Outputs->timeInfo.status.dateOk()); }
            break;
        case 76: // GNSS1::TimeInfo::Status::utcTimeValid
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss1Outputs->timeInfo.status.utcTimeValid()); }
            break;
        case 77: // GNSS1::TimeInfo::LeapSeconds
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss1Outputs->timeInfo.leapSeconds); }
            break;
        case 78: // GNSS1::DOP::g
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.gDop); }
            break;
        case 79: // GNSS1::DOP::p
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.pDop); }
            break;
        case 80: // GNSS1::DOP::t
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.tDop); }
            break;
        case 81: // GNSS1::DOP::v
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.vDop); }
            break;
        case 82: // GNSS1::DOP::h
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.hDop); }
            break;
        case 83: // GNSS1::DOP::n
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.nDop); }
            break;
        case 84: // GNSS1::DOP::e
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss1Outputs->dop.eDop); }
            break;
        case 85: // GNSS1::SatInfo::NumSats
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)) { return static_cast<double>(gnss1Outputs->satInfo.numSats); }
            break;
        case 86: // GNSS1::RawMeas::Tow [s]
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)) { return gnss1Outputs->raw.tow; }
            break;
        case 87: // GNSS1::RawMeas::Week
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)) { return static_cast<double>(gnss1Outputs->raw.week); }
            break;
        case 88: // GNSS1::RawMeas::NumSats
            if (gnss1Outputs && (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)) { return static_cast<double>(gnss1Outputs->raw.numSats); }
            break;
        // Group 5 (Attitude)
        case 89: // Att::VpeStatus::AttitudeQuality
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.attitudeQuality()); }
            break;
        case 90: // Att::VpeStatus::GyroSaturation
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.gyroSaturation()); }
            break;
        case 91: // Att::VpeStatus::GyroSaturationRecovery
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.gyroSaturationRecovery()); }
            break;
        case 92: // Att::VpeStatus::MagDisturbance
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.magDisturbance()); }
            break;
        case 93: // Att::VpeStatus::MagSaturation
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.magSaturation()); }
            break;
        case 94: // Att::VpeStatus::AccDisturbance
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.accDisturbance()); }
            break;
        case 95: // Att::VpeStatus::AccSaturation
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.accSaturation()); }
            break;
        case 96: // Att::VpeStatus::KnownMagDisturbance
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.knownMagDisturbance()); }
            break;
        case 97: // Att::VpeStatus::KnownAccelDisturbance
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)) { return static_cast<double>(attitudeOutputs->vpeStatus.knownAccelDisturbance()); }
            break;
        case 98: // Att::YawPitchRoll::Y [deg]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)) { return static_cast<double>(attitudeOutputs->ypr(0)); }
            break;
        case 99: // Att::YawPitchRoll::P [deg]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)) { return static_cast<double>(attitudeOutputs->ypr(1)); }
            break;
        case 100: // Att::YawPitchRoll::R [deg]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)) { return static_cast<double>(attitudeOutputs->ypr(2)); }
            break;
        case 101: // Att::Quaternion::w
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)) { return static_cast<double>(attitudeOutputs->qtn.w()); }
            break;
        case 102: // Att::Quaternion::x
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)) { return static_cast<double>(attitudeOutputs->qtn.x()); }
            break;
        case 103: // Att::Quaternion::y
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)) { return static_cast<double>(attitudeOutputs->qtn.y()); }
            break;
        case 104: // Att::Quaternion::z
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)) { return static_cast<double>(attitudeOutputs->qtn.z()); }
            break;
        case 105: // Att::DCM::0-0
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(0, 0)); }
            break;
        case 106: // Att::DCM::0-1
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(0, 1)); }
            break;
        case 107: // Att::DCM::0-2
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(0, 2)); }
            break;
        case 108: // Att::DCM::1-0
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(1, 0)); }
            break;
        case 109: // Att::DCM::1-1
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(1, 1)); }
            break;
        case 110: // Att::DCM::1-2
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(1, 2)); }
            break;
        case 111: // Att::DCM::2-0
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(2, 0)); }
            break;
        case 112: // Att::DCM::2-1
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(2, 1)); }
            break;
        case 113: // Att::DCM::2-2
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)) { return static_cast<double>(attitudeOutputs->dcm(2, 2)); }
            break;
        case 114: // Att::MagNed::N [Gauss]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)) { return static_cast<double>(attitudeOutputs->magNed(0)); }
            break;
        case 115: // Att::MagNed::E [Gauss]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)) { return static_cast<double>(attitudeOutputs->magNed(1)); }
            break;
        case 116: // Att::MagNed::D [Gauss]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)) { return static_cast<double>(attitudeOutputs->magNed(2)); }
            break;
        case 117: // Att::AccelNed::N [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)) { return static_cast<double>(attitudeOutputs->accelNed(0)); }
            break;
        case 118: // Att::AccelNed::E [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)) { return static_cast<double>(attitudeOutputs->accelNed(1)); }
            break;
        case 119: // Att::AccelNed::D [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)) { return static_cast<double>(attitudeOutputs->accelNed(2)); }
            break;
        case 120: // Att::LinearAccelBody::X [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)) { return static_cast<double>(attitudeOutputs->linearAccelBody(0)); }
            break;
        case 121: // Att::LinearAccelBody::Y [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)) { return static_cast<double>(attitudeOutputs->linearAccelBody(1)); }
            break;
        case 122: // Att::LinearAccelBody::Z [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)) { return static_cast<double>(attitudeOutputs->linearAccelBody(2)); }
            break;
        case 123: // Att::LinearAccelNed::N [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)) { return static_cast<double>(attitudeOutputs->linearAccelNed(0)); }
            break;
        case 124: // Att::LinearAccelNed::E [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)) { return static_cast<double>(attitudeOutputs->linearAccelNed(1)); }
            break;
        case 125: // Att::LinearAccelNed::D [m/s^2]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)) { return static_cast<double>(attitudeOutputs->linearAccelNed(2)); }
            break;
        case 126: // Att::YprU::Y [deg]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)) { return static_cast<double>(attitudeOutputs->yprU(0)); }
            break;
        case 127: // Att::YprU::P [deg]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)) { return static_cast<double>(attitudeOutputs->yprU(1)); }
            break;
        case 128: // Att::YprU::R [deg]
            if (attitudeOutputs && (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)) { return static_cast<double>(attitudeOutputs->yprU(2)); }
            break;
        // Group 6 (INS)
        case 129: // INS::InsStatus::Mode
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.mode()); }
            break;
        case 130: // INS::InsStatus::GpsFix
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.gpsFix()); }
            break;
        case 131: // INS::InsStatus::Error::IMU
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.errorIMU()); }
            break;
        case 132: // INS::InsStatus::Error::MagPres
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.errorMagPres()); }
            break;
        case 133: // INS::InsStatus::Error::GNSS
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.errorGnss()); }
            break;
        case 134: // INS::InsStatus::GpsHeadingIns
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.gpsHeadingIns()); }
            break;
        case 135: // INS::InsStatus::GpsCompass
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)) { return static_cast<double>(insOutputs->insStatus.gpsCompass()); }
            break;
        case 136: // INS::PosLla::latitude [deg]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)) { return insOutputs->posLla(0); }
            break;
        case 137: // INS::PosLla::longitude [deg]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)) { return insOutputs->posLla(1); }
            break;
        case 138: // INS::PosLla::altitude [m]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)) { return insOutputs->posLla(2); }
            break;
        case 139: // INS::PosEcef::X [m]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)) { return insOutputs->posEcef(0); }
            break;
        case 140: // INS::PosEcef::Y [m]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)) { return insOutputs->posEcef(1); }
            break;
        case 141: // INS::PosEcef::Z [m]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)) { return insOutputs->posEcef(2); }
            break;
        case 142: // INS::VelBody::X [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)) { return static_cast<double>(insOutputs->velBody(0)); }
            break;
        case 143: // INS::VelBody::Y [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)) { return static_cast<double>(insOutputs->velBody(1)); }
            break;
        case 144: // INS::VelBody::Z [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)) { return static_cast<double>(insOutputs->velBody(2)); }
            break;
        case 145: // INS::VelNed::N [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)) { return static_cast<double>(insOutputs->velNed(0)); }
            break;
        case 146: // INS::VelNed::E [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)) { return static_cast<double>(insOutputs->velNed(1)); }
            break;
        case 147: // INS::VelNed::D [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)) { return static_cast<double>(insOutputs->velNed(2)); }
            break;
        case 148: // INS::VelEcef::X [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)) { return static_cast<double>(insOutputs->velEcef(0)); }
            break;
        case 149: // INS::VelEcef::Y [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)) { return static_cast<double>(insOutputs->velEcef(1)); }
            break;
        case 150: // INS::VelEcef::Z [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)) { return static_cast<double>(insOutputs->velEcef(2)); }
            break;
        case 151: // INS::MagEcef::X [Gauss}
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)) { return static_cast<double>(insOutputs->magEcef(0)); }
            break;
        case 152: // INS::MagEcef::Y [Gauss}
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)) { return static_cast<double>(insOutputs->magEcef(1)); }
            break;
        case 153: // INS::MagEcef::Z [Gauss}
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)) { return static_cast<double>(insOutputs->magEcef(2)); }
            break;
        case 154: // INS::AccelEcef::X [m/s^2]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)) { return static_cast<double>(insOutputs->accelEcef(0)); }
            break;
        case 155: // INS::AccelEcef::Y [m/s^2]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)) { return static_cast<double>(insOutputs->accelEcef(1)); }
            break;
        case 156: // INS::AccelEcef::Z [m/s^2]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)) { return static_cast<double>(insOutputs->accelEcef(2)); }
            break;
        case 157: // INS::LinearAccelEcef::X [m/s^2]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)) { return static_cast<double>(insOutputs->linearAccelEcef(0)); }
            break;
        case 158: // INS::LinearAccelEcef::Y [m/s^2]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)) { return static_cast<double>(insOutputs->linearAccelEcef(1)); }
            break;
        case 159: // INS::LinearAccelEcef::Z [m/s^2]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)) { return static_cast<double>(insOutputs->linearAccelEcef(2)); }
            break;
        case 160: // INS::PosU [m]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)) { return static_cast<double>(insOutputs->posU); }
            break;
        case 161: // INS::VelU [m/s]
            if (insOutputs && (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)) { return static_cast<double>(insOutputs->velU); }
            break;
        // Group 7 (GNSS2)
        case 162: // GNSS2::UTC::year
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.year); }
            break;
        case 163: // GNSS2::UTC::month
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.month); }
            break;
        case 164: // GNSS2::UTC::day
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.day); }
            break;
        case 165: // GNSS2::UTC::hour
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.hour); }
            break;
        case 166: // GNSS2::UTC::min
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.min); }
            break;
        case 167: // GNSS2::UTC::sec
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.sec); }
            break;
        case 168: // GNSS2::UTC::ms
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)) { return static_cast<double>(gnss2Outputs->timeUtc.ms); }
            break;
        case 169: // GNSS2::Tow [ns]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)) { return static_cast<double>(gnss2Outputs->tow); }
            break;
        case 170: // GNSS2::Week
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)) { return static_cast<double>(gnss2Outputs->week); }
            break;
        case 171: // GNSS2::NumSats
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)) { return static_cast<double>(gnss2Outputs->numSats); }
            break;
        case 172: // GNSS2::Fix
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)) { return static_cast<double>(gnss2Outputs->fix); }
            break;
        case 173: // GNSS2::PosLla::latitude [deg]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)) { return gnss2Outputs->posLla(0); }
            break;
        case 174: // GNSS2::PosLla::longitude [deg]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)) { return gnss2Outputs->posLla(1); }
            break;
        case 175: // GNSS2::PosLla::altitude [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)) { return gnss2Outputs->posLla(2); }
            break;
        case 176: // GNSS2::PosEcef::X [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)) { return gnss2Outputs->posEcef(0); }
            break;
        case 177: // GNSS2::PosEcef::Y [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)) { return gnss2Outputs->posEcef(1); }
            break;
        case 178: // GNSS2::PosEcef::Z [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)) { return gnss2Outputs->posEcef(2); }
            break;
        case 179: // GNSS2::VelNed::N [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)) { return static_cast<double>(gnss2Outputs->velNed(0)); }
            break;
        case 180: // GNSS2::VelNed::E [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)) { return static_cast<double>(gnss2Outputs->velNed(1)); }
            break;
        case 181: // GNSS2::VelNed::D [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)) { return static_cast<double>(gnss2Outputs->velNed(2)); }
            break;
        case 182: // GNSS2::VelEcef::X [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)) { return static_cast<double>(gnss2Outputs->velEcef(0)); }
            break;
        case 183: // GNSS2::VelEcef::Y [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)) { return static_cast<double>(gnss2Outputs->velEcef(1)); }
            break;
        case 184: // GNSS2::VelEcef::Z [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)) { return static_cast<double>(gnss2Outputs->velEcef(2)); }
            break;
        case 185: // GNSS2::PosU::N [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)) { return static_cast<double>(gnss2Outputs->posU(0)); }
            break;
        case 186: // GNSS2::PosU::E [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)) { return static_cast<double>(gnss2Outputs->posU(1)); }
            break;
        case 187: // GNSS2::PosU::D [m]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)) { return static_cast<double>(gnss2Outputs->posU(2)); }
            break;
        case 188: // GNSS2::VelU [m/s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)) { return static_cast<double>(gnss2Outputs->velU); }
            break;
        case 189: // GNSS2::TimeU [s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)) { return static_cast<double>(gnss2Outputs->timeU); }
            break;
        case 190: // GNSS2::TimeInfo::Status::timeOk
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss2Outputs->timeInfo.status.timeOk()); }
            break;
        case 191: // GNSS2::TimeInfo::Status::dateOk
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss2Outputs->timeInfo.status.dateOk()); }
            break;
        case 192: // GNSS2::TimeInfo::Status::utcTimeValid
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss2Outputs->timeInfo.status.utcTimeValid()); }
            break;
        case 193: // GNSS2::TimeInfo::LeapSeconds
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)) { return static_cast<double>(gnss2Outputs->timeInfo.leapSeconds); }
            break;
        case 194: // GNSS2::DOP::g
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.gDop); }
            break;
        case 195: // GNSS2::DOP::p
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.pDop); }
            break;
        case 196: // GNSS2::DOP::t
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.tDop); }
            break;
        case 197: // GNSS2::DOP::v
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.vDop); }
            break;
        case 198: // GNSS2::DOP::h
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.hDop); }
            break;
        case 199: // GNSS2::DOP::n
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.nDop); }
            break;
        case 200: // GNSS2::DOP::e
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)) { return static_cast<double>(gnss2Outputs->dop.eDop); }
            break;
        case 201: // GNSS2::SatInfo::NumSats
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)) { return static_cast<double>(gnss2Outputs->satInfo.numSats); }
            break;
        case 202: // GNSS2::RawMeas::Tow [s]
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)) { return gnss2Outputs->raw.tow; }
            break;
        case 203: // GNSS2::RawMeas::Week
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)) { return static_cast<double>(gnss2Outputs->raw.week); }
            break;
        case 204: // GNSS2::RawMeas::NumSats
            if (gnss2Outputs && (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)) { return static_cast<double>(gnss2Outputs->raw.numSats); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Get the Sat Sys object in "INSTINCT" format
    /// @param[in, out] sys VectorNav Satellite Constellation
    /// @return Satellite System in "INSTINCT" format
    [[nodiscard]] static SatelliteSystem getSatSys(vendor::vectornav::SatSys& sys)
    {
        SatelliteSystem satSys = SatSys_None;
        switch (sys)
        {
        case vendor::vectornav::SatSys::GPS:
            satSys = GPS;
            break;
        case vendor::vectornav::SatSys::SBAS:
            satSys = SBAS;
            break;
        case vendor::vectornav::SatSys::Galileo:
            satSys = GAL;
            break;
        case vendor::vectornav::SatSys::BeiDou:
            satSys = BDS;
            break;
        case vendor::vectornav::SatSys::IMES:
            LOG_TRACE("VectorNav SatInfoElement satellite system '{}' is not supported yet. Skipping measurement.", sys);
            break;
        case vendor::vectornav::SatSys::QZSS:
            satSys = QZSS;
            break;
        case vendor::vectornav::SatSys::GLONASS:
            satSys = GLO;
            break;
        default: // IRNSS not in vectorNav
            LOG_TRACE("VectorNav SatInfoElement satellite system '{}' is not supported yet. Skipping measurement.", sys);
            break;
        }
        return satSys;
    }

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;

        // Group 2 (Time)
        if (timeOutputs)
        {
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
            {
                descriptors.emplace_back("Time::TimeStartup [ns]");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
            {
                descriptors.emplace_back("Time::TimeGps [ns]");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
            {
                descriptors.emplace_back("Time::GpsTow [ns]");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
            {
                descriptors.emplace_back("Time::GpsWeek");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
            {
                descriptors.emplace_back("Time::TimeSyncIn [ns]");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
            {
                descriptors.emplace_back("Time::TimeGpsPps [ns]");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
            {
                descriptors.emplace_back("Time::TimeUTC::year");
                descriptors.emplace_back("Time::TimeUTC::month");
                descriptors.emplace_back("Time::TimeUTC::day");
                descriptors.emplace_back("Time::TimeUTC::hour");
                descriptors.emplace_back("Time::TimeUTC::min");
                descriptors.emplace_back("Time::TimeUTC::sec");
                descriptors.emplace_back("Time::TimeUTC::ms");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
            {
                descriptors.emplace_back("Time::SyncInCnt");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
            {
                descriptors.emplace_back("Time::SyncOutCnt");
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
            {
                descriptors.emplace_back("Time::TimeStatus::timeOk");
                descriptors.emplace_back("Time::TimeStatus::dateOk");
                descriptors.emplace_back("Time::TimeStatus::utcTimeValid");
            }
        }
        // Group 3 (IMU)
        if (imuOutputs)
        {
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
            {
                descriptors.emplace_back("IMU::ImuStatus");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
            {
                descriptors.emplace_back("IMU::UncompMag::X [Gauss]");
                descriptors.emplace_back("IMU::UncompMag::Y [Gauss]");
                descriptors.emplace_back("IMU::UncompMag::Z [Gauss]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                descriptors.emplace_back("IMU::UncompAccel::X [m/s^2]");
                descriptors.emplace_back("IMU::UncompAccel::Y [m/s^2]");
                descriptors.emplace_back("IMU::UncompAccel::Z [m/s^2]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                descriptors.emplace_back("IMU::UncompGyro::X [rad/s]");
                descriptors.emplace_back("IMU::UncompGyro::Y [rad/s]");
                descriptors.emplace_back("IMU::UncompGyro::Z [rad/s]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                descriptors.emplace_back("IMU::Temp [Celsius]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                descriptors.emplace_back("IMU::Pres [kPa]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                descriptors.emplace_back("IMU::DeltaTime [s]");
                descriptors.emplace_back("IMU::DeltaTheta::X [deg]");
                descriptors.emplace_back("IMU::DeltaTheta::Y [deg]");
                descriptors.emplace_back("IMU::DeltaTheta::Z [deg]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                descriptors.emplace_back("IMU::DeltaVel::X [m/s]");
                descriptors.emplace_back("IMU::DeltaVel::Y [m/s]");
                descriptors.emplace_back("IMU::DeltaVel::Z [m/s]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                descriptors.emplace_back("IMU::Mag::X [Gauss]");
                descriptors.emplace_back("IMU::Mag::Y [Gauss]");
                descriptors.emplace_back("IMU::Mag::Z [Gauss]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                descriptors.emplace_back("IMU::Accel::X [m/s^2]");
                descriptors.emplace_back("IMU::Accel::Y [m/s^2]");
                descriptors.emplace_back("IMU::Accel::Z [m/s^2]");
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                descriptors.emplace_back("IMU::AngularRate::X [rad/s]");
                descriptors.emplace_back("IMU::AngularRate::Y [rad/s]");
                descriptors.emplace_back("IMU::AngularRate::Z [rad/s]");
            }
        }
        // Group 4 (GNSS1)
        if (gnss1Outputs)
        {
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                descriptors.emplace_back("GNSS1::UTC::year");
                descriptors.emplace_back("GNSS1::UTC::month");
                descriptors.emplace_back("GNSS1::UTC::day");
                descriptors.emplace_back("GNSS1::UTC::hour");
                descriptors.emplace_back("GNSS1::UTC::min");
                descriptors.emplace_back("GNSS1::UTC::sec");
                descriptors.emplace_back("GNSS1::UTC::ms");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                descriptors.emplace_back("GNSS1::Tow [ns]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                descriptors.emplace_back("GNSS1::Week");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                descriptors.emplace_back("GNSS1::NumSats");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                descriptors.emplace_back("GNSS1::Fix");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                descriptors.emplace_back("GNSS1::PosLla::latitude [deg]");
                descriptors.emplace_back("GNSS1::PosLla::longitude [deg]");
                descriptors.emplace_back("GNSS1::PosLla::altitude [m]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                descriptors.emplace_back("GNSS1::PosEcef::X [m]");
                descriptors.emplace_back("GNSS1::PosEcef::Y [m]");
                descriptors.emplace_back("GNSS1::PosEcef::Z [m]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                descriptors.emplace_back("GNSS1::VelNed::N [m/s]");
                descriptors.emplace_back("GNSS1::VelNed::E [m/s]");
                descriptors.emplace_back("GNSS1::VelNed::D [m/s]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                descriptors.emplace_back("GNSS1::VelEcef::X [m/s]");
                descriptors.emplace_back("GNSS1::VelEcef::Y [m/s]");
                descriptors.emplace_back("GNSS1::VelEcef::Z [m/s]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                descriptors.emplace_back("GNSS1::PosU::N [m]");
                descriptors.emplace_back("GNSS1::PosU::E [m]");
                descriptors.emplace_back("GNSS1::PosU::D [m]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                descriptors.emplace_back("GNSS1::VelU [m/s]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                descriptors.emplace_back("GNSS1::TimeU [s]");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                descriptors.emplace_back("GNSS1::TimeInfo::Status::timeOk");
                descriptors.emplace_back("GNSS1::TimeInfo::Status::dateOk");
                descriptors.emplace_back("GNSS1::TimeInfo::Status::utcTimeValid");
                descriptors.emplace_back("GNSS1::TimeInfo::LeapSeconds");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                descriptors.emplace_back("GNSS1::DOP::g");
                descriptors.emplace_back("GNSS1::DOP::p");
                descriptors.emplace_back("GNSS1::DOP::t");
                descriptors.emplace_back("GNSS1::DOP::v");
                descriptors.emplace_back("GNSS1::DOP::h");
                descriptors.emplace_back("GNSS1::DOP::n");
                descriptors.emplace_back("GNSS1::DOP::e");
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                descriptors.emplace_back("GNSS1::SatInfo::NumSats");
                for (auto& satellite : gnss1Outputs->satInfo.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag Healthy", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag Almanac", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag Ephemeris", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag DifferentialCorrection", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag UsedForNavigation", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag AzimuthElevationValid", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - flag UsedForRTK", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - cno", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - qi", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - el", satId));
                    descriptors.push_back(fmt::format("GNSS1::SatInfo::{} - az", satId));
                }
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                descriptors.emplace_back("GNSS1::RawMeas::Tow [s]");
                descriptors.emplace_back("GNSS1::RawMeas::Week");
                descriptors.emplace_back("GNSS1::RawMeas::NumSats");
                for (auto& satellite : gnss1Outputs->raw.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - sys", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - svId", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - freq", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - chan", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - slot", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - cno", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag Searching", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag Tracking", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag TimeValid", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag CodeLock", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseLock", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseHalfAmbiguity", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseHalfSub", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseSlip", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - flag PseudorangeSmoothed", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - pr", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - cp", satId));
                    descriptors.push_back(fmt::format("GNSS1::RawMeas::{} - dp", satId));
                }
            }
        }
        // Group 5 (Attitude)
        if (attitudeOutputs)
        {
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
            {
                descriptors.emplace_back("Att::VpeStatus::AttitudeQuality");
                descriptors.emplace_back("Att::VpeStatus::GyroSaturation");
                descriptors.emplace_back("Att::VpeStatus::GyroSaturationRecovery");
                descriptors.emplace_back("Att::VpeStatus::MagDisturbance");
                descriptors.emplace_back("Att::VpeStatus::MagSaturation");
                descriptors.emplace_back("Att::VpeStatus::AccDisturbance");
                descriptors.emplace_back("Att::VpeStatus::AccSaturation");
                descriptors.emplace_back("Att::VpeStatus::KnownMagDisturbance");
                descriptors.emplace_back("Att::VpeStatus::KnownAccelDisturbance");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
            {
                descriptors.emplace_back("Att::YawPitchRoll::Y [deg]");
                descriptors.emplace_back("Att::YawPitchRoll::P [deg]");
                descriptors.emplace_back("Att::YawPitchRoll::R [deg]");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
            {
                descriptors.emplace_back("Att::Quaternion::w");
                descriptors.emplace_back("Att::Quaternion::x");
                descriptors.emplace_back("Att::Quaternion::y");
                descriptors.emplace_back("Att::Quaternion::z");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
            {
                descriptors.emplace_back("Att::DCM::0-0,Att::DCM::0-1,Att::DCM::0-2");
                descriptors.emplace_back("Att::DCM::1-0,Att::DCM::1-1,Att::DCM::1-2");
                descriptors.emplace_back("Att::DCM::2-0,Att::DCM::2-1,Att::DCM::2-2");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
            {
                descriptors.emplace_back("Att::MagNed::N [Gauss]");
                descriptors.emplace_back("Att::MagNed::E [Gauss]");
                descriptors.emplace_back("Att::MagNed::D [Gauss]");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
            {
                descriptors.emplace_back("Att::AccelNed::N [m/s^2]");
                descriptors.emplace_back("Att::AccelNed::E [m/s^2]");
                descriptors.emplace_back("Att::AccelNed::D [m/s^2]");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
            {
                descriptors.emplace_back("Att::LinearAccelBody::X [m/s^2]");
                descriptors.emplace_back("Att::LinearAccelBody::Y [m/s^2]");
                descriptors.emplace_back("Att::LinearAccelBody::Z [m/s^2]");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
            {
                descriptors.emplace_back("Att::LinearAccelNed::N [m/s^2]");
                descriptors.emplace_back("Att::LinearAccelNed::E [m/s^2]");
                descriptors.emplace_back("Att::LinearAccelNed::D [m/s^2]");
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
            {
                descriptors.emplace_back("Att::YprU::Y [deg]");
                descriptors.emplace_back("Att::YprU::P [deg]");
                descriptors.emplace_back("Att::YprU::R [deg]");
            }
        }
        // Group 6 (INS)
        if (insOutputs)
        {
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
            {
                descriptors.emplace_back("INS::InsStatus::Mode");
                descriptors.emplace_back("INS::InsStatus::GpsFix");
                descriptors.emplace_back("INS::InsStatus::Error::IMU");
                descriptors.emplace_back("INS::InsStatus::Error::MagPres");
                descriptors.emplace_back("INS::InsStatus::Error::GNSS");
                descriptors.emplace_back("INS::InsStatus::GpsHeadingIns");
                descriptors.emplace_back("INS::InsStatus::GpsCompass");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
            {
                descriptors.emplace_back("INS::PosLla::latitude [deg]");
                descriptors.emplace_back("INS::PosLla::longitude [deg]");
                descriptors.emplace_back("INS::PosLla::altitude [m]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
            {
                descriptors.emplace_back("INS::PosEcef::X [m]");
                descriptors.emplace_back("INS::PosEcef::Y [m]");
                descriptors.emplace_back("INS::PosEcef::Z [m]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
            {
                descriptors.emplace_back("INS::VelBody::X [m/s]");
                descriptors.emplace_back("INS::VelBody::Y [m/s]");
                descriptors.emplace_back("INS::VelBody::Z [m/s]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
            {
                descriptors.emplace_back("INS::VelNed::N [m/s]");
                descriptors.emplace_back("INS::VelNed::E [m/s]");
                descriptors.emplace_back("INS::VelNed::D [m/s]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
            {
                descriptors.emplace_back("INS::VelEcef::X [m/s]");
                descriptors.emplace_back("INS::VelEcef::Y [m/s]");
                descriptors.emplace_back("INS::VelEcef::Z [m/s]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
            {
                descriptors.emplace_back("INS::MagEcef::X [Gauss]");
                descriptors.emplace_back("INS::MagEcef::Y [Gauss]");
                descriptors.emplace_back("INS::MagEcef::Z [Gauss]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
            {
                descriptors.emplace_back("INS::AccelEcef::X [m/s^2]");
                descriptors.emplace_back("INS::AccelEcef::Y [m/s^2]");
                descriptors.emplace_back("INS::AccelEcef::Z [m/s^2]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
            {
                descriptors.emplace_back("INS::LinearAccelEcef::X [m/s^2]");
                descriptors.emplace_back("INS::LinearAccelEcef::Y [m/s^2]");
                descriptors.emplace_back("INS::LinearAccelEcef::Z [m/s^2]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
            {
                descriptors.emplace_back("INS::PosU [m]");
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
            {
                descriptors.emplace_back("INS::VelU [m/s]");
            }
        }
        // Group 7 (GNSS2)
        if (gnss2Outputs)
        {
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                descriptors.emplace_back("GNSS2::UTC::year");
                descriptors.emplace_back("GNSS2::UTC::month");
                descriptors.emplace_back("GNSS2::UTC::day");
                descriptors.emplace_back("GNSS2::UTC::hour");
                descriptors.emplace_back("GNSS2::UTC::min");
                descriptors.emplace_back("GNSS2::UTC::sec");
                descriptors.emplace_back("GNSS2::UTC::ms");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                descriptors.emplace_back("GNSS2::Tow [ns]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                descriptors.emplace_back("GNSS2::Week");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                descriptors.emplace_back("GNSS2::NumSats");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                descriptors.emplace_back("GNSS2::Fix");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                descriptors.emplace_back("GNSS2::PosLla::latitude [deg]");
                descriptors.emplace_back("GNSS2::PosLla::longitude [deg]");
                descriptors.emplace_back("GNSS2::PosLla::altitude [m]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                descriptors.emplace_back("GNSS2::PosEcef::X [m]");
                descriptors.emplace_back("GNSS2::PosEcef::Y [m]");
                descriptors.emplace_back("GNSS2::PosEcef::Z [m]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                descriptors.emplace_back("GNSS2::VelNed::N [m/s]");
                descriptors.emplace_back("GNSS2::VelNed::E [m/s]");
                descriptors.emplace_back("GNSS2::VelNed::D [m/s]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                descriptors.emplace_back("GNSS2::VelEcef::X [m/s]");
                descriptors.emplace_back("GNSS2::VelEcef::Y [m/s]");
                descriptors.emplace_back("GNSS2::VelEcef::Z [m/s]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                descriptors.emplace_back("GNSS2::PosU::N [m]");
                descriptors.emplace_back("GNSS2::PosU::E [m]");
                descriptors.emplace_back("GNSS2::PosU::D [m]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                descriptors.emplace_back("GNSS2::VelU [m/s]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                descriptors.emplace_back("GNSS2::TimeU [s]");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                descriptors.emplace_back("GNSS2::TimeInfo::Status::timeOk");
                descriptors.emplace_back("GNSS2::TimeInfo::Status::dateOk");
                descriptors.emplace_back("GNSS2::TimeInfo::Status::utcTimeValid");
                descriptors.emplace_back("GNSS2::TimeInfo::LeapSeconds");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                descriptors.emplace_back("GNSS2::DOP::g");
                descriptors.emplace_back("GNSS2::DOP::p");
                descriptors.emplace_back("GNSS2::DOP::t");
                descriptors.emplace_back("GNSS2::DOP::v");
                descriptors.emplace_back("GNSS2::DOP::h");
                descriptors.emplace_back("GNSS2::DOP::n");
                descriptors.emplace_back("GNSS2::DOP::e");
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                descriptors.emplace_back("GNSS2::SatInfo::NumSats");
                for (auto& satellite : gnss2Outputs->satInfo.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag Healthy", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag Almanac", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag Ephemeris", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag DifferentialCorrection", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag UsedForNavigation", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag AzimuthElevationValid", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - flag UsedForRTK", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - cno", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - qi", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - el", satId));
                    descriptors.push_back(fmt::format("GNSS2::SatInfo::{} - az", satId));
                }
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                descriptors.emplace_back("GNSS2::RawMeas::Tow [s]");
                descriptors.emplace_back("GNSS2::RawMeas::Week");
                descriptors.emplace_back("GNSS2::RawMeas::NumSats");
                for (auto& satellite : gnss2Outputs->raw.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - sys", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - svId", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - freq", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - chan", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - slot", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - cno", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag Searching", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag Tracking", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag TimeValid", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag CodeLock", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseLock", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseHalfAmbiguity", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseHalfSub", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseSlip", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - flag PseudorangeSmoothed", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - pr", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - cp", satId));
                    descriptors.push_back(fmt::format("GNSS2::RawMeas::{} - dp", satId));
                }
            }
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        // Group 2 (Time)
        if (descriptor == "Time::TimeStartup [ns]") { return static_cast<double>(timeOutputs->timeStartup); }
        if (descriptor == "Time::TimeGps [ns]") { return static_cast<double>(timeOutputs->timeGps); }
        if (descriptor == "Time::GpsTow [ns]") { return static_cast<double>(timeOutputs->gpsTow); }
        if (descriptor == "Time::GpsWeek") { return static_cast<double>(timeOutputs->gpsWeek); }
        if (descriptor == "Time::TimeSyncIn [ns]") { return static_cast<double>(timeOutputs->timeSyncIn); }
        if (descriptor == "Time::TimeGpsPps [ns]") { return static_cast<double>(timeOutputs->timePPS); }
        if (descriptor == "Time::TimeUTC::year") { return static_cast<double>(timeOutputs->timeUtc.year); }
        if (descriptor == "Time::TimeUTC::month") { return static_cast<double>(timeOutputs->timeUtc.month); }
        if (descriptor == "Time::TimeUTC::day") { return static_cast<double>(timeOutputs->timeUtc.day); }
        if (descriptor == "Time::TimeUTC::hour") { return static_cast<double>(timeOutputs->timeUtc.hour); }
        if (descriptor == "Time::TimeUTC::min") { return static_cast<double>(timeOutputs->timeUtc.min); }
        if (descriptor == "Time::TimeUTC::sec") { return static_cast<double>(timeOutputs->timeUtc.sec); }
        if (descriptor == "Time::TimeUTC::ms") { return static_cast<double>(timeOutputs->timeUtc.ms); }
        if (descriptor == "Time::SyncInCnt") { return static_cast<double>(timeOutputs->syncInCnt); }
        if (descriptor == "Time::SyncOutCnt") { return static_cast<double>(timeOutputs->syncOutCnt); }
        if (descriptor == "Time::TimeStatus::timeOk") { return static_cast<double>(timeOutputs->timeStatus.timeOk()); }
        if (descriptor == "Time::TimeStatus::dateOk") { return static_cast<double>(timeOutputs->timeStatus.dateOk()); }
        if (descriptor == "Time::TimeStatus::utcTimeValid") { return static_cast<double>(timeOutputs->timeStatus.utcTimeValid()); }
        // Group 3 (IMU)
        if (descriptor == "IMU::ImuStatus") { return static_cast<double>(imuOutputs->imuStatus); }
        if (descriptor == "IMU::UncompMag::X [Gauss]") { return static_cast<double>(imuOutputs->uncompMag(0)); }
        if (descriptor == "IMU::UncompMag::Y [Gauss]") { return static_cast<double>(imuOutputs->uncompMag(1)); }
        if (descriptor == "IMU::UncompMag::Z [Gauss]") { return static_cast<double>(imuOutputs->uncompMag(2)); }
        if (descriptor == "IMU::UncompAccel::X [m/s^2]") { return static_cast<double>(imuOutputs->uncompAccel(0)); }
        if (descriptor == "IMU::UncompAccel::Y [m/s^2]") { return static_cast<double>(imuOutputs->uncompAccel(1)); }
        if (descriptor == "IMU::UncompAccel::Z [m/s^2]") { return static_cast<double>(imuOutputs->uncompAccel(2)); }
        if (descriptor == "IMU::UncompGyro::X [rad/s]") { return static_cast<double>(imuOutputs->uncompGyro(0)); }
        if (descriptor == "IMU::UncompGyro::Y [rad/s]") { return static_cast<double>(imuOutputs->uncompGyro(1)); }
        if (descriptor == "IMU::UncompGyro::Z [rad/s]") { return static_cast<double>(imuOutputs->uncompGyro(2)); }
        if (descriptor == "IMU::Temp [Celsius]") { return static_cast<double>(imuOutputs->temp); }
        if (descriptor == "IMU::Pres [kPa]") { return static_cast<double>(imuOutputs->pres); }
        if (descriptor == "IMU::DeltaTime [s]") { return static_cast<double>(imuOutputs->deltaTime); }
        if (descriptor == "IMU::DeltaTheta::X [deg]") { return static_cast<double>(imuOutputs->deltaTheta(0)); }
        if (descriptor == "IMU::DeltaTheta::Y [deg]") { return static_cast<double>(imuOutputs->deltaTheta(1)); }
        if (descriptor == "IMU::DeltaTheta::Z [deg]") { return static_cast<double>(imuOutputs->deltaTheta(2)); }
        if (descriptor == "IMU::DeltaVel::X [m/s]") { return static_cast<double>(imuOutputs->deltaV(0)); }
        if (descriptor == "IMU::DeltaVel::Y [m/s]") { return static_cast<double>(imuOutputs->deltaV(1)); }
        if (descriptor == "IMU::DeltaVel::Z [m/s]") { return static_cast<double>(imuOutputs->deltaV(2)); }
        if (descriptor == "IMU::Mag::X [Gauss]") { return static_cast<double>(imuOutputs->mag(0)); }
        if (descriptor == "IMU::Mag::Y [Gauss]") { return static_cast<double>(imuOutputs->mag(1)); }
        if (descriptor == "IMU::Mag::Z [Gauss]") { return static_cast<double>(imuOutputs->mag(2)); }
        if (descriptor == "IMU::Accel::X [m/s^2]") { return static_cast<double>(imuOutputs->accel(0)); }
        if (descriptor == "IMU::Accel::Y [m/s^2]") { return static_cast<double>(imuOutputs->accel(1)); }
        if (descriptor == "IMU::Accel::Z [m/s^2]") { return static_cast<double>(imuOutputs->accel(2)); }
        if (descriptor == "IMU::AngularRate::X [rad/s]") { return static_cast<double>(imuOutputs->angularRate(0)); }
        if (descriptor == "IMU::AngularRate::Y [rad/s]") { return static_cast<double>(imuOutputs->angularRate(1)); }
        if (descriptor == "IMU::AngularRate::Z [rad/s]") { return static_cast<double>(imuOutputs->angularRate(2)); }
        // Group 4 (GNSS1)
        if (descriptor == "GNSS1::UTC::year") { return static_cast<double>(gnss1Outputs->timeUtc.year); }
        if (descriptor == "GNSS1::UTC::month") { return static_cast<double>(gnss1Outputs->timeUtc.month); }
        if (descriptor == "GNSS1::UTC::day") { return static_cast<double>(gnss1Outputs->timeUtc.day); }
        if (descriptor == "GNSS1::UTC::hour") { return static_cast<double>(gnss1Outputs->timeUtc.hour); }
        if (descriptor == "GNSS1::UTC::min") { return static_cast<double>(gnss1Outputs->timeUtc.min); }
        if (descriptor == "GNSS1::UTC::sec") { return static_cast<double>(gnss1Outputs->timeUtc.sec); }
        if (descriptor == "GNSS1::UTC::ms") { return static_cast<double>(gnss1Outputs->timeUtc.ms); }
        if (descriptor == "GNSS1::Tow [ns]") { return static_cast<double>(gnss1Outputs->tow); }
        if (descriptor == "GNSS1::Week") { return static_cast<double>(gnss1Outputs->week); }
        if (descriptor == "GNSS1::NumSats") { return static_cast<double>(gnss1Outputs->numSats); }
        if (descriptor == "GNSS1::Fix") { return static_cast<double>(gnss1Outputs->fix); }
        if (descriptor == "GNSS1::PosLla::latitude [deg]") { return static_cast<double>(gnss1Outputs->posLla(0)); }
        if (descriptor == "GNSS1::PosLla::longitude [deg]") { return static_cast<double>(gnss1Outputs->posLla(1)); }
        if (descriptor == "GNSS1::PosLla::altitude [m]") { return static_cast<double>(gnss1Outputs->posLla(2)); }
        if (descriptor == "GNSS1::PosEcef::X [m]") { return static_cast<double>(gnss1Outputs->posEcef(0)); }
        if (descriptor == "GNSS1::PosEcef::Y [m]") { return static_cast<double>(gnss1Outputs->posEcef(1)); }
        if (descriptor == "GNSS1::PosEcef::Z [m]") { return static_cast<double>(gnss1Outputs->posEcef(2)); }
        if (descriptor == "GNSS1::VelNed::N [m/s]") { return static_cast<double>(gnss1Outputs->velNed(0)); }
        if (descriptor == "GNSS1::VelNed::E [m/s]") { return static_cast<double>(gnss1Outputs->velNed(1)); }
        if (descriptor == "GNSS1::VelNed::D [m/s]") { return static_cast<double>(gnss1Outputs->velNed(2)); }
        if (descriptor == "GNSS1::VelEcef::X [m/s]") { return static_cast<double>(gnss1Outputs->velEcef(0)); }
        if (descriptor == "GNSS1::VelEcef::Y [m/s]") { return static_cast<double>(gnss1Outputs->velEcef(1)); }
        if (descriptor == "GNSS1::VelEcef::Z [m/s]") { return static_cast<double>(gnss1Outputs->velEcef(2)); }
        if (descriptor == "GNSS1::PosU::N [m]") { return static_cast<double>(gnss1Outputs->posU(0)); }
        if (descriptor == "GNSS1::PosU::E [m]") { return static_cast<double>(gnss1Outputs->posU(1)); }
        if (descriptor == "GNSS1::PosU::D [m]") { return static_cast<double>(gnss1Outputs->posU(2)); }
        if (descriptor == "GNSS1::VelU [m/s]") { return static_cast<double>(gnss1Outputs->velU); }
        if (descriptor == "GNSS1::TimeU [s]") { return static_cast<double>(gnss1Outputs->timeU); }
        if (descriptor == "GNSS1::TimeInfo::Status::timeOk") { return static_cast<double>(gnss1Outputs->timeInfo.status.timeOk()); }
        if (descriptor == "GNSS1::TimeInfo::Status::dateOk") { return static_cast<double>(gnss1Outputs->timeInfo.status.dateOk()); }
        if (descriptor == "GNSS1::TimeInfo::Status::utcTimeValid") { return static_cast<double>(gnss1Outputs->timeInfo.status.utcTimeValid()); }
        if (descriptor == "GNSS1::TimeInfo::LeapSeconds") { return static_cast<double>(gnss1Outputs->timeInfo.leapSeconds); }
        if (descriptor == "GNSS1::DOP::g") { return static_cast<double>(gnss1Outputs->dop.gDop); }
        if (descriptor == "GNSS1::DOP::p") { return static_cast<double>(gnss1Outputs->dop.pDop); }
        if (descriptor == "GNSS1::DOP::t") { return static_cast<double>(gnss1Outputs->dop.tDop); }
        if (descriptor == "GNSS1::DOP::v") { return static_cast<double>(gnss1Outputs->dop.vDop); }
        if (descriptor == "GNSS1::DOP::h") { return static_cast<double>(gnss1Outputs->dop.hDop); }
        if (descriptor == "GNSS1::DOP::n") { return static_cast<double>(gnss1Outputs->dop.nDop); }
        if (descriptor == "GNSS1::DOP::e") { return static_cast<double>(gnss1Outputs->dop.eDop); }
        if (descriptor == "GNSS1::SatInfo::NumSats") { return static_cast<double>(gnss1Outputs->satInfo.numSats); }
        for (auto& satellite : gnss1Outputs->satInfo.satellites)
        {
            SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag Healthy", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Healthy) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag Almanac", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Almanac) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag Ephemeris", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag DifferentialCorrection", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag UsedForNavigation", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag AzimuthElevationValid", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - flag UsedForRTK", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - cno", satId)) { return static_cast<double>(satellite.cno); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - qi", satId)) { return static_cast<double>(satellite.qi); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - el", satId)) { return static_cast<double>(satellite.el); }
            if (descriptor == fmt::format("GNSS1::SatInfo::{} - az", satId)) { return static_cast<double>(satellite.az); }
        }
        if (descriptor == "GNSS1::RawMeas::Tow [s]") { return gnss1Outputs->raw.tow; }
        if (descriptor == "GNSS1::RawMeas::Week") { return static_cast<double>(gnss1Outputs->raw.week); }
        if (descriptor == "GNSS1::RawMeas::NumSats") { return static_cast<double>(gnss1Outputs->raw.numSats); }
        for (auto& satellite : gnss1Outputs->raw.satellites)
        {
            SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - freq", satId)) { return static_cast<double>(satellite.freq); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - chan", satId)) { return static_cast<double>(satellite.chan); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - slot", satId)) { return static_cast<double>(satellite.slot); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - cno", satId)) { return static_cast<double>(satellite.cno); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag Searching", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Searching) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag Tracking", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Tracking) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag TimeValid", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::TimeValid) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag CodeLock", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::CodeLock) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag PhaseLock", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag PhaseHalfAmbiguity", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag PhaseHalfSub", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag PhaseSlip", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - flag PseudorangeSmoothed", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - pr", satId)) { return satellite.pr; }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - cp", satId)) { return satellite.cp; }
            if (descriptor == fmt::format("GNSS1::RawMeas::{} - dp", satId)) { return static_cast<double>(satellite.dp); }
        }
        // Group 5 (Attitude)
        if (descriptor == "Att::VpeStatus::AttitudeQuality") { return static_cast<double>(attitudeOutputs->vpeStatus.attitudeQuality()); }
        if (descriptor == "Att::VpeStatus::GyroSaturation") { return static_cast<double>(attitudeOutputs->vpeStatus.gyroSaturation()); }
        if (descriptor == "Att::VpeStatus::GyroSaturationRecovery") { return static_cast<double>(attitudeOutputs->vpeStatus.gyroSaturationRecovery()); }
        if (descriptor == "Att::VpeStatus::MagDisturbance") { return static_cast<double>(attitudeOutputs->vpeStatus.magDisturbance()); }
        if (descriptor == "Att::VpeStatus::MagSaturation") { return static_cast<double>(attitudeOutputs->vpeStatus.magSaturation()); }
        if (descriptor == "Att::VpeStatus::AccDisturbance") { return static_cast<double>(attitudeOutputs->vpeStatus.accDisturbance()); }
        if (descriptor == "Att::VpeStatus::AccSaturation") { return static_cast<double>(attitudeOutputs->vpeStatus.accSaturation()); }
        if (descriptor == "Att::VpeStatus::KnownMagDisturbance") { return static_cast<double>(attitudeOutputs->vpeStatus.knownMagDisturbance()); }
        if (descriptor == "Att::VpeStatus::KnownAccelDisturbance") { return static_cast<double>(attitudeOutputs->vpeStatus.knownAccelDisturbance()); }
        if (descriptor == "Att::YawPitchRoll::Y [deg]") { return static_cast<double>(attitudeOutputs->ypr(0)); }
        if (descriptor == "Att::YawPitchRoll::P [deg]") { return static_cast<double>(attitudeOutputs->ypr(1)); }
        if (descriptor == "Att::YawPitchRoll::R [deg]") { return static_cast<double>(attitudeOutputs->ypr(2)); }
        if (descriptor == "Att::Quaternion::w") { return static_cast<double>(attitudeOutputs->qtn.w()); }
        if (descriptor == "Att::Quaternion::x") { return static_cast<double>(attitudeOutputs->qtn.x()); }
        if (descriptor == "Att::Quaternion::y") { return static_cast<double>(attitudeOutputs->qtn.y()); }
        if (descriptor == "Att::Quaternion::z") { return static_cast<double>(attitudeOutputs->qtn.z()); }
        if (descriptor == "Att::DCM::0-0") { return static_cast<double>(attitudeOutputs->dcm(0, 0)); }
        if (descriptor == "Att::DCM::0-1") { return static_cast<double>(attitudeOutputs->dcm(0, 1)); }
        if (descriptor == "Att::DCM::0-2") { return static_cast<double>(attitudeOutputs->dcm(0, 2)); }
        if (descriptor == "Att::DCM::1-0") { return static_cast<double>(attitudeOutputs->dcm(1, 0)); }
        if (descriptor == "Att::DCM::1-1") { return static_cast<double>(attitudeOutputs->dcm(1, 1)); }
        if (descriptor == "Att::DCM::1-2") { return static_cast<double>(attitudeOutputs->dcm(1, 2)); }
        if (descriptor == "Att::DCM::2-0") { return static_cast<double>(attitudeOutputs->dcm(2, 0)); }
        if (descriptor == "Att::DCM::2-1") { return static_cast<double>(attitudeOutputs->dcm(2, 1)); }
        if (descriptor == "Att::DCM::2-2") { return static_cast<double>(attitudeOutputs->dcm(2, 2)); }
        if (descriptor == "Att::MagNed::N [Gauss]") { return static_cast<double>(attitudeOutputs->magNed(0)); }
        if (descriptor == "Att::MagNed::E [Gauss]") { return static_cast<double>(attitudeOutputs->magNed(1)); }
        if (descriptor == "Att::MagNed::D [Gauss]") { return static_cast<double>(attitudeOutputs->magNed(2)); }
        if (descriptor == "Att::AccelNed::N [m/s^2]") { return static_cast<double>(attitudeOutputs->accelNed(0)); }
        if (descriptor == "Att::AccelNed::E [m/s^2]") { return static_cast<double>(attitudeOutputs->accelNed(1)); }
        if (descriptor == "Att::AccelNed::D [m/s^2]") { return static_cast<double>(attitudeOutputs->accelNed(2)); }
        if (descriptor == "Att::LinearAccelBody::X [m/s^2]") { return static_cast<double>(attitudeOutputs->linearAccelBody(0)); }
        if (descriptor == "Att::LinearAccelBody::Y [m/s^2]") { return static_cast<double>(attitudeOutputs->linearAccelBody(1)); }
        if (descriptor == "Att::LinearAccelBody::Z [m/s^2]") { return static_cast<double>(attitudeOutputs->linearAccelBody(2)); }
        if (descriptor == "Att::LinearAccelNed::N [m/s^2]") { return static_cast<double>(attitudeOutputs->linearAccelNed(0)); }
        if (descriptor == "Att::LinearAccelNed::E [m/s^2]") { return static_cast<double>(attitudeOutputs->linearAccelNed(1)); }
        if (descriptor == "Att::LinearAccelNed::D [m/s^2]") { return static_cast<double>(attitudeOutputs->linearAccelNed(2)); }
        if (descriptor == "Att::YprU::Y [deg]") { return static_cast<double>(attitudeOutputs->yprU(0)); }
        if (descriptor == "Att::YprU::P [deg]") { return static_cast<double>(attitudeOutputs->yprU(1)); }
        if (descriptor == "Att::YprU::R [deg]") { return static_cast<double>(attitudeOutputs->yprU(2)); }

        // Group 6 (INS)
        if (descriptor == "INS::InsStatus::Mode") { return static_cast<double>(insOutputs->insStatus.mode()); }
        if (descriptor == "INS::InsStatus::GpsFix") { return static_cast<double>(insOutputs->insStatus.gpsFix()); }
        if (descriptor == "INS::InsStatus::Error::IMU") { return static_cast<double>(insOutputs->insStatus.errorIMU()); }
        if (descriptor == "INS::InsStatus::Error::MagPres") { return static_cast<double>(insOutputs->insStatus.errorMagPres()); }
        if (descriptor == "INS::InsStatus::Error::GNSS") { return static_cast<double>(insOutputs->insStatus.errorGnss()); }
        if (descriptor == "INS::InsStatus::GpsHeadingIns") { return static_cast<double>(insOutputs->insStatus.gpsHeadingIns()); }
        if (descriptor == "INS::InsStatus::GpsCompass") { return static_cast<double>(insOutputs->insStatus.gpsCompass()); }
        if (descriptor == "INS::PosLla::latitude [deg]") { return static_cast<double>(insOutputs->posLla(0)); }
        if (descriptor == "INS::PosLla::longitude [deg]") { return static_cast<double>(insOutputs->posLla(1)); }
        if (descriptor == "INS::PosLla::altitude [m]") { return static_cast<double>(insOutputs->posLla(2)); }
        if (descriptor == "INS::PosEcef::X [m]") { return static_cast<double>(insOutputs->posEcef(0)); }
        if (descriptor == "INS::PosEcef::Y [m]") { return static_cast<double>(insOutputs->posEcef(1)); }
        if (descriptor == "INS::PosEcef::Z [m]") { return static_cast<double>(insOutputs->posEcef(2)); }
        if (descriptor == "INS::VelBody::X [m/s]") { return static_cast<double>(insOutputs->velBody(0)); }
        if (descriptor == "INS::VelBody::Y [m/s]") { return static_cast<double>(insOutputs->velBody(1)); }
        if (descriptor == "INS::VelBody::Z [m/s]") { return static_cast<double>(insOutputs->velBody(2)); }
        if (descriptor == "INS::VelNed::N [m/s]") { return static_cast<double>(insOutputs->velNed(0)); }
        if (descriptor == "INS::VelNed::E [m/s]") { return static_cast<double>(insOutputs->velNed(1)); }
        if (descriptor == "INS::VelNed::D [m/s]") { return static_cast<double>(insOutputs->velNed(2)); }
        if (descriptor == "INS::VelEcef::X [m/s]") { return static_cast<double>(insOutputs->velEcef(0)); }
        if (descriptor == "INS::VelEcef::Y [m/s]") { return static_cast<double>(insOutputs->velEcef(1)); }
        if (descriptor == "INS::VelEcef::Z [m/s]") { return static_cast<double>(insOutputs->velEcef(2)); }
        if (descriptor == "INS::MagEcef::X [Gauss]") { return static_cast<double>(insOutputs->magEcef(0)); }
        if (descriptor == "INS::MagEcef::Y [Gauss]") { return static_cast<double>(insOutputs->magEcef(1)); }
        if (descriptor == "INS::MagEcef::Z [Gauss]") { return static_cast<double>(insOutputs->magEcef(2)); }
        if (descriptor == "INS::AccelEcef::X [m/s^2]") { return static_cast<double>(insOutputs->accelEcef(0)); }
        if (descriptor == "INS::AccelEcef::Y [m/s^2]") { return static_cast<double>(insOutputs->accelEcef(1)); }
        if (descriptor == "INS::AccelEcef::Z [m/s^2]") { return static_cast<double>(insOutputs->accelEcef(2)); }
        if (descriptor == "INS::LinearAccelEcef::X [m/s^2]") { return static_cast<double>(insOutputs->linearAccelEcef(0)); }
        if (descriptor == "INS::LinearAccelEcef::Y [m/s^2]") { return static_cast<double>(insOutputs->linearAccelEcef(1)); }
        if (descriptor == "INS::LinearAccelEcef::Z [m/s^2]") { return static_cast<double>(insOutputs->linearAccelEcef(2)); }
        if (descriptor == "INS::PosU [m]") { return static_cast<double>(insOutputs->posU); }
        if (descriptor == "INS::VelU [m/s]") { return static_cast<double>(insOutputs->velU); }

        // Group 7 (GNSS2)
        if (descriptor == "GNSS2::UTC::year") { return static_cast<double>(gnss2Outputs->timeUtc.year); }
        if (descriptor == "GNSS2::UTC::month") { return static_cast<double>(gnss2Outputs->timeUtc.month); }
        if (descriptor == "GNSS2::UTC::day") { return static_cast<double>(gnss2Outputs->timeUtc.day); }
        if (descriptor == "GNSS2::UTC::hour") { return static_cast<double>(gnss2Outputs->timeUtc.hour); }
        if (descriptor == "GNSS2::UTC::min") { return static_cast<double>(gnss2Outputs->timeUtc.min); }
        if (descriptor == "GNSS2::UTC::sec") { return static_cast<double>(gnss2Outputs->timeUtc.sec); }
        if (descriptor == "GNSS2::UTC::ms") { return static_cast<double>(gnss2Outputs->timeUtc.ms); }
        if (descriptor == "GNSS2::Tow [ns]") { return static_cast<double>(gnss2Outputs->tow); }
        if (descriptor == "GNSS2::Week") { return static_cast<double>(gnss2Outputs->week); }
        if (descriptor == "GNSS2::NumSats") { return static_cast<double>(gnss2Outputs->numSats); }
        if (descriptor == "GNSS2::Fix") { return static_cast<double>(gnss2Outputs->fix); }
        if (descriptor == "GNSS2::PosLla::latitude [deg]") { return static_cast<double>(gnss2Outputs->posLla(0)); }
        if (descriptor == "GNSS2::PosLla::longitude [deg]") { return static_cast<double>(gnss2Outputs->posLla(1)); }
        if (descriptor == "GNSS2::PosLla::altitude [m]") { return static_cast<double>(gnss2Outputs->posLla(2)); }
        if (descriptor == "GNSS2::PosEcef::X [m]") { return static_cast<double>(gnss2Outputs->posEcef(0)); }
        if (descriptor == "GNSS2::PosEcef::Y [m]") { return static_cast<double>(gnss2Outputs->posEcef(1)); }
        if (descriptor == "GNSS2::PosEcef::Z [m]") { return static_cast<double>(gnss2Outputs->posEcef(2)); }
        if (descriptor == "GNSS2::VelNed::N [m/s]") { return static_cast<double>(gnss2Outputs->velNed(0)); }
        if (descriptor == "GNSS2::VelNed::E [m/s]") { return static_cast<double>(gnss2Outputs->velNed(1)); }
        if (descriptor == "GNSS2::VelNed::D [m/s]") { return static_cast<double>(gnss2Outputs->velNed(2)); }
        if (descriptor == "GNSS2::VelEcef::X [m/s]") { return static_cast<double>(gnss2Outputs->velEcef(0)); }
        if (descriptor == "GNSS2::VelEcef::Y [m/s]") { return static_cast<double>(gnss2Outputs->velEcef(1)); }
        if (descriptor == "GNSS2::VelEcef::Z [m/s]") { return static_cast<double>(gnss2Outputs->velEcef(2)); }
        if (descriptor == "GNSS2::PosU::N [m]") { return static_cast<double>(gnss2Outputs->posU(0)); }
        if (descriptor == "GNSS2::PosU::E [m]") { return static_cast<double>(gnss2Outputs->posU(1)); }
        if (descriptor == "GNSS2::PosU::D [m]") { return static_cast<double>(gnss2Outputs->posU(2)); }
        if (descriptor == "GNSS2::VelU [m/s]") { return static_cast<double>(gnss2Outputs->velU); }
        if (descriptor == "GNSS2::TimeU [s]") { return static_cast<double>(gnss2Outputs->timeU); }
        if (descriptor == "GNSS2::TimeInfo::Status::timeOk") { return static_cast<double>(gnss2Outputs->timeInfo.status.timeOk()); }
        if (descriptor == "GNSS2::TimeInfo::Status::dateOk") { return static_cast<double>(gnss2Outputs->timeInfo.status.dateOk()); }
        if (descriptor == "GNSS2::TimeInfo::Status::utcTimeValid") { return static_cast<double>(gnss2Outputs->timeInfo.status.utcTimeValid()); }
        if (descriptor == "GNSS2::TimeInfo::LeapSeconds") { return static_cast<double>(gnss2Outputs->timeInfo.leapSeconds); }
        if (descriptor == "GNSS2::DOP::g") { return static_cast<double>(gnss2Outputs->dop.gDop); }
        if (descriptor == "GNSS2::DOP::p") { return static_cast<double>(gnss2Outputs->dop.pDop); }
        if (descriptor == "GNSS2::DOP::t") { return static_cast<double>(gnss2Outputs->dop.tDop); }
        if (descriptor == "GNSS2::DOP::v") { return static_cast<double>(gnss2Outputs->dop.vDop); }
        if (descriptor == "GNSS2::DOP::h") { return static_cast<double>(gnss2Outputs->dop.hDop); }
        if (descriptor == "GNSS2::DOP::n") { return static_cast<double>(gnss2Outputs->dop.nDop); }
        if (descriptor == "GNSS2::DOP::e") { return static_cast<double>(gnss2Outputs->dop.eDop); }
        if (descriptor == "GNSS2::SatInfo::NumSats") { return static_cast<double>(gnss2Outputs->satInfo.numSats); }
        for (auto& satellite : gnss2Outputs->satInfo.satellites)
        {
            SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag Healthy", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Healthy) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag Almanac", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Almanac) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag Ephemeris", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag DifferentialCorrection", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag UsedForNavigation", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag AzimuthElevationValid", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - flag UsedForRTK", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - cno", satId)) { return static_cast<double>(satellite.cno); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - qi", satId)) { return static_cast<double>(satellite.qi); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - el", satId)) { return static_cast<double>(satellite.el); }
            if (descriptor == fmt::format("GNSS2::SatInfo::{} - az", satId)) { return static_cast<double>(satellite.az); }
        }
        if (descriptor == "GNSS2::RawMeas::Tow [s]") { return gnss2Outputs->raw.tow; }
        if (descriptor == "GNSS2::RawMeas::Week") { return static_cast<double>(gnss2Outputs->raw.week); }
        if (descriptor == "GNSS2::RawMeas::NumSats") { return static_cast<double>(gnss2Outputs->raw.numSats); }
        for (auto& satellite : gnss2Outputs->raw.satellites)
        {
            SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - freq", satId)) { return static_cast<double>(satellite.freq); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - chan", satId)) { return static_cast<double>(satellite.chan); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - slot", satId)) { return static_cast<double>(satellite.slot); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - cno", satId)) { return static_cast<double>(satellite.cno); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag Searching", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Searching) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag Tracking", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Tracking) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag TimeValid", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::TimeValid) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag CodeLock", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::CodeLock) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag PhaseLock", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag PhaseHalfAmbiguity", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag PhaseHalfSub", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag PhaseSlip", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - flag PseudorangeSmoothed", satId)) { return static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) ? 1 : 0); }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - pr", satId)) { return satellite.pr; }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - cp", satId)) { return satellite.cp; }
            if (descriptor == fmt::format("GNSS2::RawMeas::{} - dp", satId)) { return static_cast<double>(satellite.dp); }
        }

        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;

        // Group 2 (Time)
        if (timeOutputs)
        {
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
            {
                dynData.emplace_back("Time::TimeStartup [ns]", static_cast<double>(timeOutputs->timeStartup));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
            {
                dynData.emplace_back("Time::TimeGps [ns]", static_cast<double>(timeOutputs->timeGps));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
            {
                dynData.emplace_back("Time::GpsTow [ns]", static_cast<double>(timeOutputs->gpsTow));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
            {
                dynData.emplace_back("Time::GpsWeek", static_cast<double>(timeOutputs->gpsWeek));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
            {
                dynData.emplace_back("Time::TimeSyncIn [ns]", static_cast<double>(timeOutputs->timeSyncIn));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
            {
                dynData.emplace_back("Time::TimeGpsPps [ns]", static_cast<double>(timeOutputs->timePPS));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
            {
                dynData.emplace_back("Time::TimeUTC::year", static_cast<double>(timeOutputs->timeUtc.year));
                dynData.emplace_back("Time::TimeUTC::month", static_cast<double>(timeOutputs->timeUtc.month));
                dynData.emplace_back("Time::TimeUTC::day", static_cast<double>(timeOutputs->timeUtc.day));
                dynData.emplace_back("Time::TimeUTC::hour", static_cast<double>(timeOutputs->timeUtc.hour));
                dynData.emplace_back("Time::TimeUTC::min", static_cast<double>(timeOutputs->timeUtc.min));
                dynData.emplace_back("Time::TimeUTC::sec", static_cast<double>(timeOutputs->timeUtc.sec));
                dynData.emplace_back("Time::TimeUTC::ms", static_cast<double>(timeOutputs->timeUtc.ms));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
            {
                dynData.emplace_back("Time::SyncInCnt", static_cast<double>(timeOutputs->syncInCnt));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
            {
                dynData.emplace_back("Time::SyncOutCnt", static_cast<double>(timeOutputs->syncOutCnt));
            }
            if (timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
            {
                dynData.emplace_back("Time::TimeStatus::timeOk", static_cast<double>(timeOutputs->timeStatus.timeOk()));
                dynData.emplace_back("Time::TimeStatus::dateOk", static_cast<double>(timeOutputs->timeStatus.dateOk()));
                dynData.emplace_back("Time::TimeStatus::utcTimeValid", static_cast<double>(timeOutputs->timeStatus.utcTimeValid()));
            }
        }
        // Group 3 (IMU)
        if (imuOutputs)
        {
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
            {
                dynData.emplace_back("IMU::ImuStatus", static_cast<double>(imuOutputs->imuStatus));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
            {
                dynData.emplace_back("IMU::UncompMag::X [Gauss]", static_cast<double>(imuOutputs->uncompMag(0)));
                dynData.emplace_back("IMU::UncompMag::Y [Gauss]", static_cast<double>(imuOutputs->uncompMag(1)));
                dynData.emplace_back("IMU::UncompMag::Z [Gauss]", static_cast<double>(imuOutputs->uncompMag(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
            {
                dynData.emplace_back("IMU::UncompAccel::X [m/s^2]", static_cast<double>(imuOutputs->uncompAccel(0)));
                dynData.emplace_back("IMU::UncompAccel::Y [m/s^2]", static_cast<double>(imuOutputs->uncompAccel(1)));
                dynData.emplace_back("IMU::UncompAccel::Z [m/s^2]", static_cast<double>(imuOutputs->uncompAccel(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
            {
                dynData.emplace_back("IMU::UncompGyro::X [rad/s]", static_cast<double>(imuOutputs->uncompGyro(0)));
                dynData.emplace_back("IMU::UncompGyro::Y [rad/s]", static_cast<double>(imuOutputs->uncompGyro(1)));
                dynData.emplace_back("IMU::UncompGyro::Z [rad/s]", static_cast<double>(imuOutputs->uncompGyro(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
            {
                dynData.emplace_back("IMU::Temp [Celsius]", static_cast<double>(imuOutputs->temp));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
            {
                dynData.emplace_back("IMU::Pres [kPa]", static_cast<double>(imuOutputs->pres));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
            {
                dynData.emplace_back("IMU::DeltaTime [s]", static_cast<double>(imuOutputs->deltaTime));
                dynData.emplace_back("IMU::DeltaTheta::X [deg]", static_cast<double>(imuOutputs->deltaTheta(0)));
                dynData.emplace_back("IMU::DeltaTheta::Y [deg]", static_cast<double>(imuOutputs->deltaTheta(1)));
                dynData.emplace_back("IMU::DeltaTheta::Z [deg]", static_cast<double>(imuOutputs->deltaTheta(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
            {
                dynData.emplace_back("IMU::DeltaVel::X [m/s]", static_cast<double>(imuOutputs->deltaV(0)));
                dynData.emplace_back("IMU::DeltaVel::Y [m/s]", static_cast<double>(imuOutputs->deltaV(1)));
                dynData.emplace_back("IMU::DeltaVel::Z [m/s]", static_cast<double>(imuOutputs->deltaV(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
            {
                dynData.emplace_back("IMU::Mag::X [Gauss]", static_cast<double>(imuOutputs->mag(0)));
                dynData.emplace_back("IMU::Mag::Y [Gauss]", static_cast<double>(imuOutputs->mag(1)));
                dynData.emplace_back("IMU::Mag::Z [Gauss]", static_cast<double>(imuOutputs->mag(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
            {
                dynData.emplace_back("IMU::Accel::X [m/s^2]", static_cast<double>(imuOutputs->accel(0)));
                dynData.emplace_back("IMU::Accel::Y [m/s^2]", static_cast<double>(imuOutputs->accel(1)));
                dynData.emplace_back("IMU::Accel::Z [m/s^2]", static_cast<double>(imuOutputs->accel(2)));
            }
            if (imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
            {
                dynData.emplace_back("IMU::AngularRate::X [rad/s]", static_cast<double>(imuOutputs->angularRate(0)));
                dynData.emplace_back("IMU::AngularRate::Y [rad/s]", static_cast<double>(imuOutputs->angularRate(1)));
                dynData.emplace_back("IMU::AngularRate::Z [rad/s]", static_cast<double>(imuOutputs->angularRate(2)));
            }
        }

        // Group 4 (GNSS1)
        if (gnss1Outputs)
        {
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                dynData.emplace_back("GNSS1::UTC::year", static_cast<double>(gnss1Outputs->timeUtc.year));
                dynData.emplace_back("GNSS1::UTC::month", static_cast<double>(gnss1Outputs->timeUtc.month));
                dynData.emplace_back("GNSS1::UTC::day", static_cast<double>(gnss1Outputs->timeUtc.day));
                dynData.emplace_back("GNSS1::UTC::hour", static_cast<double>(gnss1Outputs->timeUtc.hour));
                dynData.emplace_back("GNSS1::UTC::min", static_cast<double>(gnss1Outputs->timeUtc.min));
                dynData.emplace_back("GNSS1::UTC::sec", static_cast<double>(gnss1Outputs->timeUtc.sec));
                dynData.emplace_back("GNSS1::UTC::ms", static_cast<double>(gnss1Outputs->timeUtc.ms));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                dynData.emplace_back("GNSS1::Tow [ns]", static_cast<double>(gnss1Outputs->tow));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                dynData.emplace_back("GNSS1::Week", static_cast<double>(gnss1Outputs->week));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                dynData.emplace_back("GNSS1::NumSats", static_cast<double>(gnss1Outputs->numSats));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                dynData.emplace_back("GNSS1::Fix", static_cast<double>(gnss1Outputs->fix));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                dynData.emplace_back("GNSS1::PosLla::latitude [deg]", static_cast<double>(gnss1Outputs->posLla(0)));
                dynData.emplace_back("GNSS1::PosLla::longitude [deg]", static_cast<double>(gnss1Outputs->posLla(1)));
                dynData.emplace_back("GNSS1::PosLla::altitude [m]", static_cast<double>(gnss1Outputs->posLla(2)));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                dynData.emplace_back("GNSS1::PosEcef::X [m]", static_cast<double>(gnss1Outputs->posEcef(0)));
                dynData.emplace_back("GNSS1::PosEcef::Y [m]", static_cast<double>(gnss1Outputs->posEcef(1)));
                dynData.emplace_back("GNSS1::PosEcef::Z [m]", static_cast<double>(gnss1Outputs->posEcef(2)));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                dynData.emplace_back("GNSS1::VelNed::N [m/s]", static_cast<double>(gnss1Outputs->velNed(0)));
                dynData.emplace_back("GNSS1::VelNed::E [m/s]", static_cast<double>(gnss1Outputs->velNed(1)));
                dynData.emplace_back("GNSS1::VelNed::D [m/s]", static_cast<double>(gnss1Outputs->velNed(2)));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                dynData.emplace_back("GNSS1::VelEcef::X [m/s]", static_cast<double>(gnss1Outputs->velEcef(0)));
                dynData.emplace_back("GNSS1::VelEcef::Y [m/s]", static_cast<double>(gnss1Outputs->velEcef(1)));
                dynData.emplace_back("GNSS1::VelEcef::Z [m/s]", static_cast<double>(gnss1Outputs->velEcef(2)));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                dynData.emplace_back("GNSS1::PosU::N [m]", static_cast<double>(gnss1Outputs->posU(0)));
                dynData.emplace_back("GNSS1::PosU::E [m]", static_cast<double>(gnss1Outputs->posU(1)));
                dynData.emplace_back("GNSS1::PosU::D [m]", static_cast<double>(gnss1Outputs->posU(2)));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                dynData.emplace_back("GNSS1::VelU [m/s]", static_cast<double>(gnss1Outputs->velU));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                dynData.emplace_back("GNSS1::TimeU [s]", static_cast<double>(gnss1Outputs->timeU));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                dynData.emplace_back("GNSS1::TimeInfo::Status::timeOk", static_cast<double>(gnss1Outputs->timeInfo.status.timeOk()));
                dynData.emplace_back("GNSS1::TimeInfo::Status::dateOk", static_cast<double>(gnss1Outputs->timeInfo.status.dateOk()));
                dynData.emplace_back("GNSS1::TimeInfo::Status::utcTimeValid", static_cast<double>(gnss1Outputs->timeInfo.status.utcTimeValid()));
                dynData.emplace_back("GNSS1::TimeInfo::LeapSeconds", static_cast<double>(gnss1Outputs->timeInfo.leapSeconds));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                dynData.emplace_back("GNSS1::DOP::g", static_cast<double>(gnss1Outputs->dop.gDop));
                dynData.emplace_back("GNSS1::DOP::p", static_cast<double>(gnss1Outputs->dop.pDop));
                dynData.emplace_back("GNSS1::DOP::t", static_cast<double>(gnss1Outputs->dop.tDop));
                dynData.emplace_back("GNSS1::DOP::v", static_cast<double>(gnss1Outputs->dop.vDop));
                dynData.emplace_back("GNSS1::DOP::h", static_cast<double>(gnss1Outputs->dop.hDop));
                dynData.emplace_back("GNSS1::DOP::n", static_cast<double>(gnss1Outputs->dop.nDop));
                dynData.emplace_back("GNSS1::DOP::e", static_cast<double>(gnss1Outputs->dop.eDop));
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                dynData.emplace_back("GNSS1::SatInfo::NumSats", static_cast<double>(gnss1Outputs->satInfo.numSats));
                for (auto& satellite : gnss1Outputs->satInfo.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag Healthy", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Healthy) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag Almanac", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Almanac) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag Ephemeris", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag DifferentialCorrection", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag UsedForNavigation", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag AzimuthElevationValid", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - flag UsedForRTK", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - cno", satId), static_cast<double>(satellite.cno));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - qi", satId), static_cast<double>(satellite.qi));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - el", satId), static_cast<double>(satellite.el));
                    dynData.emplace_back(fmt::format("GNSS1::SatInfo::{} - az", satId), static_cast<double>(satellite.az));
                }
            }
            if (gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                dynData.emplace_back("GNSS1::RawMeas::Tow [s]", gnss1Outputs->raw.tow);
                dynData.emplace_back("GNSS1::RawMeas::Week", static_cast<double>(gnss1Outputs->raw.week));
                dynData.emplace_back("GNSS1::RawMeas::NumSats", static_cast<double>(gnss1Outputs->raw.numSats));
                for (auto& satellite : gnss1Outputs->raw.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - freq", satId), static_cast<double>(satellite.freq));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - chan", satId), static_cast<double>(satellite.chan));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - slot", satId), static_cast<double>(satellite.slot));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - cno", satId), static_cast<double>(satellite.cno));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag Searching", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Searching) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag Tracking", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Tracking) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag TimeValid", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::TimeValid) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag CodeLock", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::CodeLock) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseLock", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseHalfAmbiguity", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseHalfSub", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag PhaseSlip", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - flag PseudorangeSmoothed", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - pr", satId), satellite.pr);
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - cp", satId), satellite.cp);
                    dynData.emplace_back(fmt::format("GNSS1::RawMeas::{} - dp", satId), static_cast<double>(satellite.dp));
                }
            }
        }

        // Group 5 (Attitude)
        if (attitudeOutputs)
        {
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
            {
                dynData.emplace_back("Att::VpeStatus::AttitudeQuality", static_cast<double>(attitudeOutputs->vpeStatus.attitudeQuality()));
                dynData.emplace_back("Att::VpeStatus::GyroSaturation", static_cast<double>(attitudeOutputs->vpeStatus.gyroSaturation()));
                dynData.emplace_back("Att::VpeStatus::GyroSaturationRecovery", static_cast<double>(attitudeOutputs->vpeStatus.gyroSaturationRecovery()));
                dynData.emplace_back("Att::VpeStatus::MagDisturbance", static_cast<double>(attitudeOutputs->vpeStatus.magDisturbance()));
                dynData.emplace_back("Att::VpeStatus::MagSaturation", static_cast<double>(attitudeOutputs->vpeStatus.magSaturation()));
                dynData.emplace_back("Att::VpeStatus::AccDisturbance", static_cast<double>(attitudeOutputs->vpeStatus.accDisturbance()));
                dynData.emplace_back("Att::VpeStatus::AccSaturation", static_cast<double>(attitudeOutputs->vpeStatus.accSaturation()));
                dynData.emplace_back("Att::VpeStatus::KnownMagDisturbance", static_cast<double>(attitudeOutputs->vpeStatus.knownMagDisturbance()));
                dynData.emplace_back("Att::VpeStatus::KnownAccelDisturbance", static_cast<double>(attitudeOutputs->vpeStatus.knownAccelDisturbance()));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
            {
                dynData.emplace_back("Att::YawPitchRoll::Y [deg]", static_cast<double>(attitudeOutputs->ypr(0)));
                dynData.emplace_back("Att::YawPitchRoll::P [deg]", static_cast<double>(attitudeOutputs->ypr(1)));
                dynData.emplace_back("Att::YawPitchRoll::R [deg]", static_cast<double>(attitudeOutputs->ypr(2)));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
            {
                dynData.emplace_back("Att::Quaternion::w", static_cast<double>(attitudeOutputs->qtn.w()));
                dynData.emplace_back("Att::Quaternion::x", static_cast<double>(attitudeOutputs->qtn.x()));
                dynData.emplace_back("Att::Quaternion::y", static_cast<double>(attitudeOutputs->qtn.y()));
                dynData.emplace_back("Att::Quaternion::z", static_cast<double>(attitudeOutputs->qtn.z()));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
            {
                dynData.emplace_back("Att::DCM::0-0", static_cast<double>(attitudeOutputs->dcm(0, 0)));
                dynData.emplace_back("Att::DCM::0-1", static_cast<double>(attitudeOutputs->dcm(0, 1)));
                dynData.emplace_back("Att::DCM::0-2", static_cast<double>(attitudeOutputs->dcm(0, 2)));
                dynData.emplace_back("Att::DCM::1-0", static_cast<double>(attitudeOutputs->dcm(1, 0)));
                dynData.emplace_back("Att::DCM::1-1", static_cast<double>(attitudeOutputs->dcm(1, 1)));
                dynData.emplace_back("Att::DCM::1-2", static_cast<double>(attitudeOutputs->dcm(1, 2)));
                dynData.emplace_back("Att::DCM::2-0", static_cast<double>(attitudeOutputs->dcm(2, 0)));
                dynData.emplace_back("Att::DCM::2-1", static_cast<double>(attitudeOutputs->dcm(2, 1)));
                dynData.emplace_back("Att::DCM::2-2", static_cast<double>(attitudeOutputs->dcm(2, 2)));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
            {
                dynData.emplace_back("Att::MagNed::N [Gauss]", static_cast<double>(attitudeOutputs->magNed(0)));
                dynData.emplace_back("Att::MagNed::E [Gauss]", static_cast<double>(attitudeOutputs->magNed(1)));
                dynData.emplace_back("Att::MagNed::D [Gauss]", static_cast<double>(attitudeOutputs->magNed(2)));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
            {
                dynData.emplace_back("Att::AccelNed::N [m/s^2]", static_cast<double>(attitudeOutputs->accelNed(0)));
                dynData.emplace_back("Att::AccelNed::E [m/s^2]", static_cast<double>(attitudeOutputs->accelNed(1)));
                dynData.emplace_back("Att::AccelNed::D [m/s^2]", static_cast<double>(attitudeOutputs->accelNed(2)));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
            {
                dynData.emplace_back("Att::LinearAccelBody::X [m/s^2]", static_cast<double>(attitudeOutputs->linearAccelBody(0)));
                dynData.emplace_back("Att::LinearAccelBody::Y [m/s^2]", static_cast<double>(attitudeOutputs->linearAccelBody(1)));
                dynData.emplace_back("Att::LinearAccelBody::Z [m/s^2]", static_cast<double>(attitudeOutputs->linearAccelBody(2)));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
            {
                dynData.emplace_back("Att::LinearAccelNed::N [m/s^2]", static_cast<double>(attitudeOutputs->linearAccelNed(0)));
                dynData.emplace_back("Att::LinearAccelNed::E [m/s^2]", static_cast<double>(attitudeOutputs->linearAccelNed(1)));
                dynData.emplace_back("Att::LinearAccelNed::D [m/s^2]", static_cast<double>(attitudeOutputs->linearAccelNed(2)));
            }
            if (attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
            {
                dynData.emplace_back("Att::YprU::Y [deg]", static_cast<double>(attitudeOutputs->yprU(0)));
                dynData.emplace_back("Att::YprU::P [deg]", static_cast<double>(attitudeOutputs->yprU(1)));
                dynData.emplace_back("Att::YprU::R [deg]", static_cast<double>(attitudeOutputs->yprU(2)));
            }
        }

        // Group 6 (INS)
        if (insOutputs)
        {
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
            {
                dynData.emplace_back("INS::InsStatus::Mode", static_cast<double>(insOutputs->insStatus.mode()));
                dynData.emplace_back("INS::InsStatus::GpsFix", static_cast<double>(insOutputs->insStatus.gpsFix()));
                dynData.emplace_back("INS::InsStatus::Error::IMU", static_cast<double>(insOutputs->insStatus.errorIMU()));
                dynData.emplace_back("INS::InsStatus::Error::MagPres", static_cast<double>(insOutputs->insStatus.errorMagPres()));
                dynData.emplace_back("INS::InsStatus::Error::GNSS", static_cast<double>(insOutputs->insStatus.errorGnss()));
                dynData.emplace_back("INS::InsStatus::GpsHeadingIns", static_cast<double>(insOutputs->insStatus.gpsHeadingIns()));
                dynData.emplace_back("INS::InsStatus::GpsCompass", static_cast<double>(insOutputs->insStatus.gpsCompass()));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
            {
                dynData.emplace_back("INS::PosLla::latitude [deg]", static_cast<double>(insOutputs->posLla(0)));
                dynData.emplace_back("INS::PosLla::longitude [deg]", static_cast<double>(insOutputs->posLla(1)));
                dynData.emplace_back("INS::PosLla::altitude [m]", static_cast<double>(insOutputs->posLla(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
            {
                dynData.emplace_back("INS::PosEcef::X [m]", static_cast<double>(insOutputs->posEcef(0)));
                dynData.emplace_back("INS::PosEcef::Y [m]", static_cast<double>(insOutputs->posEcef(1)));
                dynData.emplace_back("INS::PosEcef::Z [m]", static_cast<double>(insOutputs->posEcef(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
            {
                dynData.emplace_back("INS::VelBody::X [m/s]", static_cast<double>(insOutputs->velBody(0)));
                dynData.emplace_back("INS::VelBody::Y [m/s]", static_cast<double>(insOutputs->velBody(1)));
                dynData.emplace_back("INS::VelBody::Z [m/s]", static_cast<double>(insOutputs->velBody(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
            {
                dynData.emplace_back("INS::VelNed::N [m/s]", static_cast<double>(insOutputs->velNed(0)));
                dynData.emplace_back("INS::VelNed::E [m/s]", static_cast<double>(insOutputs->velNed(1)));
                dynData.emplace_back("INS::VelNed::D [m/s]", static_cast<double>(insOutputs->velNed(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
            {
                dynData.emplace_back("INS::VelEcef::X [m/s]", static_cast<double>(insOutputs->velEcef(0)));
                dynData.emplace_back("INS::VelEcef::Y [m/s]", static_cast<double>(insOutputs->velEcef(1)));
                dynData.emplace_back("INS::VelEcef::Z [m/s]", static_cast<double>(insOutputs->velEcef(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
            {
                dynData.emplace_back("INS::MagEcef::X [Gauss]", static_cast<double>(insOutputs->magEcef(0)));
                dynData.emplace_back("INS::MagEcef::Y [Gauss]", static_cast<double>(insOutputs->magEcef(1)));
                dynData.emplace_back("INS::MagEcef::Z [Gauss]", static_cast<double>(insOutputs->magEcef(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
            {
                dynData.emplace_back("INS::AccelEcef::X [m/s^2]", static_cast<double>(insOutputs->accelEcef(0)));
                dynData.emplace_back("INS::AccelEcef::Y [m/s^2]", static_cast<double>(insOutputs->accelEcef(1)));
                dynData.emplace_back("INS::AccelEcef::Z [m/s^2]", static_cast<double>(insOutputs->accelEcef(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
            {
                dynData.emplace_back("INS::LinearAccelEcef::X [m/s^2]", static_cast<double>(insOutputs->linearAccelEcef(0)));
                dynData.emplace_back("INS::LinearAccelEcef::Y [m/s^2]", static_cast<double>(insOutputs->linearAccelEcef(1)));
                dynData.emplace_back("INS::LinearAccelEcef::Z [m/s^2]", static_cast<double>(insOutputs->linearAccelEcef(2)));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
            {
                dynData.emplace_back("INS::PosU [m]", static_cast<double>(insOutputs->posU));
            }
            if (insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
            {
                dynData.emplace_back("INS::VelU [m/s]", static_cast<double>(insOutputs->velU));
            }
        }

        // Group 7 (GNSS2)
        if (gnss2Outputs)
        {
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
            {
                dynData.emplace_back("GNSS2::UTC::year", static_cast<double>(gnss2Outputs->timeUtc.year));
                dynData.emplace_back("GNSS2::UTC::month", static_cast<double>(gnss2Outputs->timeUtc.month));
                dynData.emplace_back("GNSS2::UTC::day", static_cast<double>(gnss2Outputs->timeUtc.day));
                dynData.emplace_back("GNSS2::UTC::hour", static_cast<double>(gnss2Outputs->timeUtc.hour));
                dynData.emplace_back("GNSS2::UTC::min", static_cast<double>(gnss2Outputs->timeUtc.min));
                dynData.emplace_back("GNSS2::UTC::sec", static_cast<double>(gnss2Outputs->timeUtc.sec));
                dynData.emplace_back("GNSS2::UTC::ms", static_cast<double>(gnss2Outputs->timeUtc.ms));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
            {
                dynData.emplace_back("GNSS2::Tow [ns]", static_cast<double>(gnss2Outputs->tow));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
            {
                dynData.emplace_back("GNSS2::Week", static_cast<double>(gnss2Outputs->week));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
            {
                dynData.emplace_back("GNSS2::NumSats", static_cast<double>(gnss2Outputs->numSats));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
            {
                dynData.emplace_back("GNSS2::Fix", static_cast<double>(gnss2Outputs->fix));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                dynData.emplace_back("GNSS2::PosLla::latitude [deg]", static_cast<double>(gnss2Outputs->posLla(0)));
                dynData.emplace_back("GNSS2::PosLla::longitude [deg]", static_cast<double>(gnss2Outputs->posLla(1)));
                dynData.emplace_back("GNSS2::PosLla::altitude [m]", static_cast<double>(gnss2Outputs->posLla(2)));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                dynData.emplace_back("GNSS2::PosEcef::X [m]", static_cast<double>(gnss2Outputs->posEcef(0)));
                dynData.emplace_back("GNSS2::PosEcef::Y [m]", static_cast<double>(gnss2Outputs->posEcef(1)));
                dynData.emplace_back("GNSS2::PosEcef::Z [m]", static_cast<double>(gnss2Outputs->posEcef(2)));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                dynData.emplace_back("GNSS2::VelNed::N [m/s]", static_cast<double>(gnss2Outputs->velNed(0)));
                dynData.emplace_back("GNSS2::VelNed::E [m/s]", static_cast<double>(gnss2Outputs->velNed(1)));
                dynData.emplace_back("GNSS2::VelNed::D [m/s]", static_cast<double>(gnss2Outputs->velNed(2)));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
            {
                dynData.emplace_back("GNSS2::VelEcef::X [m/s]", static_cast<double>(gnss2Outputs->velEcef(0)));
                dynData.emplace_back("GNSS2::VelEcef::Y [m/s]", static_cast<double>(gnss2Outputs->velEcef(1)));
                dynData.emplace_back("GNSS2::VelEcef::Z [m/s]", static_cast<double>(gnss2Outputs->velEcef(2)));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
            {
                dynData.emplace_back("GNSS2::PosU::N [m]", static_cast<double>(gnss2Outputs->posU(0)));
                dynData.emplace_back("GNSS2::PosU::E [m]", static_cast<double>(gnss2Outputs->posU(1)));
                dynData.emplace_back("GNSS2::PosU::D [m]", static_cast<double>(gnss2Outputs->posU(2)));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
            {
                dynData.emplace_back("GNSS2::VelU [m/s]", static_cast<double>(gnss2Outputs->velU));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
            {
                dynData.emplace_back("GNSS2::TimeU [s]", static_cast<double>(gnss2Outputs->timeU));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
            {
                dynData.emplace_back("GNSS2::TimeInfo::Status::timeOk", static_cast<double>(gnss2Outputs->timeInfo.status.timeOk()));
                dynData.emplace_back("GNSS2::TimeInfo::Status::dateOk", static_cast<double>(gnss2Outputs->timeInfo.status.dateOk()));
                dynData.emplace_back("GNSS2::TimeInfo::Status::utcTimeValid", static_cast<double>(gnss2Outputs->timeInfo.status.utcTimeValid()));
                dynData.emplace_back("GNSS2::TimeInfo::LeapSeconds", static_cast<double>(gnss2Outputs->timeInfo.leapSeconds));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
            {
                dynData.emplace_back("GNSS2::DOP::g", static_cast<double>(gnss2Outputs->dop.gDop));
                dynData.emplace_back("GNSS2::DOP::p", static_cast<double>(gnss2Outputs->dop.pDop));
                dynData.emplace_back("GNSS2::DOP::t", static_cast<double>(gnss2Outputs->dop.tDop));
                dynData.emplace_back("GNSS2::DOP::v", static_cast<double>(gnss2Outputs->dop.vDop));
                dynData.emplace_back("GNSS2::DOP::h", static_cast<double>(gnss2Outputs->dop.hDop));
                dynData.emplace_back("GNSS2::DOP::n", static_cast<double>(gnss2Outputs->dop.nDop));
                dynData.emplace_back("GNSS2::DOP::e", static_cast<double>(gnss2Outputs->dop.eDop));
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
            {
                dynData.emplace_back("GNSS2::SatInfo::NumSats", static_cast<double>(gnss2Outputs->satInfo.numSats));
                for (auto& satellite : gnss2Outputs->satInfo.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag Healthy", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Healthy) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag Almanac", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Almanac) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag Ephemeris", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::Ephemeris) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag DifferentialCorrection", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::DifferentialCorrection) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag UsedForNavigation", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForNavigation) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag AzimuthElevationValid", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::AzimuthElevationValid) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - flag UsedForRTK", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::SatInfo::SatInfoElement::Flags::UsedForRTK) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - cno", satId), static_cast<double>(satellite.cno));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - qi", satId), static_cast<double>(satellite.qi));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - el", satId), static_cast<double>(satellite.el));
                    dynData.emplace_back(fmt::format("GNSS2::SatInfo::{} - az", satId), static_cast<double>(satellite.az));
                }
            }
            if (gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
            {
                dynData.emplace_back("GNSS2::RawMeas::Tow [s]", gnss2Outputs->raw.tow);
                dynData.emplace_back("GNSS2::RawMeas::Week", static_cast<double>(gnss2Outputs->raw.week));
                dynData.emplace_back("GNSS2::RawMeas::NumSats", static_cast<double>(gnss2Outputs->raw.numSats));
                for (auto& satellite : gnss2Outputs->raw.satellites)
                {
                    SatId satId(getSatSys(satellite.sys), static_cast<uint16_t>(satellite.svId));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - freq", satId), static_cast<double>(satellite.freq));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - chan", satId), static_cast<double>(satellite.chan));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - slot", satId), static_cast<double>(satellite.slot));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - cno", satId), static_cast<double>(satellite.cno));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag Searching", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Searching) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag Tracking", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::Tracking) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag TimeValid", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::TimeValid) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag CodeLock", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::CodeLock) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseLock", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseLock) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseHalfAmbiguity", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfAmbiguity) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseHalfSub", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseHalfSub) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag PhaseSlip", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PhaseSlip) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - flag PseudorangeSmoothed", satId), static_cast<double>(static_cast<unsigned int>(satellite.flags & NAV::vendor::vectornav::RawMeas::SatRawElement::Flags::PseudorangeSmoothed) ? 1 : 0));
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - pr", satId), satellite.pr);
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - cp", satId), satellite.cp);
                    dynData.emplace_back(fmt::format("GNSS2::RawMeas::{} - dp", satId), static_cast<double>(satellite.dp));
                }
            }
        }

        return dynData;
    }

    // ------------------------------------------ Public members ---------------------------------------------

    /// @brief Binary Group 2 - Time Outputs
    std::shared_ptr<vendor::vectornav::TimeOutputs> timeOutputs = nullptr;

    /// @brief Binary Group 3 - IMU Outputs
    std::shared_ptr<vendor::vectornav::ImuOutputs> imuOutputs = nullptr;

    /// @brief Binary Group 4 - GNSS1 Outputs
    std::shared_ptr<vendor::vectornav::GnssOutputs> gnss1Outputs = nullptr;

    /// @brief Binary Group 5 - Attitude Outputs
    std::shared_ptr<vendor::vectornav::AttitudeOutputs> attitudeOutputs = nullptr;

    /// @brief Binary Group 6 - INS Outputs
    std::shared_ptr<vendor::vectornav::InsOutputs> insOutputs = nullptr;

    /// @brief Binary Group 7 - GNSS2 Outputs
    std::shared_ptr<vendor::vectornav::GnssOutputs> gnss2Outputs = nullptr;

    /// Position and rotation information for conversion from platform to body frame
    const ImuPos& imuPos;
};

} // namespace NAV