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

#include <memory>

#include "NodeData/NodeData.hpp"
#include "NodeData/IMU/ImuPos.hpp"
#include "util/Eigen.hpp"

#include "util/Vendor/VectorNav/BinaryOutputs/TimeOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/ImuOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/GnssOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/AttitudeOutputs.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/InsOutputs.hpp"

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

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetDataDescriptors()
    {
        return {
            // Group 2 (Time)
            "Time::TimeStartup [ns]",
            "Time::TimeGps [ns]",
            "Time::GpsTow [ns]",
            "Time::GpsWeek",
            "Time::TimeSyncIn [ns]",
            "Time::TimeGpsPps [ns]",
            "Time::TimeUTC::year",
            "Time::TimeUTC::month",
            "Time::TimeUTC::day",
            "Time::TimeUTC::hour",
            "Time::TimeUTC::min",
            "Time::TimeUTC::sec",
            "Time::TimeUTC::ms",
            "Time::SyncInCnt",
            "Time::SyncOutCnt",
            "Time::TimeStatus::timeOk",
            "Time::TimeStatus::dateOk",
            "Time::TimeStatus::utcTimeValid",
            // Group 3 (IMU)
            "IMU::ImuStatus",
            "IMU::UncompMag::X [Gauss]",
            "IMU::UncompMag::Y [Gauss]",
            "IMU::UncompMag::Z [Gauss]",
            "IMU::UncompAccel::X [m/s^2]",
            "IMU::UncompAccel::Y [m/s^2]",
            "IMU::UncompAccel::Z [m/s^2]",
            "IMU::UncompGyro::X [rad/s]",
            "IMU::UncompGyro::Y [rad/s]",
            "IMU::UncompGyro::Z [rad/s]",
            "IMU::Temp [Celsius]",
            "IMU::Pres [kPa]",
            "IMU::DeltaTime [s]",
            "IMU::DeltaTheta::X [deg]",
            "IMU::DeltaTheta::Y [deg]",
            "IMU::DeltaTheta::Z [deg]",
            "IMU::DeltaVel::X [m/s]",
            "IMU::DeltaVel::Y [m/s]",
            "IMU::DeltaVel::Z [m/s]",
            "IMU::Mag::X [Gauss]",
            "IMU::Mag::Y [Gauss]",
            "IMU::Mag::Z [Gauss]",
            "IMU::Accel::X [m/s^2]",
            "IMU::Accel::Y [m/s^2]",
            "IMU::Accel::Z [m/s^2]",
            "IMU::AngularRate::X [rad/s]",
            "IMU::AngularRate::Y [rad/s]",
            "IMU::AngularRate::Z [rad/s]",
            // Group 4 (GNSS1)
            "GNSS1::UTC::year",
            "GNSS1::UTC::month",
            "GNSS1::UTC::day",
            "GNSS1::UTC::hour",
            "GNSS1::UTC::min",
            "GNSS1::UTC::sec",
            "GNSS1::UTC::ms",
            "GNSS1::Tow [ns]",
            "GNSS1::Week",
            "GNSS1::NumSats",
            "GNSS1::Fix",
            "GNSS1::PosLla::latitude [deg]",
            "GNSS1::PosLla::longitude [deg]",
            "GNSS1::PosLla::altitude [m]",
            "GNSS1::PosEcef::X [m]",
            "GNSS1::PosEcef::Y [m]",
            "GNSS1::PosEcef::Z [m]",
            "GNSS1::VelNed::N [m/s]",
            "GNSS1::VelNed::E [m/s]",
            "GNSS1::VelNed::D [m/s]",
            "GNSS1::VelEcef::X [m/s]",
            "GNSS1::VelEcef::Y [m/s]",
            "GNSS1::VelEcef::Z [m/s]",
            "GNSS1::PosU::N [m]",
            "GNSS1::PosU::E [m]",
            "GNSS1::PosU::D [m]",
            "GNSS1::VelU [m/s]",
            "GNSS1::TimeU [s]",
            "GNSS1::TimeInfo::Status::timeOk",
            "GNSS1::TimeInfo::Status::dateOk",
            "GNSS1::TimeInfo::Status::utcTimeValid",
            "GNSS1::TimeInfo::LeapSeconds",
            "GNSS1::DOP::g",
            "GNSS1::DOP::p",
            "GNSS1::DOP::t",
            "GNSS1::DOP::v",
            "GNSS1::DOP::h",
            "GNSS1::DOP::n",
            "GNSS1::DOP::e",
            "GNSS1::SatInfo::NumSats",
            "GNSS1::RawMeas::Tow [s]",
            "GNSS1::RawMeas::Week",
            "GNSS1::RawMeas::NumSats",
            // Group 5 (Attitude)
            "Att::VpeStatus::AttitudeQuality",
            "Att::VpeStatus::GyroSaturation",
            "Att::VpeStatus::GyroSaturationRecovery",
            "Att::VpeStatus::MagDisturbance",
            "Att::VpeStatus::MagSaturation",
            "Att::VpeStatus::AccDisturbance",
            "Att::VpeStatus::AccSaturation",
            "Att::VpeStatus::KnownMagDisturbance",
            "Att::VpeStatus::KnownAccelDisturbance",
            "Att::YawPitchRoll::Y [deg]",
            "Att::YawPitchRoll::P [deg]",
            "Att::YawPitchRoll::R [deg]",
            "Att::Quaternion::w",
            "Att::Quaternion::x",
            "Att::Quaternion::y",
            "Att::Quaternion::z",
            "Att::DCM::0-0",
            "Att::DCM::0-1",
            "Att::DCM::0-2",
            "Att::DCM::1-0",
            "Att::DCM::1-1",
            "Att::DCM::1-2",
            "Att::DCM::2-0",
            "Att::DCM::2-1",
            "Att::DCM::2-2",
            "Att::MagNed::N [Gauss]",
            "Att::MagNed::E [Gauss]",
            "Att::MagNed::D [Gauss]",
            "Att::AccelNed::N [m/s^2]",
            "Att::AccelNed::E [m/s^2]",
            "Att::AccelNed::D [m/s^2]",
            "Att::LinearAccelBody::X [m/s^2]",
            "Att::LinearAccelBody::Y [m/s^2]",
            "Att::LinearAccelBody::Z [m/s^2]",
            "Att::LinearAccelNed::N [m/s^2]",
            "Att::LinearAccelNed::E [m/s^2]",
            "Att::LinearAccelNed::D [m/s^2]",
            "Att::YprU::Y [deg]",
            "Att::YprU::P [deg]",
            "Att::YprU::R [deg]",
            // Group 6 (INS)
            "INS::InsStatus::Mode",
            "INS::InsStatus::GpsFix",
            "INS::InsStatus::Error::IMU",
            "INS::InsStatus::Error::MagPres",
            "INS::InsStatus::Error::GNSS",
            "INS::InsStatus::GpsHeadingIns",
            "INS::InsStatus::GpsCompass",
            "INS::PosLla::latitude [deg]",
            "INS::PosLla::longitude [deg]",
            "INS::PosLla::altitude [m]",
            "INS::PosEcef::X [m]",
            "INS::PosEcef::Y [m]",
            "INS::PosEcef::Z [m]",
            "INS::VelBody::X [m/s]",
            "INS::VelBody::Y [m/s]",
            "INS::VelBody::Z [m/s]",
            "INS::VelNed::N [m/s]",
            "INS::VelNed::E [m/s]",
            "INS::VelNed::D [m/s]",
            "INS::VelEcef::X [m/s]",
            "INS::VelEcef::Y [m/s]",
            "INS::VelEcef::Z [m/s]",
            "INS::MagEcef::X [Gauss}",
            "INS::MagEcef::Y [Gauss}",
            "INS::MagEcef::Z [Gauss}",
            "INS::AccelEcef::X [m/s^2]",
            "INS::AccelEcef::Y [m/s^2]",
            "INS::AccelEcef::Z [m/s^2]",
            "INS::LinearAccelEcef::X [m/s^2]",
            "INS::LinearAccelEcef::Y [m/s^2]",
            "INS::LinearAccelEcef::Z [m/s^2]",
            "INS::PosU [m]",
            "INS::VelU [m/s]",
            // Group 7 (GNSS2)
            "GNSS2::UTC::year",
            "GNSS2::UTC::month",
            "GNSS2::UTC::day",
            "GNSS2::UTC::hour",
            "GNSS2::UTC::min",
            "GNSS2::UTC::sec",
            "GNSS2::UTC::ms",
            "GNSS2::Tow [ns]",
            "GNSS2::Week",
            "GNSS2::NumSats",
            "GNSS2::Fix",
            "GNSS2::PosLla::latitude [deg]",
            "GNSS2::PosLla::longitude [deg]",
            "GNSS2::PosLla::altitude [m]",
            "GNSS2::PosEcef::X [m]",
            "GNSS2::PosEcef::Y [m]",
            "GNSS2::PosEcef::Z [m]",
            "GNSS2::VelNed::N [m/s]",
            "GNSS2::VelNed::E [m/s]",
            "GNSS2::VelNed::D [m/s]",
            "GNSS2::VelEcef::X [m/s]",
            "GNSS2::VelEcef::Y [m/s]",
            "GNSS2::VelEcef::Z [m/s]",
            "GNSS2::PosU::N [m]",
            "GNSS2::PosU::E [m]",
            "GNSS2::PosU::D [m]",
            "GNSS2::VelU [m/s]",
            "GNSS2::TimeU [s]",
            "GNSS2::TimeInfo::Status::timeOk",
            "GNSS2::TimeInfo::Status::dateOk",
            "GNSS2::TimeInfo::Status::utcTimeValid",
            "GNSS2::TimeInfo::LeapSeconds",
            "GNSS2::DOP::g",
            "GNSS2::DOP::p",
            "GNSS2::DOP::t",
            "GNSS2::DOP::v",
            "GNSS2::DOP::h",
            "GNSS2::DOP::n",
            "GNSS2::DOP::e",
            "GNSS2::SatInfo::NumSats",
            "GNSS2::RawMeas::Tow [s]",
            "GNSS2::RawMeas::Week",
            "GNSS2::RawMeas::NumSats",
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetDescriptorCount() { return 205; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> dataDescriptors() const override { return GetDataDescriptors(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetDescriptorCount());
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

    /// @brief Binary Group 2 – Time Outputs
    std::shared_ptr<vendor::vectornav::TimeOutputs> timeOutputs = nullptr;

    /// @brief Binary Group 3 – IMU Outputs
    std::shared_ptr<vendor::vectornav::ImuOutputs> imuOutputs = nullptr;

    /// @brief Binary Group 4 – GNSS1 Outputs
    std::shared_ptr<vendor::vectornav::GnssOutputs> gnss1Outputs = nullptr;

    /// @brief Binary Group 5 – Attitude Outputs
    std::shared_ptr<vendor::vectornav::AttitudeOutputs> attitudeOutputs = nullptr;

    /// @brief Binary Group 6 – INS Outputs
    std::shared_ptr<vendor::vectornav::InsOutputs> insOutputs = nullptr;

    /// @brief Binary Group 7 – GNSS2 Outputs
    std::shared_ptr<vendor::vectornav::GnssOutputs> gnss2Outputs = nullptr;

    /// Position and rotation information for conversion from platform to body frame
    const ImuPos& imuPos;
};

} // namespace NAV