// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file VectorNavDataLoggerTests.cpp
/// @brief Test for the VectorNavDataLogger node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-05-15

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <deque>
#include <atomic>
#include <mutex>

#include "FlowTester.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Logger.hpp"

namespace NAV::TESTS::VectorNavDataLoggerTests
{
[[maybe_unused]] constexpr size_t MESSAGE_COUNT_IMU = 18;  ///< Amount of messages expected in the Imu files
[[maybe_unused]] constexpr size_t MESSAGE_COUNT_GNSS = 12; ///< Amount of messages expected in the Gnss files

std::atomic<size_t> messageCounterImuDataCsv = 0;  ///< Message Counter for the Imu data csv file
std::atomic<size_t> messageCounterImuLogCsv = 0;   ///< Message Counter for the Imu log csv file
std::atomic<size_t> messageCounterImuLogVnb = 0;   ///< Message Counter for the Imu log vnb file
std::atomic<size_t> messageCounterGnssDataCsv = 0; ///< Message Counter for the Gnss data csv fil
std::atomic<size_t> messageCounterGnssLogCsv = 0;  ///< Message Counter for the Gnss log csv file
std::atomic<size_t> messageCounterGnssLogVnb = 0;  ///< Message Counter for the Gnss log vnb file

std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>> data_vn310_imu_csv; ///< Pointer to the current observation from the file 'data/VectorNav/StaticSize/vn310-imu.csv'
std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>> logs_vn310_imu_csv; ///< Pointer to the current observation from the file 'logs/vn310-imu.csv'
std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>> logs_vn310_imu_vnb; ///< Pointer to the current observation from the file 'logs/vn310-imu.vnb'

std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>> data_vn310_gnss_csv; ///< Pointer to the current observation from the file 'data/VectorNav/StaticSize/vn310-gnss.csv'
std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>> logs_vn310_gnss_csv; ///< Pointer to the current observation from the file 'logs/vn310-gnss.csv'
std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>> logs_vn310_gnss_vnb; ///< Pointer to the current observation from the file 'logs/vn310-gnss.vnb'

std::mutex comparisonMutex;

void compareObservations(std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>>& queue_data_csv,
                         std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>>& queue_logs_csv,
                         std::deque<std::shared_ptr<const NAV::VectorNavBinaryOutput>>& queue_logs_vnb)
{
    if (queue_data_csv.empty() || queue_logs_csv.empty() || queue_logs_vnb.empty()) { return; }

    auto data_csv = queue_data_csv.front();
    queue_data_csv.pop_front();
    auto logs_csv = queue_logs_csv.front();
    queue_logs_csv.pop_front();
    auto logs_vnb = queue_logs_vnb.front();
    queue_logs_vnb.pop_front();

    // ------------------------------------------------ InsTime --------------------------------------------------
    if (!data_csv->insTime.empty())
    {
        REQUIRE(!logs_csv->insTime.empty());
        REQUIRE((data_csv->insTime - logs_csv->insTime).count() < 1e-6L);

        REQUIRE(!logs_vnb->insTime.empty());
        REQUIRE((data_csv->insTime - logs_vnb->insTime).count() < 1e-6L);
    }

    // ----------------------------------------------- TimeGroup -------------------------------------------------
    if (data_csv->timeOutputs != nullptr)
    {
        REQUIRE(logs_csv->timeOutputs != nullptr);
        REQUIRE(logs_vnb->timeOutputs != nullptr);

        REQUIRE(data_csv->timeOutputs->timeField == logs_csv->timeOutputs->timeField);
        REQUIRE(data_csv->timeOutputs->timeField == logs_vnb->timeOutputs->timeField);

        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
        {
            REQUIRE(data_csv->timeOutputs->timeStartup == logs_csv->timeOutputs->timeStartup);

            REQUIRE(data_csv->timeOutputs->timeStartup == logs_vnb->timeOutputs->timeStartup);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS)
        {
            REQUIRE(data_csv->timeOutputs->timeGps == logs_csv->timeOutputs->timeGps);

            REQUIRE(data_csv->timeOutputs->timeGps == logs_vnb->timeOutputs->timeGps);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW)
        {
            REQUIRE(data_csv->timeOutputs->gpsTow == logs_csv->timeOutputs->gpsTow);

            REQUIRE(data_csv->timeOutputs->gpsTow == logs_vnb->timeOutputs->gpsTow);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK)
        {
            REQUIRE(data_csv->timeOutputs->gpsWeek == logs_csv->timeOutputs->gpsWeek);

            REQUIRE(data_csv->timeOutputs->gpsWeek == logs_vnb->timeOutputs->gpsWeek);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN)
        {
            REQUIRE(data_csv->timeOutputs->timeSyncIn == logs_csv->timeOutputs->timeSyncIn);

            REQUIRE(data_csv->timeOutputs->timeSyncIn == logs_vnb->timeOutputs->timeSyncIn);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS)
        {
            REQUIRE(data_csv->timeOutputs->timePPS == logs_csv->timeOutputs->timePPS);

            REQUIRE(data_csv->timeOutputs->timePPS == logs_vnb->timeOutputs->timePPS);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC)
        {
            REQUIRE(data_csv->timeOutputs->timeUtc.year == logs_csv->timeOutputs->timeUtc.year);
            REQUIRE(data_csv->timeOutputs->timeUtc.month == logs_csv->timeOutputs->timeUtc.month);
            REQUIRE(data_csv->timeOutputs->timeUtc.day == logs_csv->timeOutputs->timeUtc.day);
            REQUIRE(data_csv->timeOutputs->timeUtc.hour == logs_csv->timeOutputs->timeUtc.hour);
            REQUIRE(data_csv->timeOutputs->timeUtc.min == logs_csv->timeOutputs->timeUtc.min);
            REQUIRE(data_csv->timeOutputs->timeUtc.sec == logs_csv->timeOutputs->timeUtc.sec);
            REQUIRE(data_csv->timeOutputs->timeUtc.ms == logs_csv->timeOutputs->timeUtc.ms);

            REQUIRE(data_csv->timeOutputs->timeUtc.year == logs_vnb->timeOutputs->timeUtc.year);
            REQUIRE(data_csv->timeOutputs->timeUtc.month == logs_vnb->timeOutputs->timeUtc.month);
            REQUIRE(data_csv->timeOutputs->timeUtc.day == logs_vnb->timeOutputs->timeUtc.day);
            REQUIRE(data_csv->timeOutputs->timeUtc.hour == logs_vnb->timeOutputs->timeUtc.hour);
            REQUIRE(data_csv->timeOutputs->timeUtc.min == logs_vnb->timeOutputs->timeUtc.min);
            REQUIRE(data_csv->timeOutputs->timeUtc.sec == logs_vnb->timeOutputs->timeUtc.sec);
            REQUIRE(data_csv->timeOutputs->timeUtc.ms == logs_vnb->timeOutputs->timeUtc.ms);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT)
        {
            REQUIRE(data_csv->timeOutputs->timeSyncIn == logs_csv->timeOutputs->timeSyncIn);

            REQUIRE(data_csv->timeOutputs->timeSyncIn == logs_vnb->timeOutputs->timeSyncIn);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT)
        {
            REQUIRE(data_csv->timeOutputs->syncOutCnt == logs_csv->timeOutputs->syncOutCnt);

            REQUIRE(data_csv->timeOutputs->syncOutCnt == logs_vnb->timeOutputs->syncOutCnt);
        }
        if (data_csv->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS)
        {
            REQUIRE(data_csv->timeOutputs->timeStatus.timeOk() == logs_csv->timeOutputs->timeStatus.timeOk());
            REQUIRE(data_csv->timeOutputs->timeStatus.dateOk() == logs_csv->timeOutputs->timeStatus.dateOk());
            REQUIRE(data_csv->timeOutputs->timeStatus.utcTimeValid() == logs_csv->timeOutputs->timeStatus.utcTimeValid());

            REQUIRE(data_csv->timeOutputs->timeStatus.timeOk() == logs_vnb->timeOutputs->timeStatus.timeOk());
            REQUIRE(data_csv->timeOutputs->timeStatus.dateOk() == logs_vnb->timeOutputs->timeStatus.dateOk());
            REQUIRE(data_csv->timeOutputs->timeStatus.utcTimeValid() == logs_vnb->timeOutputs->timeStatus.utcTimeValid());
        }
    }

    // ----------------------------------------------- ImuGroup --------------------------------------------------
    if (data_csv->imuOutputs != nullptr)
    {
        REQUIRE(logs_csv->imuOutputs != nullptr);
        REQUIRE(logs_vnb->imuOutputs != nullptr);

        REQUIRE(data_csv->imuOutputs->imuField == logs_csv->imuOutputs->imuField);
        REQUIRE(data_csv->imuOutputs->imuField == logs_vnb->imuOutputs->imuField);

        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS)
        {
            REQUIRE(data_csv->imuOutputs->imuStatus == logs_csv->imuOutputs->imuStatus);

            REQUIRE(data_csv->imuOutputs->imuStatus == logs_vnb->imuOutputs->imuStatus);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
        {
            REQUIRE(data_csv->imuOutputs->uncompMag == logs_csv->imuOutputs->uncompMag);

            REQUIRE(data_csv->imuOutputs->uncompMag == logs_vnb->imuOutputs->uncompMag);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
        {
            REQUIRE(data_csv->imuOutputs->uncompAccel == logs_csv->imuOutputs->uncompAccel);

            REQUIRE(data_csv->imuOutputs->uncompAccel == logs_vnb->imuOutputs->uncompAccel);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
        {
            REQUIRE(data_csv->imuOutputs->uncompGyro == logs_csv->imuOutputs->uncompGyro);

            REQUIRE(data_csv->imuOutputs->uncompGyro == logs_vnb->imuOutputs->uncompGyro);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
        {
            REQUIRE(data_csv->imuOutputs->temp == logs_csv->imuOutputs->temp);

            REQUIRE(data_csv->imuOutputs->temp == logs_vnb->imuOutputs->temp);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_PRES)
        {
            REQUIRE(data_csv->imuOutputs->pres == logs_csv->imuOutputs->pres);

            REQUIRE(data_csv->imuOutputs->pres == logs_vnb->imuOutputs->pres);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
        {
            REQUIRE(data_csv->imuOutputs->deltaTime == logs_csv->imuOutputs->deltaTime);
            REQUIRE(data_csv->imuOutputs->deltaTheta == logs_csv->imuOutputs->deltaTheta);

            REQUIRE(data_csv->imuOutputs->deltaTime == logs_vnb->imuOutputs->deltaTime);
            REQUIRE(data_csv->imuOutputs->deltaTheta == logs_vnb->imuOutputs->deltaTheta);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
        {
            REQUIRE(data_csv->imuOutputs->deltaV == logs_csv->imuOutputs->deltaV);

            REQUIRE(data_csv->imuOutputs->deltaV == logs_vnb->imuOutputs->deltaV);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
        {
            REQUIRE(data_csv->imuOutputs->mag == logs_csv->imuOutputs->mag);

            REQUIRE(data_csv->imuOutputs->mag == logs_vnb->imuOutputs->mag);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
        {
            REQUIRE(data_csv->imuOutputs->accel == logs_csv->imuOutputs->accel);

            REQUIRE(data_csv->imuOutputs->accel == logs_vnb->imuOutputs->accel);
        }
        if (data_csv->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
        {
            REQUIRE(data_csv->imuOutputs->angularRate == logs_csv->imuOutputs->angularRate);

            REQUIRE(data_csv->imuOutputs->angularRate == logs_vnb->imuOutputs->angularRate);
        }
    }

    // ---------------------------------------------- GpsGroup 1 -------------------------------------------------
    if (data_csv->gnss1Outputs != nullptr)
    {
        REQUIRE(logs_csv->gnss1Outputs != nullptr);
        REQUIRE(logs_vnb->gnss1Outputs != nullptr);

        REQUIRE(data_csv->gnss1Outputs->gnssField == logs_csv->gnss1Outputs->gnssField);
        REQUIRE(data_csv->gnss1Outputs->gnssField == logs_vnb->gnss1Outputs->gnssField);

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
        {
            REQUIRE(data_csv->gnss1Outputs->timeUtc.year == logs_csv->gnss1Outputs->timeUtc.year);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.month == logs_csv->gnss1Outputs->timeUtc.month);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.day == logs_csv->gnss1Outputs->timeUtc.day);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.hour == logs_csv->gnss1Outputs->timeUtc.hour);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.min == logs_csv->gnss1Outputs->timeUtc.min);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.sec == logs_csv->gnss1Outputs->timeUtc.sec);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.ms == logs_csv->gnss1Outputs->timeUtc.ms);

            REQUIRE(data_csv->gnss1Outputs->timeUtc.year == logs_vnb->gnss1Outputs->timeUtc.year);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.month == logs_vnb->gnss1Outputs->timeUtc.month);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.day == logs_vnb->gnss1Outputs->timeUtc.day);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.hour == logs_vnb->gnss1Outputs->timeUtc.hour);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.min == logs_vnb->gnss1Outputs->timeUtc.min);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.sec == logs_vnb->gnss1Outputs->timeUtc.sec);
            REQUIRE(data_csv->gnss1Outputs->timeUtc.ms == logs_vnb->gnss1Outputs->timeUtc.ms);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
        {
            REQUIRE(data_csv->gnss1Outputs->tow == logs_csv->gnss1Outputs->tow);

            REQUIRE(data_csv->gnss1Outputs->tow == logs_vnb->gnss1Outputs->tow);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
        {
            REQUIRE(data_csv->gnss1Outputs->week == logs_csv->gnss1Outputs->week);

            REQUIRE(data_csv->gnss1Outputs->week == logs_vnb->gnss1Outputs->week);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
        {
            REQUIRE(data_csv->gnss1Outputs->numSats == logs_csv->gnss1Outputs->numSats);

            REQUIRE(data_csv->gnss1Outputs->numSats == logs_vnb->gnss1Outputs->numSats);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
        {
            REQUIRE(data_csv->gnss1Outputs->fix == logs_csv->gnss1Outputs->fix);

            REQUIRE(data_csv->gnss1Outputs->fix == logs_vnb->gnss1Outputs->fix);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
        {
            REQUIRE(data_csv->gnss1Outputs->posLla == logs_csv->gnss1Outputs->posLla);

            REQUIRE(data_csv->gnss1Outputs->posLla == logs_vnb->gnss1Outputs->posLla);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
        {
            REQUIRE(data_csv->gnss1Outputs->posEcef == logs_csv->gnss1Outputs->posEcef);

            REQUIRE(data_csv->gnss1Outputs->posEcef == logs_vnb->gnss1Outputs->posEcef);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
        {
            REQUIRE(data_csv->gnss1Outputs->velNed == logs_csv->gnss1Outputs->velNed);

            REQUIRE(data_csv->gnss1Outputs->velNed == logs_vnb->gnss1Outputs->velNed);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
        {
            REQUIRE(data_csv->gnss1Outputs->velEcef == logs_csv->gnss1Outputs->velEcef);

            REQUIRE(data_csv->gnss1Outputs->velEcef == logs_vnb->gnss1Outputs->velEcef);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
        {
            REQUIRE(data_csv->gnss1Outputs->posU == logs_csv->gnss1Outputs->posU);

            REQUIRE(data_csv->gnss1Outputs->posU == logs_vnb->gnss1Outputs->posU);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
        {
            REQUIRE(data_csv->gnss1Outputs->velU == logs_csv->gnss1Outputs->velU);

            REQUIRE(data_csv->gnss1Outputs->velU == logs_vnb->gnss1Outputs->velU);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
        {
            REQUIRE(data_csv->gnss1Outputs->timeU == logs_csv->gnss1Outputs->timeU);

            REQUIRE(data_csv->gnss1Outputs->timeU == logs_vnb->gnss1Outputs->timeU);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
        {
            REQUIRE(data_csv->gnss1Outputs->timeInfo.status.timeOk() == logs_csv->gnss1Outputs->timeInfo.status.timeOk());
            REQUIRE(data_csv->gnss1Outputs->timeInfo.status.dateOk() == logs_csv->gnss1Outputs->timeInfo.status.dateOk());
            REQUIRE(data_csv->gnss1Outputs->timeInfo.status.utcTimeValid() == logs_csv->gnss1Outputs->timeInfo.status.utcTimeValid());
            REQUIRE(data_csv->gnss1Outputs->timeInfo.leapSeconds == logs_csv->gnss1Outputs->timeInfo.leapSeconds);

            REQUIRE(data_csv->gnss1Outputs->timeInfo.status.timeOk() == logs_vnb->gnss1Outputs->timeInfo.status.timeOk());
            REQUIRE(data_csv->gnss1Outputs->timeInfo.status.dateOk() == logs_vnb->gnss1Outputs->timeInfo.status.dateOk());
            REQUIRE(data_csv->gnss1Outputs->timeInfo.status.utcTimeValid() == logs_vnb->gnss1Outputs->timeInfo.status.utcTimeValid());
            REQUIRE(data_csv->gnss1Outputs->timeInfo.leapSeconds == logs_vnb->gnss1Outputs->timeInfo.leapSeconds);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
        {
            REQUIRE(data_csv->gnss1Outputs->dop.gDop == logs_csv->gnss1Outputs->dop.gDop);
            REQUIRE(data_csv->gnss1Outputs->dop.pDop == logs_csv->gnss1Outputs->dop.pDop);
            REQUIRE(data_csv->gnss1Outputs->dop.tDop == logs_csv->gnss1Outputs->dop.tDop);
            REQUIRE(data_csv->gnss1Outputs->dop.vDop == logs_csv->gnss1Outputs->dop.vDop);
            REQUIRE(data_csv->gnss1Outputs->dop.hDop == logs_csv->gnss1Outputs->dop.hDop);
            REQUIRE(data_csv->gnss1Outputs->dop.nDop == logs_csv->gnss1Outputs->dop.nDop);
            REQUIRE(data_csv->gnss1Outputs->dop.eDop == logs_csv->gnss1Outputs->dop.eDop);

            REQUIRE(data_csv->gnss1Outputs->dop.gDop == logs_vnb->gnss1Outputs->dop.gDop);
            REQUIRE(data_csv->gnss1Outputs->dop.pDop == logs_vnb->gnss1Outputs->dop.pDop);
            REQUIRE(data_csv->gnss1Outputs->dop.tDop == logs_vnb->gnss1Outputs->dop.tDop);
            REQUIRE(data_csv->gnss1Outputs->dop.vDop == logs_vnb->gnss1Outputs->dop.vDop);
            REQUIRE(data_csv->gnss1Outputs->dop.hDop == logs_vnb->gnss1Outputs->dop.hDop);
            REQUIRE(data_csv->gnss1Outputs->dop.nDop == logs_vnb->gnss1Outputs->dop.nDop);
            REQUIRE(data_csv->gnss1Outputs->dop.eDop == logs_vnb->gnss1Outputs->dop.eDop);
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
        {
            REQUIRE(data_csv->gnss1Outputs->satInfo.numSats == logs_csv->gnss1Outputs->satInfo.numSats);
            REQUIRE(data_csv->gnss1Outputs->satInfo.numSats == logs_vnb->gnss1Outputs->satInfo.numSats);

            for (size_t i = 0; i < data_csv->gnss1Outputs->satInfo.satellites.size(); i++)
            {
                LOG_TRACE("Comparing satInfo.satellites {}", i);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).sys == logs_csv->gnss1Outputs->satInfo.satellites.at(i).sys);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).svId == logs_csv->gnss1Outputs->satInfo.satellites.at(i).svId);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).flags == logs_csv->gnss1Outputs->satInfo.satellites.at(i).flags);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).cno == logs_csv->gnss1Outputs->satInfo.satellites.at(i).cno);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).qi == logs_csv->gnss1Outputs->satInfo.satellites.at(i).qi);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).el == logs_csv->gnss1Outputs->satInfo.satellites.at(i).el);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).az == logs_csv->gnss1Outputs->satInfo.satellites.at(i).az);

                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).sys == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).sys);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).svId == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).svId);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).flags == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).flags);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).cno == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).cno);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).qi == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).qi);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).el == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).el);
                REQUIRE(data_csv->gnss1Outputs->satInfo.satellites.at(i).az == logs_vnb->gnss1Outputs->satInfo.satellites.at(i).az);
            }
        }

        if (data_csv->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
        {
            REQUIRE(data_csv->gnss1Outputs->raw.tow == logs_csv->gnss1Outputs->raw.tow);
            REQUIRE(data_csv->gnss1Outputs->raw.week == logs_csv->gnss1Outputs->raw.week);
            REQUIRE(data_csv->gnss1Outputs->raw.numSats == logs_csv->gnss1Outputs->raw.numSats);

            REQUIRE(data_csv->gnss1Outputs->raw.tow == logs_vnb->gnss1Outputs->raw.tow);
            REQUIRE(data_csv->gnss1Outputs->raw.week == logs_vnb->gnss1Outputs->raw.week);
            REQUIRE(data_csv->gnss1Outputs->raw.numSats == logs_vnb->gnss1Outputs->raw.numSats);

            for (size_t i = 0; i < data_csv->gnss1Outputs->raw.satellites.size(); i++)
            {
                LOG_TRACE("Comparing raw.satellites {}", i);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).sys == logs_csv->gnss1Outputs->raw.satellites.at(i).sys);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).svId == logs_csv->gnss1Outputs->raw.satellites.at(i).svId);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).freq == logs_csv->gnss1Outputs->raw.satellites.at(i).freq);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).chan == logs_csv->gnss1Outputs->raw.satellites.at(i).chan);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).slot == logs_csv->gnss1Outputs->raw.satellites.at(i).slot);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).cno == logs_csv->gnss1Outputs->raw.satellites.at(i).cno);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).flags == logs_csv->gnss1Outputs->raw.satellites.at(i).flags);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).pr == logs_csv->gnss1Outputs->raw.satellites.at(i).pr);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).cp == logs_csv->gnss1Outputs->raw.satellites.at(i).cp);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).dp == logs_csv->gnss1Outputs->raw.satellites.at(i).dp);

                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).sys == logs_vnb->gnss1Outputs->raw.satellites.at(i).sys);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).svId == logs_vnb->gnss1Outputs->raw.satellites.at(i).svId);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).freq == logs_vnb->gnss1Outputs->raw.satellites.at(i).freq);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).chan == logs_vnb->gnss1Outputs->raw.satellites.at(i).chan);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).slot == logs_vnb->gnss1Outputs->raw.satellites.at(i).slot);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).cno == logs_vnb->gnss1Outputs->raw.satellites.at(i).cno);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).flags == logs_vnb->gnss1Outputs->raw.satellites.at(i).flags);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).pr == logs_vnb->gnss1Outputs->raw.satellites.at(i).pr);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).cp == logs_vnb->gnss1Outputs->raw.satellites.at(i).cp);
                REQUIRE(data_csv->gnss1Outputs->raw.satellites.at(i).dp == logs_vnb->gnss1Outputs->raw.satellites.at(i).dp);
            }
        }
    }

    // --------------------------------------------- AttitudeGroup -----------------------------------------------
    if (data_csv->attitudeOutputs != nullptr)
    {
        REQUIRE(logs_csv->attitudeOutputs != nullptr);
        REQUIRE(logs_vnb->attitudeOutputs != nullptr);

        REQUIRE(data_csv->attitudeOutputs->attitudeField == logs_csv->attitudeOutputs->attitudeField);
        REQUIRE(data_csv->attitudeOutputs->attitudeField == logs_vnb->attitudeOutputs->attitudeField);

        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS)
        {
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.attitudeQuality() == logs_csv->attitudeOutputs->vpeStatus.attitudeQuality());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.gyroSaturation() == logs_csv->attitudeOutputs->vpeStatus.gyroSaturation());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.gyroSaturationRecovery() == logs_csv->attitudeOutputs->vpeStatus.gyroSaturationRecovery());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.magDisturbance() == logs_csv->attitudeOutputs->vpeStatus.magDisturbance());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.magSaturation() == logs_csv->attitudeOutputs->vpeStatus.magSaturation());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.accDisturbance() == logs_csv->attitudeOutputs->vpeStatus.accDisturbance());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.accSaturation() == logs_csv->attitudeOutputs->vpeStatus.accSaturation());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.knownMagDisturbance() == logs_csv->attitudeOutputs->vpeStatus.knownMagDisturbance());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.knownAccelDisturbance() == logs_csv->attitudeOutputs->vpeStatus.knownAccelDisturbance());

            REQUIRE(data_csv->attitudeOutputs->vpeStatus.attitudeQuality() == logs_vnb->attitudeOutputs->vpeStatus.attitudeQuality());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.gyroSaturation() == logs_vnb->attitudeOutputs->vpeStatus.gyroSaturation());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.gyroSaturationRecovery() == logs_vnb->attitudeOutputs->vpeStatus.gyroSaturationRecovery());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.magDisturbance() == logs_vnb->attitudeOutputs->vpeStatus.magDisturbance());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.magSaturation() == logs_vnb->attitudeOutputs->vpeStatus.magSaturation());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.accDisturbance() == logs_vnb->attitudeOutputs->vpeStatus.accDisturbance());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.accSaturation() == logs_vnb->attitudeOutputs->vpeStatus.accSaturation());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.knownMagDisturbance() == logs_vnb->attitudeOutputs->vpeStatus.knownMagDisturbance());
            REQUIRE(data_csv->attitudeOutputs->vpeStatus.knownAccelDisturbance() == logs_vnb->attitudeOutputs->vpeStatus.knownAccelDisturbance());
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
        {
            REQUIRE(data_csv->attitudeOutputs->ypr == logs_csv->attitudeOutputs->ypr);

            REQUIRE(data_csv->attitudeOutputs->ypr == logs_vnb->attitudeOutputs->ypr);
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
        {
            REQUIRE(data_csv->attitudeOutputs->qtn.coeffs() == logs_csv->attitudeOutputs->qtn.coeffs());

            REQUIRE(data_csv->attitudeOutputs->qtn.coeffs() == logs_vnb->attitudeOutputs->qtn.coeffs());
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
        {
            REQUIRE(data_csv->attitudeOutputs->dcm == logs_csv->attitudeOutputs->dcm);

            REQUIRE(data_csv->attitudeOutputs->dcm == logs_vnb->attitudeOutputs->dcm);
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED)
        {
            REQUIRE(data_csv->attitudeOutputs->magNed == logs_csv->attitudeOutputs->magNed);

            REQUIRE(data_csv->attitudeOutputs->magNed == logs_vnb->attitudeOutputs->magNed);
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED)
        {
            REQUIRE(data_csv->attitudeOutputs->accelNed == logs_csv->attitudeOutputs->accelNed);

            REQUIRE(data_csv->attitudeOutputs->accelNed == logs_vnb->attitudeOutputs->accelNed);
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY)
        {
            REQUIRE(data_csv->attitudeOutputs->linearAccelBody == logs_csv->attitudeOutputs->linearAccelBody);

            REQUIRE(data_csv->attitudeOutputs->linearAccelBody == logs_vnb->attitudeOutputs->linearAccelBody);
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED)
        {
            REQUIRE(data_csv->attitudeOutputs->linearAccelNed == logs_csv->attitudeOutputs->linearAccelNed);

            REQUIRE(data_csv->attitudeOutputs->linearAccelNed == logs_vnb->attitudeOutputs->linearAccelNed);
        }
        if (data_csv->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU)
        {
            REQUIRE(data_csv->attitudeOutputs->yprU == logs_csv->attitudeOutputs->yprU);

            REQUIRE(data_csv->attitudeOutputs->yprU == logs_vnb->attitudeOutputs->yprU);
        }
    }

    // ----------------------------------------------- InsGroup --------------------------------------------------
    if (data_csv->insOutputs != nullptr)
    {
        REQUIRE(logs_csv->insOutputs != nullptr);
        REQUIRE(logs_vnb->insOutputs != nullptr);

        REQUIRE(data_csv->insOutputs->insField == logs_csv->insOutputs->insField);
        REQUIRE(data_csv->insOutputs->insField == logs_vnb->insOutputs->insField);

        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS)
        {
            REQUIRE(data_csv->insOutputs->insStatus.mode() == logs_csv->insOutputs->insStatus.mode());
            REQUIRE(data_csv->insOutputs->insStatus.gpsFix() == logs_csv->insOutputs->insStatus.gpsFix());
            REQUIRE(data_csv->insOutputs->insStatus.errorIMU() == logs_csv->insOutputs->insStatus.errorIMU());
            REQUIRE(data_csv->insOutputs->insStatus.errorMagPres() == logs_csv->insOutputs->insStatus.errorMagPres());
            REQUIRE(data_csv->insOutputs->insStatus.errorGnss() == logs_csv->insOutputs->insStatus.errorGnss());
            REQUIRE(data_csv->insOutputs->insStatus.gpsHeadingIns() == logs_csv->insOutputs->insStatus.gpsHeadingIns());
            REQUIRE(data_csv->insOutputs->insStatus.gpsCompass() == logs_csv->insOutputs->insStatus.gpsCompass());

            REQUIRE(data_csv->insOutputs->insStatus.mode() == logs_vnb->insOutputs->insStatus.mode());
            REQUIRE(data_csv->insOutputs->insStatus.gpsFix() == logs_vnb->insOutputs->insStatus.gpsFix());
            REQUIRE(data_csv->insOutputs->insStatus.errorIMU() == logs_vnb->insOutputs->insStatus.errorIMU());
            REQUIRE(data_csv->insOutputs->insStatus.errorMagPres() == logs_vnb->insOutputs->insStatus.errorMagPres());
            REQUIRE(data_csv->insOutputs->insStatus.errorGnss() == logs_vnb->insOutputs->insStatus.errorGnss());
            REQUIRE(data_csv->insOutputs->insStatus.gpsHeadingIns() == logs_vnb->insOutputs->insStatus.gpsHeadingIns());
            REQUIRE(data_csv->insOutputs->insStatus.gpsCompass() == logs_vnb->insOutputs->insStatus.gpsCompass());
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
        {
            REQUIRE(data_csv->insOutputs->posLla == logs_csv->insOutputs->posLla);

            REQUIRE(data_csv->insOutputs->posLla == logs_vnb->insOutputs->posLla);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
        {
            REQUIRE(data_csv->insOutputs->posEcef == logs_csv->insOutputs->posEcef);

            REQUIRE(data_csv->insOutputs->posEcef == logs_vnb->insOutputs->posEcef);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
        {
            REQUIRE(data_csv->insOutputs->velBody == logs_csv->insOutputs->velBody);

            REQUIRE(data_csv->insOutputs->velBody == logs_vnb->insOutputs->velBody);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
        {
            REQUIRE(data_csv->insOutputs->velNed == logs_csv->insOutputs->velNed);

            REQUIRE(data_csv->insOutputs->velNed == logs_vnb->insOutputs->velNed);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
        {
            REQUIRE(data_csv->insOutputs->velEcef == logs_csv->insOutputs->velEcef);

            REQUIRE(data_csv->insOutputs->velEcef == logs_vnb->insOutputs->velEcef);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_MAGECEF)
        {
            REQUIRE(data_csv->insOutputs->magEcef == logs_csv->insOutputs->magEcef);

            REQUIRE(data_csv->insOutputs->magEcef == logs_vnb->insOutputs->magEcef);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF)
        {
            REQUIRE(data_csv->insOutputs->accelEcef == logs_csv->insOutputs->accelEcef);

            REQUIRE(data_csv->insOutputs->accelEcef == logs_vnb->insOutputs->accelEcef);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF)
        {
            REQUIRE(data_csv->insOutputs->linearAccelEcef == logs_csv->insOutputs->linearAccelEcef);

            REQUIRE(data_csv->insOutputs->linearAccelEcef == logs_vnb->insOutputs->linearAccelEcef);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSU)
        {
            REQUIRE(data_csv->insOutputs->posU == logs_csv->insOutputs->posU);

            REQUIRE(data_csv->insOutputs->posU == logs_vnb->insOutputs->posU);
        }
        if (data_csv->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELU)
        {
            REQUIRE(data_csv->insOutputs->velU == logs_csv->insOutputs->velU);

            REQUIRE(data_csv->insOutputs->velU == logs_vnb->insOutputs->velU);
        }
    }

    // ---------------------------------------------- GpsGroup 2 -------------------------------------------------
    if (data_csv->gnss2Outputs != nullptr)
    {
        REQUIRE(logs_csv->gnss2Outputs != nullptr);
        REQUIRE(logs_vnb->gnss2Outputs != nullptr);

        REQUIRE(data_csv->gnss2Outputs->gnssField == logs_csv->gnss2Outputs->gnssField);
        REQUIRE(data_csv->gnss2Outputs->gnssField == logs_vnb->gnss2Outputs->gnssField);

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_UTC)
        {
            REQUIRE(data_csv->gnss2Outputs->timeUtc.year == logs_csv->gnss2Outputs->timeUtc.year);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.month == logs_csv->gnss2Outputs->timeUtc.month);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.day == logs_csv->gnss2Outputs->timeUtc.day);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.hour == logs_csv->gnss2Outputs->timeUtc.hour);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.min == logs_csv->gnss2Outputs->timeUtc.min);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.sec == logs_csv->gnss2Outputs->timeUtc.sec);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.ms == logs_csv->gnss2Outputs->timeUtc.ms);

            REQUIRE(data_csv->gnss2Outputs->timeUtc.year == logs_vnb->gnss2Outputs->timeUtc.year);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.month == logs_vnb->gnss2Outputs->timeUtc.month);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.day == logs_vnb->gnss2Outputs->timeUtc.day);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.hour == logs_vnb->gnss2Outputs->timeUtc.hour);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.min == logs_vnb->gnss2Outputs->timeUtc.min);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.sec == logs_vnb->gnss2Outputs->timeUtc.sec);
            REQUIRE(data_csv->gnss2Outputs->timeUtc.ms == logs_vnb->gnss2Outputs->timeUtc.ms);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TOW)
        {
            REQUIRE(data_csv->gnss2Outputs->tow == logs_csv->gnss2Outputs->tow);

            REQUIRE(data_csv->gnss2Outputs->tow == logs_vnb->gnss2Outputs->tow);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_WEEK)
        {
            REQUIRE(data_csv->gnss2Outputs->week == logs_csv->gnss2Outputs->week);

            REQUIRE(data_csv->gnss2Outputs->week == logs_vnb->gnss2Outputs->week);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS)
        {
            REQUIRE(data_csv->gnss2Outputs->numSats == logs_csv->gnss2Outputs->numSats);

            REQUIRE(data_csv->gnss2Outputs->numSats == logs_vnb->gnss2Outputs->numSats);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_FIX)
        {
            REQUIRE(data_csv->gnss2Outputs->fix == logs_csv->gnss2Outputs->fix);

            REQUIRE(data_csv->gnss2Outputs->fix == logs_vnb->gnss2Outputs->fix);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
        {
            REQUIRE(data_csv->gnss2Outputs->posLla == logs_csv->gnss2Outputs->posLla);

            REQUIRE(data_csv->gnss2Outputs->posLla == logs_vnb->gnss2Outputs->posLla);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
        {
            REQUIRE(data_csv->gnss2Outputs->posEcef == logs_csv->gnss2Outputs->posEcef);

            REQUIRE(data_csv->gnss2Outputs->posEcef == logs_vnb->gnss2Outputs->posEcef);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
        {
            REQUIRE(data_csv->gnss2Outputs->velNed == logs_csv->gnss2Outputs->velNed);

            REQUIRE(data_csv->gnss2Outputs->velNed == logs_vnb->gnss2Outputs->velNed);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
        {
            REQUIRE(data_csv->gnss2Outputs->velEcef == logs_csv->gnss2Outputs->velEcef);

            REQUIRE(data_csv->gnss2Outputs->velEcef == logs_vnb->gnss2Outputs->velEcef);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSU)
        {
            REQUIRE(data_csv->gnss2Outputs->posU == logs_csv->gnss2Outputs->posU);

            REQUIRE(data_csv->gnss2Outputs->posU == logs_vnb->gnss2Outputs->posU);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELU)
        {
            REQUIRE(data_csv->gnss2Outputs->velU == logs_csv->gnss2Outputs->velU);

            REQUIRE(data_csv->gnss2Outputs->velU == logs_vnb->gnss2Outputs->velU);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU)
        {
            REQUIRE(data_csv->gnss2Outputs->timeU == logs_csv->gnss2Outputs->timeU);

            REQUIRE(data_csv->gnss2Outputs->timeU == logs_vnb->gnss2Outputs->timeU);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO)
        {
            REQUIRE(data_csv->gnss2Outputs->timeInfo.status.timeOk() == logs_csv->gnss2Outputs->timeInfo.status.timeOk());
            REQUIRE(data_csv->gnss2Outputs->timeInfo.status.dateOk() == logs_csv->gnss2Outputs->timeInfo.status.dateOk());
            REQUIRE(data_csv->gnss2Outputs->timeInfo.status.utcTimeValid() == logs_csv->gnss2Outputs->timeInfo.status.utcTimeValid());
            REQUIRE(data_csv->gnss2Outputs->timeInfo.leapSeconds == logs_csv->gnss2Outputs->timeInfo.leapSeconds);

            REQUIRE(data_csv->gnss2Outputs->timeInfo.status.timeOk() == logs_vnb->gnss2Outputs->timeInfo.status.timeOk());
            REQUIRE(data_csv->gnss2Outputs->timeInfo.status.dateOk() == logs_vnb->gnss2Outputs->timeInfo.status.dateOk());
            REQUIRE(data_csv->gnss2Outputs->timeInfo.status.utcTimeValid() == logs_vnb->gnss2Outputs->timeInfo.status.utcTimeValid());
            REQUIRE(data_csv->gnss2Outputs->timeInfo.leapSeconds == logs_vnb->gnss2Outputs->timeInfo.leapSeconds);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_DOP)
        {
            REQUIRE(data_csv->gnss2Outputs->dop.gDop == logs_csv->gnss2Outputs->dop.gDop);
            REQUIRE(data_csv->gnss2Outputs->dop.pDop == logs_csv->gnss2Outputs->dop.pDop);
            REQUIRE(data_csv->gnss2Outputs->dop.tDop == logs_csv->gnss2Outputs->dop.tDop);
            REQUIRE(data_csv->gnss2Outputs->dop.vDop == logs_csv->gnss2Outputs->dop.vDop);
            REQUIRE(data_csv->gnss2Outputs->dop.hDop == logs_csv->gnss2Outputs->dop.hDop);
            REQUIRE(data_csv->gnss2Outputs->dop.nDop == logs_csv->gnss2Outputs->dop.nDop);
            REQUIRE(data_csv->gnss2Outputs->dop.eDop == logs_csv->gnss2Outputs->dop.eDop);

            REQUIRE(data_csv->gnss2Outputs->dop.gDop == logs_vnb->gnss2Outputs->dop.gDop);
            REQUIRE(data_csv->gnss2Outputs->dop.pDop == logs_vnb->gnss2Outputs->dop.pDop);
            REQUIRE(data_csv->gnss2Outputs->dop.tDop == logs_vnb->gnss2Outputs->dop.tDop);
            REQUIRE(data_csv->gnss2Outputs->dop.vDop == logs_vnb->gnss2Outputs->dop.vDop);
            REQUIRE(data_csv->gnss2Outputs->dop.hDop == logs_vnb->gnss2Outputs->dop.hDop);
            REQUIRE(data_csv->gnss2Outputs->dop.nDop == logs_vnb->gnss2Outputs->dop.nDop);
            REQUIRE(data_csv->gnss2Outputs->dop.eDop == logs_vnb->gnss2Outputs->dop.eDop);
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO)
        {
            REQUIRE(data_csv->gnss2Outputs->satInfo.numSats == logs_csv->gnss2Outputs->satInfo.numSats);
            REQUIRE(data_csv->gnss2Outputs->satInfo.numSats == logs_vnb->gnss2Outputs->satInfo.numSats);

            for (size_t i = 0; i < data_csv->gnss2Outputs->satInfo.satellites.size(); i++)
            {
                LOG_TRACE("Comparing satInfo.satellites {}", i);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).sys == logs_csv->gnss2Outputs->satInfo.satellites.at(i).sys);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).svId == logs_csv->gnss2Outputs->satInfo.satellites.at(i).svId);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).flags == logs_csv->gnss2Outputs->satInfo.satellites.at(i).flags);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).cno == logs_csv->gnss2Outputs->satInfo.satellites.at(i).cno);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).qi == logs_csv->gnss2Outputs->satInfo.satellites.at(i).qi);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).el == logs_csv->gnss2Outputs->satInfo.satellites.at(i).el);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).az == logs_csv->gnss2Outputs->satInfo.satellites.at(i).az);

                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).sys == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).sys);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).svId == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).svId);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).flags == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).flags);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).cno == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).cno);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).qi == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).qi);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).el == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).el);
                REQUIRE(data_csv->gnss2Outputs->satInfo.satellites.at(i).az == logs_vnb->gnss2Outputs->satInfo.satellites.at(i).az);
            }
        }

        if (data_csv->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
        {
            REQUIRE(data_csv->gnss2Outputs->raw.tow == logs_csv->gnss2Outputs->raw.tow);
            REQUIRE(data_csv->gnss2Outputs->raw.week == logs_csv->gnss2Outputs->raw.week);
            REQUIRE(data_csv->gnss2Outputs->raw.numSats == logs_csv->gnss2Outputs->raw.numSats);

            REQUIRE(data_csv->gnss2Outputs->raw.tow == logs_vnb->gnss2Outputs->raw.tow);
            REQUIRE(data_csv->gnss2Outputs->raw.week == logs_vnb->gnss2Outputs->raw.week);
            REQUIRE(data_csv->gnss2Outputs->raw.numSats == logs_vnb->gnss2Outputs->raw.numSats);

            for (size_t i = 0; i < data_csv->gnss2Outputs->raw.satellites.size(); i++)
            {
                LOG_TRACE("Comparing raw.satellites {}", i);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).sys == logs_csv->gnss2Outputs->raw.satellites.at(i).sys);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).svId == logs_csv->gnss2Outputs->raw.satellites.at(i).svId);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).freq == logs_csv->gnss2Outputs->raw.satellites.at(i).freq);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).chan == logs_csv->gnss2Outputs->raw.satellites.at(i).chan);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).slot == logs_csv->gnss2Outputs->raw.satellites.at(i).slot);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).cno == logs_csv->gnss2Outputs->raw.satellites.at(i).cno);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).flags == logs_csv->gnss2Outputs->raw.satellites.at(i).flags);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).pr == logs_csv->gnss2Outputs->raw.satellites.at(i).pr);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).cp == logs_csv->gnss2Outputs->raw.satellites.at(i).cp);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).dp == logs_csv->gnss2Outputs->raw.satellites.at(i).dp);

                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).sys == logs_vnb->gnss2Outputs->raw.satellites.at(i).sys);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).svId == logs_vnb->gnss2Outputs->raw.satellites.at(i).svId);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).freq == logs_vnb->gnss2Outputs->raw.satellites.at(i).freq);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).chan == logs_vnb->gnss2Outputs->raw.satellites.at(i).chan);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).slot == logs_vnb->gnss2Outputs->raw.satellites.at(i).slot);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).cno == logs_vnb->gnss2Outputs->raw.satellites.at(i).cno);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).flags == logs_vnb->gnss2Outputs->raw.satellites.at(i).flags);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).pr == logs_vnb->gnss2Outputs->raw.satellites.at(i).pr);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).cp == logs_vnb->gnss2Outputs->raw.satellites.at(i).cp);
                REQUIRE(data_csv->gnss2Outputs->raw.satellites.at(i).dp == logs_vnb->gnss2Outputs->raw.satellites.at(i).dp);
            }
        }
    }

    // ------------------------------------------------- Reset ---------------------------------------------------
    data_csv.reset();
    logs_csv.reset();
    logs_vnb.reset();
}

TEST_CASE("[VectorNavDataLogger][flow] Read and log files and compare content", "[VectorNavDataLogger][flow]")
{
#if !__APPLE__ // This test continously fails on MacOS but can be recovered by restarting the test a few times. So we decided to disable it

    messageCounterImuDataCsv = 0;
    messageCounterImuLogCsv = 0;
    messageCounterImuLogVnb = 0;
    messageCounterGnssDataCsv = 0;
    messageCounterGnssLogCsv = 0;
    messageCounterGnssLogVnb = 0;

    data_vn310_imu_csv.clear();
    logs_vn310_imu_csv.clear();
    logs_vn310_imu_vnb.clear();
    data_vn310_gnss_csv.clear();
    logs_vn310_gnss_csv.clear();
    logs_vn310_gnss_vnb.clear();

    auto logger = initializeTestLogger();

    // ###########################################################################################################
    //                                         VectorNavDataLogger.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/vn310-imu.csv") (2)              VectorNavDataLogger("logs/vn310-imu.csv") (4)
    //                    (1) Binary Output |>  --(11)-->  |> Binary Output (3)
    //                                          \
    //                                           \          VectorNavDataLogger("logs/vn310-imu.vnb") (9)
    //                                            (12)-->  |> Binary Output (10)
    //
    // VectorNavFile("data/vn310-gnss.csv") (6)              VectorNavDataLogger("logs/vn310-gnss.csv") (14)
    //                     (7) Binary Output |>  --(19)-->  |> Binary Output (15)
    //                                           \
    //                                            \          VectorNavDataLogger("logs/vn310-gnss.vnb") (17)
    //                                             (20)-->  |> Binary Output (18)
    //
    // ###########################################################################################################

    REQUIRE(testFlow("test/flow/Nodes/DataLogger/IMU/VectorNavDataLogger.flow"));

    // ###########################################################################################################
    //                                       VectorNavDataLoggerCheck.flow
    // ###########################################################################################################
    //
    // VectorNavFile("data/VectorNav/FixedSize/vn310-imu.csv") (2)
    //                                         (1) Binary Output |>  --(90)->  |> (88) Terminator (89)
    constexpr size_t PIN_ID_VN_IMU_SRC = 88;
    //
    // VectorNavFile("logs/vn310-imu.csv") (28)
    //                    (29) Binary Output |>  --(106)->  |> (93) Terminator (92)
    constexpr size_t PIN_ID_VN_IMU_LOG_CSV = 93;
    //
    // VectorNavFile("logs/vn310-imu.vnb") (31)
    //                    (32) Binary Output |>  --(107)->  |> (96) Terminator (95)
    constexpr size_t PIN_ID_VN_IMU_LOG_BIN = 96;
    //
    //
    // VectorNavFile("data/VectorNav/FixedSize/vn310-gnss.csv") (6)
    //                                         (7) Binary Output |>  --(108)->  |> (99) Terminator (98)
    constexpr size_t PIN_ID_VN_GNSS_SRC = 99;
    //
    // VectorNavFile("logs/vn310-gnss.csv") (34)
    //                    (35) Binary Output |>  --(109)->  |> (102) Terminator (101)
    constexpr size_t PIN_ID_VN_GNSS_LOG_CSV = 102;
    //
    // VectorNavFile("logs/vn310-gnss.vnb") (37)
    //                    (38) Binary Output |>  --(110)->  |> (105) Terminator (104)
    constexpr size_t PIN_ID_VN_GNSS_LOG_BIN = 105;
    //
    // ###########################################################################################################

    // -------------------------------------------------- IMU ----------------------------------------------------

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_VN_IMU_SRC, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterImuDataCsv++;

        LOG_TRACE("messageCounterImuDataCsv = {}, messageCounterImuLogCsv = {}, messageCounterImuLogVnb = {}",
                  fmt::streamed(messageCounterImuDataCsv),
                  fmt::streamed(messageCounterImuLogCsv),
                  fmt::streamed(messageCounterImuLogVnb));

        std::lock_guard lk(comparisonMutex);
        data_vn310_imu_csv.push_back(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()));

        compareObservations(data_vn310_imu_csv, logs_vn310_imu_csv, logs_vn310_imu_vnb);
    });

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_VN_IMU_LOG_CSV, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterImuLogCsv++;

        LOG_TRACE("messageCounterImuDataCsv = {}, messageCounterImuLogCsv = {}, messageCounterImuLogVnb = {}",
                  fmt::streamed(messageCounterImuDataCsv),
                  fmt::streamed(messageCounterImuLogCsv),
                  fmt::streamed(messageCounterImuLogVnb));

        std::lock_guard lk(comparisonMutex);
        logs_vn310_imu_csv.push_back(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()));

        compareObservations(data_vn310_imu_csv, logs_vn310_imu_csv, logs_vn310_imu_vnb);
    });

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_VN_IMU_LOG_BIN, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterImuLogVnb++;

        LOG_TRACE("messageCounterImuDataCsv = {}, messageCounterImuLogCsv = {}, messageCounterImuLogVnb = {}",
                  fmt::streamed(messageCounterImuDataCsv),
                  fmt::streamed(messageCounterImuLogCsv),
                  fmt::streamed(messageCounterImuLogVnb));

        std::lock_guard lk(comparisonMutex);
        logs_vn310_imu_vnb.push_back(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()));

        compareObservations(data_vn310_imu_csv, logs_vn310_imu_csv, logs_vn310_imu_vnb);
    });

    // ------------------------------------------------- GNSS ----------------------------------------------------

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_VN_GNSS_SRC, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterGnssDataCsv++;

        LOG_TRACE("messageCounterGnssDataCsv = {}, messageCounterGnssLogCsv = {}, messageCounterGnssLogVnb = {}",
                  fmt::streamed(messageCounterGnssDataCsv),
                  fmt::streamed(messageCounterGnssLogCsv),
                  fmt::streamed(messageCounterGnssLogVnb));

        std::lock_guard lk(comparisonMutex);
        data_vn310_gnss_csv.push_back(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()));

        compareObservations(data_vn310_gnss_csv, logs_vn310_gnss_csv, logs_vn310_gnss_vnb);
    });

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_VN_GNSS_LOG_CSV, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterGnssLogCsv++;

        LOG_TRACE("messageCounterGnssDataCsv = {}, messageCounterGnssLogCsv = {}, messageCounterGnssLogVnb = {}",
                  fmt::streamed(messageCounterGnssDataCsv),
                  fmt::streamed(messageCounterGnssLogCsv),
                  fmt::streamed(messageCounterGnssLogVnb));

        std::lock_guard lk(comparisonMutex);
        logs_vn310_gnss_csv.push_back(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()));

        compareObservations(data_vn310_gnss_csv, logs_vn310_gnss_csv, logs_vn310_gnss_vnb);
    });

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_VN_GNSS_LOG_BIN, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounterGnssLogVnb++;

        LOG_TRACE("messageCounterGnssDataCsv = {}, messageCounterGnssLogCsv = {}, messageCounterGnssLogVnb = {}",
                  fmt::streamed(messageCounterGnssDataCsv),
                  fmt::streamed(messageCounterGnssLogCsv),
                  fmt::streamed(messageCounterGnssLogVnb));

        std::lock_guard lk(comparisonMutex);
        logs_vn310_gnss_vnb.push_back(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()));

        compareObservations(data_vn310_gnss_csv, logs_vn310_gnss_csv, logs_vn310_gnss_vnb);
    });

    REQUIRE(testFlow("test/flow/Nodes/DataLogger/IMU/VectorNavDataLoggerCheck.flow"));

    CHECK(messageCounterImuDataCsv == MESSAGE_COUNT_IMU);
    CHECK(messageCounterImuLogCsv == MESSAGE_COUNT_IMU);
    CHECK(messageCounterImuLogVnb == MESSAGE_COUNT_IMU);
    CHECK(messageCounterGnssDataCsv == MESSAGE_COUNT_GNSS);
    CHECK(messageCounterGnssLogCsv == MESSAGE_COUNT_GNSS);
    CHECK(messageCounterGnssLogVnb == MESSAGE_COUNT_GNSS);

#endif
}

} // namespace NAV::TESTS::VectorNavDataLoggerTests
