// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file VectorNavFileTests.cpp
/// @brief Tests for the VectorNavFile node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-21

#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>

#include "FlowTester.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

#include "VectorNavFileTestsData.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TEST::VectorNavFileTests
{
auto extractBit(auto& group, auto value)
{
    auto ret = group & value;
    group &= ~value;
    return ret;
}

constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();
constexpr double EPSILON_FLOAT = 10.0 * std::numeric_limits<float>::epsilon();

namespace FixedSize
{

void compareImuObservation(const std::shared_ptr<const NAV::VectorNavBinaryOutput>& obs, size_t messageCounterImuData)
{
    // ------------------------------------------------ InsTime --------------------------------------------------
    REQUIRE(!obs->insTime.empty());

    REQUIRE(obs->insTime.toGPSweekTow().gpsCycle == static_cast<int32_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GpsCycle)));
    REQUIRE(obs->insTime.toGPSweekTow().gpsWeek == static_cast<int32_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GpsWeek)));
    REQUIRE(obs->insTime.toGPSweekTow().tow == Approx(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GpsTow)).margin(EPSILON));

    // ----------------------------------------------- TimeGroup -------------------------------------------------
    REQUIRE(obs->timeOutputs != nullptr);

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP));
    REQUIRE(obs->timeOutputs->timeStartup == static_cast<uint64_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStartup)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW));
    REQUIRE(obs->timeOutputs->gpsTow == static_cast<uint64_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_GpsTow)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK));
    REQUIRE(obs->timeOutputs->gpsWeek == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_GpsWeek)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS));
    REQUIRE(obs->timeOutputs->timeStatus.timeOk() == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStatus_timeOk)));
    REQUIRE(obs->timeOutputs->timeStatus.dateOk() == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStatus_dateOk)));
    REQUIRE(obs->timeOutputs->timeStatus.utcTimeValid() == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Time_TimeStatus_utcTimeValid)));

    REQUIRE(obs->timeOutputs->timeField == vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);

    // ----------------------------------------------- ImuGroup --------------------------------------------------
    REQUIRE(obs->imuOutputs != nullptr);

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG));
    REQUIRE(obs->imuOutputs->uncompMag(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompMag_X)));
    REQUIRE(obs->imuOutputs->uncompMag(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompMag_Y)));
    REQUIRE(obs->imuOutputs->uncompMag(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompMag_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL));
    REQUIRE(obs->imuOutputs->uncompAccel(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompAccel_X)));
    REQUIRE(obs->imuOutputs->uncompAccel(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompAccel_Y)));
    REQUIRE(obs->imuOutputs->uncompAccel(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompAccel_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO));
    REQUIRE(obs->imuOutputs->uncompGyro(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompGyro_X)));
    REQUIRE(obs->imuOutputs->uncompGyro(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompGyro_Y)));
    REQUIRE(obs->imuOutputs->uncompGyro(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_UncompGyro_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_TEMP));
    REQUIRE(obs->imuOutputs->temp == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Temp)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_PRES));
    REQUIRE(obs->imuOutputs->pres == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Pres)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA));
    REQUIRE(obs->imuOutputs->deltaTime == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTime)));
    REQUIRE(obs->imuOutputs->deltaTheta(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTheta_X)));
    REQUIRE(obs->imuOutputs->deltaTheta(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTheta_Y)));
    REQUIRE(obs->imuOutputs->deltaTheta(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaTheta_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL));
    REQUIRE(obs->imuOutputs->deltaV(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaVel_X)));
    REQUIRE(obs->imuOutputs->deltaV(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaVel_Y)));
    REQUIRE(obs->imuOutputs->deltaV(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_DeltaVel_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_MAG));
    REQUIRE(obs->imuOutputs->mag(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Mag_X)));
    REQUIRE(obs->imuOutputs->mag(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Mag_Y)));
    REQUIRE(obs->imuOutputs->mag(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Mag_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL));
    REQUIRE(obs->imuOutputs->accel(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Accel_X)));
    REQUIRE(obs->imuOutputs->accel(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Accel_Y)));
    REQUIRE(obs->imuOutputs->accel(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_Accel_Z)));

    REQUIRE(extractBit(obs->imuOutputs->imuField, vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE));
    REQUIRE(obs->imuOutputs->angularRate(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_AngularRate_X)));
    REQUIRE(obs->imuOutputs->angularRate(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_AngularRate_Y)));
    REQUIRE(obs->imuOutputs->angularRate(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_IMU_AngularRate_Z)));

    REQUIRE(obs->imuOutputs->imuField == vn::protocol::uart::ImuGroup::IMUGROUP_NONE);

    // ---------------------------------------------- GpsGroup 1 -------------------------------------------------
    REQUIRE(obs->gnss1Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW));
    REQUIRE(obs->gnss1Outputs->tow == static_cast<uint64_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_Tow)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK));
    REQUIRE(obs->gnss1Outputs->week == static_cast<uint16_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_Week)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss1Outputs->timeInfo.leapSeconds == static_cast<int8_t>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_GNSS1_TimeInfo_LeapSeconds)));

    REQUIRE(obs->gnss1Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // --------------------------------------------- AttitudeGroup -----------------------------------------------
    REQUIRE(obs->attitudeOutputs != nullptr);

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL));
    REQUIRE(obs->attitudeOutputs->ypr(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YawPitchRoll_Y)));
    REQUIRE(obs->attitudeOutputs->ypr(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YawPitchRoll_P)));
    REQUIRE(obs->attitudeOutputs->ypr(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YawPitchRoll_R)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION));
    REQUIRE(obs->attitudeOutputs->qtn.w() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_w)));
    REQUIRE(obs->attitudeOutputs->qtn.x() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_x)));
    REQUIRE(obs->attitudeOutputs->qtn.y() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_y)));
    REQUIRE(obs->attitudeOutputs->qtn.z() == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_Quaternion_z)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU));
    REQUIRE(obs->attitudeOutputs->yprU(0) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YprU_Y)));
    REQUIRE(obs->attitudeOutputs->yprU(1) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YprU_P)));
    REQUIRE(obs->attitudeOutputs->yprU(2) == static_cast<float>(IMU_REFERENCE_DATA.at(messageCounterImuData).at(ImuRef_Att_YprU_R)));

    REQUIRE(obs->attitudeOutputs->attitudeField == vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);

    // ----------------------------------------------- InsGroup --------------------------------------------------
    REQUIRE(obs->insOutputs == nullptr);

    // ---------------------------------------------- GpsGroup 2 -------------------------------------------------
    REQUIRE(obs->gnss2Outputs == nullptr);
}

TEST_CASE("[VectorNavFile][flow] Read 'data/VectorNav/FixedSize/vn310-imu.csv' and compare content with hardcoded values", "[VectorNavFile][flow]")
{
    Logger logger;

    // ##########################################################################################################
    //                                            VectorNavFile.flow
    // ##########################################################################################################
    //
    //   VectorNavFile (2)                 Plot (8)
    //      (1) Binary Output |>  --(9)->  |> Pin 1 (3)
    //
    // ##########################################################################################################

    nm::RegisterPreInitCallback([&]() { dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = "VectorNav/FixedSize/vn310-imu.csv"; });

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(3, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareImuObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile.flow"));

    REQUIRE(messageCounter == IMU_REFERENCE_DATA.size());
}

TEST_CASE("[VectorNavFile][flow] Read 'data/VectorNav/FixedSize/vn310-imu.vnb' and compare content with hardcoded values", "[VectorNavFile][flow]")
{
    Logger logger;

    // ##########################################################################################################
    //                                            VectorNavFile.flow
    // ##########################################################################################################
    //
    //   VectorNavFile (2)                 Plot (8)
    //      (1) Binary Output |>  --(9)->  |> Pin 1 (3)
    //
    // ##########################################################################################################

    nm::RegisterPreInitCallback([&]() { dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = "VectorNav/FixedSize/vn310-imu.vnb"; });

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(3, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareImuObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile.flow"));

    REQUIRE(messageCounter == IMU_REFERENCE_DATA.size());
}

void compareGnssObservation(const std::shared_ptr<const NAV::VectorNavBinaryOutput>& obs, size_t messageCounterGnssData)
{
    // ------------------------------------------------ InsTime --------------------------------------------------
    REQUIRE(!obs->insTime.empty());

    REQUIRE(obs->insTime.toGPSweekTow().gpsCycle == static_cast<int32_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GpsCycle)));
    REQUIRE(obs->insTime.toGPSweekTow().gpsWeek == static_cast<int32_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GpsWeek)));
    REQUIRE(obs->insTime.toGPSweekTow().tow == Approx(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GpsTow)).margin(EPSILON));

    // ----------------------------------------------- TimeGroup -------------------------------------------------
    REQUIRE(obs->timeOutputs != nullptr);

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP));
    REQUIRE(obs->timeOutputs->timeStartup == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStartup)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW));
    REQUIRE(obs->timeOutputs->gpsTow == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_GpsTow)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK));
    REQUIRE(obs->timeOutputs->gpsWeek == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_GpsWeek)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC));
    REQUIRE(obs->timeOutputs->timeUtc.year == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_year)));
    REQUIRE(obs->timeOutputs->timeUtc.month == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_month)));
    REQUIRE(obs->timeOutputs->timeUtc.day == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_day)));
    REQUIRE(obs->timeOutputs->timeUtc.hour == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_hour)));
    REQUIRE(obs->timeOutputs->timeUtc.min == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_min)));
    REQUIRE(obs->timeOutputs->timeUtc.sec == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_sec)));
    REQUIRE(obs->timeOutputs->timeUtc.ms == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeUTC_ms)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS));
    REQUIRE(obs->timeOutputs->timeStatus.timeOk() == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStatus_timeOk)));
    REQUIRE(obs->timeOutputs->timeStatus.dateOk() == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStatus_dateOk)));
    REQUIRE(obs->timeOutputs->timeStatus.utcTimeValid() == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Time_TimeStatus_utcTimeValid)));

    REQUIRE(obs->timeOutputs->timeField == vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);

    // ----------------------------------------------- ImuGroup --------------------------------------------------
    REQUIRE(obs->imuOutputs == nullptr);

    // ---------------------------------------------- GpsGroup 1 -------------------------------------------------
    REQUIRE(obs->gnss1Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_UTC));
    REQUIRE(obs->gnss1Outputs->timeUtc.year == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_year)));
    REQUIRE(obs->gnss1Outputs->timeUtc.month == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_month)));
    REQUIRE(obs->gnss1Outputs->timeUtc.day == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_day)));
    REQUIRE(obs->gnss1Outputs->timeUtc.hour == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_hour)));
    REQUIRE(obs->gnss1Outputs->timeUtc.min == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_min)));
    REQUIRE(obs->gnss1Outputs->timeUtc.sec == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_sec)));
    REQUIRE(obs->gnss1Outputs->timeUtc.ms == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_UTC_ms)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW));
    REQUIRE(obs->gnss1Outputs->tow == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_Tow)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK));
    REQUIRE(obs->gnss1Outputs->week == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_Week)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS));
    REQUIRE(obs->gnss1Outputs->numSats == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_NumSats)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX));
    REQUIRE(obs->gnss1Outputs->fix == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_Fix)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA));
    REQUIRE(obs->gnss1Outputs->posLla(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosLla_latitude)));
    REQUIRE(obs->gnss1Outputs->posLla(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosLla_longitude)));
    REQUIRE(obs->gnss1Outputs->posLla(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosLla_altitude)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF));
    REQUIRE(obs->gnss1Outputs->posEcef(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosEcef_X)));
    REQUIRE(obs->gnss1Outputs->posEcef(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosEcef_Y)));
    REQUIRE(obs->gnss1Outputs->posEcef(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosEcef_Z)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELNED));
    REQUIRE(obs->gnss1Outputs->velNed(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelNed_N)));
    REQUIRE(obs->gnss1Outputs->velNed(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelNed_E)));
    REQUIRE(obs->gnss1Outputs->velNed(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelNed_D)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF));
    REQUIRE(obs->gnss1Outputs->velEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelEcef_X)));
    REQUIRE(obs->gnss1Outputs->velEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelEcef_Y)));
    REQUIRE(obs->gnss1Outputs->velEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelEcef_Z)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSU));
    REQUIRE(obs->gnss1Outputs->posU(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosU_N)));
    REQUIRE(obs->gnss1Outputs->posU(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosU_E)));
    REQUIRE(obs->gnss1Outputs->posU(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_PosU_D)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELU));
    REQUIRE(obs->gnss1Outputs->velU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_VelU)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU));
    REQUIRE(obs->gnss1Outputs->timeU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeU)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss1Outputs->timeInfo.leapSeconds == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_TimeInfo_LeapSeconds)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_DOP));
    REQUIRE(obs->gnss1Outputs->dop.gDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_g)));
    REQUIRE(obs->gnss1Outputs->dop.pDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_p)));
    REQUIRE(obs->gnss1Outputs->dop.tDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_t)));
    REQUIRE(obs->gnss1Outputs->dop.vDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_v)));
    REQUIRE(obs->gnss1Outputs->dop.hDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_h)));
    REQUIRE(obs->gnss1Outputs->dop.nDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_n)));
    REQUIRE(obs->gnss1Outputs->dop.eDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS1_DOP_e)));

    REQUIRE(obs->gnss1Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // --------------------------------------------- AttitudeGroup -----------------------------------------------
    REQUIRE(obs->attitudeOutputs != nullptr);

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL));
    REQUIRE(obs->attitudeOutputs->ypr(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YawPitchRoll_Y)));
    REQUIRE(obs->attitudeOutputs->ypr(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YawPitchRoll_P)));
    REQUIRE(obs->attitudeOutputs->ypr(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YawPitchRoll_R)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION));
    REQUIRE(obs->attitudeOutputs->qtn.w() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_w)));
    REQUIRE(obs->attitudeOutputs->qtn.x() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_x)));
    REQUIRE(obs->attitudeOutputs->qtn.y() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_y)));
    REQUIRE(obs->attitudeOutputs->qtn.z() == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_Quaternion_z)));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU));
    REQUIRE(obs->attitudeOutputs->yprU(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YprU_Y)));
    REQUIRE(obs->attitudeOutputs->yprU(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YprU_P)));
    REQUIRE(obs->attitudeOutputs->yprU(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_Att_YprU_R)));

    REQUIRE(obs->attitudeOutputs->attitudeField == vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);

    // ----------------------------------------------- InsGroup --------------------------------------------------
    REQUIRE(obs->insOutputs != nullptr);

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS));
    REQUIRE(obs->insOutputs->insStatus.mode() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Mode)));
    REQUIRE(obs->insOutputs->insStatus.gpsFix() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_GpsFix)));
    REQUIRE(obs->insOutputs->insStatus.errorIMU() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Error_IMU)));
    REQUIRE(obs->insOutputs->insStatus.errorMagPres() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Error_MagPres)));
    REQUIRE(obs->insOutputs->insStatus.errorGnss() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_Error_GNSS)));
    REQUIRE(obs->insOutputs->insStatus.gpsHeadingIns() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_GpsHeadingIns)));
    REQUIRE(obs->insOutputs->insStatus.gpsCompass() == static_cast<bool>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_InsStatus_GpsCompass)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSLLA));
    REQUIRE(obs->insOutputs->posLla(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosLla_latitude)));
    REQUIRE(obs->insOutputs->posLla(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosLla_longitude)));
    REQUIRE(obs->insOutputs->posLla(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosLla_altitude)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSECEF));
    REQUIRE(obs->insOutputs->posEcef(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosEcef_X)));
    REQUIRE(obs->insOutputs->posEcef(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosEcef_Y)));
    REQUIRE(obs->insOutputs->posEcef(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELBODY));
    REQUIRE(obs->insOutputs->velBody(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelBody_X)));
    REQUIRE(obs->insOutputs->velBody(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelBody_Y)));
    REQUIRE(obs->insOutputs->velBody(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelBody_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELNED));
    REQUIRE(obs->insOutputs->velNed(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelNed_N)));
    REQUIRE(obs->insOutputs->velNed(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelNed_E)));
    REQUIRE(obs->insOutputs->velNed(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelNed_D)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELECEF));
    REQUIRE(obs->insOutputs->velEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelEcef_X)));
    REQUIRE(obs->insOutputs->velEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelEcef_Y)));
    REQUIRE(obs->insOutputs->velEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_MAGECEF));
    REQUIRE(obs->insOutputs->magEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_MagEcef_X)));
    REQUIRE(obs->insOutputs->magEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_MagEcef_Y)));
    REQUIRE(obs->insOutputs->magEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_MagEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF));
    REQUIRE(obs->insOutputs->accelEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_AccelEcef_X)));
    REQUIRE(obs->insOutputs->accelEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_AccelEcef_Y)));
    REQUIRE(obs->insOutputs->accelEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_AccelEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF));
    REQUIRE(obs->insOutputs->linearAccelEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_LinearAccelEcef_X)));
    REQUIRE(obs->insOutputs->linearAccelEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_LinearAccelEcef_Y)));
    REQUIRE(obs->insOutputs->linearAccelEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_LinearAccelEcef_Z)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSU));
    REQUIRE(obs->insOutputs->posU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_PosU)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELU));
    REQUIRE(obs->insOutputs->velU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_INS_VelU)));

    REQUIRE(obs->insOutputs->insField == vn::protocol::uart::InsGroup::INSGROUP_NONE);

    // ---------------------------------------------- GpsGroup 2 -------------------------------------------------
    REQUIRE(obs->gnss2Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_UTC));
    REQUIRE(obs->gnss2Outputs->timeUtc.year == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_year)));
    REQUIRE(obs->gnss2Outputs->timeUtc.month == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_month)));
    REQUIRE(obs->gnss2Outputs->timeUtc.day == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_day)));
    REQUIRE(obs->gnss2Outputs->timeUtc.hour == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_hour)));
    REQUIRE(obs->gnss2Outputs->timeUtc.min == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_min)));
    REQUIRE(obs->gnss2Outputs->timeUtc.sec == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_sec)));
    REQUIRE(obs->gnss2Outputs->timeUtc.ms == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_UTC_ms)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TOW));
    REQUIRE(obs->gnss2Outputs->tow == static_cast<uint64_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_Tow)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_WEEK));
    REQUIRE(obs->gnss2Outputs->week == static_cast<uint16_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_Week)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS));
    REQUIRE(obs->gnss2Outputs->numSats == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_NumSats)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX));
    REQUIRE(obs->gnss2Outputs->fix == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_Fix)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA));
    REQUIRE(obs->gnss2Outputs->posLla(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosLla_latitude)));
    REQUIRE(obs->gnss2Outputs->posLla(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosLla_longitude)));
    REQUIRE(obs->gnss2Outputs->posLla(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosLla_altitude)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF));
    REQUIRE(obs->gnss2Outputs->posEcef(0) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosEcef_X)));
    REQUIRE(obs->gnss2Outputs->posEcef(1) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosEcef_Y)));
    REQUIRE(obs->gnss2Outputs->posEcef(2) == static_cast<double>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosEcef_Z)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELNED));
    REQUIRE(obs->gnss2Outputs->velNed(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelNed_N)));
    REQUIRE(obs->gnss2Outputs->velNed(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelNed_E)));
    REQUIRE(obs->gnss2Outputs->velNed(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelNed_D)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF));
    REQUIRE(obs->gnss2Outputs->velEcef(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelEcef_X)));
    REQUIRE(obs->gnss2Outputs->velEcef(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelEcef_Y)));
    REQUIRE(obs->gnss2Outputs->velEcef(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelEcef_Z)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_POSU));
    REQUIRE(obs->gnss2Outputs->posU(0) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosU_N)));
    REQUIRE(obs->gnss2Outputs->posU(1) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosU_E)));
    REQUIRE(obs->gnss2Outputs->posU(2) == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_PosU_D)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_VELU));
    REQUIRE(obs->gnss2Outputs->velU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_VelU)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU));
    REQUIRE(obs->gnss2Outputs->timeU == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeU)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss2Outputs->timeInfo.leapSeconds == static_cast<int8_t>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_TimeInfo_LeapSeconds)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_DOP));
    REQUIRE(obs->gnss2Outputs->dop.gDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_g)));
    REQUIRE(obs->gnss2Outputs->dop.pDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_p)));
    REQUIRE(obs->gnss2Outputs->dop.tDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_t)));
    REQUIRE(obs->gnss2Outputs->dop.vDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_v)));
    REQUIRE(obs->gnss2Outputs->dop.hDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_h)));
    REQUIRE(obs->gnss2Outputs->dop.nDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_n)));
    REQUIRE(obs->gnss2Outputs->dop.eDop == static_cast<float>(GNSS_REFERENCE_DATA.at(messageCounterGnssData).at(GnssRef_GNSS2_DOP_e)));

    REQUIRE(obs->gnss2Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
}

TEST_CASE("[VectorNavFile][flow] Read 'data/VectorNav/FixedSize/vn310-gnss.csv' and compare content with hardcoded values", "[VectorNavFile][flow]")
{
    Logger logger;

    // ##########################################################################################################
    //                                            VectorNavFile.flow
    // ##########################################################################################################
    //
    //   VectorNavFile (2)                 Plot (8)
    //      (1) Binary Output |>  --(9)->  |> Pin 1 (3)
    //
    // ##########################################################################################################

    nm::RegisterPreInitCallback([&]() { dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = "VectorNav/FixedSize/vn310-gnss.csv"; });

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(3, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareGnssObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile.flow"));

    REQUIRE(messageCounter == GNSS_REFERENCE_DATA.size());
}

TEST_CASE("[VectorNavFile][flow] Read 'data/VectorNav/FixedSize/vn310-gnss.vnb' and compare content with hardcoded values", "[VectorNavFile][flow]")
{
    Logger logger;

    // ##########################################################################################################
    //                                            VectorNavFile.flow
    // ##########################################################################################################
    //
    //   VectorNavFile (2)                 Plot (8)
    //      (1) Binary Output |>  --(9)->  |> Pin 1 (3)
    //
    // ##########################################################################################################

    nm::RegisterPreInitCallback([&]() { dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = "VectorNav/FixedSize/vn310-gnss.vnb"; });

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(3, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareGnssObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile.flow"));

    REQUIRE(messageCounter == GNSS_REFERENCE_DATA.size());
}

} // namespace FixedSize

namespace DynamicSize
{

void compareDynamicSizeObservation(const std::shared_ptr<const NAV::VectorNavBinaryOutput>& obs, size_t messageCounterData)
{
    // ------------------------------------------------ InsTime --------------------------------------------------
    REQUIRE(!obs->insTime.empty());

    REQUIRE(obs->insTime.toGPSweekTow().gpsCycle == static_cast<int32_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GpsCycle)));
    REQUIRE(obs->insTime.toGPSweekTow().gpsWeek == static_cast<int32_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GpsWeek)));
    REQUIRE(obs->insTime.toGPSweekTow().tow == Approx(REFERENCE_DATA.at(messageCounterData).at(Ref_GpsTow)).margin(EPSILON));

    // ----------------------------------------------- TimeGroup -------------------------------------------------
    REQUIRE(obs->timeOutputs != nullptr);

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP));
    REQUIRE(obs->timeOutputs->timeStartup == static_cast<uint64_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeStartup)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS));
    REQUIRE(obs->timeOutputs->timeGps == static_cast<uint64_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeGps)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW));
    REQUIRE(obs->timeOutputs->gpsTow == static_cast<uint64_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_GpsTow)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK));
    REQUIRE(obs->timeOutputs->gpsWeek == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_GpsWeek)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN));
    REQUIRE(obs->timeOutputs->timeSyncIn == static_cast<uint64_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeSyncIn)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS));
    REQUIRE(obs->timeOutputs->timePPS == static_cast<uint64_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeGpsPps)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC));
    REQUIRE(obs->timeOutputs->timeUtc.year == static_cast<int8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_year)));
    REQUIRE(obs->timeOutputs->timeUtc.month == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_month)));
    REQUIRE(obs->timeOutputs->timeUtc.day == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_day)));
    REQUIRE(obs->timeOutputs->timeUtc.hour == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_hour)));
    REQUIRE(obs->timeOutputs->timeUtc.min == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_min)));
    REQUIRE(obs->timeOutputs->timeUtc.sec == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_sec)));
    REQUIRE(obs->timeOutputs->timeUtc.ms == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeUTC_ms)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT));
    REQUIRE(obs->timeOutputs->syncInCnt == static_cast<uint32_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_SyncInCnt)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT));
    REQUIRE(obs->timeOutputs->syncOutCnt == static_cast<uint32_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_SyncOutCnt)));

    REQUIRE(extractBit(obs->timeOutputs->timeField, vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS));
    REQUIRE(obs->timeOutputs->timeStatus.timeOk() == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeStatus_timeOk)));
    REQUIRE(obs->timeOutputs->timeStatus.dateOk() == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeStatus_dateOk)));
    REQUIRE(obs->timeOutputs->timeStatus.utcTimeValid() == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_Time_TimeStatus_utcTimeValid)));

    REQUIRE(obs->timeOutputs->timeField == vn::protocol::uart::TimeGroup::TIMEGROUP_NONE);

    // ----------------------------------------------- ImuGroup --------------------------------------------------
    REQUIRE(obs->imuOutputs == nullptr);

    // ---------------------------------------------- GpsGroup 1 -------------------------------------------------
    REQUIRE(obs->gnss1Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS));
    REQUIRE(obs->gnss1Outputs->numSats == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_NumSats)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX));
    REQUIRE(obs->gnss1Outputs->fix == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_Fix)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss1Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss1Outputs->timeInfo.leapSeconds == static_cast<int8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_TimeInfo_LeapSeconds)));

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO));
    REQUIRE(obs->gnss1Outputs->satInfo.numSats == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_SatInfo_NumSats)));
    REQUIRE(obs->gnss1Outputs->satInfo.satellites.size() == static_cast<size_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_SatInfo_NumSats)));
    REQUIRE(obs->gnss1Outputs->satInfo.satellites.size() == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).size());
    for (size_t i = 0; i < obs->gnss1Outputs->satInfo.satellites.size(); ++i)
    {
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).sys == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).sys);
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).svId == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).svId);
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).flags == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).flags);
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).cno == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).cno);
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).qi == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).qi);
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).el == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).el);
        REQUIRE(obs->gnss1Outputs->satInfo.satellites.at(i).az == REFERENCE_DATA_SAT_INFO_1.at(messageCounterData).at(i).az);
    }

    REQUIRE(extractBit(obs->gnss1Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS));
    REQUIRE(obs->gnss1Outputs->raw.tow == static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_RawMeas_Tow)));
    REQUIRE(obs->gnss1Outputs->raw.week == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_RawMeas_Week)));
    REQUIRE(obs->gnss1Outputs->raw.numSats == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_RawMeas_NumSats)));
    REQUIRE(obs->gnss1Outputs->raw.satellites.size() == static_cast<size_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS1_RawMeas_NumSats)));
    REQUIRE(obs->gnss1Outputs->raw.satellites.size() == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).size());
    for (size_t i = 0; i < obs->gnss1Outputs->raw.satellites.size(); ++i)
    {
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).sys == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).sys);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).svId == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).svId);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).freq == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).freq);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).chan == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).chan);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).slot == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).slot);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).cno == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).cno);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).flags == REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).flags);
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).pr == Approx(REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).pr).margin(EPSILON));
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).cp == Approx(REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).cp).margin(EPSILON));
        REQUIRE(obs->gnss1Outputs->raw.satellites.at(i).dp == Approx(REFERENCE_DATA_RAW_MEAS_1.at(messageCounterData).at(i).dp).margin(EPSILON_FLOAT));
    }

    REQUIRE(obs->gnss1Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);

    // --------------------------------------------- AttitudeGroup -----------------------------------------------
    REQUIRE(obs->attitudeOutputs != nullptr);

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL));
    REQUIRE(obs->attitudeOutputs->ypr(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_YawPitchRoll_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->ypr(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_YawPitchRoll_P))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->ypr(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_YawPitchRoll_R))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION));
    REQUIRE(obs->attitudeOutputs->qtn.w() == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Quaternion_w))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->qtn.x() == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Quaternion_x))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->qtn.y() == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Quaternion_y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->qtn.z() == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Quaternion_z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM));
    REQUIRE(obs->attitudeOutputs->dcm(0, 0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_00))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(0, 1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_01))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(0, 2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_02))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(1, 0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_10))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(1, 1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_11))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(1, 2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_12))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(2, 0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_20))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(2, 1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_21))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->dcm(2, 2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Dcm_22))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED));
    REQUIRE(obs->attitudeOutputs->magNed(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Mag_N))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->magNed(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Mag_E))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->magNed(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Mag_D))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED));
    REQUIRE(obs->attitudeOutputs->accelNed(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Accel_N))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->accelNed(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Accel_E))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->accelNed(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_Accel_D))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY));
    REQUIRE(obs->attitudeOutputs->linearAccelBody(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_LinearAccelBody_X))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->linearAccelBody(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_LinearAccelBody_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->linearAccelBody(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_LinearAccelBody_Z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED));
    REQUIRE(obs->attitudeOutputs->linearAccelNed(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_LinearAccel_N))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->linearAccelNed(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_LinearAccel_E))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->linearAccelNed(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_LinearAccel_D))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->attitudeOutputs->attitudeField, vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU));
    REQUIRE(obs->attitudeOutputs->yprU(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_YprU_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->yprU(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_YprU_P))).margin(EPSILON_FLOAT));
    REQUIRE(obs->attitudeOutputs->yprU(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_Att_YprU_R))).margin(EPSILON_FLOAT));

    REQUIRE(obs->attitudeOutputs->attitudeField == vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_NONE);

    // ----------------------------------------------- InsGroup --------------------------------------------------
    REQUIRE(obs->insOutputs != nullptr);

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS));
    REQUIRE(obs->insOutputs->insStatus.mode() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_Mode)));
    REQUIRE(obs->insOutputs->insStatus.gpsFix() == static_cast<bool>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_GpsFix)));
    REQUIRE(obs->insOutputs->insStatus.errorIMU() == static_cast<bool>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_Error_IMU)));
    REQUIRE(obs->insOutputs->insStatus.errorMagPres() == static_cast<bool>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_Error_MagPres)));
    REQUIRE(obs->insOutputs->insStatus.errorGnss() == static_cast<bool>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_Error_GNSS)));
    REQUIRE(obs->insOutputs->insStatus.gpsHeadingIns() == static_cast<bool>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_GpsHeadingIns)));
    REQUIRE(obs->insOutputs->insStatus.gpsCompass() == static_cast<bool>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_InsStatus_GpsCompass)));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSLLA));
    REQUIRE(obs->insOutputs->posLla(0) == Approx(static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosLla_latitude))).margin(EPSILON));
    REQUIRE(obs->insOutputs->posLla(1) == Approx(static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosLla_longitude))).margin(EPSILON));
    REQUIRE(obs->insOutputs->posLla(2) == Approx(static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosLla_altitude))).margin(EPSILON));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSECEF));
    REQUIRE(obs->insOutputs->posEcef(0) == Approx(static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosEcef_X))).margin(EPSILON));
    REQUIRE(obs->insOutputs->posEcef(1) == Approx(static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosEcef_Y))).margin(EPSILON));
    REQUIRE(obs->insOutputs->posEcef(2) == Approx(static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosEcef_Z))).margin(EPSILON));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELBODY));
    REQUIRE(obs->insOutputs->velBody(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelBody_X))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->velBody(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelBody_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->velBody(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelBody_Z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELNED));
    REQUIRE(obs->insOutputs->velNed(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelNed_N))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->velNed(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelNed_E))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->velNed(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelNed_D))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELECEF));
    REQUIRE(obs->insOutputs->velEcef(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelEcef_X))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->velEcef(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelEcef_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->velEcef(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelEcef_Z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_MAGECEF));
    REQUIRE(obs->insOutputs->magEcef(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_MagEcef_X))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->magEcef(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_MagEcef_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->magEcef(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_MagEcef_Z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF));
    REQUIRE(obs->insOutputs->accelEcef(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_AccelEcef_X))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->accelEcef(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_AccelEcef_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->accelEcef(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_AccelEcef_Z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF));
    REQUIRE(obs->insOutputs->linearAccelEcef(0) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_LinearAccelEcef_X))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->linearAccelEcef(1) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_LinearAccelEcef_Y))).margin(EPSILON_FLOAT));
    REQUIRE(obs->insOutputs->linearAccelEcef(2) == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_LinearAccelEcef_Z))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_POSU));
    REQUIRE(obs->insOutputs->posU == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_PosU))).margin(EPSILON_FLOAT));

    REQUIRE(extractBit(obs->insOutputs->insField, vn::protocol::uart::InsGroup::INSGROUP_VELU));
    REQUIRE(obs->insOutputs->velU == Approx(static_cast<float>(REFERENCE_DATA.at(messageCounterData).at(Ref_INS_VelU))).margin(EPSILON_FLOAT));

    REQUIRE(obs->insOutputs->insField == vn::protocol::uart::InsGroup::INSGROUP_NONE);

    // ---------------------------------------------- GpsGroup 2 -------------------------------------------------
    REQUIRE(obs->gnss2Outputs != nullptr);

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS));
    REQUIRE(obs->gnss2Outputs->numSats == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_NumSats)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_FIX));
    REQUIRE(obs->gnss2Outputs->fix == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_Fix)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.timeOk() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_TimeInfo_Status_timeOk)));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.dateOk() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_TimeInfo_Status_dateOk)));
    REQUIRE(obs->gnss2Outputs->timeInfo.status.utcTimeValid() == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_TimeInfo_Status_utcTimeValid)));
    REQUIRE(obs->gnss2Outputs->timeInfo.leapSeconds == static_cast<int8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_TimeInfo_LeapSeconds)));

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO));
    REQUIRE(obs->gnss2Outputs->satInfo.numSats == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_SatInfo_NumSats)));
    REQUIRE(obs->gnss2Outputs->satInfo.satellites.size() == static_cast<size_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_SatInfo_NumSats)));
    REQUIRE(obs->gnss2Outputs->satInfo.satellites.size() == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).size());
    for (size_t i = 0; i < obs->gnss2Outputs->satInfo.satellites.size(); ++i)
    {
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).sys == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).sys);
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).svId == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).svId);
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).flags == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).flags);
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).cno == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).cno);
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).qi == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).qi);
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).el == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).el);
        REQUIRE(obs->gnss2Outputs->satInfo.satellites.at(i).az == REFERENCE_DATA_SAT_INFO_2.at(messageCounterData).at(i).az);
    }

    REQUIRE(extractBit(obs->gnss2Outputs->gnssField, vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS));
    REQUIRE(obs->gnss2Outputs->raw.tow == static_cast<double>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_RawMeas_Tow)));
    REQUIRE(obs->gnss2Outputs->raw.week == static_cast<uint16_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_RawMeas_Week)));
    REQUIRE(obs->gnss2Outputs->raw.numSats == static_cast<uint8_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_RawMeas_NumSats)));
    REQUIRE(obs->gnss2Outputs->raw.satellites.size() == static_cast<size_t>(REFERENCE_DATA.at(messageCounterData).at(Ref_GNSS2_RawMeas_NumSats)));
    REQUIRE(obs->gnss2Outputs->raw.satellites.size() == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).size());
    for (size_t i = 0; i < obs->gnss2Outputs->raw.satellites.size(); ++i)
    {
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).sys == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).sys);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).svId == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).svId);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).freq == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).freq);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).chan == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).chan);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).slot == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).slot);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).cno == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).cno);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).flags == REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).flags);
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).pr == Approx(REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).pr).margin(EPSILON));
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).cp == Approx(REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).cp).margin(EPSILON));
        REQUIRE(obs->gnss2Outputs->raw.satellites.at(i).dp == Approx(REFERENCE_DATA_RAW_MEAS_2.at(messageCounterData).at(i).dp).margin(EPSILON_FLOAT));
    }

    REQUIRE(obs->gnss2Outputs->gnssField == vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
}

TEST_CASE("[VectorNavFile][flow] Read 'data/VectorNav/DynamicSize/vn310-gnss.csv' and compare content with hardcoded values", "[VectorNavFile][flow]")
{
    Logger logger;

    // ##########################################################################################################
    //                                            VectorNavFile.flow
    // ##########################################################################################################
    //
    //   VectorNavFile (2)                 Plot (8)
    //      (1) Binary Output |>  --(9)->  |> Pin 1 (3)
    //
    // ##########################################################################################################

    nm::RegisterPreInitCallback([&]() { dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = "VectorNav/DynamicSize/vn310-gnss.csv"; });

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(3, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareDynamicSizeObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile.flow"));

    REQUIRE(messageCounter == REFERENCE_DATA.size());
}

TEST_CASE("[VectorNavFile][flow] Read 'data/VectorNav/DynamicSize/vn310-gnss.vnb' and compare content with hardcoded values", "[VectorNavFile][flow]")
{
    Logger logger;

    // ##########################################################################################################
    //                                            VectorNavFile.flow
    // ##########################################################################################################
    //
    //   VectorNavFile (2)                 Plot (8)
    //      (1) Binary Output |>  --(9)->  |> Pin 1 (3)
    //
    // ##########################################################################################################

    nm::RegisterPreInitCallback([&]() { dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = "VectorNav/DynamicSize/vn310-gnss.vnb"; });

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(3, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareDynamicSizeObservation(std::dynamic_pointer_cast<const NAV::VectorNavBinaryOutput>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/VectorNavFile.flow"));

    REQUIRE(messageCounter == REFERENCE_DATA.size());
}

} // namespace DynamicSize

} // namespace NAV::TEST::VectorNavFileTests
