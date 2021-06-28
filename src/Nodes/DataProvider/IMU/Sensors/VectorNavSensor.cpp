#include "VectorNavSensor.hpp"

#include "util/Debug.hpp"
#include "util/Logger.hpp"
#include "vn/searcher.h"

#include "gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include <imgui_internal.h>

#include "NodeData/IMU/VectorNavObs.hpp"

#include "util/Time/TimeBase.hpp"

#include <map>

const std::array<NAV::VectorNavSensor::BinaryGroupData, 15> NAV::VectorNavSensor::binaryGroupCommon = { {
    /*  0 */ { "TimeStartup", vn::protocol::uart::CommonGroup::COMMONGROUP_TIMESTARTUP, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Time since startup.\n\nThe system time since startup measured in nano seconds. The time since startup is based upon the internal\nTXCO oscillator for the MCU. The accuracy of the internal TXCO is +/- 20ppm (-40C to 85C). This field is\nequivalent to the TimeStartup field in group 2."); } },
    /*  1 */ { "TimeGps", vn::protocol::uart::CommonGroup::COMMONGROUP_TIMEGPS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GPS time.\n\nThe absolute GPS time since start of GPS epoch 1980 expressed in nano seconds. This field is equivalent to\nthe TimeGps field in group 2."); } },
    /*  2 */ { "TimeSyncIn", vn::protocol::uart::CommonGroup::COMMONGROUP_SYNCINCNT, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Time since last SyncIn trigger.\n\nThe time since the last SyncIn trigger event expressed in nano seconds. This field is equivalent to the\nTimeSyncIn field in group 2."); } },
    /*  3 */ { "YawPitchRoll", vn::protocol::uart::CommonGroup::COMMONGROUP_YAWPITCHROLL, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Estimated attitude as yaw pitch and roll angles.\n\nThe estimated attitude Yaw, Pitch, and Roll angles measured in degrees. The attitude is given as a 3,2,1 Euler\nangle sequence describing the body frame with respect to the local North East Down (NED) frame. This field\nis equivalent to the YawPitchRoll field in group 5.\n\nYaw [+/- 180°]\nPitch [+/- 90°]\nRoll [+/- 180°]"); } },
    /*  4 */ { "Quaternion", vn::protocol::uart::CommonGroup::COMMONGROUP_QUATERNION, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Estimated attitude as a quaternion.\n\nThe estimated attitude quaternion. The last term is the scalar value. The attitude is given as the body frame\nwith respect to the local North East Down (NED) frame. This field is equivalent to the Quaternion field in\ngroup 5."); } },
    /*  5 */ { "AngularRate", vn::protocol::uart::CommonGroup::COMMONGROUP_ANGULARRATE, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated angular rate.\n\nThe estimated angular rate measured in rad/s. The angular rates are compensated by the onboard filter bias\nestimates. The angular rate is expressed in the body frame. This field is equivalent to the AngularRate field\nin group 3."); } },
    /*  6 */ { "Position", vn::protocol::uart::CommonGroup::COMMONGROUP_POSITION, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Estimated position. (LLA)\n\nThe estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectively. This field\nis equivalent to the PosLla field in group 6."); } },
    /*  7 */ { "Velocity", vn::protocol::uart::CommonGroup::COMMONGROUP_VELOCITY, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Estimated velocity. (NED)\n\nThe estimated velocity in the North East Down (NED) frame, given in m/s. This field is equivalent to the\nVelNed field in group 6."); } },
    /*  8 */ { "Accel", vn::protocol::uart::CommonGroup::COMMONGROUP_ACCEL, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Estimated acceleration (compensated). (Body)\n\nThe estimated acceleration in the body frame, given in m/s^2. This acceleration includes gravity and has\nbeen bias compensated by the onboard INS Kalman filter. This field is equivalent to the Accel field in group 3."); } },
    /*  9 */ { "Imu", vn::protocol::uart::CommonGroup::COMMONGROUP_IMU, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Calibrated uncompensated gyro and accelerometer measurements.\n\nThe uncompensated IMU acceleration and angular rate measurements. The acceleration is given in m/s^2,\nand the angular rate is given in rad/s. These measurements correspond to the calibrated angular rate and\nacceleration measurements straight from the IMU. The measurements have not been corrected for bias\noffset by the onboard Kalman filter. These are equivalent to the UncompAccel and UncompGyro fields in\ngroup 3."); } },
    /* 10 */ { "MagPres", vn::protocol::uart::CommonGroup::COMMONGROUP_MAGPRES, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Calibrated magnetic (compensated), temperature, and pressure measurements.\n\nThe compensated magnetic, temperature, and pressure measurements from the IMU. The magnetic\nmeasurement is given in Gauss, and has been corrected for hard/soft iron corrections (if enabled). The\ntemperature measurement is given in Celsius. The pressure measurement is given in kPa. This field is\nequivalent to the Mag, Temp, and Pres fields in group 3.\n\nThe IP-68 enclosure on the tactical series forms an airtight (hermetic) seal isolating the internal\nsensors from the external environment. The pressure sensor is internal to this seal, and as such\nwill not measure the outside environment atmospheric pressure. It will instead read the pressure\ninside the sealed enclosure. The purpose of this sensor is to provide a means of ensuring the\nseal integrity over the lifetime of the product. Based on the Ideal Gas Law the ratio of pressure\ndivided by temperature should remain constant over both time and environmental temperature.\nWhen this is no longer the case, it can be assumed that the seal integrity has been compromised."); } },
    /* 11 */ { "DeltaTheta", vn::protocol::uart::CommonGroup::COMMONGROUP_DELTATHETA, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Delta time, theta, and velocity.\n\nThe delta time, angle, and velocity measurements. The delta time (dtime) is the time interval that the delta\nangle and velocities are integrated over. The delta theta (dtheta) is the delta rotation angles incurred due to\nrotation, by the local body reference frame, since the last time the values were outputted by the device. The\ndelta velocity (dvel) is the delta velocity incurred due to motion, by the local body reference frame, since the\nlast time the values were outputted by the device. The frame of reference of these delta measurements are\ndetermined by the IntegrationFrame field in the Delta Theta and Delta Velocity Configuration Register\n(Register 82). These delta angles and delta velocities are calculated based upon the onboard coning and\nsculling integration performed onboard the sensor at the full IMU rate (default 800Hz). The integration for\nboth the delta angles and velocities are reset each time either of the values are either polled or sent out due\nto a scheduled asynchronous ASCII or binary output. Delta Theta and Delta Velocity values correctly capture\nthe nonlinearities involved in measuring motion from a rotating strapdown platform (as opposed to the older\nmechanically inertial navigation systems), thus providing you with the ability to integrate velocity and angular\nrate at much lower speeds (say for example 10 Hz, reducing bandwidth and computational complexity), while\nstill maintaining the same numeric precision as if you had performed the integration at the full IMU\nmeasurement rate of 800Hz. This field is equivalent to the DeltaTheta and DeltaVel fields in group 3 with the\ninclusion of the additional delta time parameter."); } },
    /* 12 */ { "InsStatus", vn::protocol::uart::CommonGroup::COMMONGROUP_INSSTATUS, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("INS status.\n\nThe INS status bitfield. This field is equivalent to the InsSatus field in group 6. See INS Solution LLA Register\nfor more information on the individual bits in this field."); } },
    /* 13 */ { "SyncInCnt", vn::protocol::uart::CommonGroup::COMMONGROUP_SYNCINCNT, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("SyncIn count.\n\nThe number of SyncIn trigger events that have occurred. This field is equivalent to the SyncInCnt field in\ngroup 2."); } },
    /* 14 */ { "TimeGpsPps", vn::protocol::uart::CommonGroup::COMMONGROUP_TIMEGPSPPS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Time since last GNSS PPS trigger.\n\nThe time since the last GPS PPS trigger event expressed in nano seconds. This field is equivalent to the\nTimePPS field in group 2."); } },
} };

const std::array<NAV::VectorNavSensor::BinaryGroupData, 10> NAV::VectorNavSensor::binaryGroupTime = { {
    /*  0 */ { "TimeStartup", vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Time since startup.\n\nThe system time since startup measured in nano seconds. The time since startup is based upon the internal\nTXCO oscillator for the MCU. The accuracy of the internal TXCO is +/- 20ppm (-40C to 85C)."); } },
    /*  1 */ { "TimeGps", vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Absolute GPS time.\n\nThe absolute GPS time since start of GPS epoch 1980 expressed in nano seconds."); } },
    /*  2 */ { "GpsTow", vn::protocol::uart::TimeGroup::TIMEGROUP_GPSTOW, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Time since start of GPS week.\n\nThe time since the start of the current GPS time week expressed in nano seconds."); } },
    /*  3 */ { "GpsWeek", vn::protocol::uart::TimeGroup::TIMEGROUP_GPSWEEK, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GPS week.\n\nThe current GPS week."); } },
    /*  4 */ { "TimeSyncIn", vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESYNCIN, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Time since last SyncIn trigger.\n\nThe time since the last SyncIn event trigger expressed in nano seconds."); } },
    /*  5 */ { "TimeGpsPps", vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEGPSPPS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Time since last GPS PPS trigger.\n\nThe time since the last GPS PPS trigger event expressed in nano seconds."); } },
    /*  6 */ { "TimeUTC", vn::protocol::uart::TimeGroup::TIMEGROUP_TIMEUTC, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("UTC time.\n\nThe current UTC time. The year is given as a signed byte year offset from the year 2000. For example the\nyear 2013 would be given as year 13."); } },
    /*  7 */ { "SyncInCnt", vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCINCNT, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("SyncIn trigger count.\n\nThe number of SyncIn trigger events that have occurred."); } },
    /*  8 */ { "SyncOutCnt", vn::protocol::uart::TimeGroup::TIMEGROUP_SYNCOUTCNT, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("SyncOut trigger count.\n\nThe number of SyncOut trigger events that have occurred."); } },
    /*  9 */ { "TimeStatus", vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTATUS, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Time valid status flags.");
                                                                                                                                                if (ImGui::BeginTable("VectorNavTimeStatusTooltip", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                {
                                                                                                                                                    ImGui::TableSetupColumn("Bit Offset");
                                                                                                                                                    ImGui::TableSetupColumn("Field");
                                                                                                                                                    ImGui::TableSetupColumn("Description");
                                                                                                                                                    ImGui::TableHeadersRow();

                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("timeOk");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("1 - GpsTow is valid");

                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("dateOk");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("1 - TimeGps and GpsWeek are valid");

                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("utcTimeValid");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("1 - UTC time is valid");

                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("3 - 7");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("resv");
                                                                                                                                                    ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for future use");

                                                                                                                                                    ImGui::EndTable();
                                                                                                                                                } } },
} };

const std::array<NAV::VectorNavSensor::BinaryGroupData, 11> NAV::VectorNavSensor::binaryGroupIMU{ {
    /*  0 */ { "ImuStatus", vn::protocol::uart::ImuGroup::IMUGROUP_IMUSTATUS, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Status is reserved for future use. Not currently used in the current code, as such will always report 0."); } },
    /*  1 */ { "UncompMag", vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Uncompensated magnetic measurement.\n\nThe IMU magnetic field measured in units of Gauss, given in the body frame. This measurement is\ncompensated by the static calibration (individual factory calibration stored in flash), and the user\ncompensation, however it is not compensated by the onboard Hard/Soft Iron estimator."); } },
    /*  2 */ { "UncompAccel", vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Uncompensated acceleration measurement.\n\nThe IMU acceleration measured in units of m/s^2, given in the body frame. This measurement is\ncompensated by the static calibration (individual factory calibration stored in flash), however it is not\ncompensated by any dynamic calibration such as bias compensation from the onboard INS Kalman filter."); } },
    /*  3 */ { "UncompGyro", vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Uncompensated angular rate measurement.\n\nThe IMU angular rate measured in units of rad/s, given in the body frame. This measurement is compensated\nby the static calibration (individual factory calibration stored in flash), however it is not compensated by any\ndynamic calibration such as the bias compensation from the onboard AHRS/INS Kalman filters."); } },
    /*  4 */ { "Temp", vn::protocol::uart::ImuGroup::IMUGROUP_TEMP, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Temperature measurement.\n\nThe IMU temperature measured in units of Celsius."); } },
    /*  5 */ { "Pres", vn::protocol::uart::ImuGroup::IMUGROUP_PRES, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Pressure measurement.\n\nThe IMU pressure measured in kilopascals. This is an absolute pressure measurement. Typical pressure at sea level would be around 100 kPa."); } },
    /*  6 */ { "DeltaTheta", vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Delta theta angles.\n\nThe delta time (dtime) is the time interval that the delta angle and velocities are integrated over. The delta\ntheta (dtheta) is the delta rotation angles incurred due to rotation, by the local body reference frame, since\nthe last time the values were outputted by the device. The delta velocity (dvel) is the delta velocity incurred\ndue to motion, by the local body reference frame, since the last time the values were outputted by the device.\nThe frame of reference of these delta measurements are determined by the IntegrationFrame field in the\nDelta Theta and Delta Velocity Configuration Register (Register 82). These delta angles and delta velocities\nare calculated based upon the onboard coning and sculling integration performed onboard the sensor at the\nfull IMU rate (default 800Hz). The integration for both the delta angles and velocities are reset each time\neither of the values are either polled or sent out due to a scheduled asynchronous ASCII or binary output.\nDelta Theta and Delta Velocity values correctly capture the nonlinearities involved in measuring motion from\na rotating strapdown platform (as opposed to the older mechanically inertial navigation systems), thus\nproviding you with the ability to integrate velocity and angular rate at much lower speeds (say for example\n10 Hz, reducing bandwidth and computational complexity), while still maintaining the same numeric\nprecision as if you had performed the integration at the full IMU measurement rate of 800Hz. Time is given\nin seconds. Delta angles are given in degrees."); } },
    /*  7 */ { "DeltaVel", vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Delta velocity.\n\nThe delta velocity (dvel) is the delta velocity incurred due to motion, since the last time the values were output\nby the device. The delta velocities are calculated based upon the onboard conning and sculling integration\nperformed onboard the sensor at the IMU sampling rate (nominally 800Hz). The integration for the delta\nvelocities are reset each time the values are either polled or sent out due to a scheduled asynchronous ASCII\nor binary output. Delta velocity is given in meters per second."); } },
    /*  8 */ { "Mag", vn::protocol::uart::ImuGroup::IMUGROUP_MAG, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated magnetic measurement.\n\nThe IMU compensated magnetic field measured units of Gauss, and given in the body frame. This\nmeasurement is compensated by the static calibration (individual factory calibration stored in flash), the user\ncompensation, and the dynamic calibration from the onboard Hard/Soft Iron estimator."); } },
    /*  9 */ { "Accel", vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated acceleration measurement.\n\nThe compensated acceleration measured in units of m/s^2, and given in the body frame. This measurement\nis compensated by the static calibration (individual factory calibration stored in flash), the user\ncompensation, and the dynamic bias compensation from the onboard INS Kalman filter."); } },
    /* 10 */ { "AngularRate", vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated angular rate measurement.\n\nThe compensated angular rate measured in units of rad/s, and given in the body frame. This measurement\nis compensated by the static calibration (individual factor calibration stored in flash), the user compensation,\nand the dynamic bias compensation from the onboard INS Kalman filter."); } },
} };

const std::array<NAV::VectorNavSensor::BinaryGroupData, 16> NAV::VectorNavSensor::binaryGroupGNSS{ {
    /*  0 */ { "UTC", vn::protocol::uart::GpsGroup::GPSGROUP_UTC, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GPS UTC Time\n\nThe current UTC time. The year is given as a signed byte year offset from the year 2000. For example the\nyear 2013 would be given as year 13."); } },
    /*  1 */ { "Tow", vn::protocol::uart::GpsGroup::GPSGROUP_TOW, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GPS time of week\n\nThe GPS time of week given in nano seconds."); } },
    /*  2 */ { "Week", vn::protocol::uart::GpsGroup::GPSGROUP_WEEK, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GPS week\n\nThe current GPS week."); } },
    /*  3 */ { "NumSats", vn::protocol::uart::GpsGroup::GPSGROUP_NUMSATS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Number of tracked satellites\n\nThe number of tracked GNSS satellites."); } },
    /*  4 */ { "Fix", vn::protocol::uart::GpsGroup::GPSGROUP_FIX, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS fix\n\nThe current GNSS fix.");
                                                                                                                                                          if (ImGui::BeginTable("VectorNavFixTooltip", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                                                                                                                                                          {
                                                                                                                                                              ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                              ImGui::TableSetupColumn("Description", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                              ImGui::TableHeadersRow();

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("No fix");

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("Time only");

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("2D");

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("3D");

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("SBAS");

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("7");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("RTK Float (only GNSS1)");

                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("8");
                                                                                                                                                              ImGui::TableNextColumn(); ImGui::TextUnformatted("RTK Fixed (only GNSS1)");

                                                                                                                                                              ImGui::EndTable();
                                                                                                                                                          } } },
    /*  5 */ { "PosLla", vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS position (latitude, longitude, altitude)\n\nThe current GNSS position measurement given as the geodetic latitude, longitude and altitude above the\nellipsoid. The units are in [deg, deg, m] respectively."); } },
    /*  6 */ { "PosEcef", vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS position (ECEF)\n\nThe current GNSS position given in the Earth centered Earth fixed (ECEF) coordinate frame, given in meters."); } },
    /*  7 */ { "VelNed", vn::protocol::uart::GpsGroup::GPSGROUP_VELNED, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS velocity (NED)\n\nThe current GNSS velocity in the North East Down (NED) coordinate frame, given in m/s."); } },
    /*  8 */ { "VelEcef", vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS velocity (ECEF)\n\nThe current GNSS velocity in the Earth centered Earth fixed (ECEF) coordinate frame, given in m/s."); } },
    /*  9 */ { "PosU", vn::protocol::uart::GpsGroup::GPSGROUP_POSU, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS position uncertainty (NED)\n\nThe current GNSS position uncertainty in the North East Down (NED) coordinate frame, given in meters (1 Sigma)."); } },
    /* 10 */ { "VelU", vn::protocol::uart::GpsGroup::GPSGROUP_VELU, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS velocity uncertainty\n\nThe current GNSS velocity uncertainty, given in m/s (1 Sigma)."); } },
    /* 11 */ { "TimeU", vn::protocol::uart::GpsGroup::GPSGROUP_TIMEU, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS time uncertainty\n\nThe current GPS time uncertainty, given in seconds (1 Sigma)."); } },
    /* 12 */ { "TimeInfo", vn::protocol::uart::GpsGroup::GPSGROUP_TIMEINFO, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS time status and leap seconds\n\nFlags for valid GPS TOW, week number and UTC and current leap seconds.");
                                                                                                                                                                    if (ImGui::BeginTable("VectorNavTimeInfoTooltip", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                    {
                                                                                                                                                                        ImGui::TableSetupColumn("Bit Offset");
                                                                                                                                                                        ImGui::TableSetupColumn("Field");
                                                                                                                                                                        ImGui::TableSetupColumn("Description");
                                                                                                                                                                        ImGui::TableHeadersRow();

                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("timeOk");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("1 - GpsTow is valid");

                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("dateOk");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("1 - TimeGps and GpsWeek are valid");

                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("utcTimeValid");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("1 - UTC time is valid");

                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("3 - 7");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("resv");
                                                                                                                                                                        ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for future use");

                                                                                                                                                                        ImGui::EndTable();
                                                                                                                                                                    } } },
    /* 13 */ { "DOP", vn::protocol::uart::GpsGroup::GPSGROUP_DOP, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Dilution of precision"); } },
    /* 14 */ { "SatInfo", vn::protocol::uart::GpsGroup::GPSGROUP_SATINFO, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Satellite Information\n\nInformation and measurements pertaining to each GNSS satellite in view.\n\nSatInfo Element:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatInfoTooltip", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Name");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("sys");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("GNSS constellation indicator. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("svId");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Space vehicle Id");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("flags");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Tracking info flags. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("cno");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Carrier-to-noise density ratio (signal strength) [dB-Hz]");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("qi");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Quality Indicator. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("el");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Elevation in degrees");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("az");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Azimuth angle in degrees");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::BeginChild("VectorNavSatInfoTooltipGNSSConstelationChild", ImVec2(230, 217));
                                                                                                                                                                  ImGui::TextUnformatted("\nGNSS constellation indicator:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatInfoTooltipGNSSConstelation", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Value");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("GPS");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("SBAS");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Galileo");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("BeiDou");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("IMES");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("QZSS");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("GLONASS");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::EndChild();
                                                                                                                                                                  ImGui::SameLine();
                                                                                                                                                                  ImGui::BeginChild("VectorNavSatInfoTooltipFlagsChild", ImVec2(260, 217));
                                                                                                                                                                  ImGui::TextUnformatted("\nTracking info flags:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatInfoTooltipFlags", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Bit Offset");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Healthy");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Almanac");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Ephemeris");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Differential Correction");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Used for Navigation");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Azimuth / Elevation Valid");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Used for RTK");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::EndChild();
                                                                                                                                                                  ImGui::TextUnformatted("\nQuality Indicators:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatInfoTooltipQuality", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Value");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("No signal");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Searching signal");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Signal acquired");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Signal detected but unstable");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Code locked and time synchronized");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5, 6, 7");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Code and carrier locked and time synchronized");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  } } },
    /* 15 */ { "RawMeas", vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("GNSS Raw Measurements.\n\nSatRaw Element:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatRawTooltip", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Name");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("sys");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("GNSS constellation indicator. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("svId");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Space vehicle Id");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("freq");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Frequency indicator. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("chan");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Channel Indicator. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("slot");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Slot Id");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("cno");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Carrier-to-noise density ratio (signal strength) [dB-Hz]");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("flags");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Tracking info flags. See table below for details.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("pr");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Pseudorange measurement in meters.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("cp");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Carrier phase measurement in cycles.");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("dp");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Doppler measurement in Hz. Positive sign for approaching satellites.");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::BeginChild("VectorNavSatRawTooltipGNSSConstelationChild", ImVec2(180, 217));
                                                                                                                                                                  ImGui::TextUnformatted("\nConstellation indicator:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatRawTooltipGNSSConstelation", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Value");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("GPS");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("SBAS");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Galileo");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("BeiDou");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("IMES");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("QZSS");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("GLONASS");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::EndChild();
                                                                                                                                                                  ImGui::SameLine();
                                                                                                                                                                  ImGui::BeginChild("VectorNavSatRawTooltipFreqChild", ImVec2(270, 235));
                                                                                                                                                                  ImGui::TextUnformatted("\nFrequency indicator:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatRawTooltipFreq", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Value");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Rx Channel");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("L1(GPS,QZSS,SBAS), G1(GLO),\nE2-L1-E1(GAL), B1(BDS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("L2(GPS,QZSS), G2(GLO)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("L5(GPS,QZSS,SBAS), E5a(GAL)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("E6(GAL), LEX(QZSS), B3(BDS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("E5b(GAL), B2(BDS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("E5a+b(GAL)");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::EndChild();
                                                                                                                                                                  ImGui::SameLine();
                                                                                                                                                                  ImGui::BeginChild("VectorNavSatRawTooltipFlagChild", ImVec2(255, 260));
                                                                                                                                                                  ImGui::TextUnformatted("\nTracking info flags:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatRawTooltipFlags", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Bit Offset");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Searching");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Tracking");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Time Valid");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Code Lock");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Phase Lock");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Phase Half Ambiguity");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Phase Half Sub");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("7");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Phase Slip");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("8");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Pseudorange Smoothed");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  }
                                                                                                                                                                  ImGui::EndChild();
                                                                                                                                                                  ImGui::TextUnformatted("\nChannel indicator:");
                                                                                                                                                                  if (ImGui::BeginTable("VectorNavSatRawTooltipChan", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                  {
                                                                                                                                                                      ImGui::TableSetupColumn("Value");
                                                                                                                                                                      ImGui::TableSetupColumn("Description");
                                                                                                                                                                      ImGui::TableHeadersRow();

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("P-code (GPS,GLO)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("1");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("C/A-code (GPS,GLO,SBAS,QZSS), C chan (GAL)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("semi-codeless (GPS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Y-code (GPS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("M-code (GPS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("5");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("codeless (GPS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("A chan (GAL)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("7");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("B chan (GAL)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("8");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("I chan (GPS,GAL,QZSS,BDS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("9");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("Q chan (GPS,GAL,QZSS,BDS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("10");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("M chan (L2CGPS, L2CQZSS), D chan (GPS,QZSS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("11");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("L chan (L2CGPS, L2CQZSS), P chan (GPS,QZSS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("12");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("B+C chan (GAL), I+Q chan (GPS,GAL,QZSS,BDS),\nM+L chan (GPS,QZSS), D+P chan (GPS,QZSS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("13");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("based on Z-tracking (GPS)");

                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("14");
                                                                                                                                                                      ImGui::TableNextColumn(); ImGui::TextUnformatted("A+B+C (GAL)");

                                                                                                                                                                      ImGui::EndTable();
                                                                                                                                                                  } } },
} };

const std::array<NAV::VectorNavSensor::BinaryGroupData, 9> NAV::VectorNavSensor::binaryGroupAttitude{ {
    /*  0 */ { "VpeStatus", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_VPESTATUS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN100_VN110; }, []() { ImGui::TextUnformatted("VPE Status bitfield\n\n");
                                                                                                                                                                                      if (ImGui::BeginTable("VectorNavSatRawTooltipChan", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                                                                                                                                                                                      {
                                                                                                                                                                                          ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                                          ImGui::TableSetupColumn("Bit Offset", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                                          ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                                          ImGui::TableSetupColumn("Description", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                                          ImGui::TableHeadersRow();

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("AttitudeQuality");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("2 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Provides an indication of the quality of the attitude solution.\n0 - Excellent\n1 - Good\n2 - Bad\n3 - Not tracking");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("GyroSaturation");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("At least one gyro axis is currently saturated.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("GyroSaturationRecovery");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Filter is in the process of recovering from a gyro\nsaturation event.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("MagDisturbance");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("4");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("2 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("A magnetic DC disturbance has been detected.\n0 - No magnetic disturbance\n1 to 3 - Magnetic disturbance is present.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("MagSaturation");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("6");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("At least one magnetometer axis is currently saturated.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("AccDisturbance");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("7");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("2 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("A strong acceleration disturbance has been detected.\n0 - No acceleration disturbance.\n1 to 3 - Acceleration disturbance has been detected.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("AccSaturation");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("9");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("At least one accelerometer axis is currently saturated.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("10");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for internal use. May change state at run- time.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("KnownMagDisturbance");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("11");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("A known magnetic disturbance has been reported by\nthe user and the magnetometer is currently tuned out.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("KnownAccelDisturbance");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("12");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("A known acceleration disturbance has been reported by\nthe user and the accelerometer is currently tuned out.");

                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("13");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("3 bits");
                                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for future use.");

                                                                                                                                                                                          ImGui::EndTable();
                                                                                                                                                                                      } } },
    /*  1 */ { "YawPitchRoll", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Yaw Pitch Roll\n\nThe estimated attitude Yaw, Pitch, and Roll angles measured in degrees. The attitude is given as a 3,2,1\nEuler angle sequence describing the body frame with respect to the local North East Down (NED) frame.\n\nYaw [+/- 180°]\nPitch [+/- 90°]\nRoll [+/- 180°]"); } },
    /*  2 */ { "Quaternion", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Quaternion\n\nThe estimated attitude quaternion. The last term is the scalar value. The attitude is given as the body\nframe with respect to the local North East Down (NED) frame."); } },
    /*  3 */ { "DCM", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Directional Cosine Matrix\n\nThe estimated attitude directional cosine matrix given in column major order. The DCM maps vectors\nfrom the North East Down (NED) frame into the body frame."); } },
    /*  4 */ { "MagNed", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_MAGNED, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated magnetic (NED)\n\nThe current estimated magnetic field (Gauss), given in the North East Down (NED) frame. The current\nattitude solution is used to map the measurement from the measured body frame to the inertial (NED)\nframe. This measurement is compensated by both the static calibration (individual factory calibration\nstored in flash), and the dynamic calibration such as the user or onboard Hard/Soft Iron compensation\nregisters."); } },
    /*  5 */ { "AccelNed", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_ACCELNED, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated acceleration (NED)\n\nThe estimated acceleration (with gravity) reported in m/s^2, given in the North East Down (NED) frame.\nThe acceleration measurement has been bias compensated by the onboard INS filter. This measurement\nis attitude dependent, since the attitude is used to map the measurement from the body frame into the\ninertial (NED) frame. If the device is stationary and the INS filter is tracking, the measurement should be\nnominally equivalent to the gravity reference vector in the inertial frame (NED)."); } },
    /*  6 */ { "LinearAccelBody", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated linear acceleration (no gravity)\n\nThe estimated linear acceleration (without gravity) reported in m/s^2, and given in the body frame. The\nacceleration measurement has been bias compensated by the onboard INS filter, and the gravity\ncomponent has been removed using the current gravity reference vector model. This measurement is\nattitude dependent, since the attitude solution is required to map the gravity reference vector (known in\nthe inertial NED frame), into the body frame so that it can be removed from the measurement. If the\ndevice is stationary and the onboard INS filter is tracking, the measurement nominally will read 0 in all\nthree axes."); } },
    /*  7 */ { "LinearAccelNed", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_LINEARACCELNED, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Compensated linear acceleration (no gravity) (NED)\n\nThe estimated linear acceleration (without gravity) reported in m/s^2, and given in the North East Down\n(NED) frame. This measurement is attitude dependent as the attitude solution is used to map the\nmeasurement from the body frame into the inertial (NED) frame. This acceleration measurement has\nbeen bias compensated by the onboard INS filter, and the gravity component has been removed using the\ncurrent gravity reference vector estimate. If the device is stationary and the onboard INS filter is tracking,\nthe measurement nominally will read 0 in all three axes."); } },
    /*  8 */ { "YprU", vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YPRU, [](VectorNavModel /* sensorModel */) { return true; }, []() { ImGui::TextUnformatted("Yaw Pitch Roll uncertainty\n\nThe estimated attitude (Yaw, Pitch, Roll) uncertainty (1 Sigma), reported in degrees.\n\nThe estimated attitude (YprU) field is not valid when the INS Scenario mode in the INS Basic\nConfiguration register is set to AHRS mode. See the INS Basic Configuration Register in the INS\nsection for more details."); } },
} };

const std::array<NAV::VectorNavSensor::BinaryGroupData, 11> NAV::VectorNavSensor::binaryGroupINS{ {
    /*  0 */ { "InsStatus", vn::protocol::uart::InsGroup::INSGROUP_INSSTATUS, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Status bitfield:");
                                                                                                                                                                      if (ImGui::BeginTable("VectorNavInsStatusTooltip", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                                                                                                                                                                      {
                                                                                                                                                                          ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                          ImGui::TableSetupColumn("Bit Offset", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                          ImGui::TableSetupColumn("Format", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                          ImGui::TableSetupColumn("Description", ImGuiTableColumnFlags_WidthAutoResize);
                                                                                                                                                                          ImGui::TableHeadersRow();

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Mode");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("0");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("2 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Indicates the current mode of the INS filter.\n\n0 = Not tracking. GNSS Compass is initializing. Output heading is based on\nmagnetometer measurements.\n1 = Aligning.\nINS Filter is dynamically aligning.\nFor a stationary startup: GNSS Compass has initialized and INS Filter is\naligning from the magnetic heading to the GNSS Compass heading.\nFor a dynamic startup: INS Filter has initialized and is dynamically aligning to\nTrue North heading.\nIn operation, if the INS Filter drops from INS Mode 2 back down to 1, the\nattitude uncertainty has increased above 2 degrees.\n2 = Tracking. The INS Filter is tracking and operating within specification.\n3 = Loss of GNSS. A GNSS outage has lasted more than 45 seconds. The INS\nFilter will no longer update the position and velocity outputs, but the attitude\nremains valid.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("GpsFix");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("2");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Indicates whether the GNSS has a proper fix.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Error");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("3");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("4 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Sensor measurement error code. See table below.\n0 = No errors detected.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("7");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for internal use. May toggle state during runtime and should be ignored.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("GpsHeadingIns");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("8");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("In stationary operation, if set the INS Filter has fully aligned to the GNSS\nCompass solution.\nIn dynamic operation, the GNSS Compass solution is currently aiding the INS\nFilter heading solution.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("GpsCompass");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("9");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("1 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Indicates if the GNSS compass is operational and reporting a heading\nsolution.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("10");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("6 bits");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for internal use. These bits will toggle state and should be ignored.");

                                                                                                                                                                          ImGui::EndTable();
                                                                                                                                                                      }
                                                                                                                                                                      ImGui::TextUnformatted("\nError Bitfield:");
                                                                                                                                                                      if (ImGui::BeginTable("VectorNavInsStatusTooltipError", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0f, 0.0f)))
                                                                                                                                                                      {
                                                                                                                                                                          ImGui::TableSetupColumn("Name");
                                                                                                                                                                          ImGui::TableSetupColumn("Description");
                                                                                                                                                                          ImGui::TableHeadersRow();

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Reserved for future use and not currently used.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("IMU Error");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("High if IMU communication error is detected.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("Mag/Pres Error");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("High if Magnetometer or Pressure sensor error is detected.");

                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("GNSS Error");
                                                                                                                                                                          ImGui::TableNextColumn(); ImGui::TextUnformatted("High if GNSS communication error is detected.");

                                                                                                                                                                          ImGui::EndTable();
                                                                                                                                                                      } } },
    /*  1 */ { "PosLla", vn::protocol::uart::InsGroup::INSGROUP_POSLLA, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Position (latitude, longitude, altitude)\n\nThe estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectively."); } },
    /*  2 */ { "PosEcef", vn::protocol::uart::InsGroup::INSGROUP_POSECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Position (ECEF)\n\nThe estimated position given in the Earth centered Earth fixed (ECEF) frame, reported in meters."); } },
    /*  3 */ { "VelBody", vn::protocol::uart::InsGroup::INSGROUP_VELBODY, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Velocity (Body)\n\nThe estimated velocity in the body frame, given in m/s."); } },
    /*  4 */ { "VelNed", vn::protocol::uart::InsGroup::INSGROUP_VELNED, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Velocity (NED)\n\nThe estimated velocity in the North East Down (NED) frame, given in m/s."); } },
    /*  5 */ { "VelEcef", vn::protocol::uart::InsGroup::INSGROUP_VELECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Velocity (ECEF)\n\nThe estimated velocity in the Earth centered Earth fixed (ECEF) frame, given in m/s."); } },
    /*  6 */ { "MagEcef", vn::protocol::uart::InsGroup::INSGROUP_MAGECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Compensated magnetic (ECEF)\n\nThe compensated magnetic measurement in the Earth centered Earth fixed (ECEF) frame, given in Gauss."); } },
    /*  7 */ { "AccelEcef", vn::protocol::uart::InsGroup::INSGROUP_ACCELECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Compensated acceleration (ECEF)\n\nThe estimated acceleration (with gravity) reported in m/s^2, given in the Earth centered Earth fixed (ECEF)\nframe. The acceleration measurement has been bias compensated by the onboard INS filter. This\nmeasurement is attitude dependent, since the attitude is used to map the measurement from the body frame\ninto the inertial (ECEF) frame. If the device is stationary and the INS filter is tracking, the measurement\nshould be nominally equivalent to the gravity reference vector in the inertial frame (ECEF)."); } },
    /*  8 */ { "LinearAccelEcef", vn::protocol::uart::InsGroup::INSGROUP_LINEARACCELECEF, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Compensated linear acceleration (no gravity) (ECEF)\n\nThe estimated linear acceleration (without gravity) reported in m/s^2, and given in the Earth centered Earth\nfixed (ECEF) frame. This measurement is attitude dependent as the attitude solution is used to map the\nmeasurement from the body frame into the inertial (ECEF) frame. This acceleration measurement has been\nbias compensated by the onboard INS filter, and the gravity component has been removed using the current\ngravity reference vector estimate. If the device is stationary and the onboard INS filter is tracking, the\nmeasurement will nominally read 0 in all three axes."); } },
    /*  9 */ { "PosU", vn::protocol::uart::InsGroup::INSGROUP_POSU, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Position Uncertainty\n\nThe estimated uncertainty (1 Sigma) in the current position estimate, given in meters."); } },
    /* 10 */ { "VelU", vn::protocol::uart::InsGroup::INSGROUP_VELU, [](VectorNavModel sensorModel) { return sensorModel == VectorNavModel::VN310; }, []() { ImGui::TextUnformatted("Ins Velocity Uncertainty\n\nThe estimated uncertainty (1 Sigma) in the current velocity estimate, given in m/s."); } },
} };

NAV::VectorNavSensor::VectorNavSensor()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 917, 623 };

    nm::CreateOutputPin(this, "VectorNavObs", Pin::Type::Flow, NAV::VectorNavObs::type());

    dividerFrequency = []() {
        std::map<int, int, std::greater<>> divFreq;
        for (int freq = 1; freq <= IMU_DEFAULT_FREQUENCY; freq++)
        {
            int divider = static_cast<int>(std::round(IMU_DEFAULT_FREQUENCY / freq));

            if (!divFreq.contains(divider)
                || std::abs(divider - IMU_DEFAULT_FREQUENCY / freq) < std::abs(divider - IMU_DEFAULT_FREQUENCY / divFreq.at(divider)))
            {
                divFreq[divider] = freq;
            }
        }
        std::vector<uint16_t> divs;
        std::vector<std::string> freqs;
        for (auto& [divider, freq] : divFreq)
        {
            divs.push_back(static_cast<uint16_t>(divider));
            freqs.push_back(std::to_string(freq) + " Hz");
        }
        return std::make_pair(divs, freqs);
    }();
}

NAV::VectorNavSensor::~VectorNavSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavSensor::typeStatic()
{
    return "VectorNavSensor";
}

std::string NAV::VectorNavSensor::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavSensor::category()
{
    return "Data Provider";
}

void NAV::VectorNavSensor::guiConfig()
{
    if (ImGui::Combo("Sensor", reinterpret_cast<int*>(&sensorModel), "VN-100 / VN-110\0VN-310\0\0"))
    {
        LOG_DEBUG("{}: Sensor changed to {}", nameId(), sensorModel);
        flow::ApplyChanges();
        deinitializeNode();

        for (auto& item : binaryGroupCommon)
        {
            if (!item.isEnabled(sensorModel))
            {
                config.commonField &= ~vn::protocol::uart::CommonGroup(item.flagsValue);
            }
        }
        for (auto& item : binaryGroupTime)
        {
            if (!item.isEnabled(sensorModel))
            {
                config.timeField &= ~vn::protocol::uart::TimeGroup(item.flagsValue);
            }
        }
        for (auto& item : binaryGroupIMU)
        {
            if (!item.isEnabled(sensorModel))
            {
                config.imuField &= ~vn::protocol::uart::ImuGroup(item.flagsValue);
            }
        }
        for (auto& item : binaryGroupGNSS)
        {
            if (!item.isEnabled(sensorModel))
            {
                config.gnss1Field &= ~vn::protocol::uart::GpsGroup(item.flagsValue);
                config.gnss2Field &= ~vn::protocol::uart::GpsGroup(item.flagsValue);
            }
        }
        for (auto& item : binaryGroupAttitude)
        {
            if (!item.isEnabled(sensorModel))
            {
                config.attitudeField &= ~vn::protocol::uart::AttitudeGroup(item.flagsValue);
            }
        }
        for (auto& item : binaryGroupINS)
        {
            if (!item.isEnabled(sensorModel))
            {
                config.insField &= ~vn::protocol::uart::InsGroup(item.flagsValue);
            }
        }
    }

    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyUSB0", &sensorPort))
    {
        LOG_DEBUG("{}: SensorPort changed to {}", nameId(), sensorPort);
        flow::ApplyChanges();
        deinitializeNode();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("COM port where the sensor is attached to\n"
                             "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                             "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                             "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                             "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                             "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)");

    std::array<const char*, 10> items = { "Fastest", "9600", "19200", "38400", "57600", "115200", "128000", "230400", "460800", "921600" };
    if (ImGui::Combo("Baudrate", &selectedBaudrate, items.data(), items.size()))
    {
        LOG_DEBUG("{}: Baudrate changed to {}", nameId(), sensorBaudrate());
        flow::ApplyChanges();
        deinitializeNode();
    }

    const char* currentFrequency = (selectedFrequency >= 0 && static_cast<size_t>(selectedFrequency) < dividerFrequency.second.size())
                                       ? dividerFrequency.second.at(static_cast<size_t>(selectedFrequency)).c_str()
                                       : "Unknown";
    if (ImGui::SliderInt("Frequency", &selectedFrequency, 0, static_cast<int>(dividerFrequency.second.size()) - 1, currentFrequency))
    {
        LOG_DEBUG("{}: Frequency changed to {}", nameId(), dividerFrequency.second.at(static_cast<size_t>(selectedFrequency)));
        flow::ApplyChanges();
        deinitializeNode();
    }

    if (ImGui::BeginTable(fmt::format("##VectorNavSensorConfig ({})", id.AsPointer()).c_str(), 7,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Common", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("IMU", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("GNSS1", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("Attitude", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("INS", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableSetupColumn("GNSS2", ImGuiTableColumnFlags_WidthAutoResize);
        ImGui::TableHeadersRow();

        auto CheckboxFlags = [](int index, const char* label, int* flags, int flags_value, bool enabled = true) {
            ImGui::TableSetColumnIndex(index);

            if (!enabled)
            {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
            }

            ImGui::CheckboxFlags(label, flags, flags_value);

            if (!enabled)
            {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }
        };

        // --------------------------------------------- Common Group ------------------------------------------------
        for (size_t i = 0; i < 16; i++)
        {
            if (i < binaryGroupCommon.size() || i < binaryGroupTime.size() || i < binaryGroupIMU.size()
                || i < binaryGroupGNSS.size() || i < binaryGroupAttitude.size() || i < binaryGroupINS.size())
            {
                ImGui::TableNextRow();
            }
            if (i < binaryGroupCommon.size())
            {
                const auto& binaryGroupItem = binaryGroupCommon.at(i);
                CheckboxFlags(0, (binaryGroupItem.name + ("##Common" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.commonField), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < binaryGroupTime.size())
            {
                const auto& binaryGroupItem = binaryGroupTime.at(i);
                CheckboxFlags(1, (binaryGroupItem.name + ("##Time" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.timeField), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < binaryGroupIMU.size())
            {
                const auto& binaryGroupItem = binaryGroupIMU.at(i);
                CheckboxFlags(2, (binaryGroupItem.name + ("##IMU" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.imuField), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < binaryGroupGNSS.size())
            {
                const auto& binaryGroupItem = binaryGroupGNSS.at(i);
                CheckboxFlags(3, (binaryGroupItem.name + ("##GNSS1" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.gnss1Field), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < binaryGroupAttitude.size())
            {
                const auto& binaryGroupItem = binaryGroupAttitude.at(i);
                CheckboxFlags(4, (binaryGroupItem.name + ("##Attitude" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.attitudeField), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < binaryGroupINS.size())
            {
                const auto& binaryGroupItem = binaryGroupINS.at(i);
                CheckboxFlags(5, (binaryGroupItem.name + ("##INS" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.insField), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
            if (i < binaryGroupGNSS.size())
            {
                const auto& binaryGroupItem = binaryGroupGNSS.at(i);
                CheckboxFlags(6, (binaryGroupItem.name + ("##GNSS2" + std::to_string(size_t(id)))).c_str(), reinterpret_cast<int*>(&config.gnss2Field), binaryGroupItem.flagsValue, binaryGroupItem.isEnabled(sensorModel));
                if (ImGui::IsItemHovered() && binaryGroupItem.tooltip != nullptr)
                {
                    ImGui::BeginTooltip();
                    binaryGroupItem.tooltip();
                    ImGui::EndTooltip();
                }
            }
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::VectorNavSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();
    j["Frequency"] = dividerFrequency.second.at(static_cast<size_t>(selectedFrequency));

    return j;
}

void NAV::VectorNavSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
    if (j.contains("Frequency"))
    {
        std::string frequency;
        j.at("Frequency").get_to(frequency);
        for (size_t i = 0; i < dividerFrequency.second.size(); i++)
        {
            if (dividerFrequency.second.at(i) == frequency)
            {
                selectedFrequency = static_cast<int>(i);
                break;
            }
        }
    }
}

bool NAV::VectorNavSensor::resetNode()
{
    return true;
}

bool NAV::VectorNavSensor::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // Choose baudrate
    Baudrate targetBaudrate = sensorBaudrate() == BAUDRATE_FASTEST
                                  ? static_cast<Baudrate>(vn::sensors::VnSensor::supportedBaudrates()[vn::sensors::VnSensor::supportedBaudrates().size() - 1])
                                  : sensorBaudrate();

    Baudrate connectedBaudrate{};
    // Search for the VectorNav Sensor
    if (int32_t foundBaudrate = 0;
        vn::sensors::Searcher::search(sensorPort, &foundBaudrate))
    {
        // Sensor was found at specified port with the baudrate 'foundBaudrate'
        connectedBaudrate = static_cast<Baudrate>(foundBaudrate);
    }
    else if (std::vector<std::pair<std::string, uint32_t>> foundSensors = vn::sensors::Searcher::search();
             !foundSensors.empty())
    {
        if (foundSensors.size() == 1)
        {
            sensorPort = foundSensors.at(0).first;
            connectedBaudrate = static_cast<Baudrate>(foundSensors.at(0).second);
        }
        else
        {
            sensorPort = "";
            // Some VectorNav sensors where found, try to identify the wanted one by it's name
            for (auto [port, baudrate] : foundSensors)
            {
                vs.connect(port, baudrate);
                std::string modelNumber = vs.readModelNumber();
                vs.disconnect();

                LOG_DEBUG("{} found VectorNav Sensor {} on port {} with baudrate {}", nameId(), modelNumber, port, baudrate);

                // Regex search may be better, but simple find is used here
                if (modelNumber.find(name) != std::string::npos)
                {
                    sensorPort = port;
                    connectedBaudrate = static_cast<Baudrate>(baudrate);
                    break;
                }
            }
            // Sensor could not be identified
            if (sensorPort.empty())
            {
                // This point is also reached if a sensor is connected with USB but external power is off
                LOG_ERROR("{} could not connect", nameId());
                return false;
            }
        }
    }
    else
    {
        LOG_ERROR("{} could not connect. Is the sensor connected and do you have read permissions?", nameId());
        return false;
    }

    try
    {
        // Connect to the sensor (vs.verifySensorConnectivity does not have to be called as sensor is already tested)
        vs.connect(sensorPort, connectedBaudrate);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("{}: Failed to connect to sensor on port {} with baudrate {} with error: {}", nameId(),
                  sensorPort, connectedBaudrate, e.what());
        return false;
    }

    if (!vs.verifySensorConnectivity())
    {
        LOG_ERROR("{}: Connected to sensor on port {} with baudrate {} but sensor does not answer", nameId(),
                  sensorPort, connectedBaudrate);
        return false;
    }
    // Query the sensor's model number
    LOG_DEBUG("{} connected on port {} with baudrate {}", vs.readModelNumber(), sensorPort, connectedBaudrate);

    // Change Connection Baudrate
    if (targetBaudrate != connectedBaudrate)
    {
        auto suppBaud = vn::sensors::VnSensor::supportedBaudrates();
        if (std::find(suppBaud.begin(), suppBaud.end(), targetBaudrate) != suppBaud.end())
        {
            vs.changeBaudRate(targetBaudrate);
            LOG_DEBUG("{} baudrate changed to {}", nameId(), static_cast<size_t>(targetBaudrate));
        }
        else
        {
            LOG_ERROR("{} does not support baudrate {}", nameId(), static_cast<size_t>(targetBaudrate));
            return false;
        }
    }
    ASSERT(vs.readSerialBaudRate() == targetBaudrate, "Baudrate was not changed");

    // Change Heading Mode (and enable Filtering Mode, Tuning Mode)
    vn::sensors::VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
    vpeReg.headingMode = config.headingMode;
    vs.writeVpeBasicControl(vpeReg);
    ASSERT(vs.readVpeBasicControl().headingMode == config.headingMode, "Heading Mode was not changed");

    vn::sensors::DeltaThetaAndDeltaVelocityConfigurationRegister dtdvConfReg(config.delThetaDelVeloIntegrationFrame,
                                                                             config.delThetaDelVeloGyroCompensation,
                                                                             config.delThetaDelVeloAccelCompensation);
    vs.writeDeltaThetaAndDeltaVelocityConfiguration(dtdvConfReg);
    ASSERT(vs.readDeltaThetaAndDeltaVelocityConfiguration().integrationFrame == config.delThetaDelVeloIntegrationFrame, "Integration Frame was not changed");
    ASSERT(vs.readDeltaThetaAndDeltaVelocityConfiguration().gyroCompensation == config.delThetaDelVeloGyroCompensation, "Gyro Compensation was not changed");
    ASSERT(vs.readDeltaThetaAndDeltaVelocityConfiguration().accelCompensation == config.delThetaDelVeloAccelCompensation, "Acceleration Compensation was not changed");

    // Stop the AsciiAsync messages
    vs.writeAsyncDataOutputType(vn::protocol::uart::AsciiAsync::VNOFF);

    // Configure Binary Output 1
    vn::sensors::BinaryOutputRegister bor(config.asyncMode,
                                          dividerFrequency.first.at(static_cast<size_t>(selectedFrequency)),
                                          config.commonField,
                                          config.timeField,
                                          config.imuField,
                                          vn::protocol::uart::GpsGroup::GPSGROUP_NONE,
                                          config.attitudeField,
                                          vn::protocol::uart::InsGroup::INSGROUP_NONE,
                                          vn::protocol::uart::GpsGroup::GPSGROUP_NONE);
    try
    {
        vs.writeBinaryOutput1(bor);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("{} could not configure binary output register ({})", nameId(), e.what());
        deinitializeNode();
        return false;
    }

    vs.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    LOG_DEBUG("{} successfully initialized", nameId());

    return true;
}

void NAV::VectorNavSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized())
    {
        return;
    }

    if (vs.isConnected())
    {
        try
        {
            vs.unregisterAsyncPacketReceivedHandler();
        }
        catch (...)
        {}
        try
        {
            vs.reset(true);
        }
        catch (...)
        {}
        try
        {
            vs.disconnect();
        }
        catch (...)
        {}
    }
}

void NAV::VectorNavSensor::asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, [[maybe_unused]] size_t index)
{
    auto* vnSensor = static_cast<VectorNavSensor*>(userData);

    if (p.type() == vn::protocol::uart::Packet::TYPE_BINARY)
    {
        // Make sure that the binary packet is from the type we expect
        if (p.isCompatible(vnSensor->config.commonField,
                           vnSensor->config.timeField,
                           vnSensor->config.imuField,
                           vn::protocol::uart::GpsGroup::GPSGROUP_NONE,
                           vnSensor->config.attitudeField,
                           vn::protocol::uart::InsGroup::INSGROUP_NONE,
                           vn::protocol::uart::GpsGroup::GPSGROUP_NONE))
        {
            auto obs = std::make_shared<VectorNavObs>(vnSensor->imuPos);

            // Group 1 (Common)
            obs->timeSinceStartup.emplace(p.extractUint64());
            obs->timeSinceSyncIn.emplace(p.extractUint64());
            obs->dtime.emplace(p.extractFloat());
            auto dtheta = p.extractVec3f();
            obs->dtheta.emplace(dtheta.x, dtheta.y, dtheta.z);
            auto dvel = p.extractVec3f();
            obs->dvel.emplace(dvel.x, dvel.y, dvel.z);
            obs->syncInCnt.emplace(p.extractUint32());
            // Group 2 (Time)
            // Group 3 (IMU)
            auto magUncompXYZ = p.extractVec3f();
            obs->magUncompXYZ.emplace(magUncompXYZ.x, magUncompXYZ.y, magUncompXYZ.z);
            auto accelUncompXYZ = p.extractVec3f();
            obs->accelUncompXYZ.emplace(accelUncompXYZ.x, accelUncompXYZ.y, accelUncompXYZ.z);
            auto gyroUncompXYZ = p.extractVec3f();
            obs->gyroUncompXYZ.emplace(gyroUncompXYZ.x, gyroUncompXYZ.y, gyroUncompXYZ.z);
            obs->temperature.emplace(p.extractFloat());
            obs->pressure.emplace(p.extractFloat());
            auto magCompXYZ = p.extractVec3f();
            obs->magCompXYZ.emplace(magCompXYZ.x, magCompXYZ.y, magCompXYZ.z);
            auto accelCompXYZ = p.extractVec3f();
            obs->accelCompXYZ.emplace(accelCompXYZ.x, accelCompXYZ.y, accelCompXYZ.z);
            auto gyroCompXYZ = p.extractVec3f();
            obs->gyroCompXYZ.emplace(gyroCompXYZ.x, gyroCompXYZ.y, gyroCompXYZ.z);
            // Group 4 (GPS)
            // Group 5 (Attitude)
            obs->vpeStatus.emplace(p.extractUint16());
            auto yawPitchRoll = p.extractVec3f();
            obs->yawPitchRoll.emplace(yawPitchRoll.x, yawPitchRoll.y, yawPitchRoll.z);
            auto quaternion = p.extractVec4f();
            obs->quaternion.emplace(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
            auto magCompNED = p.extractVec3f();
            obs->magCompNED.emplace(magCompNED.x, magCompNED.y, magCompNED.z);
            auto accelCompNED = p.extractVec3f();
            obs->accelCompNED.emplace(accelCompNED.x, accelCompNED.y, accelCompNED.z);
            auto linearAccelXYZ = p.extractVec3f();
            obs->linearAccelXYZ.emplace(linearAccelXYZ.x, linearAccelXYZ.y, linearAccelXYZ.z);
            auto linearAccelNED = p.extractVec3f();
            obs->linearAccelNED.emplace(linearAccelNED.x, linearAccelNED.y, linearAccelNED.z);
            auto yawPitchRollUncertainty = p.extractVec3f();
            obs->yawPitchRollUncertainty.emplace(yawPitchRollUncertainty.x, yawPitchRollUncertainty.y, yawPitchRollUncertainty.z);

            LOG_DATA("DATA({}): {}, {}, {}, {}, {}",
                     vnSensor->nameId(), obs->timeSinceStartup.value(), obs->syncInCnt.value(), obs->timeSinceSyncIn.value(),
                     obs->vpeStatus.value().status, obs->temperature.value());

            // Calls all the callbacks
            if (InsTime currentTime = util::time::GetCurrentTime();
                !currentTime.empty())
            {
                obs->insTime = currentTime;
            }
            vnSensor->invokeCallbacks(VectorNavSensor::OutputPortIndex_VectorNavObs, obs);
        }
        else if (p.type() == vn::protocol::uart::Packet::TYPE_ASCII)
        {
            LOG_WARN("{} received an ASCII Async message: {}", vnSensor->nameId(), p.datastr());
        }
    }
}