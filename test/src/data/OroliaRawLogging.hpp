// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file OroliaRawLogging.hpp
/// @brief Orolia raw logging file description
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-11

#pragma once

#include <cstddef>

namespace NAV::TESTS
{

/// @brief Orolia raw logging file description
enum OroliaRawLogging : size_t
{
    OroliaRawLogging_Elapsed_Time,                   ///< Elapsed Time (ms)                                       | The elapsed time of the simulation in milliseconds.
    OroliaRawLogging_ECEF_X,                         ///< ECEF X (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    OroliaRawLogging_ECEF_Y,                         ///< ECEF Y (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    OroliaRawLogging_ECEF_Z,                         ///< ECEF Z (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    OroliaRawLogging_ECEF_Error_X,                   ///< ECEF Error X (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    OroliaRawLogging_ECEF_Error_Y,                   ///< ECEF Error Y (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    OroliaRawLogging_ECEF_Error_Z,                   ///< ECEF Error Z (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    OroliaRawLogging_Body_Azimuth,                   ///< Body Azimuth (rad)                                      | Satellite’s azimuth, in radians, from the receiver’s body position relative to North.
    OroliaRawLogging_Body_Elevation,                 ///< Body Elevation (rad)                                    | Satellite elevation, in radians, from the receiver’s body position relative to the horizon.
    OroliaRawLogging_Range,                          ///< Range (m)                                               | Geometrical distance, in meters, between the satellite’s and receiver’s antennas.
    OroliaRawLogging_PSR,                            ///< PSR (m)                                                 | Pseudorange, in meters, between the satellite’s and receiver’s antennas.
    OroliaRawLogging_ADR,                            ///< ADR                                                     | Accumulated Doppler range, in number of cycles, between the satellite and receiver.
    OroliaRawLogging_Clock_Correction,               ///< Clock Correction (s)                                    | Satellite’s clock correction, in seconds.
    OroliaRawLogging_Clock_Noise,                    ///< Clock Noise (m)                                         | Additional clock error, in meters, not accounted for in navigation message.
    OroliaRawLogging_Delta_Af0,                      ///< Delta Af0 (s)                                           | Clock offset in seconds.
    OroliaRawLogging_Delta_Af1,                      ///< Delta Af1 (s/s)                                         | Clock drift in seconds per seconds.
    OroliaRawLogging_Iono_Correction,                ///< Iono Correction (m)                                     | Ionospheric corrections, in meters.
    OroliaRawLogging_Tropo_Correction,               ///< Tropo Correction (m)                                    | Tropospheric corrections, in meters.
    OroliaRawLogging_PSR_Offset,                     ///< PSR Offset (m)                                          | Pseudorange offset, in meters.
    OroliaRawLogging_Receiver_Antenna_Azimuth,       ///< Receiver Antenna Azimuth (rad)                          | Satellite’s azimuth, in radians, from the receiver’s antenna position.
    OroliaRawLogging_Receiver_Antenna_Elevation,     ///< Receiver Antenna Elevation (rad)                        | Satellite’s elevation, in radians, from the receiver’s antenna position.
    OroliaRawLogging_Receiver_Antenna_Gain,          ///< Receiver Antenna Gain (dBi)                             | Receiver’s antenna gain, in dBi.
    OroliaRawLogging_SV_Antenna_Azimuth,             ///< SV Antenna Azimuth (rad)                                | Receiver’s azimuth, in radians, from the satellite’s antenna position.
    OroliaRawLogging_SV_Antenna_Elevation,           ///< SV Antenna Elevation (rad)                              | Receiver’s elevation, in radians, from the satellite’s antenna position.
    OroliaRawLogging_Relative_Power_Level,           ///< Relative Power Level (incl. Receiver Antenna gain) (dB) | Signal’s relative power level, in dB, corresponding to the sum of the global power offset, the user’s power offset, the receiver’s antenna gain and the satellite’s antenna gain.
    OroliaRawLogging_Doppler_Frequency,              ///< Doppler Frequency (Hz)                                  | Doppler frequency, in Hertz, due to satellites' and receivers' antennas dynamics'.
    OroliaRawLogging_PSR_Change_Rate,                ///< PSR Change Rate (m/s)                                   | Pseudorange rate, in meters per second, due to satellites' and receivers' antennas dynamics'.
    OroliaRawLogging_Receiver_Carrier_Phase_Offset,  ///< Receiver Carrier Phase Offset (rad)                     | Phase offset, in radians, caused by the receiver’s antenna phase pattern. This column does not appear in echo log files.
    OroliaRawLogging_Satellite_Carrier_Phase_Offset, ///< Satellite Carrier Phase Offset (rad)                    | Phase offset, in radians, caused by the satellite’s antenna phase pattern. This column does not appear in echo log files.
    OroliaRawLogging_GPS_TOW,                        ///< GPS TOW                                                 | GPS time of week, in seconds.
    OroliaRawLogging_GPS_Week_Number,                ///< GPS Week Number                                         | GPS week number.
    OroliaRawLogging_SBAS_t0,                        ///< SBAS t0                                                 | SBAS time of the day, in seconds.
    OroliaRawLogging_Calibration_offset,             ///< Calibration offset (m)                                  | Offset, in meters, applied to the signal during Wavefront simulation. Used to compensate for hardware differences between elements.
    OroliaRawLogging_PSR_satellite_time,             ///< PSR satellite time (ms)                                 | The elapsed time of the simulation when the signal was emitted from the satellite, in milliseconds.
    OroliaRawLogging_COUNT                           ///< Count variable
};

} // namespace NAV::TESTS