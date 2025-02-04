// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SkydelSatData.hpp
/// @brief Skydel raw logging file description
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-11

#pragma once

#include <cstddef>
#include <cstdint>
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "util/StringUtil.hpp"

#include <catch2/catch_test_macros.hpp>

namespace NAV::TESTS
{
/// @brief Skydel raw logging file description
enum SkydelSatData : uint8_t
{
    SkydelSatData_Elapsed_Time,                   ///< Elapsed Time (ms)                                       | The elapsed time of the simulation in milliseconds.
    SkydelSatData_ECEF_X,                         ///< ECEF X (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    SkydelSatData_ECEF_Y,                         ///< ECEF Y (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    SkydelSatData_ECEF_Z,                         ///< ECEF Z (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    SkydelSatData_ECEF_Error_X,                   ///< ECEF Error X (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    SkydelSatData_ECEF_Error_Y,                   ///< ECEF Error Y (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    SkydelSatData_ECEF_Error_Z,                   ///< ECEF Error Z (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    SkydelSatData_Body_Azimuth,                   ///< Body Azimuth (rad)                                      | Satellite’s azimuth, in radians, from the receiver’s body position relative to North.
    SkydelSatData_Body_Elevation,                 ///< Body Elevation (rad)                                    | Satellite elevation, in radians, from the receiver’s body position relative to the horizon.
    SkydelSatData_Range,                          ///< Range (m)                                               | Geometrical distance, in meters, between the satellite’s and receiver’s antennas.
    SkydelSatData_PSR,                            ///< PSR (m)                                                 | Pseudorange, in meters, between the satellite’s and receiver’s antennas.
    SkydelSatData_ADR,                            ///< ADR                                                     | Accumulated Doppler range, in number of cycles, between the satellite and receiver.
    SkydelSatData_Clock_Correction,               ///< Clock Correction (s)                                    | Satellite’s clock correction, in seconds.
    SkydelSatData_Clock_Noise,                    ///< Clock Noise (m)                                         | Additional clock error, in meters, not accounted for in navigation message.
    SkydelSatData_Delta_Af0,                      ///< Delta Af0 (s)                                           | Clock offset in seconds.
    SkydelSatData_Delta_Af1,                      ///< Delta Af1 (s/s)                                         | Clock drift in seconds per seconds.
    SkydelSatData_Iono_Correction,                ///< Iono Correction (m)                                     | Ionospheric corrections, in meters.
    SkydelSatData_Tropo_Correction,               ///< Tropo Correction (m)                                    | Tropospheric corrections, in meters.
    SkydelSatData_PSR_Offset,                     ///< PSR Offset (m)                                          | Pseudorange offset, in meters.
    SkydelSatData_Receiver_Antenna_Azimuth,       ///< Receiver Antenna Azimuth (rad)                          | Satellite’s azimuth, in radians, from the receiver’s antenna position.
    SkydelSatData_Receiver_Antenna_Elevation,     ///< Receiver Antenna Elevation (rad)                        | Satellite’s elevation, in radians, from the receiver’s antenna position.
    SkydelSatData_Receiver_Antenna_Gain,          ///< Receiver Antenna Gain (dBi)                             | Receiver’s antenna gain, in dBi.
    SkydelSatData_SV_Antenna_Azimuth,             ///< SV Antenna Azimuth (rad)                                | Receiver’s azimuth, in radians, from the satellite’s antenna position.
    SkydelSatData_SV_Antenna_Elevation,           ///< SV Antenna Elevation (rad)                              | Receiver’s elevation, in radians, from the satellite’s antenna position.
    SkydelSatData_Relative_Power_Level,           ///< Relative Power Level (incl. Receiver Antenna gain) (dB) | Signal’s relative power level, in dB, corresponding to the sum of the global power offset, the user’s power offset, the receiver’s antenna gain and the satellite’s antenna gain.
    SkydelSatData_Doppler_Frequency,              ///< Doppler Frequency (Hz)                                  | Doppler frequency, in Hertz, due to satellites' and receivers' antennas dynamics'.
    SkydelSatData_PSR_Change_Rate,                ///< PSR Change Rate (m/s)                                   | Pseudorange rate, in meters per second, due to satellites' and receivers' antennas dynamics'.
    SkydelSatData_Receiver_Carrier_Phase_Offset,  ///< Receiver Carrier Phase Offset (rad)                     | Phase offset, in radians, caused by the receiver’s antenna phase pattern. This column does not appear in echo log files.
    SkydelSatData_Satellite_Carrier_Phase_Offset, ///< Satellite Carrier Phase Offset (rad)                    | Phase offset, in radians, caused by the satellite’s antenna phase pattern. This column does not appear in echo log files.
    SkydelSatData_GPS_TOW,                        ///< GPS TOW                                                 | GPS time of week, in seconds.
    SkydelSatData_GPS_Week_Number,                ///< GPS Week Number                                         | GPS week number.
    SkydelSatData_SBAS_t0,                        ///< SBAS t0                                                 | SBAS time of the day, in seconds.
    SkydelSatData_Calibration_offset,             ///< Calibration offset (m)                                  | Offset, in meters, applied to the signal during Wavefront simulation. Used to compensate for hardware differences between elements.
    SkydelSatData_PSR_satellite_time,             ///< PSR satellite time (ms)                                 | The elapsed time of the simulation when the signal was emitted from the satellite, in milliseconds.
    SkydelSatData_COUNT                           ///< Count variable
};

struct SkydelRawData
{
    /// @brief Reads the data from a line string
    /// @param line Sring with comma separated data
    explicit SkydelRawData(const std::string& line)
    {
        std::vector<std::string> v = str::split(line, ",");
        REQUIRE(v.size() == SkydelSatData_COUNT);

        recvTime = InsTime(0, std::stoi(v[SkydelSatData_GPS_Week_Number]), std::stold(v[SkydelSatData_GPS_TOW]), GPST);

        Elapsed_Time = std::stod(v.at(SkydelSatData_Elapsed_Time));
        ECEF_X = std::stod(v.at(SkydelSatData_ECEF_X));
        ECEF_Y = std::stod(v.at(SkydelSatData_ECEF_Y));
        ECEF_Z = std::stod(v.at(SkydelSatData_ECEF_Z));
        ECEF_Error_X = std::stod(v.at(SkydelSatData_ECEF_Error_X));
        ECEF_Error_Y = std::stod(v.at(SkydelSatData_ECEF_Error_Y));
        ECEF_Error_Z = std::stod(v.at(SkydelSatData_ECEF_Error_Z));
        Body_Azimuth = std::stod(v.at(SkydelSatData_Body_Azimuth));
        Body_Elevation = std::stod(v.at(SkydelSatData_Body_Elevation));
        Range = std::stod(v.at(SkydelSatData_Range));
        PSR = std::stod(v.at(SkydelSatData_PSR));
        ADR = std::stod(v.at(SkydelSatData_ADR));
        Clock_Correction = std::stod(v.at(SkydelSatData_Clock_Correction));
        Clock_Noise = std::stod(v.at(SkydelSatData_Clock_Noise));
        Delta_Af0 = std::stod(v.at(SkydelSatData_Delta_Af0));
        Delta_Af1 = std::stod(v.at(SkydelSatData_Delta_Af1));
        Iono_Correction = std::stod(v.at(SkydelSatData_Iono_Correction));
        Tropo_Correction = std::stod(v.at(SkydelSatData_Tropo_Correction));
        PSR_Offset = std::stod(v.at(SkydelSatData_PSR_Offset));
        Receiver_Antenna_Azimuth = std::stod(v.at(SkydelSatData_Receiver_Antenna_Azimuth));
        Receiver_Antenna_Elevation = std::stod(v.at(SkydelSatData_Receiver_Antenna_Elevation));
        Receiver_Antenna_Gain = std::stod(v.at(SkydelSatData_Receiver_Antenna_Gain));
        SV_Antenna_Azimuth = std::stod(v.at(SkydelSatData_SV_Antenna_Azimuth));
        SV_Antenna_Elevation = std::stod(v.at(SkydelSatData_SV_Antenna_Elevation));
        Relative_Power_Level = std::stod(v.at(SkydelSatData_Relative_Power_Level));
        Doppler_Frequency = std::stod(v.at(SkydelSatData_Doppler_Frequency));
        PSR_Change_Rate = std::stod(v.at(SkydelSatData_PSR_Change_Rate));
        Receiver_Carrier_Phase_Offset = std::stod(v.at(SkydelSatData_Receiver_Carrier_Phase_Offset));
        Satellite_Carrier_Phase_Offset = std::stod(v.at(SkydelSatData_Satellite_Carrier_Phase_Offset));
        GPS_TOW = std::stod(v.at(SkydelSatData_GPS_TOW));
        GPS_Week_Number = std::stod(v.at(SkydelSatData_GPS_Week_Number));
        SBAS_t0 = std::stod(v.at(SkydelSatData_SBAS_t0));
        Calibration_offset = std::stod(v.at(SkydelSatData_Calibration_offset));
        PSR_satellite_time = std::stod(v.at(SkydelSatData_PSR_satellite_time));
    }

    InsTime recvTime;
    double Elapsed_Time = 0.0;                   ///< Elapsed Time (ms)                                       | The elapsed time of the simulation in milliseconds.
    double ECEF_X = 0.0;                         ///< ECEF X (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    double ECEF_Y = 0.0;                         ///< ECEF Y (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    double ECEF_Z = 0.0;                         ///< ECEF Z (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
    double ECEF_Error_X = 0.0;                   ///< ECEF Error X (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    double ECEF_Error_Y = 0.0;                   ///< ECEF Error Y (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    double ECEF_Error_Z = 0.0;                   ///< ECEF Error Z (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
    double Body_Azimuth = 0.0;                   ///< Body Azimuth (rad)                                      | Satellite’s azimuth, in radians, from the receiver’s body position relative to North.
    double Body_Elevation = 0.0;                 ///< Body Elevation (rad)                                    | Satellite elevation, in radians, from the receiver’s body position relative to the horizon.
    double Range = 0.0;                          ///< Range (m)                                               | Geometrical distance, in meters, between the satellite’s and receiver’s antennas.
    double PSR = 0.0;                            ///< PSR (m)                                                 | Pseudorange, in meters, between the satellite’s and receiver’s antennas.
    double ADR = 0.0;                            ///< ADR                                                     | Accumulated Doppler range, in number of cycles, between the satellite and receiver.
    double Clock_Correction = 0.0;               ///< Clock Correction (s)                                    | Satellite’s clock correction, in seconds.
    double Clock_Noise = 0.0;                    ///< Clock Noise (m)                                         | Additional clock error, in meters, not accounted for in navigation message.
    double Delta_Af0 = 0.0;                      ///< Delta Af0 (s)                                           | Clock offset in seconds.
    double Delta_Af1 = 0.0;                      ///< Delta Af1 (s/s)                                         | Clock drift in seconds per seconds.
    double Iono_Correction = 0.0;                ///< Iono Correction (m)                                     | Ionospheric corrections, in meters.
    double Tropo_Correction = 0.0;               ///< Tropo Correction (m)                                    | Tropospheric corrections, in meters.
    double PSR_Offset = 0.0;                     ///< PSR Offset (m)                                          | Pseudorange offset, in meters.
    double Receiver_Antenna_Azimuth = 0.0;       ///< Receiver Antenna Azimuth (rad)                          | Satellite’s azimuth, in radians, from the receiver’s antenna position.
    double Receiver_Antenna_Elevation = 0.0;     ///< Receiver Antenna Elevation (rad)                        | Satellite’s elevation, in radians, from the receiver’s antenna position.
    double Receiver_Antenna_Gain = 0.0;          ///< Receiver Antenna Gain (dBi)                             | Receiver’s antenna gain, in dBi.
    double SV_Antenna_Azimuth = 0.0;             ///< SV Antenna Azimuth (rad)                                | Receiver’s azimuth, in radians, from the satellite’s antenna position.
    double SV_Antenna_Elevation = 0.0;           ///< SV Antenna Elevation (rad)                              | Receiver’s elevation, in radians, from the satellite’s antenna position.
    double Relative_Power_Level = 0.0;           ///< Relative Power Level (incl. Receiver Antenna gain) (dB) | Signal’s relative power level, in dB, corresponding to the sum of the global power offset, the user’s power offset, the receiver’s antenna gain and the satellite’s antenna gain.
    double Doppler_Frequency = 0.0;              ///< Doppler Frequency (Hz)                                  | Doppler frequency, in Hertz, due to satellites' and receivers' antennas dynamics'.
    double PSR_Change_Rate = 0.0;                ///< PSR Change Rate (m/s)                                   | Pseudorange rate, in meters per second, due to satellites' and receivers' antennas dynamics'.
    double Receiver_Carrier_Phase_Offset = 0.0;  ///< Receiver Carrier Phase Offset (rad)                     | Phase offset, in radians, caused by the receiver’s antenna phase pattern. This column does not appear in echo log files.
    double Satellite_Carrier_Phase_Offset = 0.0; ///< Satellite Carrier Phase Offset (rad)                    | Phase offset, in radians, caused by the satellite’s antenna phase pattern. This column does not appear in echo log files.
    double GPS_TOW = 0.0;                        ///< GPS TOW                                                 | GPS time of week, in seconds.
    double GPS_Week_Number = 0.0;                ///< GPS Week Number                                         | GPS week number.
    double SBAS_t0 = 0.0;                        ///< SBAS t0                                                 | SBAS time of the day, in seconds.
    double Calibration_offset = 0.0;             ///< Calibration offset (m)                                  | Offset, in meters, applied to the signal during Wavefront simulation. Used to compensate for hardware differences between elements.
    double PSR_satellite_time = 0.0;             ///< PSR satellite time (ms)                                 | The elapsed time of the simulation when the signal was emitted from the satellite, in milliseconds.

  private:
};

/// @brief SkydelReference for SPP calcualtion
struct SkydelReference
{
    struct Margin
    {
        double clock = 0.0;
        double pos = 0.0;
        double satElevation = 0.0;
        double satAzimuth = 0.0;
        double dpsr_I = 0.0;
        double dpsr_T = 0.0;
        double timeDiffRecvTrans = 0.0;
        double geometricDist = 0.0;
    };

    /// @brief Constructor
    /// @param[in] satSigId Signal identifier
    /// @param[in] path Path to the reference file
    SkydelReference(SatSigId satSigId, const std::string& path)
        : satSigId(satSigId)
    {
        std::ifstream fs(path, std::ios_base::binary);
        REQUIRE(fs.good());
        std::string line;
        std::getline(fs, line); // Header line

        while (!fs.eof() && std::getline(fs, line) && !line.empty())
        {
            refData.emplace_back(line);
        }
    }

    SatSigId satSigId;                  ///< GNSS frequency and satellite number
    std::vector<SkydelRawData> refData; ///< Reference data read from the file
    size_t counter = 0;                 ///< Counter to see how many lines were read
};

} // namespace NAV::TESTS