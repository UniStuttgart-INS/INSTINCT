// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SpirentAsciiSatelliteData.hpp
/// @brief Spirent ASCII satellite data parameters file description
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-11

#pragma once

#include <cstddef>
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

namespace NAV::TESTS
{

/// @brief Spirent ASCII satellite data parameters file description
enum SpirentAsciiSatelliteData_ : size_t
{
    SpirentAsciiSatelliteData_Time_ms,               ///< Time_ms
    SpirentAsciiSatelliteData_Channel,               ///< Channel
    SpirentAsciiSatelliteData_Sat_type,              ///< Sat_type
    SpirentAsciiSatelliteData_Sat_ID,                ///< Sat_ID
    SpirentAsciiSatelliteData_Sat_PRN,               ///< Sat_PRN
    SpirentAsciiSatelliteData_Echo_Num,              ///< Echo_Num
    SpirentAsciiSatelliteData_Sat_Pos_X,             ///< Sat_Pos_X
    SpirentAsciiSatelliteData_Sat_Pos_Y,             ///< Sat_Pos_Y
    SpirentAsciiSatelliteData_Sat_Pos_Z,             ///< Sat_Pos_Z
    SpirentAsciiSatelliteData_Sat_Vel_X,             ///< Sat_Vel_X
    SpirentAsciiSatelliteData_Sat_Vel_Y,             ///< Sat_Vel_Y
    SpirentAsciiSatelliteData_Sat_Vel_Z,             ///< Sat_Vel_Z
    SpirentAsciiSatelliteData_Azimuth,               ///< Azimuth
    SpirentAsciiSatelliteData_Elevation,             ///< Elevation
    SpirentAsciiSatelliteData_Range,                 ///< Range
    SpirentAsciiSatelliteData_P_Range_Group_A,       ///< P-Range Group A
    SpirentAsciiSatelliteData_P_Range_Group_B,       ///< P-Range Group B
    SpirentAsciiSatelliteData_P_Range_Group_C,       ///< P-Range Group C
    SpirentAsciiSatelliteData_P_Range_Group_D,       ///< P-Range Group D
    SpirentAsciiSatelliteData_P_Range_Group_E,       ///< P-Range Group E
    SpirentAsciiSatelliteData_P_Range_Group_F,       ///< P-Range Group F
    SpirentAsciiSatelliteData_P_R_rate,              ///< P-R_rate
    SpirentAsciiSatelliteData_Iono_delay_Group_A,    ///< Iono_delay Group A
    SpirentAsciiSatelliteData_Iono_delay_Group_B,    ///< Iono_delay Group B
    SpirentAsciiSatelliteData_Iono_delay_Group_C,    ///< Iono_delay Group C
    SpirentAsciiSatelliteData_Iono_delay_Group_D,    ///< Iono_delay Group D
    SpirentAsciiSatelliteData_Iono_delay_Group_E,    ///< Iono_delay Group E
    SpirentAsciiSatelliteData_Iono_delay_Group_F,    ///< Iono_delay Group F
    SpirentAsciiSatelliteData_Tropo_delay,           ///< Tropo_delay
    SpirentAsciiSatelliteData_P_R_Error,             ///< P-R_Error
    SpirentAsciiSatelliteData_Signal_dB_Group_A,     ///< Signal_dB Group A
    SpirentAsciiSatelliteData_Signal_dB_Group_B,     ///< Signal_dB Group B
    SpirentAsciiSatelliteData_Signal_dB_Group_C,     ///< Signal_dB Group C
    SpirentAsciiSatelliteData_Signal_dB_Group_D,     ///< Signal_dB Group D
    SpirentAsciiSatelliteData_Signal_dB_Group_E,     ///< Signal_dB Group E
    SpirentAsciiSatelliteData_Signal_dB_Group_F,     ///< Signal_dB Group F
    SpirentAsciiSatelliteData_Ant_azimuth,           ///< Ant_azimuth
    SpirentAsciiSatelliteData_Ant_elevation,         ///< Ant_elevation
    SpirentAsciiSatelliteData_Range_rate,            ///< Range_rate
    SpirentAsciiSatelliteData_P_R_Error_rate,        ///< P-R_Error_rate
    SpirentAsciiSatelliteData_Doppler_shift_Group_A, ///< Doppler_shift Group A
    SpirentAsciiSatelliteData_Doppler_shift_Group_B, ///< Doppler_shift Group B
    SpirentAsciiSatelliteData_Doppler_shift_Group_C, ///< Doppler_shift Group C
    SpirentAsciiSatelliteData_Doppler_shift_Group_D, ///< Doppler_shift Group D
    SpirentAsciiSatelliteData_Doppler_shift_Group_E, ///< Doppler_shift Group E
    SpirentAsciiSatelliteData_Doppler_shift_Group_F, ///< Doppler_shift Group F
    SpirentAsciiSatelliteData_Delta_range_Group_A,   ///< Delta_range Group A
    SpirentAsciiSatelliteData_Delta_range_Group_B,   ///< Delta_range Group B
    SpirentAsciiSatelliteData_Delta_range_Group_C,   ///< Delta_range Group C
    SpirentAsciiSatelliteData_Delta_range_Group_D,   ///< Delta_range Group D
    SpirentAsciiSatelliteData_Delta_range_Group_E,   ///< Delta_range Group E
    SpirentAsciiSatelliteData_Delta_range_Group_F,   ///< Delta_range Group F
    SpirentAsciiSatelliteData_Sat_Acc_X,             ///< Sat_Acc_X
    SpirentAsciiSatelliteData_Sat_Acc_Y,             ///< Sat_Acc_Y
    SpirentAsciiSatelliteData_Sat_Acc_Z,             ///< Sat_Acc_Z
    SpirentAsciiSatelliteData_COUNT                  ///< Count variable
};

/// Storage class for Spirent Satellite Data
struct SpirentAsciiSatelliteData
{
    InsTime recvTime; ///< Receive Time
    SatId satId;      ///< Satellite Id

    size_t Time_ms;               ///< Time_ms [ms]
    size_t Channel;               ///< Channel
    std::string Sat_type;         ///< Sat_type
    size_t Sat_ID;                ///< Sat_ID
    size_t Sat_PRN;               ///< Sat_PRN
    size_t Echo_Num;              ///< Echo_Num
    Eigen::Vector3d Sat_Pos;      ///< Sat_Pos_X, Sat_Pos_Y, Sat_Pos_Z [m]
    Eigen::Vector3d Sat_Vel;      ///< Sat_Vel_X, Sat_Vel_Y, Sat_Vel_Z [m/s]
    double Azimuth;               ///< Azimuth [rad]
    double Elevation;             ///< Elevation [rad]
    double Range;                 ///< Range [m]
    double P_Range_Group_A;       ///< P-Range Group A [m]
    double P_Range_Group_B;       ///< P-Range Group B [m]
    double P_Range_Group_C;       ///< P-Range Group C [m]
    double P_Range_Group_D;       ///< P-Range Group D [m]
    double P_Range_Group_E;       ///< P-Range Group E [m]
    double P_Range_Group_F;       ///< P-Range Group F [m]
    double P_R_rate;              ///< P-R_rate [m/s]
    double Iono_delay_Group_A;    ///< Iono_delay Group A [s]
    double Iono_delay_Group_B;    ///< Iono_delay Group B [s]
    double Iono_delay_Group_C;    ///< Iono_delay Group C [s]
    double Iono_delay_Group_D;    ///< Iono_delay Group D [s]
    double Iono_delay_Group_E;    ///< Iono_delay Group E [s]
    double Iono_delay_Group_F;    ///< Iono_delay Group F [s]
    double Tropo_delay;           ///< Tropo_delay [s]
    double P_R_Error;             ///< P-R_Error
    double Signal_dB_Group_A;     ///< Signal_dB Group A [dB]
    double Signal_dB_Group_B;     ///< Signal_dB Group B [dB]
    double Signal_dB_Group_C;     ///< Signal_dB Group C [dB]
    double Signal_dB_Group_D;     ///< Signal_dB Group D [dB]
    double Signal_dB_Group_E;     ///< Signal_dB Group E [dB]
    double Signal_dB_Group_F;     ///< Signal_dB Group F [dB]
    double Ant_azimuth;           ///< Ant_azimuth [rad]
    double Ant_elevation;         ///< Ant_elevation [rad]
    double Range_rate;            ///< Range_rate [m/s]
    double P_R_Error_rate;        ///< P-R_Error_rate [m/s]
    double Doppler_shift_Group_A; ///< Doppler_shift Group A [Hz]
    double Doppler_shift_Group_B; ///< Doppler_shift Group B [Hz]
    double Doppler_shift_Group_C; ///< Doppler_shift Group C [Hz]
    double Doppler_shift_Group_D; ///< Doppler_shift Group D [Hz]
    double Doppler_shift_Group_E; ///< Doppler_shift Group E [Hz]
    double Doppler_shift_Group_F; ///< Doppler_shift Group F [Hz]
    double Delta_range_Group_A;   ///< Delta_range Group A [m/s]
    double Delta_range_Group_B;   ///< Delta_range Group B [m/s]
    double Delta_range_Group_C;   ///< Delta_range Group C [m/s]
    double Delta_range_Group_D;   ///< Delta_range Group D [m/s]
    double Delta_range_Group_E;   ///< Delta_range Group E [m/s]
    double Delta_range_Group_F;   ///< Delta_range Group F [m/s]
    double Sat_Acc_X;             ///< Sat_Acc_X [m/s^2]
    double Sat_Acc_Y;             ///< Sat_Acc_Y [m/s^2]
    double Sat_Acc_Z;             ///< Sat_Acc_Z [m/s^2]
};

} // namespace NAV::TESTS