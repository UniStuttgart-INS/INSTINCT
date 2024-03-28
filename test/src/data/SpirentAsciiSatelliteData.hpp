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
#include <optional>
#include <functional>
#include <vector>
#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

#include <catch2/catch_test_macros.hpp>

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
    explicit SpirentAsciiSatelliteData(const std::string& line)
    {
        std::vector<std::string> v = str::split(line, ",");
        REQUIRE(v.size() == size_t(SpirentAsciiSatelliteData_COUNT));

        SatelliteSystem satSys;
        auto satNum = static_cast<uint16_t>(std::stoul(v[SpirentAsciiSatelliteData_Sat_ID]));
        if (v[SpirentAsciiSatelliteData_Sat_type] == "GPS") { satSys = GPS; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "GALILEO") { satSys = GAL; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "GLONASS") { satSys = GLO; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "BeiDou") { satSys = BDS; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "Quasi-Zenith") { satSys = QZSS; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "IRNSS") { satSys = IRNSS; }
        REQUIRE(satSys != SatSys_None);

        recvTime = InsTime{ 2023, 1, 8, 10, 0, std::stod(v[SpirentAsciiSatelliteData_Time_ms]) * 1e-3 - 60.0, GPST };
        LOG_DEBUG("  [{} GPST][{}] Found in Spirent file", recvTime.toYMDHMS(GPST), SatId{ satSys, satNum });

        // The Spirent reference frame is rotated by the signal transmit time
        auto rotateDataFrame = [&v](const Eigen::Vector3d& e_satPos) -> Eigen::Vector3d {
            auto dt = std::stod(v[SpirentAsciiSatelliteData_Range]) / InsConst<>::C;

            // see \cite SpringerHandbookGNSS2017 Springer Handbook GNSS ch. 21.2, eq. 21.18, p. 610
            return Eigen::AngleAxisd(InsConst<>::omega_ie * dt, Eigen::Vector3d::UnitZ()) * e_satPos;
        };

        satId = SatId{ satSys, satNum };
        Time_ms = std::stoul(v[SpirentAsciiSatelliteData_Time_ms]);
        Channel = std::stoul(v[SpirentAsciiSatelliteData_Channel]);
        Sat_type = v[SpirentAsciiSatelliteData_Sat_type];
        Sat_ID = std::stoul(v[SpirentAsciiSatelliteData_Sat_ID]);
        Sat_PRN = std::stoul(v[SpirentAsciiSatelliteData_Sat_PRN]);
        Echo_Num = std::stoul(v[SpirentAsciiSatelliteData_Echo_Num]);
        Sat_Pos = rotateDataFrame({ std::stod(v[SpirentAsciiSatelliteData_Sat_Pos_X]),
                                    std::stod(v[SpirentAsciiSatelliteData_Sat_Pos_Y]),
                                    std::stod(v[SpirentAsciiSatelliteData_Sat_Pos_Z]) });
        Sat_Vel = rotateDataFrame({ std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_X]),
                                    std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Y]),
                                    std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Z]) });
        Azimuth = std::stod(v[SpirentAsciiSatelliteData_Azimuth]);
        Elevation = std::stod(v[SpirentAsciiSatelliteData_Elevation]);
        Range = std::stod(v[SpirentAsciiSatelliteData_Range]);
        P_Range_Group_A = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_A]);
        P_Range_Group_B = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_B]);
        P_Range_Group_C = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_C]);
        P_Range_Group_D = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_D]);
        P_Range_Group_E = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_E]);
        P_Range_Group_F = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_F]);
        P_R_rate = std::stod(v[SpirentAsciiSatelliteData_P_R_rate]);
        Iono_delay_Group_A = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_A]) * InsConst<>::C;
        Iono_delay_Group_B = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_B]) * InsConst<>::C;
        Iono_delay_Group_C = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_C]) * InsConst<>::C;
        Iono_delay_Group_D = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_D]) * InsConst<>::C;
        Iono_delay_Group_E = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_E]) * InsConst<>::C;
        Iono_delay_Group_F = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_F]) * InsConst<>::C;
        Tropo_delay = std::stod(v[SpirentAsciiSatelliteData_Tropo_delay]) * InsConst<>::C;
        P_R_Error = std::stod(v[SpirentAsciiSatelliteData_P_R_Error]);
        Signal_dB_Group_A = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_A]);
        Signal_dB_Group_B = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_B]);
        Signal_dB_Group_C = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_C]);
        Signal_dB_Group_D = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_D]);
        Signal_dB_Group_E = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_E]);
        Signal_dB_Group_F = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_F]);
        Ant_azimuth = std::stod(v[SpirentAsciiSatelliteData_Ant_azimuth]);
        Ant_elevation = std::stod(v[SpirentAsciiSatelliteData_Ant_elevation]);
        Range_rate = std::stod(v[SpirentAsciiSatelliteData_Range_rate]);
        P_R_Error_rate = std::stod(v[SpirentAsciiSatelliteData_P_R_Error_rate]);
        Doppler_shift_Group_A = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_A]);
        Doppler_shift_Group_B = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_B]);
        Doppler_shift_Group_C = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_C]);
        Doppler_shift_Group_D = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_D]);
        Doppler_shift_Group_E = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_E]);
        Doppler_shift_Group_F = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_F]);
        Delta_range_Group_A = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_A]);
        Delta_range_Group_B = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_B]);
        Delta_range_Group_C = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_C]);
        Delta_range_Group_D = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_D]);
        Delta_range_Group_E = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_E]);
        Delta_range_Group_F = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_F]);
        Sat_Acc_X = std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_X]);
        Sat_Acc_Y = std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Y]);
        Sat_Acc_Z = std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Z]);
    }

    static size_t GetGroupIncr(Frequency freq)
    {
        switch (Frequency_(freq)) // TODO: Check sorting of GLONASS
        {
        case G01:
        case E01:
        case R01:
        case J01:
        case I09:
        case S01:
            return 0; // Group A (L1/E1/S/B1I)
        case G02:
        case R02:
        case J02:
            return 1; // Group B (L2/E6/B2I)
        case G05:
        case E05:
        case B05:
        case J05:
        case I05:
        case S05:
            return 2; // Group C (L5/E5/B2A/C1)
        case J06:
            return 3; // Group D (L6/B1C/C2)
        case B06:
            return 4; // Group E (B3I/C3)
        case B07:
            return 5; // Group F (B2b)

        case Freq_None:
        case R03: // TODO: sort these
        case R04:
        case R06:
        case E06:
        case E07:
        case E08:
        case B01:
        case B02:
        case B08:
            break;
        }

        REQUIRE("The frequency is not supported by the Spirent sat_data file" == std::string(""));
        return 100;
    }

    [[nodiscard]] double getP_Range(Frequency freq) const
    {
        switch (GetGroupIncr(freq))
        {
        case 0:
            return P_Range_Group_A;
        case 1:
            return P_Range_Group_B;
        case 2:
            return P_Range_Group_C;
        case 3:
            return P_Range_Group_D;
        case 4:
            return P_Range_Group_E;
        case 5:
            return P_Range_Group_F;
        default:
            break;
        }

        REQUIRE("The frequency is not supported by the Spirent sat_data file" == std::string(""));
        return 0.0;
    }

    [[nodiscard]] double getIono_delay(Frequency freq) const
    {
        switch (GetGroupIncr(freq))
        {
        case 0:
            return Iono_delay_Group_A;
        case 1:
            return Iono_delay_Group_B;
        case 2:
            return Iono_delay_Group_C;
        case 3:
            return Iono_delay_Group_D;
        case 4:
            return Iono_delay_Group_E;
        case 5:
            return Iono_delay_Group_F;
        default:
            break;
        }

        REQUIRE("The frequency is not supported by the Spirent sat_data file" == std::string(""));
        return 0.0;
    }

    [[nodiscard]] double getSignal_dB(Frequency freq) const
    {
        switch (GetGroupIncr(freq))
        {
        case 0:
            return Signal_dB_Group_A;
        case 1:
            return Signal_dB_Group_B;
        case 2:
            return Signal_dB_Group_C;
        case 3:
            return Signal_dB_Group_D;
        case 4:
            return Signal_dB_Group_E;
        case 5:
            return Signal_dB_Group_F;
        default:
            break;
        }

        REQUIRE("The frequency is not supported by the Spirent sat_data file" == std::string(""));
        return 0.0;
    }

    [[nodiscard]] double getDoppler_shift(Frequency freq) const
    {
        switch (GetGroupIncr(freq))
        {
        case 0:
            return Doppler_shift_Group_A;
        case 1:
            return Doppler_shift_Group_B;
        case 2:
            return Doppler_shift_Group_C;
        case 3:
            return Doppler_shift_Group_D;
        case 4:
            return Doppler_shift_Group_E;
        case 5:
            return Doppler_shift_Group_F;
        default:
            break;
        }

        REQUIRE("The frequency is not supported by the Spirent sat_data file" == std::string(""));
        return 0.0;
    }

    [[nodiscard]] double getDelta_range(Frequency freq) const
    {
        switch (GetGroupIncr(freq))
        {
        case 0:
            return Delta_range_Group_A;
        case 1:
            return Delta_range_Group_B;
        case 2:
            return Delta_range_Group_C;
        case 3:
            return Delta_range_Group_D;
        case 4:
            return Delta_range_Group_E;
        case 5:
            return Delta_range_Group_F;
        default:
            break;
        }

        REQUIRE("The frequency is not supported by the Spirent sat_data file" == std::string(""));
        return 0.0;
    }

    InsTime recvTime; ///< Receive Time
    SatId satId;      ///< Satellite Id

    size_t Time_ms;          ///< Time_ms [ms]
    size_t Channel;          ///< Channel
    std::string Sat_type;    ///< Sat_type
    size_t Sat_ID;           ///< Sat_ID
    size_t Sat_PRN;          ///< Sat_PRN
    size_t Echo_Num;         ///< Echo_Num
    Eigen::Vector3d Sat_Pos; ///< Sat_Pos_X, Sat_Pos_Y, Sat_Pos_Z [m]
    Eigen::Vector3d Sat_Vel; ///< Sat_Vel_X, Sat_Vel_Y, Sat_Vel_Z [m/s]
    double Azimuth;          ///< Azimuth [rad]
    double Elevation;        ///< Elevation [rad]
    double Range;            ///< Range [m]
    double P_R_rate;         ///< P-R_rate [m/s]
    double Tropo_delay;      ///< Tropo_delay [m]
    double P_R_Error;        ///< P-R_Error
    double Ant_azimuth;      ///< Ant_azimuth [rad]
    double Ant_elevation;    ///< Ant_elevation [rad]
    double Range_rate;       ///< Range_rate [m/s]
    double P_R_Error_rate;   ///< P-R_Error_rate [m/s]
    double Sat_Acc_X;        ///< Sat_Acc_X [m/s^2]
    double Sat_Acc_Y;        ///< Sat_Acc_Y [m/s^2]
    double Sat_Acc_Z;        ///< Sat_Acc_Z [m/s^2]

    bool checked = false; ///< Boolean to check if this data was visited

  private:
    double P_Range_Group_A; ///< P-Range Group A [m]
    double P_Range_Group_B; ///< P-Range Group B [m]
    double P_Range_Group_C; ///< P-Range Group C [m]
    double P_Range_Group_D; ///< P-Range Group D [m]
    double P_Range_Group_E; ///< P-Range Group E [m]
    double P_Range_Group_F; ///< P-Range Group F [m]

    double Iono_delay_Group_A; ///< Iono_delay Group A [m]
    double Iono_delay_Group_B; ///< Iono_delay Group B [m]
    double Iono_delay_Group_C; ///< Iono_delay Group C [m]
    double Iono_delay_Group_D; ///< Iono_delay Group D [m]
    double Iono_delay_Group_E; ///< Iono_delay Group E [m]
    double Iono_delay_Group_F; ///< Iono_delay Group F [m]

    double Signal_dB_Group_A; ///< Signal_dB Group A [dB]
    double Signal_dB_Group_B; ///< Signal_dB Group B [dB]
    double Signal_dB_Group_C; ///< Signal_dB Group C [dB]
    double Signal_dB_Group_D; ///< Signal_dB Group D [dB]
    double Signal_dB_Group_E; ///< Signal_dB Group E [dB]
    double Signal_dB_Group_F; ///< Signal_dB Group F [dB]

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
};

struct SpirentSatDataFile
{
    struct Margin
    {
        double pos = 0.0;
        double vel = 0.0;
        double satElevation = 0.0;
        double satAzimuth = 0.0;
        double dpsr_I = 0.0;
        double dpsr_T = 0.0;
        double pseudorange = 0.0;
        double geometricDist = 0.0;
    };

    /// @brief Constructor
    /// @param path Path to the reference file
    explicit SpirentSatDataFile(const std::string& path)
    {
        std::ifstream fs(path, std::ios_base::binary);
        REQUIRE(fs.good());
        std::string line;
        std::getline(fs, line); // Header line: Build,GPS Time,Interval ms,Channels,SIR ms,Header rows,N_freqs
        std::getline(fs, line); // Header line
        std::vector<std::string> v = str::split(line, ",");
        double gpsTime = std::stod(v.at(1));
        InsTime startTime(1980, 1, 6, 0, 0, gpsTime, GPST);
        LOG_DEBUG("{}", startTime);

        for (size_t i = 0; i < 3; i++) { std::getline(fs, line); } // Remaining Header lines

        while (!fs.eof() && std::getline(fs, line) && !line.empty())
        {
            refData.emplace_back(line);
        }
    }

    /// @brief Searches for the refeence data
    /// @param recvTime Receive Time
    /// @param satId Satellite Id
    /// @return The data or none if not found
    [[nodiscard]] std::optional<std::reference_wrapper<const SpirentAsciiSatelliteData>> get(InsTime recvTime, SatId satId) const
    {
        auto iter = std::find_if(refData.begin(), refData.end(), [&](const SpirentAsciiSatelliteData& spirentSatData) {
            return spirentSatData.recvTime == recvTime
                   && spirentSatData.satId == satId;
        });
        if (iter != refData.end())
        {
            return *iter;
        }
        return std::nullopt;
    }

    /// @brief Searches for the refeence data
    /// @param recvTime Receive Time
    /// @param satId Satellite Id
    /// @return The data or none if not found
    std::optional<std::reference_wrapper<SpirentAsciiSatelliteData>> get(InsTime recvTime, SatId satId)
    {
        auto iter = std::find_if(refData.begin(), refData.end(), [&](const SpirentAsciiSatelliteData& spirentSatData) {
            return spirentSatData.recvTime == recvTime
                   && spirentSatData.satId == satId;
        });
        if (iter != refData.end())
        {
            return *iter;
        }
        return std::nullopt;
    }

    std::vector<SpirentAsciiSatelliteData> refData; ///< Reference data read from the file
};

} // namespace NAV::TESTS