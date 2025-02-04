// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AntexReader.hpp
/// @brief ANTEX file reader
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-05-19

#pragma once

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Time/TimeSystem.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "internal/FlowManager.hpp"
#include "util/Eigen.hpp"
#include "util/Logger.hpp"
#include "util/Container/Pair.hpp"
#include "util/StringUtil.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <fmt/core.h>

namespace NAV
{

/// @brief ANTEX file reader
class AntexReader
{
  public:
    /// Antenna frequency dependant information
    struct AntennaFreqInfo
    {
        /// Eccentricities of the mean antenna phase center relative to the antenna reference point (ARP). North, east and up component in [m]
        Eigen::Vector3d phaseCenterOffsetToARP = Eigen::Vector3d::Zero();
        /// Phase center variation pattern independent from azimuth [mm]
        /// - First row is zenith angle in [rad]
        /// - Second row is the values
        Eigen::Matrix2Xd patternAzimuthIndependent;
        /// Phase center variation [mm]
        /// - First row is zenith angle in [rad]
        /// - First column is the azimuth angle in [rad]
        Eigen::MatrixXd pattern;
    };

    /// Antenna information
    struct Antenna
    {
        /// Antenna info
        struct AntennaInfo
        {
            /// @brief Constructor
            /// @param[in] date Date of measurement
            /// @param[in] from Valid from
            /// @param[in] until Valid until
            AntennaInfo(const InsTime& date, const InsTime& from, const InsTime& until)
                : date(date), from(from), until(until) {}

            InsTime date;  ///< Date of measurement
            InsTime from;  ///< Valid from
            InsTime until; ///< Valid until

            double zenithStart = 0.0;  ///< Zenith start of the phase center variation pattern in [rad]
            double zenithEnd = 0.0;    ///< Zenith end of the phase center variation pattern in [rad]
            double zenithDelta = 0.0;  ///< Zenith delta of the phase center variation pattern in [rad]
            double azimuthStart = 0.0; ///< Azimuth start of the phase center variation pattern in [rad]
            double azimuthEnd = 0.0;   ///< Azimuth end of the phase center variation pattern in [rad]
            double azimuthDelta = 0.0; ///< Azimuth delta of the phase center variation pattern in [rad]

            /// Frequency dependant information
            std::unordered_map<Frequency_, AntennaFreqInfo> freqInformation;
        };
        /// Serial number
        std::string serialNumber;

        /// Antenna Information
        std::vector<AntennaInfo> antennaInfo;
    };

    /// @brief Get the static Instance of the reader
    static AntexReader& Get()
    {
        static AntexReader self;
        return self;
    }

    /// @brief Initialize from ANTEX file
    void initialize()
    {
        if (!_antennas.empty()) { return; }

        LOG_DEBUG("Reading ANTEX files started...");

        auto path = flow::GetProgramRootPath() / "resources" / "gnss" / "antex";
        if (!std::filesystem::exists(path))
        {
            LOG_WARN("Not reading ANTEX files because path does not exist: {}", path);
            return;
        }
        for (const auto& entry : std::filesystem::directory_iterator(path))
        {
            if (entry.path().extension() != ".atx") { continue; }
            LOG_DEBUG("Reading {}...", entry.path().string());

            std::ifstream fs(entry.path());
            if (!fs.good())
            {
                LOG_ERROR("Could not read ANTEX file: {}", entry.path().string());
                return;
            }

            auto extHeaderLabel = [](const std::string& line) {
                return str::trim_copy(std::string_view(line).substr(60, 20));
            };

            std::string line;
            size_t lineNumber = 0;

            bool antennaStarted = false;
            Antenna* antenna = nullptr;
            std::vector<NAV::AntexReader::Antenna::AntennaInfo>::iterator antInfo;
            std::string antennaType;
            InsTime date;
            InsTime validFrom;
            InsTime validUntil;
            Frequency_ frequency = Freq_None;
            const double azimuthStart = 0.0;
            const double azimuthEnd = deg2rad(360.0);
            double azimuthDelta = 0.0;
            double zenithStart = 0.0;
            double zenithEnd = 0.0;
            double zenithDelta = 0.0;
#if LOG_LEVEL <= LOG_LEVEL_DATA
            bool patternLogging = false;
#endif
            while (std::getline(fs, line) && !fs.eof())
            {
                lineNumber++;
                auto label = extHeaderLabel(line);
                if (label == "START OF ANTENNA") { antennaStarted = true; }
                else if (label == "END OF ANTENNA")
                {
                    antennaStarted = false;
                    antenna = nullptr;
                    antInfo = {};
                    antennaType.clear();
                    date.reset();
                    validFrom.reset();
                    validUntil.reset();
                    azimuthDelta = 0.0;
                    zenithStart = 0.0;
                    zenithEnd = 0.0;
                    zenithDelta = 0.0;
                }
                else if (antennaStarted)
                {
                    if (label == "TYPE / SERIAL NO")
                    {
                        antennaType = str::trim_copy(line.substr(0, 20));
                        std::string serialNumber = str::trim_copy(line.substr(20, 20));
                        if (!serialNumber.empty()) { antennaType += ":" + serialNumber; }

                        antenna = &_antennas[antennaType];
                        antenna->serialNumber = serialNumber;
                        _antennaNames.insert(antennaType);
                    }
                    else if (label == "METH / BY / # / DATE")
                    {
                        // "                    COD/ESA                  0    29-JAN-17 METH / BY / # / DATE"
                        auto strDate = line.substr(50, 10);
                        auto day = std::stoi(strDate.substr(0, 2));
                        auto strMon = strDate.substr(3, 3);
                        int mon = 0;
                        if (strMon == "JAN") { mon = 1; }
                        if (strMon == "FEB") { mon = 2; }
                        if (strMon == "MAR") { mon = 3; }
                        if (strMon == "APR") { mon = 4; }
                        if (strMon == "MAY") { mon = 5; }
                        if (strMon == "JUN") { mon = 6; }
                        if (strMon == "JUL") { mon = 7; }
                        if (strMon == "AUG") { mon = 8; }
                        if (strMon == "SEP") { mon = 9; }
                        if (strMon == "OCT") { mon = 10; }
                        if (strMon == "NOV") { mon = 11; }
                        if (strMon == "DEZ") { mon = 12; }
                        auto year = 2000 + std::stoi(strDate.substr(7, 2));
                        date = InsTime(year, mon, day, 0, 0, 0.0, GPST);
                    }
                    else if (label == "DAZI")
                    {
                        // Increment of the azimuth:                  2X,F6.1,52X
                        //   0 to 360 with increment 'DAZI' (in degrees).
                        //   360 degrees have to be divisible by 'DAZI'.
                        //   Common value for 'DAZI': 5.0
                        // For non-azimuth-dependent phase center
                        // variations '0.0' has to be specified.
                        //
                        //     5.0                                                    DAZI
                        azimuthDelta = deg2rad(std::stod(line.substr(2, 6)));
                    }
                    else if (label == "ZEN1 / ZEN2 / DZEN")
                    {
                        // Receiver antenna:                          2X,3F6.1,40X
                        //   Definition of the grid in zenith angle:
                        //   Zenith distance 'ZEN1' to 'ZEN2' with increment 'DZEN' (in degrees).
                        //   'DZEN' has to be > 0.0.
                        //   'ZEN1' and 'ZEN2' always have to be multiples of 'DZEN'.
                        //   'ZEN2' always has to be greater than 'ZEN1'.
                        //   Common value for 'DZEN': 5.0
                        //   Example: '     0.0  90.0   5.0'
                        //
                        //     0.0  90.0   5.0                                        ZEN1 / ZEN2 / DZEN
                        zenithStart = deg2rad(std::stod(line.substr(2, 6)));
                        zenithEnd = deg2rad(std::stod(line.substr(8, 6)));
                        zenithDelta = deg2rad(std::stod(line.substr(14, 6)));
                    }
                    else if (label == "VALID FROM")
                    {
                        validFrom = InsTime(std::stoi(line.substr(0, 6)),
                                            std::stoi(line.substr(6, 6)),
                                            std::stoi(line.substr(12, 6)),
                                            std::stoi(line.substr(18, 6)),
                                            std::stoi(line.substr(24, 6)),
                                            std::stod(line.substr(30, 13)),
                                            GPST);
                    }
                    else if (label == "VALID UNTIL")
                    {
                        validUntil = InsTime(std::stoi(line.substr(0, 6)),
                                             std::stoi(line.substr(6, 6)),
                                             std::stoi(line.substr(12, 6)),
                                             std::stoi(line.substr(18, 6)),
                                             std::stoi(line.substr(24, 6)),
                                             std::stod(line.substr(30, 13)),
                                             GPST);
                    }
                    else if (label == "START OF FREQUENCY")
                    {
                        frequency = Frequency_(Frequency::fromString(line.substr(3, 3)));
                        if (frequency == Freq_None)
                        {
                            LOG_WARN("  AntexReader: Invalid frequency [{}] in line {} of file '{}'", line.substr(3, 3), lineNumber, entry.path());
                            continue;
                        }

#if LOG_LEVEL <= LOG_LEVEL_DATA
                        patternLogging = true;
#endif

                        antInfo = std::ranges::find_if(antenna->antennaInfo, [&](const Antenna::AntennaInfo& antInfo) {
                            return antInfo.from == validFrom && antInfo.until == validUntil;
                        });
                        if (antInfo == antenna->antennaInfo.end())
                        {
                            antenna->antennaInfo.emplace_back(date, validFrom, validUntil);
                            antInfo = antenna->antennaInfo.end() - 1;
                        }

                        antInfo->zenithStart = zenithStart;
                        antInfo->zenithEnd = zenithEnd;
                        antInfo->zenithDelta = zenithDelta;
                        antInfo->azimuthStart = azimuthStart;
                        antInfo->azimuthEnd = azimuthEnd;
                        antInfo->azimuthDelta = azimuthDelta;

                        if (antInfo->freqInformation.contains(frequency) && antInfo->date > date)
                        {
                            LOG_TRACE("  Antenna '{}' [{}]{} already exists.", antennaType, Frequency(frequency),
                                      !validFrom.empty() || !validUntil.empty() ? fmt::format(" (valid [{}] - [{}])", validFrom.toYMDHMS(GPST), validUntil.toYMDHMS(GPST)) : "");
                            frequency = Freq_None;
                        }
                    }
                    else if (label == "END OF FREQUENCY") { frequency = Freq_None; }
                    else if (frequency != Freq_None)
                    {
                        if (label == "NORTH / EAST / UP")
                        {
                            antInfo->freqInformation[frequency].phaseCenterOffsetToARP = Eigen::Vector3d(str::stod(line.substr(0, 10), 0.0),
                                                                                                         str::stod(line.substr(10, 10), 0.0),
                                                                                                         str::stod(line.substr(20, 10), 0.0))
                                                                                         * 1e-3;
                            LOG_DATA("  Adding antenna '{}' [{}]{} phaseCenterOffsetToARP: {}", antennaType, Frequency(frequency),
                                     !validFrom.empty() || !validUntil.empty() ? fmt::format(" (valid [{}] - [{}])", validFrom.toYMDHMS(GPST), validUntil.toYMDHMS(GPST)) : "",
                                     antInfo->freqInformation.at(frequency).phaseCenterOffsetToARP.transpose());
                        }
                        else if (line.substr(3, 5) == "NOAZI")
                        {
                            // (Values of a non-   | The flag 'NOAZI' denotes a non-azimuth-  | 3X,A5,mF8.2
                            // azimuth-dependent   | dependent pattern that has to be         |
                            // pattern)            | specified in any case (also if 'DAZI' >  |
                            //                     | 0.0).                                    |
                            //                     | Phase pattern values in millimeters from |
                            //                     | 'ZEN1' to 'ZEN2' (with increment 'DZEN').|
                            //                     | All values on one line.                  |
                            Eigen::Matrix2Xd& pattern = antInfo->freqInformation.at(frequency).patternAzimuthIndependent;
                            pattern = Eigen::Matrix2Xd::Zero(2, static_cast<int>(std::round(zenithEnd / zenithDelta)) + 1);
                            pattern.row(0).setLinSpaced(zenithStart, zenithEnd);
                            for (int c = 0; c < pattern.cols(); c++)
                            {
                                pattern(1, c) = std::stod(line.substr((static_cast<size_t>(c) + 1) * 8, 8)) * 1e-3;
                            }
                            LOG_DATA("  Adding antenna '{}' [{}] NOAZI pattern", antennaType, Frequency(frequency));
                        }
                        else if (azimuthDelta > 0.0)
                        {
                            // (Values of an       | The azimuth-dependent pattern has to be  | F8.1,mF8.2
                            // azimuth-dependent   | specified, if 'DAZI' > 0.0. The first    |
                            // pattern)            | value in each line denotes the azimuth   |
                            //                     | angle followed by the phase pattern      |
                            //                     | values in millimeters from 'ZEN1' to     |
                            //                     | 'ZEN2' (with increment 'DZEN').          |
                            //                     | All values of one azimuth angle on one   |
                            //                     | line.                                    |
                            Eigen::MatrixXd& pattern = antInfo->freqInformation.at(frequency).pattern;

                            if (pattern.cols() == 0)
                            {
                                pattern = Eigen::MatrixXd::Zero(static_cast<int>(std::round(azimuthEnd / azimuthDelta)) + 2,
                                                                static_cast<int>(std::round(zenithEnd / zenithDelta)) + 2);
                                pattern.row(0).rightCols(pattern.cols() - 1).setLinSpaced(zenithStart, zenithEnd);
                                pattern.col(0).bottomRows(pattern.rows() - 1).setLinSpaced(azimuthStart, azimuthEnd);
                            }
                            double azimuth = deg2rad(std::stod(line.substr(0, 8)));
                            int r = static_cast<int>(std::round((azimuth - azimuthStart) / azimuthDelta)) + 1;
                            for (int c = 0; c < pattern.cols() - 1; c++)
                            {
                                pattern(r, c + 1) = std::stod(line.substr((static_cast<size_t>(c) + 1) * 8, 8)) * 1e-3;
                            }

#if LOG_LEVEL <= LOG_LEVEL_DATA
                            if (patternLogging)
                            {
                                LOG_DATA("  Adding antenna '{}' [{}] azimuth dependent pattern", antennaType, Frequency(frequency));
                                patternLogging = false;
                            }
#endif
                        }
                    }
                }
            }
        }
        LOG_DEBUG("Reading ANTEX file finished.");
    }

    /// @brief Reset the temporary variables
    void reset()
    {
        _notFoundAnt.clear();
        _notFoundFreq.clear();
    }

    /// @brief Get the Antenna Phase Center Offset To ARP if it is found in the ANTEX file
    /// @param[in] antennaType Antenna Type
    /// @param[in] freq Frequency
    /// @param[in] insTime Time
    /// @param[in] nameId NameId of the calling node for Log output
    /// @return Phase center offset in north, east, up components in [m]
    std::optional<Eigen::Vector3d> getAntennaPhaseCenterOffsetToARP(const std::string& antennaType, Frequency_ freq, const InsTime& insTime,
                                                                    [[maybe_unused]] const std::string& nameId) const
    {
        if (auto antFreqInfo = getAntennaFrequencyInfo(antennaType, freq, insTime, nameId))
        {
            return antFreqInfo->get().phaseCenterOffsetToARP;
        }

        return std::nullopt;
    }

    /// @brief Gets the phase center variation for given elevation and azimuth
    /// @param[in] antennaType Antenna Type
    /// @param[in] freq Frequency
    /// @param[in] insTime Time
    /// @param[in] elevation Elevation angle in [rad]
    /// @param[in] azimuth Azimuth in [rad] or nullopt to use the azimuth independent (NOAZI)
    /// @param[in] nameId NameId of the calling node for Log output
    /// @return The interpolated phase center variation in [m]
    std::optional<double> getAntennaPhaseCenterVariation(const std::string& antennaType, Frequency_ freq, const InsTime& insTime,
                                                         double elevation, std::optional<double> azimuth,
                                                         [[maybe_unused]] const std::string& nameId) const
    {
        LOG_DATA("{}: getAntennaPhaseCenterVariation({}, {}, {}, {}, {})", nameId, antennaType, Frequency(freq), insTime.toYMDHMS(GPST), elevation, azimuth);
        auto antInfo = getAntennaInfo(antennaType, insTime, nameId);
        if (!antInfo.has_value()) { return std::nullopt; }

        double zenith = deg2rad(90.0) - elevation;

        if (zenith < antInfo->get().zenithStart || zenith > antInfo->get().zenithEnd
            || (azimuth && (azimuth < antInfo->get().azimuthStart || azimuth > antInfo->get().azimuthEnd)))
        {
            LOG_DATA("{}: The zenith or azimuth provided are outside the pattern in the ANTEX file", nameId);
            return std::nullopt;
        }
        auto antFreqInfo = getAntennaFrequencyInfo(antInfo->get(), antennaType, freq, nameId);
        if (!antFreqInfo.has_value()) { return std::nullopt; }

        if (!azimuth.has_value())
        {
            const Eigen::Matrix2Xd& pattern = antFreqInfo->get().patternAzimuthIndependent;
            Eigen::Index zenithLoc = -1;
            if (zenith == pattern(0, 0)) { zenithLoc = 1; }
            else if (zenith == pattern(0, Eigen::last)) { zenithLoc = pattern.cols() - 1; }
            else
            {
                zenithLoc = std::distance(
                    pattern.row(0).begin(),
                    std::upper_bound(pattern.row(0).begin(),
                                     pattern.row(0).end(),
                                     zenith));
                if (zenithLoc == 0) { zenithLoc++; }
            }
            Eigen::Index uLoc = zenithLoc - 1;
            double a = pattern(0, uLoc);
            double b = pattern(0, zenithLoc);
            double t = (zenith - a) / (b - a);

            LOG_DATA("{}: t = {:.3f} [a = {:.1f}°, z = {:.1f}°, b = {:.1f}°]", nameId, t, rad2deg(a), rad2deg(zenith), rad2deg(b));
            LOG_DATA("{}: zenith {:.1f}° at idx {} = {:.5f}", nameId, rad2deg(zenith), zenithLoc, std::lerp(pattern(1, uLoc), pattern(1, zenithLoc), t));

            return std::lerp(pattern(1, uLoc), pattern(1, zenithLoc), t);
        }

        const Eigen::MatrixXd& pattern = antFreqInfo->get().pattern;
        Eigen::Index zenithLoc = -1;
        Eigen::Index azimuthLoc = -1;
        if (zenith == pattern(0, 1)) { zenithLoc = 2; }
        else if (zenith == pattern(0, Eigen::last)) { zenithLoc = pattern.cols() - 1; }
        else
        {
            zenithLoc = std::distance(
                pattern.row(0).rightCols(pattern.cols() - 1).begin(),
                std::upper_bound(pattern.row(0).rightCols(pattern.cols() - 1).begin(),
                                 pattern.row(0).rightCols(pattern.cols() - 1).end(),
                                 zenith));
            if (zenithLoc != pattern.cols() - 1) { zenithLoc++; }
        }
        if (*azimuth == pattern(1, 0)) { azimuthLoc = 2; }
        else if (*azimuth == pattern(Eigen::last, 0)) { azimuthLoc = pattern.rows() - 1; }
        else
        {
            azimuthLoc = std::distance(
                pattern.col(0).bottomRows(pattern.rows() - 1).begin(),
                std::upper_bound(pattern.col(0).bottomRows(pattern.rows() - 1).begin(),
                                 pattern.col(0).bottomRows(pattern.rows() - 1).end(),
                                 *azimuth));
            if (azimuthLoc != pattern.rows() - 1) { azimuthLoc++; }
        }

        Eigen::Index uZenithLoc = zenithLoc - 1;
        double za = pattern(0, uZenithLoc);
        double zb = pattern(0, zenithLoc);
        double zt = (zenith - za) / (zb - za);
        LOG_DATA("{}: zenith:  t = {:.3f} [a = {:.1f}°, {:.1f}°, b = {:.1f}°]", nameId, zt, rad2deg(za), rad2deg(zenith), rad2deg(zb));
        Eigen::Index uAzimuthLoc = azimuthLoc - 1;
        double aa = pattern(uAzimuthLoc, 0);
        double ab = pattern(azimuthLoc, 0);
        double at = (*azimuth - aa) / (ab - aa);
        LOG_DATA("{}: azimuth: t = {:.3f} [a = {:.1f}°, {:.1f}°, b = {:.1f}°]", nameId, at, rad2deg(aa), rad2deg(*azimuth), rad2deg(ab));

        LOG_DATA("{}: bilinearInterpolation(tx = {:.1f}, ty = {:.1f}, c00 = {:.5f}, c10 = {:.5f}, c01 = {:.5f}, c11 = {:.5f})", nameId,
                 zt, at,
                 pattern(uAzimuthLoc, uZenithLoc), pattern(azimuthLoc, uZenithLoc),
                 pattern(uAzimuthLoc, zenithLoc), pattern(azimuthLoc, zenithLoc));
        double v = math::bilinearInterpolation(at, zt,
                                               pattern(uAzimuthLoc, uZenithLoc), pattern(azimuthLoc, uZenithLoc),
                                               pattern(uAzimuthLoc, zenithLoc), pattern(azimuthLoc, zenithLoc));
        LOG_DATA("{}: azimuth {:.1f}°, zenith {:.1f}° at idx ({},{}) = {:.5f}", nameId, rad2deg(*azimuth), rad2deg(zenith), azimuthLoc, zenithLoc, v);

        return v;
    }

    /// Antennas read from the ANTEX files
    const std::set<std::string>& antennas() const
    {
        return _antennaNames;
    };

  private:
    /// @brief Constructor
    AntexReader() = default;

    /// Antennas read from the ANTEX files
    std::unordered_map<std::string, Antenna> _antennas;

    /// Ordered names of all antennas
    std::set<std::string> _antennaNames;

    /// List of Antennas not found to emit a warning
    mutable std::unordered_set<std::string> _notFoundAnt;

    /// List of Frequencies not found to emit a warning
    mutable std::unordered_set<std::pair<std::string, Frequency_>> _notFoundFreq;

    /// @brief Get the antenna info object
    /// @param[in] antennaType Antenna Type
    /// @param[in] insTime Time
    /// @param[in] nameId NameId of the calling node for Log output
    std::optional<std::reference_wrapper<const Antenna::AntennaInfo>> getAntennaInfo(const std::string& antennaType, const InsTime& insTime,
                                                                                     [[maybe_unused]] const std::string& nameId) const
    {
        if (!_antennas.contains(antennaType))
        {
            if (!_notFoundAnt.contains(antennaType))
            {
                LOG_WARN("{}: Antenna type '{}' is not found in the ANTEX files.",
                         nameId, antennaType);
                _notFoundAnt.insert(antennaType);
            }
            return std::nullopt;
        }

        const auto& antenna = _antennas.at(antennaType);

        auto antInfo = antenna.antennaInfo.cend();
        if (antenna.antennaInfo.size() == 1) // One element only, so take it
        {
            antInfo = antenna.antennaInfo.begin();
        }
        else // No element is not possible, so more than one here, so search for time
        {
            antInfo = std::ranges::find_if(antenna.antennaInfo, [&](const Antenna::AntennaInfo& antInfo) {
                return (antInfo.from.empty() && antInfo.until.empty())
                       || (antInfo.from.empty() && insTime <= antInfo.until)
                       || (antInfo.until.empty() && antInfo.from <= insTime)
                       || (antInfo.from <= insTime && insTime <= antInfo.until);
            });
        }
        if (antInfo == antenna.antennaInfo.end()) // None matching, so take last
        {
            antInfo = antenna.antennaInfo.cend() - 1;
        }

        return *antInfo;
    }

    /// @brief Get the antenna frequency info object
    /// @param[in] antennaType Antenna Type
    /// @param[in] freq Frequency
    /// @param[in] insTime Time
    /// @param[in] nameId NameId of the calling node for Log output
    std::optional<std::reference_wrapper<const AntennaFreqInfo>> getAntennaFrequencyInfo(const std::string& antennaType, Frequency_ freq,
                                                                                         const InsTime& insTime,
                                                                                         const std::string& nameId) const
    {
        if (auto antInfo = getAntennaInfo(antennaType, insTime, nameId))
        {
            return getAntennaFrequencyInfo(antInfo->get(), antennaType, freq, nameId);
        }

        return std::nullopt;
    }

    /// @brief Get the antenna frequency info object
    /// @param[in] antInfo Antenna Info object
    /// @param[in] antennaType Antenna Type
    /// @param[in] freq Frequency
    /// @param[in] nameId NameId of the calling node for Log output
    std::optional<std::reference_wrapper<const AntennaFreqInfo>> getAntennaFrequencyInfo(const Antenna::AntennaInfo& antInfo,
                                                                                         const std::string& antennaType, Frequency_ freq,
                                                                                         [[maybe_unused]] const std::string& nameId) const
    {
        if (antInfo.freqInformation.contains(freq))
        {
            return antInfo.freqInformation.at(freq);
        }
        if (!_notFoundFreq.contains(std::make_pair(antennaType, freq)))
        {
            LOG_WARN("{}: Antenna type '{}' is does not have frequency [{}] in the ANTEX file."
                     " Please provide a new ANTEX file under 'resources/gnss/antex' or consider not using the frequency, as this can introduce height errors of several centimeter.",
                     nameId, antennaType, Frequency(freq));
            _notFoundFreq.insert(std::make_pair(antennaType, freq));
        }

        return std::nullopt;
    }
};

} // namespace NAV