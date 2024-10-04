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

            /// Frequency dependant information
            std::unordered_map<Frequency_, AntennaFreqInfo> freqInformation;
        };

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

            bool antenna = false;
            std::string antennaType;
            InsTime date;
            InsTime validFrom;
            InsTime validUntil;
            Frequency_ frequency = Freq_None;
            while (std::getline(fs, line) && !fs.eof())
            {
                auto label = extHeaderLabel(line);
                if (label == "START OF ANTENNA") { antenna = true; }
                else if (label == "END OF ANTENNA")
                {
                    antenna = false;
                    antennaType.clear();
                    date.reset();
                    validFrom.reset();
                    validUntil.reset();
                }
                else if (antenna)
                {
                    if (label == "TYPE / SERIAL NO")
                    {
                        antennaType = str::trim_copy(line.substr(0, 20));
                        // if (auto pos = antennaType.find("  ");
                        //     pos != std::string::npos)
                        // {
                        //     antennaType = antennaType.substr(0, pos);
                        // }
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
                        frequency = getFreqFromString(line.substr(3, 3));
                    }
                    else if (label == "END OF FREQUENCY") { frequency = Freq_None; }
                    else if (frequency != Freq_None)
                    {
                        if (label == "NORTH / EAST / UP")
                        {
                            auto& antenna = _antennas[antennaType];
                            _antennaNames.insert(antennaType);
                            auto antInfo = std::find_if(antenna.antennaInfo.begin(), antenna.antennaInfo.end(), [&](const Antenna::AntennaInfo& antInfo) {
                                return antInfo.from == validFrom && antInfo.until == validUntil;
                            });
                            if (antInfo == antenna.antennaInfo.end())
                            {
                                antenna.antennaInfo.emplace_back(date, validFrom, validUntil);
                                antInfo = antenna.antennaInfo.end() - 1;
                            }

                            if (antInfo->freqInformation.contains(frequency) && antInfo->date > date)
                            {
                                LOG_TRACE("  Antenna '{}' [{}]{} already exists.", antennaType, Frequency(frequency),
                                          !validFrom.empty() || !validUntil.empty() ? fmt::format(" (valid [{}] - [{}])", validFrom.toYMDHMS(GPST), validUntil.toYMDHMS(GPST)) : "");
                            }
                            else
                            {
                                antInfo->freqInformation[frequency].phaseCenterOffsetToARP = Eigen::Vector3d(str::stod(line.substr(0, 10), 0.0),
                                                                                                             str::stod(line.substr(10, 10), 0.0),
                                                                                                             str::stod(line.substr(20, 10), 0.0))
                                                                                             * 1e-3;
                                LOG_DATA("  Adding antenna '{}' [{}]{} phaseCenterOffsetToARP: {}", antennaType, Frequency(frequency),
                                         !validFrom.empty() || !validUntil.empty() ? fmt::format(" (valid [{}] - [{}])", validFrom.toYMDHMS(GPST), validUntil.toYMDHMS(GPST)) : "",
                                         antInfo->freqInformation.at(frequency).phaseCenterOffsetToARP.transpose());
                            }
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
    std::optional<Eigen::Vector3d> getAntennaPhaseCenterOffsetToARP(const std::string& antennaType, Frequency_ freq, const InsTime& insTime,
                                                                    [[maybe_unused]] const std::string& nameId)
    {
        if (_antennas.contains(antennaType))
        {
            const auto& antenna = _antennas.at(antennaType);

            auto antInfo = antenna.antennaInfo.cend();
            if (antenna.antennaInfo.size() == 1) // One element only, so take it
            {
                antInfo = antenna.antennaInfo.begin();
            }
            else // No element is not possible, so more than one here, so search for time
            {
                antInfo = std::find_if(antenna.antennaInfo.begin(), antenna.antennaInfo.end(), [&](const Antenna::AntennaInfo& antInfo) {
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

            if (antInfo->freqInformation.contains(freq))
            {
                return antInfo->freqInformation.at(freq).phaseCenterOffsetToARP;
            }
            if (!_notFoundFreq.contains(std::make_pair(antennaType, freq)))
            {
                LOG_WARN("{}: Cannot determine phase center offset, because antenna type '{}' is does not have frequency [{}] in the ANTEX file."
                         " Please provide a new ANTEX file under 'resources/gnss/antex' or consider not using the frequency, as this can introduce height errors of several centimeter.",
                         nameId, antennaType, Frequency(freq));
                _notFoundFreq.insert(std::make_pair(antennaType, freq));
            }
        }
        else if (!_notFoundAnt.contains(antennaType))
        {
            LOG_WARN("{}: Cannot determine phase center offset, because antenna type '{}' is not found in the ANTEX files.",
                     nameId, antennaType);
            _notFoundAnt.insert(antennaType);
        }

        return std::nullopt;
    }

    /// Antennas read from the ANTEX files
    const std::set<std::string>& antennas() const { return _antennaNames; };

  private:
    /// @brief Constructor
    AntexReader() = default;

    /// Antennas read from the ANTEX files
    std::unordered_map<std::string, Antenna> _antennas;

    /// Ordered names of all antennas
    std::set<std::string> _antennaNames;

    /// List of Antennas not found to emit a warning
    std::unordered_set<std::string> _notFoundAnt;

    /// List of Frequencies not found to emit a warning
    std::unordered_set<std::pair<std::string, Frequency_>> _notFoundFreq;

    /// @brief Get the Freq from a string
    /// @param[in] str Frequency as string
    static Frequency_ getFreqFromString(const std::string& str)
    {
        if (str == "G01") { return G01; }
        if (str == "G02") { return G02; }
        if (str == "G05") { return G05; }
        if (str == "R01") { return R01; }
        if (str == "R02") { return R02; }
        if (str == "E01") { return E01; }
        if (str == "E05") { return E05; }
        if (str == "E07") { return E07; }
        if (str == "E08") { return E08; }
        if (str == "E06") { return E06; }
        if (str == "C01") { return B01; }
        if (str == "C02") { return B02; }
        if (str == "C07") { return B07; }
        if (str == "C06") { return B06; }
        if (str == "J01") { return J01; }
        if (str == "J02") { return J02; }
        if (str == "J05") { return J05; }
        if (str == "J06") { return J06; }
        if (str == "S01") { return S01; }
        if (str == "S05") { return S05; }

        return Freq_None;
    }
};

} // namespace NAV