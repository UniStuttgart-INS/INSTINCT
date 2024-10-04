// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssObs.hpp
/// @brief GNSS Observation messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-26

#pragma once

#include <limits>
#include <optional>
#include <vector>
#include <algorithm>
#include <Eigen/Core>

#include "NodeData/NodeData.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "util/Assert.h"

namespace NAV
{
/// GNSS Observation message information
class GnssObs : public NodeData
{
  public:
    /// @brief Observation types
    enum ObservationType
    {
        Pseudorange,           ///< Pseudorange
        Carrier,               ///< Carrier-Phase
        Doppler,               ///< Doppler (Pseudorange rate)
        ObservationType_COUNT, ///< Count
    };

    /// @brief Stores the satellites observations
    struct ObservationData
    {
        /// Pseudorange
        struct Pseudorange
        {
            /// Pseudorange measurement [m]
            double value = 0.0;

            /// @brief Signal Strength Indicator (SSI) projected into interval 1-9
            ///
            /// Carrier to Noise ratio(dbHz) | Carrier to Noise ratio(RINEX)
            /// :-:   | ---
            ///   -   | 0 or blank: not known, don't care
            /// < 12  | 1 (minimum possible signal strength)
            /// 12-17 | 2
            /// 18-23 | 3
            /// 24-29 | 4
            /// 30-35 | 5 (threshold for good tracking)
            /// 36-41 | 6
            /// 42-47 | 7
            /// 48-53 | 8
            /// ≥ 54  | 9 (maximum possible signal strength)
            uint8_t SSI = 0;
        };

        /// Carrier phase
        struct CarrierPhase
        {
            /// Carrier phase measurement [cycles]
            double value = 0.0;

            /// @brief Signal Strength Indicator (SSI) projected into interval 1-9
            ///
            /// Carrier to Noise ratio(dbHz) | Carrier to Noise ratio(RINEX)
            /// :-:   | ---
            ///   -   | 0 or blank: not known, don't care
            /// < 12  | 1 (minimum possible signal strength)
            /// 12-17 | 2
            /// 18-23 | 3
            /// 24-29 | 4
            /// 30-35 | 5 (threshold for good tracking)
            /// 36-41 | 6
            /// 42-47 | 7
            /// 48-53 | 8
            /// ≥ 54  | 9 (maximum possible signal strength)
            uint8_t SSI = 0;

            /// Loss of Lock Indicator [0...6] (only associated with the phase observation)
            uint8_t LLI = 0;
        };

        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (frequency and satellite number)
        explicit ObservationData(const SatSigId& satSigId) : satSigId(satSigId) {}

#ifdef TESTING
        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (frequency and satellite number)
        /// @param[in] pseudorange Pseudorange measurement [m] and Signal Strength Indicator (SSI)
        /// @param[in] carrierPhase Carrier phase measurement [cycles], Signal Strength Indicator (SSI) and Loss of Lock Indicator (LLI)
        /// @param[in] doppler Doppler measurement [Hz]
        /// @param[in] CN0 Carrier-to-Noise density [dBHz]
        ObservationData(const SatSigId& satSigId,
                        std::optional<Pseudorange> pseudorange,
                        std::optional<CarrierPhase> carrierPhase,
                        std::optional<double> doppler,
                        std::optional<double> CN0)
            : satSigId(satSigId),
              pseudorange(pseudorange),
              carrierPhase(carrierPhase),
              doppler(doppler),
              CN0(CN0)
        {}
#endif

        SatSigId satSigId = { Code::None, 0 };    ///< SignalId and satellite number
        std::optional<Pseudorange> pseudorange;   ///< Pseudorange measurement [m]
        std::optional<CarrierPhase> carrierPhase; ///< Carrier phase measurement [cycles]
        std::optional<double> doppler;            ///< Doppler measurement [Hz]
        std::optional<double> CN0;                ///< Carrier-to-Noise density [dBHz]
    };

    /// @brief Useful information of the satellites
    struct SatelliteData
    {
        SatId satId;                       ///< Satellite identifier
        Frequency frequencies = Freq_None; ///< Frequencies transmitted by this satellite
    };

#ifdef TESTING
    /// Default constructor
    GnssObs() = default;

    /// @brief Constructor
    /// @param[in] insTime Epoch time
    /// @param[in] data Observation data
    /// @param[in] satData Satellite data
    GnssObs(const InsTime& insTime, std::vector<ObservationData> data, std::vector<SatelliteData> satData)
        : data(std::move(data)), _satData(std::move(satData))
    {
        this->insTime = insTime;
    }
#endif
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "GnssObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// @brief Satellite observations
    std::vector<ObservationData> data;

    /// @brief Access or insert the satellite data
    /// @param satId Satellite identifier
    /// @return The satellite data
    SatelliteData& satData(const SatId& satId)
    {
        auto iter = std::find_if(_satData.begin(), _satData.end(), [&satId](const SatelliteData& sat) {
            return sat.satId == satId;
        });
        if (iter != _satData.end())
        {
            return *iter;
        }

        _satData.emplace_back();
        _satData.back().satId = satId;
        return _satData.back();
    }

    /// @brief Access the satellite data
    /// @param satId Satellite identifier
    /// @return The satellite data if in the list
    [[nodiscard]] std::optional<std::reference_wrapper<const SatelliteData>> satData(const SatId& satId) const
    {
        auto iter = std::find_if(_satData.begin(), _satData.end(), [&satId](const SatelliteData& sat) {
            return sat.satId == satId;
        });
        if (iter != _satData.end())
        {
            return *iter;
        }
        return std::nullopt;
    }

    /// @brief Checks if an element with the identifier exists
    /// @param[in] satSigId Signal id
    /// @return True if the element exists
    [[nodiscard]] bool contains(const SatSigId& satSigId) const
    {
        auto iter = std::find_if(data.begin(), data.end(), [&satSigId](const ObservationData& idData) {
            return idData.satSigId == satSigId;
        });
        return iter != data.end();
    }

    /// @brief Return the element with the identifier or a newly constructed one if it did not exist
    /// @param[in] satSigId Signal id
    /// @return The element found in the observations or a newly constructed one
    ObservationData& operator()(const SatSigId& satSigId)
    {
        auto iter = std::find_if(data.begin(), data.end(), [&satSigId](const ObservationData& idData) {
            return idData.satSigId == satSigId;
        });
        if (iter != data.end())
        {
            return *iter;
        }

        data.emplace_back(satSigId);
        return data.back();
    }

    /// @brief Return the element with the identifier
    /// @param[in] satSigId Signal id
    /// @return The element found in the observations
    [[nodiscard]] std::optional<std::reference_wrapper<const ObservationData>> operator()(const SatSigId& satSigId) const
    {
        auto iter = std::find_if(data.begin(), data.end(), [&satSigId](const ObservationData& idData) {
            return idData.satSigId == satSigId;
        });

        if (iter != data.end())
        {
            return *iter;
        }
        return std::nullopt;
    }

    /// @brief Useful information of the satellites
    [[nodiscard]] const std::vector<SatelliteData>& getSatData() const { return _satData; }

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;
        descriptors.reserve(data.size() * 7);

        for (const auto& obsData : data)
        {
            descriptors.push_back(fmt::format("{} Pseudorange [m]", obsData.satSigId));
            descriptors.push_back(fmt::format("{} Pseudorange SSI", obsData.satSigId));

            descriptors.push_back(fmt::format("{} Carrier-phase [cycles]", obsData.satSigId));
            descriptors.push_back(fmt::format("{} Carrier-phase SSI", obsData.satSigId));
            descriptors.push_back(fmt::format("{} Carrier-phase LLI", obsData.satSigId));

            descriptors.push_back(fmt::format("{} Doppler [Hz]", obsData.satSigId));
            descriptors.push_back(fmt::format("{} Carrier-to-Noise density [dBHz]", obsData.satSigId));
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        for (const auto& obsData : data)
        {
            if (descriptor == fmt::format("{} Pseudorange [m]", obsData.satSigId) && obsData.pseudorange)
            {
                return obsData.pseudorange->value;
            }
            if (descriptor == fmt::format("{} Pseudorange SSI", obsData.satSigId) && obsData.pseudorange)
            {
                return obsData.pseudorange->SSI;
            }
            if (descriptor == fmt::format("{} Carrier-phase [cycles]", obsData.satSigId) && obsData.carrierPhase)
            {
                return obsData.carrierPhase->value;
            }
            if (descriptor == fmt::format("{} Carrier-phase SSI", obsData.satSigId) && obsData.carrierPhase)
            {
                return obsData.carrierPhase->SSI;
            }
            if (descriptor == fmt::format("{} Carrier-phase LLI", obsData.satSigId) && obsData.carrierPhase)
            {
                return obsData.carrierPhase->LLI;
            }
            if (descriptor == fmt::format("{} Doppler [Hz]", obsData.satSigId) && obsData.doppler)
            {
                return obsData.doppler.value();
            }
            if (descriptor == fmt::format("{} Carrier-to-Noise density [dBHz]", obsData.satSigId) && obsData.CN0)
            {
                return obsData.CN0.value();
            }
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;
        dynData.reserve(data.size() * 7);
        for (const auto& obsData : data)
        {
            if (obsData.pseudorange) { dynData.emplace_back(fmt::format("{} Pseudorange [m]", obsData.satSigId), obsData.pseudorange->value); }
            if (obsData.pseudorange) { dynData.emplace_back(fmt::format("{} Pseudorange SSI", obsData.satSigId), obsData.pseudorange->SSI); }

            if (obsData.carrierPhase) { dynData.emplace_back(fmt::format("{} Carrier-phase [cycles]", obsData.satSigId), obsData.carrierPhase->value); }
            if (obsData.carrierPhase) { dynData.emplace_back(fmt::format("{} Carrier-phase SSI", obsData.satSigId), obsData.carrierPhase->SSI); }
            if (obsData.carrierPhase) { dynData.emplace_back(fmt::format("{} Carrier-phase LLI", obsData.satSigId), obsData.carrierPhase->LLI); }

            if (obsData.doppler) { dynData.emplace_back(fmt::format("{} Doppler [Hz]", obsData.satSigId), obsData.doppler.value()); }

            if (obsData.CN0) { dynData.emplace_back(fmt::format("{} Carrier-to-Noise density [dBHz]", obsData.satSigId), obsData.CN0.value()); }
        }
        return dynData;
    }

    /// Receiver Information, e.g. from RINEX header
    struct ReceiverInfo
    {
        ///< Approximate receiver position in [m], e.g. from RINEX header
        std::optional<Eigen::Vector3d> e_approxPos;

        /// Antenna Type. Empty if unknown
        std::string antennaType;

        /// @brief Antenna Delta (North, East, Up) in [m]
        ///
        /// - Horizontal eccentricity of ARP relative to the marker (north/east)
        /// - Height of the antenna reference point (ARP) above the marker
        Eigen::Vector3d antennaDeltaNEU = Eigen::Vector3d::Zero();
    };

    /// Optional Receiver Information, e.g. from RINEX header
    std::optional<std::reference_wrapper<ReceiverInfo>> receiverInfo;

  private:
    /// @brief Useful information of the satellites
    std::vector<SatelliteData> _satData;
};

/// @brief Converts the enum to a string
/// @param[in] obsType Enum value to convert into text
/// @return String representation of the enum
constexpr const char* to_string(GnssObs::ObservationType obsType)
{
    switch (obsType)
    {
    case GnssObs::Pseudorange:
        return "Pseudorange";
    case GnssObs::Carrier:
        return "Carrier";
    case GnssObs::Doppler:
        return "Doppler";
    case GnssObs::ObservationType_COUNT:
        return "COUNT";
    }
    return "";
}

} // namespace NAV

#ifndef DOXYGEN_IGNORE

template<>
struct fmt::formatter<NAV::GnssObs::ObservationType> : fmt::formatter<const char*>
{
    /// @brief Defines how to format Frequency structs
    /// @param[in] obsType Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GnssObs::ObservationType& obsType, FormatContext& ctx) const
    {
        return fmt::formatter<const char*>::format(to_string(obsType), ctx);
    }
};

#endif

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::GnssObs::ObservationType& obj);