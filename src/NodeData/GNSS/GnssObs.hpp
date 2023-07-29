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
        Pseudorange, ///< Pseudorange
        Carrier,     ///< Carrier-Phase
        Doppler,     ///< Doppler (Pseudorange rate)
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
    GnssObs(const InsTime& insTime, std::vector<ObservationData> data)
        : data(std::move(data))
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
    std::optional<std::reference_wrapper<const ObservationData>> operator()(const SatSigId& satSigId) const
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
    }
    return "";
}

} // namespace NAV
