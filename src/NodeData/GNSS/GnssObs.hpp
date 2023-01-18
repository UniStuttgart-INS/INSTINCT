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
        /// @param[in] code Signal code
        ObservationData(const SatSigId& satSigId, const Code code) : satSigId(satSigId), code(code) {}

#ifdef TESTING
        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (frequency and satellite number)
        /// @param[in] code Signal code
        /// @param[in] pseudorange Pseudorange measurement [m] and Signal Strength Indicator (SSI)
        /// @param[in] carrierPhase Carrier phase measurement [cycles], Signal Strength Indicator (SSI) and Loss of Lock Indicator (LLI)
        /// @param[in] doppler Doppler measurement [Hz]
        /// @param[in] CN0 Carrier-to-Noise density [dBHz]
        ObservationData(const SatSigId& satSigId,
                        const Code code,
                        std::optional<Pseudorange> pseudorange,
                        std::optional<CarrierPhase> carrierPhase,
                        std::optional<double> doppler,
                        std::optional<double> CN0)
            : satSigId(satSigId),
              code(code),
              pseudorange(pseudorange),
              carrierPhase(carrierPhase),
              doppler(doppler),
              CN0(CN0)
        {}
#endif

        SatSigId satSigId = { Freq_None, 0 };     ///< Frequency and satellite number
        Code code;                                ///< GNSS Code
        std::optional<Pseudorange> pseudorange;   ///< Pseudorange measurement
        std::optional<CarrierPhase> carrierPhase; ///< Carrier phase measurement
        std::optional<double> doppler;            ///< Doppler measurement [Hz]
        std::optional<double> CN0;                ///< Carrier-to-Noise density [dBHz]
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

    /// @brief Return the element with the identifier or a newly constructed one if it did not exist
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    /// @param[in] code Signal code
    /// @return The element found in the observations or a newly constructed one
    ObservationData& operator()(const Frequency& freq, uint16_t satNum, Code code)
    {
        auto iter = std::find_if(data.begin(), data.end(), [freq, satNum, code](const ObservationData& idData) {
            return idData.satSigId.freq == freq && idData.satSigId.satNum == satNum && idData.code == code;
        });
        if (iter != data.end())
        {
            return *iter;
        }

        data.emplace_back(SatSigId{ freq, satNum }, code);
        return data.back();
    }

    /// @brief Return the element with the identifier
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    /// @param[in] code Signal code
    /// @return The element found in the observations
    const ObservationData& operator()(const Frequency& freq, uint16_t satNum, Code code) const
    {
        auto iter = std::find_if(data.begin(), data.end(), [freq, satNum, code](const ObservationData& idData) {
            return idData.satSigId.freq == freq && idData.satSigId.satNum == satNum && idData.code == code;
        });

        INS_ASSERT_USER_ERROR(iter != data.end(), "You can not insert new elements in a const context.");
        return *iter;
    }
};

} // namespace NAV
