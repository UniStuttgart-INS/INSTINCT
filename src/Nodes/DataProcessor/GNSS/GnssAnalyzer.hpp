// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssAnalyzer.hpp
/// @brief Allows creation of GNSS measurement combinations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-17

#pragma once

#include <vector>

#include "internal/Node/Node.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector.hpp"
#include "Navigation/Math/Polynomial.hpp"

namespace NAV
{
/// @brief Allows creation of GNSS measurement combinations
class GnssAnalyzer : public Node
{
  public:
    /// @brief Default constructor
    GnssAnalyzer();
    /// @brief Destructor
    ~GnssAnalyzer() override;
    /// @brief Copy constructor
    GnssAnalyzer(const GnssAnalyzer&) = delete;
    /// @brief Move constructor
    GnssAnalyzer(GnssAnalyzer&&) = delete;
    /// @brief Copy assignment operator
    GnssAnalyzer& operator=(const GnssAnalyzer&) = delete;
    /// @brief Move assignment operator
    GnssAnalyzer& operator=(GnssAnalyzer&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t INPUT_PORT_INDEX_GNSS_OBS = 0;          ///< @brief Flow (GnssObs)
    constexpr static size_t OUTPUT_PORT_INDEX_GNSS_COMBINATION = 0; ///< @brief Flow (GnssCombination)

    /// Combination of GNSS measurements
    struct Combination
    {
        /// Possible units to calculate the combination in
        enum class Unit : uint8_t
        {
            Meters, ///< Meters
            Cycles, ///< Cycles
        };

        /// Unit to calculate the combination in
        Unit unit = Unit::Meters;

        /// Term of a combination equation
        struct Term
        {
            /// @brief Observation types
            enum class ObservationType : uint8_t
            {
                Pseudorange, ///< Pseudorange
                Carrier,     ///< Carrier-Phase
            };

            int sign = +1;                                      ///< +1 or -1
            SatSigId satSigId = { Code::G1C, 1 };               ///< SignalId and satellite number
            int8_t freqNum = -128;                              ///< Frequency number. Only used for GLONASS G1 and G2 // TODO: Set this somewhere
            ObservationType obsType = ObservationType::Carrier; ///< Observation Type
            bool receivedDuringRun = false;                     ///< Flag weather the signal was received
        };
        std::vector<Term> terms{ Term() }; ///< List of terms making up the combination

        /// Cycle-slip detector
        PolynomialCycleSlipDetector<std::string> polynomialCycleSlipDetector{ /* windowSize = */ 4, /* polyDegree = */ 2 };
        /// Threshold to categorize a measurement as cycle slip [% of smallest wavelength]
        double polynomialCycleSlipDetectorThresholdPercentage = 0.5;
        /// Whether to output the prediction even when the window size is not reached
        bool polynomialCycleSlipDetectorOutputWhenWindowSizeNotReached = false;
        /// Whether the polynomials should be outputted
        bool polynomialCycleSlipDetectorOutputPolynomials = false;
        /// Polynomial collection
        std::vector<std::pair<InsTime, Polynomial<double>>> polynomials;

        /// @brief Get a string description of the combination
        [[nodiscard]] std::string description() const
        {
            std::string desc;
            for (const auto& term : terms)
            {
                if (!desc.empty()) { desc += " "; }
                desc += term.sign == 1 ? "+" : "-";
                desc += " ";
                switch (term.obsType)
                {
                case Term::ObservationType::Pseudorange:
                    desc += unit == Unit::Cycles ? "P" : "p";
                    break;
                case Term::ObservationType::Carrier:
                    desc += unit == Unit::Cycles ? "Φ" : "φ";
                    break;
                }
                desc += fmt::format("({})", term.satSigId);
            }
            if (unit == Unit::Cycles) { desc += " [cycles]"; }
            else { desc += " [m]"; }

            return desc;
        }

        /// @brief Calculates the combined frequency of all terms
        [[nodiscard]] double calcCombinationFrequency() const
        {
            double combinedFreq = 0.0;
            for (const auto& term : terms)
            {
                double freq = term.satSigId.freq().getFrequency(term.freqNum);
                combinedFreq += static_cast<double>(term.sign) * freq;
            }
            return combinedFreq == 0 ? terms.front().satSigId.freq().getFrequency(terms.front().freqNum) : combinedFreq;
        }
    };

    /// Combinations to calculate
    std::vector<Combination> _combinations{ Combination() };

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Gnss observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Write info to a json object
    /// @param[out] j Json output
    /// @param[in] data Object to read info from
    friend void to_json(json& j, const Combination& data);
    /// @brief Read info from a json object
    /// @param[in] j Json variable to read info from
    /// @param[out] data Output object
    friend void from_json(const json& j, Combination& data);
    /// @brief Write info to a json object
    /// @param[out] j Json output
    /// @param[in] data Object to read info from
    friend void to_json(json& j, const Combination::Term& data);
    /// @brief Read info from a json object
    /// @param[in] j Json variable to read info from
    /// @param[out] data Output object
    friend void from_json(const json& j, Combination::Term& data);
};

} // namespace NAV
