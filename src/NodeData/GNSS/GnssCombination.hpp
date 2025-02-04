// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssCombination.hpp
/// @brief GNSS measurement combinations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-17

#pragma once

#include <vector>
#include <tuple>

#include "NodeData/NodeData.hpp"
#include "GnssObs.hpp"
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector/PolynomialCycleSlipDetector.hpp"

namespace NAV
{

/// GNSS measurement combinations
class GnssCombination : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "GnssCombination";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// Combination of GNSS measurements
    struct Combination
    {
        /// Term of a combination equation
        struct Term
        {
            int sign = +1;                                                        ///< +1 or -1
            SatSigId satSigId = { Code::None, 0 };                                ///< SignalId and satellite number
            GnssObs::ObservationType obsType = GnssObs::ObservationType::Carrier; ///< Observation Type
            std::optional<double> value;                                          ///< Measurement (if present)
        };

        std::string description;      ///< String describing the combination
        std::optional<double> result; ///< Calculated combination (only set if all terms where present)
        std::vector<Term> terms;      ///< List of terms making up the combination

        std::optional<PolynomialCycleSlipDetectorResult> cycleSlipResult;                  ///< Cycle-slip result
        std::optional<double> cycleSlipPrediction;                                         ///< Predicted value from the cycle-slip detector (polynomial fit)
        std::optional<double> cycleSlipMeasMinPred;                                        ///< Measurement minus predicted value from the cycle-slip detector
        std::vector<std::tuple<InsTime, Polynomial<double>, double>> cycleSlipPolynomials; ///< Polynomial fits
    };

    /// List of combinations
    std::vector<Combination> combinations;

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;
        descriptors.reserve(combinations.size() * 4);

        for (const auto& comb : combinations)
        {
            descriptors.push_back(comb.description);
            descriptors.push_back(comb.description + " Cycle Slip");
            descriptors.push_back(comb.description + " Prediction");
            descriptors.push_back(comb.description + " Meas - Pred");
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        for (const auto& comb : combinations)
        {
            if (descriptor == comb.description) { return comb.result; }
            if (descriptor == comb.description + " Cycle Slip" && comb.cycleSlipResult) { return static_cast<double>(*comb.cycleSlipResult); }
            if (descriptor == comb.description + " Prediction") { return comb.cycleSlipPrediction; }
            if (descriptor == comb.description + " Meas - Pred") { return comb.cycleSlipMeasMinPred; }
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;
        dynData.reserve(combinations.size() * 4);
        for (const auto& comb : combinations)
        {
            if (comb.result) { dynData.emplace_back(comb.description, *comb.result); }
            if (comb.cycleSlipResult) { dynData.emplace_back(comb.description + " Cycle Slip", static_cast<double>(*comb.cycleSlipResult)); }
            if (comb.cycleSlipPrediction) { dynData.emplace_back(comb.description + " Prediction", *comb.cycleSlipPrediction); }
            if (comb.cycleSlipMeasMinPred) { dynData.emplace_back(comb.description + " Meas - Pred", *comb.cycleSlipMeasMinPred); }
        }
        return dynData;
    }
};

} // namespace NAV