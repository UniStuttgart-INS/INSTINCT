// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CycleSlipDetector.hpp
/// @brief Combination of different cycle-slip detection algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-11-10

#pragma once

#include <variant>
#include <array>
#include "NodeData/GNSS/GnssObs.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "CycleSlipDetector/PolynomialCycleSlipDetector.hpp"

namespace NAV
{

/// @brief Cycle-slip detector
class CycleSlipDetector
{
  public:
    /// @brief Detectors in use
    enum class Detector
    {
        LLI,             ///< Loss-of-Lock Indicator check
        SingleFrequency, ///< Single frequency detector
        DualFrequency,   ///< Dual frequency detector
    };

    /// @brief Is the cycle-slip detector enabled?
    /// @param[in] detector Detector to request data for
    [[nodiscard]] bool isEnabled(const Detector& detector) const
    {
        switch (detector)
        {
        case Detector::LLI:
            return _enableLLICheck;
        case Detector::SingleFrequency:
            return _singleFrequencyDetector.isEnabled();
        case Detector::DualFrequency:
            return _dualFrequencyDetector.isEnabled();
        };
        return false;
    }
    /// @brief Sets the enabled state
    /// @param[in] enabled Whether to enabled or not
    /// @param[in] detector Detector to modify
    void setEnabled(bool enabled, const Detector& detector)
    {
        switch (detector)
        {
        case Detector::LLI:
            _enableLLICheck = enabled;
            break;
        case Detector::SingleFrequency:
            _singleFrequencyDetector.setEnabled(enabled);
            break;
        case Detector::DualFrequency:
            _dualFrequencyDetector.setEnabled(enabled);
            break;
        };
    }

    /// @brief Get the window size for the polynomial fit
    /// @param[in] detector Detector to request data for
    [[nodiscard]] size_t getWindowSize(const Detector& detector) const
    {
        return detector == Detector::SingleFrequency ? _singleFrequencyDetector.getWindowSize() : _dualFrequencyDetector.getWindowSize();
    }
    /// @brief Sets the amount of points used for the fit (sliding window)
    /// @param[in] windowSize Amount of points to use for the fit
    /// @param[in] detector Detector to modify
    void setWindowSize(size_t windowSize, const Detector& detector)
    {
        if (detector == Detector::SingleFrequency) { _singleFrequencyDetector.setWindowSize(windowSize); }
        else { _dualFrequencyDetector.setWindowSize(windowSize); }
    }

    /// @brief Get the threshold to categorize a measurement as cycle slip [% of smallest wavelength]
    /// @param[in] detector Detector to request data for
    [[nodiscard]] double getThreshold(const Detector& detector) const
    {
        return detector == Detector::SingleFrequency ? _singleFrequencyThresholdPercentage : _dualFrequencyThresholdPercentage;
    }
    /// @brief Sets the threshold to categorize a measurement as cycle slip
    /// @param[in] threshold Threshold value in [% of smallest wavelength]
    /// @param[in] detector Detector to modify
    void setThreshold(double threshold, const Detector& detector)
    {
        if (detector == Detector::SingleFrequency) { _singleFrequencyThresholdPercentage = threshold; }
        else { _dualFrequencyThresholdPercentage = threshold; }
    }

    /// @brief Get the degree of the polynomial which is used for fitting
    /// @param[in] detector Detector to request data for
    [[nodiscard]] size_t getPolynomialDegree(const Detector& detector) const
    {
        return detector == Detector::SingleFrequency ? _singleFrequencyDetector.getPolynomialDegree() : _dualFrequencyDetector.getPolynomialDegree();
    }
    /// @brief Sets the degree of the polynomial which is used for fitting
    /// @param[in] polyDegree Polynomial degree to fit
    /// @param[in] detector Detector to modify
    void setPolynomialDegree(size_t polyDegree, const Detector& detector)
    {
        if (detector == Detector::SingleFrequency) { _singleFrequencyDetector.setPolynomialDegree(polyDegree); }
        else { _dualFrequencyDetector.setPolynomialDegree(polyDegree); }
    }

    /// Strategies for fitting
    using Strategy = PolynomialRegressor<>::Strategy;

    /// @brief Get the strategy used for fitting
    /// @param[in] detector Detector to request data for
    [[nodiscard]] Strategy getFitStrategy(const Detector& detector) const
    {
        return detector == Detector::SingleFrequency ? _singleFrequencyDetector.getFitStrategy() : _dualFrequencyDetector.getFitStrategy();
    }
    /// @brief Sets the strategy used for fitting
    /// @param[in] strategy Strategy for fitting
    /// @param[in] detector Detector to modify
    void setFitStrategy(Strategy strategy, const Detector& detector)
    {
        if (detector == Detector::SingleFrequency) { _singleFrequencyDetector.setFitStrategy(strategy); }
        else { _dualFrequencyDetector.setFitStrategy(strategy); }
    }

    /// @brief Cycle-slip because LLI was set
    struct CycleSlipLossOfLockIndicator
    {
        SatSigId signal; ///< Signal identifier where the cycle-slip occurred
    };
    /// @brief Cycle-slip found in single frequency carrier-phase observation
    struct CycleSlipSingleFrequency
    {
        SatSigId signal; ///< Signal identifier where the cycle-slip occurred
    };
    /// @brief Cycle-slip found in dual frequency combination
    struct CycleSlipDualFrequency
    {
        std::array<SatSigId, 2> signals; ///< Signal identifiers where the cycle-slip occurred
    };

    /// @brief Result of the cycle-slip detection test
    using Result = std::variant<CycleSlipLossOfLockIndicator, CycleSlipDualFrequency, CycleSlipSingleFrequency>;

    /// Satellite observations ordered per satellite
    struct SatelliteObservation
    {
        /// @brief Signal for a code
        struct Signal
        {
            Code code;                                          ///< Code
            GnssObs::ObservationData::CarrierPhase measurement; ///< Carrier-phase measurement and LLI flag
        };

        SatId satId;                 ///< Satellite identifier
        std::vector<Signal> signals; ///< List of signals
    };

    /// @brief Checks for a cycle slip
    /// @param[in] insTime Time of the measurement
    /// @param[in] satObs Satellite observations
    /// @return Cycle-slip result
    [[nodiscard]] std::vector<Result> checkForCycleSlip(InsTime insTime, const std::vector<SatelliteObservation>& satObs);

    /// @brief Resets all data related to the provided signal
    /// @param satSigId Satellite signal identifier
    void resetSignal(const SatSigId& satSigId);

    /// @brief Resets all data
    void reset();

    /// Dual frequency combination
    struct DualFrequencyCombination
    {
        /// @brief Equal comparison (needed for unordered_map)
        /// @param[in] rhs Right hand side of the operator
        /// @return True if the elements are equal
        constexpr bool operator==(const DualFrequencyCombination& rhs) const { return satId == rhs.satId && sig1 == rhs.sig1 && sig2 == rhs.sig2; }

        /// @brief Less than comparison (needed for map)
        /// @param[in] rhs Right hand side of the operator
        /// @return True if lhs < rhs
        constexpr bool operator<(const DualFrequencyCombination& rhs) const
        {
            return satId == rhs.satId ? (sig1 == rhs.sig1
                                             ? sig2 < rhs.sig2
                                             : sig1 < rhs.sig1)
                                      : satId < rhs.satId;
        }

        SatId satId; ///< Satellite Identifier
        Code sig1;   ///< Signal code/frequency (f(sig1) > f(sig2), e.g. L1 if L1-L2)
        Code sig2;   ///< Signal code/frequency (f(sig2) < f(sig1), e.g. L2 if L1-L2)
    };

  private:
    /// @brief Whether to check for LLI flag
    bool _enableLLICheck = true;

    double _singleFrequencyThresholdPercentage = 0.9; ///< Threshold to detect a cycle-slip in [% of smallest wavelength]
    double _dualFrequencyThresholdPercentage = 0.6;   ///< Threshold to detect a cycle-slip in [% of smallest wavelength]

    /// Single Frequency carrier-phase cycle-slip detector using polynomial fits
    PolynomialCycleSlipDetector<SatSigId> _singleFrequencyDetector{ /* windowSize = */ 4, /* polyDegree = */ 2 };

    /// Dual Frequency cycle-slip detector using polynomial fits
    PolynomialCycleSlipDetector<DualFrequencyCombination> _dualFrequencyDetector{ /* windowSize = */ 2, /* polyDegree = */ 1 };

    friend bool CycleSlipDetectorGui(const char* label, CycleSlipDetector& cycleSlipDetector, float width, bool dualFrequencyAvailable);
    friend void to_json(json& j, const CycleSlipDetector& data);
    friend void from_json(const json& j, CycleSlipDetector& data);
};

/// @brief Shows a GUI for advanced configuration of the CycleSlipDetector
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] cycleSlipDetector Reference to the cycle-slip detector to configure
/// @param[in] width Width of the widget
/// @param[in] dualFrequencyAvailable Whether dual frequency is available
bool CycleSlipDetectorGui(const char* label, CycleSlipDetector& cycleSlipDetector, float width = 0.0F, bool dualFrequencyAvailable = true);

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const CycleSlipDetector& data);
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, CycleSlipDetector& data);

/// @brief Converts the detector result into a string
/// @param cycleSlip Cycle-slip
[[nodiscard]] std::string to_string(const CycleSlipDetector::Result& cycleSlip);

} // namespace NAV

namespace std
{
/// @brief Hash function for DualFrequencyCombination (needed for unordered_map)
template<>
struct hash<NAV::CycleSlipDetector::DualFrequencyCombination>
{
    /// @brief Hash function for SatId
    /// @param[in] c Dual frequency combination
    std::size_t operator()(const NAV::CycleSlipDetector::DualFrequencyCombination& c) const
    {
        auto hash1 = std::hash<NAV::SatId>{}(c.satId);
        auto hash2 = std::hash<NAV::Code>{}(c.sig1);
        auto hash3 = std::hash<NAV::Code>{}(c.sig2);

        return hash1 | (hash2 << 24) | (hash3 << 48);
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::CycleSlipDetector::Result> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] cycleSlip Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::CycleSlipDetector::Result& cycleSlip, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(to_string(cycleSlip), ctx);
    }
};

#endif