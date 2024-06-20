// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PolynomialCycleSlipDetector.hpp
/// @brief Polynomial Cycle-slip detection algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-30

#pragma once

#include <vector>
#include <utility>
#include <optional>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/Math/PolynomialRegressor.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "util/Container/Unordered_map.hpp"
#include "util/Container/Pair.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"

namespace NAV
{

/// GnssAnalyzer forward declaration
class GnssAnalyzer;
/// CycleSlipDetector forward declaration
class CycleSlipDetector;

/// Cycle-slip detection result type
enum class PolynomialCycleSlipDetectorResult
{
    Disabled,               ///< The cycle-slip detector is disabled
    LessDataThanWindowSize, ///< Less data than the specified window size (cannot predict cycle-slip yet)
    NoCycleSlip,            ///< No cycle-slip found
    CycleSlip,              ///< Cycle-slip found
};

/// @brief Cycle-slip detection
template<typename Key>
class PolynomialCycleSlipDetector
{
  public:
    /// @brief Constructor
    /// @param[in] windowSize Amount of points to use for the fit (sliding window)
    /// @param[in] polyDegree Polynomial degree to fit
    explicit PolynomialCycleSlipDetector(size_t windowSize, size_t polyDegree)
        : _windowSize(windowSize), _polyDegree(polyDegree) {}

    /// @brief Checks for a cycle slip
    /// @param[in] key Key of the detector
    /// @param[in] insTime Time of the measurement
    /// @param[in] measurementDifference Measurement difference
    /// @param[in] threshold Threshold to categorize a measurement as cycle slip
    /// @return Cycle-slip result
    [[nodiscard]] PolynomialCycleSlipDetectorResult checkForCycleSlip(const Key& key, InsTime insTime, double measurementDifference, double threshold)
    {
        if (!_enabled) { return PolynomialCycleSlipDetectorResult::Disabled; }
        if (!_detectors.contains(key))
        {
            addMeasurement(key, insTime, measurementDifference);
            return PolynomialCycleSlipDetectorResult::LessDataThanWindowSize;
        }

        const auto& detector = _detectors.at(key);
        if (!detector.polyReg.windowSizeReached())
        {
            addMeasurement(key, insTime, measurementDifference);
            return PolynomialCycleSlipDetectorResult::LessDataThanWindowSize;
        }

        auto polynomial = detector.polyReg.calcPolynomial();
        auto predictedValue = polynomial.f(calcRelativeTime(insTime, detector));

        if (std::abs(measurementDifference - predictedValue) > threshold)
        {
            reset(key);
            addMeasurement(key, insTime, measurementDifference);
            return PolynomialCycleSlipDetectorResult::CycleSlip;
        }
        addMeasurement(key, insTime, measurementDifference);
        return PolynomialCycleSlipDetectorResult::NoCycleSlip;
    }

    /// Empties the collected polynomials
    void clear()
    {
        _detectors.clear();
    }

    /// @brief Reset the polynomial for the given combination
    /// @param[in] key Key of the detector
    void reset(const Key& key)
    {
        if (_detectors.contains(key))
        {
            _detectors.erase(key);
        }
    }

    /// @brief Is the cycle-slip detector enabled?
    [[nodiscard]] bool isEnabled() const
    {
        return _enabled;
    }
    /// @brief Sets the enabled state
    /// @param[in] enabled Whether to enabled or not
    void setEnabled(bool enabled)
    {
        _enabled = enabled;
    }

    /// @brief Get the window size for the polynomial fit
    [[nodiscard]] size_t getWindowSize() const { return _windowSize; }
    /// @brief Sets the amount of points used for the fit (sliding window)
    /// @param[in] windowSize Amount of points to use for the fit
    void setWindowSize(size_t windowSize)
    {
        _windowSize = windowSize;
        for (auto& detector : _detectors)
        {
            detector.second.polyReg.setWindowSize(windowSize);
        }
    }

    /// @brief Get the degree of the polynomial which is used for fitting
    [[nodiscard]] size_t getPolynomialDegree() const { return _polyDegree; }
    /// @brief Sets the degree of the polynomial which is used for fitting
    /// @param[in] polyDegree Polynomial degree to fit
    void setPolynomialDegree(size_t polyDegree)
    {
        _polyDegree = polyDegree;
        for (auto& detector : _detectors)
        {
            detector.second.polyReg.setPolynomialDegree(polyDegree);
        }
    }

    /// Strategies for fitting
    using Strategy = PolynomialRegressor<>::Strategy;

    /// @brief Get the strategy used for fitting
    [[nodiscard]] Strategy getFitStrategy() const { return _strategy; }
    /// @brief Sets the strategy used for fitting
    /// @param[in] strategy Strategy for fitting
    void setFitStrategy(Strategy strategy)
    {
        _strategy = strategy;
        for (auto& detector : _detectors)
        {
            detector.second.polyReg.setStrategy(strategy);
        }
    }

  private:
    /// @brief Signal Detector struct
    struct SignalDetector
    {
        /// @brief Constructor
        /// @param[in] startTime Time when the first message for this detector was received
        /// @param[in] windowSize Window size for the sliding window
        /// @param[in] polyDegree Polynomial degree to fit
        /// @param[in] strategy Strategy for fitting
        SignalDetector(InsTime startTime, size_t windowSize, size_t polyDegree, Strategy strategy)
            : startTime(startTime), polyReg(polyDegree, windowSize, strategy) {}

        InsTime startTime;                   ///< Time when the first message for this detector was received
        PolynomialRegressor<double> polyReg; ///< Polynomial Regressor
    };

    bool _enabled = true;                          ///< Whether the cycle-slip detector is enabled
    size_t _windowSize;                            ///< Window size for the sliding window
    size_t _polyDegree = 2;                        ///< Polynomial degree to fit
    Strategy _strategy = Strategy::HouseholderQR;  ///< Strategy used for fitting
    unordered_map<Key, SignalDetector> _detectors; ///< Detectors, one for each key

    /// @brief Calculate the relative time to the start time of the detector
    /// @param[in] insTime Time of the measurement
    /// @param[in] detector Detector to use
    [[nodiscard]] static double calcRelativeTime(const InsTime& insTime, const SignalDetector& detector)
    {
        return static_cast<double>((insTime - detector.startTime).count());
    }
    /// @brief Calculate the relative time to the start time of the detector
    /// @param[in] key Key of the detector
    /// @param[in] insTime Time of the measurement
    [[nodiscard]] std::optional<double> calcRelativeTime(const Key& key, const InsTime& insTime) const
    {
        if (!_detectors.contains(key)) { return {}; }

        return calcRelativeTime(insTime, _detectors.at(key));
    }

    /// @brief Predicts a value from the collected data and polynomial fit
    /// @param[in] key Key of the detector
    /// @param[in] insTime Time of the measurement
    [[nodiscard]] std::optional<double> predictValue(const Key& key, const InsTime& insTime) const
    {
        if (!_detectors.contains(key)) { return {}; }

        const auto& detector = _detectors.at(key);

        auto polynomial = detector.polyReg.calcPolynomial();
        return polynomial.f(calcRelativeTime(insTime, detector));
    }

    /// @brief Calculates the polynomial from the collected data
    /// @param[in] key Key of the detector
    [[nodiscard]] std::optional<Polynomial<double>> calcPolynomial(const Key& key) const
    {
        if (!_detectors.contains(key)) { return {}; }

        return _detectors.at(key).polyReg.calcPolynomial();
    }

    /// @brief Add a measurement to the polynomial fit
    /// @param[in] key Key of the detector
    /// @param[in] insTime Time of the measurement
    /// @param[in] measurementDifference Measurement difference
    void addMeasurement(const Key& key, InsTime insTime, double measurementDifference)
    {
        if (!_enabled) { return; }
        auto& detector = _detectors.insert({ key, SignalDetector(insTime, _windowSize, _polyDegree, _strategy) }).first->second;

        detector.polyReg.push_back(calcRelativeTime(insTime, detector), measurementDifference);
    }

    friend class GnssAnalyzer;
    friend class CycleSlipDetector;
};

/// @brief Shows a GUI for advanced configuration of the PolynomialCycleSlipDetector
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] polynomialCycleSlipDetector Reference to the cycle-slip detector to configure
/// @param[in] width Width of the widget
template<typename Key>
bool PolynomialCycleSlipDetectorGui(const char* label, PolynomialCycleSlipDetector<Key>& polynomialCycleSlipDetector, float width = 0.0F)
{
    bool changed = false;

    bool enabled = polynomialCycleSlipDetector.isEnabled();
    if (ImGui::Checkbox(fmt::format("Enabled##{}", label).c_str(), &enabled))
    {
        changed = true;
        polynomialCycleSlipDetector.setEnabled(enabled);
    }

    if (!enabled) { ImGui::BeginDisabled(); }

    ImGui::SetNextItemWidth(width);
    if (int windowSize = static_cast<int>(polynomialCycleSlipDetector.getWindowSize());
        ImGui::InputIntL(fmt::format("Window size##{}", label).c_str(), &windowSize,
                         std::max(1, static_cast<int>(polynomialCycleSlipDetector.getPolynomialDegree()) + 1)))
    {
        changed = true;
        polynomialCycleSlipDetector.setWindowSize(static_cast<size_t>(windowSize));
    }

    ImGui::SetNextItemWidth(width);
    if (int polyDegree = static_cast<int>(polynomialCycleSlipDetector.getPolynomialDegree());
        ImGui::InputIntL(fmt::format("Polynomial Degree##{}", label).c_str(), &polyDegree,
                         0, std::min(static_cast<int>(polynomialCycleSlipDetector.getWindowSize()) - 1, std::numeric_limits<int>::max())))
    {
        changed = true;
        polynomialCycleSlipDetector.setPolynomialDegree(static_cast<size_t>(polyDegree));
    }

    ImGui::SetNextItemWidth(width);
    if (auto strategy = polynomialCycleSlipDetector.getFitStrategy();
        gui::widgets::EnumCombo(fmt::format("Strategy##{}", label).c_str(), strategy))
    {
        changed = true;
        polynomialCycleSlipDetector.setFitStrategy(strategy);
    }

    if (!enabled) { ImGui::EndDisabled(); }

    return changed;
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
template<typename Key>
void to_json(json& j, const PolynomialCycleSlipDetector<Key>& data)
{
    j = json{
        { "enabled", data.isEnabled() },
        { "windowSize", data.getWindowSize() },
        { "polynomialDegree", data.getPolynomialDegree() },
        { "strategy", data.getFitStrategy() },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
template<typename Key>
void from_json(const json& j, PolynomialCycleSlipDetector<Key>& data)
{
    if (j.contains("enabled"))
    {
        auto enabled = j.at("enabled").get<bool>();
        data.setEnabled(enabled);
    }
    if (j.contains("windowSize"))
    {
        auto windowSize = j.at("windowSize").get<size_t>();
        data.setWindowSize(windowSize);
    }
    if (j.contains("polynomialDegree"))
    {
        auto polynomialDegree = j.at("polynomialDegree").get<size_t>();
        data.setPolynomialDegree(polynomialDegree);
    }
    if (j.contains("strategy"))
    {
        auto strategy = j.at("strategy").get<size_t>();
        data.setFitStrategy(static_cast<typename PolynomialCycleSlipDetector<Key>::Strategy>(strategy));
    }
}

} // namespace NAV
