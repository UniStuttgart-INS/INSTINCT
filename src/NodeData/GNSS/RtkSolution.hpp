// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtkSolution.hpp
/// @brief RTK Node/Algorithm output
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-28

#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include "Navigation/GNSS/Positioning/ObservationFilter.hpp"
#include "util/Assert.h"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/State/PosVel.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Positioning/RTK/Keys.hpp"
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Container/UncertainValue.hpp"
#include "util/Container/KeyedMatrix.hpp"

namespace NAV
{
/// SPP Algorithm output
class RtkSolution : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "RtkSolution";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = PosVel::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Solution Type");
        desc.emplace_back("Number satellites");
        desc.emplace_back("Number pseudorange observables");
        desc.emplace_back("Number carrier observables");
        desc.emplace_back("Number doppler observables");
        desc.emplace_back("Number pseudorange observables (unique per satellite)");
        desc.emplace_back("Number carrier observables (unique per satellite)");
        desc.emplace_back("Number doppler observables (unique per satellite)");
        desc.emplace_back("Number of Ambiguities fixed");
        desc.emplace_back("NIS Triggered (Initial)");
        desc.emplace_back("NIS value (Initial)");
        desc.emplace_back("NIS r2 upper boundary (Initial)");
        desc.emplace_back("NIS removed observations");
        desc.emplace_back("NIS Triggered (Final)");
        desc.emplace_back("NIS value (Final)");
        desc.emplace_back("NIS r2 upper boundary (Final)");
        desc.emplace_back("Distance Rover-Base [m]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return PosVel::GetStaticDescriptorCount() + 17; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        if (idx < PosVel::GetStaticDescriptorCount()) { return PosVel::getValueAt(idx); }
        switch (idx)
        {
        case PosVel::GetStaticDescriptorCount() + 0: // Solution Type
            return static_cast<double>(solType);
        case PosVel::GetStaticDescriptorCount() + 1: // Number satellites
            return static_cast<double>(nSatellites);
        case PosVel::GetStaticDescriptorCount() + 2: // Number pseudorange observables
            if (nObservations.contains(GnssObs::Pseudorange)) { return static_cast<double>(nObservations.at(GnssObs::Pseudorange)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 3: // Number carrier observables
            if (nObservations.contains(GnssObs::Carrier)) { return static_cast<double>(nObservations.at(GnssObs::Carrier)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 4: // Number doppler observables
            if (nObservations.contains(GnssObs::Doppler)) { return static_cast<double>(nObservations.at(GnssObs::Doppler)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 5: // Number pseudorange observables unique satellite
            if (nObservationsUniqueSatellite.contains(GnssObs::Pseudorange)) { return static_cast<double>(nObservationsUniqueSatellite.at(GnssObs::Pseudorange)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 6: // Number carrier observables unique satellite
            if (nObservationsUniqueSatellite.contains(GnssObs::Carrier)) { return static_cast<double>(nObservationsUniqueSatellite.at(GnssObs::Carrier)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 7: // Number doppler observables unique satellite
            if (nObservationsUniqueSatellite.contains(GnssObs::Doppler)) { return static_cast<double>(nObservationsUniqueSatellite.at(GnssObs::Doppler)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 8: // Number of Ambiguities fixed
            if (nAmbiguitiesFixed) { return static_cast<double>(nAmbiguitiesFixed.value()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 9: // NIS Triggered (Initial)
            if (nisResultInitial) { return static_cast<double>(nisResultInitial->triggered); }
            break;
        case PosVel::GetStaticDescriptorCount() + 10: // NIS value (Initial)
            if (nisResultInitial) { return nisResultInitial->NIS; }
            break;
        case PosVel::GetStaticDescriptorCount() + 11: // NIS r2 upper boundary (Initial)
            if (nisResultInitial) { return nisResultInitial->r2; }
            break;
        case PosVel::GetStaticDescriptorCount() + 12: // NIS removed observations
            return static_cast<double>(nisRemovedCnt);
        case PosVel::GetStaticDescriptorCount() + 13: // NIS Triggered (Final)
            if (nisResultFinal) { return static_cast<double>(nisResultFinal->triggered); }
            break;
        case PosVel::GetStaticDescriptorCount() + 14: // NIS value (Final)
            if (nisResultFinal) { return nisResultFinal->NIS; }
            break;
        case PosVel::GetStaticDescriptorCount() + 15: // NIS r2 upper boundary (Final)
            if (nisResultFinal) { return nisResultFinal->r2; }
            break;
        case PosVel::GetStaticDescriptorCount() + 16: // Distance Rover-Base [m]
            return distanceBaseRover;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;
        descriptors.reserve(ambiguityDD_br.size() * 2 + static_cast<size_t>(measInnovation.rows()));

        for (const auto& ambDD : ambiguityDD_br)
        {
            descriptors.push_back(fmt::format("AmbDD {} [cycles]", ambDD.satSigId));
            descriptors.push_back(fmt::format("AmbDD StDev {} [cycles]", ambDD.satSigId));
        }

        for (const auto& key : measInnovation.rowKeys())
        {
            if (std::holds_alternative<RTK::Meas::PsrDD>(key)) { descriptors.push_back(fmt::format("Innovation {} [m]", key)); }
            else if (std::holds_alternative<RTK::Meas::CarrierDD>(key)) { descriptors.push_back(fmt::format("Innovation {} [m]", key)); }
            else if (std::holds_alternative<RTK::Meas::DopplerDD>(key)) { descriptors.push_back(fmt::format("Innovation {} [m/s]", key)); }
            else if (std::holds_alternative<RTK::States::AmbiguityDD>(key)) { descriptors.push_back(fmt::format("Innovation {} [cyc]", key)); }
        }
        for (const auto& [satId, satData] : satData)
        {
            descriptors.push_back(fmt::format("{} Elevation [deg]", satId));
            descriptors.push_back(fmt::format("{} Azimuth [deg]", satId));
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        for (const auto& ambDD : ambiguityDD_br)
        {
            if (descriptor == fmt::format("AmbDD {} [cycles]", ambDD.satSigId)) { return ambDD.value.value; }
            if (descriptor == fmt::format("AmbDD StDev {} [cycles]", ambDD.satSigId)) { return ambDD.value.stdDev; }
        }
        for (const auto& key : measInnovation.rowKeys())
        {
            if (descriptor.starts_with(fmt::format("Innovation {}", key))) { return measInnovation(key); }
        }
        for (const auto& [satId, satData] : satData)
        {
            if (descriptor == fmt::format("{} Elevation [deg]", satId)) { return rad2deg(satData.satElevation); }
            if (descriptor == fmt::format("{} Azimuth [deg]", satId)) { return rad2deg(satData.satAzimuth); }
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;
        dynData.reserve(ambiguityDD_br.size() * 2 + static_cast<size_t>(measInnovation.rows()));
        for (const auto& ambDD : ambiguityDD_br)
        {
            dynData.emplace_back(fmt::format("AmbDD {} [cycles]", ambDD.satSigId), ambDD.value.value);
            dynData.emplace_back(fmt::format("AmbDD StDev {} [cycles]", ambDD.satSigId), ambDD.value.stdDev);
        }
        for (const auto& key : measInnovation.rowKeys())
        {
            if (std::holds_alternative<RTK::Meas::PsrDD>(key)) { dynData.emplace_back(fmt::format("Innovation {} [m]", key), measInnovation(key)); }
            else if (std::holds_alternative<RTK::Meas::CarrierDD>(key)) { dynData.emplace_back(fmt::format("Innovation {} [m]", key), measInnovation(key)); }
            else if (std::holds_alternative<RTK::Meas::DopplerDD>(key)) { dynData.emplace_back(fmt::format("Innovation {} [m/s]", key), measInnovation(key)); }
            else if (std::holds_alternative<RTK::States::AmbiguityDD>(key)) { dynData.emplace_back(fmt::format("Innovation {} [cyc]", key), measInnovation(key)); }
        }
        for (const auto& [satId, satData] : satData)
        {
            dynData.emplace_back(fmt::format("{} Elevation [deg]", satId), rad2deg(satData.satElevation));
            dynData.emplace_back(fmt::format("{} Azimuth [deg]", satId), rad2deg(satData.satAzimuth));
        }
        return dynData;
    }

    /// @brief Shows a GUI tooltip to look into details of the observation
    /// @param[in] detailView Flag to show the detailed view
    /// @param[in] firstOpen Flag whether the tooltip is opened once
    /// @param[in] displayName Data identifier, can be used in dynamic data to identify the correct data
    /// @param[in] id Unique identifier
    /// @param[in] rootWindow Pointer to the root window opening the tooltip
    void guiTooltip(bool detailView, bool firstOpen, const char* displayName, const char* id, int* rootWindow) const override;

    /// @brief Return whether this data has a tooltip
    [[nodiscard]] bool hasTooltip() const override { return true; }

    // --------------------------------------------------------- Public Members ------------------------------------------------------------

    /// Possible types of the RTK solution
    enum class SolutionType : uint8_t
    {
        None,      ///< No solution type specified
        SPP,       ///< Solution calculated via SPP algorithm because of missing data for RTK
        Predicted, ///< Only predicted by Kalman Filter
        RTK_Float, ///< RTK solution with floating point ambiguities
        RTK_Fixed, ///< RTK solution with fixed ambiguities to integers
    };

    /// Type of th solution
    SolutionType solType = SolutionType::None;

    /// Amount of satellites used
    size_t nSatellites = 0;

    /// Distance of Rover to base [m]
    double distanceBaseRover = 0.0;

    std::unordered_map<GnssObs::ObservationType, size_t> nObservations;                ///< Number of utilized observations (including pivot)
    std::unordered_map<GnssObs::ObservationType, size_t> nObservationsUniqueSatellite; ///< Number of utilized observations (counted once for each satellite)

    std::optional<size_t> nAmbiguitiesFixed; ///< Number of Ambiguities fixed

    /// Cycle slip detector results and name of the receiver
    std::vector<std::pair<CycleSlipDetector::Result, std::string>> cycleSlipDetectorResult;

    /// Pivot Change information
    struct PivotChange
    {
        /// Possible reasons for a pivot change
        enum class Reason : uint8_t
        {
            None,                    ///< No reason selected yet
            NewCode,                 ///< Code was not observed before
            PivotNotObservedInEpoch, ///< Old pivot satellite was not observed this epoch
            PivotCycleSlip,          ///< The pivot satellite had a cycle-slip
            HigherElevationFound,    ///< A satellite with higher elevation was observed
            PivotOutlier,            ///< Old pivot satellite was flagged as outlier
        };

        Reason reason = Reason::NewCode;                                   ///< Reason
        GnssObs::ObservationType obsType = GnssObs::ObservationType_COUNT; ///< Observation type

        SatSigId oldPivotSat;           ///< Old SatSig identifier
        double oldPivotElevation = 0.0; ///< Old Satellite elevation [rad]

        SatSigId newPivotSat;           ///< New SatSig identifier
        double newPivotElevation = 0.0; ///< New Satellite elevation [rad]
    };

    /// List of pivot satellite changes
    std::unordered_map<std::pair<Code, GnssObs::ObservationType>, PivotChange> changedPivotSatellites;

    /// Observable
    struct Observable
    {
        /// @brief Constructor
        /// @param[in] satSigId Satellite Signal Id
        /// @param[in] obsType Observation Type
        Observable(SatSigId satSigId, GnssObs::ObservationType obsType)
            : satSigId(satSigId), obsType(obsType) {}

        SatSigId satSigId;                                                 ///< Satellite Signal Id
        GnssObs::ObservationType obsType = GnssObs::ObservationType_COUNT; ///< Observation Type

        /// @brief Less than comparison (needed for map)
        /// @param[in] rhs Right hand side of the operator
        /// @return True if lhs < rhs
        bool operator<(const Observable& rhs) const
        {
            return satSigId == rhs.satSigId ? obsType < rhs.obsType
                                            : satSigId < rhs.satSigId;
        }
    };

    /// List of pivot satellites
    std::multiset<Observable> pivots;
    /// Observables available from receivers (only if double diff possible)
    std::multiset<Observable> observableReceived;
    /// Observables available from receivers, but filtered by GUI settings
    std::multiset<Observable> observableFiltered;
    /// Observables used for the final solution
    std::multiset<Observable> observableUsed;

    /// Signals filtered by the observation filter
    ObservationFilter::Filtered filtered;

    /// Outlier information
    struct Outlier
    {
        /// Outlier Type
        enum class Type : uint8_t
        {
            None, ///< None
            NIS,  ///< Normalized Innovation Squared (NIS)
        };

        /// @brief Constructor
        /// @param[in] type Outlier Type
        /// @param[in] satSigId Satellite Signal Id
        /// @param[in] obsType Observation Type
        Outlier(const Type& type, const SatSigId& satSigId, const GnssObs::ObservationType& obsType)
            : type(type), satSigId(satSigId), obsType(obsType) {}

        Type type = Type::None;                                            ///< Outlier Type
        SatSigId satSigId;                                                 ///< Satellite Signal Id
        GnssObs::ObservationType obsType = GnssObs::ObservationType_COUNT; ///< Observation Type
    };

    /// List of found outliers
    std::vector<Outlier> outliers;
    /// Normalized Innovation Squared (NIS) test result (before removing anything)
    std::optional<KeyedKalmanFilter<double, RTK::States::StateKeyType, RTK::Meas::MeasKeyTypes>::NISResult> nisResultInitial;
    /// Normalized Innovation Squared (NIS) test result (last NIS iteration)
    std::optional<KeyedKalmanFilter<double, RTK::States::StateKeyType, RTK::Meas::MeasKeyTypes>::NISResult> nisResultFinal;
    /// Amount of observations removed by NIS
    size_t nisRemovedCnt = 0;

    /// Ambiguity double differences
    struct AmbiguityDD
    {
        SatSigId pivotSatSigId = SatSigId(Code::None, 0);                                     ///< Pivot satellite Signal Id
        SatSigId satSigId = SatSigId(Code::None, 0);                                          ///< Satellite Signal id
        UncertainValue<double> value = UncertainValue<double>{ .value = 0.0, .stdDev = 0.0 }; ///< Value
    };

    /// Newly estimated ambiguities
    std::vector<SatSigId> newEstimatedAmbiguity;
    /// @brief Double differenced ambiguities
    std::vector<AmbiguityDD> ambiguityDD_br;

    /// 𝐳 Measurement vector
    KeyedVectorXd<RTK::Meas::MeasKeyTypes> measInnovation;

    /// Satellite specific data
    struct SatData
    {
        double satElevation = 0.0; ///< Satellite Elevation [rad]
        double satAzimuth = 0.0;   ///< Satellite Azimuth [rad]
    };

    /// Extended data for each satellite
    std::vector<std::pair<SatId, SatData>> satData;

  private:
    /// @brief Print a table for the satellites
    /// @param[in] satsReceived List of received satellites
    /// @param[in] id Unique identifier
    void guiTooltipSatellites(const std::map<SatelliteSystem, std::unordered_set<SatId>>& satsReceived, const char* id) const;

    /// @brief Print an observation table to the GUI
    /// @param[in] observables Observables
    /// @param[in] showSatCounts Whether to show the observable count in the table header
    /// @param[in] colorPivots Whether to color the pivot satellite
    /// @param[in] colorNotUsed Whether to color observations not used
    /// @param[in] colorCycleSlips Whether to color cycle-slips
    /// @param[in] colorPivotChanges Whether to color pivot changes
    /// @param[in] id Unique identifier
    void guiTooltipObservationTable(const std::multiset<RtkSolution::Observable>& observables,
                                    bool showSatCounts,
                                    bool colorPivots,
                                    bool colorNotUsed,
                                    bool colorCycleSlips,
                                    bool colorPivotChanges,
                                    const char* id) const;

    /// @brief Print a table for the ambiguities
    /// @param[in] id Unique identifier
    void guiTooltipAmbiguities(const char* id) const;
};

} // namespace NAV

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RtkSolution::SolutionType> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] solType Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RtkSolution::SolutionType& solType, FormatContext& ctx) const
    {
        switch (solType)
        {
        case NAV::RtkSolution::SolutionType::None:
            return fmt::formatter<const char*>::format("None", ctx);
        case NAV::RtkSolution::SolutionType::SPP:
            return fmt::formatter<const char*>::format("SPP", ctx);
        case NAV::RtkSolution::SolutionType::Predicted:
            return fmt::formatter<const char*>::format("Predicted", ctx);
        case NAV::RtkSolution::SolutionType::RTK_Float:
            return fmt::formatter<const char*>::format("Float", ctx);
        case NAV::RtkSolution::SolutionType::RTK_Fixed:
            return fmt::formatter<const char*>::format("Fixed", ctx);
        }
        return ctx.out();
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RtkSolution::Outlier::Type> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] outlierType Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RtkSolution::Outlier::Type& outlierType, FormatContext& ctx) const
    {
        switch (outlierType)
        {
        case NAV::RtkSolution::Outlier::Type::None:
            return fmt::formatter<const char*>::format("None", ctx);
        case NAV::RtkSolution::Outlier::Type::NIS:
            return fmt::formatter<const char*>::format("NIS check", ctx);
        }
        return ctx.out();
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RtkSolution::PivotChange> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] pivot Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RtkSolution::PivotChange& pivot, FormatContext& ctx) const
    {
        switch (pivot.reason)
        {
        case NAV::RtkSolution::PivotChange::Reason::None:
            return fmt::formatter<std::string>::format("Pivot change reason is unknown", ctx);
        case NAV::RtkSolution::PivotChange::Reason::PivotNotObservedInEpoch:
            return fmt::formatter<std::string>::format(fmt::format("Pivot change [{}]: [{}] -> [{}]\n"
                                                                   "Old pivot not observed this epoch\n"
                                                                   "New pivot elevation {:.4}°",
                                                                   pivot.obsType, pivot.oldPivotSat, pivot.newPivotSat,
                                                                   NAV::rad2deg(pivot.newPivotElevation)),
                                                       ctx);
        case NAV::RtkSolution::PivotChange::Reason::PivotCycleSlip:
            return fmt::formatter<std::string>::format(fmt::format("Pivot change [{}]: [{}] -> [{}]\n"
                                                                   "Old pivot had cycle-slip\n"
                                                                   "Elevation {:.4}° -> {:.4}°",
                                                                   pivot.obsType, pivot.oldPivotSat, pivot.newPivotSat,
                                                                   NAV::rad2deg(pivot.oldPivotElevation),
                                                                   NAV::rad2deg(pivot.newPivotElevation)),
                                                       ctx);
        case NAV::RtkSolution::PivotChange::Reason::HigherElevationFound:
            return fmt::formatter<std::string>::format(fmt::format("Pivot change [{}]: [{}] -> [{}]\n"
                                                                   "Satellite with higher elevation found\n"
                                                                   "Elevation {:.4}° -> {:.4}°",
                                                                   pivot.obsType, pivot.oldPivotSat, pivot.newPivotSat,
                                                                   NAV::rad2deg(pivot.oldPivotElevation),
                                                                   NAV::rad2deg(pivot.newPivotElevation)),
                                                       ctx);
        case NAV::RtkSolution::PivotChange::Reason::NewCode:
            return fmt::formatter<std::string>::format(fmt::format("New pivot [{}][{}]\n"
                                                                   "Elevation {:.4}°",
                                                                   pivot.newPivotSat, pivot.obsType,
                                                                   NAV::rad2deg(pivot.newPivotElevation)),
                                                       ctx);
        case NAV::RtkSolution::PivotChange::Reason::PivotOutlier:
            return fmt::formatter<std::string>::format(fmt::format("Pivot change [{}]: [{}] -> [{}]\n"
                                                                   "Old pivot flagged as outlier\n"
                                                                   "Elevation {:.4}°",
                                                                   pivot.obsType, pivot.oldPivotSat, pivot.newPivotSat,
                                                                   NAV::rad2deg(pivot.newPivotElevation)),
                                                       ctx);
        }
        return ctx.out();
    }
};

#endif