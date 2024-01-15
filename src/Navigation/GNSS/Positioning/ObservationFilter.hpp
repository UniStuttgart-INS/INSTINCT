// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ObservationFilter.hpp
/// @brief Observation Filter
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#pragma once

#include <unordered_set>
#include <utility>
#include <array>
#include <vector>

#include <imgui.h>
#include "internal/gui/widgets/imgui_ex.hpp"

#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/Receiver.hpp"
#include "Navigation/GNSS/SNRMask.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

#include "util/Container/STL.hpp"
#include "util/Json.hpp"

namespace NAV
{

/// Observation Filter
class ObservationFilter
{
  public:
    /// @brief Constructor
    /// @param[in] receiverCount Number of receivers
    /// @param[in] availableObsTypes Available observation types (e.g. SPP does not have Carrier)
    /// @param[in] neededObsTypes Needed observation types (cannot be unchecked)
    explicit ObservationFilter(size_t receiverCount,
                               const std::unordered_set<GnssObs::ObservationType>& availableObsTypes = { GnssObs::Pseudorange, GnssObs::Carrier, GnssObs::Doppler },
                               std::unordered_set<GnssObs::ObservationType> neededObsTypes = {})
        : _snrMask(receiverCount), _availableObsTypes(availableObsTypes), _neededObsTypes(std::move(neededObsTypes)), _usedObsTypes(availableObsTypes) {}

    /// @brief Destructor
    ~ObservationFilter() = default;
    /// @brief Copy constructor
    ObservationFilter(const ObservationFilter& other) = default;
    /// @brief Move constructor
    ObservationFilter(ObservationFilter&& other) noexcept = default;
    /// @brief Copy assignment operator
    ObservationFilter& operator=(const ObservationFilter& other)
    {
        if (this != &other) // not a self-assignment
        {
            _filterFreq = other._filterFreq;
            _filterCode = other._filterCode;
            _excludedSatellites = other._excludedSatellites;
            _elevationMask = other._elevationMask;
            _snrMask = other._snrMask;
            _sameSnrMaskForAllReceivers = other._sameSnrMaskForAllReceivers;
            _neededObsTypes = other._neededObsTypes;
            _usedObsTypes = other._usedObsTypes;
            std::vector<GnssObs::ObservationType> obsTypeToRemove;
            for (const auto& obsType : _usedObsTypes)
            {
                if (!_availableObsTypes.contains(obsType)) { obsTypeToRemove.push_back(obsType); }
            }
            for (const auto& obsType : obsTypeToRemove)
            {
                _usedObsTypes.erase(obsType);
            }
        }
        return *this;
    }
    /// @brief Move assignment operator
    ObservationFilter& operator=(ObservationFilter&& other) noexcept
    {
        if (this != &other) // not a self-assignment
        {
            _filterFreq = other._filterFreq;
            _filterCode = other._filterCode;
            _excludedSatellites = std::move(other._excludedSatellites);
            _elevationMask = other._elevationMask;
            _snrMask = std::move(other._snrMask);
            _sameSnrMaskForAllReceivers = other._sameSnrMaskForAllReceivers;
            _neededObsTypes = std::move(other._neededObsTypes);
            _usedObsTypes = std::move(other._usedObsTypes);
            std::vector<GnssObs::ObservationType> obsTypeToRemove;
            for (const auto& obsType : _usedObsTypes)
            {
                if (!_availableObsTypes.contains(obsType)) { obsTypeToRemove.push_back(obsType); }
            }
            for (const auto& obsType : obsTypeToRemove)
            {
                _usedObsTypes.erase(obsType);
            }
        }
        return *this;
    }

    /// @brief Returns a list of satellites and observations filtered by GUI settings & NAV data available & ...)
    /// @param[in] receivers List of receivers
    /// @param[in] gnssNavInfos Collection of navigation data providers
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @param[in] receiverUsedForOrbitCalculation Index of the receiver used for the orbit calculations
    /// @param[in] ignoreElevationMask Flag wether the elevation mask should be ignored
    /// @return 0: List of satellite data; 1: List of observations
    template<typename ReceiverType>
    [[nodiscard]] Observations selectObservationsForCalculation(const std::array<Receiver<ReceiverType>, ReceiverType::ReceiverType_COUNT>& receivers,
                                                                const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                                [[maybe_unused]] const std::string& nameId,
                                                                size_t receiverUsedForOrbitCalculation,
                                                                bool ignoreElevationMask = false)
    {
        Observations observations;
        observations.signals.reserve(receivers.front().gnssObs->data.size());

        std::array<std::unordered_set<SatId>, GnssObs::ObservationType_COUNT> nMeasUniqueSat;

        for (const auto& obsData : receivers.front().gnssObs->data)
        {
            SatId satId = obsData.satSigId.toSatId();

            if (!(obsData.satSigId.freq() & _filterFreq)                                                                  // frequency is not selected in GUI
                || !(obsData.satSigId.code & _filterCode)                                                                 // code is not selected in GUI
                || !obsData.pseudorange                                                                                   // has an invalid pseudorange
                || std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) != _excludedSatellites.end()) // is excluded
            {
                LOG_DATA("{}:  [{}] Skipping obs due to GUI filter selections", nameId, obsData.satSigId);
                continue;
            }

            LOG_DATA("{}:  [{}] Searching if observation in all receivers", nameId, obsData.satSigId);
            unordered_map<GnssObs::ObservationType, size_t> availableObservations;
            if (std::any_of(receivers.begin(), receivers.end(),
                            [&](const Receiver<ReceiverType>& recv) {
                                auto obsDataOther = std::find_if(recv.gnssObs->data.begin(), recv.gnssObs->data.end(),
                                                                 [&obsData](const GnssObs::ObservationData& obsDataOther) {
                                                                     return obsDataOther.satSigId == obsData.satSigId;
                                                                 });

                                if (obsDataOther == recv.gnssObs->data.end())
                                {
                                    LOG_DATA("{}:  [{}] Receiver '{}' did not observe the signal.", nameId, obsData.satSigId, recv.type);
                                    return true;
                                } // All receivers must have this observation

                                for (const auto& obsType : _usedObsTypes)
                                {
                                    switch (obsType)
                                    {
                                    case GnssObs::Pseudorange:
                                        if (obsDataOther->pseudorange) { availableObservations[obsType]++; }
                                        break;
                                    case GnssObs::Carrier:
                                        if (obsDataOther->carrierPhase) { availableObservations[obsType]++; }
                                        break;
                                    case GnssObs::Doppler:
                                        if (obsDataOther->doppler) { availableObservations[obsType]++; }
                                        break;
                                    case GnssObs::ObservationType_COUNT:
                                        break;
                                    }
                                }
                                if (!obsDataOther->pseudorange)
                                {
                                    LOG_DATA("{}:   [{}]   Receiver '{}' did not have a pseudorange observation.", nameId, obsData.satSigId, recv.type);
                                }
                                return !obsDataOther->pseudorange; // Pseudorange must be available for all receivers
                            }))
            {
                continue;
            }
            LOG_DATA("{}:  [{}]   Observed psr {}x, carrier {}x, doppler {}x", nameId, obsData.satSigId,
                     availableObservations.contains(GnssObs::Pseudorange) ? availableObservations.at(GnssObs::Pseudorange) : 0,
                     availableObservations.contains(GnssObs::Carrier) ? availableObservations.at(GnssObs::Carrier) : 0,
                     availableObservations.contains(GnssObs::Doppler) ? availableObservations.at(GnssObs::Doppler) : 0);

            std::shared_ptr<NAV::SatNavData> satNavData = nullptr;
            for (const auto& gnssNavInfo : gnssNavInfos)
            {
                auto satNav = gnssNavInfo->searchNavigationData(satId, receivers.front().gnssObs->insTime);
                if (satNav && satNav->isHealthy())
                {
                    satNavData = satNav;
                    break;
                }
            }
            if (satNavData == nullptr) { continue; } // can calculate satellite position

            int8_t freqNum = -128;
            if (satId.satSys == GLO)
            {
                if (auto gloSatNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(satNavData))
                {
                    freqNum = gloSatNavData->frequencyNumber;
                }
            }

            auto recvObsData = std::find_if(receivers.at(receiverUsedForOrbitCalculation).gnssObs->data.begin(),
                                            receivers.at(receiverUsedForOrbitCalculation).gnssObs->data.end(),
                                            [&obsData](const GnssObs::ObservationData& recvObsData) {
                                                return recvObsData.satSigId == obsData.satSigId;
                                            });
            auto satClk = satNavData->calcClockCorrections(receivers.at(receiverUsedForOrbitCalculation).gnssObs->insTime,
                                                           recvObsData->pseudorange->value,
                                                           recvObsData->satSigId.freq());
            auto satPosVel = satNavData->calcSatellitePosVel(satClk.transmitTime);

            Observations::SignalObservation sigObs(satNavData, satPosVel.e_pos, satPosVel.e_vel, satClk, freqNum);

            bool skipObservation = false;
            for (const auto& recv : receivers)
            {
                auto recvObsData = std::find_if(recv.gnssObs->data.begin(), recv.gnssObs->data.end(),
                                                [&obsData](const GnssObs::ObservationData& recvObsData) {
                                                    return recvObsData.satSigId == obsData.satSigId;
                                                });

                LOG_DATA("{}: Adding satellite [{}] for receiver {}", nameId, obsData.satSigId, recv.type);
                sigObs.recvObs.emplace_back(recv.gnssObs, static_cast<size_t>(recvObsData - recv.gnssObs->data.begin()),
                                            recv.e_pos, recv.lla_pos, recv.e_vel,
                                            satPosVel.e_pos, satPosVel.e_vel);

                if (!ignoreElevationMask)
                {
                    const auto& satElevation = sigObs.recvObs.back().satElevation();
                    if (satElevation < _elevationMask)
                    {
                        LOG_DATA("{}: Signal {} is skipped because of elevation mask. ({} < {})", nameId, obsData.satSigId,
                                 rad2deg(satElevation), rad2deg(_elevationMask));
                        skipObservation = true;
                        break;
                    }
                    if (recvObsData->CN0 // If no CN0 available, we use the signal
                        && !_snrMask
                                .at(_sameSnrMaskForAllReceivers ? static_cast<ReceiverType>(0) : recv.type)
                                .checkSNRMask(obsData.satSigId.freq(), satElevation, recvObsData->CN0.value()))
                    {
                        LOG_DEBUG("{}: [{}] SNR mask triggered for [{}] on receiver [{}] with CN0 {} dbHz",
                                  nameId, receivers.front().gnssObs->insTime.toYMDHMS(GPST), obsData.satSigId, recv.type, recvObsData->CN0.value());
                        skipObservation = true;
                        break;
                    }
                }
            }
            if (skipObservation) { continue; }

            for (const auto& recv : receivers)
            {
                auto& recvObsData = sigObs.recvObs.at(recv.type);

                for (const auto& obsType : _usedObsTypes)
                {
                    if (availableObservations.contains(obsType) && availableObservations.at(obsType) == receivers.size())
                    {
                        LOG_DATA("{}:  [{}]   Taking {} observation into account on {} receiver", nameId, obsData.satSigId, obsType, recv.type);
                        observations.nObservables.at(obsType)++;
                        nMeasUniqueSat.at(obsType).insert(satId);
                        switch (obsType)
                        {
                        case GnssObs::Pseudorange:
                            recvObsData.obs[obsType].measurement = recvObsData.gnssObsData().pseudorange->value;
                            break;
                        case GnssObs::Carrier:
                            recvObsData.obs[obsType].measurement = InsConst::C / obsData.satSigId.freq().getFrequency(freqNum)
                                                                   * recvObsData.gnssObsData().carrierPhase->value;
                            break;
                        case GnssObs::Doppler:
                            recvObsData.obs[obsType].measurement = doppler2rangeRate(recvObsData.gnssObsData().doppler.value(),
                                                                                     obsData.satSigId.freq(),
                                                                                     freqNum);
                            break;
                        case GnssObs::ObservationType_COUNT:
                            break;
                        }
                    }
                }
            }

            observations.systems.insert(satId.satSys);
            observations.satellites.insert(satId);
            observations.signals.insert(std::make_pair(obsData.satSigId, sigObs));
        }

        for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
        {
            observations.nObservablesUniqueSatellite.at(obsType) = nMeasUniqueSat.at(obsType).size();
        }

#if LOG_LEVEL <= LOG_LEVEL_DATA
        LOG_DATA("{}: usedSatSystems = [{}]", nameId, joinToString(observations.systems));
        size_t nMeasTotal = 0;
        std::string nMeasStr;
        for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
        {
            auto& nMeas = observations.nObservables.at(obsType);
            nMeas /= receivers.size();
            nMeasStr += fmt::format("{} {}, ", nMeas, static_cast<GnssObs::ObservationType>(obsType));
            nMeasTotal += nMeas;
        }
        if (nMeasStr.ends_with(", ")) { nMeasStr = nMeasStr.erase(nMeasStr.length() - 2); }

        LOG_DATA("{}: Using {} measurements ({}) from {} satellites", nameId, nMeasTotal, nMeasStr, observations.satellites.size());

        unordered_map<SatId, std::pair<Frequency, Code>> satData;
        for (const auto& obs : observations.signals)
        {
            satData[obs.first.toSatId()].first |= obs.first.freq();
            satData[obs.first.toSatId()].second |= obs.first.code;
        }
        for (const auto& [satId, freqCode] : satData)
        {
            LOG_DATA("{}:   [{}] on frequencies [{}] with codes [{}]", nameId, satId, freqCode.first, freqCode.second);
        }
#endif

        return observations;
    }

    /// @brief Shows the GUI input to select the options
    /// @param[in] id Unique id for ImGui.
    /// @param[in] itemWidth Width of the widgets
    template<typename ReceiverType>
    bool ShowGuiWidgets(const char* id, float itemWidth)
    {
        bool changed = false;

        ImGui::SetNextItemWidth(itemWidth);
        if (ShowFrequencySelector(fmt::format("Satellite Frequencies##{}", id).c_str(), _filterFreq))
        {
            changed = true;
        }

        ImGui::SetNextItemWidth(itemWidth);
        if (ShowCodeSelector(fmt::format("Signal Codes##{}", id).c_str(), _filterCode, _filterFreq))
        {
            changed = true;
        }

        ImGui::SetNextItemWidth(itemWidth);
        if (ShowSatelliteSelector(fmt::format("Excluded satellites##{}", id).c_str(), _excludedSatellites))
        {
            changed = true;
        }

        double elevationMaskDeg = rad2deg(_elevationMask);
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("Elevation mask##{}", id).c_str(), &elevationMaskDeg, 0.0, 90.0, 5.0, 5.0, "%.1f°", ImGuiInputTextFlags_AllowTabInput))
        {
            _elevationMask = deg2rad(elevationMaskDeg);
            LOG_DEBUG("{}: Elevation mask changed to {}°", id, elevationMaskDeg);
            changed = true;
        }

        for (size_t i = 0; i < ReceiverType::ReceiverType_COUNT; ++i)
        {
            if (i != 0)
            {
                ImGui::SameLine();
                if (_sameSnrMaskForAllReceivers) { ImGui::BeginDisabled(); }
            }
            if (_snrMask.at(i).ShowGuiWidgets(fmt::format("{} SNR Mask", static_cast<ReceiverType>(i)).c_str()))
            {
                changed = true;
            }
            if (i != 0 && _sameSnrMaskForAllReceivers) { ImGui::EndDisabled(); }
        }
        if (ReceiverType::ReceiverType_COUNT > 1)
        {
            ImGui::SameLine();
            if (ImGui::Checkbox(fmt::format("Use same SNR for all receivers##{}", id).c_str(), &_sameSnrMaskForAllReceivers))
            {
                changed = true;
            }
        }

        ImGui::BeginHorizontal(fmt::format("Observables##{}", id).c_str(),
                               ImVec2(itemWidth - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x, 0.0F));
        for (size_t i = 0; i < GnssObs::ObservationType_COUNT; i++)
        {
            auto obsType = static_cast<GnssObs::ObservationType>(i);
            if (!_availableObsTypes.contains(obsType)) { continue; }
            if (_neededObsTypes.contains(obsType)) { ImGui::BeginDisabled(); }
            bool enabled = _usedObsTypes.contains(obsType);
            if (ImGui::Checkbox(fmt::format("{}##{}", obsType, id).c_str(), &enabled))
            {
                LOG_DEBUG("{}: Using {}: {}", id, obsType, enabled);
                if (enabled) { _usedObsTypes.insert(obsType); }
                else { _usedObsTypes.erase(obsType); }
                changed = true;
            }
            if (_neededObsTypes.contains(obsType)) { ImGui::EndDisabled(); }
        }
        ImGui::EndHorizontal();

        ImGui::SameLine();
        ImGui::TextUnformatted("Used observables");

        return changed;
    }

    /// @brief Checks if the satellite is allowed. Does not check elevation or SNR mask
    /// @param[in] satId Satellite Identifier
    [[nodiscard]] bool isSatelliteAllowed(const SatId& satId) const
    {
        return (satId.satSys & _filterFreq)
               && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) == _excludedSatellites.end();
    }

    /// @brief Checks if the Observation type is used by the GUI settings
    /// @param[in] obsType Observation Type
    [[nodiscard]] bool isObsTypeUsed(GnssObs::ObservationType obsType) const
    {
        return _usedObsTypes.contains(obsType);
    }

    /// @brief Set the observation type to use
    /// @param obsType Observation Type
    void useObsType(GnssObs::ObservationType obsType)
    {
        _usedObsTypes.insert(obsType);
    }

    /// @brief Set the observation type as needed (cannot be unchecked in the GUI) or unneeded
    /// @param obsType Observation Type
    /// @param needed Needed or unneeded
    void markObsTypeAsNeeded(GnssObs::ObservationType obsType, bool needed = true)
    {
        if (needed) { _neededObsTypes.insert(obsType); }
        else if (_neededObsTypes.contains(obsType)) { _neededObsTypes.erase(obsType); }
    }

    /// @brief Get the Frequency Filter
    [[nodiscard]] const Frequency& getFrequencyFilter() const
    {
        return _filterFreq;
    }

    /// @brief Get the used observation types
    [[nodiscard]] const std::unordered_set<GnssObs::ObservationType>& getUsedObservationTypes() const
    {
        return _usedObsTypes;
    }

  private:
    /// Frequencies used for calculation (GUI filter)
    Frequency _filterFreq = G01 | G02 | G05
                            | E01 | E05 | E06 | E07 | E08;
    /// Codes used for calculation (GUI filter)
    Code _filterCode = Code_Default;
    /// List of satellites to exclude
    std::vector<SatId> _excludedSatellites;
    /// Elevation cut-off angle for satellites in [rad]
    double _elevationMask = static_cast<double>(10.0_deg);
    /// SNR Mask for all receivers
    std::vector<SNRMask> _snrMask;
    /// Flag wether to use the same SNR mask for all receivers
    bool _sameSnrMaskForAllReceivers = true;
    /// Available observation types (e.g. SPP does not have Carrier)
    const std::unordered_set<GnssObs::ObservationType> _availableObsTypes;
    /// Needed observation types (cannot be unchecked in GUI)
    std::unordered_set<GnssObs::ObservationType> _neededObsTypes;
    /// Utilized observations
    std::unordered_set<GnssObs::ObservationType> _usedObsTypes;

    /// @brief Converts the provided object into json
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] obj Object to convert into json
    friend void to_json(json& j, const ObservationFilter& obj)
    {
        j = json{
            { "frequencies", Frequency_(obj._filterFreq) },
            { "codes", obj._filterCode },
            { "excludedSatellites", obj._excludedSatellites },
            { "elevationMask", rad2deg(obj._elevationMask) },
            { "snrMask", obj._snrMask },
            { "sameSnrMaskForAllReceivers", obj._sameSnrMaskForAllReceivers },
            { "usedObsTypes", obj._usedObsTypes },
            { "neededObsType", obj._neededObsTypes },
        };
    }
    /// @brief Converts the provided json object into a node object
    /// @param[in] j Json object with the needed values
    /// @param[out] obj Object to fill from the json
    friend void from_json(const json& j, ObservationFilter& obj)
    {
        if (j.contains("frequencies"))
        {
            uint64_t value = 0;
            j.at("frequencies").get_to(value);
            obj._filterFreq = Frequency_(value);
        }
        if (j.contains("codes")) { j.at("codes").get_to(obj._filterCode); }
        if (j.contains("excludedSatellites")) { j.at("excludedSatellites").get_to(obj._excludedSatellites); }
        if (j.contains("elevationMask"))
        {
            j.at("elevationMask").get_to(obj._elevationMask);
            obj._elevationMask = deg2rad(obj._elevationMask);
        }
        if (j.contains("snrMask")) { j.at("snrMask").get_to(obj._snrMask); }
        if (j.contains("sameSnrMaskForAllReceivers")) { j.at("sameSnrMaskForAllReceivers").get_to(obj._sameSnrMaskForAllReceivers); }
        if (j.contains("usedObsTypes")) { j.at("usedObsTypes").get_to(obj._usedObsTypes); }
        if (j.contains("neededObsTypes")) { j.at("neededObsTypes").get_to(obj._neededObsTypes); }
    }
};

} // namespace NAV
