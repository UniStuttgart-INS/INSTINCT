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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <array>
#include <vector>

#include <imgui.h>
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/Receiver.hpp"
#include "Navigation/GNSS/SNRMask.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

#include "util/Assert.h"
#include "util/Container/STL.hpp"
#include "util/Json.hpp"
#include "util/Logger.hpp"
#include <fmt/core.h>

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
        : _snrMask(receiverCount), _availableObsTypes(availableObsTypes), _neededObsTypes(std::move(neededObsTypes)), _usedObsTypes(availableObsTypes)
    {
        // Disable Geostationary satellites, as they not working correctly
        for (const auto& satSys : SatelliteSystem::GetAll())
        {
            for (const auto& satNum : satSys.getSatellites())
            {
                if (SatId satId(satSys, satNum);
                    satId.isGeo()) { _excludedSatellites.push_back(satId); }
            }
        }
    }

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

    /// @brief Reset the temporary settings
    void reset()
    {
        _temporarilyExcludedSignalsSatellites.clear();
    }

    /// Filtered signals
    struct Filtered
    {
        std::vector<SatSigId> frequencyFilter;                           ///< Signals excluded because the frequency is not used
        std::vector<SatSigId> codeFilter;                                ///< Signals excluded because the code is not used
        std::vector<SatSigId> excludedSatellites;                        ///< Signals excluded because the satellite is excluded
        std::vector<SatSigId> tempExcludedSignal;                        ///< Signals temporarily excluded
        std::vector<SatSigId> notAllReceiversObserved;                   ///< Signals not observed by all receivers
        std::vector<SatSigId> noPseudorangeMeasurement;                  ///< Signals without pseudorange measurement
        std::vector<SatSigId> navigationDataMissing;                     ///< Signals without navigation data
        std::vector<std::pair<SatSigId, double>> elevationMaskTriggered; ///< Signals triggering the elevation mask. Also includes elevation [rad]
        std::vector<std::pair<SatSigId, double>> snrMaskTriggered;       ///< Signals triggering the SNR mask. Also includes the Carrier-to-Noise density [dBHz]
    };

    /// @brief Returns a list of satellites and observations filtered by GUI settings & NAV data available & ...)
    /// @param[in] receiverType Receiver type index to filter
    /// @param[in] e_posMarker Marker Position in ECEF frame [m]
    /// @param[in] lla_posMarker Marker Position in LLA frame [rad, rad, m]
    /// @param[in] gnssObs GNSS observation
    /// @param[in] gnssNavInfos Collection of navigation data providers
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @param[in] observations List of observations which will be filled. If you have multiple receivers, the observations list will be the same object
    /// @param[in] filtered Optional Filtered object to get back the filtered signals
    /// @param[in] ignoreElevationMask Flag wether the elevation mask should be ignored
    template<typename ReceiverType, typename DerivedPe, typename DerivedPn>
    void selectObservationsForCalculation(ReceiverType receiverType,
                                          const Eigen::MatrixBase<DerivedPe>& e_posMarker,
                                          const Eigen::MatrixBase<DerivedPn>& lla_posMarker,
                                          const std::shared_ptr<const GnssObs>& gnssObs,
                                          const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                          Observations& observations,
                                          Filtered* filtered,
                                          [[maybe_unused]] const std::string& nameId,
                                          bool ignoreElevationMask = false)
    {
        bool firstReceiver = observations.receivers.empty();
        observations.receivers.insert(receiverType);

        observations.signals.reserve(gnssObs->data.size());

        for (size_t obsIdx = 0; obsIdx < gnssObs->data.size(); obsIdx++)
        {
            const GnssObs::ObservationData& obsData = gnssObs->data.at(obsIdx);
            SatSigId satSigId = obsData.satSigId;
            SatId satId = satSigId.toSatId();
            LOG_DATA("{}: Considering [{}] for receiver {}", nameId, satSigId, receiverType);

            // Decrease the temporary exclude counter
            if (firstReceiver && _temporarilyExcludedSignalsSatellites.contains(satSigId))
            {
                if (_temporarilyExcludedSignalsSatellites.at(satSigId)-- == 0)
                {
                    _temporarilyExcludedSignalsSatellites.erase(satSigId);
                }
            }

            if (!(satSigId.freq() & _filterFreq))
            {
                LOG_DATA("{}:  [{}] Skipping obs due to GUI frequency filter", nameId, satSigId);
                if (filtered) { filtered->frequencyFilter.push_back(satSigId); }
                continue;
            }
            if (!(satSigId.code & _filterCode))
            {
                LOG_DATA("{}:  [{}] Skipping obs due to GUI code filter", nameId, satSigId);
                if (filtered) { filtered->codeFilter.push_back(satSigId); }
                continue;
            }
            if (std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) != _excludedSatellites.end())
            {
                LOG_DATA("{}:  [{}] Skipping obs due to GUI excluded satellites", nameId, satSigId);
                if (filtered) { filtered->excludedSatellites.push_back(satSigId); }
                continue;
            }
            if (_temporarilyExcludedSignalsSatellites.contains(satSigId))
            {
                LOG_DATA("{}:  [{}] Skipping obs because temporarily excluded signal", nameId, satSigId);
                if (filtered) { filtered->tempExcludedSignal.push_back(satSigId); }
                continue;
            }

            if (!obsData.pseudorange)
            {
                LOG_DATA("{}:  [{}] Skipping obs because no pseudorange measurement (needed for satellite position calculation)", nameId, satSigId);
                if (filtered) { filtered->noPseudorangeMeasurement.push_back(satSigId); }
                continue;
            }

            if (!firstReceiver && !observations.signals.contains(satSigId)) // TODO:
            {
                bool signalWithSameFrequencyFound = false;
                for (const auto& signals : observations.signals)
                {
                    if (signals.first.toSatId() == satId && signals.first.freq() == satSigId.freq() // e.g. Rover has [G5Q], but Base has [G5X]
                        && signals.second.recvObs.size() != observations.receivers.size())          // But not: Rover has [G5Q], but Base has [G5Q] and [G5X]
                    {
                        LOG_DATA("{}:  [{}] Not observed by all receivers, but other receivers have [{}]. Treating as such.",
                                 nameId, satSigId, signals.first);
                        satSigId = signals.first;
                        satId = satSigId.toSatId();
                        signalWithSameFrequencyFound = true;
                        break;
                    }
                }
                if (!signalWithSameFrequencyFound)
                {
                    LOG_DATA("{}:  [{}] Skipping obs because not observed by all receivers", nameId, satSigId);
                    if (filtered) { filtered->notAllReceiversObserved.push_back((satSigId)); }
                    continue;
                }
            }

            std::shared_ptr<NAV::SatNavData> satNavData = nullptr;
            for (const auto* gnssNavInfo : gnssNavInfos)
            {
                auto satNav = gnssNavInfo->searchNavigationData(satId, gnssObs->insTime);
                if (satNav && satNav->isHealthy())
                {
                    satNavData = satNav;
                    break;
                }
            }
            if (satNavData == nullptr)
            {
                LOG_DATA("{}:  [{}] Skipping obs because no navigation data available to calculaten the satellite position", nameId, satSigId);
                if (filtered) { filtered->navigationDataMissing.push_back(satSigId); }
                continue;
            }

            int8_t freqNum = -128;
            if (satId.satSys == GLO)
            {
                if (auto gloSatNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(satNavData))
                {
                    freqNum = gloSatNavData->frequencyNumber;
                }
            }

            auto satClk = satNavData->calcClockCorrections(gnssObs->insTime,
                                                           obsData.pseudorange->value,
                                                           satSigId.freq());
            auto satPosVel = satNavData->calcSatellitePosVel(satClk.transmitTime);

            auto recvData = std::make_shared<Observations::SignalObservation::ReceiverSpecificData>(
                gnssObs, obsIdx,
                satPosVel.e_pos, satPosVel.e_vel, satClk);

            if (!ignoreElevationMask)
            {
                const auto& satElevation = recvData->satElevation(e_posMarker, lla_posMarker);
                if (satElevation < _elevationMask)
                {
                    LOG_DATA("{}: Signal {} is skipped because of elevation mask. ({} < {})", nameId, satSigId,
                             rad2deg(satElevation), rad2deg(_elevationMask));
                    if (filtered) { filtered->elevationMaskTriggered.emplace_back(satSigId, satElevation); }
                    continue;
                }
                if (obsData.CN0 // If no CN0 available, we do not check the SNR mask, so we use the signal
                    && !_snrMask
                            .at(_sameSnrMaskForAllReceivers ? static_cast<ReceiverType>(0) : receiverType)
                            .checkSNRMask(satSigId.freq(), satElevation, obsData.CN0.value()))
                {
                    LOG_DATA("{}: [{}] SNR mask triggered for [{}] on receiver [{}] with CN0 {} dbHz",
                             nameId, gnssObs->insTime.toYMDHMS(GPST), satSigId, receiverType, *obsData.CN0);
                    if (filtered) { filtered->snrMaskTriggered.emplace_back(satSigId, *obsData.CN0); }
                    continue;
                }
            }

            for (const GnssObs::ObservationType& obsType : _usedObsTypes)
            {
                auto removeObsTypeIfExist = [&]() {
                    if (!observations.signals.contains(satSigId)) { return; }
                    std::for_each(observations.signals.at(satSigId).recvObs.begin(),
                                  observations.signals.at(satSigId).recvObs.end(),
                                  [&](auto& r) {
                                      if (r.second->obs.contains(obsType))
                                      {
                                          LOG_DATA("{}:  [{}] Erasing previously added obs '{}' on this signal.", nameId, satSigId, obsType);
                                          r.second->obs.erase(obsType);
                                      }
                                  });
                };

                if (!firstReceiver
                    && std::any_of(observations.signals.at(satSigId).recvObs.begin(),
                                   observations.signals.at(satSigId).recvObs.end(),
                                   [&](const auto& r) {
                                       return !r.second->obs.contains(obsType);
                                   }))
                {
                    LOG_DATA("{}:  [{}][{}] Skipping '{}' measurement. Not all receivers have this observation.", nameId, receiverType, satSigId, obsType);
                    removeObsTypeIfExist();
                    continue;
                }
                switch (obsType)
                {
                case GnssObs::Pseudorange:
                    if (recvData->gnssObsData().pseudorange)
                    {
                        recvData->obs[obsType].measurement = recvData->gnssObsData().pseudorange->value;
                        LOG_DATA("{}:  [{}] Taking {:11} observation into account on {:5} receiver ({:.3f} [m])", nameId, satSigId,
                                 obsType, receiverType, recvData->obs[obsType].measurement);
                    }
                    else { removeObsTypeIfExist(); }
                    break;
                case GnssObs::Carrier:
                    if (recvData->gnssObsData().carrierPhase)
                    {
                        recvData->obs[obsType].measurement = InsConst::C / satSigId.freq().getFrequency(freqNum)
                                                             * recvData->gnssObsData().carrierPhase->value;
                        LOG_DATA("{}:  [{}] Taking {:11} observation into account on {:5} receiver ({:.3f} [m] = {:.3f} [cycles])", nameId, satSigId,
                                 obsType, receiverType, recvData->obs[obsType].measurement, recvData->gnssObsData().carrierPhase->value);
                    }
                    else { removeObsTypeIfExist(); }
                    break;
                case GnssObs::Doppler:
                    if (recvData->gnssObsData().doppler)
                    {
                        recvData->obs[obsType].measurement = doppler2rangeRate(recvData->gnssObsData().doppler.value(),
                                                                               satSigId.freq(),
                                                                               freqNum);
                        LOG_DATA("{}:  [{}] Taking {:11} observation into account on {:5} receiver ({:.3f} [m/s] = {:.3f} [Hz])", nameId, satSigId,
                                 obsType, receiverType, recvData->obs[obsType].measurement, recvData->gnssObsData().doppler.value());
                    }
                    else { removeObsTypeIfExist(); }
                    break;
                case GnssObs::ObservationType_COUNT:
                    break;
                }
            }

            if (!firstReceiver
                && std::any_of(observations.signals.at(satSigId).recvObs.begin(),
                               observations.signals.at(satSigId).recvObs.end(),
                               [&](const auto& r) {
                                   return r.second->obs.empty();
                               }))
            {
                LOG_DATA("{}:  [{}] Skipping obs because not observed by all receivers", nameId, satSigId);
                if (filtered) { filtered->notAllReceiversObserved.push_back(satSigId); }
                observations.signals.erase(satSigId);
                continue;
            }
            LOG_DATA("{}: Adding satellite [{}] for receiver {}", nameId, satSigId, receiverType);
            if (!observations.signals.contains(satSigId))
            {
                observations.signals.insert(std::make_pair(satSigId,
                                                           Observations::SignalObservation{ satNavData, freqNum }));
            }
            observations.signals.at(satSigId).recvObs.emplace(receiverType, recvData);
        }
        std::vector<SatSigId> sigToRemove;
        for (const auto& [satSigId, sigObs] : observations.signals)
        {
            if (sigObs.recvObs.size() != observations.receivers.size())
            {
                sigToRemove.push_back(satSigId);
            }
        }
        for (const auto& satSigId : sigToRemove)
        {
            LOG_DATA("{}: [{}] Removing signal because not observed by all receivers.", nameId, satSigId);
            if (filtered) { filtered->notAllReceiversObserved.push_back(satSigId); }
            observations.signals.erase(satSigId);
        }

        observations.recalcObservableCounts();

#if LOG_LEVEL <= LOG_LEVEL_DATA
        LOG_DATA("{}: usedSatSystems = [{}]", nameId, joinToString(observations.systems));
        size_t nMeasTotal = 0;
        std::string nMeasStr;
        for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
        {
            auto& nMeas = observations.nObservables.at(obsType);
            nMeasStr += fmt::format("{} {}, ", nMeas, static_cast<GnssObs::ObservationType>(obsType));
            nMeasTotal += nMeas;
        }
        if (nMeasStr.ends_with(", ")) { nMeasStr = nMeasStr.erase(nMeasStr.length() - 2); }

        LOG_DATA("{}: Using {} measurements ({}) from {} satellites", nameId, nMeasTotal, nMeasStr, observations.satellites.size());

        unordered_map<SatId, std::pair<Frequency, Code>> satData;
        unordered_map<SatSigId, std::set<GnssObs::ObservationType>> sigData;
        for (const auto& obs : observations.signals)
        {
            satData[obs.first.toSatId()].first |= obs.first.freq();
            satData[obs.first.toSatId()].second |= obs.first.code;
            for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
            {
                if (std::all_of(obs.second.recvObs.begin(), obs.second.recvObs.end(), [&obsType](const auto& recvObs) {
                        return recvObs.second->obs.contains(static_cast<GnssObs::ObservationType>(obsType));
                    }))
                {
                    sigData[obs.first].insert(static_cast<GnssObs::ObservationType>(obsType));
                }
            }
        }
        for ([[maybe_unused]] const auto& [satId, freqCode] : satData)
        {
            LOG_DATA("{}:   [{}] on frequencies [{}] with codes [{}]", nameId, satId, freqCode.first, freqCode.second);
            for (const auto& [satSigId, obs] : sigData)
            {
                if (satSigId.toSatId() != satId) { continue; }
                std::string str;
                for (const auto& o : obs)
                {
                    if (!str.empty()) { str += ", "; }
                    str += fmt::format("{}", o);
                }
                LOG_DATA("{}:       [{}] has obs: {}", nameId, satSigId.code, str);
            }
        }
#endif
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

        for (size_t i = 0; i < _snrMask.size(); ++i)
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
        if (_snrMask.size() > 1)
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

    /// @brief Temporarily excludes a signal
    /// @param[in] satSigId Satellite Signal Id
    /// @param[in] count Amount of function calls to exclude
    void excludeSignalTemporarily(const SatSigId& satSigId, size_t count)
    {
        if (count == 0) { return; }
        _temporarilyExcludedSignalsSatellites.emplace(satSigId, count);
    }

    /// @brief Get the Frequency Filter
    [[nodiscard]] const Frequency& getFrequencyFilter() const
    {
        return _filterFreq;
    }
    /// @brief Get the Code Filter
    [[nodiscard]] const Code& getCodeFilter() const
    {
        return _filterCode;
    }

    /// @brief Get the Satellite System Filter
    [[nodiscard]] SatelliteSystem getSystemFilter() const
    {
        return _filterFreq.getSatSys();
    }

    /// @brief Get the used observation types
    [[nodiscard]] const std::unordered_set<GnssObs::ObservationType>& getUsedObservationTypes() const
    {
        return _usedObsTypes;
    }

    /// @brief Opens all settings to the maximum, disabling the filter
    void disableFilter()
    {
        for (const auto& freq : Frequency::GetAll()) { _filterFreq |= freq; }
        _filterCode = Code_ALL;
        _excludedSatellites.clear();
        _elevationMask = 0.0;
        for (auto& snrMask : _snrMask) { snrMask.disable(); }
        _usedObsTypes = { GnssObs::Pseudorange, GnssObs::Carrier, GnssObs::Doppler };
        _temporarilyExcludedSignalsSatellites.clear();
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

    /// List of signals to exclude temporarily
    std::unordered_map<SatSigId, size_t> _temporarilyExcludedSignalsSatellites;

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
        if (j.contains("excludedSatellites"))
        {
            j.at("excludedSatellites").get_to(obj._excludedSatellites);
            // Disable Geostationary satellites, as they not working correctly
            for (const auto& satSys : SatelliteSystem::GetAll())
            {
                for (const auto& satNum : satSys.getSatellites())
                {
                    if (SatId satId(satSys, satNum);
                        satId.isGeo()
                        && std::find(obj._excludedSatellites.begin(),
                                     obj._excludedSatellites.end(),
                                     satId)
                               == obj._excludedSatellites.end())
                    {
                        obj._excludedSatellites.push_back(satId);
                    }
                }
            }
        }
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
