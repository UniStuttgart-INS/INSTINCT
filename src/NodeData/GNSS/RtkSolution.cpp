// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtkSolution.cpp
/// @brief RTK Solution data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-07-17

#include "RtkSolution.hpp"
#include <algorithm>
#include <cstddef>
#include <imgui.h>
#include <numeric>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Positioning/RTK/Keys.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include <fmt/core.h>

void NAV::RtkSolution::guiTooltipSatellites(const std::map<SatelliteSystem, std::unordered_set<SatId>>& satsReceived, const char* id) const
{
    if (ImGui::BeginTable(fmt::format("Satellites {}", id).c_str(), 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit
                              | ImGuiTableFlags_NoHostExtendX))
    {
        for (const auto& [satSys, satsRecv] : satsReceived)
        {
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(fmt::format("{:<4}", satSys).c_str());
            ImGui::SameLine();
            ImGui::TextUnformatted(fmt::format("(# {:2})", satsRecv.size()).c_str());

            ImGui::TableNextColumn();
            size_t printed = 0;
            for (const SatId& satId : satsRecv)
            {
                if (satId.satSys != satSys) { continue; }

                bool sameLine = printed != 0 && (printed % 7) != 0;
                if (sameLine)
                {
                    ImGui::SameLine();
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x);
                    ImGui::TextUnformatted(", ");
                    ImGui::SameLine();
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x);
                }
                auto sat = std::ranges::find_if(satData, [&](const std::pair<SatId, RtkSolution::SatData>& satData) {
                    return satData.first == satId;
                });
                bool isUnused = std::ranges::find_if(observableUsed, [&](const Observable& used) {
                                    return satId == used.satSigId.toSatId();
                                })
                                == observableUsed.end();
                bool isFiltered = std::ranges::find_if(observableFiltered, [&](const Observable& used) {
                                      return satId == used.satSigId.toSatId();
                                  })
                                  == observableFiltered.end();

                if (isFiltered) { ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled)); } // Gray
                else if (isUnused) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(115, 147, 179).Value); }                 // Blue Gray

                ImGui::TextUnformatted(fmt::format("{} ({:2.0f}°)", satId, rad2deg(sat->second.satElevation)).c_str());

                if (isFiltered || isUnused) { ImGui::PopStyleColor(); }

                if (ImGui::IsItemHovered())
                {
                    if (isFiltered)
                    {
                        ImGui::BeginTooltip();
                        if (std::ranges::any_of(filtered.frequencyFilter,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Signals excluded due to Frequency filter"); }
                        if (std::ranges::any_of(filtered.codeFilter,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Signals excluded due to Code filter"); }
                        if (std::ranges::any_of(filtered.excludedSatellites,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Satellite excluded due to satellite exclusion list"); }
                        if (std::ranges::any_of(filtered.tempExcludedSignal,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Signals temporarily excluded"); }
                        if (std::ranges::any_of(filtered.notAllReceiversObserved,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Signals not observed by all receivers"); }
                        if (std::ranges::any_of(filtered.singleObservation,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("No second signal for double difference."); }
                        if (std::ranges::any_of(filtered.noPseudorangeMeasurement,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Signals without pseudorange measurement"); }
                        if (std::ranges::any_of(filtered.navigationDataMissing,
                                                [&satId](const SatSigId& satSigId) {
                                                    return satSigId.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Satellite without navigation data"); }
                        if (std::ranges::any_of(filtered.elevationMaskTriggered,
                                                [&satId](const std::pair<SatSigId, double>& satSigId) {
                                                    return satSigId.first.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Satellite triggered elevation mask"); }
                        if (std::ranges::any_of(filtered.snrMaskTriggered,
                                                [&satId](const std::pair<SatSigId, double>& satSigId) {
                                                    return satSigId.first.toSatId() == satId;
                                                })) { ImGui::TextUnformatted("Signals triggered SNR mask"); }
                        ImGui::EndTooltip();
                    }
                    else if (isUnused) { ImGui::SetTooltip("Removed due to outlier check"); }
                }

                printed++;
            }
        }

        ImGui::EndTable();
    }
}

void NAV::RtkSolution::guiTooltipObservationTable(const std::multiset<RtkSolution::Observable>& observables,
                                                  bool showSatCounts,
                                                  bool colorPivots,
                                                  bool colorNotUsed,
                                                  bool colorCycleSlips,
                                                  bool colorPivotChanges,
                                                  const char* id) const
{
    std::array<bool, GnssObs::ObservationType_COUNT> hasObsType{};
    std::map<SatelliteSystem, size_t> obsCount;
    std::map<SatelliteSystem, std::set<SatId>> satellites;
    for (const RtkSolution::Observable& obs : observables)
    {
        hasObsType.at(obs.obsType) = true;
        auto satId = obs.satSigId.toSatId();
        obsCount[satId.satSys]++;
        satellites[satId.satSys].insert(satId);
    }
    auto obsTypeCount = static_cast<int>(std::ranges::count(hasObsType, true));

    if (ImGui::BeginTable(fmt::format("Pivot sats {}", id).c_str(), obsTypeCount + 1,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit
                              | ImGuiTableFlags_NoHostExtendX))
    {
        ImGui::TableSetupColumn("");
        auto headerColumn = [&](const char* desc, const GnssObs::ObservationType& obsType) {
            if (hasObsType.at(obsType))
            {
                if (showSatCounts)
                {
                    ImGui::TableSetupColumn(fmt::format("{} #{} ({} sats)", desc, nObservations.at(obsType),
                                                        nObservationsUniqueSatellite.at(obsType))
                                                .c_str());
                }
                else { ImGui::TableSetupColumn(desc); }
            }
        };
        headerColumn("Pseudorange", GnssObs::Pseudorange);
        headerColumn("Carrier", GnssObs::Carrier);
        headerColumn("Doppler", GnssObs::Doppler);

        ImGui::TableHeadersRow();

        for (const auto& [satSys, sats] : satellites)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(fmt::format("{}", satSys).c_str());

            if (showSatCounts)
            {
                ImGui::TextUnformatted(fmt::format("#{:3}", obsCount.at(satSys)).c_str());
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Number of signals"); }

                ImGui::TextUnformatted(fmt::format("S{:3}", sats.size()).c_str());
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Number of satellites"); }
            }

            for (size_t o = 0; o < GnssObs::ObservationType_COUNT; o++)
            {
                if (!hasObsType.at(o)) { continue; }
                ImGui::TableSetColumnIndex(static_cast<int>(o + 1));

                size_t printed = 0;
                Frequency lastFreq = Freq_None;
                for (const auto& code : Code::GetAll())
                {
                    if (code.getFrequency().getSatSys() != satSys) { continue; }

                    for (const RtkSolution::Observable& obs : observables)
                    {
                        if (obs.satSigId.code != code || obs.obsType != o) { continue; }

                        bool sameLine = printed != 0 && (printed % 2) != 0;
                        if (sameLine)
                        {
                            ImGui::SameLine();
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x);
                            if (lastFreq != code.getFrequency()) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(243, 156, 18).Value); } // Orange
                            ImGui::TextUnformatted(", ");
                            if (lastFreq != code.getFrequency()) { ImGui::PopStyleColor(); }
                            ImGui::SameLine();
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x);
                        }
                        auto sat = std::ranges::find_if(satData, [&](const std::pair<SatId, RtkSolution::SatData>& satData) {
                            return satData.first == obs.satSigId.toSatId();
                        });

                        bool isPivot = std::ranges::find_if(pivots, [&](const Observable& piv) {
                                           return piv.satSigId == obs.satSigId && piv.obsType == obs.obsType;
                                       })
                                       != pivots.end();
                        bool isFiltered = std::ranges::find_if(observableFiltered, [&](const Observable& used) {
                                              return obs.satSigId == used.satSigId && obs.obsType == used.obsType;
                                          })
                                          == observableFiltered.end();
                        bool isUnused = std::ranges::find_if(observableUsed, [&](const Observable& used) {
                                            return obs.satSigId == used.satSigId && obs.obsType == used.obsType;
                                        })
                                        == observableUsed.end();
                        bool isNewlyEstimated = std::ranges::find_if(newEstimatedAmbiguity, [&](const SatSigId& satSigId) {
                                                    return obs.satSigId == satSigId && obs.obsType == GnssObs::Carrier;
                                                })
                                                != newEstimatedAmbiguity.end();

                        auto cycleSlip = std::ranges::find_if(cycleSlipDetectorResult,
                                                              [&obs](const std::pair<CycleSlipDetector::Result, std::string>& cycleSlip) {
                                                                  if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipLossOfLockIndicator>(&cycleSlip.first))
                                                                  {
                                                                      return s->signal == obs.satSigId;
                                                                  }
                                                                  if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipSingleFrequency>(&cycleSlip.first))
                                                                  {
                                                                      return s->signal == obs.satSigId;
                                                                  }
                                                                  if (const auto* s = std::get_if<CycleSlipDetector::CycleSlipDualFrequency>(&cycleSlip.first))
                                                                  {
                                                                      return s->signals.front() == obs.satSigId || s->signals.back() == obs.satSigId;
                                                                  }
                                                                  return false;
                                                              });
                        bool isCycleSlip = obs.obsType == GnssObs::Carrier && cycleSlip != cycleSlipDetectorResult.end();

                        bool pivChanged = isPivot && changedPivotSatellites.contains(std::make_pair(code, obs.obsType));

                        if (colorNotUsed && isFiltered) { ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled)); } // Gray
                        else if (colorNotUsed && isUnused) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(115, 147, 179).Value); }                 // Blue Gray
                        else if (colorCycleSlips && isCycleSlip) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(255, 191, 0).Value); }             // Yellow
                        else if (colorCycleSlips && isNewlyEstimated) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(245, 245, 220).Value); }      // Beige
                        else if (colorPivotChanges && pivChanged) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(255, 191, 0).Value); }            // Yellow
                        else if (colorPivots && isPivot) { ImGui::PushStyleColor(ImGuiCol_Text, ImColor(80, 200, 120).Value); }                    // Green

                        ImGui::TextUnformatted(fmt::format("{} ({:2.0f}°)", obs.satSigId, rad2deg(sat->second.satElevation)).c_str());

                        if ((colorPivots && isPivot)
                            || (colorNotUsed && isFiltered)
                            || (colorNotUsed && isUnused)
                            || (colorCycleSlips && (isCycleSlip || isNewlyEstimated))
                            || (colorPivotChanges && pivChanged)) { ImGui::PopStyleColor(); }

                        if (ImGui::IsItemHovered())
                        {
                            if (isPivot && !colorPivotChanges)
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextUnformatted("Pivot");
                                ImGui::EndTooltip();
                            }
                            if (isFiltered)
                            {
                                ImGui::BeginTooltip();
                                if (std::ranges::any_of(filtered.frequencyFilter,
                                                        [&obs](const SatSigId& satSigId) {
                                                            return satSigId == obs.satSigId;
                                                        })) { ImGui::TextUnformatted("Signal excluded due to Frequency filter"); }
                                else if (std::ranges::any_of(filtered.codeFilter,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("Signal excluded due to Code filter"); }
                                else if (std::ranges::any_of(filtered.excludedSatellites,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("Signal excluded due to satellite exclusion list"); }
                                else if (std::ranges::any_of(filtered.tempExcludedSignal,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("Signal temporarily excluded"); }
                                else if (std::ranges::any_of(filtered.notAllReceiversObserved,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("Signal not observed by all receivers"); }
                                else if (std::ranges::any_of(filtered.singleObservation,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("No second signal for double difference."); }
                                else if (std::ranges::any_of(filtered.noPseudorangeMeasurement,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("Signal excluded because no pseudorange measurement to calculate satellite position"); }
                                else if (std::ranges::any_of(filtered.navigationDataMissing,
                                                             [&obs](const SatSigId& satSigId) {
                                                                 return satSigId == obs.satSigId;
                                                             })) { ImGui::TextUnformatted("Signal excluded because no navigation data"); }
                                else if (auto sig = std::ranges::find_if(filtered.elevationMaskTriggered,
                                                                         [&obs](const std::pair<SatSigId, double>& satSigId) {
                                                                             return satSigId.first == obs.satSigId;
                                                                         });
                                         sig != filtered.elevationMaskTriggered.end())
                                {
                                    ImGui::TextUnformatted(fmt::format("Satellite triggered elevation mask (elevation {:2.0f}°)", rad2deg(sig->second)).c_str());
                                }
                                else if (auto sig = std::ranges::find_if(filtered.snrMaskTriggered,
                                                                         [&obs](const std::pair<SatSigId, double>& satSigId) {
                                                                             return satSigId.first == obs.satSigId;
                                                                         });
                                         sig != filtered.snrMaskTriggered.end())
                                {
                                    ImGui::TextUnformatted(fmt::format("Signal triggered SNR mask (SNR {:.0f} [dBHz])", sig->second).c_str());
                                }
                                ImGui::EndTooltip();
                            }
                            else if (isUnused)
                            {
                                if (auto outlier = std::ranges::find_if(outliers, [&obs](const Outlier& outlier) {
                                        return outlier.satSigId == obs.satSigId && outlier.obsType == obs.obsType;
                                    });
                                    outlier != outliers.end())
                                {
                                    ImGui::BeginTooltip();
                                    ImGui::TextUnformatted(fmt::format("Outlier: {}", outlier->type).c_str());
                                    ImGui::EndTooltip();
                                }
                                else
                                {
                                    ImGui::BeginTooltip();
                                    ImGui::TextUnformatted("Cannot calc double difference with single observation");
                                    ImGui::EndTooltip();
                                }
                            }
                            if (isCycleSlip)
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextUnformatted(fmt::format("{}: {}", cycleSlip->second, cycleSlip->first).c_str());
                                ImGui::EndTooltip();
                            }
                            else if (isNewlyEstimated)
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextUnformatted("Signal is newly estimated this epoch");
                                ImGui::EndTooltip();
                            }
                            if (pivChanged)
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextUnformatted(fmt::format("{}", changedPivotSatellites.at(std::make_pair(code, obs.obsType))).c_str());
                                ImGui::EndTooltip();
                            }
                            RTK::Meas::MeasKeyTypes key;
                            switch (obs.obsType)
                            {
                            case GnssObs::Pseudorange:
                                key = RTK::Meas::PsrDD{ obs.satSigId };
                                break;
                            case GnssObs::Carrier:
                                key = RTK::Meas::CarrierDD{ obs.satSigId };
                                break;
                            case GnssObs::Doppler:
                                key = RTK::Meas::DopplerDD{ obs.satSigId };
                                break;
                            case GnssObs::ObservationType_COUNT:
                                break;
                            }
                            if (measInnovation.hasRow(key))
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextUnformatted(fmt::format("Meas. innovation: {:.2g}", measInnovation(key)).c_str());
                                ImGui::EndTooltip();
                            }
                        }
                        printed++;
                        lastFreq = code.getFrequency();
                    }
                }
            }
        }

        ImGui::EndTable();
    }
}

void NAV::RtkSolution::guiTooltipAmbiguities(const char* id) const
{
    std::set<SatSigId> pivotSats;
    for (const auto& amb : ambiguityDD_br)
    {
        pivotSats.insert(amb.pivotSatSigId);
    }

    if (ImGui::BeginTable(fmt::format("Ambiguities {}", id).c_str(), static_cast<int>(pivotSats.size()),
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit
                              | ImGuiTableFlags_NoHostExtendX))
    {
        for (const auto& pivotSatSigId : pivotSats)
        {
            ImGui::TableSetupColumn(fmt::format("{}", pivotSatSigId).c_str());
        }
        ImGui::TableHeadersRow();

        bool addedEntry = true;
        for (size_t row = 0; addedEntry; row++)
        {
            addedEntry = false;
            int p = 0;
            for (const auto& pivotSatSigId : pivotSats)
            {
                size_t i = 0;
                for (const auto& amb : ambiguityDD_br)
                {
                    if (amb.pivotSatSigId != pivotSatSigId) { continue; }
                    if (i++ < row) { continue; } // Skip entries to be in the correct row

                    if (!addedEntry) { ImGui::TableNextRow(); }
                    ImGui::TableSetColumnIndex(p);
                    ImGui::TextUnformatted(fmt::format("{}", amb.satSigId).c_str());

                    ImGui::TextUnformatted(fmt::format("V {:7.1f}", amb.value.value).c_str());
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Value [cycles]"); }

                    ImGui::TextUnformatted(fmt::format("S {:7.1e}", amb.value.stdDev).c_str());
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Standard deviation [cycles]"); }

                    addedEntry = true;
                    break;
                }
                p++;
            }
        }
        ImGui::EndTable();
    }
}

void NAV::RtkSolution::guiTooltip(bool detailView, bool firstOpen, const char* /* displayName */, const char* id, int* /* rootWindow */) const
{
    if (nAmbiguitiesFixed && *nAmbiguitiesFixed - 1 < ambiguityDD_br.size())
    {
        ImGui::BulletText("%s", fmt::format("Solution Type: {}Partial Fix {} / {}",
                                            solType == SolutionType::RTK_Float ? "" : fmt::format("{} - ", solType),
                                            *nAmbiguitiesFixed - 1,
                                            ambiguityDD_br.size())
                                    .c_str());
    }
    else
    {
        ImGui::BulletText("%s", fmt::format("Solution Type: {}", solType).c_str());
    }

    ImGui::BulletText("Rover obs time: %s", fmt::format("{}", insTime.toYMDHMS(GPST)).c_str());
    ImGui::BulletText("Base  obs time: %s", fmt::format("{}", baseTime.toYMDHMS(GPST)).c_str());

    std::map<SatelliteSystem, std::unordered_set<SatId>> satsReceived;
    for (const auto& obs : observableReceived)
    {
        satsReceived[obs.satSigId.toSatId().satSys].insert(obs.satSigId.toSatId());
    }
    size_t nSatellitesReceived = 0;
    for (const auto& satRecv : satsReceived) { nSatellitesReceived += satRecv.second.size(); }

    ImGui::SetNextItemOpen(detailView, firstOpen ? ImGuiCond_Always : ImGuiCond_Once);
    if (ImGui::TreeNode(fmt::format("Satellites:  used {:3d} / {:3d} received", nSatellites, nSatellitesReceived).c_str()))
    {
        guiTooltipSatellites(satsReceived, id);
        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(detailView, firstOpen ? ImGuiCond_Always : ImGuiCond_Once);
    if (ImGui::TreeNode(fmt::format("Observables: used {:3d} / {:3d} received", observableUsed.size(), observableReceived.size()).c_str()))
    {
        guiTooltipObservationTable(observableReceived, true, true, true, true, false, id);
        ImGui::TreePop();
    }

    if (ImGui::TreeNode(fmt::format("Pivots: {} ({} changed)", pivots.size(), changedPivotSatellites.size()).c_str()))
    {
        guiTooltipObservationTable(pivots, false, false, true, false, true, id);
        ImGui::TreePop();
    }

    if (nisResultInitial)
    {
        if (nisResultInitial->triggered)
        {
            ImGui::SetNextItemOpen(detailView, firstOpen ? ImGuiCond_Always : ImGuiCond_Once);
            if (ImGui::TreeNode(fmt::format("NIS Outlier check: {} observables removed", nisRemovedCnt).c_str()))
            {
                ImGui::BulletText("%s", fmt::format("Initial NIS {:.2g} > {:.2g} r2", nisResultInitial->NIS, nisResultInitial->r2).c_str());
                ImGui::BulletText("%s", fmt::format("Final   NIS {:.2g} {} {:.2g} r2",
                                                    nisResultFinal->NIS, nisResultFinal->NIS < nisResultFinal->r2 ? "<" : ">", nisResultFinal->r2)
                                            .c_str());
                ImGui::TreePop();
            }
        }
        else
        {
            ImGui::BulletText("NIS Outlier check not triggered");
        }
    }

    ImGui::SetNextItemOpen(detailView, firstOpen ? ImGuiCond_Always : ImGuiCond_Once);
    if (ImGui::TreeNode(fmt::format("Measurement innovation: Largest {:.2g}, Mean {:.2g}",
                                    measInnovation.rows() ? measInnovation(all).cwiseAbs().maxCoeff() : std::nan(""),
                                    measInnovation.rows() ? measInnovation(all).mean() : std::nan(""))
                            .c_str()))
    {
        auto showLargestAndMean = [&]<typename T>(const char* desc, const char* unit) {
            if (std::any_of(measInnovation.rowKeys().begin(), measInnovation.rowKeys().end(), [](const auto& key) {
                    return std::holds_alternative<T>(key);
                }))
            {
                std::vector<RTK::Meas::MeasKeyTypes> keys;
                for (const auto& key : measInnovation.rowKeys())
                {
                    if (std::holds_alternative<T>(key)) { keys.push_back(key); }
                }
                ImGui::BulletText("%s", fmt::format("{} Largest {:.2g}, Mean {:.2g} [{}]",
                                                    desc,
                                                    measInnovation(keys).rows() ? measInnovation(keys).cwiseAbs().maxCoeff() : std::nan(""),
                                                    measInnovation(keys).rows() ? measInnovation(keys).mean() : std::nan(""),
                                                    unit)
                                            .c_str());
            }
        };
        showLargestAndMean.operator()<RTK::Meas::PsrDD>("Pseudorange:", "m");
        showLargestAndMean.operator()<RTK::Meas::CarrierDD>("Carrier:    ", "m");
        showLargestAndMean.operator()<RTK::Meas::DopplerDD>("Doppler:    ", "m/s");

        if (ImGui::TreeNode("Vector"))
        {
            gui::widgets::KeyedVectorView(fmt::format("Measurement innovation##{}", id).c_str(), &measInnovation, 300.0F);
            ImGui::TreePop();
        }
        ImGui::TreePop();
    }

    if (ImGui::TreeNode(fmt::format("Ambiguities: {}", ambiguityDD_br.size()).c_str()))
    {
        guiTooltipAmbiguities(id);
        ImGui::TreePop();
    }
}