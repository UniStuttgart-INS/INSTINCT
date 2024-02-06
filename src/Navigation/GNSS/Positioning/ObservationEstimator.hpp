// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ObservationEstimator.hpp
/// @brief Calculates Observation estimates
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#pragma once

#include <vector>

#include <imgui.h>
#include "internal/gui/widgets/imgui_ex.hpp"

#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/GNSS/Errors/MeasurementErrors.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/Receiver.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

#include "util/Json.hpp"

namespace NAV
{

/// @brief Calculates Observation estimates
class ObservationEstimator
{
  public:
    /// @brief How the observation gets used. Influenced the measurement variance
    enum ObservationDifference
    {
        NoDifference,     ///< Estimation is not differenced
        SingleDifference, ///< Single Difference
        DoubleDifference, ///< Double Difference
    };

    /// @brief Calculates the observation estimates
    /// @param[in, out] observations List of GNSS observation data used for the calculation of this epoch
    /// @param[in] ionosphericCorrections Ionospheric correction parameters collected from the Nav data
    /// @param[in] receivers List of receivers
    /// @param[in] nameId Name and Id of the node used for log messages only
    /// @param[in] obsDiff Observation Difference type to estimate
    template<typename ReceiverType>
    void calcObservationEstimates(Observations& observations,
                                  const std::array<Receiver<ReceiverType>, ReceiverType::ReceiverType_COUNT>& receivers,
                                  const IonosphericCorrections& ionosphericCorrections,
                                  [[maybe_unused]] const std::string& nameId,
                                  ObservationDifference obsDiff)
    {
        LOG_DATA("{}: Calculating observation estimates:", nameId);

        for (auto& [satSigId, observation] : observations.signals)
        {
            const Frequency freq = satSigId.freq();
            const SatelliteSystem satSys = freq.getSatSys();

            for (size_t r = 0; r < observation.recvObs.size(); r++)
            {
                auto& recvObs = observation.recvObs.at(r);
                [[maybe_unused]] auto recv = static_cast<ReceiverType>(r);
                const auto& receiver = receivers.at(r);

                // Receiver-Satellite Range [m]
                double rho_r_s = (observation.e_satPos() - receiver.e_pos).norm();
                recvObs.terms.rho_r_s = rho_r_s;
                // Troposphere
                auto tropo_r_s = calcTroposphericDelayAndMapping(receiver.gnssObs->insTime, receiver.lla_pos,
                                                                 recvObs.satElevation(), recvObs.satAzimuth(), _troposphereModels);
                recvObs.terms.tropoZenithDelay = tropo_r_s;
                // Estimated troposphere propagation error [m]
                double dpsr_T_r_s = tropo_r_s.ZHD * tropo_r_s.zhdMappingFactor + tropo_r_s.ZWD * tropo_r_s.zwdMappingFactor;
                recvObs.terms.dpsr_T_r_s = dpsr_T_r_s;
                // Estimated ionosphere propagation error [m]
                double dpsr_I_r_s = calcIonosphericDelay(static_cast<double>(receiver.gnssObs->insTime.toGPSweekTow().tow),
                                                         freq, observation.freqNum(), receiver.lla_pos, recvObs.satElevation(), recvObs.satAzimuth(),
                                                         _ionosphereModel, &ionosphericCorrections);
                recvObs.terms.dpsr_I_r_s = dpsr_I_r_s;
                // Sagnac correction [m]
                double dpsr_ie_r_s = calcSagnacCorrection(receiver.e_pos, observation.e_satPos());
                recvObs.terms.dpsr_ie_r_s = dpsr_ie_r_s;

                double cn0 = recvObs.gnssObsData().CN0.value_or(1.0);

                for (auto& [obsType, obsData] : recvObs.obs)
                {
                    switch (obsType)
                    {
                    case GnssObs::Pseudorange:
                        obsData.estimate = rho_r_s
                                           + dpsr_ie_r_s
                                           + dpsr_T_r_s
                                           + dpsr_I_r_s
                                           + InsConst<>::C
                                                 * (receiver.recvClk.bias.value
                                                    - observation.satClock().bias
                                                    + receiver.recvClk.sysTimeDiffBias.at(satSys.toEnumeration()).value);
                        obsData.measVar = _gnssMeasurementErrorModel.psrMeasErrorVar(satSys, recvObs.satElevation(), cn0);
                        LOG_DATA("{}:   [{}][{:11}][{:5}] {} [m] = {} + {} + {} + {} + c * ({} - {} + {}); diff to meas: {}",
                                 nameId, satSigId, obsType, recv, obsData.estimate,
                                 rho_r_s, dpsr_ie_r_s, dpsr_T_r_s, dpsr_I_r_s, receiver.recvClk.bias.value, observation.satClock().bias,
                                 receiver.recvClk.sysTimeDiffBias.at(satSys.toEnumeration()).value, obsData.measurement - obsData.estimate);
                        break;
                    case GnssObs::Carrier:
                        obsData.estimate = rho_r_s
                                           + dpsr_ie_r_s
                                           + dpsr_T_r_s
                                           - dpsr_I_r_s
                                           + InsConst<>::C
                                                 * (receiver.recvClk.bias.value
                                                    - observation.satClock().bias
                                                    + receiver.recvClk.sysTimeDiffBias.at(satSys.toEnumeration()).value);
                        obsData.measVar = _gnssMeasurementErrorModel.carrierMeasErrorVar(satSys, recvObs.satElevation(), cn0);
                        LOG_DATA("{}:   [{}][{:11}][{:5}] {} [m] = {} + {} + {} - {} + c * ({} - {} + {}); diff to meas: {}",
                                 nameId, satSigId, obsType, recv, obsData.estimate,
                                 rho_r_s, dpsr_ie_r_s, dpsr_T_r_s, dpsr_I_r_s, receiver.recvClk.bias.value, observation.satClock().bias,
                                 receiver.recvClk.sysTimeDiffBias.at(satSys.toEnumeration()).value, obsData.measurement - obsData.estimate);
                        break;
                    case GnssObs::Doppler:
                        obsData.estimate = recvObs.e_pLOS().dot(observation.e_satVel() - receiver.e_vel)
                                           - calcSagnacRateCorrection(receiver.e_pos, observation.e_satPos(), receiver.e_vel, observation.e_satVel())
                                           + InsConst<>::C
                                                 * (receiver.recvClk.drift.value
                                                    - observation.satClock().drift
                                                    + receiver.recvClk.sysTimeDiffDrift.at(satSys.toEnumeration()).value);
                        obsData.measVar = _gnssMeasurementErrorModel.psrRateMeasErrorVar(freq, observation.freqNum(), recvObs.satElevation(), cn0);
                        LOG_DATA("{}:   [{}][{:11}][{:5}] {} [m/s] = {} - {} + c * ({} - {} + {}); diff to meas: {}",
                                 nameId, satSigId, obsType, recv, obsData.estimate,
                                 recvObs.e_pLOS().dot(observation.e_satVel() - receiver.e_vel),
                                 calcSagnacRateCorrection(receiver.e_pos, observation.e_satPos(), receiver.e_vel, observation.e_satVel()),
                                 receiver.recvClk.drift.value, observation.satClock().drift, receiver.recvClk.sysTimeDiffDrift.at(satSys.toEnumeration()).value,
                                 obsData.measurement - obsData.estimate);
                        break;
                    case GnssObs::ObservationType_COUNT:
                        break;
                    }
                    LOG_DATA("{}:   [{}][{:11}][{:5}] Observation error variance", nameId, satSigId, obsType, recv);
                    LOG_DATA("{}:   [{}][{:11}][{:5}]     {:.4f} [{}] Measurement error variance", nameId, satSigId, obsType, recv, obsData.measVar,
                             obsType == GnssObs::Doppler ? "m^2/s^2" : "m^2");

                    if (obsDiff == NoDifference)
                    {
                        if (obsType == GnssObs::Pseudorange || obsType == GnssObs::Carrier)
                        {
                            obsData.measVar += observation.navData()->calcSatellitePositionVariance()
                                               + ionoErrorVar(dpsr_I_r_s, freq, observation.freqNum())
                                               + tropoErrorVar(dpsr_T_r_s, recvObs.satElevation());
                            LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Satellite position variance", nameId, satSigId, obsType, recv, observation.navData()->calcSatellitePositionVariance());
                            LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Ionosphere variance", nameId, satSigId, obsType, recv, ionoErrorVar(dpsr_I_r_s, freq, observation.freqNum()));
                            LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Troposphere variance", nameId, satSigId, obsType, recv, tropoErrorVar(dpsr_T_r_s, recvObs.satElevation()));
                        }
                        if (obsType == GnssObs::Pseudorange)
                        {
                            obsData.measVar += _gnssMeasurementErrorModel.codeBiasErrorVar();
                            LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [{}] Code bias variance", nameId, satSigId, obsType, recv, _gnssMeasurementErrorModel.codeBiasErrorVar(),
                                     obsType == GnssObs::Doppler ? "m^2/s^2" : "m^2");
                        }
                    }
                    if (obsDiff != DoubleDifference)
                    {
                        if (obsType == GnssObs::Pseudorange || obsType == GnssObs::Carrier)
                        {
                            double recvClockVariance = std::pow(InsConst<>::C, 2)
                                                       * (std::pow(receiver.recvClk.bias.stdDev, 2)
                                                          + std::pow(receiver.recvClk.sysTimeDiffBias.at(satSys.toEnumeration()).stdDev, 2));
                            obsData.measVar += recvClockVariance;
                            LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2] Receiver clock bias variance", nameId, satSigId, obsType, recv, recvClockVariance);
                        }
                        else if (obsType == GnssObs::Doppler)
                        {
                            double recvClockVariance = std::pow(InsConst<>::C, 2)
                                                       * (std::pow(receiver.recvClk.drift.stdDev, 2)
                                                          + std::pow(receiver.recvClk.sysTimeDiffDrift.at(satSys.toEnumeration()).stdDev, 2));
                            obsData.measVar += recvClockVariance;
                            LOG_DATA("{}:   [{}][{:11}][{:5}]   + {:.4f} [m^2/s^2] Receiver clock drift variance", nameId, satSigId, obsType, recv, recvClockVariance);
                        }
                    }
                    LOG_DATA("{}:   [{}][{:11}][{:5}]   = {:.4f} [{}] Observation error variance", nameId, satSigId, obsType, recv, obsData.measVar,
                             obsType == GnssObs::Doppler ? "m^2/s^2" : "m^2");
                }
            }
        }
    }

    /// @brief Shows the GUI input to select the options
    /// @param[in] id Unique id for ImGui.
    /// @param[in] itemWidth Width of the widgets
    bool ShowGuiWidgets(const char* id, float itemWidth)
    {
        bool changed = false;

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Compensation models##{}", id).c_str()))
        {
            ImGui::SetNextItemWidth(itemWidth - ImGui::GetStyle().IndentSpacing);
            if (ComboIonosphereModel(fmt::format("Ionosphere Model##{}", id).c_str(), _ionosphereModel))
            {
                LOG_DEBUG("{}: Ionosphere Model changed to {}", id, NAV::to_string(_ionosphereModel));
                changed = true;
            }
            if (ComboTroposphereModel(fmt::format("Troposphere Model##{}", id).c_str(), _troposphereModels, itemWidth - ImGui::GetStyle().IndentSpacing))
            {
                changed = true;
            }
            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("GNSS Measurement Error##{}", id).c_str()))
        {
            if (_gnssMeasurementErrorModel.ShowGuiWidgets(id, itemWidth - ImGui::GetStyle().IndentSpacing))
            {
                LOG_DEBUG("{}: GNSS Measurement Error Model changed.", id);
                changed = true;
            }
            ImGui::TreePop();
        }

        return changed;
    }

  private:
    IonosphereModel _ionosphereModel = IonosphereModel::Klobuchar; ///< Ionosphere Model used for the calculation
    TroposphereModelSelection _troposphereModels;                  ///< Troposphere Models used for the calculation
    GnssMeasurementErrorModel _gnssMeasurementErrorModel;          ///< GNSS measurement error model to use

    /// @brief Converts the provided object into json
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] obj Object to convert into json
    friend void to_json(json& j, const ObservationEstimator& obj)
    {
        j = json{
            { "ionosphereModel", Frequency_(obj._ionosphereModel) },
            { "troposphereModels", obj._troposphereModels },
            { "gnssMeasurementError", obj._gnssMeasurementErrorModel },
        };
    }
    /// @brief Converts the provided json object into a node object
    /// @param[in] j Json object with the needed values
    /// @param[out] obj Object to fill from the json
    friend void from_json(const json& j, ObservationEstimator& obj)
    {
        if (j.contains("ionosphereModel")) { j.at("ionosphereModel").get_to(obj._ionosphereModel); }
        if (j.contains("troposphereModels")) { j.at("troposphereModels").get_to(obj._troposphereModels); }
        if (j.contains("gnssMeasurementError")) { j.at("gnssMeasurementError").get_to(obj._gnssMeasurementErrorModel); }
    }
};

} // namespace NAV
