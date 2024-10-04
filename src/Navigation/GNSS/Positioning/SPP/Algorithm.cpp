// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Algorithm.cpp
/// @brief Single Point Positioning Algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-20

#include "Algorithm.hpp"
#include <algorithm>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>

#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/GNSS/Positioning/Observation.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"
#include "util/Logger.hpp"
#include <fmt/format.h>

#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

#include "Navigation/Atmosphere/Ionosphere/IonosphericCorrections.hpp"
#include "Navigation/Math/KeyedLeastSquares.hpp"

#include "util/Container/STL.hpp"

namespace NAV
{
namespace SPP
{

bool Algorithm::ShowGuiWidgets(const char* id, float itemWidth, float unitWidth)
{
    bool changed = false;

    changed |= _obsFilter.ShowGuiWidgets<ReceiverType>(id, itemWidth);

    ImGui::SetNextItemWidth(itemWidth);
    changed |= gui::widgets::EnumCombo(fmt::format("Estimation algorithm##{}", id).c_str(), _estimatorType);

    if (!canEstimateInterFrequencyBias()) { ImGui::BeginDisabled(); }
    changed |= ImGui::Checkbox(fmt::format("Estimate inter-frequency biases##{}", id).c_str(), &_estimateInterFreqBiases);
    if (!canEstimateInterFrequencyBias()) { ImGui::EndDisabled(); }

    changed |= _obsEstimator.ShowGuiWidgets<ReceiverType>(id, itemWidth);

    if (_estimatorType == EstimatorType::KalmanFilter)
    {
        changed |= _kalmanFilter.ShowGuiWidgets(id, _obsFilter.isObsTypeUsed(GnssObs::Doppler),
                                                _obsFilter.getSystemFilter().toVector().size() != 1,
                                                _estimateInterFreqBiases && canEstimateInterFrequencyBias(), itemWidth, unitWidth);
    }

    return changed;
}

void Algorithm::reset()
{
    _receiver = Receiver(Rover, _obsFilter.getSystemFilter().toVector());
    _obsFilter.reset();
    _kalmanFilter.reset(_obsFilter.getSystemFilter().toVector());
    _lastUpdate.reset();
}

std::shared_ptr<SppSolution> Algorithm::calcSppSolution(const std::shared_ptr<const GnssObs>& gnssObs,
                                                        const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                        const std::string& nameId)
{
    _receiver.gnssObs = gnssObs;

    if (gnssNavInfos.empty())
    {
        LOG_ERROR("{}: [{}] SPP cannot calculate position because no navigation data provided.", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST));
        return nullptr;
    }
    LOG_DATA("{}: [{}] Calculating SPP", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST));

    // Collection of all connected Ionospheric Corrections
    auto ionosphericCorrections = std::make_shared<const IonosphericCorrections>(gnssNavInfos);

    double dt = _lastUpdate.empty() ? 0.0 : static_cast<double>((_receiver.gnssObs->insTime - _lastUpdate).count());
    LOG_DATA("{}: dt = {}s", nameId, dt);
    _lastUpdate = _receiver.gnssObs->insTime;

    auto sppSol = std::make_shared<SppSolution>();
    sppSol->insTime = _receiver.gnssObs->insTime;

    if (_estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized())
    {
        _kalmanFilter.predict(dt, _receiver.lla_posMarker, nameId);
        assignKalmanFilterResult(_kalmanFilter.getState(), _kalmanFilter.getErrorCovarianceMatrix(), nameId);
    }

    constexpr size_t N_ITER_MAX_LSQ = 10;
    size_t nIter = _estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized() ? 1 : N_ITER_MAX_LSQ;
    Eigen::Vector3d e_oldPos = _receiver.e_posMarker;
    bool accuracyAchieved = false;
    for (size_t iteration = 0; iteration < nIter; iteration++)
    {
        LOG_DATA("{}: [{}] iteration {}/{}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), iteration + 1, nIter);
        LOG_DATA("{}: [{}]   e_pos    = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_posMarker.transpose());
        LOG_DATA("{}: [{}]   e_vel    = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_vel.transpose());
        LOG_DATA("{}: [{}] lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
                 rad2deg(_receiver.lla_posMarker.x()), rad2deg(_receiver.lla_posMarker.y()), _receiver.lla_posMarker.z());
        for ([[maybe_unused]] const auto& satSys : _receiver.recvClk.satelliteSystems)
        {
            LOG_DATA("{}: [{}] {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), satSys);
            LOG_DATA("{}: [{}]   clkBias  = {} [s] (StdDev = {})", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), *_receiver.recvClk.biasFor(satSys), *_receiver.recvClk.biasStdDevFor(satSys));
            LOG_DATA("{}: [{}]   clkDrift = {} [s/s] (StdDev = {})", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), *_receiver.recvClk.driftFor(satSys), *_receiver.recvClk.driftStdDevFor(satSys));
        }
        for ([[maybe_unused]] const auto& freq : _receiver.interFrequencyBias)
        {
            LOG_DATA("{}: [{}]   IFBBias [{:3}] = {} [s] (StdDev = {})", nameId,
                     _receiver.gnssObs->insTime.toYMDHMS(GPST), freq.first, freq.second.value, freq.second.stdDev);
        }

        bool firstEpoch = e_oldPos.isZero();
        Observations observations;
        _obsFilter.selectObservationsForCalculation(Rover,
                                                    _receiver.e_posMarker,
                                                    _receiver.lla_posMarker,
                                                    _receiver.gnssObs,
                                                    gnssNavInfos,
                                                    observations,
                                                    nullptr,
                                                    nameId,
                                                    firstEpoch);
        if (observations.signals.empty())
        {
            LOG_TRACE("{}: [{}] SPP cannot calculate position because no valid observations. Try changing filter settings or reposition your antenna.",
                      nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST));
            return nullptr;
        }

        size_t nParams = SPP::States::POS_STATE_COUNT + observations.systems.size() - 1;

        sppSol->nSatellites = observations.satellites.size();
        sppSol->nMeasPsr = observations.nObservables[GnssObs::Pseudorange];
        sppSol->nMeasDopp = observations.nObservables[GnssObs::Doppler];
        LOG_DATA("{}: nParams     = {}", nameId, nParams);
        LOG_DATA("{}: nSatellites = {}", nameId, sppSol->nSatellites);
        LOG_DATA("{}: nMeasPsr    = {}", nameId, sppSol->nMeasPsr);
        LOG_DATA("{}: nMeasDopp   = {}", nameId, sppSol->nMeasDopp);
        for (const auto& satSys : observations.systems)
        {
            [[maybe_unused]] auto satCount = std::count_if(observations.satellites.begin(), observations.satellites.end(), [&](const SatId& satId) {
                return satId.satSys == satSys;
            });

            LOG_DATA("{}: nSat {:5}  = {}", nameId, satSys, satCount);
        }

        if (size_t nPsrMeas = observations.nObservablesUniqueSatellite[GnssObs::Pseudorange];
            (_estimatorType != EstimatorType::KalmanFilter || !_kalmanFilter.isInitialized()) && nPsrMeas <= nParams)
        {
            LOG_TRACE("{}: [{}] SPP cannot calculate position because only {} satellites with pseudorange observations ({} needed). Try changing filter settings or reposition your antenna.",
                      nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), nPsrMeas, nParams + 1);
            return nullptr;
        }

        updateInterFrequencyBiases(observations, nameId);

        _obsEstimator.calcObservationEstimates(observations, _receiver, ionosphericCorrections, nameId, ObservationEstimator::NoDifference);

        auto stateKeys = determineStateKeys(observations.systems, observations.nObservables[GnssObs::Doppler], nameId);
        auto measKeys = determineMeasKeys(observations, sppSol->nMeasPsr, sppSol->nMeasDopp, nameId);

        sppSol->nParam = stateKeys.size();

        auto H = calcMatrixH(stateKeys, measKeys, observations, nameId);
        auto R = calcMatrixR(measKeys, observations, nameId);
        auto dz = calcMeasInnovation(measKeys, observations, nameId);

        if (_estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized())
        {
            std::string highInnovation;
            size_t highInnovationCounter = 0;
            for (const auto& key : dz.rowKeys())
            {
                if (double val = dz(key); std::abs(val) > 1000)
                {
                    highInnovation += fmt::format("{}[{} {:.0f}], ", highInnovationCounter++ % 3 == 0 ? "\n" : "", key, val);
                }
            }
            if (highInnovationCounter != 0)
            {
                std::string msg = fmt::format("Potential clock jump detected. Adjusting KF clock error covariance.\n"
                                              "Too large innovations: {}",
                                              highInnovation.substr(0, highInnovation.length() - 2));
                LOG_WARN("{}: [{}] {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), msg);
                _kalmanFilter.setClockBiasErrorCovariance(1e1);
                sppSol->addEvent(msg);
            }
        }

        if (_estimatorType != EstimatorType::KalmanFilter || !_kalmanFilter.isInitialized())
        {
            KeyedLeastSquaresResult<double, States::StateKeyType> lsq;
            if (_estimatorType == EstimatorType::LeastSquares)
            {
                lsq = solveLinearLeastSquaresUncertainties(H, dz);
            }
            else /* if (_estimatorType == EstimatorType::WeightedLeastSquares) */
            {
                auto W = KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>(Eigen::MatrixXd(R(all, all).diagonal().cwiseInverse().asDiagonal()), R.colKeys(), R.rowKeys());
                LOG_DATA("{}: W =\n{}", nameId, W);
                lsq = solveWeightedLinearLeastSquaresUncertainties(H, W, dz);
            }
            LOG_DATA("{}: LSQ sol (dx) =\n{}", nameId, lsq.solution.transposed());
            LOG_DATA("{}: LSQ var =\n{}", nameId, lsq.variance.transposed());

            assignLeastSquaresResult(lsq.solution, lsq.variance, e_oldPos,
                                     nParams, observations.nObservablesUniqueSatellite[GnssObs::Doppler], dt, nameId);

            accuracyAchieved = lsq.solution(all).norm() < 1e-4;
            if (accuracyAchieved) { LOG_DATA("{}: [{}] Accuracy achieved on iteration {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), iteration + 1); }
            else { LOG_DATA("{}: [{}] Bad accuracy on iteration {}: {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), iteration + 1, lsq.solution(all).norm()); }
            if (accuracyAchieved || iteration == nIter - 1)
            {
                if (_estimatorType == EstimatorType::KalmanFilter && !_kalmanFilter.isInitialized()
                    && sppSol->nMeasPsr > nParams // Variance can only be calculated if more measurements than parameters
                    && (!_obsFilter.isObsTypeUsed(GnssObs::Doppler) || sppSol->nMeasDopp > nParams))
                {
                    LOG_TRACE("{}: [{}] Initializing KF with LSQ solution", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST));
                    KeyedVectorXd<States::StateKeyType> x(Eigen::VectorXd::Zero(lsq.solution.rows()), lsq.solution.rowKeys());
                    x.segment<3>(PosKey) = _receiver.e_posMarker;
                    if (x.hasAnyRows(VelKey)) { x.segment<3>(VelKey) = _receiver.e_vel; }
                    for (size_t i = 0; i < _receiver.recvClk.satelliteSystems.size(); i++)
                    {
                        auto satSys = _receiver.recvClk.satelliteSystems.at(i);
                        x(Keys::RecvClkBias{ satSys }) = _receiver.recvClk.bias.at(i) * InsConst::C;
                        if (x.hasRow(Keys::RecvClkDrift{ satSys }))
                        {
                            x(Keys::RecvClkDrift{ satSys }) = _receiver.recvClk.drift.at(i) * InsConst::C;
                        }
                    }
                    for (const auto& state : x.rowKeys())
                    {
                        if (const auto* bias = std::get_if<Keys::InterFreqBias>(&state))
                        {
                            x(*bias) = _receiver.interFrequencyBias.at(bias->freq).value * InsConst::C;
                        }
                    }

                    _kalmanFilter.initialize(x, lsq.variance);
                    LOG_DATA("{}: x =\n{}", nameId, _kalmanFilter.getState().transposed());
                    LOG_DATA("{}: P =\n{}", nameId, _kalmanFilter.getErrorCovarianceMatrix());
                }
                sppSol->setPositionAndStdDev_e(_receiver.e_posMarker, lsq.variance.block<3>(PosKey, PosKey));
                if (lsq.variance.hasRows(VelKey))
                {
                    sppSol->setVelocityAndStdDev_e(_receiver.e_vel, lsq.variance.block<3>(VelKey, VelKey));
                    sppSol->setPosVelCovarianceMatrix_e(lsq.variance(PosVelKey, PosVelKey));
                }
                else
                {
                    sppSol->setPosCovarianceMatrix_e(lsq.variance(PosKey, PosKey));
                }

                sppSol->satData.reserve(observations.satellites.size());
                for (const auto& [satSigId, signalObs] : observations.signals)
                {
                    if (std::find_if(sppSol->satData.begin(), sppSol->satData.end(),
                                     [&satSigId = satSigId](const auto& satIdData) { return satIdData.first == satSigId.toSatId(); })
                        == sppSol->satData.end())
                    {
                        sppSol->satData.emplace_back(satSigId.toSatId(),
                                                     SppSolution::SatData{ .satElevation = signalObs.recvObs.at(Rover)->satElevation(_receiver.e_posMarker, _receiver.lla_posMarker),
                                                                           .satAzimuth = signalObs.recvObs.at(Rover)->satAzimuth(_receiver.e_posMarker, _receiver.lla_posMarker) });
                    }
                }

                break;
            }
        }
        else // if (_estimatorType == EstimatorType::KalmanFilter)
        {
            _kalmanFilter.update(measKeys, H, R, dz, nameId);

            if (double posDiff = (_kalmanFilter.getState()(PosKey) - _receiver.e_posMarker).norm();
                posDiff > 100)
            {
                std::string msg = fmt::format("Potential clock jump detected. Reinitializing KF with WLSQ.\nPosition difference to previous epoch {:.1f}m", posDiff);
                LOG_WARN("{}: [{}] {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), msg);
                sppSol->addEvent(msg);
                // _kalmanFilter.deinitialize();
                // nIter = N_ITER_MAX_LSQ + 1;
                // continue;
            }

            assignKalmanFilterResult(_kalmanFilter.getState(), _kalmanFilter.getErrorCovarianceMatrix(), nameId);
            sppSol->setPositionAndStdDev_e(_receiver.e_posMarker, _kalmanFilter.getErrorCovarianceMatrix().block<3>(PosKey, PosKey));
            sppSol->setVelocityAndStdDev_e(_receiver.e_vel, _kalmanFilter.getErrorCovarianceMatrix().block<3>(VelKey, VelKey));
            sppSol->setPosVelCovarianceMatrix_e(_kalmanFilter.getErrorCovarianceMatrix()(PosVelKey, PosVelKey));

            sppSol->satData.reserve(observations.satellites.size());
            for (const auto& [satSigId, signalObs] : observations.signals)
            {
                if (std::find_if(sppSol->satData.begin(), sppSol->satData.end(),
                                 [&satSigId = satSigId](const auto& satIdData) { return satIdData.first == satSigId.toSatId(); })
                    == sppSol->satData.end())
                {
                    sppSol->satData.emplace_back(satSigId.toSatId(),
                                                 SppSolution::SatData{ .satElevation = signalObs.recvObs.at(Rover)->satElevation(_receiver.e_posMarker, _receiver.lla_posMarker),
                                                                       .satAzimuth = signalObs.recvObs.at(Rover)->satAzimuth(_receiver.e_posMarker, _receiver.lla_posMarker) });
                }
            }
        }
    }

    sppSol->recvClk = _receiver.recvClk;
    sppSol->interFrequencyBias = _receiver.interFrequencyBias;

#if LOG_LEVEL <= LOG_LEVEL_DATA
    if (sppSol->e_position() != _receiver.e_posMarker)
    {
        LOG_DATA("{}: [{}] Receiver:   e_pos    = {:.6f}m, {:.6f}m, {:.6f}m", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
                 _receiver.e_posMarker(0), _receiver.e_posMarker(1), _receiver.e_posMarker(2));
        LOG_DATA("{}: [{}] Receiver: lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
                 rad2deg(_receiver.lla_posMarker.x()), rad2deg(_receiver.lla_posMarker.y()), _receiver.lla_posMarker.z());
    }
    if (sppSol->e_velocity() != _receiver.e_vel)
    {
        LOG_DATA("{}: [{}] Receiver:   e_vel    = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_vel.transpose());
    }
    for (const auto& satSys : _receiver.recvClk.satelliteSystems)
    {
        if (*sppSol->recvClk.biasFor(satSys) != *_receiver.recvClk.biasFor(satSys))
        {
            LOG_DATA("{}: [{}] Receiver:   clkBias  = {} s", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), *_receiver.recvClk.biasFor(satSys));
        }
        if (*sppSol->recvClk.driftFor(satSys) != *_receiver.recvClk.driftFor(satSys))
        {
            LOG_DATA("{}: [{}] Receiver:   clkDrift = {} s/s", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), *_receiver.recvClk.driftFor(satSys));
        }
    }
#endif

    LOG_DATA("{}: [{}] Solution:   e_pos    = {:.6f}m, {:.6f}m, {:.6f}m", nameId, sppSol->insTime.toYMDHMS(GPST),
             sppSol->e_position()(0), sppSol->e_position()(1), sppSol->e_position()(2));
    LOG_DATA("{}: [{}] Solution: lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, sppSol->insTime.toYMDHMS(GPST),
             rad2deg(sppSol->latitude()), rad2deg(sppSol->longitude()), sppSol->altitude());
    LOG_DATA("{}: [{}] Solution:   e_vel    = {}", nameId, sppSol->insTime.toYMDHMS(GPST), sppSol->e_velocity().transpose());
    for (size_t i = 0; i < sppSol->recvClk.satelliteSystems.size(); i++) // NOLINT
    {
        LOG_DATA("{}: [{}] Solution:   clkBias  [{:5}] = {} s", nameId, sppSol->insTime.toYMDHMS(GPST),
                 sppSol->recvClk.satelliteSystems.at(i), sppSol->recvClk.bias.at(i));
    }
    for (size_t i = 0; i < sppSol->recvClk.satelliteSystems.size(); i++) // NOLINT
    {
        LOG_DATA("{}: [{}] Solution:   clkDrift [{:5}] = {} s/s", nameId, sppSol->insTime.toYMDHMS(GPST),
                 sppSol->recvClk.satelliteSystems.at(i), sppSol->recvClk.drift.at(i));
    }
    for ([[maybe_unused]] const auto& freq : sppSol->interFrequencyBias)
    {
        LOG_DATA("{}: [{}] Solution:   IFBBias [{:5}] = {} s", nameId, sppSol->insTime.toYMDHMS(GPST), freq.first, freq.second.value);
    }

    return sppSol;
}

bool Algorithm::canCalculateVelocity(size_t nDoppMeas) const
{
    if (_estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized()) { return true; }

    return _obsFilter.isObsTypeUsed(GnssObs::Doppler) && nDoppMeas >= 4;
}

bool Algorithm::canEstimateInterFrequencyBias() const
{
    for (const auto& satSys : SatelliteSystem::GetAll())
    {
        auto filter = Frequency(_obsFilter.getFrequencyFilter() & satSys);
        if (filter.count() > 1) { return true; }
    }

    return false;
}

void Algorithm::updateInterFrequencyBiases(const Observations& observations, [[maybe_unused]] const std::string& nameId)
{
    if (_estimateInterFreqBiases)
    {
        std::set<Frequency> observedFrequencies;
        Frequency allObservedFrequencies;
        for (const auto& obs : observations.signals)
        {
            observedFrequencies.insert(obs.first.freq());
            allObservedFrequencies |= obs.first.freq();
        }
        LOG_TRACE("{}: Observed frequencies {}", nameId, fmt::join(observedFrequencies, ", "));

        for (const auto& freq : observedFrequencies)
        {
            if (!freq.isFirstFrequency(allObservedFrequencies) && !_receiver.interFrequencyBias.contains(freq))
            {
                LOG_TRACE("{}: Estimating Inter-Frequency bias for {}", nameId, freq);
                _receiver.interFrequencyBias.emplace(freq, UncertainValue<double>{});
                _kalmanFilter.addInterFrequencyBias(freq);
            }
        }
    }
}

std::vector<States::StateKeyType> Algorithm::determineStateKeys(const std::set<SatelliteSystem>& usedSatSystems, size_t nDoppMeas,
                                                                [[maybe_unused]] const std::string& nameId) const
{
    if (_estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized())
    {
        LOG_DATA("{}: stateKeys = [{}]", nameId, joinToString(_kalmanFilter.getStateKeys()));
        return _kalmanFilter.getStateKeys();
    }

    std::vector<States::StateKeyType> stateKeys = PosKey;
    stateKeys.reserve(stateKeys.size() + 1
                      + canCalculateVelocity(nDoppMeas) * (VelKey.size() + 1)
                      + usedSatSystems.size() * (1 + canCalculateVelocity(nDoppMeas)));
    if (canCalculateVelocity(nDoppMeas))
    {
        std::copy(VelKey.cbegin(), VelKey.cend(), std::back_inserter(stateKeys));
    }

    for (const auto& satSys : usedSatSystems)
    {
        stateKeys.emplace_back(Keys::RecvClkBias{ satSys });
        if (canCalculateVelocity(nDoppMeas)) { stateKeys.emplace_back(Keys::RecvClkDrift{ satSys }); }
    }
    for (const auto& freq : _receiver.interFrequencyBias)
    {
        stateKeys.emplace_back(Keys::InterFreqBias{ freq.first });
    }

    LOG_DATA("{}: stateKeys = [{}]", nameId, joinToString(stateKeys));
    return stateKeys;
}

std::vector<Meas::MeasKeyTypes> Algorithm::determineMeasKeys(const Observations& observations, size_t nPsrMeas, size_t nDoppMeas, [[maybe_unused]] const std::string& nameId) const
{
    std::vector<Meas::MeasKeyTypes> measKeys;
    measKeys.reserve(nPsrMeas + nDoppMeas);

    if (_obsFilter.isObsTypeUsed(GnssObs::Pseudorange))
    {
        for (const auto& signalObs : observations.signals)
        {
            if (signalObs.second.recvObs.at(Rover)->obs.contains(GnssObs::Pseudorange))
            {
                measKeys.emplace_back(Meas::Psr{ signalObs.first });
            }
        }
    }
    if (_obsFilter.isObsTypeUsed(GnssObs::Doppler))
    {
        for (const auto& signalObs : observations.signals)
        {
            if (signalObs.second.recvObs.at(Rover)->obs.contains(GnssObs::Doppler))
            {
                measKeys.emplace_back(Meas::Doppler{ signalObs.first });
            }
        }
    }

    LOG_DATA("{}: measKeys = [{}]", nameId, joinToString(measKeys));
    return measKeys;
}

KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyType> Algorithm::calcMatrixH(const std::vector<States::StateKeyType>& stateKeys,
                                                                               const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                               const Observations& observations,
                                                                               const std::string& nameId) const
{
    KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyType> H(Eigen::MatrixXd::Zero(static_cast<int>(measKeys.size()),
                                                                                    static_cast<int>(stateKeys.size())),
                                                              measKeys, stateKeys);

    for (const auto& [satSigId, signalObs] : observations.signals)
    {
        auto satId = satSigId.toSatId();

        const auto& roverObs = signalObs.recvObs.at(Rover);
        for (const auto& [obsType, obsData] : roverObs->obs)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                H.block<3>(Meas::Psr{ satSigId }, PosKey) = -roverObs->e_pLOS(_receiver.e_posMarker).transpose();
                H(Meas::Psr{ satSigId }, Keys::RecvClkBias{ satId.satSys }) = 1;
                if (_receiver.interFrequencyBias.contains(satSigId.freq()))
                {
                    H(Meas::Psr{ satSigId }, Keys::InterFreqBias{ satSigId.freq() }) = 1;
                }
                break;
            case GnssObs::Doppler:
                H.block<3>(Meas::Doppler{ satSigId }, PosKey) = -roverObs->e_vLOS(_receiver.e_posMarker, _receiver.e_vel).transpose();
                H.block<3>(Meas::Doppler{ satSigId }, VelKey) = -roverObs->e_pLOS(_receiver.e_posMarker).transpose();
                H(Meas::Doppler{ satSigId }, Keys::RecvClkDrift{ satId.satSys }) = 1;
                break;
            case GnssObs::Carrier:
            case GnssObs::ObservationType_COUNT:
                LOG_CRITICAL("{}: [{}] is not supported by the SPP algorithm as observation type.", nameId, obsType);
                break;
            }
        }
    }

    LOG_DATA("{}: H =\n{}", nameId, H);

    return H;
}

KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes> Algorithm::calcMatrixR(const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                             const Observations& observations,
                                                                             const std::string& nameId)
{
    KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes> R(Eigen::MatrixXd::Zero(static_cast<int>(measKeys.size()),
                                                                                  static_cast<int>(measKeys.size())),
                                                            measKeys);

    for (const auto& [satSigId, signalObs] : observations.signals)
    {
        for (const auto& [obsType, obsData] : signalObs.recvObs.at(Rover)->obs)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                R(Meas::Psr{ satSigId }, Meas::Psr{ satSigId }) = obsData.measVar;
                break;
            case GnssObs::Doppler:
                R(Meas::Doppler{ satSigId }, Meas::Doppler{ satSigId }) = obsData.measVar;
                break;
            case GnssObs::Carrier:
            case GnssObs::ObservationType_COUNT:
                LOG_CRITICAL("{}: These observation types are not supported by the SPP algorithm", nameId);
                break;
            }
        }
    }

    LOG_DATA("{}: R =\n{}", nameId, R);

    return R;
}

KeyedVectorXd<Meas::MeasKeyTypes> Algorithm::calcMeasInnovation(const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                const Observations& observations,
                                                                const std::string& nameId)
{
    KeyedVectorXd<Meas::MeasKeyTypes> dz(Eigen::VectorXd::Zero(static_cast<int>(measKeys.size())), measKeys);

    for (const auto& [satSigId, signalObs] : observations.signals)
    {
        for (const auto& [obsType, obsData] : signalObs.recvObs.at(Rover)->obs)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                dz(Meas::Psr{ satSigId }) = obsData.measurement - obsData.estimate;
                break;
            case GnssObs::Doppler:
                dz(Meas::Doppler{ satSigId }) = obsData.measurement - obsData.estimate;
                break;
            case GnssObs::Carrier:
            case GnssObs::ObservationType_COUNT:
                LOG_CRITICAL("{}: These observation types are not supported by the SPP algorithm", nameId);
                break;
            }
        }
    }

    LOG_DATA("{}: dz =\n{}", nameId, dz);

    return dz;
}

void Algorithm::assignLeastSquaresResult(const KeyedVectorXd<States::StateKeyType>& state,
                                         const KeyedMatrixXd<States::StateKeyType, States::StateKeyType>& variance,
                                         const Eigen::Vector3d& e_oldPos,
                                         size_t nParams, size_t nUniqueDopplerMeas, double dt,
                                         [[maybe_unused]] const std::string& nameId)
{
    _receiver.e_posMarker += state.segment<3>(PosKey);
    _receiver.lla_posMarker = trafo::ecef2lla_WGS84(_receiver.e_posMarker);
    for (const auto& s : state.rowKeys())
    {
        if (const auto* bias = std::get_if<Keys::RecvClkBias>(&s))
        {
            size_t idx = _receiver.recvClk.getIdx(bias->satSys).value();
            _receiver.recvClk.bias.at(idx) += state(*bias) / InsConst::C;
            if (variance(*bias, *bias) < 0)
            {
                _receiver.recvClk.biasStdDev.at(idx) = 1000 / InsConst::C;
                LOG_WARN("{}: Negative variance for {}. Defauting to {:.0f} [m]", nameId, *bias, _receiver.recvClk.biasStdDev.at(idx) * InsConst::C);
            }
            else
            {
                _receiver.recvClk.biasStdDev.at(idx) = std::sqrt(variance(*bias, *bias)) / InsConst::C;
            }
            LOG_DATA("{}: Setting Clk Bias  [{}] = {} [s] (StdDev = {})", nameId, bias->satSys, _receiver.recvClk.bias.at(idx), _receiver.recvClk.biasStdDev.at(idx));
        }
        else if (const auto* bias = std::get_if<Keys::InterFreqBias>(&s))
        {
            auto& freqDiff = _receiver.interFrequencyBias.at(bias->freq);
            freqDiff.value += state(*bias) / InsConst::C;
            if (variance(*bias, *bias) < 0)
            {
                freqDiff.stdDev = 1000 / InsConst::C;
                LOG_WARN("{}: Negative variance for {}. Defauting to {:.0f} [m]", nameId, *bias, freqDiff.stdDev * InsConst::C);
            }
            else
            {
                freqDiff.stdDev = std::sqrt(variance(*bias, *bias)) / InsConst::C;
            }
            LOG_DATA("{}: Setting IFB Bias  [{}] = {} [s] (StdDev = {})", nameId, bias->freq, freqDiff.value, freqDiff.stdDev);
        }
    }

    if (nUniqueDopplerMeas >= nParams)
    {
        _receiver.e_vel += state.segment<3>(VelKey);
        for (const auto& s : state.rowKeys())
        {
            if (const auto* drift = std::get_if<Keys::RecvClkDrift>(&s))
            {
                size_t idx = _receiver.recvClk.getIdx(drift->satSys).value();
                _receiver.recvClk.drift.at(idx) += state(*drift) / InsConst::C;
                if (variance(*drift, *drift) < 0)
                {
                    _receiver.recvClk.driftStdDev.at(idx) = 1000 / InsConst::C;
                    LOG_WARN("{}: Negative variance for {}. Defauting to {:.0f} [m/s]", nameId, *drift, _receiver.recvClk.driftStdDev.at(idx) * InsConst::C);
                }
                else
                {
                    _receiver.recvClk.driftStdDev.at(idx) = std::sqrt(variance(*drift, *drift)) / InsConst::C;
                }
                LOG_DATA("{}: Setting Clk Drift [{}] = {} [s/s] (StdDev = {})", nameId, drift->satSys,
                         _receiver.recvClk.drift.at(idx), _receiver.recvClk.driftStdDev.at(idx));
            }
        }
    }
    else
    {
        if (dt > 1e-6 && !e_oldPos.isZero())
        {
            if (_obsFilter.isObsTypeUsed(GnssObs::Doppler))
            {
                LOG_TRACE("{}: [{}] SPP has only {} satellites with doppler observations ({} needed). Calculating velocity as position difference.",
                          nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), nUniqueDopplerMeas, nParams);
            }
            LOG_DATA("{}: [{}] e_oldPos = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), e_oldPos.transpose());
            LOG_DATA("{}: [{}] e_newPos = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_posMarker.transpose());
            LOG_DATA("{}: [{}] dt = {}s", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), dt);
            _receiver.e_vel = (_receiver.e_posMarker - e_oldPos) / dt;
            LOG_DATA("{}: [{}] e_vel = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_vel.transpose());
        }
    }

    LOG_DATA("{}: [{}] Assigning solution to _receiver", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST));
    LOG_DATA("{}: [{}]     e_pos    = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_posMarker.transpose());
    LOG_DATA("{}: [{}]     e_vel    = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_vel.transpose());
    LOG_DATA("{}: [{}]   lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
             rad2deg(_receiver.lla_posMarker.x()), rad2deg(_receiver.lla_posMarker.y()), _receiver.lla_posMarker.z());
    for ([[maybe_unused]] const auto& satSys : _receiver.recvClk.satelliteSystems)
    {
        LOG_DATA("{}: [{}]     clkBias  = {} [s] (StdDev = {})", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
                 *_receiver.recvClk.biasFor(satSys), *_receiver.recvClk.biasStdDevFor(satSys));
        LOG_DATA("{}: [{}]     clkDrift = {} [s/s] (StdDev = {})", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
                 *_receiver.recvClk.driftFor(satSys), *_receiver.recvClk.driftStdDevFor(satSys));
    }

    for ([[maybe_unused]] const auto& freq : _receiver.interFrequencyBias)
    {
        LOG_DATA("{}: [{}]     IFBBias [{:3}] = {} [s] (StdDev = {})", nameId,
                 _receiver.gnssObs->insTime.toYMDHMS(GPST), freq.first, freq.second.value, freq.second.stdDev);
    }
}

void Algorithm::assignKalmanFilterResult(const KeyedVectorXd<States::StateKeyType>& state,
                                         const KeyedMatrixXd<States::StateKeyType, States::StateKeyType>& variance,
                                         [[maybe_unused]] const std::string& nameId)
{
    _receiver.e_posMarker = state(PosKey);
    _receiver.lla_posMarker = trafo::ecef2lla_WGS84(_receiver.e_posMarker);
    _receiver.e_vel = state(VelKey);
    for (const auto& s : state.rowKeys())
    {
        if (const auto* bias = std::get_if<Keys::RecvClkBias>(&s))
        {
            size_t idx = _receiver.recvClk.getIdx(bias->satSys).value();
            _receiver.recvClk.bias.at(idx) = state(*bias) / InsConst::C;
            if (variance(*bias, *bias) < 0)
            {
                _receiver.recvClk.biasStdDev.at(idx) = 1000 / InsConst::C;
                LOG_WARN("{}: Negative variance for {}. Defauting to {:.0f} [m]", nameId, *bias, _receiver.recvClk.biasStdDev.at(idx) * InsConst::C);
            }
            else
            {
                _receiver.recvClk.biasStdDev.at(idx) = std::sqrt(variance(*bias, *bias)) / InsConst::C;
            }
            LOG_DATA("{}: Setting Clock Bias  [{}] = {}", nameId, bias->satSys, _receiver.recvClk.bias.at(idx));
        }
        else if (const auto* drift = std::get_if<Keys::RecvClkDrift>(&s))
        {
            size_t idx = _receiver.recvClk.getIdx(drift->satSys).value();
            _receiver.recvClk.drift.at(idx) = state(*drift) / InsConst::C;
            if (variance(*drift, *drift) < 0)
            {
                _receiver.recvClk.driftStdDev.at(idx) = 1000 / InsConst::C;
                LOG_WARN("{}: Negative variance for {}. Defauting to {:.0f} [m/s]", nameId, *drift, _receiver.recvClk.driftStdDev.at(idx) * InsConst::C);
            }
            else
            {
                _receiver.recvClk.driftStdDev.at(idx) = std::sqrt(variance(*drift, *drift)) / InsConst::C;
            }
            LOG_DATA("{}: Setting Clock Drift [{}] = {}", nameId, drift->satSys, _receiver.recvClk.drift.at(idx));
        }
        else if (const auto* bias = std::get_if<Keys::InterFreqBias>(&s))
        {
            auto& freqDiff = _receiver.interFrequencyBias.at(bias->freq);
            freqDiff.value = state(*bias) / InsConst::C;
            if (variance(*bias, *bias) < 0)
            {
                freqDiff.stdDev = 1000 / InsConst::C;
                LOG_WARN("{}: Negative variance for {}. Defauting to {:.0f} [m/s]", nameId, *bias, freqDiff.stdDev * InsConst::C);
            }
            else
            {
                freqDiff.stdDev = std::sqrt(variance(*bias, *bias)) / InsConst::C;
            }
            LOG_DATA("{}: Setting Inter-Freq Bias [{}] = {}", nameId, bias->freq, freqDiff.value);
        }
    }

    LOG_DATA("{}: [{}] Assigning solution to _receiver", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST));
    LOG_DATA("{}: [{}]     e_pos    = {:.6f}m, {:.6f}m, {:.6f}m", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
             _receiver.e_posMarker(0), _receiver.e_posMarker(1), _receiver.e_posMarker(2));
    LOG_DATA("{}: [{}]     e_vel    = {}", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), _receiver.e_vel.transpose());
    LOG_DATA("{}: [{}]   lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST),
             rad2deg(_receiver.lla_posMarker.x()), rad2deg(_receiver.lla_posMarker.y()), _receiver.lla_posMarker.z());
    for ([[maybe_unused]] const auto& satSys : _receiver.recvClk.satelliteSystems)
    {
        LOG_DATA("{}: [{}]     clkBias  = {} s", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), *_receiver.recvClk.biasFor(satSys));
        LOG_DATA("{}: [{}]     clkDrift = {} s/s", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), *_receiver.recvClk.driftFor(satSys));
    }

    for ([[maybe_unused]] const auto& freq : _receiver.interFrequencyBias)
    {
        LOG_DATA("{}: [{}]     IFBBias [{:3}] = {} s", nameId, _receiver.gnssObs->insTime.toYMDHMS(GPST), freq.first, freq.second.value);
    }
}

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const Algorithm& obj)
{
    j = json{
        { "obsFilter", obj._obsFilter },
        { "obsEstimator", obj._obsEstimator },
        { "estimatorType", obj._estimatorType },
        { "kalmanFilter", obj._kalmanFilter },
        { "estimateInterFrequencyBiases", obj._estimateInterFreqBiases },
    };
}
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, Algorithm& obj)
{
    if (j.contains("obsFilter")) { j.at("obsFilter").get_to(obj._obsFilter); }
    if (j.contains("obsEstimator")) { j.at("obsEstimator").get_to(obj._obsEstimator); }
    if (j.contains("estimatorType")) { j.at("estimatorType").get_to(obj._estimatorType); }
    if (j.contains("kalmanFilter")) { j.at("kalmanFilter").get_to(obj._kalmanFilter); }
    if (j.contains("estimateInterFrequencyBiases")) { j.at("estimateInterFrequencyBiases").get_to(obj._estimateInterFreqBiases); }
}

} // namespace SPP

const char* to_string(SPP::Algorithm::EstimatorType estimatorType)
{
    switch (estimatorType)
    {
    case SPP::Algorithm::EstimatorType::LeastSquares:
        return "Least Squares";
    case SPP::Algorithm::EstimatorType::WeightedLeastSquares:
        return "Weighted Least Squares";
    case SPP::Algorithm::EstimatorType::KalmanFilter:
        return "Kalman Filter";
    case SPP::Algorithm::EstimatorType::COUNT:
        break;
    }
    return "";
}

const char* to_string(SPP::Algorithm::ReceiverType receiver)
{
    switch (receiver)
    {
    case SPP::Algorithm::ReceiverType::Rover:
        return "Rover";
    case SPP::Algorithm::ReceiverType::ReceiverType_COUNT:
        break;
    }
    return "";
}

} // namespace NAV

std::ostream& operator<<(std::ostream& os, const NAV::SPP::Algorithm::EstimatorType& obj)
{
    return os << fmt::format("{}", obj);
}

std::ostream& operator<<(std::ostream& os, const NAV::SPP::Algorithm::ReceiverType& obj)
{
    return os << fmt::format("{}", obj);
}