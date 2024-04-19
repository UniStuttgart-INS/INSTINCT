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

    changed |= _obsEstimator.ShowGuiWidgets(id, itemWidth);

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
    for (auto& receiver : _receiver) { receiver = Receiver(receiver.type); }
    _kalmanFilter.reset();
    _lastUpdate.reset();
}

std::shared_ptr<SppSolution> Algorithm::calcSppSolution(const std::shared_ptr<const GnssObs>& gnssObs,
                                                        const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                        const std::string& nameId)
{
    _receiver[Rover].gnssObs = gnssObs;

    if (gnssNavInfos.empty())
    {
        LOG_ERROR("{}: [{}] SPP cannot calculate position because no navigation data provided.", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
        return nullptr;
    }
    LOG_DATA("{}: [{}] Calculating SPP", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));

    // Collection of all connected Ionospheric Corrections
    IonosphericCorrections ionosphericCorrections(gnssNavInfos);

    double dt = _lastUpdate.empty() ? 0.0 : static_cast<double>((_receiver[Rover].gnssObs->insTime - _lastUpdate).count());
    LOG_DATA("{}: dt = {}s", nameId, dt);
    _lastUpdate = _receiver[Rover].gnssObs->insTime;

    auto sppSol = std::make_shared<SppSolution>();
    sppSol->insTime = _receiver[Rover].gnssObs->insTime;

    if (_estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized())
    {
        _kalmanFilter.predict(dt, _receiver[Rover].lla_pos, nameId);
        assignKalmanFilterResult(_kalmanFilter.getState(), _kalmanFilter.getErrorCovarianceMatrix(), nameId);
    }

    constexpr size_t N_ITER_MAX_LSQ = 10;
    size_t nIter = _estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized() ? 1 : N_ITER_MAX_LSQ;
    Eigen::Vector3d e_oldPos = _receiver[Rover].e_pos;
    for (size_t iteration = 0; iteration < nIter; iteration++)
    {
        LOG_DATA("{}: [{}] iteration {}/{}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), iteration + 1, nIter);
        LOG_DATA("{}: [{}]   e_pos    = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_pos.transpose());
        LOG_DATA("{}: [{}]   e_vel    = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_vel.transpose());
        LOG_DATA("{}: [{}] lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                 rad2deg(_receiver[Rover].lla_pos.x()), rad2deg(_receiver[Rover].lla_pos.y()), _receiver[Rover].lla_pos.z());
        LOG_DATA("{}: [{}]   clkBias  = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.bias.value);
        LOG_DATA("{}: [{}]   clkDrift = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.drift.value);
        for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffBias.size(); i++)
        {
            LOG_DATA("{}: [{}]   ISBBias  [{:5}] = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), _receiver[Rover].recvClk.sysTimeDiffBias.at(i).value);
        }
        for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffDrift.size(); i++)
        {
            LOG_DATA("{}: [{}]   ISBDrift [{:5}] = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), _receiver[Rover].recvClk.sysTimeDiffDrift.at(i).value);
        }

        auto observations = _obsFilter.selectObservationsForCalculation(_receiver, gnssNavInfos, nameId, e_oldPos.isZero());
        if (observations.signals.empty())
        {
            LOG_ERROR("{}: [{}] SPP cannot calculate position because no valid observations. Try changing filter settings or reposition your antenna.",
                      nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
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

        if (size_t nPsrMeas = observations.nObservablesUniqueSatellite[GnssObs::Pseudorange];
            (_estimatorType != EstimatorType::KalmanFilter || !_kalmanFilter.isInitialized()) && nPsrMeas <= nParams)
        {
            LOG_ERROR("{}: [{}] SPP cannot calculate position because only {} satellites with pseudorange observations ({} needed). Try changing filter settings or reposition your antenna.",
                      nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), nPsrMeas, nParams + 1);
            return nullptr;
        }

        updateInterSystemTimeDifferences(observations.systems, observations.nObservables[GnssObs::Doppler], nameId);

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
                LOG_WARN("{}: [{}] {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), msg);
                _kalmanFilter.setClockBiasErrorCovariance(1e6);
                sppSol->addEvent(msg);
            }
        }

        if (_estimatorType != EstimatorType::KalmanFilter || !_kalmanFilter.isInitialized())
        {
            KeyedLeastSquaresResult<double, States::StateKeyTypes> lsq;
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

            bool accuracyAchieved = lsq.solution(all).norm() < 1e-4;
            if (accuracyAchieved) { LOG_DATA("{}: [{}] Accuracy achieved on iteration {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), iteration + 1); }
            else { LOG_DATA("{}: [{}] Bad accuracy on iteration {}: {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), iteration + 1, lsq.solution(all).norm()); }
            if (accuracyAchieved || iteration == nIter - 1)
            {
                if (_estimatorType == EstimatorType::KalmanFilter && !_kalmanFilter.isInitialized()
                    && sppSol->nMeasPsr > nParams // Variance can only be calculated if more measurements than parameters
                    && (!_obsFilter.isObsTypeUsed(GnssObs::Doppler) || sppSol->nMeasDopp > nParams))
                {
                    LOG_TRACE("{}: [{}] Initializing KF with LSQ solution", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
                    KeyedVectorXd<States::StateKeyTypes> x(Eigen::VectorXd::Zero(lsq.solution.rows()), lsq.solution.rowKeys());
                    KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes> P(Eigen::MatrixXd::Zero(lsq.variance.rows(), lsq.variance.cols()),
                                                                                  lsq.variance.rowKeys(), lsq.variance.colKeys());
                    x.segment<3>(States::Pos) = _receiver[Rover].e_pos;
                    if (x.hasAnyRows(States::Vel)) { x.segment<3>(States::Vel) = _receiver[Rover].e_vel; }
                    x(States::RecvClkErr) = _receiver[Rover].recvClk.bias.value * InsConst<>::C;
                    if (x.hasRow(States::RecvClkDrift)) { x(States::RecvClkDrift) = _receiver[Rover].recvClk.drift.value * InsConst<>::C; }
                    for (const auto& state : x.rowKeys())
                    {
                        if (const auto* bias = std::get_if<States::InterSysBias>(&state))
                        {
                            x(*bias) = _receiver[Rover].recvClk.sysTimeDiffBias.at(bias->satSys.toEnumeration()).value * InsConst<>::C;
                        }
                        else if (const auto* drift = std::get_if<States::InterSysDrift>(&state))
                        {
                            x(*drift) = _receiver[Rover].recvClk.sysTimeDiffDrift.at(drift->satSys.toEnumeration()).value * InsConst<>::C;
                        }
                        else if (const auto* bias = std::get_if<States::InterFreqBias>(&state))
                        {
                            x(*bias) = _receiver[Rover].interFrequencyBias.at(bias->freq).value * InsConst<>::C;
                        }
                    }

                    _kalmanFilter.initialize(x, lsq.variance);
                    LOG_DATA("{}: x =\n{}", nameId, _kalmanFilter.getState().transposed());
                    LOG_DATA("{}: P =\n{}", nameId, _kalmanFilter.getErrorCovarianceMatrix());
                }
                sppSol->setPositionAndStdDev_e(_receiver[Rover].e_pos, lsq.variance.block<3>(States::Pos, States::Pos));
                if (canCalculateVelocity(observations.nObservables[GnssObs::Doppler]))
                {
                    sppSol->setVelocityAndStdDev_e(_receiver[Rover].e_vel, lsq.variance.block<3>(States::Vel, States::Vel));
                }
                sppSol->setCovarianceMatrix(lsq.variance);

                sppSol->satData.reserve(observations.satellites.size());
                for (const auto& [satSigId, signalObs] : observations.signals)
                {
                    if (std::find_if(sppSol->satData.begin(), sppSol->satData.end(),
                                     [&satSigId = satSigId](const auto& satIdData) { return satIdData.first == satSigId.toSatId(); })
                        == sppSol->satData.end())
                    {
                        sppSol->satData.emplace_back(satSigId.toSatId(), SppSolution::SatData{ .satElevation = signalObs.recvObs[Rover].satElevation(),
                                                                                               .satAzimuth = signalObs.recvObs[Rover].satAzimuth() });
                    }
                }
                break;
            }
        }
        else // if (_estimatorType == EstimatorType::KalmanFilter)
        {
            _kalmanFilter.update(measKeys, H, R, dz, nameId);

            if (double posDiff = (_kalmanFilter.getState()(States::Pos) - _receiver[Rover].e_pos).norm();
                posDiff > 100)
            {
                std::string msg = fmt::format("Potential clock jump detected. Reinitializing KF with WLSQ.\nPosition difference to previous epoch {:.1f}m", posDiff);
                LOG_WARN("{}: [{}] {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), msg);
                sppSol->addEvent(msg);
                _kalmanFilter.deinitialize();
                nIter = N_ITER_MAX_LSQ + 1;
                continue;
            }

            assignKalmanFilterResult(_kalmanFilter.getState(), _kalmanFilter.getErrorCovarianceMatrix(), nameId);
            sppSol->setPositionAndStdDev_e(_receiver[Rover].e_pos, _kalmanFilter.getErrorCovarianceMatrix().block<3>(States::Pos, States::Pos));
            sppSol->setVelocityAndStdDev_e(_receiver[Rover].e_vel, _kalmanFilter.getErrorCovarianceMatrix().block<3>(States::Vel, States::Vel));
            sppSol->setCovarianceMatrix(_kalmanFilter.getErrorCovarianceMatrix());

            sppSol->satData.reserve(observations.satellites.size());
            for (const auto& [satSigId, signalObs] : observations.signals)
            {
                if (std::find_if(sppSol->satData.begin(), sppSol->satData.end(),
                                 [&satSigId = satSigId](const auto& satIdData) { return satIdData.first == satSigId.toSatId(); })
                    == sppSol->satData.end())
                {
                    sppSol->satData.emplace_back(satSigId.toSatId(), SppSolution::SatData{ .satElevation = signalObs.recvObs[Rover].satElevation(),
                                                                                           .satAzimuth = signalObs.recvObs[Rover].satAzimuth() });
                }
            }
        }
    }

    sppSol->recvClk = _receiver[Rover].recvClk;
    sppSol->interFrequencyBias = _receiver[Rover].interFrequencyBias;

#if LOG_LEVEL <= LOG_LEVEL_DATA
    if (sppSol->e_position() != _receiver[Rover].e_pos)
    {
        LOG_DATA("{}: [{}] Receiver:   e_pos    = {:.6f}m, {:.6f}m, {:.6f}m", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                 _receiver[Rover].e_pos(0), _receiver[Rover].e_pos(1), _receiver[Rover].e_pos(2));
        LOG_DATA("{}: [{}] Receiver: lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                 rad2deg(_receiver[Rover].lla_pos.x()), rad2deg(_receiver[Rover].lla_pos.y()), _receiver[Rover].lla_pos.z());
    }
    if (sppSol->e_velocity() != _receiver[Rover].e_vel)
    {
        LOG_DATA("{}: [{}] Receiver:   e_vel    = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_vel.transpose());
    }
    if (sppSol->recvClk.bias.value != _receiver[Rover].recvClk.bias.value)
    {
        LOG_DATA("{}: [{}] Receiver:   clkBias  = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.bias.value);
    }
    if (sppSol->recvClk.drift.value != _receiver[Rover].recvClk.drift.value)
    {
        LOG_DATA("{}: [{}] Receiver:   clkDrift = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.drift.value);
    }
#endif

    LOG_DATA("{}: [{}] Solution:   e_pos    = {:.6f}m, {:.6f}m, {:.6f}m", nameId, sppSol->insTime.toYMDHMS(GPST),
             sppSol->e_position()(0), sppSol->e_position()(1), sppSol->e_position()(2));
    LOG_DATA("{}: [{}] Solution:   e_vel    = {}", nameId, sppSol->insTime.toYMDHMS(GPST), sppSol->e_velocity().transpose());
    LOG_DATA("{}: [{}] Solution: lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, sppSol->insTime.toYMDHMS(GPST),
             rad2deg(sppSol->latitude()), rad2deg(sppSol->longitude()), sppSol->altitude());
    LOG_DATA("{}: [{}] Solution:   clkBias  = {} s", nameId, sppSol->insTime.toYMDHMS(GPST), sppSol->recvClk.bias.value);
    LOG_DATA("{}: [{}] Solution:   clkDrift = {} s/s", nameId, sppSol->insTime.toYMDHMS(GPST), sppSol->recvClk.drift.value);
    for (size_t i = 0; i < sppSol->recvClk.sysTimeDiffBias.size(); i++)
    {
        if (sppSol->recvClk.sysTimeDiffBias.at(i).value != 0.0)
        {
            LOG_DATA("{}: [{}] Solution:   ISBBias  [{:5}] = {} s", nameId, sppSol->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), sppSol->recvClk.sysTimeDiffBias.at(i).value);
        }
    }
    for (size_t i = 0; i < sppSol->recvClk.sysTimeDiffDrift.size(); i++)
    {
        if (sppSol->recvClk.sysTimeDiffDrift.at(i).value != 0.0)
        {
            LOG_DATA("{}: [{}] Solution:   ISBDrift [{:5}] = {} s/s", nameId, sppSol->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), sppSol->recvClk.sysTimeDiffDrift.at(i).value);
        }
    }
    for ([[maybe_unused]] const auto& freq : sppSol->interFrequencyBias)
    {
        LOG_DATA("{}: [{}] Solution:   IFBBias [{:5}] = {} s", nameId, sppSol->insTime.toYMDHMS(GPST), freq.first, freq.second.value);
    }

    return sppSol;
}

bool Algorithm::canCalculateVelocity(size_t nDoppMeas) const
{
    if (_estimatorType == EstimatorType::KalmanFilter) { return true; }

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

void Algorithm::updateInterSystemTimeDifferences(const std::set<SatelliteSystem>& usedSatSystems, size_t nDoppMeas, const std::string& nameId)
{
    SatelliteSystem oldRefSys = _receiver[Rover].recvClk.referenceTimeSatelliteSystem;
    if (_receiver[Rover].recvClk.referenceTimeSatelliteSystem == SatSys_None)
    {
        _receiver[Rover].recvClk.referenceTimeSatelliteSystem = *usedSatSystems.begin();
    }

    if (_estimatorType == EstimatorType::KalmanFilter)
    {
        _receiver[Rover].recvClk.referenceTimeSatelliteSystem = _kalmanFilter.updateInterSystemTimeDifferences(usedSatSystems,
                                                                                                               oldRefSys,
                                                                                                               *usedSatSystems.begin(),
                                                                                                               nameId);
        if (oldRefSys != _receiver[Rover].recvClk.referenceTimeSatelliteSystem)
        {
            for (const auto& state : _kalmanFilter.getStateKeys())
            {
                if (const auto* bias = std::get_if<States::InterSysBias>(&state))
                {
                    _receiver[Rover].recvClk.sysTimeDiffBias.at(bias->satSys.toEnumeration()).value = _kalmanFilter.getState()(*bias) / InsConst<>::C;
                    _receiver[Rover].recvClk.sysTimeDiffBias.at(bias->satSys.toEnumeration()).stdDev = std::sqrt(_kalmanFilter.getErrorCovarianceMatrix()(*bias, *bias)) / InsConst<>::C;
                }
                else if (const auto* drift = std::get_if<States::InterSysDrift>(&state))
                {
                    _receiver[Rover].recvClk.sysTimeDiffDrift.at(drift->satSys.toEnumeration()).value = _kalmanFilter.getState()(*drift) / InsConst<>::C;
                    _receiver[Rover].recvClk.sysTimeDiffDrift.at(drift->satSys.toEnumeration()).stdDev = std::sqrt(_kalmanFilter.getErrorCovarianceMatrix()(*drift, *drift)) / InsConst<>::C;
                }
            }
        }
    }
    else if (oldRefSys != *usedSatSystems.begin())
    {
        _receiver[Rover].recvClk.referenceTimeSatelliteSystem = *usedSatSystems.begin();
        auto refSatSysEnum = _receiver[Rover].recvClk.referenceTimeSatelliteSystem.toEnumeration();

        for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffBias.size(); ++i)
        {
            if (refSatSysEnum == i) { continue; }

            auto& sysBias = _receiver[Rover].recvClk.sysTimeDiffBias.at(i);
            LOG_DATA("{}: [{}] Bias  [{:5}] {:.3g} - {:.3g} = {:.3g}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)),
                     sysBias.value, _receiver[Rover].recvClk.sysTimeDiffBias.at(refSatSysEnum).value,
                     sysBias.value - _receiver[Rover].recvClk.sysTimeDiffBias.at(refSatSysEnum).value);
            sysBias.value -= _receiver[Rover].recvClk.sysTimeDiffBias.at(refSatSysEnum).value;
            sysBias.stdDev += _receiver[Rover].recvClk.sysTimeDiffBias.at(refSatSysEnum).stdDev;
        }
        if (canCalculateVelocity(nDoppMeas))
        {
            for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffDrift.size(); ++i)
            {
                if (refSatSysEnum == i) { continue; }

                auto& sysDrift = _receiver[Rover].recvClk.sysTimeDiffDrift.at(i);
                LOG_DATA("{}: [{}] Drift [{:5}] {:.3g} - {:.3g} = {:.3g}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                         SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)),
                         sysDrift.value, _receiver[Rover].recvClk.sysTimeDiffDrift.at(refSatSysEnum).value,
                         sysDrift.value - _receiver[Rover].recvClk.sysTimeDiffDrift.at(refSatSysEnum).value);
                sysDrift.value -= _receiver[Rover].recvClk.sysTimeDiffDrift.at(refSatSysEnum).value;
                sysDrift.stdDev += _receiver[Rover].recvClk.sysTimeDiffDrift.at(refSatSysEnum).stdDev;
            }
        }
    }

    if (oldRefSys != _receiver[Rover].recvClk.referenceTimeSatelliteSystem)
    {
        LOG_TRACE("{}: [{}] Using [{}] as new reference system for inter system clock biases", nameId,
                  _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.referenceTimeSatelliteSystem);
    }
}

void Algorithm::updateInterFrequencyBiases(const Observations& observations, [[maybe_unused]] const std::string& nameId)
{
    if (_estimateInterFreqBiases)
    {
        auto isFirstFrequency = [&](const Frequency& freq, const Frequency& filter) -> bool {
            Frequency_ f = filter & freq.getSatSys();
            f = Frequency_(f ^ Frequency_(freq));
            LOG_DATA("{}: Freq {} is {} in filter {}", nameId, freq, (f == Freq_None || Frequency_(freq) < f) ? "first" : "not first", filter);

            return f == Freq_None || Frequency_(freq) < f;
        };

        std::set<Frequency> observedFrequencies;
        for (const auto& obs : observations.signals) { observedFrequencies.insert(obs.first.freq()); }

        for (const auto& freq : observedFrequencies)
        {
            if (!isFirstFrequency(freq, _obsFilter.getFrequencyFilter()) && !_receiver[Rover].interFrequencyBias.contains(freq))
            {
                LOG_TRACE("{}: Estimating Inter-Frequency bias for {}", nameId, freq);
                _receiver[Rover].interFrequencyBias.emplace(freq, UncertainValue<double>{});
                _kalmanFilter.addInterFrequencyBias(freq);
            }
        }
    }
}

std::vector<States::StateKeyTypes> Algorithm::determineStateKeys(const std::set<SatelliteSystem>& usedSatSystems, size_t nDoppMeas,
                                                                 [[maybe_unused]] const std::string& nameId) const
{
    if (_estimatorType == EstimatorType::KalmanFilter && _kalmanFilter.isInitialized())
    {
        LOG_DATA("{}: stateKeys = [{}]", nameId, joinToString(_kalmanFilter.getStateKeys()));
        return _kalmanFilter.getStateKeys();
    }

    std::vector<States::StateKeyTypes> stateKeys = States::Pos;
    stateKeys.reserve(stateKeys.size() + 1
                      + canCalculateVelocity(nDoppMeas) * (States::Vel.size() + 1)
                      + (usedSatSystems.size() - 1) * (1 + canCalculateVelocity(nDoppMeas)));
    if (canCalculateVelocity(nDoppMeas))
    {
        std::copy(States::Vel.cbegin(), States::Vel.cend(), std::back_inserter(stateKeys));
    }
    stateKeys.emplace_back(States::RecvClkErr);
    if (canCalculateVelocity(nDoppMeas))
    {
        stateKeys.emplace_back(States::RecvClkDrift);
    }

    for (const auto& satSys : usedSatSystems)
    {
        if (satSys != _receiver[Rover].recvClk.referenceTimeSatelliteSystem)
        {
            stateKeys.emplace_back(States::InterSysBias{ satSys });
            if (canCalculateVelocity(nDoppMeas)) { stateKeys.emplace_back(States::InterSysDrift{ satSys }); }
        }
    }
    for (const auto& freq : _receiver[Rover].interFrequencyBias)
    {
        stateKeys.emplace_back(States::InterFreqBias{ freq.first });
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
            if (signalObs.second.recvObs.at(Rover).obs.contains(GnssObs::Pseudorange))
            {
                measKeys.emplace_back(Meas::Psr{ signalObs.first });
            }
        }
    }
    if (_obsFilter.isObsTypeUsed(GnssObs::Doppler))
    {
        for (const auto& signalObs : observations.signals)
        {
            if (signalObs.second.recvObs.at(Rover).obs.contains(GnssObs::Doppler))
            {
                measKeys.emplace_back(Meas::Doppler{ signalObs.first });
            }
        }
    }

    LOG_DATA("{}: measKeys = [{}]", nameId, joinToString(measKeys));
    return measKeys;
}

KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes> Algorithm::calcMatrixH(const std::vector<States::StateKeyTypes>& stateKeys,
                                                                                const std::vector<Meas::MeasKeyTypes>& measKeys,
                                                                                const Observations& observations,
                                                                                const std::string& nameId) const
{
    KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes> H(Eigen::MatrixXd::Zero(static_cast<int>(measKeys.size()),
                                                                                     static_cast<int>(stateKeys.size())),
                                                               measKeys, stateKeys);

    for (const auto& [satSigId, signalObs] : observations.signals)
    {
        auto satId = satSigId.toSatId();

        const auto& roverObs = signalObs.recvObs.at(Rover);
        for (const auto& [obsType, obsData] : roverObs.obs)
        {
            switch (obsType)
            {
            case GnssObs::Pseudorange:
                H.block<3>(Meas::Psr{ satSigId }, States::Pos) = -roverObs.e_pLOS().transpose();
                H(Meas::Psr{ satSigId }, States::RecvClkErr) = 1;
                if (satId.satSys != _receiver[Rover].recvClk.referenceTimeSatelliteSystem)
                {
                    H(Meas::Psr{ satSigId }, States::InterSysBias{ satId.satSys }) = 1;
                }
                if (_receiver[Rover].interFrequencyBias.contains(satSigId.freq()))
                {
                    H(Meas::Psr{ satSigId }, States::InterFreqBias{ satSigId.freq() }) = 1;
                }
                break;
            case GnssObs::Doppler:
                H.block<3>(Meas::Doppler{ satSigId }, States::Pos) = -roverObs.e_vLOS().transpose();
                H.block<3>(Meas::Doppler{ satSigId }, States::Vel) = -roverObs.e_pLOS().transpose();
                H(Meas::Doppler{ satSigId }, States::RecvClkDrift) = 1;
                if (satId.satSys != _receiver[Rover].recvClk.referenceTimeSatelliteSystem)
                {
                    H(Meas::Doppler{ satSigId }, States::InterSysDrift{ satId.satSys }) = 1;
                }
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
        for (const auto& [obsType, obsData] : signalObs.recvObs.at(Rover).obs)
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
        for (const auto& [obsType, obsData] : signalObs.recvObs.at(Rover).obs)
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

    LOG_DATA("{}: dz =\n{}", nameId, dz.transposed());

    return dz;
}

void Algorithm::assignLeastSquaresResult(const KeyedVectorXd<States::StateKeyTypes>& state,
                                         const KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes>& variance,
                                         const Eigen::Vector3d& e_oldPos,
                                         size_t nParams, size_t nUniqueDopplerMeas, double dt,
                                         [[maybe_unused]] const std::string& nameId)
{
    _receiver[Rover].e_pos += state.segment<3>(States::Pos);
    _receiver[Rover].lla_pos = trafo::ecef2lla_WGS84(_receiver[Rover].e_pos);
    _receiver[Rover].recvClk.bias.value += state(States::RecvClkErr) / InsConst<>::C;
    _receiver[Rover].recvClk.bias.stdDev = std::sqrt(variance(States::RecvClkErr, States::RecvClkErr)) / InsConst<>::C;
    for (const auto& s : state.rowKeys())
    {
        if (const auto* bias = std::get_if<States::InterSysBias>(&s))
        {
            auto& sysTimeDiff = _receiver[Rover].recvClk.sysTimeDiffBias.at(bias->satSys.toEnumeration());
            sysTimeDiff.value += state(*bias) / InsConst<>::C;
            sysTimeDiff.stdDev = std::sqrt(variance(*bias, *bias)) / InsConst<>::C;
            LOG_DATA("{}: Setting ISB Bias  [{}] = {}", nameId, bias->satSys, sysTimeDiff.value);
        }
        else if (const auto* bias = std::get_if<States::InterFreqBias>(&s))
        {
            auto& freqDiff = _receiver[Rover].interFrequencyBias.at(bias->freq);
            freqDiff.value += state(*bias) / InsConst<>::C;
            freqDiff.stdDev = std::sqrt(variance(*bias, *bias)) / InsConst<>::C;
            LOG_DATA("{}: Setting IFB Bias  [{}] = {}", nameId, bias->freq, freqDiff.value);
        }
    }

    if (nUniqueDopplerMeas >= nParams)
    {
        _receiver[Rover].e_vel += state.segment<3>(States::Vel);
        _receiver[Rover].recvClk.drift.value += state(States::RecvClkDrift) / InsConst<>::C;
        _receiver[Rover].recvClk.drift.stdDev = std::sqrt(variance(States::RecvClkDrift, States::RecvClkDrift)) / InsConst<>::C;
        for (const auto& s : state.rowKeys())
        {
            if (const auto* drift = std::get_if<States::InterSysDrift>(&s))
            {
                auto& sysTimeDrift = _receiver[Rover].recvClk.sysTimeDiffDrift.at(drift->satSys.toEnumeration());
                sysTimeDrift.value += state(*drift) / InsConst<>::C;
                sysTimeDrift.stdDev = std::sqrt(variance(*drift, *drift)) / InsConst<>::C;
                LOG_DATA("{}: Setting ISB Drift [{}] = {}", nameId, drift->satSys, sysTimeDrift.value);
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
                          nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), nUniqueDopplerMeas, nParams);
            }
            LOG_DATA("{}: [{}] e_oldPos = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), e_oldPos.transpose());
            LOG_DATA("{}: [{}] e_newPos = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_pos.transpose());
            LOG_DATA("{}: [{}] dt = {}s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), dt);
            _receiver[Rover].e_vel = (_receiver[Rover].e_pos - e_oldPos) / dt;
            LOG_DATA("{}: [{}] e_vel = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_vel.transpose());
        }
    }

    LOG_DATA("{}: [{}] Assigning solution to _receiver[Rover]", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
    LOG_DATA("{}: [{}]     e_pos    = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_pos.transpose());
    LOG_DATA("{}: [{}]     e_vel    = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_vel.transpose());
    LOG_DATA("{}: [{}]   lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
             rad2deg(_receiver[Rover].lla_pos.x()), rad2deg(_receiver[Rover].lla_pos.y()), _receiver[Rover].lla_pos.z());
    LOG_DATA("{}: [{}]     clkBias  = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.bias.value);
    LOG_DATA("{}: [{}]     clkDrift = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.drift.value);
    for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffBias.size(); i++)
    {
        if (_receiver[Rover].recvClk.sysTimeDiffBias.at(i).value != 0.0)
        {
            LOG_DATA("{}: [{}]     ISBBias  [{:5}] = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), _receiver[Rover].recvClk.sysTimeDiffBias.at(i).value);
        }
    }
    for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffDrift.size(); i++)
    {
        if (_receiver[Rover].recvClk.sysTimeDiffDrift.at(i).value != 0.0)
        {
            LOG_DATA("{}: [{}]     ISBDrift [{:5}] = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), _receiver[Rover].recvClk.sysTimeDiffDrift.at(i).value);
        }
    }

    for ([[maybe_unused]] const auto& freq : _receiver[Rover].interFrequencyBias)
    {
        LOG_DATA("{}: [{}]     IFBBias [{:3}] = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), freq.first, freq.second.value);
    }
}

void Algorithm::assignKalmanFilterResult(const KeyedVectorXd<States::StateKeyTypes>& state,
                                         const KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes>& variance,
                                         [[maybe_unused]] const std::string& nameId)
{
    _receiver[Rover].e_pos = state(States::Pos);
    _receiver[Rover].lla_pos = trafo::ecef2lla_WGS84(_receiver[Rover].e_pos);
    _receiver[Rover].e_vel = state(States::Vel);
    _receiver[Rover].recvClk.bias.value = state(States::RecvClkErr) / InsConst<>::C;
    _receiver[Rover].recvClk.bias.stdDev = std::sqrt(variance(States::RecvClkErr, States::RecvClkErr)) / InsConst<>::C;
    for (const auto& s : state.rowKeys())
    {
        if (const auto* bias = std::get_if<States::InterSysBias>(&s))
        {
            auto& sysTimeDiff = _receiver[Rover].recvClk.sysTimeDiffBias.at(bias->satSys.toEnumeration());
            sysTimeDiff.value = state(*bias) / InsConst<>::C;
            sysTimeDiff.stdDev = std::sqrt(variance(*bias, *bias)) / InsConst<>::C;
        }
        else if (const auto* bias = std::get_if<States::InterFreqBias>(&s))
        {
            auto& freqDiff = _receiver[Rover].interFrequencyBias.at(bias->freq);
            freqDiff.value = state(*bias) / InsConst<>::C;
            freqDiff.stdDev = std::sqrt(variance(*bias, *bias)) / InsConst<>::C;
        }
    }
    _receiver[Rover].recvClk.drift.value = state(States::RecvClkDrift) / InsConst<>::C;
    _receiver[Rover].recvClk.drift.stdDev = std::sqrt(variance(States::RecvClkDrift, States::RecvClkDrift)) / InsConst<>::C;
    for (const auto& s : state.rowKeys())
    {
        if (const auto* drift = std::get_if<States::InterSysDrift>(&s))
        {
            auto& sysTimeDrift = _receiver[Rover].recvClk.sysTimeDiffDrift.at(drift->satSys.toEnumeration());
            sysTimeDrift.value = state(*drift) / InsConst<>::C;
            sysTimeDrift.stdDev = std::sqrt(variance(*drift, *drift)) / InsConst<>::C;
        }
    }

    LOG_DATA("{}: [{}] Assigning solution to _receiver[Rover]", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST));
    LOG_DATA("{}: [{}]     e_pos    = {:.6f}m, {:.6f}m, {:.6f}m", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
             _receiver[Rover].e_pos(0), _receiver[Rover].e_pos(1), _receiver[Rover].e_pos(2));
    LOG_DATA("{}: [{}]     e_vel    = {}", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].e_vel.transpose());
    LOG_DATA("{}: [{}]   lla_pos    = {:.6f}°, {:.6f}°, {:.3f}m", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
             rad2deg(_receiver[Rover].lla_pos.x()), rad2deg(_receiver[Rover].lla_pos.y()), _receiver[Rover].lla_pos.z());
    LOG_DATA("{}: [{}]     clkBias  = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.bias.value);
    LOG_DATA("{}: [{}]     clkDrift = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), _receiver[Rover].recvClk.drift.value);

    for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffBias.size(); i++)
    {
        if (_receiver[Rover].recvClk.sysTimeDiffBias.at(i).value != 0.0)
        {
            LOG_DATA("{}: [{}]     ISBBias  [{:5}] = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), _receiver[Rover].recvClk.sysTimeDiffBias.at(i).value);
        }
    }
    for (size_t i = 0; i < _receiver[Rover].recvClk.sysTimeDiffDrift.size(); i++)
    {
        if (_receiver[Rover].recvClk.sysTimeDiffDrift.at(i).value != 0.0)
        {
            LOG_DATA("{}: [{}]     ISBDrift [{:5}] = {} s/s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST),
                     SatelliteSystem::fromEnum(static_cast<SatelliteSystem::Enum>(i)), _receiver[Rover].recvClk.sysTimeDiffDrift.at(i).value);
        }
    }
    for ([[maybe_unused]] const auto& freq : _receiver[Rover].interFrequencyBias)
    {
        LOG_DATA("{}: [{}]     IFBBias [{:3}] = {} s", nameId, _receiver[Rover].gnssObs->insTime.toYMDHMS(GPST), freq.first, freq.second.value);
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