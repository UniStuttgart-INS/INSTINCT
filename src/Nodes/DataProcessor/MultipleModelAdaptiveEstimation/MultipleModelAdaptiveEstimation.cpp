// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultipleModelAdaptiveEstimation.hpp"
#include <algorithm>
#include <cstdint>
#include <imgui.h>
#include <limits>
#include <string>
#include <boost/sort/spreadsort/spreadsort.hpp>
#include <boost/range/algorithm/unique.hpp>

#include "NodeData/State/MmaeSolution.hpp"
#include "Navigation/INS/InertialIntegrator.hpp"
#include "Navigation/INS/LocalNavFrame/ErrorEquations.hpp"
#include "QuaternionAverage.hpp"
#include <variant>

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Nodes/DataProcessor/KalmanFilter/LckfKeys.hpp"
#include "util/Logger.hpp"
#include "internal/FlowManager.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/Quaternion.h>

#include <Eigen/src/Core/Matrix.h>
#include <fmt/core.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

/// @brief Scale factor to convert the latitude and longitude error
constexpr double SCALE_FACTOR_LAT_LON = NAV::InsConst::pseudometre;
/// @brief Scale factor to convert the acceleration error
constexpr double SCALE_FACTOR_ACCELERATION = 1e3 / NAV::InsConst::G_NORM;
/// @brief Scale factor to convert the angular rate error
constexpr double SCALE_FACTOR_ANGULAR_RATE = 1e3;

namespace NAV
{
MultipleModelAdaptiveEstimation::MultipleModelAdaptiveEstimation()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 502., 207. };

    _dynamicInputPins.addPin(this);
    _dynamicInputPins.addPin(this);
    CreateOutputPin("Comb", Pin::Type::Flow, { MmaeSolution::type() });
}

MultipleModelAdaptiveEstimation::~MultipleModelAdaptiveEstimation()
{
    LOG_TRACE("{}: called", nameId());
}

std::string MultipleModelAdaptiveEstimation::typeStatic()
{
    return "MultipleModelAdaptiveEstimation";
}

std::string MultipleModelAdaptiveEstimation::type() const
{
    return typeStatic();
}

std::string MultipleModelAdaptiveEstimation::category()
{
    return "Data Processor";
}

void MultipleModelAdaptiveEstimation::guiConfig()
{
    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader(fmt::format("Pins##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        std::vector<size_t> pinIds;
        pinIds.reserve(inputPins.size());
        for (const auto& pin : inputPins) { pinIds.push_back(size_t(pin.id)); }
        if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this))
        {
            std::vector<size_t> inputPinIds;
            inputPinIds.reserve(inputPins.size());
            for (const auto& pin : inputPins) { inputPinIds.push_back(size_t(pin.id)); }
            LOG_DATA("{}: old Input pin ids {}", nameId(), fmt::join(pinIds, ", "));
            LOG_DATA("{}: new Input pin ids {}", nameId(), fmt::join(inputPinIds, ", "));
            flow::ApplyChanges();
        }
    }
    if (ImGui::CollapsingHeader(fmt::format("MMAE settings##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Scalar penalty##{}", size_t(id)).c_str(), &_alpha, 0.0, std::numeric_limits<double>::max(), 0.0, 0.0, "%.3f"))
        {
            LOG_DEBUG("{}: Scalar penalty 'alpha' changed to {}", nameId(), _alpha);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Used in conditional density function of the measurement");
        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::InputDoubleL(fmt::format("Lower limit of LCKF weights##{}", size_t(id)).c_str(), &_weightsLowerLimit, 0.0, WEIGHTS_UPPER_LIMIT, 0.0, 0.0, "%.3e"))
        {
            LOG_DEBUG("{}: 'weightsLowerLimit' changed to {}", nameId(), _weightsLowerLimit);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("MMAE tends to stick to an elemental LCKF, if it weights the LCKF at nearly 100%. Setting this lower limit on the weights allows for a change, even if the model parameters change slowly. Be aware, however, that the lower limit of weights should not exceed the weight of uniform distribution of all elemental LCKFs (i.e. lower limit values for a higher number of LCKFs).");
        ImGui::SetNextItemWidth(columnWidth);
        if (auto initDistributionWeights = static_cast<int>(_initDistributionWeights);
            ImGui::Combo(fmt::format("Initial distribution of weights##{}", size_t(id)).c_str(), &initDistributionWeights, "Uniform\0First LCKF (Pin)\0\0"))
        {
            _initDistributionWeights = static_cast<decltype(_initDistributionWeights)>(initDistributionWeights);
            LOG_DEBUG("{}: Initial distribution of weights changed to {}", nameId(), fmt::underlying(_initDistributionWeights));
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Use the uniform distribution to make all LCKFs have the same initial weight. Use the 'First LCKF (Pin)' option to give the LCKF on the first Pin maximum weight, while all others have a weight close to zero (case: nominal LCKF at the start).");
    }
}

[[nodiscard]] json MultipleModelAdaptiveEstimation::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "dynamicInputPins", _dynamicInputPins },
        { "alpha", _alpha },
        { "initDistributionWeights", _initDistributionWeights },
    };
}

void MultipleModelAdaptiveEstimation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins")) { NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this); }
    if (j.contains("alpha")) { j.at("alpha").get_to(_alpha); }
    if (j.contains("initDistributionWeights")) { j.at("initDistributionWeights").get_to(_initDistributionWeights); }
}

void MultipleModelAdaptiveEstimation::pinAddCallback(Node* node)
{
    node->CreateInputPin(fmt::format("Pin {}", node->inputPins.size() + 1).c_str(), Pin::Type::Flow, { InsGnssLCKFSolution::type() }, &MultipleModelAdaptiveEstimation::receiveData);
}

void MultipleModelAdaptiveEstimation::pinDeleteCallback(Node* node, size_t pinIdx)
{
    node->DeleteInputPin(pinIdx);
}

bool MultipleModelAdaptiveEstimation::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _nFilters = inputPins.size();
    _pinData.clear();
    _lckfIndices.clear();
    _probabilities.resize(_nFilters);
    if (_initDistributionWeights == InitDistributionWeights::Uniform)
    {
        for (size_t i = 0; i < _nFilters; i++)
        {
            _probabilities.at(i) = 1.0 / static_cast<double>(_nFilters);
        }
    }
    else // InitDistributionWeights::FirstKF
    {
        _probabilities.at(0) = 1.0;
        for (size_t i = 1; i < _nFilters; i++)
        {
            _probabilities.at(i) = 0.0;
        }
    }

    return true;
}

void MultipleModelAdaptiveEstimation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void MultipleModelAdaptiveEstimation::receiveData(InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto lckfSolution = std::static_pointer_cast<const InsGnssLCKFSolution>(queue.extract_front());

    if (lckfSolution->mmaeData.has_value())
    {
        _pinData.push_back({ .lckfIdx = pinIdx, .lckfSol = lckfSolution, .updateType = UpdateType::PredictionOnly }); // Prediction only
        _lckfIndices.push_back(pinIdx);

        if (_pinData.back().lckfSol->mmaeData->innovationVector.has_value() && !_pinData.back().lckfSol->mmaeData->innovationVectorBaro.has_value()) // PosVel update
        {
            _pinData.back().updateType = UpdateType::PosVel;
        }
        else if (!_pinData.back().lckfSol->mmaeData->innovationVector.has_value() && _pinData.back().lckfSol->mmaeData->innovationVectorBaro.has_value()) // Baro update
        {
            _pinData.back().updateType = UpdateType::BaroHgt;
        }
        else if (_pinData.back().lckfSol->mmaeData->innovationVector.has_value() && _pinData.back().lckfSol->mmaeData->innovationVectorBaro.has_value()) // PosVel and Baro update
        {
            _pinData.back().updateType = UpdateType::PosVelAndBaroHgt;
        }

        if (_pinData.size() == _nFilters)
        {
            boost::sort::spreadsort::spreadsort(_lckfIndices.begin(), _lckfIndices.end());
            boost::range::unique(_lckfIndices);
            if (_lckfIndices.size() < _pinData.size())
            {
                LOG_ERROR("{}: MMAE did not receive LCKF solutions from every single LCKF. Available LCKFs: {}", nameId(), _lckfIndices);
            }
            calcAdaptiveEstimate();
            _pinData.clear();
            _lckfIndices.clear();
        }
    }
};

void MultipleModelAdaptiveEstimation::calcAdaptiveEstimate()
{
    auto adaptiveEstimate = std::make_shared<MmaeSolution>();

    adaptiveEstimate->insTime = _pinData.data()->lckfSol->insTime;
    adaptiveEstimate->data.resize(_nFilters);

    if (!(_pinData.back().updateType == UpdateType::PredictionOnly))
    {
        std::vector<double> condDensitiesMeas;
        condDensitiesMeas.resize(_nFilters);
        double sumWeightedCondDensities{};

        for (size_t i = 0; i < _nFilters; i++)
        {
            uint8_t nMeas{};
            Eigen::VectorXd innovVector;
            Eigen::MatrixXd innovCovMat;

            if (_pinData.at(i).updateType == UpdateType::PosVel || _pinData.at(i).updateType == UpdateType::BaroHgt)
            {
                nMeas = _pinData.at(i).updateType == UpdateType::PosVel ? NUM_MEASUREMENTS_POSVEL : NUM_MEASUREMENTS_BARO;
                innovVector.resize(nMeas);
                innovCovMat.resize(nMeas, nMeas);
                if (_pinData.at(i).updateType == UpdateType::PosVel)
                {
                    innovVector = _pinData.at(i).lckfSol->mmaeData->innovationVector.value()(all);
                    innovCovMat = _pinData.at(i).lckfSol->mmaeData->innovationCovMatrix.value()(all, all);
                }
                else if (_pinData.at(i).updateType == UpdateType::BaroHgt)
                {
                    innovVector = _pinData.at(i).lckfSol->mmaeData->innovationVectorBaro.value()(all);
                    innovCovMat = _pinData.at(i).lckfSol->mmaeData->innovationCovMatrixBaro.value()(all, all);
                }
            }
            else if (_pinData.at(i).updateType == UpdateType::PosVelAndBaroHgt)
            {
                nMeas = NUM_MEASUREMENTS_POSVEL + NUM_MEASUREMENTS_BARO;
                innovVector.resize(nMeas);
                innovVector.block<NUM_MEASUREMENTS_POSVEL, 1>(0, 0) = _pinData.at(i).lckfSol->mmaeData->innovationVector.value()(all);
                innovVector.block<NUM_MEASUREMENTS_BARO, 1>(NUM_MEASUREMENTS_POSVEL, 0) = _pinData.at(i).lckfSol->mmaeData->innovationVectorBaro.value()(all);

                innovCovMat.resize(nMeas, nMeas);
                innovCovMat.block<NUM_MEASUREMENTS_POSVEL, NUM_MEASUREMENTS_POSVEL>(0, 0) = _pinData.at(i).lckfSol->mmaeData->innovationCovMatrix.value()(all, all);
                innovCovMat.block<NUM_MEASUREMENTS_BARO, NUM_MEASUREMENTS_BARO>(NUM_MEASUREMENTS_POSVEL, NUM_MEASUREMENTS_POSVEL) = _pinData.at(i).lckfSol->mmaeData->innovationCovMatrixBaro.value()(all, all);
            }
            auto beta = 1.0 / (std::pow(2.0 * M_PI, static_cast<double>(nMeas) / 2.0) * std::sqrt((innovCovMat.determinant())));

            // Conditional density function of the measurement (multivariate Gaussian distribution). Lower-bounded to avoid NaNs (see 'beta dominance effect' in Maybeck and Hanlon (1993) for more info)
            condDensitiesMeas.at(_pinData.at(i).lckfIdx) = std::max(beta * std::exp(-_alpha * innovVector.transpose() * innovCovMat.inverse() * innovVector), LOWER_LIMIT);

            sumWeightedCondDensities += condDensitiesMeas.at(_pinData.at(i).lckfIdx) * _probabilities.at(_pinData.at(i).lckfIdx);
        }

        std::vector<double> newProbabilities;
        newProbabilities.resize(_nFilters);
        double sumProbabilities{};
        for (size_t i = 0; i < _nFilters; i++)
        {
            // Update conditional probabilities
            auto weightedCondDensity = condDensitiesMeas.at(_pinData.at(i).lckfIdx) * _probabilities.at(_pinData.at(i).lckfIdx);
            newProbabilities.at(_pinData.at(i).lckfIdx) = std::max(weightedCondDensity / sumWeightedCondDensities, _weightsLowerLimit);
            sumProbabilities += newProbabilities.at(_pinData.at(i).lckfIdx);
        }

        for (size_t i = 0; i < _nFilters; i++)
        {
            auto newProbability = newProbabilities.at(_pinData.at(i).lckfIdx) / sumProbabilities; // Normalize probabilities if lower limit of weights is used --> guaranteed sum of 1
            adaptiveEstimate->data.at(i) = { .kfId = _pinData.at(i).lckfIdx, .weight = newProbability };
        }

        _probabilities = newProbabilities;
    }

    // Data preparation for averaging x and P
    if (std::holds_alternative<KeyedVector<double, LckfKeys::KFStates, LckfKeys::KFStates_COUNT>>(_pinData.at(_pinData.at(0).lckfIdx).lckfSol->mmaeData->kfTotalStateVector)) // length(stateVector == 17: variant with RPY)
    {
        for (size_t i = 0; i < _nFilters; i++)
        {
            if (adaptiveEstimate->data.at(i).weight < LOWER_LIMIT)
            {
                adaptiveEstimate->data.at(i) = { .kfId = _pinData.at(i).lckfIdx, .weight = _probabilities.at(i) }; // Initializes weights and sets weights during prediction to the values from the latest update
            }
            auto kfStateVector = std::get<KeyedVector<double, LckfKeys::KFStates, LckfKeys::KFStates_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfTotalStateVector)(all);
            _mmaeKalmanFilter.x(all) += kfStateVector * _probabilities.at(_pinData.at(i).lckfIdx);
        }

        for (size_t i = 0; i < _nFilters; i++)
        {
            auto kfStateVector = std::get<KeyedVector<double, LckfKeys::KFStates, LckfKeys::KFStates_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfTotalStateVector)(all);
            auto kfErrorCovMat = std::get<KeyedMatrix<double, LckfKeys::KFStates, LckfKeys::KFStates, LckfKeys::KFStates_COUNT, LckfKeys::KFStates_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfErrorCovMatrix)(all, all);
            _mmaeKalmanFilter.P(all, all) += _probabilities.at(_pinData.at(i).lckfIdx) * (kfErrorCovMat + (kfStateVector - _mmaeKalmanFilter.x(all)) * (kfStateVector - _mmaeKalmanFilter.x(all)).transpose());
        }
    }
    else // std::holds_alternative<KeyedVector<double, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT>>(kfTotalStateVector) // length(stateVector == 18: variant with attitude quaternion)
    {
        std::vector<Eigen::Quaterniond> attitudeQuaternions{};
        std::vector<Eigen::Matrix4d> kfQuaternionCovariances{};
        for (size_t i = 0; i < _nFilters; i++)
        {
            if (adaptiveEstimate->data.at(i).weight < LOWER_LIMIT)
            {
                adaptiveEstimate->data.at(i) = { .kfId = _pinData.at(i).lckfIdx, .weight = _probabilities.at(i) }; // Initializes weights and sets weights during prediction to the values from the latest update
            }

            auto kfTotalStateVector = std::get<KeyedVector<double, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfTotalStateVector);
            Eigen::Quaterniond attitudeQuat{ kfTotalStateVector(LckfKeys::KFStatesQuat::QuatW), kfTotalStateVector(LckfKeys::KFStatesQuat::QuatX), kfTotalStateVector(LckfKeys::KFStatesQuat::QuatY), kfTotalStateVector(LckfKeys::KFStatesQuat::QuatZ) };
            attitudeQuaternions.push_back(attitudeQuat);

            Eigen::Matrix4d kfQuaternionCov = std::get<KeyedMatrix<double, LckfKeys::KFStatesQuat, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT, LckfKeys::KFQuat_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfErrorCovMatrix)(LckfKeys::KFAttQuat, LckfKeys::KFAttQuat);
            kfQuaternionCovariances.push_back(kfQuaternionCov);

            // Average the rest of the states (all states, except for the attitude quaternion)
            auto kfStateVector = std::get<KeyedVector<double, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfTotalStateVector)(LckfKeys::KFRestQuat);
            _mmaeKalmanFilter.x(LckfKeys::RestStates) += _probabilities.at(_pinData.at(i).lckfIdx) * kfStateVector;
        }

        double maxEigenValue{};
        auto averagedQuaternion = calcQuaternionAverage(attitudeQuaternions, _probabilities, maxEigenValue);
        Eigen::Quaterniond avgQuat = { averagedQuaternion(3).real(), averagedQuaternion(0).real(), averagedQuaternion(1).real(), averagedQuaternion(2).real() };
        _mmaeKalmanFilter.x(LckfKeys::KFAtt) = trafo::quat2eulerZYX(avgQuat);

        for (size_t i = 0; i < _nFilters; i++)
        {
            auto kfStateVector = std::get<KeyedVector<double, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfTotalStateVector)(LckfKeys::KFRestQuat);
            auto kfErrorCovMat = std::get<KeyedMatrix<double, LckfKeys::KFStatesQuat, LckfKeys::KFStatesQuat, LckfKeys::KFQuat_COUNT, LckfKeys::KFQuat_COUNT>>(_pinData.at(_pinData.at(i).lckfIdx).lckfSol->mmaeData->kfErrorCovMatrix)(LckfKeys::KFRestQuat, LckfKeys::KFRestQuat);
            _mmaeKalmanFilter.P(LckfKeys::RestStates, LckfKeys::RestStates) += _probabilities.at(_pinData.at(i).lckfIdx) * (kfErrorCovMat + (kfStateVector - _mmaeKalmanFilter.x(LckfKeys::RestStates)) * (kfStateVector - _mmaeKalmanFilter.x(LckfKeys::RestStates)).transpose());
        }

        auto AveragedQuaternionCov = calcQuaternionAverageUncertainty(attitudeQuaternions, _probabilities, kfQuaternionCovariances, maxEigenValue, averagedQuaternion);
        Eigen::Matrix<double, 3, 4> J = trafo::covQuat2RPYJacobian(avgQuat);
        _mmaeKalmanFilter.P(LckfKeys::KFAtt, LckfKeys::KFAtt) = J * AveragedQuaternionCov * J.transpose();
    }

    // Prime vertical radius of curvature (East/West) [m]
    double R_E = 0.0;
    // Meridian radius of curvature in [m]
    double R_N = 0.0;

    if (_pinData.at(0).lckfSol->mmaeData->integrationFrame == InertialIntegrator::IntegrationFrame::NED)
    {
        R_E = calcEarthRadius_E(_mmaeKalmanFilter.x.segment(LckfKeys::KFPos)(0));
        LOG_DATA("{}:     R_E = {} [m]", nameId(), R_E);
        R_N = calcEarthRadius_N(_mmaeKalmanFilter.x.segment(LckfKeys::KFPos)(0));
        LOG_DATA("{}:     R_N = {} [m]", nameId(), R_N);
    }

    setSolutionPosVelAttAndCov(adaptiveEstimate, R_N, R_E);

    adaptiveEstimate->b_biasAccel.value = -_mmaeKalmanFilter.x.segment(LckfKeys::KFAccBias);
    adaptiveEstimate->b_biasAccel.stdDev = Eigen::Vector3d{
        std::sqrt(_mmaeKalmanFilter.P(LckfKeys::KFStates::AccBiasX, LckfKeys::KFStates::AccBiasX) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
        std::sqrt(_mmaeKalmanFilter.P(LckfKeys::KFStates::AccBiasY, LckfKeys::KFStates::AccBiasY) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
        std::sqrt(_mmaeKalmanFilter.P(LckfKeys::KFStates::AccBiasZ, LckfKeys::KFStates::AccBiasZ) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2)))
    };
    adaptiveEstimate->b_biasGyro.value = -_mmaeKalmanFilter.x.segment(LckfKeys::KFGyrBias);
    adaptiveEstimate->b_biasGyro.stdDev = Eigen::Vector3d{
        std::sqrt(_mmaeKalmanFilter.P(LckfKeys::KFStates::GyrBiasX, LckfKeys::KFStates::GyrBiasX) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
        std::sqrt(_mmaeKalmanFilter.P(LckfKeys::KFStates::GyrBiasY, LckfKeys::KFStates::GyrBiasY) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
        std::sqrt(_mmaeKalmanFilter.P(LckfKeys::KFStates::GyrBiasZ, LckfKeys::KFStates::GyrBiasZ) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2)))
    };

#ifndef NDEBUG
    std::string updateTypeString{};
    if (_pinData.back().updateType == UpdateType::PredictionOnly)
    {
        updateTypeString = "PredictionOnly";
    }
    else if (_pinData.back().updateType == UpdateType::PosVel)
    {
        updateTypeString = "PosVel";
    }
    else if (_pinData.back().updateType == UpdateType::BaroHgt)
    {
        updateTypeString = "BaroHgt";
    }
    else if (_pinData.back().updateType == UpdateType::PosVelAndBaroHgt)
    {
        updateTypeString = "PosVelAndBaroHgt";
    }
    LOG_TRACE("{}: At {}, updateType: {}", nameId(), adaptiveEstimate->insTime.toYMDHMS(), updateTypeString);
#endif

    invokeCallbacks(OUTPUT_PORT_INDEX_DYN_DATA, adaptiveEstimate);

    _mmaeKalmanFilter.setZero();
};

void NAV::MultipleModelAdaptiveEstimation::setSolutionPosVelAttAndCov(const std::shared_ptr<PosVelAtt>& mmaeSolution, double R_N, double R_E)
{
    Eigen::Matrix<double, 10, 9> J = Eigen::Matrix<double, 10, 9>::Zero();
    J.topLeftCorner<6, 6>().setIdentity();

    Eigen::Matrix9d J_units = Eigen::Matrix9d::Identity();
    J_units.bottomRightCorner<3, 3>().diagonal().setConstant(std::numbers::pi_v<double> / 180.0);

    const Eigen::Vector3d& position = _mmaeKalmanFilter.x.segment(LckfKeys::KFPos);
    const Eigen::Vector3d& velocity = _mmaeKalmanFilter.x.segment(LckfKeys::KFVel);
    const Eigen::Vector3d& attitude = _mmaeKalmanFilter.x.segment(LckfKeys::KFAtt);
    const Eigen::Quaterniond& Quat_b = trafo::euler2quat(attitude);

    if (_pinData.at(0).lckfSol->mmaeData->integrationFrame == InertialIntegrator::IntegrationFrame::NED)
    {
        J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(Quat_b);
        J_units.topLeftCorner<3, 3>().diagonal() = 1.0
                                                   / n_F_dr_dv(position.x(),
                                                               position.z(),
                                                               R_N,
                                                               R_E)
                                                         .diagonal()
                                                         .array();

        J_units.topLeftCorner<2, 2>().diagonal() *= 1.0 / SCALE_FACTOR_LAT_LON;

        mmaeSolution->setPosVelAttAndCov_n(position,
                                           velocity,
                                           Quat_b,
                                           J * (J_units * _mmaeKalmanFilter.P(LckfKeys::KFPosVelAtt, LckfKeys::KFPosVelAtt) * J_units.transpose()) * J.transpose());
    }
    else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
    {
        mmaeSolution->setPosVelAttAndCov_e(position,
                                           velocity,
                                           Quat_b,
                                           J * (J_units * _mmaeKalmanFilter.P(LckfKeys::KFPosVelAtt, LckfKeys::KFPosVelAtt) * J_units.transpose()) * J.transpose());
    }
}

} // namespace NAV