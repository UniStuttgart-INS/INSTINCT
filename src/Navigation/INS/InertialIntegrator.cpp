// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InertialIntegrator.cpp
/// @brief Inertial Measurement Integrator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-09

#include "InertialIntegrator.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV
{

InertialIntegrator::InertialIntegrator(IntegrationFrame integrationFrame)
    : _integrationFrame(integrationFrame), _lockIntegrationFrame(true) {}

void InertialIntegrator::reset()
{
    _states.clear();
    _p_lastBiasAcceleration.setZero();
    _p_lastBiasAngularRate.setZero();
    setBufferSizes();
}

bool InertialIntegrator::hasInitialPosition() const
{
    return !_states.empty();
}

void InertialIntegrator::setInitialState(const PosVelAtt& state, const char* nameId)
{
    _states.clear();
    addState(state, nameId);
}

void InertialIntegrator::addState(const PosVelAtt& state, [[maybe_unused]] const char* nameId)
{
    _states.push_back(State{
        .epoch = state.insTime,
        .position = _integrationFrame == IntegrationFrame::ECEF ? state.e_position() : state.lla_position(),
        .velocity = _integrationFrame == IntegrationFrame::ECEF ? state.e_velocity() : state.n_velocity(),
        .attitude = _integrationFrame == IntegrationFrame::ECEF ? state.e_Quat_b() : state.n_Quat_b(),
        .m = Measurement{
            .averagedMeasurement = false,
            .dt = 0.0,
            .p_acceleration = Eigen::Vector3d::Zero(),
            .p_angularRate = Eigen::Vector3d::Zero(),
        } });
    LOG_DATA("{}: Adding state for [{}]. Now has {} states", nameId, state.insTime.toYMDHMS(GPST), _states.size());
}

void InertialIntegrator::setTotalSensorBiases(const Eigen::Vector3d& p_biasAcceleration, const Eigen::Vector3d& p_biasAngularRate)
{
    _p_lastBiasAcceleration = p_biasAcceleration;
    _p_lastBiasAngularRate = p_biasAngularRate;
}

void InertialIntegrator::applySensorBiasesIncrements(const Eigen::Vector3d& p_deltaBiasAcceleration, const Eigen::Vector3d& p_deltaBiasAngularRate)
{
    _p_lastBiasAcceleration += p_deltaBiasAcceleration;
    _p_lastBiasAngularRate += p_deltaBiasAngularRate;
}

void InertialIntegrator::applyStateErrors_n(const Eigen::Vector3d& lla_positionError, const Eigen::Vector3d& n_velocityError, const Eigen::Vector3d& n_attitudeError_b)
{
    INS_ASSERT_USER_ERROR(_integrationFrame == IntegrationFrame::NED, "You can only apply errors to the selected integration frame");
    INS_ASSERT_USER_ERROR(!_states.empty(), "You can only apply errors if the states vector is not empty");

    _states.back().position -= lla_positionError;
    _states.back().velocity -= n_velocityError;
    // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
    Eigen::Matrix3d n_Dcm_b = (Eigen::Matrix3d::Identity() - math::skewSymmetricMatrix(n_attitudeError_b)) * _states.back().attitude.toRotationMatrix();
    _states.back().attitude = Eigen::Quaterniond(n_Dcm_b).normalized();

    // TODO: Test this out again
    // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.16
    // Eigen::Quaterniond n_Quat_b = posVelAtt->n_Quat_b()
    //                                  * (Eigen::AngleAxisd(attError(0), Eigen::Vector3d::UnitX())
    //                                     * Eigen::AngleAxisd(attError(1), Eigen::Vector3d::UnitY())
    //                                     * Eigen::AngleAxisd(attError(2), Eigen::Vector3d::UnitZ()))
    //                                        .normalized();
    // posVelAttCorrected->setAttitude_n_Quat_b(n_Quat_b.normalized());

    // Eigen::Vector3d attError = pvaError->n_attitudeError();
    // const Eigen::Quaterniond& n_Quat_b = posVelAtt->n_Quat_b();
    // Eigen::Quaterniond n_Quat_b_c{ n_Quat_b.w() + 0.5 * (+attError(0) * n_Quat_b.x() + attError(1) * n_Quat_b.y() + attError(2) * n_Quat_b.z()),
    //                            n_Quat_b.x() + 0.5 * (-attError(0) * n_Quat_b.w() + attError(1) * n_Quat_b.z() - attError(2) * n_Quat_b.y()),
    //                            n_Quat_b.y() + 0.5 * (-attError(0) * n_Quat_b.z() - attError(1) * n_Quat_b.w() + attError(2) * n_Quat_b.x()),
    //                            n_Quat_b.z() + 0.5 * (+attError(0) * n_Quat_b.y() - attError(1) * n_Quat_b.x() - attError(2) * n_Quat_b.w()) };
    // posVelAttCorrected->setAttitude_n_Quat_b(n_Quat_b_c.normalized());
}

void InertialIntegrator::applyStateErrors_e(const Eigen::Vector3d& e_positionError, const Eigen::Vector3d& e_velocityError, const Eigen::Vector3d& e_attitudeError_b)
{
    INS_ASSERT_USER_ERROR(_integrationFrame == IntegrationFrame::ECEF, "You can only apply errors to the selected integration frame");
    INS_ASSERT_USER_ERROR(!_states.empty(), "You can only apply errors if the states vector is not empty");

    _states.back().position -= e_positionError;
    _states.back().velocity -= e_velocityError;
    // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
    Eigen::Matrix3d e_Dcm_b = (Eigen::Matrix3d::Identity() - math::skewSymmetricMatrix(e_attitudeError_b)) * _states.back().attitude.toRotationMatrix();
    _states.back().attitude = Eigen::Quaterniond(e_Dcm_b).normalized();
}

std::optional<std::reference_wrapper<const InertialIntegrator::State>> InertialIntegrator::getLatestState() const
{
    if (_states.empty()) { return {}; }
    return _states.back();
}

const Eigen::Vector3d& InertialIntegrator::p_getLastAccelerationBias() const
{
    return _p_lastBiasAcceleration;
}

const Eigen::Vector3d& InertialIntegrator::p_getLastAngularRateBias() const
{
    return _p_lastBiasAngularRate;
}

InertialIntegrator::IntegrationFrame InertialIntegrator::getIntegrationFrame() const
{
    return _integrationFrame;
}

const PosVelAttDerivativeConstants& InertialIntegrator::getConstants() const
{
    return _posVelAttDerivativeConstants;
}

bool InertialIntegrator::areAccelerationsAveragedMeasurements() const
{
    return _accelerationsAreAveragedMeasurements;
}

std::optional<Eigen::Vector3d> InertialIntegrator::p_calcCurrentAcceleration() const
{
    if (_states.empty()) { return {}; }

    return _states.back().m.p_acceleration + _states.back().p_biasAcceleration;
}

std::optional<Eigen::Vector3d> InertialIntegrator::p_calcCurrentAngularRate() const
{
    if (_states.empty()) { return {}; }

    return _states.back().m.p_angularRate + _states.back().p_biasAngularRate;
}

void InertialIntegrator::addMeasurement(const InsTime& epoch, double dt, const Eigen::Vector3d& p_acceleration, const Eigen::Vector3d& p_angularRate, [[maybe_unused]] const char* nameId)
{
    LOG_DATA("{}: Adding measurement [{}]. Last state at [{}]", nameId, epoch.toYMDHMS(GPST), _states.back().epoch.toYMDHMS(GPST));
    INS_ASSERT_USER_ERROR(!_states.empty(), "You need to add an initial state first");

    if (_states.back().epoch == epoch)
    {
        LOG_DATA("{}:   Updating existing state", nameId);
        _states.back().m.averagedMeasurement = _accelerationsAreAveragedMeasurements;
        _states.back().m.dt = dt;
        _states.back().m.p_acceleration = p_acceleration;
        _states.back().m.p_angularRate = p_angularRate;
        _states.back().p_biasAcceleration = _p_lastBiasAcceleration;
        _states.back().p_biasAngularRate = _p_lastBiasAngularRate;
        return;
    }

    LOG_DATA("{}:   Adding as new state", nameId);
    _states.push_back(State{
        .epoch = epoch,
        .position = _states.back().position,
        .velocity = _states.back().velocity,
        .attitude = _states.back().attitude,
        .m = Measurement{
            .averagedMeasurement = _accelerationsAreAveragedMeasurements,
            .dt = dt,
            .p_acceleration = p_acceleration,
            .p_angularRate = p_angularRate,
        },
        .p_biasAcceleration = _p_lastBiasAcceleration,
        .p_biasAngularRate = _p_lastBiasAngularRate,
    });
}

void InertialIntegrator::addDeltaMeasurement(const InsTime& epoch, double dt, double deltaTime, const Eigen::Vector3d& p_deltaVelocity, const Eigen::Vector3d& p_deltaTheta, const char* nameId)
{
    LOG_DATA("{}: Adding delta measurement for [{}]", nameId, epoch.toYMDHMS(GPST));
    Eigen::Vector3d p_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_angularRate = Eigen::Vector3d::Zero();

    if (deltaTime > 0.0) // dt given by sensor (should never be 0 or negative, but check here just in case)
    {
        p_acceleration = p_deltaVelocity / deltaTime;
        p_angularRate = p_deltaTheta / deltaTime;
    }
    else if (std::abs(dt) > 0.0) // Time difference between messages (differs from dt if message lost)
    {
        // Negative values of dTimeLastState should not happen, but algorithm can work with it to propagate backwards
        p_acceleration = p_deltaVelocity / dt;
        p_angularRate = p_deltaTheta / dt;
    }

    addMeasurement(epoch, dt, p_acceleration, p_angularRate, nameId);
    _states.back().m.averagedMeasurement = true;
}

std::shared_ptr<PosVelAtt> InertialIntegrator::lastStateAsPosVelAtt() const
{
    INS_ASSERT_USER_ERROR(!_states.empty(), "You need to add an initial state first");

    auto posVelAtt = std::make_shared<PosVelAtt>();
    posVelAtt->insTime = _states.back().epoch;
    switch (_integrationFrame)
    {
    case IntegrationFrame::NED:
        posVelAtt->setPosVelAtt_n(_states.back().position, _states.back().velocity, _states.back().attitude);
        break;
    case IntegrationFrame::ECEF:
        posVelAtt->setPosVelAtt_e(_states.back().position, _states.back().velocity, _states.back().attitude);
        break;
    }

    return posVelAtt;
}

std::shared_ptr<PosVelAtt> InertialIntegrator::calcInertialSolution(const InsTime& obsTime,
                                                                    const Eigen::Vector3d& p_acceleration, const Eigen::Vector3d& p_angularRate,
                                                                    const ImuPos& imuPos, const char* nameId)
{
    if (!hasInitialPosition() || obsTime < _states.back().epoch) { return nullptr; }
    if (_states.back().epoch.empty()) { _states.back().epoch = obsTime; }

    auto dt = static_cast<double>((obsTime - _states.back().epoch).count());
    addMeasurement(obsTime, dt, p_acceleration, p_angularRate, nameId);
    if (std::abs(dt) < 1e-8)
    {
        LOG_DATA("{}: Returning state [{}], as no time difference between measurement.", nameId, _states.back().epoch.toYMDHMS(GPST));
        return lastStateAsPosVelAtt();
    }

    return calcInertialSolutionFromMeasurementBuffer(imuPos, nameId);
}

std::shared_ptr<PosVelAtt> InertialIntegrator::calcInertialSolutionDelta(const InsTime& obsTime, double deltaTime,
                                                                         const Eigen::Vector3d& p_deltaVelocity, const Eigen::Vector3d& p_deltaTheta,
                                                                         const ImuPos& imuPos, const char* nameId)
{
    if (!hasInitialPosition() || obsTime < _states.back().epoch) { return nullptr; }
    if (_states.back().epoch.empty()) { _states.back().epoch = obsTime; }

    auto dt = static_cast<double>((obsTime - _states.back().epoch).count());
    addDeltaMeasurement(obsTime, dt, deltaTime, p_deltaVelocity, p_deltaTheta, nameId);
    if (std::abs(dt) < 1e-8)
    {
        LOG_DATA("{}: Returning state [{}], as no time difference between measurement.", nameId, _states.back().epoch.toYMDHMS(GPST));
        return lastStateAsPosVelAtt();
    }

    return calcInertialSolutionFromMeasurementBuffer(imuPos, nameId);
}

std::shared_ptr<PosVelAtt> InertialIntegrator::calcInertialSolutionFromMeasurementBuffer(const ImuPos& imuPos, const char* nameId)
{
    INS_ASSERT_USER_ERROR(!_states.empty(), "You need to add an initial state first");
    INS_ASSERT_USER_ERROR(_states.size() >= 2, "You need to add an initial state and at least one measurement first");

    LOG_DATA("{}: Calculating inertial solution from buffer. States t0 = [{}], t1 = [{}]",
             nameId, _states.back().epoch.toYMDHMS(GPST), _states.at(_states.size() - 2).epoch.toYMDHMS(GPST));

    auto y = calcInertialSolution(imuPos, _states.back(), _states.at(_states.size() - 2), nameId);

    auto posVelAtt = std::make_shared<PosVelAtt>();
    posVelAtt->insTime = _states.back().epoch;
    switch (_integrationFrame)
    {
    case IntegrationFrame::NED:
        posVelAtt->setPosVelAtt_n(y.segment<3>(0), y.segment<3>(3), Eigen::Quaterniond(y.segment<4>(6)));
        break;
    case IntegrationFrame::ECEF:
        posVelAtt->setPosVelAtt_e(y.segment<3>(0), y.segment<3>(3), Eigen::Quaterniond(y.segment<4>(6)));
        break;
    }

    switch (_integrationFrame)
    {
    case IntegrationFrame::NED:
        _states.back().position = posVelAtt->lla_position();
        _states.back().velocity = posVelAtt->n_velocity();
        _states.back().attitude = posVelAtt->n_Quat_b();
        break;
    case IntegrationFrame::ECEF:
        _states.back().position = posVelAtt->e_position();
        _states.back().velocity = posVelAtt->e_velocity();
        _states.back().attitude = posVelAtt->e_Quat_b();
        break;
    }

    return posVelAtt;
}

void InertialIntegrator::setBufferSizes()
{
    switch (_integrationAlgorithm)
    {
    case IntegrationAlgorithm::SingleStepRungeKutta1:
    case IntegrationAlgorithm::SingleStepRungeKutta2:
    case IntegrationAlgorithm::SingleStepHeun2:
    case IntegrationAlgorithm::SingleStepRungeKutta3:
    case IntegrationAlgorithm::SingleStepHeun3:
    case IntegrationAlgorithm::SingleStepRungeKutta4:
        _states.resize(2);
        break;
    case IntegrationAlgorithm::MultiStepRK3:
    case IntegrationAlgorithm::MultiStepRK4:
        _states.resize(3);
        break;
    case IntegrationAlgorithm::COUNT:
        break;
    }
}

bool InertialIntegratorGui(const char* label, InertialIntegrator& integrator, bool& preferAccelerationOverDeltaMeasurements, float width)
{
    bool changed = false;

    if (integrator._lockIntegrationFrame) { ImGui::BeginDisabled(); }
    ImGui::SetNextItemWidth(width * gui::NodeEditorApplication::windowFontRatio());
    if (auto integrationFrame = static_cast<int>(integrator._integrationFrame);
        ImGui::Combo(fmt::format("Integration Frame##{}", label).c_str(), &integrationFrame, "ECEF\0NED\0\0"))
    {
        integrator._integrationFrame = static_cast<decltype(integrator._integrationFrame)>(integrationFrame);
        LOG_DEBUG("Integration Frame changed to {}", integrator._integrationFrame == InertialIntegrator::IntegrationFrame::NED ? "NED" : "ECEF");
        changed = true;
    }
    if (integrator._lockIntegrationFrame) { ImGui::EndDisabled(); }

    ImGui::SetNextItemWidth(width * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::BeginCombo(fmt::format("Integration Algorithm##{}", label).c_str(), to_string(integrator._integrationAlgorithm)))
    {
        for (size_t i = 0; i < static_cast<size_t>(InertialIntegrator::IntegrationAlgorithm::MultiStepRK3); i++) // TODO: InertialIntegrator::IntegrationAlgorithm::COUNT
        {
            const bool is_selected = (static_cast<size_t>(integrator._integrationAlgorithm) == i);
            if (ImGui::Selectable(to_string(static_cast<InertialIntegrator::IntegrationAlgorithm>(i)), is_selected))
            {
                integrator._integrationAlgorithm = static_cast<InertialIntegrator::IntegrationAlgorithm>(i);
                integrator.setBufferSizes();
                LOG_DEBUG("Integration Algorithm Attitude changed to {}", fmt::underlying(integrator._integrationAlgorithm));
                changed = true;
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }

    changed |= ImGui::Checkbox(fmt::format("Prefer raw measurements over delta##{}", label).c_str(), &preferAccelerationOverDeltaMeasurements);

    if (preferAccelerationOverDeltaMeasurements)
    {
        changed |= ImGui::Checkbox(fmt::format("Accumulated Acceleration##{}", label).c_str(), &integrator._accelerationsAreAveragedMeasurements);
        ImGui::SameLine();
        gui::widgets::HelpMarker("Some IMUs operate at higher rates than their output rate. (e.g. internal rate 800Hz and output rate is 100Hz)\n"
                                 "- Such IMUs often output averaged accelerations between the current and last output.\n"
                                 "- If the connected IMU (e.g. a VectorNavSensor node) is such as sensor, please check this.\n"
                                 "- If delta measurements are used, this is automatically true");
    }

    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Compensation models##{}", label).c_str()))
    {
        ImGui::TextUnformatted("Acceleration compensation");
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(230 * gui::NodeEditorApplication::windowFontRatio());
            if (ComboGravitationModel(fmt::format("Gravitation Model##{}", label).c_str(), integrator._posVelAttDerivativeConstants.gravitationModel))
            {
                LOG_DEBUG("Gravity Model changed to {}", NAV::to_string(integrator._posVelAttDerivativeConstants.gravitationModel));
                changed = true;
            }
            if (ImGui::Checkbox(fmt::format("Coriolis acceleration ##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.coriolisAccelerationCompensationEnabled))
            {
                LOG_DEBUG("coriolisAccelerationCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.coriolisAccelerationCompensationEnabled);
                changed = true;
            }
            if (ImGui::Checkbox(fmt::format("Centrifugal acceleration##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.centrifgalAccelerationCompensationEnabled))
            {
                LOG_DEBUG("centrifgalAccelerationCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.centrifgalAccelerationCompensationEnabled);
                changed = true;
            }
            ImGui::Unindent();
        }
        ImGui::TextUnformatted("Angular rate compensation");
        {
            ImGui::Indent();
            if (ImGui::Checkbox(fmt::format("Earth rotation rate##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.angularRateEarthRotationCompensationEnabled))
            {
                LOG_DEBUG("angularRateEarthRotationCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.angularRateEarthRotationCompensationEnabled);
                changed = true;
            }
            if (integrator._integrationFrame == InertialIntegrator::IntegrationFrame::NED)
            {
                if (ImGui::Checkbox(fmt::format("Transport rate##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.angularRateTransportRateCompensationEnabled))
                {
                    LOG_DEBUG("angularRateTransportRateCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.angularRateTransportRateCompensationEnabled);
                    changed = true;
                }
            }
            ImGui::Unindent();
        }

        ImGui::TreePop();
    }

    return changed;
}

void to_json(json& j, const InertialIntegrator& data)
{
    j = json{
        { "integrationFrame", data._integrationFrame },
        { "lockIntegrationFrame", data._lockIntegrationFrame },
        { "integrationAlgorithm", data._integrationAlgorithm },
        { "accelerationsAreAveragedMeasurements", data._accelerationsAreAveragedMeasurements },
        { "posVelAttDerivativeConstants", data._posVelAttDerivativeConstants },
    };
}

void from_json(const json& j, InertialIntegrator& data)
{
    if (j.contains("integrationFrame")) { j.at("integrationFrame").get_to(data._integrationFrame); }
    if (j.contains("lockIntegrationFrame")) { j.at("lockIntegrationFrame").get_to(data._lockIntegrationFrame); }
    if (j.contains("integrationAlgorithm"))
    {
        j.at("integrationAlgorithm").get_to(data._integrationAlgorithm);
        data.setBufferSizes();
    }
    if (j.contains("accelerationsAreAveragedMeasurements")) { j.at("accelerationsAreAveragedMeasurements").get_to(data._accelerationsAreAveragedMeasurements); }
    if (j.contains("posVelAttDerivativeConstants")) { j.at("posVelAttDerivativeConstants").get_to(data._posVelAttDerivativeConstants); }
}

const char* to_string(InertialIntegrator::IntegrationAlgorithm algorithm)
{
    switch (algorithm)
    {
    case InertialIntegrator::IntegrationAlgorithm::SingleStepRungeKutta1:
        return "Runge-Kutta 1st order (explicit) / (Forward) Euler method";
    case InertialIntegrator::IntegrationAlgorithm::SingleStepRungeKutta2:
        return "Runge-Kutta 2nd order (explicit)";
    case InertialIntegrator::IntegrationAlgorithm::SingleStepHeun2:
        return "Heun's method (2nd order) (explicit)";
    case InertialIntegrator::IntegrationAlgorithm::SingleStepRungeKutta3:
        return "Runge-Kutta 3rd order (explicit) / Simpson's rule";
    case InertialIntegrator::IntegrationAlgorithm::SingleStepHeun3:
        return "Heun's method (3nd order) (explicit)";
    case InertialIntegrator::IntegrationAlgorithm::SingleStepRungeKutta4:
        return "Runge-Kutta 4th order (explicit)";
    case InertialIntegrator::IntegrationAlgorithm::MultiStepRK3:
        return "Runge-Kutta 3rd order (explicit) / Simpson's rule (multistep)";
    case InertialIntegrator::IntegrationAlgorithm::MultiStepRK4:
        return "Runge-Kutta 4th order (explicit) (multistep)";
    case InertialIntegrator::IntegrationAlgorithm::COUNT:
        return "";
    }
    return "";
}

const char* to_string(InertialIntegrator::IntegrationFrame frame)
{
    switch (frame)
    {
    case InertialIntegrator::IntegrationFrame::ECEF:
        return "ECEF";
    case InertialIntegrator::IntegrationFrame::NED:
        return "NED";
    }
    return "";
}

} // namespace NAV