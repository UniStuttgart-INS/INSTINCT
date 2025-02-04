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
#include "util/Logger.hpp"

#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/LocalNavFrame/Mechanization.hpp"
#include "Navigation/INS/EcefFrame/Mechanization.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV
{

void InertialIntegrator::reset()
{
    _measurements.clear();
    _states.clear();
    p_lastBiasAcceleration = Eigen::Vector3d::Zero();
    p_lastBiasAngularRate = Eigen::Vector3d::Zero();
    setBufferSizes();
}

bool InertialIntegrator::hasInitialPosition() const
{
    return !_states.empty();
}

void InertialIntegrator::setInitialState(const PosVelAtt& state)
{
    _states.clear();
    _states.push_back(state);
}

void InertialIntegrator::setState(const PosVelAtt& state)
{
    _states.push_back(state);
}

void InertialIntegrator::setTotalSensorBiases(const Eigen::Vector3d& p_biasAcceleration, const Eigen::Vector3d& p_biasAngularRate)
{
    p_lastBiasAcceleration = p_biasAcceleration;
    p_lastBiasAngularRate = p_biasAngularRate;
    if (!_measurements.empty())
    {
        _measurements.back().p_biasAcceleration = p_lastBiasAcceleration;
        _measurements.back().p_biasAngularRate = p_lastBiasAngularRate;
    }
}

void InertialIntegrator::applySensorBiasesIncrements(const Eigen::Vector3d& p_deltaBiasAcceleration, const Eigen::Vector3d& p_deltaBiasAngularRate)
{
    p_lastBiasAcceleration += p_deltaBiasAcceleration;
    p_lastBiasAngularRate += p_deltaBiasAngularRate;
    if (!_measurements.empty())
    {
        _measurements.back().p_biasAcceleration = p_lastBiasAcceleration;
        _measurements.back().p_biasAngularRate = p_lastBiasAngularRate;
    }
}

void InertialIntegrator::applyStateErrors_n(const Eigen::Vector3d& lla_positionError, const Eigen::Vector3d& n_velocityError, const Eigen::Vector3d& n_attitudeError_b)
{
    if (!_states.empty())
    {
        _states.back().setPosition_lla(_states.back().lla_position() - lla_positionError);
        _states.back().setVelocity_n(_states.back().n_velocity() - n_velocityError);
        // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
        Eigen::Matrix3d n_Dcm_b = (Eigen::Matrix3d::Identity() - math::skewSymmetricMatrix(n_attitudeError_b)) * _states.back().n_Quat_b().toRotationMatrix();
        _states.back().setAttitude_n_Quat_b(Eigen::Quaterniond(n_Dcm_b).normalized());

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
}

void InertialIntegrator::applyStateErrors_e(const Eigen::Vector3d& e_positionError, const Eigen::Vector3d& e_velocityError, const Eigen::Vector3d& e_attitudeError_b)
{
    if (!_states.empty())
    {
        _states.back().setPosition_e(_states.back().e_position() - e_positionError);
        _states.back().setVelocity_e(_states.back().e_velocity() - e_velocityError);
        // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
        Eigen::Matrix3d e_Dcm_b = (Eigen::Matrix3d::Identity() - math::skewSymmetricMatrix(e_attitudeError_b)) * _states.back().e_Quat_b().toRotationMatrix();
        _states.back().setAttitude_e_Quat_b(Eigen::Quaterniond(e_Dcm_b).normalized());
    }
}

const ScrollingBuffer<InertialIntegrator::Measurement>& InertialIntegrator::getMeasurements() const
{
    return _measurements;
}

std::optional<std::reference_wrapper<const PosVelAtt>> InertialIntegrator::getLatestState() const
{
    if (_states.empty()) { return {}; }
    return _states.back();
}

const Eigen::Vector3d& InertialIntegrator::p_getLastAccelerationBias() const
{
    return p_lastBiasAcceleration;
}

const Eigen::Vector3d& InertialIntegrator::p_getLastAngularRateBias() const
{
    return p_lastBiasAngularRate;
}

InertialIntegrator::IntegrationFrame InertialIntegrator::getIntegrationFrame() const
{
    return _integrationFrame;
}

std::optional<Eigen::Vector3d> InertialIntegrator::p_calcCurrentAcceleration() const
{
    if (_measurements.empty()) { return {}; }

    return _measurements.back().p_acceleration + _measurements.back().p_biasAcceleration;
}

std::optional<Eigen::Vector3d> InertialIntegrator::p_calcCurrentAngularRate() const
{
    if (_measurements.empty()) { return {}; }

    return _measurements.back().p_angularRate + _measurements.back().p_biasAngularRate;
}

std::shared_ptr<PosVelAtt> InertialIntegrator::calcInertialSolution(const InsTime& obsTime, const Eigen::Vector3d& p_acceleration,
                                                                    const Eigen::Vector3d& p_angularRate, const ImuPos& imuPos)
{
    if (!hasInitialPosition() || obsTime < _states.back().insTime) { return nullptr; }
    if (_states.back().insTime.empty()) { _states.back().insTime = obsTime; }

    _measurements.push_back(Measurement{ .dt = static_cast<double>((obsTime - _states.back().insTime).count()),
                                         .p_acceleration = p_acceleration,
                                         .p_angularRate = p_angularRate,
                                         .p_biasAcceleration = p_lastBiasAcceleration,
                                         .p_biasAngularRate = p_lastBiasAngularRate });

    return calcInertialSolutionFromMeasurementBuffer(imuPos);
}

std::shared_ptr<PosVelAtt> InertialIntegrator::calcInertialSolutionDelta(const InsTime& obsTime, const double& dt,
                                                                         const Eigen::Vector3d& p_deltaVelocity, const Eigen::Vector3d& p_deltaTheta,
                                                                         const ImuPos& imuPos)
{
    if (!hasInitialPosition() || obsTime < _states.back().insTime) { return nullptr; }
    if (_states.back().insTime.empty()) { _states.back().insTime = obsTime; }

    double dTimeLastState = static_cast<double>((obsTime - _states.back().insTime).count());

    Eigen::Vector3d p_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_angularRate = Eigen::Vector3d::Zero();

    if (dt > 0.0) // dt given by sensor (should never be 0 or negative, but check here just in case)
    {
        p_acceleration = p_deltaVelocity / dt;
        p_angularRate = p_deltaTheta / dt;
    }
    else if (std::abs(dTimeLastState) > 0.0) // Time difference between messages (differs from dt if message lost)
    {
        // Negative values of dTimeLastState should not happen, but algorithm can work with it to propagate backwards
        p_acceleration = p_deltaVelocity / dTimeLastState;
        p_angularRate = p_deltaTheta / dTimeLastState;
    }

    _measurements.push_back(Measurement{ .dt = dTimeLastState,
                                         .p_acceleration = p_acceleration,
                                         .p_angularRate = p_angularRate,
                                         .p_biasAcceleration = p_lastBiasAcceleration,
                                         .p_biasAngularRate = p_lastBiasAngularRate });

    return calcInertialSolutionFromMeasurementBuffer(imuPos);
}

std::shared_ptr<PosVelAtt> InertialIntegrator::calcInertialSolutionFromMeasurementBuffer(const ImuPos& imuPos)
{
    LOG_DATA("New measurement: dt = {:.5f}, p_accel [{}], p_angRate [{}], p_biasAccel [{}], p_biasAngRate [{}]", _measurements.back().dt,
             _measurements.back().p_acceleration.transpose(), _measurements.back().p_angularRate.transpose(),
             _measurements.back().p_biasAcceleration.transpose(), _measurements.back().p_biasAngularRate.transpose());

    if (std::abs(_measurements.back().dt) < 1e-8) // e.g. Initial state at 0.0s, first measurement at 0.0s --> Send out initial state
    {
        return std::make_shared<PosVelAtt>(_states.back());
    }

    if (_measurements.size() == 1) // e.g. Initial state at 0.0s, first measurement at 0.1s -> Assuming constant acceleration and angular rate
    {
        _measurements.push_back(_measurements.back());
    }

    const auto& posVelAtt__t1 = _states.back();

    Eigen::Matrix<double, 10, 1> y;
    switch (_integrationFrame)
    {
    case IntegrationFrame::NED:
        //  0  1  2  3   4    5    6   7  8  9
        // [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
        y.segment<4>(0) = Eigen::Vector4d{ posVelAtt__t1.n_Quat_b().w(), posVelAtt__t1.n_Quat_b().x(), posVelAtt__t1.n_Quat_b().y(), posVelAtt__t1.n_Quat_b().z() };
        y.segment<3>(4) = posVelAtt__t1.n_velocity();
        y.segment<3>(7) = posVelAtt__t1.lla_position();
        break;
    case IntegrationFrame::ECEF:
        //  0  1  2  3   4    5    6   7  8  9
        // [w, x, y, z, v_x, v_y, v_z, x, y, z]^T
        y.segment<4>(0) = Eigen::Vector4d{ posVelAtt__t1.e_Quat_b().w(), posVelAtt__t1.e_Quat_b().x(), posVelAtt__t1.e_Quat_b().y(), posVelAtt__t1.e_Quat_b().z() };
        y.segment<3>(4) = posVelAtt__t1.e_velocity();
        y.segment<3>(7) = posVelAtt__t1.e_position();
        break;
    }

    double dt = _measurements.back().dt;

    if (_measurements.size() == 2)
    {
        Eigen::Vector3d b_accel__t1 = imuPos.b_quatAccel_p() * (_measurements.front().p_acceleration + _measurements.front().p_biasAcceleration);
        Eigen::Vector3d b_gyro__t1 = imuPos.b_quatGyro_p() * (_measurements.front().p_angularRate + _measurements.front().p_biasAngularRate);
        Eigen::Vector3d b_accel__t0 = imuPos.b_quatAccel_p() * (_measurements.back().p_acceleration + _measurements.back().p_biasAcceleration);
        Eigen::Vector3d b_gyro__t0 = imuPos.b_quatGyro_p() * (_measurements.back().p_angularRate + _measurements.back().p_biasAngularRate);

        LOG_DATA("[{:.1f}] p_accel__t0 [{:+.10f} {:+.10f} {:+.10f}]; p_accel__t1 [{:+.10f} {:+.10f} {:+.10f}]",
                 dt, b_accel__t0.x(), b_accel__t0.y(), b_accel__t0.z(), b_accel__t1.x(), b_accel__t1.y(), b_accel__t1.z());
        LOG_DATA("[{:.1f}] p_gyro__t0 [{:+.10f} {:+.10f} {:+.10f}]; p_gyro__t1 [{:+.10f} {:+.10f} {:+.10f}]",
                 dt, b_gyro__t0.x(), b_gyro__t0.y(), b_gyro__t0.z(), b_gyro__t1.x(), b_gyro__t1.y(), b_gyro__t1.z());

        if (_integrationAlgorithm == IntegrationAlgorithm::SingleStepRungeKutta1)
        {
            std::array<Eigen::Vector<double, 6>, 1> z;
            z[0] << b_accel__t1, b_gyro__t1;
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta1(y, z, dt, n_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta1(y, z, dt, e_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            }
        }
        else if (_integrationAlgorithm == IntegrationAlgorithm::SingleStepRungeKutta2)
        {
            std::array<Eigen::Vector<double, 6>, 2> z;
            z[0] << b_accel__t1, b_gyro__t1;
            z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta2(y, z, dt, n_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta2(y, z, dt, e_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            }
        }
        else if (_integrationAlgorithm == IntegrationAlgorithm::SingleStepHeun2)
        {
            std::array<Eigen::Vector<double, 6>, 2> z;
            z[0] << b_accel__t1, b_gyro__t1;
            z[1] << b_accel__t0, b_gyro__t0;
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = Heun2(y, z, dt, n_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = Heun2(y, z, dt, e_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            }
        }
        else if (_integrationAlgorithm == IntegrationAlgorithm::SingleStepRungeKutta3
                 || _integrationAlgorithm == IntegrationAlgorithm::MultiStepRK3)
        {
            std::array<Eigen::Vector<double, 6>, 3> z;
            z[0] << b_accel__t1, b_gyro__t1;
            z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
            z[2] << b_accel__t0, b_gyro__t0;
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta3(y, z, dt, n_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta3(y, z, dt, e_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            }
        }
        else if (_integrationAlgorithm == IntegrationAlgorithm::SingleStepHeun3)
        {
            std::array<Eigen::Vector<double, 6>, 3> z;
            z[0] << b_accel__t1, b_gyro__t1;
            z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 1.0 / 3.0);
            z[2] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 2.0 / 3.0);
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta3(y, z, dt, n_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta3(y, z, dt, e_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            }
        }
        else if (_integrationAlgorithm == IntegrationAlgorithm::SingleStepRungeKutta4
                 || _integrationAlgorithm == IntegrationAlgorithm::MultiStepRK4)
        {
            std::array<Eigen::Vector<double, 6>, 4> z;
            z[0] << b_accel__t1, b_gyro__t1;
            z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
            z[2] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
            z[3] << b_accel__t0, b_gyro__t0;
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta4(y, z, dt, n_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta4(y, z, dt, e_calcPosVelAttDerivative<double>, _posVelAttDerivativeConstants);
                break;
            }
        }
    }
    else // if (_measurements.size() == 3)
    {
        if (_integrationAlgorithm == IntegrationAlgorithm::MultiStepRK3)
        {
            LOG_ERROR("Not implemented yet"); // TODO
        }
        else if (_integrationAlgorithm == IntegrationAlgorithm::MultiStepRK4)
        {
            LOG_ERROR("Not implemented yet"); // TODO
        }
    }

    auto posVelAtt__t0 = std::make_shared<PosVelAtt>();
    posVelAtt__t0->insTime = posVelAtt__t1.insTime + std::chrono::duration<double>(dt);
    switch (_integrationFrame)
    {
    case IntegrationFrame::NED:
        posVelAtt__t0->setState_n(y.segment<3>(7), y.segment<3>(4), Eigen::Quaterniond{ y(0), y(1), y(2), y(3) }.normalized());
        break;
    case IntegrationFrame::ECEF:
        posVelAtt__t0->setState_e(y.segment<3>(7), y.segment<3>(4), Eigen::Quaterniond{ y(0), y(1), y(2), y(3) }.normalized());
        break;
    }

    LOG_DATA("posVelAtt__t0->e_position() = {}", posVelAtt__t0->e_position().transpose());
    LOG_DATA("posVelAtt__t0->lla_position() - posVelAtt__t1->lla_position() = {} [m]",
             calcGeographicalDistance(posVelAtt__t0->latitude(), posVelAtt__t0->longitude(), posVelAtt__t1.latitude(), posVelAtt__t1.longitude()));
    LOG_DATA("posVelAtt__t0->n_velocity() = {}", posVelAtt__t0->n_velocity().transpose());
    LOG_DATA("posVelAtt__t0->e_velocity() = {}", posVelAtt__t0->e_velocity().transpose());
    LOG_DATA("posVelAtt__t0->n_Quat_b() = {}", posVelAtt__t0->n_Quat_b());
    LOG_DATA("posVelAtt__t0->e_Quat_b() = {}", posVelAtt__t0->e_Quat_b());

    _states.push_back(*posVelAtt__t0);
    return posVelAtt__t0;
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
        _measurements.resize(2);
        _states.resize(1);
        break;
    case IntegrationAlgorithm::MultiStepRK3:
    case IntegrationAlgorithm::MultiStepRK4:
        _measurements.resize(3);
        _states.resize(2);
        break;
    case IntegrationAlgorithm::COUNT:
        break;
    }
}

bool InertialIntegratorGui(const char* label, InertialIntegrator& integrator, float width)
{
    bool changed = false;

    ImGui::SetNextItemWidth(width * gui::NodeEditorApplication::windowFontRatio());
    if (auto integrationFrame = static_cast<int>(integrator._integrationFrame);
        ImGui::Combo(fmt::format("Integration Frame##{}", label).c_str(), &integrationFrame, "ECEF\0NED\0\0"))
    {
        integrator._integrationFrame = static_cast<decltype(integrator._integrationFrame)>(integrationFrame);
        LOG_DEBUG("Integration Frame changed to {}", integrator._integrationFrame == InertialIntegrator::IntegrationFrame::NED ? "NED" : "ECEF");
        changed = true;
    }

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
        { "integrationAlgorithm", data._integrationAlgorithm },
        { "posVelAttDerivativeConstants", data._posVelAttDerivativeConstants },
    };
}

void from_json(const json& j, InertialIntegrator& data)
{
    if (j.contains("integrationFrame")) { j.at("integrationFrame").get_to(data._integrationFrame); }
    if (j.contains("integrationAlgorithm"))
    {
        j.at("integrationAlgorithm").get_to(data._integrationAlgorithm);
        data.setBufferSizes();
    }
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

} // namespace NAV