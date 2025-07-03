// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InertialPreIntegrator.cpp
/// @brief Inertial Measurement Preintegrator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2025-07-03

#include "InertialPreIntegrator.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

#include "Navigation/Math/Math.hpp"
#include "util/Eigen.hpp"
#include "util/Assert.h"

namespace NAV
{

InertialPreIntegrator::InertialPreIntegrator(IntegrationFrame integrationFrame)
    : _integrationFrame(integrationFrame), _lockIntegrationFrame(true) {}

void InertialPreIntegrator::reset()
{
    _timeIncrement = 0.0;
    _attitudeIncrement.setIdentity();
    _velocityIncrement.setZero();
    _positionIncrement.setZero();
    _covMatrix.setZero();

    _pAtt_pOmega.setZero();
    _pVel_pAccel.setZero();
    _pVel_pOmega.setZero();
    _pPos_pAccel.setZero();
    _pPos_pOmega.setZero();
}

void InertialPreIntegrator::setImuNoiseScaleMatrix(const Eigen::Matrix6d& W)
{
    _imuNoiseScale_W = W;
}

void InertialPreIntegrator::addInertialMeasurement(const Measurement& meas, const ImuPos& imuPos, [[maybe_unused]] const std::string& nameId)
{
    INS_ASSERT_USER_ERROR((_attitudeIncrement == Eigen::Quaterniond::Identity()
                           && _velocityIncrement == Eigen::Vector3d::Zero()
                           && _positionIncrement == Eigen::Vector3d::Zero())
                              || _imuState == meas.imu,
                          "The IMU state (bias, scale factor, misalignment) must be constant over the whole increment period.");

    // LOG_DATA("{}: addInertialMeasurement (dt = {})", nameId, meas.meas.dt);
    // LOG_DATA("{}:   timeIncrement     (before) = {}", nameId, _timeIncrement);
    // LOG_DATA("{}:   attitudeIncrement (before) = {}", nameId, _attitudeIncrement);
    // LOG_DATA("{}:   velocityIncrement (before) = {}", nameId, _velocityIncrement.transpose());
    // LOG_DATA("{}:   positionIncrement (before) = {}", nameId, _positionIncrement.transpose());

    Eigen::Vector3d b_deltaRot = imuPos.b_quat_p()
                                 * (/* (meas.imu.misalignmentGyro * */ meas.meas.p_angularRate /* ).cwiseProduct(meas.imu.scaleFactorGyro) */
                                    + meas.imu.p_biasAngularRate)
                                 * meas.meas.dt;
    Eigen::Quaterniond bk_quat_bkp1 = math::expMapQuat(b_deltaRot);
    // LOG_DATA("{}:   bk_quat_bkp1 = {}", nameId, bk_quat_bkp1);
    Eigen::Vector3d b_accel = (imuPos.b_quat_p()
                               * (/* (meas.imu.misalignmentAccel * */ meas.meas.p_acceleration /* ).cwiseProduct(meas.imu.scaleFactorAccel) */
                                  + meas.imu.p_biasAcceleration));
    Eigen::Vector3d dv_k_kp1 = _attitudeIncrement * b_accel * meas.meas.dt;
    // LOG_DATA("{}:   dv_k_kp1 = {}", nameId, dv_k_kp1.transpose());
    Eigen::Vector3d dp_k_kp1 = (_velocityIncrement + 0.5 * dv_k_kp1) * meas.meas.dt;
    // LOG_DATA("{}:   dp_k_kp1 = {}", nameId, dp_k_kp1.transpose());

    // --------------------------------------- Jacobian calculation ------------------------------------------
    Eigen::Matrix3d i_R_k = _attitudeIncrement.toRotationMatrix();
    Eigen::Matrix3d b_accelSkew = math::skewSymmetricMatrix(b_accel);
    Eigen::Matrix3d J_r_rot = math::J_r(b_deltaRot);
    Eigen::Quaterniond bkp1_quat_bk = bk_quat_bkp1.conjugate();

    _pAtt_pOmega = bkp1_quat_bk * _pAtt_pOmega + J_r_rot * meas.meas.dt;
    _pVel_pAccel += i_R_k * meas.meas.dt;
    Eigen::Matrix3d pDeltaVel_pOmega = _attitudeIncrement * b_accelSkew * _pAtt_pOmega * meas.meas.dt;
    _pVel_pOmega -= pDeltaVel_pOmega;
    _pPos_pAccel += _pVel_pAccel * meas.meas.dt - i_R_k * (std::pow(meas.meas.dt, 2) / 2.0);
    _pPos_pOmega += _pVel_pOmega * meas.meas.dt - pDeltaVel_pOmega * (meas.meas.dt / 2.0);

    // -------------------------------------- Covariance propagation -----------------------------------------
    if (_imuNoiseScale_W)
    {
        Eigen::Matrix9d A = Eigen::Matrix9d::Zero();
        A.block<3, 3>(0, 0) = bkp1_quat_bk.toRotationMatrix();
        A.block<3, 3>(3, 0) = -i_R_k * b_accelSkew * meas.meas.dt;
        A.block<3, 3>(3, 3).setIdentity();
        A.block<3, 3>(6, 0) = 0.5 * A.block<3, 3>(3, 0) * meas.meas.dt;
        A.block<3, 3>(6, 3).diagonal().setConstant(meas.meas.dt);
        A.block<3, 3>(6, 6).setIdentity();

        Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
        B.block<3, 3>(0, 0) = J_r_rot * meas.meas.dt;
        B.block<3, 3>(3, 3) = i_R_k * meas.meas.dt;
        B.block<3, 3>(6, 3) = 0.5 * B.block<3, 3>(3, 3) * meas.meas.dt;

        _covMatrix = A * _covMatrix * A.transpose() + B * (*_imuNoiseScale_W * meas.meas.dt) * B.transpose();
    }

    // --------------------------------------- Accumulate increments -----------------------------------------

    _timeIncrement += meas.meas.dt;
    _attitudeIncrement *= bk_quat_bkp1; // bi_q_bk --> bi_q_bkp1 ( final value: bi_q_bj )
    _velocityIncrement += dv_k_kp1;
    _positionIncrement += dp_k_kp1;
    // LOG_DATA("{}:   timeIncrement     (after)  = {}", nameId, _timeIncrement);
    // LOG_DATA("{}:   attitudeIncrement (after)  = {}", nameId, _attitudeIncrement);
    // LOG_DATA("{}:   velocityIncrement (after)  = {}", nameId, _velocityIncrement.transpose());
    // LOG_DATA("{}:   positionIncrement (after)  = {}", nameId, _positionIncrement.transpose());

    _imuState = meas.imu;
}

[[nodiscard]] const Eigen::Matrix9d& InertialPreIntegrator::getCovMatrix() const
{
    return _covMatrix;
}

[[nodiscard]] InertialPreIntegrator::IntegrationFrame InertialPreIntegrator::getIntegrationFrame() const
{
    return _integrationFrame;
}

bool InertialPreIntegratorGui(const char* label, InertialPreIntegrator& integrator, float width)
{
    bool changed = false;

    if (integrator._lockIntegrationFrame) { ImGui::BeginDisabled(); }
    ImGui::SetNextItemWidth(width * gui::NodeEditorApplication::windowFontRatio());
    if (auto integrationFrame = static_cast<int>(integrator._integrationFrame);
        ImGui::Combo(fmt::format("Integration Frame##{}", label).c_str(), &integrationFrame, "ECEF\0NED\0\0"))
    {
        integrator._integrationFrame = static_cast<decltype(integrator._integrationFrame)>(integrationFrame);
        LOG_DEBUG("Integration Frame changed to {}", integrator._integrationFrame == InertialPreIntegrator::IntegrationFrame::NED ? "NED" : "ECEF");
        changed = true;
    }
    if (integrator._lockIntegrationFrame) { ImGui::EndDisabled(); }

    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Compensation models##{}", label).c_str()))
    {
        ImGui::TextUnformatted("Acceleration compensation");
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(230 * gui::NodeEditorApplication::windowFontRatio());
            if (ComboGravitationModel(fmt::format("Gravitation Model##{}", label).c_str(), integrator._gravitationModel))
            {
                LOG_DEBUG("Gravity Model changed to {}", NAV::to_string(integrator._gravitationModel));
                changed = true;
            }
            // if (ImGui::Checkbox(fmt::format("Coriolis acceleration ##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.coriolisAccelerationCompensationEnabled))
            // {
            //     LOG_DEBUG("coriolisAccelerationCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.coriolisAccelerationCompensationEnabled);
            //     changed = true;
            // }
            // if (ImGui::Checkbox(fmt::format("Centrifugal acceleration##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.centrifgalAccelerationCompensationEnabled))
            // {
            //     LOG_DEBUG("centrifgalAccelerationCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.centrifgalAccelerationCompensationEnabled);
            //     changed = true;
            // }
            ImGui::Unindent();
        }
        // ImGui::TextUnformatted("Angular rate compensation");
        // {
        //     ImGui::Indent();
        //     if (ImGui::Checkbox(fmt::format("Earth rotation rate##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.angularRateEarthRotationCompensationEnabled))
        //     {
        //         LOG_DEBUG("angularRateEarthRotationCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.angularRateEarthRotationCompensationEnabled);
        //         changed = true;
        //     }
        //     if (integrator._integrationFrame == InertialPreIntegrator::IntegrationFrame::NED)
        //     {
        //         if (ImGui::Checkbox(fmt::format("Transport rate##{}", label).c_str(), &integrator._posVelAttDerivativeConstants.angularRateTransportRateCompensationEnabled))
        //         {
        //             LOG_DEBUG("angularRateTransportRateCompensationEnabled changed to {}", integrator._posVelAttDerivativeConstants.angularRateTransportRateCompensationEnabled);
        //             changed = true;
        //         }
        //     }
        //     ImGui::Unindent();
        // }

        ImGui::TreePop();
    }

    return changed;
}

void to_json(json& j, const InertialPreIntegrator& data)
{
    j = json{
        { "integrationFrame", data._integrationFrame },
        { "gravitationModel", data._gravitationModel },
        { "lockIntegrationFrame", data._lockIntegrationFrame },
    };
}

void from_json(const json& j, InertialPreIntegrator& data)
{
    if (j.contains("integrationFrame")) { j.at("integrationFrame").get_to(data._integrationFrame); }
    if (j.contains("gravitationModel")) { j.at("gravitationModel").get_to(data._gravitationModel); }
    if (j.contains("lockIntegrationFrame")) { j.at("lockIntegrationFrame").get_to(data._lockIntegrationFrame); }
}

const char* to_string(InertialPreIntegrator::IntegrationFrame frame)
{
    switch (frame)
    {
    case InertialPreIntegrator::IntegrationFrame::ECEF:
        return "ECEF";
    case InertialPreIntegrator::IntegrationFrame::NED:
        return "NED";
    }
    return "";
}

} // namespace NAV