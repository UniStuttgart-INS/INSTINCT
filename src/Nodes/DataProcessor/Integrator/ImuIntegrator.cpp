#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/LocalNavFrame/Mechanization.hpp"
#include "Navigation/INS/EcefFrame/Mechanization.hpp"
#include "Navigation/Math/Math.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include <imgui_internal.h>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/InertialNavSol.hpp"

#include <algorithm>

NAV::ImuIntegrator::ImuIntegrator()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 483, 350 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuIntegrator::recvImuObs,
                       [](const Node* node, const InputPin& inputPin) {
                           auto imuIntegrator = static_cast<const ImuIntegrator*>(node);
                           return !inputPin.queue.empty() && !imuIntegrator->_posVelAttStates.empty();
                       });
    nm::CreateInputPin(this, "PosVelAttInit", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &ImuIntegrator::recvPosVelAttInit);

    nm::CreateOutputPin(this, "InertialNavSol", Pin::Type::Flow, { NAV::InertialNavSol::type() });
}

NAV::ImuIntegrator::~ImuIntegrator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuIntegrator::typeStatic()
{
    return "ImuIntegrator";
}

std::string NAV::ImuIntegrator::type() const
{
    return typeStatic();
}

std::string NAV::ImuIntegrator::category()
{
    return "Data Processor";
}

void NAV::ImuIntegrator::guiConfig()
{
    ImGui::SetNextItemWidth(250 * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::Combo(fmt::format("Integration Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_integrationFrame), "ECEF\0NED\0\0"))
    {
        LOG_DEBUG("{}: Integration Frame changed to {}", nameId(), _integrationFrame == IntegrationFrame::NED ? "NED" : "ECEF");
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(250 * gui::NodeEditorApplication::windowFontRatio());
    if (ImGui::BeginCombo(fmt::format("Integration Algorithm##{}", size_t(id)).c_str(), to_string(_integrationAlgorithm)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IntegrationAlgorithm::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(_integrationAlgorithm) == i);
            if (ImGui::Selectable(to_string(static_cast<IntegrationAlgorithm>(i)), is_selected))
            {
                _integrationAlgorithm = static_cast<IntegrationAlgorithm>(i);
                LOG_DEBUG("{}: Integration Algorithm Attitude changed to {}", nameId(), _integrationAlgorithm);
                flow::ApplyChanges();
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }

    if (ImGui::TreeNode(fmt::format("Data selection##{}", size_t(id)).c_str()))
    {
        if (ImGui::Checkbox(fmt::format("Prefere TimeSinceStartup over InsTime##{}", size_t(id)).c_str(), &_prefereTimeSinceStartupOverInsTime))
        {
            LOG_DEBUG("{}: prefereTimeSinceStartupOverInsTime changed to {}", nameId(), _prefereTimeSinceStartupOverInsTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Takes the IMU internal 'TimeSinceStartup' value instead of the absolute 'insTime'");

        if (ImGui::Checkbox(fmt::format("Prefere uncompensated data##{}", size_t(id)).c_str(), &_prefereUncompensatedData))
        {
            LOG_DEBUG("{}: prefereUncompensatedData changed to {}", nameId(), _prefereUncompensatedData);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Takes the uncompensated acceleration and angular rates instead of compensated values from the sensor. This option should be used when a Kalman Filter is used to correct the biases.");

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Compensation models##{}", size_t(id)).c_str()))
    {
        ImGui::TextUnformatted("Acceleration compensation");
        {
            ImGui::Indent();
            ImGui::SetNextItemWidth(230 * gui::NodeEditorApplication::windowFontRatio());
            if (ComboGravitationModel(fmt::format("Gravitation Model##{}", size_t(id)).c_str(), _gravitationModel))
            {
                LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), NAV::to_string(_gravitationModel));
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Coriolis acceleration ##{}", size_t(id)).c_str(), &_coriolisAccelerationCompensationEnabled))
            {
                LOG_DEBUG("{}: coriolisAccelerationCompensationEnabled changed to {}", nameId(), _coriolisAccelerationCompensationEnabled);
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Centrifugal acceleration##{}", size_t(id)).c_str(), &_centrifgalAccelerationCompensationEnabled))
            {
                LOG_DEBUG("{}: centrifgalAccelerationCompensationEnabled changed to {}", nameId(), _centrifgalAccelerationCompensationEnabled);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TextUnformatted("Angular rate compensation");
        {
            ImGui::Indent();
            if (ImGui::Checkbox(fmt::format("Earth rotation rate##{}", size_t(id)).c_str(), &_angularRateEarthRotationCompensationEnabled))
            {
                LOG_DEBUG("{}: angularRateEarthRotationCompensationEnabled changed to {}", nameId(), _angularRateEarthRotationCompensationEnabled);
                flow::ApplyChanges();
            }
            if (_integrationFrame == IntegrationFrame::NED)
            {
                if (ImGui::Checkbox(fmt::format("Transport rate##{}", size_t(id)).c_str(), &_angularRateTransportRateCompensationEnabled))
                {
                    LOG_DEBUG("{}: angularRateTransportRateCompensationEnabled changed to {}", nameId(), _angularRateTransportRateCompensationEnabled);
                    flow::ApplyChanges();
                }
            }
            ImGui::Unindent();
        }
        ImGui::TreePop();
    }

    ImGui::Separator();
    if (ImGui::Checkbox(fmt::format("Show Corrections input pins##{}", size_t(id)).c_str(), &_showCorrectionsInputPin))
    {
        LOG_DEBUG("{}: showCorrectionsInputPin changed to {}", nameId(), _showCorrectionsInputPin);

        if (_showCorrectionsInputPin && inputPins.size() < 4)
        {
            nm::CreateInputPin(this, "Errors", Pin::Type::Flow, { LcKfInsGnssErrors::type() }, &ImuIntegrator::recvLcKfInsGnssErrors);
            nm::CreateInputPin(this, "Predict", Pin::Type::Flow, { ImuObs::type() }, &ImuIntegrator::recvImuObs);
        }
        else if (!_showCorrectionsInputPin)
        {
            while (inputPins.size() >= 3)
            {
                inputPins.pop_back();
            }
        }

        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::ImuIntegrator::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["integrationFrame"] = _integrationFrame;
    j["integrationAlgorithm"] = _integrationAlgorithm;
    // #########################################################################################################################################
    j["prefereTimeSinceStartupOverInsTime"] = _prefereTimeSinceStartupOverInsTime;
    j["prefereUncompensatedData"] = _prefereUncompensatedData;
    // #########################################################################################################################################
    j["gravitationModel"] = _gravitationModel;
    j["coriolisAccelerationCompensationEnabled"] = _coriolisAccelerationCompensationEnabled;
    j["centrifgalAccelerationCompensationEnabled"] = _centrifgalAccelerationCompensationEnabled;
    j["angularRateEarthRotationCompensationEnabled"] = _angularRateEarthRotationCompensationEnabled;
    j["angularRateTransportRateCompensationEnabled"] = _angularRateTransportRateCompensationEnabled;
    // #########################################################################################################################################
    j["showCorrectionsInputPin"] = _showCorrectionsInputPin;

    return j;
}

void NAV::ImuIntegrator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("integrationFrame"))
    {
        j.at("integrationFrame").get_to(_integrationFrame);
    }
    if (j.contains("integrationAlgorithm"))
    {
        j.at("integrationAlgorithm").get_to(_integrationAlgorithm);
    }
    // #########################################################################################################################################
    if (j.contains("prefereTimeSinceStartupOverInsTime"))
    {
        _prefereTimeSinceStartupOverInsTime = j.at("prefereTimeSinceStartupOverInsTime");
    }
    if (j.contains("prefereUncompensatedData"))
    {
        _prefereUncompensatedData = j.at("prefereUncompensatedData");
    }
    // #########################################################################################################################################
    if (j.contains("gravitationModel"))
    {
        j.at("gravitationModel").get_to(_gravitationModel);
    }
    if (j.contains("coriolisAccelerationCompensationEnabled"))
    {
        _coriolisAccelerationCompensationEnabled = j.at("coriolisAccelerationCompensationEnabled");
    }
    if (j.contains("centrifgalAccelerationCompensationEnabled"))
    {
        _centrifgalAccelerationCompensationEnabled = j.at("centrifgalAccelerationCompensationEnabled");
    }
    if (j.contains("angularRateEarthRotationCompensationEnabled"))
    {
        _angularRateEarthRotationCompensationEnabled = j.at("angularRateEarthRotationCompensationEnabled");
    }
    if (j.contains("angularRateTransportRateCompensationEnabled"))
    {
        _angularRateTransportRateCompensationEnabled = j.at("angularRateTransportRateCompensationEnabled");
    }
    // #########################################################################################################################################
    if (j.contains("showCorrectionsInputPin"))
    {
        _showCorrectionsInputPin = j.at("showCorrectionsInputPin");
        if (_showCorrectionsInputPin && inputPins.size() < 4)
        {
            nm::CreateInputPin(this, "Errors", Pin::Type::Flow, { LcKfInsGnssErrors::type() }, &ImuIntegrator::recvLcKfInsGnssErrors);
            nm::CreateInputPin(this, "Predict", Pin::Type::Flow, { ImuObs::type() }, &ImuIntegrator::recvImuObs);
        }
        else if (!_showCorrectionsInputPin)
        {
            while (inputPins.size() >= 3)
            {
                inputPins.pop_back();
            }
        }
    }
}

bool NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // This should be dependant on the integration algorithm
    switch (_integrationAlgorithm)
    {
    case IntegrationAlgorithm::Heun:
    case IntegrationAlgorithm::RungeKutta1:
    case IntegrationAlgorithm::RungeKutta2:
    case IntegrationAlgorithm::RungeKutta3:
    case IntegrationAlgorithm::RungeKutta4:
        _maxSizeImuObservations = 2; // Has to be >= 2
        _maxSizeStates = 1;
        break;
    case IntegrationAlgorithm::COUNT:
        return false;
    }

    _imuObservations.clear();
    _posVelAttStates.clear();
    _lckfErrors.reset();

    _time__init.reset();
    _timeSinceStartup__init = 0;
    inputPins[INPUT_PORT_INDEX_POS_VEL_ATT_INIT].queueBlocked = false;

    LOG_DEBUG("ImuIntegrator initialized");

    return true;
}

void NAV::ImuIntegrator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::ImuIntegrator::recvPosVelAttInit(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    LOG_DATA("{}: recvPosVelAttInit", nameId());
    auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(queue.extract_front());

    inputPins[INPUT_PORT_INDEX_POS_VEL_ATT_INIT].queueBlocked = true;

    // Fill the list with the initial state to the start of the list
    if (_posVelAttStates.empty())
    {
        while (_posVelAttStates.size() < _maxSizeStates)
        {
            LOG_DATA("{}: Adding posVelAtt to the start of the list {}", nameId(), posVelAtt);
            _posVelAttStates.push_front(posVelAtt);
        }

        if (_imuObservations.size() >= _maxSizeImuObservations - 1)
        {
            // Push out a message with the initial state and a matching imu Observation
            auto inertialNavSol = std::make_shared<InertialNavSol>();

            inertialNavSol->setState_n(posVelAtt->lla_position(), posVelAtt->n_velocity(), posVelAtt->n_Quat_b());

            auto imuObsIndex = std::min(static_cast<size_t>(1), _imuObservations.size() - 1); // Casting to int, because Windows does not support std::min(size_t, size_t)

            inertialNavSol->insTime = _imuObservations.at(imuObsIndex)->insTime;
            inertialNavSol->imuObs = _imuObservations.at(imuObsIndex);

            invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, inertialNavSol);
        }

        // If enough imu observations received, integrate the observation
        if (_imuObservations.size() == _maxSizeImuObservations)
        {
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                integrateObservationNED();
                break;
            case IntegrationFrame::ECEF:
                integrateObservationECEF();
                break;
            }
        }
    }
}

void NAV::ImuIntegrator::recvImuObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto imuObs = std::static_pointer_cast<const ImuObs>(queue.extract_front());
    LOG_DATA("{}: recvImuObs at time [{}]", nameId(), imuObs->insTime.toYMDHMS());

    if (imuObs->insTime.empty() && !imuObs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime/timeSinceStartup)", nameId());
        return;
    }

    // Add imuObs tₖ to the start of the list
    _imuObservations.push_front(imuObs);

    // Remove observations at the end of the list till the max size is reached
    while (_imuObservations.size() > _maxSizeImuObservations)
    {
        if (!_posVelAttStates.empty())
        {
            LOG_WARN("{}: Receive new Imu observation, but list is full --> discarding oldest observation", nameId());
        }
        _imuObservations.pop_back();
    }

    // First ImuObs and already has state
    if (_imuObservations.size() == 1 && _posVelAttStates.size() == _maxSizeStates)
    {
        // Push out a message with the initial state and a matching imu Observation
        auto inertialNavSol = std::make_shared<InertialNavSol>();

        inertialNavSol->setState_n(_posVelAttStates.front()->lla_position(),
                                   _posVelAttStates.front()->n_velocity(),
                                   _posVelAttStates.front()->n_Quat_b());

        inertialNavSol->insTime = imuObs->insTime;
        inertialNavSol->imuObs = imuObs;

        invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, inertialNavSol);
        return;
    }

    // If enough imu observations and states received, integrate the observation
    if (_imuObservations.size() == _maxSizeImuObservations
        && _posVelAttStates.size() == _maxSizeStates)
    {
        switch (_integrationFrame)
        {
        case IntegrationFrame::NED:
            integrateObservationNED();
            break;
        case IntegrationFrame::ECEF:
            integrateObservationECEF();
            break;
        }
    }
}

void NAV::ImuIntegrator::recvLcKfInsGnssErrors(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto lcKfInsGnssErrors = std::static_pointer_cast<const LcKfInsGnssErrors>(queue.extract_front());
    LOG_DATA("{}: recvLcKfInsGnssErrors at time [{}]", nameId(), lcKfInsGnssErrors->insTime.toYMDHMS());

    for (auto& posVelAtt : _posVelAttStates)
    {
        LOG_DATA("{}: Correcting posVelAtt at time [{}] with error from time [{}]", nameId(), posVelAtt->insTime.toYMDHMS(), lcKfInsGnssErrors->insTime.toYMDHMS());
        auto posVelAttCorrected = std::make_shared<PosVelAtt>(*posVelAtt);

        if (lcKfInsGnssErrors->frame == LcKfInsGnssErrors::Frame::NED)
        {
            LOG_DATA("{}:     velocity ({}) - ({})", nameId(), posVelAtt->n_velocity().transpose(), lcKfInsGnssErrors->velocityError.transpose());
            LOG_DATA("{}:     position ({}) - ({})", nameId(), posVelAtt->lla_position().transpose(), lcKfInsGnssErrors->positionError.transpose());

            posVelAttCorrected->setPosition_lla(posVelAtt->lla_position() - lcKfInsGnssErrors->positionError);

            posVelAttCorrected->setVelocity_n(posVelAtt->n_velocity() - lcKfInsGnssErrors->velocityError);

            // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
            Eigen::Matrix3d n_DcmCorrected_b = (Eigen::Matrix3d::Identity() - math::skewSymmetricMatrix(lcKfInsGnssErrors->attitudeError)) * posVelAtt->n_Quat_b().toRotationMatrix();
            posVelAttCorrected->setAttitude_n_Quat_b(Eigen::Quaterniond(n_DcmCorrected_b).normalized());
        }
        else // if (lcKfInsGnssErrors->frame == LcKfInsGnssErrors::Frame::ECEF)
        {
            LOG_DATA("{}:     velocity ({}) - ({})", nameId(), posVelAtt->e_velocity().transpose(), lcKfInsGnssErrors->velocityError.transpose());
            LOG_DATA("{}:     position ({}) - ({})", nameId(), posVelAtt->e_position().transpose(), lcKfInsGnssErrors->positionError.transpose());
            posVelAttCorrected->setPosition_e(posVelAtt->e_position() - lcKfInsGnssErrors->positionError);

            posVelAttCorrected->setVelocity_e(posVelAtt->e_velocity() - lcKfInsGnssErrors->velocityError);

            // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
            Eigen::Matrix3d e_DcmCorrected_b = (Eigen::Matrix3d::Identity() - math::skewSymmetricMatrix(lcKfInsGnssErrors->attitudeError)) * posVelAtt->e_Quat_b().toRotationMatrix();
            posVelAttCorrected->setAttitude_e_Quat_b(Eigen::Quaterniond(e_DcmCorrected_b).normalized());
        }

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

        posVelAtt = posVelAttCorrected;

        LOG_DATA("{}:     = lla_position ({})", nameId(), posVelAtt->lla_position().transpose());
        LOG_DATA("{}:     = n_velocity ({})", nameId(), posVelAtt->n_velocity().transpose());
    }

    _lckfErrors = lcKfInsGnssErrors;
}

void NAV::ImuIntegrator::integrateObservationECEF()
{
    // IMU Observation at the time tₖ
    const std::shared_ptr<const ImuObs>& imuObs__t0 = _imuObservations.at(0);
    // IMU Observation at the time tₖ₋₁
    const std::shared_ptr<const ImuObs>& imuObs__t1 = _imuObservations.at(1);

    // Result State Data at the time tₖ
    auto posVelAtt__t0 = std::make_shared<InertialNavSol>();

    posVelAtt__t0->imuObs = imuObs__t0;

    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0;

    if (!imuObs__t0->insTime.empty() && !(_prefereTimeSinceStartupOverInsTime && imuObs__t0->timeSinceStartup.has_value()))
    {
        // tₖ₋₁ Time at previous epoch
        const InsTime& time__t1 = imuObs__t1->insTime;
        // tₖ Current Time
        const InsTime& time__t0 = imuObs__t0->insTime;

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec = (time__t0 - time__t1).count();

        // Update time
        posVelAtt__t0->insTime = imuObs__t0->insTime;

        LOG_DATA("{}: time__t0 - time__t1 = {} - {} = {}", nameId(), time__t0.toYMDHMS(), time__t1.toYMDHMS(), timeDifferenceSec);
    }
    else
    {
        // tₖ₋₁ Time at previous epoch
        const auto& time__t1 = imuObs__t1->timeSinceStartup.value();
        // tₖ Current Time
        const auto& time__t0 = imuObs__t0->timeSinceStartup.value();

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec = static_cast<long double>(time__t0 - time__t1) * 1e-9L;

        if (_timeSinceStartup__init == 0)
        {
            _timeSinceStartup__init = imuObs__t0->timeSinceStartup.value();
            _time__init = !imuObs__t0->insTime.empty() ? imuObs__t0->insTime : InsTime(2000, 1, 1, 1, 1, 1);
        }

        // Update time
        posVelAtt__t0->insTime = _time__init + std::chrono::nanoseconds(imuObs__t0->timeSinceStartup.value() - _timeSinceStartup__init);

        LOG_DATA("{}: time__t0 - time__t1 = [{}] - [{}] = {}", nameId(), time__t0, time__t1, timeDifferenceSec);
    }
    auto dt = fmt::format("{:0.5f}", timeDifferenceSec);
    dt.erase(std::find_if(dt.rbegin(), dt.rend(), [](char ch) { return ch != '0'; }).base(), dt.end());
    LOG_DATA("{}: Integrating (dt = {}s) from [{}] to [{}] in ECEF frame", nameId(), dt,
             imuObs__t1->insTime.toYMDHMS(), imuObs__t0->insTime.toYMDHMS());

    // Position, Velocity and Attitude at the time tₖ₋₁
    const std::shared_ptr<const PosVelAtt>& posVelAtt__t1 = _posVelAttStates.at(0);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = imuObs__t0->imuPos;

    // ω_ip_p (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d p_omega_ip__t1 = !_prefereUncompensatedData && imuObs__t1->gyroCompXYZ.has_value()
                                         ? imuObs__t1->gyroCompXYZ.value()
                                         : imuObs__t1->gyroUncompXYZ.value();

    // ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d p_omega_ip__t0 = !_prefereUncompensatedData && imuObs__t0->gyroCompXYZ.has_value()
                                         ? imuObs__t0->gyroCompXYZ.value()
                                         : imuObs__t0->gyroUncompXYZ.value();

    // a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d p_f_ip__t1 = !_prefereUncompensatedData && imuObs__t1->accelCompXYZ.has_value()
                                     ? imuObs__t1->accelCompXYZ.value()
                                     : imuObs__t1->accelUncompXYZ.value();

    // a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    Eigen::Vector3d p_f_ip__t0 = !_prefereUncompensatedData && imuObs__t0->accelCompXYZ.has_value()
                                     ? imuObs__t0->accelCompXYZ.value()
                                     : imuObs__t0->accelUncompXYZ.value();

    if (_lckfErrors)
    {
        LOG_DATA("{}: Applying IMU Biases p_quatGyro_b {}, p_quatAccel_b {}", nameId(), imuPosition.p_quatGyro_b(), imuPosition.p_quatAccel_b());
        p_omega_ip__t1 -= imuPosition.p_quatGyro_b() * _lckfErrors->b_biasGyro;
        p_omega_ip__t0 -= imuPosition.p_quatGyro_b() * _lckfErrors->b_biasGyro;
        p_f_ip__t1 -= imuPosition.p_quatAccel_b() * _lckfErrors->b_biasAccel;
        p_f_ip__t0 -= imuPosition.p_quatAccel_b() * _lckfErrors->b_biasAccel;
    }
    LOG_DATA("{}: p_omega_ip__t1 = {}", nameId(), p_omega_ip__t1.transpose());
    LOG_DATA("{}: p_omega_ip__t0 = {}", nameId(), p_omega_ip__t0.transpose());
    LOG_DATA("{}: p_f_ip__t1 = {}", nameId(), p_f_ip__t1.transpose());
    LOG_DATA("{}: p_f_ip__t0 = {}", nameId(), p_f_ip__t0.transpose());

    // q (tₖ₋₁) Quaternion, from body to ECEF coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& e_Quat_b__t1 = posVelAtt__t1->e_Quat_b();
    LOG_DATA("{}: e_Quat_b__t1 = {}", nameId(), e_Quat_b__t1);

    // e_velocity (tₖ₋₁) Velocity in [m/s], in ECEF coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& e_velocity__t1 = posVelAtt__t1->e_velocity();
    LOG_DATA("{}: e_velocity__t1 = {}", nameId(), e_velocity__t1.transpose());

    // x (tₖ₋₁) Position in [m] in ECEF coordinates at the time tₖ₋₁
    const Eigen::Vector3d& e_position__t1 = posVelAtt__t1->e_position();
    LOG_DATA("{}: e_position__t1 = {}", nameId(), e_position__t1.transpose());

    // ω_ip_b (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ₋₁
    [[maybe_unused]] Eigen::Vector3d b_omega_ip__t1 = imuPosition.b_quatGyro_p() * p_omega_ip__t1;
    LOG_DATA("{}: b_omega_ip__t1 = {}", nameId(), b_omega_ip__t1.transpose());
    // ω_ip_b (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ
    [[maybe_unused]] Eigen::Vector3d b_omega_ip__t0 = imuPosition.b_quatGyro_p() * p_omega_ip__t0;
    LOG_DATA("{}: b_omega_ip__t0 = {}", nameId(), b_omega_ip__t0.transpose());

    // f_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
    [[maybe_unused]] Eigen::Vector3d b_f__t1 = imuPosition.b_quatAccel_p() * p_f_ip__t1;
    LOG_DATA("{}: b_f__t1 = {}", nameId(), b_f__t1.transpose());
    // f_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
    [[maybe_unused]] Eigen::Vector3d b_f__t0 = imuPosition.b_quatAccel_p() * p_f_ip__t0;
    LOG_DATA("{}: b_f__t0 = {}", nameId(), b_f__t0.transpose());

    //  0  1  2  3   4    5    6   7  8  9  10  11  12  13  14  15
    // [w, x, y, z, v_x, v_y, v_z, x, y, z, fx, fy, fz, ωx, ωy, ωz]^T
    Eigen::Matrix<double, 16, 1> y;
    y.segment<4>(0) = Eigen::Vector4d{ e_Quat_b__t1.w(), e_Quat_b__t1.x(), e_Quat_b__t1.y(), e_Quat_b__t1.z() };
    y.segment<3>(4) = e_velocity__t1;
    y.segment<3>(7) = e_position__t1;
    y.segment<3>(10) = b_f__t1;
    y.segment<3>(13) = b_omega_ip__t1;

    PosVelAttDerivativeConstants_e c;
    c.b_omega_ib_dot = (b_omega_ip__t0 - b_omega_ip__t1) / static_cast<double>(timeDifferenceSec);
    c.b_measuredForce_dot = (b_f__t0 - b_f__t1) / static_cast<double>(timeDifferenceSec);
    c.gravitationModel = _gravitationModel;
    c.coriolisAccelerationCompensationEnabled = _coriolisAccelerationCompensationEnabled;
    c.centrifgalAccelerationCompensationEnabled = _centrifgalAccelerationCompensationEnabled;
    c.angularRateEarthRotationCompensationEnabled = _angularRateEarthRotationCompensationEnabled;

    if (_integrationAlgorithm == IntegrationAlgorithm::Heun)
    {
        y = Heun(e_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta1)
    {
        y = RungeKutta1(e_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta2)
    {
        y = RungeKutta2(e_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta3)
    {
        y = RungeKutta3(e_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta4)
    {
        y = RungeKutta4(e_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }

    posVelAtt__t0->setState_e(y.segment<3>(7), y.segment<3>(4), Eigen::Quaterniond{ y(0), y(1), y(2), y(3) }.normalized());

    LOG_DATA("{}: posVelAtt__t0->e_position() = {}", nameId(), posVelAtt__t0->e_position().transpose());
    LOG_DATA("{}: posVelAtt__t0->e_velocity() = {}", nameId(), posVelAtt__t0->e_velocity().transpose());
    LOG_DATA("{}: posVelAtt__t0->e_Quat_b() = {}", nameId(), posVelAtt__t0->e_Quat_b());

    // Cycle lists
    _imuObservations.pop_back();
    _posVelAttStates.pop_back();
    _posVelAttStates.push_front(posVelAtt__t0);

    // Push out new data
    invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, posVelAtt__t0);
}

void NAV::ImuIntegrator::integrateObservationNED()
{
    // IMU Observation at the time tₖ
    const std::shared_ptr<const ImuObs>& imuObs__t0 = _imuObservations.at(0);
    // IMU Observation at the time tₖ₋₁
    const std::shared_ptr<const ImuObs>& imuObs__t1 = _imuObservations.at(1);

    // Result State Data at the time tₖ
    auto posVelAtt__t0 = std::make_shared<InertialNavSol>();

    posVelAtt__t0->imuObs = imuObs__t0;

    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0;

    if (!imuObs__t0->insTime.empty() && !(_prefereTimeSinceStartupOverInsTime && imuObs__t0->timeSinceStartup.has_value()))
    {
        // tₖ₋₁ Time at previous epoch
        const InsTime& time__t1 = imuObs__t1->insTime;
        // tₖ Current Time
        const InsTime& time__t0 = imuObs__t0->insTime;

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec = (time__t0 - time__t1).count();

        // Update time
        posVelAtt__t0->insTime = imuObs__t0->insTime;

        LOG_DATA("{}: time__t0 - time__t1 = {} - {} = {}", nameId(), time__t0.toYMDHMS(), time__t1.toYMDHMS(), timeDifferenceSec);
    }
    else
    {
        // tₖ₋₁ Time at previous epoch
        const auto& time__t1 = imuObs__t1->timeSinceStartup.value();
        // tₖ Current Time
        const auto& time__t0 = imuObs__t0->timeSinceStartup.value();

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec = static_cast<long double>(time__t0 - time__t1) * 1e-9L;

        if (_timeSinceStartup__init == 0)
        {
            _timeSinceStartup__init = imuObs__t0->timeSinceStartup.value();
            _time__init = !imuObs__t0->insTime.empty() ? imuObs__t0->insTime : InsTime(2000, 1, 1, 1, 1, 1);
        }

        // Update time
        posVelAtt__t0->insTime = _time__init + std::chrono::nanoseconds(imuObs__t0->timeSinceStartup.value() - _timeSinceStartup__init);

        LOG_DATA("{}: time__t0 - time__t1 = [{}] - [{}] = {}", nameId(), time__t0, time__t1, timeDifferenceSec);
    }
    auto dt = fmt::format("{:0.5f}", timeDifferenceSec);
    dt.erase(std::find_if(dt.rbegin(), dt.rend(), [](char ch) { return ch != '0'; }).base(), dt.end());
    LOG_DATA("{}: Integrating (dt = {}s) from [{}] to [{}] in NED frame", nameId(), dt,
             imuObs__t1->insTime.toYMDHMS(), imuObs__t0->insTime.toYMDHMS());

    // Position, Velocity and Attitude at the time tₖ₋₁
    const std::shared_ptr<const PosVelAtt>& posVelAtt__t1 = _posVelAttStates.at(0);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = imuObs__t0->imuPos;

    // ω_ip_p (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d p_omega_ip__t1 = !_prefereUncompensatedData && imuObs__t1->gyroCompXYZ.has_value()
                                         ? imuObs__t1->gyroCompXYZ.value()
                                         : imuObs__t1->gyroUncompXYZ.value();

    // ω_ip_p (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d p_omega_ip__t0 = !_prefereUncompensatedData && imuObs__t0->gyroCompXYZ.has_value()
                                         ? imuObs__t0->gyroCompXYZ.value()
                                         : imuObs__t0->gyroUncompXYZ.value();

    // a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d p_f_ip__t1 = !_prefereUncompensatedData && imuObs__t1->accelCompXYZ.has_value()
                                     ? imuObs__t1->accelCompXYZ.value()
                                     : imuObs__t1->accelUncompXYZ.value();

    // a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    Eigen::Vector3d p_f_ip__t0 = !_prefereUncompensatedData && imuObs__t0->accelCompXYZ.has_value()
                                     ? imuObs__t0->accelCompXYZ.value()
                                     : imuObs__t0->accelUncompXYZ.value();

    if (_lckfErrors)
    {
        LOG_DATA("{}: Applying IMU Biases p_quatGyro_b {}, p_quatAccel_b {}", nameId(), imuPosition.p_quatGyro_b(), imuPosition.p_quatAccel_b());
        p_omega_ip__t1 -= imuPosition.p_quatGyro_b() * _lckfErrors->b_biasGyro;
        p_omega_ip__t0 -= imuPosition.p_quatGyro_b() * _lckfErrors->b_biasGyro;
        p_f_ip__t1 -= imuPosition.p_quatAccel_b() * _lckfErrors->b_biasAccel;
        p_f_ip__t0 -= imuPosition.p_quatAccel_b() * _lckfErrors->b_biasAccel;
    }
    LOG_DATA("{}: p_omega_ip__t1 = {}", nameId(), p_omega_ip__t1.transpose());
    LOG_DATA("{}: p_omega_ip__t0 = {}", nameId(), p_omega_ip__t0.transpose());
    LOG_DATA("{}: p_f_ip__t1 = {}", nameId(), p_f_ip__t1.transpose());
    LOG_DATA("{}: p_f_ip__t0 = {}", nameId(), p_f_ip__t0.transpose());

    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond& n_Quat_b__t1 = posVelAtt__t1->n_Quat_b();
    LOG_DATA("{}: n_Quat_b__t1 = {}", nameId(), n_Quat_b__t1);

    // n_velocity (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& n_velocity__t1 = posVelAtt__t1->n_velocity();
    LOG_DATA("{}: n_velocity__t1 = {}", nameId(), n_velocity__t1.transpose());

    // [latitude 𝜙, longitude λ, altitude h] (tₖ₋₁) Position in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position__t1 = posVelAtt__t1->lla_position();
    LOG_DATA("{}: lla_position__t1 = {}", nameId(), lla_position__t1.transpose());

    // ω_ip_b (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ₋₁
    Eigen::Vector3d b_omega_ip__t1 = imuPosition.b_quatGyro_p() * p_omega_ip__t1;
    LOG_DATA("{}: b_omega_ip__t1 = {}", nameId(), b_omega_ip__t1.transpose());
    // ω_ip_b (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ
    Eigen::Vector3d b_omega_ip__t0 = imuPosition.b_quatGyro_p() * p_omega_ip__t0;
    LOG_DATA("{}: b_omega_ip__t0 = {}", nameId(), b_omega_ip__t0.transpose());

    // f_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
    Eigen::Vector3d b_f__t1 = imuPosition.b_quatAccel_p() * p_f_ip__t1;
    LOG_DATA("{}: b_f__t1 = {}", nameId(), b_f__t1.transpose());
    // f_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
    Eigen::Vector3d b_f__t0 = imuPosition.b_quatAccel_p() * p_f_ip__t0;
    LOG_DATA("{}: b_f__t0 = {}", nameId(), b_f__t0.transpose());

    //  0  1  2  3   4    5    6   7  8  9  10  11  12  13  14  15
    // [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h, fx, fy, fz, ωx, ωy, ωz]^T
    Eigen::Matrix<double, 16, 1> y;
    y.segment<4>(0) = Eigen::Vector4d{ n_Quat_b__t1.w(), n_Quat_b__t1.x(), n_Quat_b__t1.y(), n_Quat_b__t1.z() };
    y.segment<3>(4) = n_velocity__t1;
    y.segment<3>(7) = lla_position__t1;
    y.segment<3>(10) = b_f__t1;
    y.segment<3>(13) = b_omega_ip__t1;

    PosVelAttDerivativeConstants_n c;
    c.b_omega_ib_dot = (b_omega_ip__t0 - b_omega_ip__t1) / static_cast<double>(timeDifferenceSec);
    c.b_measuredForce_dot = (b_f__t0 - b_f__t1) / static_cast<double>(timeDifferenceSec);
    c.gravitationModel = _gravitationModel;
    c.coriolisAccelerationCompensationEnabled = _coriolisAccelerationCompensationEnabled;
    c.centrifgalAccelerationCompensationEnabled = _centrifgalAccelerationCompensationEnabled;
    c.angularRateEarthRotationCompensationEnabled = _angularRateEarthRotationCompensationEnabled;
    c.angularRateTransportRateCompensationEnabled = _angularRateTransportRateCompensationEnabled;

    if (_integrationAlgorithm == IntegrationAlgorithm::Heun)
    {
        y = Heun(n_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta1)
    {
        y = RungeKutta1(n_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta2)
    {
        y = RungeKutta2(n_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta3)
    {
        y = RungeKutta3(n_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }
    else if (_integrationAlgorithm == IntegrationAlgorithm::RungeKutta4)
    {
        y = RungeKutta4(n_calcPosVelAttDerivative, timeDifferenceSec, y, c);
    }

    posVelAtt__t0->setState_n(y.segment<3>(7), y.segment<3>(4), Eigen::Quaterniond{ y(0), y(1), y(2), y(3) }.normalized());

    LOG_DATA("{}: posVelAtt__t0->e_position() = {}", nameId(), posVelAtt__t0->e_position().transpose());
    LOG_DATA("{}: posVelAtt__t0->lla_position() - posVelAtt__t1->lla_position() = {} [m]", nameId(),
             calcGeographicalDistance(posVelAtt__t0->latitude(), posVelAtt__t0->longitude(), posVelAtt__t1->latitude(), posVelAtt__t1->longitude()));
    LOG_DATA("{}: posVelAtt__t0->n_velocity() = {}", nameId(), posVelAtt__t0->n_velocity().transpose());
    LOG_DATA("{}: posVelAtt__t0->n_Quat_b() = {}", nameId(), posVelAtt__t0->n_Quat_b());

    // Cycle lists
    _imuObservations.pop_back();
    _posVelAttStates.pop_back();
    _posVelAttStates.push_front(posVelAtt__t0);

    // Push out new data
    invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, posVelAtt__t0);
}