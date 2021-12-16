#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/Mechanization.hpp"
#include "Navigation/Math/Math.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include <imgui_internal.h>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/InertialNavSol.hpp"

#include <algorithm>

NAV::ImuIntegrator::ImuIntegrator()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 483, 350 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuIntegrator::recvImuObs);
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
    ImGui::SetNextItemWidth(250);
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
    if (ImGui::Combo(fmt::format("Integration Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&integrationFrame), "ECEF\0NED\0\0"))
    {
        LOG_DEBUG("{}: Integration Frame changed to {}", nameId(), integrationFrame == IntegrationFrame::NED ? "NED" : "ECEF");
        flow::ApplyChanges();
    }
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();

    ImGui::SetNextItemWidth(250);
    if (ImGui::BeginCombo(fmt::format("Integration Algorithm##{}", size_t(id)).c_str(), to_string(integrationAlgorithm)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IntegrationAlgorithm::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(integrationAlgorithm) == i);
            if (ImGui::Selectable(to_string(static_cast<IntegrationAlgorithm>(i)), is_selected))
            {
                integrationAlgorithm = static_cast<IntegrationAlgorithm>(i);
                LOG_DEBUG("{}: Integration Algorithm Attitude changed to {}", nameId(), integrationAlgorithm);
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
        if (ImGui::Checkbox(fmt::format("Prefere TimeSinceStartup over InsTime##{}", size_t(id)).c_str(), &prefereTimeSinceStartupOverInsTime))
        {
            LOG_DEBUG("{}: prefereTimeSinceStartupOverInsTime changed to {}", nameId(), prefereTimeSinceStartupOverInsTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Takes the IMU internal 'TimeSinceStartup' value instead of the absolute 'insTime'");

        if (ImGui::Checkbox(fmt::format("Prefere uncompensated data##{}", size_t(id)).c_str(), &prefereUncompensatedData))
        {
            LOG_DEBUG("{}: prefereUncompensatedData changed to {}", nameId(), prefereUncompensatedData);
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
            ImGui::SetNextItemWidth(230);
            if (ImGui::BeginCombo(fmt::format("Gravity Model##{}", size_t(id)).c_str(), NAV::to_string(gravityModel)))
            {
                for (size_t i = 0; i < static_cast<size_t>(GravityModel::COUNT); i++)
                {
                    const bool is_selected = (static_cast<size_t>(gravityModel) == i);
                    if (ImGui::Selectable(NAV::to_string(static_cast<GravityModel>(i)), is_selected))
                    {
                        gravityModel = static_cast<GravityModel>(i);
                        LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), NAV::to_string(gravityModel));
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
            if (ImGui::Checkbox(fmt::format("Coriolis acceleration ##{}", size_t(id)).c_str(), &coriolisAccelerationCompensationEnabled))
            {
                LOG_DEBUG("{}: coriolisAccelerationCompensationEnabled changed to {}", nameId(), coriolisAccelerationCompensationEnabled);
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Centrifugal acceleration##{}", size_t(id)).c_str(), &centrifgalAccelerationCompensationEnabled))
            {
                LOG_DEBUG("{}: centrifgalAccelerationCompensationEnabled changed to {}", nameId(), centrifgalAccelerationCompensationEnabled);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TextUnformatted("Angular rate compensation");
        {
            ImGui::Indent();
            if (ImGui::Checkbox(fmt::format("Earth rotation rate##{}", size_t(id)).c_str(), &angularRateEarthRotationCompensationEnabled))
            {
                LOG_DEBUG("{}: angularRateEarthRotationCompensationEnabled changed to {}", nameId(), angularRateEarthRotationCompensationEnabled);
                flow::ApplyChanges();
            }
            if (ImGui::Checkbox(fmt::format("Transport rate##{}", size_t(id)).c_str(), &angularRateTransportRateCompensationEnabled))
            {
                LOG_DEBUG("{}: angularRateTransportRateCompensationEnabled changed to {}", nameId(), angularRateTransportRateCompensationEnabled);
                flow::ApplyChanges();
            }
            ImGui::Unindent();
        }
        ImGui::TreePop();
    }

    ImGui::Separator();
    if (ImGui::Checkbox(fmt::format("Show Corrections input pins##{}", size_t(id)).c_str(), &showCorrectionsInputPin))
    {
        LOG_DEBUG("{}: showCorrectionsInputPin changed to {}", nameId(), showCorrectionsInputPin);

        if (showCorrectionsInputPin && inputPins.size() < 4)
        {
            nm::CreateInputPin(this, "PVAError", Pin::Type::Flow, { PVAError::type() }, &ImuIntegrator::recvPVAError);
            nm::CreateInputPin(this, "ImuBiases", Pin::Type::Flow, { ImuBiases::type() }, &ImuIntegrator::recvImuBiases);
        }
        else if (!showCorrectionsInputPin)
        {
            while (inputPins.size() >= 3)
            {
                if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.back().id))
                {
                    nm::DeleteLink(connectedLink->id);
                }
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

    j["integrationFrame"] = integrationFrame;
    j["integrationAlgorithm"] = integrationAlgorithm;
    // #########################################################################################################################################
    j["prefereTimeSinceStartupOverInsTime"] = prefereTimeSinceStartupOverInsTime;
    j["prefereUncompensatedData"] = prefereUncompensatedData;
    // #########################################################################################################################################
    j["gravityModel"] = gravityModel;
    j["coriolisAccelerationCompensationEnabled"] = coriolisAccelerationCompensationEnabled;
    j["centrifgalAccelerationCompensationEnabled"] = centrifgalAccelerationCompensationEnabled;
    j["angularRateEarthRotationCompensationEnabled"] = angularRateEarthRotationCompensationEnabled;
    j["angularRateTransportRateCompensationEnabled"] = angularRateTransportRateCompensationEnabled;
    // #########################################################################################################################################
    j["showCorrectionsInputPin"] = showCorrectionsInputPin;

    return j;
}

void NAV::ImuIntegrator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("integrationFrame"))
    {
        j.at("integrationFrame").get_to(integrationFrame);
    }
    if (j.contains("integrationAlgorithm"))
    {
        j.at("integrationAlgorithm").get_to(integrationAlgorithm);
    }
    // #########################################################################################################################################
    if (j.contains("prefereTimeSinceStartupOverInsTime"))
    {
        prefereTimeSinceStartupOverInsTime = j.at("prefereTimeSinceStartupOverInsTime");
    }
    if (j.contains("prefereUncompensatedData"))
    {
        prefereUncompensatedData = j.at("prefereUncompensatedData");
    }
    // #########################################################################################################################################
    if (j.contains("gravityModel"))
    {
        j.at("gravityModel").get_to(gravityModel);
    }
    if (j.contains("coriolisAccelerationCompensationEnabled"))
    {
        coriolisAccelerationCompensationEnabled = j.at("coriolisAccelerationCompensationEnabled");
    }
    if (j.contains("centrifgalAccelerationCompensationEnabled"))
    {
        centrifgalAccelerationCompensationEnabled = j.at("centrifgalAccelerationCompensationEnabled");
    }
    if (j.contains("angularRateEarthRotationCompensationEnabled"))
    {
        angularRateEarthRotationCompensationEnabled = j.at("angularRateEarthRotationCompensationEnabled");
    }
    if (j.contains("angularRateTransportRateCompensationEnabled"))
    {
        angularRateTransportRateCompensationEnabled = j.at("angularRateTransportRateCompensationEnabled");
    }
    // #########################################################################################################################################
    if (j.contains("showCorrectionsInputPin"))
    {
        showCorrectionsInputPin = j.at("showCorrectionsInputPin");
        if (showCorrectionsInputPin && inputPins.size() < 4)
        {
            nm::CreateInputPin(this, "PVAError", Pin::Type::Flow, { PVAError::type() }, &ImuIntegrator::recvPVAError);
            nm::CreateInputPin(this, "ImuBiases", Pin::Type::Flow, { ImuBiases::type() }, &ImuIntegrator::recvImuBiases);
        }
        else if (!showCorrectionsInputPin)
        {
            while (inputPins.size() >= 3)
            {
                if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.back().id))
                {
                    nm::DeleteLink(connectedLink->id);
                }
                inputPins.pop_back();
            }
        }
    }
}

bool NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // This should be dependant on the integration algorithm
    switch (integrationAlgorithm)
    {
    case IntegrationAlgorithm::Heun:
    case IntegrationAlgorithm::RungeKutta1:
    case IntegrationAlgorithm::RungeKutta2:
    case IntegrationAlgorithm::RungeKutta3:
    case IntegrationAlgorithm::RungeKutta4:
        maxSizeImuObservations = 2; // Has to be >= 2
        maxSizeStates = 1;
        break;
    case IntegrationAlgorithm::COUNT:
        return false;
    }

    imuObservations.clear();
    posVelAttStates.clear();
    imuBiases.reset();

    time__init = InsTime();
    timeSinceStartup__init = 0;

    LOG_DEBUG("ImuIntegrator initialized");

    return true;
}

void NAV::ImuIntegrator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::ImuIntegrator::recvPosVelAttInit(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(nodeData);

    // Fill the list with the initial state to the start of the list
    if (posVelAttStates.empty())
    {
        while (posVelAttStates.size() < maxSizeStates)
        {
            LOG_DEBUG("{}: Adding posVelAtt to the start of the list {}", nameId(), posVelAtt);
            posVelAttStates.push_front(posVelAtt);
        }

        if (imuObservations.size() >= maxSizeImuObservations - 1)
        {
            // Push out a message with the initial state and a matching imu Observation
            auto inertialNavSol = std::make_shared<InertialNavSol>();

            inertialNavSol->setState_n(posVelAtt->latLonAlt(), posVelAtt->velocity_n(), posVelAtt->quaternion_nb());

            size_t imuObsIndex = std::min(1UL, imuObservations.size() - 1);

            inertialNavSol->insTime = imuObservations.at(imuObsIndex)->insTime;
            inertialNavSol->imuObs = imuObservations.at(imuObsIndex);

            invokeCallbacks(OutputPortIndex_InertialNavSol, inertialNavSol);
        }

        // If enough imu observations received, integrate the observation
        if (imuObservations.size() == maxSizeImuObservations)
        {
            integrateObservation();
        }
    }
}

void NAV::ImuIntegrator::recvImuObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto imuObs = std::static_pointer_cast<const ImuObs>(nodeData);

    if (!imuObs->insTime.has_value() && !imuObs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime/timeSinceStartup)", nameId());
        return;
    }

    // Add imuObs tₖ to the start of the list
    imuObservations.push_front(imuObs);

    // Remove observations at the end of the list till the max size is reached
    while (imuObservations.size() > maxSizeImuObservations)
    {
        if (!posVelAttStates.empty())
        {
            LOG_WARN("{}: Receive new Imu observation, but list is full --> discarding oldest observation", nameId());
        }
        imuObservations.pop_back();
    }

    // First ImuObs and already has state
    if (imuObservations.size() == 1 && posVelAttStates.size() == maxSizeStates)
    {
        // Push out a message with the initial state and a matching imu Observation
        auto inertialNavSol = std::make_shared<InertialNavSol>();

        inertialNavSol->setState_n(posVelAttStates.front()->latLonAlt(),
                                   posVelAttStates.front()->velocity_n(),
                                   posVelAttStates.front()->quaternion_nb());

        inertialNavSol->insTime = imuObs->insTime;
        inertialNavSol->imuObs = imuObs;

        invokeCallbacks(OutputPortIndex_InertialNavSol, inertialNavSol);
        return;
    }

    // If enough imu observations and states received, integrate the observation
    if (imuObservations.size() == maxSizeImuObservations
        && posVelAttStates.size() == maxSizeStates)
    {
        integrateObservation();
    }
}

void NAV::ImuIntegrator::recvPVAError(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    pvaError = std::static_pointer_cast<const PVAError>(nodeData);
}

void NAV::ImuIntegrator::recvImuBiases(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    imuBiases = std::static_pointer_cast<const ImuBiases>(nodeData);
}

std::shared_ptr<const NAV::PosVelAtt> NAV::ImuIntegrator::correctPosVelAtt(const std::shared_ptr<const NAV::PosVelAtt>& posVelAtt, const std::shared_ptr<const NAV::PVAError>& pvaError)
{
    auto posVelAttCorrected = std::make_shared<PosVelAtt>(*posVelAtt);
    posVelAttCorrected->setPosition_lla(posVelAtt->latLonAlt() - pvaError->positionError_lla());

    posVelAttCorrected->setVelocity_n(posVelAtt->velocity_n() - pvaError->velocityError_n());

    // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.15
    Eigen::Vector3d attError = pvaError->attitudeError_n();
    Eigen::Matrix3d dcm_c = (Eigen::Matrix3d::Identity() + skewSymmetricMatrix(attError)) * posVelAtt->quaternion_nb().toRotationMatrix();
    posVelAttCorrected->setAttitude_nb(Eigen::Quaterniond(dcm_c).normalized());

    // Attitude correction, see Titterton and Weston (2004), p. 407 eq. 13.16
    // const Eigen::Quaterniond& q_nb = posVelAtt->quaternion_nb()
    //                                  * (Eigen::AngleAxisd(attError(0), Eigen::Vector3d::UnitX())
    //                                     * Eigen::AngleAxisd(attError(1), Eigen::Vector3d::UnitY())
    //                                     * Eigen::AngleAxisd(attError(2), Eigen::Vector3d::UnitZ()))
    //                                        .normalized();
    // posVelAttCorrected->setAttitude_nb(q_nb.normalized());

    // Eigen::Vector3d attError = pvaError->attitudeError_n();
    // const Eigen::Quaterniond& q_nb = posVelAtt->quaternion_nb();
    // Eigen::Quaterniond q_nb_c{ q_nb.w() + 0.5 * (+attError(0) * q_nb.x() + attError(1) * q_nb.y() + attError(2) * q_nb.z()),
    //                            q_nb.x() + 0.5 * (-attError(0) * q_nb.w() + attError(1) * q_nb.z() - attError(2) * q_nb.y()),
    //                            q_nb.y() + 0.5 * (-attError(0) * q_nb.z() - attError(1) * q_nb.w() + attError(2) * q_nb.x()),
    //                            q_nb.z() + 0.5 * (+attError(0) * q_nb.y() - attError(1) * q_nb.x() - attError(2) * q_nb.w()) };
    // posVelAttCorrected->setAttitude_nb(q_nb_c.normalized());

    return posVelAttCorrected;
}

void NAV::ImuIntegrator::integrateObservation()
{
    if (pvaError)
    {
        LOG_DATA("{}: Applying corrections to {} and {}", nameId(), posVelAttStates.at(0)->insTime.value().toYMDHMS(), posVelAttStates.at(1)->insTime.value().toYMDHMS());

        for (auto& posVelAtt : posVelAttStates)
        {
            posVelAtt = correctPosVelAtt(posVelAtt, pvaError);
        }
        LOG_DATA("{}: posVelAttStates.at(0) = {}, posVelAttStates.at(1) = {} (after KF update)", nameId(), posVelAttStates.at(0), posVelAttStates.at(1));

        pvaError.reset();
    }

    // IMU Observation at the time tₖ
    const std::shared_ptr<const ImuObs>& imuObs__t0 = imuObservations.at(0);
    // IMU Observation at the time tₖ₋₁
    const std::shared_ptr<const ImuObs>& imuObs__t1 = imuObservations.at(1);

    // Position, Velocity and Attitude at the time tₖ₋₁
    const std::shared_ptr<const PosVelAtt>& posVelAtt__t1 = posVelAttStates.at(0);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = imuObs__t0->imuPos;

    // Result State Data at the time tₖ
    auto posVelAtt__t0 = std::make_shared<InertialNavSol>();

    posVelAtt__t0->imuObs = imuObs__t0;

    // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    [[maybe_unused]] long double timeDifferenceSec__t1 = 0; // TODO: Use in Heun algorithm
    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec = 0;

    if (imuObs__t0->insTime.has_value() && !(prefereTimeSinceStartupOverInsTime && imuObs__t0->timeSinceStartup.has_value()))
    {
        // tₖ₋₁ Time at previous epoch
        const InsTime& time__t1 = imuObs__t1->insTime.value();
        // tₖ Current Time
        const InsTime& time__t0 = imuObs__t0->insTime.value();

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec = (time__t0 - time__t1).count();

        // Update time
        posVelAtt__t0->insTime = imuObs__t0->insTime;

        LOG_DATA("time__t0 - time__t1 = {} - {} = {}", nameId(), time__t0.toYMDHMS(), time__t1.toYMDHMS(), timeDifferenceSec);
    }
    else
    {
        // tₖ₋₁ Time at previous epoch
        const auto& time__t1 = imuObs__t1->timeSinceStartup.value();
        // tₖ Current Time
        const auto& time__t0 = imuObs__t0->timeSinceStartup.value();

        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec = static_cast<long double>(time__t0 - time__t1) * 1e-9L;

        if (timeSinceStartup__init == 0)
        {
            timeSinceStartup__init = imuObs__t0->timeSinceStartup.value();
            time__init = imuObs__t0->insTime.has_value() ? imuObs__t0->insTime.value() : InsTime(2000, 1, 1, 1, 1, 1);
        }

        // Update time
        posVelAtt__t0->insTime = time__init + std::chrono::nanoseconds(imuObs__t0->timeSinceStartup.value() - timeSinceStartup__init);

        LOG_DATA("time__t0 - time__t1 = {} - {} = {}", nameId(), time__t0, time__t1, timeDifferenceSec);
    }

    // ω_ip_p (tₖ₋₁) Angular velocity in [rad/s],
    // of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d angularVelocity_ip_p__t1 = !prefereUncompensatedData && imuObs__t1->gyroCompXYZ.has_value()
                                                   ? imuObs__t1->gyroCompXYZ.value()
                                                   : imuObs__t1->gyroUncompXYZ.value();

    // ω_ip_p (tₖ) Angular velocity in [rad/s],
    // of the inertial to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d angularVelocity_ip_p__t0 = !prefereUncompensatedData && imuObs__t0->gyroCompXYZ.has_value()
                                                   ? imuObs__t0->gyroCompXYZ.value()
                                                   : imuObs__t0->gyroUncompXYZ.value();

    // a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d acceleration_p__t1 = !prefereUncompensatedData && imuObs__t1->accelCompXYZ.has_value()
                                             ? imuObs__t1->accelCompXYZ.value()
                                             : imuObs__t1->accelUncompXYZ.value();

    // a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    Eigen::Vector3d acceleration_p__t0 = !prefereUncompensatedData && imuObs__t0->accelCompXYZ.has_value()
                                             ? imuObs__t0->accelCompXYZ.value()
                                             : imuObs__t0->accelUncompXYZ.value();

    if (imuBiases)
    {
        angularVelocity_ip_p__t1 -= imuPosition.quatGyro_pb() * imuBiases->biasGyro_b;
        angularVelocity_ip_p__t0 -= imuPosition.quatGyro_pb() * imuBiases->biasGyro_b;
        acceleration_p__t1 -= imuPosition.quatAccel_pb() * imuBiases->biasAccel_b;
        acceleration_p__t0 -= imuPosition.quatAccel_pb() * imuBiases->biasAccel_b;
    }
    LOG_DATA("{}: angularVelocity_ip_p__t1 = {}", nameId(), angularVelocity_ip_p__t1.transpose());
    LOG_DATA("{}: angularVelocity_ip_p__t0 = {}", nameId(), angularVelocity_ip_p__t0.transpose());
    LOG_DATA("{}: acceleration_p__t1 = {}", nameId(), acceleration_p__t1.transpose());
    LOG_DATA("{}: acceleration_p__t0 = {}", nameId(), acceleration_p__t0.transpose());

    // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
    const Eigen::Quaterniond quaternion_nb__t1 = posVelAtt__t1->quaternion_nb();
    LOG_DATA("{}: quaternion_nb__t1 = {}", nameId(), quaternion_nb__t1.coeffs().transpose());

    // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& velocity_n__t1 = posVelAtt__t1->velocity_n();
    LOG_DATA("{}: velocity_n__t1 = {}", nameId(), velocity_n__t1.transpose());

    // [latitude 𝜙, longitude λ, altitude h] (tₖ₋₁) Position in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d position_lla__t1 = posVelAtt__t1->latLonAlt();
    LOG_DATA("{}: position_lla__t1 = {}", nameId(), position_lla__t1.transpose());

    // ω_ip_b (tₖ₋₁) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d omega_ip_b__t1 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t1;
    LOG_DATA("{}: omega_ip_b__t1 = {}", nameId(), omega_ip_b__t1.transpose());
    // ω_ip_b (tₖ) Angular velocity in [rad/s], of the inertial to platform system, in body coordinates, at the time tₖ
    const Eigen::Vector3d omega_ip_b__t0 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t0;
    LOG_DATA("{}: omega_ip_b__t0 = {}", nameId(), omega_ip_b__t0.transpose());

    // f_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
    const Eigen::Vector3d f_b__t1 = imuPosition.quatAccel_bp() * acceleration_p__t1;
    LOG_DATA("{}: f_b__t1 = {}", nameId(), f_b__t1.transpose());
    // f_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
    const Eigen::Vector3d f_b__t0 = imuPosition.quatAccel_bp() * acceleration_p__t0;
    LOG_DATA("{}: f_b__t0 = {}", nameId(), f_b__t0.transpose());

    //  0  1  2  3   4    5    6   7  8  9
    // [w, x, y, z, v_N, v_E, v_D, 𝜙, λ, h]^T
    Eigen::Matrix<double, 10, 1> y;
    y.segment<4>(0) = Eigen::Vector4d{ quaternion_nb__t1.w(), quaternion_nb__t1.x(), quaternion_nb__t1.y(), quaternion_nb__t1.z() };
    y.segment<3>(4) = velocity_n__t1;
    y.segment<3>(7) = position_lla__t1;

    PosVelAttDerivativeConstants_n c;
    c.omega_ib_b = omega_ip_b__t1; // platform system does not rotate with respect to body system
    c.f_b = f_b__t1;
    c.gravityModel = gravityModel;
    c.coriolisAccelerationCompensationEnabled = coriolisAccelerationCompensationEnabled;
    c.centrifgalAccelerationCompensationEnabled = centrifgalAccelerationCompensationEnabled;
    c.angularRateEarthRotationCompensationEnabled = angularRateEarthRotationCompensationEnabled;
    c.angularRateTransportRateCompensationEnabled = angularRateTransportRateCompensationEnabled;

    if (integrationAlgorithm == IntegrationAlgorithm::Heun)
    {
        // Values needed to calculate the PosVelAttDerivative for the local-navigation frame
        struct AccelGyroMeasurement
        {
            Eigen::Vector3d omega_ib_b; // ω_ip_b Angular velocity in [rad/s], of the inertial to platform system, in body coordinates
            Eigen::Vector3d f_b;        // f_b Acceleration in [m/s^2], in body coordinates
        };
        auto posVelAttDerivativeWrapper = [](const Eigen::Matrix<double, 10, 1>& y, const AccelGyroMeasurement& z, const PosVelAttDerivativeConstants_n& c) {
            PosVelAttDerivativeConstants_n c_new;
            c_new.omega_ib_b = z.omega_ib_b;
            c_new.f_b = z.f_b;
            c_new.gravityModel = c.gravityModel;
            c_new.coriolisAccelerationCompensationEnabled = c.coriolisAccelerationCompensationEnabled;
            c_new.centrifgalAccelerationCompensationEnabled = c.centrifgalAccelerationCompensationEnabled;
            c_new.angularRateEarthRotationCompensationEnabled = c.angularRateEarthRotationCompensationEnabled;
            c_new.angularRateTransportRateCompensationEnabled = c.angularRateTransportRateCompensationEnabled;

            return calcPosVelAttDerivative_n(y, c_new);
        };
        AccelGyroMeasurement z__t0{ omega_ip_b__t0, f_b__t0 };
        AccelGyroMeasurement z__t1{ omega_ip_b__t1, f_b__t1 };

        y = Heun(posVelAttDerivativeWrapper, timeDifferenceSec, y, z__t1, z__t0, c);
    }
    else if (integrationAlgorithm == IntegrationAlgorithm::RungeKutta1)
    {
        y = RungeKutta1(calcPosVelAttDerivative_n, timeDifferenceSec, y, c);
    }
    else if (integrationAlgorithm == IntegrationAlgorithm::RungeKutta2)
    {
        y = RungeKutta2(calcPosVelAttDerivative_n, timeDifferenceSec, y, c);
    }
    else if (integrationAlgorithm == IntegrationAlgorithm::RungeKutta3)
    {
        y = RungeKutta3(calcPosVelAttDerivative_n, timeDifferenceSec, y, c);
    }
    else if (integrationAlgorithm == IntegrationAlgorithm::RungeKutta4)
    {
        y = RungeKutta4(calcPosVelAttDerivative_n, timeDifferenceSec, y, c);
    }

    posVelAtt__t0->setState_n(y.segment<3>(7), y.segment<3>(4), Eigen::Quaterniond{ y(0), y(1), y(2), y(3) });

    LOG_DATA("{}: posVelAtt__t0->position_ecef() = {}", nameId(), posVelAtt__t0->position_ecef().transpose());
    LOG_DATA("{}: posVelAtt__t0->velocity_n() = {}", nameId(), posVelAtt__t0->velocity_n().transpose());
    LOG_DATA("{}: posVelAtt__t0->quaternion_nb() = {}", nameId(), posVelAtt__t0->quaternion_nb().coeffs().transpose());

    // Cycle lists
    imuObservations.pop_back();
    posVelAttStates.pop_back();
    posVelAttStates.push_front(posVelAtt__t0);

    // Push out new data
    invokeCallbacks(OutputPortIndex_InertialNavSol, posVelAtt__t0);
}