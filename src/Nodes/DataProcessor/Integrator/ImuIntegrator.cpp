#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"
#include "util/InsGravity.hpp"
#include "util/InsMath.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include <imgui_internal.h>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/InertialNavSol.hpp"

NAV::ImuIntegrator::ImuIntegrator()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 483, 350 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuIntegrator::recvImuObs__t0);
    nm::CreateInputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &ImuIntegrator::recvState__t1);

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
    if (ImGui::Combo(fmt::format("Integration Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&integrationFrame), "ECEF\0NED\0\0"))
    {
        LOG_DEBUG("{}: Integration Frame changed to {}", nameId(), integrationFrame == IntegrationFrame::NED ? "NED" : "ECEF");
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(250);
    if (ImGui::Combo(fmt::format("Gravity Model##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&gravityModel), "WGS84\0WGS84_Skydel\0Somigliana\0EGM96\0OFF\0\0"))
    {
        if (gravityModel == GravityModel::WGS84)
        {
            LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), "WGS84");
        }
        else if (gravityModel == GravityModel::WGS84_Skydel)
        {
            LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), "WGS84_Skydel");
        }
        else if (gravityModel == GravityModel::Somigliana)
        {
            LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), "Somigliana");
        }
        else if (gravityModel == GravityModel::EGM96)
        {
            LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), "EGM96");
        }
        else if (gravityModel == GravityModel::OFF)
        {
            LOG_DEBUG("{}: Gravity Model changed to {}", nameId(), "OFF");
        }
        flow::ApplyChanges();
    }

    ImGui::SetNextItemWidth(250);
    if (ImGui::BeginCombo(fmt::format("Integration Algorithm Attitude##{}", size_t(id)).c_str(), to_string(integrationAlgorithmAttitude)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IntegrationAlgorithm::COUNT); i++)
        {
            if (i != static_cast<size_t>(IntegrationAlgorithm::RungeKutta1) && i != static_cast<size_t>(IntegrationAlgorithm::RungeKutta3))
            {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
            }

            const bool is_selected = (static_cast<size_t>(integrationAlgorithmAttitude) == i);
            if (ImGui::Selectable(to_string(static_cast<IntegrationAlgorithm>(i)), is_selected))
            {
                integrationAlgorithmAttitude = static_cast<IntegrationAlgorithm>(i);
                LOG_DEBUG("{}: Integration Algorithm Attitude changed to {}", nameId(), integrationAlgorithmAttitude);
                flow::ApplyChanges();
            }

            if (i != static_cast<size_t>(IntegrationAlgorithm::RungeKutta1) && i != static_cast<size_t>(IntegrationAlgorithm::RungeKutta3))
            {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }

    ImGui::SetNextItemWidth(250);
    if (ImGui::BeginCombo(fmt::format("Integration Algorithm Velocity##{}", size_t(id)).c_str(), to_string(integrationAlgorithmVelocity)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IntegrationAlgorithm::COUNT); i++)
        {
            if (i == static_cast<size_t>(IntegrationAlgorithm::RectangularRule))
            {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
            }

            const bool is_selected = (static_cast<size_t>(integrationAlgorithmVelocity) == i);
            if (ImGui::Selectable(to_string(static_cast<IntegrationAlgorithm>(i)), is_selected))
            {
                integrationAlgorithmVelocity = static_cast<IntegrationAlgorithm>(i);
                LOG_DEBUG("{}: Integration Algorithm Velocity changed to {}", nameId(), integrationAlgorithmVelocity);
                flow::ApplyChanges();
            }

            if (i == static_cast<size_t>(IntegrationAlgorithm::RectangularRule))
            {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }

    ImGui::SetNextItemWidth(250);
    if (ImGui::BeginCombo(fmt::format("Integration Algorithm Position##{}", size_t(id)).c_str(), to_string(integrationAlgorithmPosition)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IntegrationAlgorithm::COUNT); i++)
        {
            if (i != static_cast<size_t>(IntegrationAlgorithm::RectangularRule))
            {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
            }

            const bool is_selected = (static_cast<size_t>(integrationAlgorithmPosition) == i);
            if (ImGui::Selectable(to_string(static_cast<IntegrationAlgorithm>(i)), is_selected))
            {
                integrationAlgorithmPosition = static_cast<IntegrationAlgorithm>(i);
                LOG_DEBUG("{}: Integration Algorithm Position changed to {}", nameId(), integrationAlgorithmPosition);
                flow::ApplyChanges();
            }

            if (i != static_cast<size_t>(IntegrationAlgorithm::RectangularRule))
            {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }

    if (ImGui::Checkbox(fmt::format("Calculate intermediate values##{}", size_t(id)).c_str(), &calculateIntermediateValues))
    {
        LOG_DEBUG("{}: calculateIntermediateValues changed to {}", nameId(), calculateIntermediateValues);
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Runge Kutta uses intermediate observations but propagates only every other state. Because of this 2 separate state solutions can coexist."
                             "To avoid this only every seconds state can be output resulting in halving the output frequency. The accuracy of the results is not affected by this.");

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

    if (ImGui::Checkbox(fmt::format("Apply centrifugal acceleration compensation##{}", size_t(id)).c_str(), &centrifugalAccCompensation))
    {
        LOG_DEBUG("{}: centrifugalAccCompensation changed to {}", nameId(), centrifugalAccCompensation);
        flow::ApplyChanges();
    }

    if (ImGui::Checkbox(fmt::format("Apply coriolis acceleration compensation##{}", size_t(id)).c_str(), &coriolisCompensation))
    {
        LOG_DEBUG("{}: coriolisCompensation changed to {}", nameId(), coriolisCompensation);
        flow::ApplyChanges();
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
    j["gravityModel"] = gravityModel;
    j["integrationAlgorithmAttitude"] = integrationAlgorithmAttitude;
    j["integrationAlgorithmVelocity"] = integrationAlgorithmVelocity;
    j["integrationAlgorithmPosition"] = integrationAlgorithmPosition;
    j["calculateIntermediateValues"] = calculateIntermediateValues;
    j["prefereTimeSinceStartupOverInsTime"] = prefereTimeSinceStartupOverInsTime;
    j["centrifugalAccCompensation"] = centrifugalAccCompensation;
    j["coriolisCompensation"] = coriolisCompensation;
    j["showCorrectionsInputPin"] = showCorrectionsInputPin;
    j["prefereUncompensatedData"] = prefereUncompensatedData;

    return j;
}

void NAV::ImuIntegrator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("integrationFrame"))
    {
        integrationFrame = static_cast<IntegrationFrame>(j.at("integrationFrame").get<int>());
    }
    if (j.contains("gravityModel"))
    {
        gravityModel = static_cast<GravityModel>(j.at("gravityModel").get<int>());
    }
    if (j.contains("integrationAlgorithmAttitude"))
    {
        integrationAlgorithmAttitude = static_cast<IntegrationAlgorithm>(j.at("integrationAlgorithmAttitude").get<int>());
    }
    if (j.contains("integrationAlgorithmVelocity"))
    {
        integrationAlgorithmVelocity = static_cast<IntegrationAlgorithm>(j.at("integrationAlgorithmVelocity").get<int>());
    }
    if (j.contains("integrationAlgorithmPosition"))
    {
        integrationAlgorithmPosition = static_cast<IntegrationAlgorithm>(j.at("integrationAlgorithmPosition").get<int>());
    }
    if (j.contains("calculateIntermediateValues"))
    {
        calculateIntermediateValues = j.at("calculateIntermediateValues");
    }
    if (j.contains("prefereTimeSinceStartupOverInsTime"))
    {
        prefereTimeSinceStartupOverInsTime = j.at("prefereTimeSinceStartupOverInsTime");
    }
    if (j.contains("centrifugalAccCompensation"))
    {
        centrifugalAccCompensation = j.at("centrifugalAccCompensation");
    }
    if (j.contains("coriolisCompensation"))
    {
        coriolisCompensation = j.at("coriolisCompensation");
    }
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
    if (j.contains("prefereUncompensatedData"))
    {
        prefereUncompensatedData = j.at("prefereUncompensatedData");
    }
}

bool NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // This should be dependant on the integration algorithm
    maxSizeImuObservations = 3;
    maxSizeStates = 2;

    imuObservations.clear();
    posVelAttStates.clear();

    skipIntermediateCalculation = false;

    time__init = InsTime();
    timeSinceStartup__init = 0;

    imuBiases.biasAccel_p.setZero();
    imuBiases.biasGyro_p.setZero();

    LOG_DEBUG("ImuIntegrator initialized");

    return true;
}

void NAV::ImuIntegrator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::ImuIntegrator::recvImuObs__t0(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto imuObs = std::dynamic_pointer_cast<const ImuObs>(nodeData);

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

    // If enough imu observations and states received, integrate the observation
    if (imuObservations.size() == maxSizeImuObservations
        && posVelAttStates.size() == maxSizeStates)
    {
        integrateObservation();
    }
}

void NAV::ImuIntegrator::recvState__t1(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto posVelAtt = std::dynamic_pointer_cast<const PosVelAtt>(nodeData);

    // Add imuObs tₖ₋₁ to the start of the list
    if (posVelAttStates.empty())
    {
        while (posVelAttStates.size() < maxSizeStates)
        {
            LOG_DEBUG("{}: Adding posVelAtt to the start of the list {}", nameId(), posVelAtt);
            posVelAttStates.push_front(posVelAtt);
        }
    }
    else
    {
        posVelAttStates.push_front(posVelAtt);
    }

    // Remove states at the end of the list till the max size is reached
    while (posVelAttStates.size() > maxSizeStates)
    {
        LOG_WARN("{}: Receive new state, but list is full --> discarding oldest state", nameId());
        posVelAttStates.pop_back();
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
    pvaError = std::dynamic_pointer_cast<const PVAError>(nodeData);
}

void NAV::ImuIntegrator::recvImuBiases(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto imuBiasObs = std::dynamic_pointer_cast<const ImuBiases>(nodeData);

    imuBiases.biasAccel_p += imuBiasObs->biasAccel_p;
    imuBiases.biasGyro_p += imuBiasObs->biasGyro_p;
}

void NAV::ImuIntegrator::integrateObservation()
{
    LOG_DATA("{}: imuObservations.at(0) = {}, imuObservations.at(1) = {}, imuObservations.at(2) = {}", nameId(), imuObservations.at(0), imuObservations.at(1), imuObservations.at(2));
    LOG_DATA("{}: posVelAttStates.at(0) = {}, posVelAttStates.at(1) = {}", nameId(), posVelAttStates.at(0), posVelAttStates.at(1));
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

    /// IMU Observation at the time tₖ
    const std::shared_ptr<const ImuObs>& imuObs__t0 = imuObservations.at(0);
    /// IMU Observation at the time tₖ₋₁
    const std::shared_ptr<const ImuObs>& imuObs__t1 = imuObservations.at(1);
    /// IMU Observation at the time tₖ₋₂
    const std::shared_ptr<const ImuObs>& imuObs__t2 = imuObservations.at(2);

    /// Position, Velocity and Attitude at the time tₖ₋₁
    const std::shared_ptr<const PosVelAtt>& posVelAtt__t1 = posVelAttStates.at(0);
    /// Position, Velocity and Attitude at the time tₖ₋₂
    const std::shared_ptr<const PosVelAtt>& posVelAtt__t2 = posVelAttStates.at(1);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = imuObs__t0->imuPos;

    /// Result State Data at the time tₖ
    auto posVelAtt__t0 = std::make_shared<InertialNavSol>();

    posVelAtt__t0->imuObs = imuObs__t0;

    // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    long double timeDifferenceSec__t1 = 0;
    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    long double timeDifferenceSec__t0 = 0;

    if (imuObs__t0->insTime.has_value() && !(prefereTimeSinceStartupOverInsTime && imuObs__t0->timeSinceStartup.has_value()))
    {
        /// tₖ₋₂ Time at prior to previous epoch
        const InsTime& time__t2 = imuObs__t2->insTime.value();
        /// tₖ₋₁ Time at previous epoch
        const InsTime& time__t1 = imuObs__t1->insTime.value();
        /// tₖ Current Time
        const InsTime& time__t0 = imuObs__t0->insTime.value();

        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
        timeDifferenceSec__t1 = (time__t1 - time__t2).count();
        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec__t0 = (time__t0 - time__t1).count();

        // Update time
        posVelAtt__t0->insTime = imuObs__t0->insTime;

        LOG_DATA("{}: time__t2 {}", nameId(), time__t2.toYMDHMS());
        LOG_DATA("{}: time__t1 {}: DiffSec__t1 {}", nameId(), time__t1.toYMDHMS(), timeDifferenceSec__t1);
        LOG_DATA("{}: time__t0 {}: DiffSec__t0 {}", nameId(), time__t0.toYMDHMS(), timeDifferenceSec__t0);
    }
    else
    {
        /// tₖ₋₂ Time at prior to previous epoch
        const auto& time__t2 = imuObs__t2->timeSinceStartup.value();
        /// tₖ₋₁ Time at previous epoch
        const auto& time__t1 = imuObs__t1->timeSinceStartup.value();
        /// tₖ Current Time
        const auto& time__t0 = imuObs__t0->timeSinceStartup.value();

        // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
        timeDifferenceSec__t1 = static_cast<long double>(time__t1 - time__t2) * 1e-9L;
        // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
        timeDifferenceSec__t0 = static_cast<long double>(time__t0 - time__t1) * 1e-9L;

        if (timeSinceStartup__init == 0)
        {
            timeSinceStartup__init = imuObs__t0->timeSinceStartup.value();
            time__init = imuObs__t0->insTime.has_value() ? imuObs__t0->insTime.value() : InsTime(2000, 1, 1, 1, 1, 1);
        }

        // Update time
        posVelAtt__t0->insTime = time__init + std::chrono::nanoseconds(imuObs__t0->timeSinceStartup.value() - timeSinceStartup__init);

        LOG_DATA("{}: time__t2 {}", nameId(), time__t2);
        LOG_DATA("{}: time__t1 {}; DiffSec__t1 {}", nameId(), time__t1, timeDifferenceSec__t1);
        LOG_DATA("{}: time__t0 {}: DiffSec__t0 {}", nameId(), time__t0, timeDifferenceSec__t0);
    }

    /// ω_ip_p (tₖ₋₁) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d angularVelocity_ip_p__t1 = !prefereUncompensatedData && imuObs__t1->gyroCompXYZ.has_value()
                                                   ? imuObs__t1->gyroCompXYZ.value()
                                                   : imuObs__t1->gyroUncompXYZ.value();

    angularVelocity_ip_p__t1 -= imuBiases.biasGyro_p;
    LOG_DATA("{}: angularVelocity_ip_p__t1 = {}", nameId(), angularVelocity_ip_p__t1.transpose());

    /// ω_ip_p (tₖ) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d angularVelocity_ip_p__t0 = !prefereUncompensatedData && imuObs__t0->gyroCompXYZ.has_value()
                                                   ? imuObs__t0->gyroCompXYZ.value()
                                                   : imuObs__t0->gyroUncompXYZ.value();

    angularVelocity_ip_p__t0 -= imuBiases.biasGyro_p;
    LOG_DATA("{}: angularVelocity_ip_p__t0 = {}", nameId(), angularVelocity_ip_p__t0.transpose());

    /// a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d acceleration_p__t1 = !prefereUncompensatedData && imuObs__t1->accelCompXYZ.has_value()
                                             ? imuObs__t1->accelCompXYZ.value()
                                             : imuObs__t1->accelUncompXYZ.value();

    acceleration_p__t1 -= imuBiases.biasAccel_p;
    LOG_DATA("{}: acceleration_p__t1 = {}", nameId(), acceleration_p__t1.transpose());

    /// a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    Eigen::Vector3d acceleration_p__t0 = !prefereUncompensatedData && imuObs__t0->accelCompXYZ.has_value()
                                             ? imuObs__t0->accelCompXYZ.value()
                                             : imuObs__t0->accelUncompXYZ.value();

    acceleration_p__t0 -= imuBiases.biasAccel_p;
    LOG_DATA("{}: acceleration_p__t0 = {}", nameId(), acceleration_p__t0.transpose());

    /// v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& velocity_n__t1 = posVelAtt__t1->velocity_n();
    LOG_DATA("{}: velocity_n__t1 = {}", nameId(), velocity_n__t1.transpose());
    /// v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
    const Eigen::Vector3d& velocity_n__t2 = posVelAtt__t2->velocity_n();
    LOG_DATA("{}: velocity_n__t2 = {}", nameId(), velocity_n__t2.transpose());
    /// v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
    const Eigen::Vector3d velocity_e__t2 = posVelAtt__t2->velocity_e();
    LOG_DATA("{}: velocity_e__t2 = {}", nameId(), velocity_e__t2.transpose());
    /// v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
    const Eigen::Vector3d velocity_e__t1 = posVelAtt__t1->velocity_e();
    LOG_DATA("{}: velocity_e__t1 = {}", nameId(), velocity_e__t1.transpose());
    /// x_e (tₖ₋₂) Position in [m], in ECEF coordinates, at the time tₖ₋₂
    const Eigen::Vector3d position_e__t2 = posVelAtt__t2->position_ecef();
    LOG_DATA("{}: position_e__t2 = {}", nameId(), position_e__t2.transpose());
    /// x_e (tₖ₋₁) Position in [m], in ECEF coordinates, at the time tₖ₋₁
    const Eigen::Vector3d position_e__t1 = posVelAtt__t1->position_ecef();
    LOG_DATA("{}: position_e__t1 = {}", nameId(), position_e__t1.transpose());

    /// g_n Gravity vector in [m/s^2], in navigation coordinates
    Eigen::Vector3d gravity_n__t1;

    /// Gravity vector determination
    if (gravityModel == GravityModel::Somigliana)
    {
        LOG_DATA("{}: Gravity calculated with Somigliana model", nameId());
        gravity_n__t1 = gravity::gravity_SomiglianaAltitude(posVelAtt__t1->latitude(), posVelAtt__t1->altitude());
    }
    else if (gravityModel == GravityModel::WGS84_Skydel) // TODO: This function becomes obsolete, once the ImuStream is deactivated due to the 'InstinctDataStream'
    {
        LOG_DATA("{}: Gravity calculated with WGS84 model as in the Skydel Simulator plug-in", nameId());
        double gravityMagnitude = gravity::gravityMagnitude_WGS84_Skydel(posVelAtt__t1->latitude(), posVelAtt__t1->altitude());
        // Gravity vector NED
        const Eigen::Vector3d gravityVector(0.0, 0.0, gravityMagnitude);
        gravity_n__t1 = gravityVector;
    }
    else if (gravityModel == GravityModel::EGM96)
    {
        LOG_DATA("{}: Gravity calculated with EGM96", nameId());
        int egm96degree = 10;
        gravity_n__t1 = gravity::gravity_EGM96(posVelAtt__t1->latitude(), posVelAtt__t1->longitude(), posVelAtt__t1->altitude(), egm96degree);
    }
    else if (gravityModel == GravityModel::OFF)
    {
        LOG_DATA("{}: Gravity set to zero", nameId());
        gravity_n__t1 = Eigen::Vector3d::Zero();
    }
    else
    {
        LOG_DATA("{}: Gravity calculated with WGS84 model (derivation of the gravity potential after 'r')", nameId());
        gravity_n__t1 = gravity::gravity_WGS84(posVelAtt__t1->latitude(), posVelAtt__t1->altitude());
    }

    if (centrifugalAccCompensation)
    {
        auto centrifugalAcceleration = gravity::centrifugalAcceleration(posVelAtt__t1->latitude(), posVelAtt__t1->altitude());
        LOG_DATA("{}: Applying centrifugal acceleration: {}", nameId(), centrifugalAcceleration.transpose());
        gravity_n__t1 += centrifugalAcceleration;
    }
    LOG_DATA("{}: gravity_n__t1 = {}", nameId(), gravity_n__t1.transpose());

    /// g_e Gravity vector in [m/s^2], in earth coordinates
    const Eigen::Vector3d gravity_e__t1 = posVelAtt__t1->quaternion_en() * gravity_n__t1;

    if (integrationFrame == IntegrationFrame::ECEF)
    {
        LOG_DATA("{}: Integrating in ECEF frame", nameId());

        /// q (tₖ₋₂) Quaternion, from gyro platform to earth coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_gyro_ep__t2 = posVelAtt__t2->quaternion_eb() * imuPosition.quatGyro_bp();
        LOG_DATA("{}: quaternion_gyro_ep__t2 = {}", nameId(), quaternion_gyro_ep__t2.coeffs().transpose());
        /// q (tₖ₋₁) Quaternion, from gyro platform to earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_gyro_ep__t1 = posVelAtt__t1->quaternion_eb() * imuPosition.quatGyro_bp();
        LOG_DATA("{}: quaternion_gyro_ep__t1 = {}", nameId(), quaternion_gyro_ep__t1.coeffs().transpose());

        /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
        const Eigen::Vector3d& angularVelocity_ie_e__t0 = InsConst::angularVelocity_ie_e;
        LOG_DATA("{}: angularVelocity_ie_e__t0 = {}", nameId(), angularVelocity_ie_e__t0.transpose());

        /// q (tₖ₋₂) Quaternion, from accel platform to earth coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_accel_ep__t2 = posVelAtt__t2->quaternion_eb() * imuPosition.quatAccel_bp();
        LOG_DATA("{}: quaternion_accel_ep__t2 = {}", nameId(), quaternion_accel_ep__t2.coeffs().transpose());

        /// q (tₖ₋₁) Quaternion, from accel platform to earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_accel_ep__t1 = posVelAtt__t1->quaternion_eb() * imuPosition.quatAccel_bp();
        LOG_DATA("{}: quaternion_accel_ep__t1 = {}", nameId(), quaternion_accel_ep__t1.coeffs().transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Attitude Update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// q (tₖ) Quaternion, from platform to earth coordinates, at the current time tₖ
        Eigen::Quaterniond quaternion_gyro_ep__t0 = Eigen::Quaterniond::Identity();

        if (integrationAlgorithmAttitude == IntegrationAlgorithm::RungeKutta3)
        {
            quaternion_gyro_ep__t0 = updateQuaternion_ep_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                     angularVelocity_ip_p__t0, angularVelocity_ip_p__t1,
                                                                     angularVelocity_ie_e__t0,
                                                                     quaternion_gyro_ep__t1, quaternion_gyro_ep__t2);
        }
        // TODO: Implement RungeKutta1
        else
        {
            LOG_CRITICAL("{}: Selected integration algorithm not supported", nameId());
        }
        LOG_DATA("{}: quaternion_gyro_ep__t0 = {}", nameId(), quaternion_gyro_ep__t0.coeffs().transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                    Specific force frame transformation                                   */
        /* -------------------------------------------------------------------------------------------------------- */

        /// q (tₖ) Quaternion, from accel platform to earth coordinates, at the time tₖ
        const Eigen::Quaterniond quaternion_accel_ep__t0 = quaternion_gyro_ep__t0 * imuPosition.quatGyro_pb() * imuPosition.quatAccel_bp();
        LOG_DATA("{}: quaternion_accel_ep__t0 = {}", nameId(), quaternion_accel_ep__t0.coeffs().transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Velocity update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// v (tₖ), Velocity in [m/s], in earth coordinates, at the current time tₖ
        Eigen::Vector3d velocity_e__t0 = Eigen::Vector3d::Zero();

        if (integrationAlgorithmVelocity == IntegrationAlgorithm::Simpson)
        {
            velocity_e__t0 = updateVelocity_e_Simpson(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                      acceleration_p__t0, acceleration_p__t1,
                                                      velocity_e__t2,
                                                      position_e__t2,
                                                      gravity_e__t1,
                                                      quaternion_accel_ep__t0,
                                                      quaternion_accel_ep__t1,
                                                      quaternion_accel_ep__t2,
                                                      !coriolisCompensation);
        }
        // TODO: Implement RungeKutta1 & RungeKutta3
        else
        {
            LOG_CRITICAL("{}: Selected integration algorithm not supported", nameId());
        }
        LOG_DATA("{}: velocity_e__t0 = {}", nameId(), velocity_e__t0.transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Position update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// x_e (tₖ) Position in [m], in earth coordinates, at the time tₖ
        Eigen::Vector3d position_e__t0 = Eigen::Vector3d::Zero();

        if (integrationAlgorithmPosition == IntegrationAlgorithm::RectangularRule)
        {
            position_e__t0 = updatePosition_e(timeDifferenceSec__t0, position_e__t1, velocity_e__t1);
        }
        else
        {
            LOG_CRITICAL("{}: Selected integration algorithm not supported", nameId());
        }
        LOG_DATA("{}: position_e__t0 = {}", nameId(), position_e__t0.transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                               Store Results                                              */
        /* -------------------------------------------------------------------------------------------------------- */

        posVelAtt__t0->setState_e(position_e__t0, velocity_e__t0, quaternion_gyro_ep__t0 * imuPosition.quatGyro_pb());
    }
    else if (integrationFrame == IntegrationFrame::NED)
    {
        LOG_DATA("{}: Integrating in NED frame", nameId());
        /// ω_ip_b (tₖ₋₁) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ₋₁
        const Eigen::Vector3d angularVelocity_ip_b__t1 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t1;
        LOG_DATA("{}: angularVelocity_ip_b__t1 = {}", nameId(), angularVelocity_ip_b__t1.transpose());
        /// ω_ip_b (tₖ) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ
        const Eigen::Vector3d angularVelocity_ip_b__t0 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t0;
        LOG_DATA("{}: angularVelocity_ip_b__t0 = {}", nameId(), angularVelocity_ip_b__t0.transpose());

        /// a_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
        const Eigen::Vector3d acceleration_b__t1 = imuPosition.quatAccel_bp() * acceleration_p__t1;
        LOG_DATA("{}: acceleration_b__t1 = {}", nameId(), acceleration_b__t1.transpose());

        /// a_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
        const Eigen::Vector3d acceleration_b__t0 = imuPosition.quatAccel_bp() * acceleration_p__t0;
        LOG_DATA("{}: acceleration_b__t0 = {}", nameId(), acceleration_b__t0.transpose());

        /// q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_nb__t1 = posVelAtt__t1->quaternion_nb();
        LOG_DATA("{}: quaternion_nb__t1 = {}", nameId(), quaternion_nb__t1.coeffs().transpose());
        /// q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_nb__t2 = posVelAtt__t2->quaternion_nb();
        LOG_DATA("{}: quaternion_nb__t2 = {}", nameId(), quaternion_nb__t2.coeffs().transpose());

        /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
        Eigen::Vector3d angularVelocity_ie_n__t1 = posVelAtt__t1->quaternion_ne() * InsConst::angularVelocity_ie_e;
        LOG_DATA("{}: angularVelocity_ie_n__t1 = {}", nameId(), angularVelocity_ie_n__t1.transpose());

        /// North/South (meridian) earth radius [m]
        double R_N = earthRadius_N(posVelAtt__t1->latitude());
        LOG_DATA("{}: R_N = {}", nameId(), R_N);
        /// East/West (prime vertical) earth radius [m]
        double R_E = earthRadius_E(posVelAtt__t1->latitude());
        LOG_DATA("{}: R_E = {}", nameId(), R_E);

        /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
        Eigen::Vector3d angularVelocity_en_n__t1 = transportRate(posVelAtt__t1->latLonAlt(), velocity_n__t1, R_N, R_E);
        LOG_DATA("{}: angularVelocity_en_n__t1 = {}", nameId(), angularVelocity_en_n__t1.transpose());

        /// [latitude 𝜙, longitude λ, altitude h] (tₖ₋₁) Position in [rad, rad, m] at the time tₖ₋₁
        Eigen::Vector3d latLonAlt__t1 = posVelAtt__t1->latLonAlt();
        LOG_DATA("{}: latLonAlt__t1 = {}", nameId(), latLonAlt__t1.transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Attitude Update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// q (tₖ) Quaternion, from body to navigation coordinates, at the current time tₖ
        Eigen::Quaterniond quaternion_nb__t0 = Eigen::Quaterniond::Identity();

        if (integrationAlgorithmAttitude == IntegrationAlgorithm::RungeKutta3)
        {
            quaternion_nb__t0 = updateQuaternion_nb_RungeKutta3(timeDifferenceSec__t0,
                                                                timeDifferenceSec__t1,
                                                                angularVelocity_ip_b__t0,
                                                                angularVelocity_ip_b__t1,
                                                                angularVelocity_ie_n__t1,
                                                                angularVelocity_en_n__t1,
                                                                quaternion_nb__t1,
                                                                quaternion_nb__t2);
        }
        else if (integrationAlgorithmAttitude == IntegrationAlgorithm::RungeKutta1)
        {
            quaternion_nb__t0 = updateQuaternion_nb_RungeKutta1(timeDifferenceSec__t0,
                                                                angularVelocity_ip_b__t0,
                                                                angularVelocity_ie_n__t1,
                                                                angularVelocity_en_n__t1,
                                                                quaternion_nb__t1);
        }
        else
        {
            LOG_CRITICAL("{}: Selected integration algorithm not supported", nameId());
        }
        LOG_DATA("{}: quaternion_nb__t0 = {}", nameId(), quaternion_nb__t0.coeffs().transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                           Specific force frame transformation & Velocity update                          */
        /* -------------------------------------------------------------------------------------------------------- */

        /// v (tₖ), Velocity in navigation coordinates, at the current time tₖ
        Eigen::Vector3d velocity_n__t0 = Eigen::Vector3d::Zero();

        if (integrationAlgorithmVelocity == IntegrationAlgorithm::Simpson)
        {
            velocity_n__t0 = updateVelocity_n_Simpson(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                      acceleration_b__t0, acceleration_b__t1,
                                                      velocity_n__t1, velocity_n__t2,
                                                      gravity_n__t1,
                                                      angularVelocity_ie_n__t1,
                                                      angularVelocity_en_n__t1,
                                                      quaternion_nb__t0, quaternion_nb__t1, quaternion_nb__t2,
                                                      !coriolisCompensation);
        }
        else if (integrationAlgorithmVelocity == IntegrationAlgorithm::RungeKutta1)
        {
            velocity_n__t0 = updateVelocity_n_RungeKutta1(timeDifferenceSec__t0,
                                                          acceleration_b__t0,
                                                          velocity_n__t1,
                                                          gravity_n__t1,
                                                          angularVelocity_ie_n__t1,
                                                          angularVelocity_en_n__t1,
                                                          quaternion_nb__t1);
        }
        else if (integrationAlgorithmVelocity == IntegrationAlgorithm::RungeKutta3)
        {
            velocity_n__t0 = updateVelocity_n_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                          acceleration_b__t0, acceleration_b__t1,
                                                          velocity_n__t2,
                                                          gravity_n__t1,
                                                          angularVelocity_ie_n__t1,
                                                          angularVelocity_en_n__t1,
                                                          quaternion_nb__t0, quaternion_nb__t1, quaternion_nb__t2,
                                                          !coriolisCompensation);
        }
        else
        {
            LOG_CRITICAL("{}: Selected integration algorithm not supported", nameId());
        }
        LOG_DATA("{}: velocity_n__t0 = {}", nameId(), velocity_n__t0.transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Position update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// [latitude 𝜙, longitude λ, altitude h] (tₖ) Position in [rad, rad, m] at the time tₖ
        Eigen::Vector3d latLonAlt__t0 = Eigen::Vector3d::Zero();

        if (integrationAlgorithmPosition == IntegrationAlgorithm::RectangularRule)
        {
            latLonAlt__t0 = updatePosition_lla(timeDifferenceSec__t0, latLonAlt__t1, velocity_n__t1, R_N, R_E);
        }
        else
        {
            LOG_CRITICAL("{}: Selected integration algorithm not supported", nameId());
        }
        LOG_DATA("{}: latLonAlt__t0 = {}", nameId(), latLonAlt__t0.transpose());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                               Store Results                                              */
        /* -------------------------------------------------------------------------------------------------------- */

        posVelAtt__t0->setState_n(latLonAlt__t0, velocity_n__t0, quaternion_nb__t0);
    }

    LOG_DATA("{}: posVelAtt__t0->position_ecef() = {}", nameId(), posVelAtt__t0->position_ecef().transpose());
    LOG_DATA("{}: posVelAtt__t0->velocity_n() = {}", nameId(), posVelAtt__t0->velocity_n().transpose());
    LOG_DATA("{}: posVelAtt__t0->quaternion_nb() = {}", nameId(), posVelAtt__t0->quaternion_nb().coeffs().transpose());

    // Cycle lists
    imuObservations.pop_back();
    posVelAttStates.pop_back();

    if (!skipIntermediateCalculation)
    {
        // Push out new data
        invokeCallbacks(OutputPortIndex_InertialNavSol__t0, posVelAtt__t0);

        if (!calculateIntermediateValues)
        {
            skipIntermediateCalculation = true;
        }
    }
    else
    {
        posVelAttStates.push_front(posVelAtt__t0);
        // Remove states at the end of the list till the max size is reached
        while (posVelAttStates.size() > maxSizeStates)
        {
            posVelAttStates.pop_back();
        }

        skipIntermediateCalculation = false;
    }
    LOG_DATA("{}: posVelAttStates.at(0) = {}, posVelAttStates.at(1) = {}", nameId(), posVelAttStates.at(0), posVelAttStates.at(1));
}

const char* NAV::ImuIntegrator::to_string(IntegrationAlgorithm algorithm)
{
    switch (algorithm)
    {
    case IntegrationAlgorithm::RectangularRule:
        return "Rectangular Rule";
    case IntegrationAlgorithm::Simpson:
        return "Simpson";
    case IntegrationAlgorithm::RungeKutta1:
        return "Runge Kutta 1st Order";
    case IntegrationAlgorithm::RungeKutta3:
        return "Runge Kutta 3rd Order";
    case IntegrationAlgorithm::COUNT:
        return "";
    }
    return "";
}