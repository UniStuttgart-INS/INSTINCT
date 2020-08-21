#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

NAV::ImuIntegrator::ImuIntegrator(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : Node(name)
{
}

void NAV::ImuIntegrator::integrateObservation(std::shared_ptr<NAV::ImuObs>& obs)
{
    // Get the current state data
    const auto& sourceNode = incomingLinks[1].first.lock();
    auto& sourcePortIndex = incomingLinks[1].second;
    auto currentStateData = std::static_pointer_cast<StateData>(sourceNode->requestOutputData(sourcePortIndex));

    // Fill if empty
    if (prevObs.empty())
    {
        prevObs.push_back(obs);
        prevObs.push_back(obs);
        prevStates.push_back(currentStateData);
        prevStates.push_back(currentStateData);
    }

    /// Time at 2 previous epochs
    const auto& tₖ₋₂ = prevObs.at(1)->insTime.value();
    /// Time at previous epoch
    const auto& tₖ₋₁ = prevObs.at(0)->insTime.value();
    /// Current Time
    const auto& tₖ = obs->insTime.value();

    /// Rotation rate in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const auto& ω_ip_p__tₖ₋₁ = prevObs.at(0)->gyroUncompXYZ.value();
    /// Rotation rate in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    const auto& ω_ip_p__tₖ = obs->gyroUncompXYZ.value();

    // Delta time in [seconds] (tₖ₋₁ - tₖ₋₂)
    auto Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂).count();
    // Delta time in [seconds] (tₖ - tₖ₋₁)
    auto Δtₖ = (tₖ - tₖ₋₁).count();

    /// The delta rotation angles in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d Δα_ip_p__tₖ₋₁ = Δtₖ₋₁ * ω_ip_p__tₖ₋₁;
    /// The delta rotation angles in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d Δα_ip_p__tₖ = Δtₖ * ω_ip_p__tₖ;

    /// Direction Cosine Matrix at the time tₖ₋₂
    const auto& DCM_e2p__tₖ₋₂ = prevStates.at(1)->DCM_e2p();
    /// Direction Cosine Matrix at the time tₖ₋₁
    const auto& DCM_e2p__tₖ₋₁ = prevStates.at(0)->DCM_e2p();

    /// Rotation rate in [rad/s],
    /// of the inertial to earth system, in platform coordinates, at the time tₖ
    Eigen::Vector3d ω_ie_e;

    /// The delta rotation angles in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d Δ⁠β⁠_ep_p__tₖ₋₁ = Δα_ip_p__tₖ₋₁ - DCM_e2p__tₖ₋₂ * ω_ie_e * Δtₖ₋₁;
    /// The delta rotation angles in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d Δβ⁠_ep_p__tₖ = Δα_ip_p__tₖ - DCM_e2p__tₖ₋₁ * ω_ie_e * Δtₖ;

    LOG_INFO("Δ⁠β⁠_ep_p__tₖ₋₁: {}", Δ⁠β⁠_ep_p__tₖ₋₁);
    LOG_INFO("Δβ⁠_ep_p__tₖ: {}", Δβ⁠_ep_p__tₖ);
    // auto dt = 2 * delta_t;

    // omega_ep^p = omega_ip^p - C_e^p * omega_ie^e
    // omega_ip^p = gyroUncompXYZ

    // Rotate Data
    if (prevObs.size() == 2)
    {
        prevObs.pop_back();
        prevStates.pop_back();
    }
    prevObs.push_front(obs);
    prevStates.push_front(currentStateData);

    invokeCallbacks(obs);
}